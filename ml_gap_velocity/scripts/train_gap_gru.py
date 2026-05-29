#!/usr/bin/env python3

import os
import json
import argparse

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split


BASE_INPUT_FEATURES = [
    "sector_density",
    "sector_dynamic_raw_gap_point_count",
    "contained_raw_gap_point_count",
    "sector_area",
    "sector_angle_rad",
    "sector_radius",
]

DERIVED_FEATURE_CONFIG = {
    "density_rate_of_change": "drate",
    "aspect_ratio":           "ar",
}

OUTPUT_FEATURES = [
    "gt_sector_density",
]

# Minimum angle (radians) used when computing aspect_ratio to
# prevent division by zero or extreme values.
MIN_SECTOR_ANGLE_RAD = 1e-4

def build_input_features(use_density_rate: bool, use_aspect_ratio: bool) -> list:
    """
    Returns the ordered list of input feature names for this run.
 
    Base features always come first (same order as the original
    training script). Enabled derived features are appended after.
 
    This list is the single source of truth for feature ordering:
    - The dataset uses it to build numpy arrays.
    - It is saved in the stats JSON.
    - The ROS inference node reads it from the stats JSON and knows
      exactly which features to compute and in which order.
    """
    features = list(BASE_INPUT_FEATURES)
    if use_density_rate:
        features.append("density_rate_of_change")
    if use_aspect_ratio:
        features.append("aspect_ratio")
    return features


def safe_name(name):
    return (
        str(name)
        .replace("/", "_")
        .replace("\\", "_")
        .replace(" ", "_")
        .replace(":", "-")
        .replace("(", "")
        .replace(")", "")
        .replace(",", "")
        .replace("'", "")
        .replace('"', "")
    )


def safe_float_name(value):
    """
    Turns floats into filename-safe strings.

    Examples:
        0.001 -> 1em3
        0.0005 -> 5em4
        1e-05 -> 1em5
        0.01 -> 0p01
    """
    s = f"{value:g}"

    if "e-" in s:
        base, exp = s.split("e-")
        base = base.replace(".", "p")
        return f"{base}em{exp}"

    if "e+" in s:
        base, exp = s.split("e+")
        base = base.replace(".", "p")
        return f"{base}ep{exp}"

    return s.replace(".", "p")

def make_feature_suffix(use_density_rate: bool, use_aspect_ratio: bool) -> str:
    """
    Builds a short suffix that encodes which derived features are
    active. Examples:
        both off  -> ""
        drate on  -> "_feat_drate"
        ar on     -> "_feat_ar"
        both on   -> "_feat_drate_ar"
 
    This ends up in all output filenames so you can tell runs apart
    at a glance and never accidentally load the wrong stats JSON.
    """
    active = []
    if use_density_rate:
        active.append(DERIVED_FEATURE_CONFIG["density_rate_of_change"])
    if use_aspect_ratio:
        active.append(DERIVED_FEATURE_CONFIG["aspect_ratio"])
    if not active:
        return ""
    return "_feat_" + "_".join(active)


def make_auto_run_name(args):
    """
    Creates the concise automatic part of the filename.

    Example:
        factory_mse_sl10_h64_l2_bs64_lr1em3_e500
    """
    user_name = args.run_name

    if user_name is None or user_name.strip() == "":
        csv_base = os.path.splitext(os.path.basename(args.csv))[0]
        user_name = csv_base
    
    feature_suffix = make_feature_suffix(
        args.use_density_rate,
        args.use_aspect_ratio,
    )
 
    pieces = [
        safe_name(user_name),
        "density",
        safe_name(args.loss),
        f"sl{args.seq_len}",
        f"h{args.hidden_size}",
        f"l{args.num_layers}",
        f"bs{args.batch_size}",
        f"lr{safe_float_name(args.lr)}",
        f"e{args.epochs}",
    ]
 
    # Append feature suffix to the run name piece so it shows up
    # near the front of every filename, e.g.:
    #   myenv_density_feat_drate_ar_mse_sl10_h64_l2_bs64_lr1em3_e500
    pieces[0] = pieces[0] + feature_suffix
 
    return "_".join(pieces)


def make_output_base_name(args):
    csv_base = os.path.splitext(os.path.basename(args.csv))[0]
    auto_run_name = make_auto_run_name(args)
    return f"{auto_run_name}__data_{safe_name(csv_base)}"


def make_loss_fn(loss_name):
    loss_name = loss_name.lower()

    if loss_name == "mse":
        return nn.MSELoss()

    if loss_name == "l1":
        return nn.L1Loss()

    if loss_name == "smooth_l1":
        return nn.SmoothL1Loss()

    raise RuntimeError(
        f"Unknown loss function: {loss_name}. "
        "Use one of: mse, l1, smooth_l1"
    )


class GapSectorDensitySequenceDataset(Dataset):
    def __init__(
        self,
        csv_path: str,
        seq_len: int = 10,
        use_density_rate: bool = False,
        use_aspect_ratio: bool = False
    ):
        self.seq_len = seq_len
        self.use_density_rate = use_density_rate
        self.use_aspect_ratio = use_aspect_ratio

        # The final ordered list of features this model instance uses.
        self.input_features = build_input_features(
            use_density_rate,
            use_aspect_ratio,
        )

        df = pd.read_csv(csv_path)

        required_cols = [
            "sample_idx",
            "model_id",
            "side",
            *BASE_INPUT_FEATURES,
            *OUTPUT_FEATURES,
        ]

        missing_cols = [col for col in required_cols if col not in df.columns]
        if len(missing_cols) > 0:
            raise RuntimeError(
                f"CSV is missing required columns: {missing_cols}\n"
                f"Available columns: {list(df.columns)}"
            )

        # ////////////////////////////////////////////////////////////
        # Only use one row per simplified gap.
        #
        # Left and right simplified gap endpoint rows contain the
        # same sector-level density information, so using both would
        # duplicate the same training target.
        # ////////////////////////////////////////////////////////////

        df = df[df["side"] == "left"].copy()

        if len(df) == 0:
            raise RuntimeError(
                "No rows with side == 'left' were found in the CSV."
            )

        df = df.dropna(subset=required_cols)

        df = df.sort_values(
            ["model_id", "sample_idx"]
        ).reset_index(drop=True)

        self.samples_x = []
        self.samples_y = []

        # ////////////////////////////////////////////////////////////
        # Group by left simplified gap point model ID.
        #
        # This preserves temporal sequences for the same tracked
        # simplified gap side over time.
        # ////////////////////////////////////////////////////////////

        group_cols = ["model_id"]

        for _, group in df.groupby(group_cols):
            group = group.sort_values("sample_idx").reset_index(drop=True)

            # --------------------------------------------------------
            # Compute derived features PER GROUP so that temporal
            # differences (density_rate_of_change) are computed within
            # the same gap track and not across different tracks.
            # --------------------------------------------------------
 
            if self.use_density_rate:
                # sector_density[t] - sector_density[t-1].
                # The very first observation has no prior, so we use 0.0.
                # This is consistent with the ROS node, which also
                # outputs 0.0 when the buffer is empty.
                group["density_rate_of_change"] = (
                    group["sector_density"].diff().fillna(0.0)
                )
 
            if self.use_aspect_ratio:
                # 1 / sector_angle_rad. Clipped to avoid extreme values
                # when a gap's angular span is very small.
                group["aspect_ratio"] = 1.0 / group["sector_angle_rad"].clip(
                    lower=MIN_SECTOR_ANGLE_RAD
                )

            features = group[
                self.input_features
            ].values.astype(np.float32)

            targets = group[
                OUTPUT_FEATURES
            ].values.astype(np.float32)

            if len(group) < seq_len:
                continue

            for i in range(seq_len - 1, len(group)):
                x_seq = features[i - seq_len + 1 : i + 1]
                y_label = targets[i]

                self.samples_x.append(x_seq)
                self.samples_y.append(y_label)

        self.samples_x = np.array(self.samples_x, dtype=np.float32)
        self.samples_y = np.array(self.samples_y, dtype=np.float32)

        if len(self.samples_x) == 0:
            raise RuntimeError(
                "No training sequences were created. "
                "Try lowering seq_len or collecting longer left-side gap tracks."
            )

        num_input_features = len(self.input_features)
        num_output_features = len(OUTPUT_FEATURES)

        self.x_mean = self.samples_x.reshape(
            -1,
            num_input_features
        ).mean(axis=0)

        self.x_std = self.samples_x.reshape(
            -1,
            num_input_features
        ).std(axis=0) + 1e-8

        self.y_mean = self.samples_y.reshape(
            -1,
            num_output_features
        ).mean(axis=0)

        self.y_std = self.samples_y.reshape(
            -1,
            num_output_features
        ).std(axis=0) + 1e-8

        self.samples_x = (self.samples_x - self.x_mean) / self.x_std
        self.samples_y = (self.samples_y - self.y_mean) / self.y_std

        self.num_sequences = len(self.samples_x)
        self.num_groups = df.groupby(group_cols).ngroups
        self.num_left_rows = len(df)

    def __len__(self):
        return len(self.samples_x)

    def __getitem__(self, idx):
        x = torch.tensor(self.samples_x[idx], dtype=torch.float32)
        y = torch.tensor(self.samples_y[idx], dtype=torch.float32)
        return x, y


class GapSectorDensityGRU(nn.Module):
    def __init__(
        self,
        input_size=6,
        hidden_size=64,
        num_layers=2,
        output_size=1,
    ):
        super().__init__()

        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
        )

        self.head = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Linear(64, output_size),
        )

    def forward(self, x):
        # x shape:
        # [batch, seq_len, 6]
        out, _ = self.gru(x)

        # Use final GRU output.
        final_out = out[:, -1, :]

        # Predict:
        # [gt_sector_density]
        pred = self.head(final_out)
        return pred


def check_overwrite(paths, overwrite):
    if overwrite:
        return

    existing = [p for p in paths if os.path.exists(p)]

    if len(existing) > 0:
        msg = "These output files already exist:\n"
        msg += "\n".join(existing)
        msg += "\n\nUse --overwrite if you intentionally want to replace them."
        raise RuntimeError(msg)


def train(args):
    device = torch.device(
        "cuda" if torch.cuda.is_available() and not args.cpu else "cpu"
    )
    print(f"device: {device}")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    dataset = GapSectorDensitySequenceDataset(
        args.csv,
        seq_len=args.seq_len,
        use_density_rate=args.use_density_rate,
        use_aspect_ratio=args.use_aspect_ratio
    )

    input_features = dataset.input_features

    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size

    train_dataset, val_dataset = random_split(
        dataset,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(args.seed),
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
    )

    model = GapSectorDensityGRU(
        input_size=len(input_features),
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        output_size=len(OUTPUT_FEATURES),
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = make_loss_fn(args.loss)

    best_val_loss = float("inf")

    os.makedirs(args.model_dir, exist_ok=True)

    output_base_name = make_output_base_name(args)
    auto_run_name = make_auto_run_name(args)

    best_model_path = os.path.join(
        args.model_dir,
        f"{output_base_name}_gap_sector_density_gru.pt",
    )

    stats_path = os.path.join(
        args.model_dir,
        f"{output_base_name}_norm_stats.json",
    )

    config_path = os.path.join(
        args.model_dir,
        f"{output_base_name}_train_config.json",
    )

    history_path = os.path.join(
        args.model_dir,
        f"{output_base_name}_train_history.csv",
    )

    check_overwrite(
        [best_model_path, stats_path, config_path, history_path],
        overwrite=args.overwrite,
    )

    train_config = {
        "task": "gap_sector_density_prediction",
        "run_name": args.run_name,
        "auto_run_name": auto_run_name,
        "output_base_name": output_base_name,
        "csv": args.csv,
        "model_path": best_model_path,
        "stats_path": stats_path,
        "config_path": config_path,
        "history_path": history_path,
        "seq_len": args.seq_len,
        "hidden_size": args.hidden_size,
        "num_layers": args.num_layers,
        "batch_size": args.batch_size,
        "epochs": args.epochs,
        "lr": args.lr,
        "loss": args.loss,
        "seed": args.seed,
        "input_features": input_features,
        "output_features": OUTPUT_FEATURES,
        "row_filter": "side == left",
        "group_cols": ["model_id"],
        "num_samples": len(dataset),
        "num_groups": dataset.num_groups,
        "num_left_rows": dataset.num_left_rows,
        "train_size": train_size,
        "val_size": val_size,
    }

    with open(config_path, "w") as f:
        json.dump(train_config, f, indent=4)

    print("")
    print(f"training data: {args.csv}")
    print(f"run name: {args.run_name}")
    print(f"auto run name: {auto_run_name}")
    print(f"output base name: {output_base_name}")
    print(f"loss: {args.loss}")
    print(f"seq_len: {args.seq_len}")
    print(f"hidden_size: {args.hidden_size}")
    print(f"num_layers: {args.num_layers}")
    print(f"batch_size: {args.batch_size}")
    print(f"epochs: {args.epochs}")
    print(f"lr: {args.lr}")
    print(f"use_density_rate: {args.use_density_rate}")
    print(f"use_aspect_ratio: {args.use_aspect_ratio}")
    print(f"input features: {input_features}")
    print(f"output features: {OUTPUT_FEATURES}")
    print("using only rows where side == 'left'")
    print(f"num left-side rows: {dataset.num_left_rows}")
    print(f"num sequences: {len(dataset)}")
    print(f"num groups: {dataset.num_groups}")
    print(f"model will save to: {best_model_path}")
    print(f"stats will save to: {stats_path}")
    print(f"config will save to: {config_path}")
    print(f"history will save to: {history_path}")
    print("")

    history_rows = []

    for epoch in range(args.epochs):
        model.train()
        train_losses = []

        for x_batch, y_batch in train_loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)

            pred = model(x_batch)
            loss = loss_fn(pred, y_batch)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_losses.append(loss.item())

        model.eval()
        val_losses = []

        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch = x_batch.to(device)
                y_batch = y_batch.to(device)

                pred = model(x_batch)
                loss = loss_fn(pred, y_batch)

                val_losses.append(loss.item())

        train_loss = float(np.mean(train_losses))
        val_loss = float(np.mean(val_losses))

        history_rows.append({
            "epoch": epoch + 1,
            "train_loss": train_loss,
            "val_loss": val_loss,
            "best_val_loss_so_far": min(best_val_loss, val_loss),
        })

        print(
            f"epoch {epoch + 1:04d} | "
            f"train_loss={train_loss:.6f} | "
            f"val_loss={val_loss:.6f}"
        )

        if val_loss < best_val_loss:
            best_val_loss = val_loss

            model_cpu = model.to("cpu")
            model_cpu.eval()

            example_input = torch.zeros(
                1,
                args.seq_len,
                len(input_features),
            )

            traced = torch.jit.trace(model_cpu, example_input)
            traced.save(best_model_path)

            model.to(device)

            stats = {
                "task": "gap_sector_density_prediction",
                "run_name": args.run_name,
                "auto_run_name": auto_run_name,
                "output_base_name": output_base_name,
                "csv": args.csv,
                "model_path": best_model_path,
                "best_val_loss": best_val_loss,
                "seq_len": args.seq_len,
                "hidden_size": args.hidden_size,
                "num_layers": args.num_layers,
                "batch_size": args.batch_size,
                "epochs": args.epochs,
                "lr": args.lr,
                "loss": args.loss,
                "x_mean": dataset.x_mean.tolist(),
                "x_std": dataset.x_std.tolist(),
                "y_mean": dataset.y_mean.tolist(),
                "y_std": dataset.y_std.tolist(),
                "input_features": input_features,
                "output_features": OUTPUT_FEATURES,
                "use_density_rate": args.use_density_rate,
                "use_aspect_ratio": args.use_aspect_ratio,
                "row_filter": "side == left",
                "group_cols": ["model_id"],
            }

            with open(stats_path, "w") as f:
                json.dump(stats, f, indent=4)

            print(f"saved best model to {best_model_path}")
            print(f"saved normalization stats to {stats_path}")

    pd.DataFrame(history_rows).to_csv(history_path, index=False)

    print("")
    print("done")
    print(f"best_val_loss={best_val_loss:.6f}")
    print("")
    print("This model predicts gt_sector_density from sector-level history.")
    print("Your old velocity testing script will not match this model yet.")
    print("You will need a separate density test/eval script.")
    print("")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv", required=True)
    parser.add_argument("--model-dir", required=True)

    # This should just be the environment / short experiment name.
    # Example:
    # --run-name empty_density_test
    parser.add_argument("--run-name", default=None)

    parser.add_argument("--seq-len", type=int, default=10)
    parser.add_argument("--hidden-size", type=int, default=64)
    parser.add_argument("--num-layers", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-3)

    # Default is MSE.
    # Use --loss l1 if you want L1.
    # Use --loss smooth_l1 if you want SmoothL1.
    parser.add_argument(
        "--loss",
        choices=["mse", "l1", "smooth_l1"],
        default="mse",
    )

    parser.add_argument(
        "--use-density-rate",
        action="store_true",
        help=(
            "Add density_rate_of_change as an input feature. "
            "Computed as sector_density[t] - sector_density[t-1] "
            "within each gap track. Penalizes gaps whose congestion "
            "is spiking even if current density looks acceptable."
        ),
    )
    parser.add_argument(
        "--use-aspect-ratio",
        action="store_true",
        help=(
            "Add aspect_ratio = 1/sector_angle_rad as an input feature. "
            "Penalizes narrow sectors geometrically. A small theta means "
            "a long skinny corridor with little maneuvering room."
        ),
    )

    parser.add_argument("--seed", type=int, default=42)

    parser.add_argument("--cpu", action="store_true")
    parser.add_argument("--overwrite", action="store_true")

    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()