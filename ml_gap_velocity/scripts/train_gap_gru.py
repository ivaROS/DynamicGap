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

SOURCE_DENSITY_COL = "gt_sector_density"
MIN_SECTOR_ANGLE_RAD = 1e-4


def build_input_features(use_density_rate: bool, use_aspect_ratio: bool) -> list:
    features = list(BASE_INPUT_FEATURES)
    if use_density_rate:
        features.append("density_rate_of_change")
    if use_aspect_ratio:
        features.append("aspect_ratio")
    return features


def build_output_feature_name(future_steps: int) -> str:
    if future_steps == 0:
        return "gt_sector_density"
    return f"future_sector_density_k{future_steps}"


def safe_name(name):
    return (
        str(name)
        .replace("/", "_").replace("\\", "_").replace(" ", "_")
        .replace(":", "-").replace("(", "").replace(")", "")
        .replace(",", "").replace("'", "").replace('"', "")
    )


def safe_float_name(value):
    s = f"{value:g}"
    if "e-" in s:
        base, exp = s.split("e-")
        return f"{base.replace('.', 'p')}em{exp}"
    if "e+" in s:
        base, exp = s.split("e+")
        return f"{base.replace('.', 'p')}ep{exp}"
    return s.replace(".", "p")


def make_feature_suffix(use_density_rate, use_aspect_ratio, future_steps, dropout):
    active = []
    if use_density_rate:
        active.append(DERIVED_FEATURE_CONFIG["density_rate_of_change"])
    if use_aspect_ratio:
        active.append(DERIVED_FEATURE_CONFIG["aspect_ratio"])
    suffix = ""
    if active:
        suffix += "_feat_" + "_".join(active)
    if future_steps > 0:
        suffix += f"_k{future_steps}"
    if dropout > 0.0:
        suffix += f"_do{safe_float_name(dropout)}"
    return suffix


def make_auto_run_name(args):
    user_name = args.run_name
    if user_name is None or user_name.strip() == "":
        user_name = os.path.splitext(os.path.basename(args.csv))[0]

    feature_suffix = make_feature_suffix(
        args.use_density_rate, args.use_aspect_ratio, args.future_steps, args.dropout
    )

    pieces = [
        safe_name(user_name) + feature_suffix,
        "density",
        safe_name(args.loss),
        f"sl{args.seq_len}",
        f"h{args.hidden_size}",
        f"l{args.num_layers}",
        f"bs{args.batch_size}",
        f"lr{safe_float_name(args.lr)}",
        f"e{args.epochs}",
    ]
    return "_".join(pieces)


def make_output_base_name(args):
    csv_base = os.path.splitext(os.path.basename(args.csv))[0]
    return f"{make_auto_run_name(args)}__data_{safe_name(csv_base)}"


def make_loss_fn(loss_name):
    loss_name = loss_name.lower()
    if loss_name == "mse":
        return nn.MSELoss()
    if loss_name == "l1":
        return nn.L1Loss()
    if loss_name == "smooth_l1":
        return nn.SmoothL1Loss()
    raise RuntimeError(f"Unknown loss: {loss_name}. Use mse, l1, or smooth_l1")


class GapSectorDensitySequenceDataset(Dataset):
    """
    Builds GRU training sequences from gap sector CSV data.

    future_steps == 0: predict gt_sector_density at time t (original behaviour)
    future_steps == k: predict gt_sector_density at time t+k within the same
                       gap track. Rows where t+k doesn't exist are dropped.
    """

    def __init__(
        self,
        csv_path,
        seq_len=10,
        use_density_rate=False,
        use_aspect_ratio=False,
        future_steps=0,
    ):
        self.seq_len = seq_len
        self.use_density_rate = use_density_rate
        self.use_aspect_ratio = use_aspect_ratio
        self.future_steps = future_steps

        self.input_features = build_input_features(use_density_rate, use_aspect_ratio)
        self.output_feature = build_output_feature_name(future_steps)

        df = pd.read_csv(csv_path)

        required_cols = ["sample_idx", "model_id", "side", *BASE_INPUT_FEATURES, SOURCE_DENSITY_COL]
        missing = [c for c in required_cols if c not in df.columns]
        if missing:
            raise RuntimeError(
                f"CSV is missing required columns: {missing}\n"
                f"Available: {list(df.columns)}"
            )

        df = df[df["side"] == "left"].copy()
        if len(df) == 0:
            raise RuntimeError("No rows with side == 'left' found.")

        df = df.dropna(subset=required_cols)
        df = df.sort_values(["model_id", "sample_idx"]).reset_index(drop=True)

        self.samples_x = []
        self.samples_y = []

        for _, group in df.groupby(["model_id"]):
            group = group.sort_values("sample_idx").reset_index(drop=True)

            if self.use_density_rate:
                group["density_rate_of_change"] = (
                    group["sector_density"].diff().fillna(0.0)
                )

            if self.use_aspect_ratio:
                group["aspect_ratio"] = 1.0 / group["sector_angle_rad"].clip(
                    lower=MIN_SECTOR_ANGLE_RAD
                )

            if self.future_steps == 0:
                group[self.output_feature] = group[SOURCE_DENSITY_COL]
            else:
                group[self.output_feature] = group[SOURCE_DENSITY_COL].shift(
                    -self.future_steps
                )

            group = group.dropna(subset=[self.output_feature])

            if len(group) < seq_len:
                continue

            features = group[self.input_features].values.astype(np.float32)
            targets  = group[[self.output_feature]].values.astype(np.float32)

            for i in range(seq_len - 1, len(group)):
                self.samples_x.append(features[i - seq_len + 1 : i + 1])
                self.samples_y.append(targets[i])

        self.samples_x = np.array(self.samples_x, dtype=np.float32)
        self.samples_y = np.array(self.samples_y, dtype=np.float32)

        if len(self.samples_x) == 0:
            raise RuntimeError(
                "No training sequences created. "
                "Try lowering --seq-len, --future-steps, or collecting longer tracks."
            )

        n_in  = len(self.input_features)
        n_out = 1

        self.x_mean = self.samples_x.reshape(-1, n_in).mean(axis=0)
        self.x_std  = self.samples_x.reshape(-1, n_in).std(axis=0) + 1e-8
        self.y_mean = self.samples_y.reshape(-1, n_out).mean(axis=0)
        self.y_std  = self.samples_y.reshape(-1, n_out).std(axis=0) + 1e-8

        self.samples_x = (self.samples_x - self.x_mean) / self.x_std
        self.samples_y = (self.samples_y - self.y_mean) / self.y_std

        self.num_sequences = len(self.samples_x)
        self.num_groups    = df.groupby(["model_id"]).ngroups
        self.num_left_rows = len(df)

    def __len__(self):
        return len(self.samples_x)

    def __getitem__(self, idx):
        x = torch.tensor(self.samples_x[idx], dtype=torch.float32)
        y = torch.tensor(self.samples_y[idx], dtype=torch.float32)
        return x, y


class GapSectorDensityGRU(nn.Module):
    def __init__(self, input_size=6, hidden_size=64, num_layers=2, output_size=1, dropout=0.0):
        super().__init__()

        # dropout only applies between stacked layers; no effect when num_layers=1
        gru_dropout = dropout if num_layers > 1 else 0.0

        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=gru_dropout,
        )

        self.head = nn.Sequential(
            nn.Dropout(p=dropout),
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Dropout(p=dropout),
            nn.Linear(64, output_size),
        )

    def forward(self, x):
        out, _ = self.gru(x)
        return self.head(out[:, -1, :])


def check_overwrite(paths, overwrite):
    if overwrite:
        return
    existing = [p for p in paths if os.path.exists(p)]
    if existing:
        raise RuntimeError(
            "These output files already exist:\n" + "\n".join(existing) +
            "\n\nUse --overwrite to replace them."
        )


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
        use_aspect_ratio=args.use_aspect_ratio,
        future_steps=args.future_steps,
    )

    input_features = dataset.input_features
    output_feature = dataset.output_feature

    train_size = int(0.8 * len(dataset))
    val_size   = len(dataset) - train_size

    train_dataset, val_dataset = random_split(
        dataset, [train_size, val_size],
        generator=torch.Generator().manual_seed(args.seed),
    )

    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    val_loader   = DataLoader(val_dataset,   batch_size=args.batch_size, shuffle=False)

    model = GapSectorDensityGRU(
        input_size=len(input_features),
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        output_size=1,
        dropout=args.dropout,
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn   = make_loss_fn(args.loss)
    best_val_loss = float("inf")

    os.makedirs(args.model_dir, exist_ok=True)

    output_base_name = make_output_base_name(args)
    auto_run_name    = make_auto_run_name(args)

    best_model_path = os.path.join(args.model_dir, f"{output_base_name}_gap_sector_density_gru.pt")
    stats_path      = os.path.join(args.model_dir, f"{output_base_name}_norm_stats.json")
    config_path     = os.path.join(args.model_dir, f"{output_base_name}_train_config.json")
    history_path    = os.path.join(args.model_dir, f"{output_base_name}_train_history.csv")

    check_overwrite([best_model_path, stats_path, config_path, history_path], args.overwrite)

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
        "output_features": [output_feature],
        "use_density_rate": args.use_density_rate,
        "use_aspect_ratio": args.use_aspect_ratio,
        "dropout": args.dropout,
        "future_steps": args.future_steps,
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
    print(f"training data:     {args.csv}")
    print(f"auto run name:     {auto_run_name}")
    print(f"loss:              {args.loss}")
    print(f"seq_len:           {args.seq_len}")
    print(f"hidden_size:       {args.hidden_size}")
    print(f"num_layers:        {args.num_layers}")
    print(f"use_density_rate:  {args.use_density_rate}")
    print(f"use_aspect_ratio:  {args.use_aspect_ratio}")
    print(f"dropout:           {args.dropout}")
    print(f"future_steps (k):  {args.future_steps}  (~{args.future_steps * 40}ms ahead)")
    print(f"input features:    {input_features}")
    print(f"output feature:    {output_feature}")
    print(f"num sequences:     {len(dataset)}")
    print(f"model  -> {best_model_path}")
    print(f"stats  -> {stats_path}")
    print("")

    history_rows = []

    for epoch in range(args.epochs):
        model.train()
        train_losses = []
        for x_batch, y_batch in train_loader:
            x_batch, y_batch = x_batch.to(device), y_batch.to(device)
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
                x_batch, y_batch = x_batch.to(device), y_batch.to(device)
                val_losses.append(loss_fn(model(x_batch), y_batch).item())

        train_loss = float(np.mean(train_losses))
        val_loss   = float(np.mean(val_losses))

        history_rows.append({
            "epoch": epoch + 1,
            "train_loss": train_loss,
            "val_loss": val_loss,
            "best_val_loss_so_far": min(best_val_loss, val_loss),
        })

        print(f"epoch {epoch + 1:04d} | train_loss={train_loss:.6f} | val_loss={val_loss:.6f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss

            model_cpu = model.to("cpu")
            model_cpu.eval()
            example_input = torch.zeros(1, args.seq_len, len(input_features))
            torch.jit.trace(model_cpu, example_input).save(best_model_path)
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
                "x_std":  dataset.x_std.tolist(),
                "y_mean": dataset.y_mean.tolist(),
                "y_std":  dataset.y_std.tolist(),
                "input_features":  input_features,
                "output_features": [output_feature],
                "use_density_rate": args.use_density_rate,
                "use_aspect_ratio": args.use_aspect_ratio,
                "dropout": args.dropout,
                "future_steps": args.future_steps,
                "row_filter": "side == left",
                "group_cols": ["model_id"],
            }

            with open(stats_path, "w") as f:
                json.dump(stats, f, indent=4)

            print(f"  saved best model -> {best_model_path}")
            print(f"  saved stats      -> {stats_path}")

    pd.DataFrame(history_rows).to_csv(history_path, index=False)

    print("")
    print("done")
    print(f"best_val_loss={best_val_loss:.6f}")
    print(f"output feature: {output_feature}")
    if args.future_steps > 0:
        print(
            f"this model predicts density {args.future_steps} steps "
            f"(~{args.future_steps * 40}ms) into the future."
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv",         required=True)
    parser.add_argument("--model-dir",   required=True)
    parser.add_argument("--run-name",    default=None)
    parser.add_argument("--seq-len",     type=int,   default=10)
    parser.add_argument("--hidden-size", type=int,   default=64)
    parser.add_argument("--num-layers",  type=int,   default=2)
    parser.add_argument("--batch-size",  type=int,   default=64)
    parser.add_argument("--epochs",      type=int,   default=100)
    parser.add_argument("--lr",          type=float, default=1e-3)
    parser.add_argument("--loss", choices=["mse", "l1", "smooth_l1"], default="mse")

    parser.add_argument(
        "--use-density-rate", action="store_true",
        help="Add density_rate_of_change as an input feature.",
    )
    parser.add_argument(
        "--use-aspect-ratio", action="store_true",
        help="Add aspect_ratio = 1/sector_angle_rad as an input feature.",
    )
    parser.add_argument(
        "--dropout", type=float, default=0.0,
        help="Dropout probability (default 0.0). Recommended sweep: 0.1, 0.2, 0.3.",
    )
    parser.add_argument(
        "--future-steps", type=int, default=0,
        help="Predict density k steps into the future (0 = current density).",
    )

    parser.add_argument("--seed",      type=int, default=42)
    parser.add_argument("--cpu",       action="store_true")
    parser.add_argument("--overwrite", action="store_true")

    args = parser.parse_args()

    if args.future_steps < 0:
        raise ValueError("--future-steps must be >= 0")

    train(args)


if __name__ == "__main__":
    main()