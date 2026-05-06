#!/usr/bin/env python3

import os
import json
import argparse

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split


def safe_name(name):
    return (
        str(name)
        .replace("/", "_")
        .replace(" ", "_")
        .replace(":", "-")
        .replace("(", "")
        .replace(")", "")
        .replace(",", "")
        .replace("'", "")
        .replace('"', "")
    )


def make_output_base_name(run_name, csv_path):
    csv_base = os.path.splitext(os.path.basename(csv_path))[0]
    return f"{safe_name(run_name)}__data_{safe_name(csv_base)}"


class GapSequenceDataset(Dataset):
    def __init__(self, csv_path, seq_len=10):
        self.seq_len = seq_len

        df = pd.read_csv(csv_path)

        required_cols = [
            "sample_idx",
            "model_id",
            "side",
            "x",
            "y",
            "perfect_rel_vx",
            "perfect_rel_vy",
        ]

        missing_cols = [col for col in required_cols if col not in df.columns]
        if len(missing_cols) > 0:
            raise RuntimeError(
                f"CSV is missing required columns: {missing_cols}\n"
                f"Available columns: {list(df.columns)}"
            )

        df = df.dropna(subset=required_cols)

        df = df.sort_values(["model_id", "side", "sample_idx"]).reset_index(drop=True)

        self.samples_x = []
        self.samples_y = []

        group_cols = ["model_id", "side"]

        for _, group in df.groupby(group_cols):
            group = group.sort_values("sample_idx").reset_index(drop=True)

            positions = group[["x", "y"]].values.astype(np.float32)

            perfect_rel_vels = group[
                ["perfect_rel_vx", "perfect_rel_vy"]
            ].values.astype(np.float32)

            if len(group) < seq_len:
                continue

            for i in range(seq_len - 1, len(group)):
                x_seq = positions[i - seq_len + 1 : i + 1]
                y_label = perfect_rel_vels[i]

                self.samples_x.append(x_seq)
                self.samples_y.append(y_label)

        self.samples_x = np.array(self.samples_x, dtype=np.float32)
        self.samples_y = np.array(self.samples_y, dtype=np.float32)

        if len(self.samples_x) == 0:
            raise RuntimeError(
                "No training sequences were created. "
                "Try lowering seq_len or collecting longer gap tracks."
            )

        self.x_mean = self.samples_x.reshape(-1, 2).mean(axis=0)
        self.x_std = self.samples_x.reshape(-1, 2).std(axis=0) + 1e-8

        self.y_mean = self.samples_y.mean(axis=0)
        self.y_std = self.samples_y.std(axis=0) + 1e-8

        self.samples_x = (self.samples_x - self.x_mean) / self.x_std
        self.samples_y = (self.samples_y - self.y_mean) / self.y_std

        self.num_sequences = len(self.samples_x)
        self.num_groups = df.groupby(group_cols).ngroups

    def __len__(self):
        return len(self.samples_x)

    def __getitem__(self, idx):
        x = torch.tensor(self.samples_x[idx], dtype=torch.float32)
        y = torch.tensor(self.samples_y[idx], dtype=torch.float32)
        return x, y


class GapVelocityGRU(nn.Module):
    def __init__(self, input_size=2, hidden_size=64, num_layers=2, output_size=2):
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
        # x shape: [batch, seq_len, 2]
        out, _ = self.gru(x)

        # Use final GRU output.
        final_out = out[:, -1, :]

        # Predict [perfect_rel_vx, perfect_rel_vy].
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
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"device: {device}")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    dataset = GapSequenceDataset(args.csv, seq_len=args.seq_len)

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

    model = GapVelocityGRU(
        input_size=2,
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        output_size=2,
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    best_val_loss = float("inf")

    os.makedirs(args.model_dir, exist_ok=True)

    output_base_name = make_output_base_name(args.run_name, args.csv)

    best_model_path = os.path.join(
        args.model_dir,
        f"{output_base_name}_gap_gru.pt",
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
        "run_name": args.run_name,
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
        "seed": args.seed,
        "input_features": ["x", "y"],
        "output_features": ["perfect_rel_vx", "perfect_rel_vy"],
        "group_cols": ["model_id", "side"],
        "num_samples": len(dataset),
        "num_groups": dataset.num_groups,
        "train_size": train_size,
        "val_size": val_size,
    }

    with open(config_path, "w") as f:
        json.dump(train_config, f, indent=4)

    print("")
    print(f"training data: {args.csv}")
    print(f"output base name: {output_base_name}")
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

            example_input = torch.zeros(1, args.seq_len, 2)
            traced = torch.jit.trace(model_cpu, example_input)
            traced.save(best_model_path)

            model.to(device)

            stats = {
                "run_name": args.run_name,
                "output_base_name": output_base_name,
                "csv": args.csv,
                "model_path": best_model_path,
                "best_val_loss": best_val_loss,
                "seq_len": args.seq_len,
                "hidden_size": args.hidden_size,
                "num_layers": args.num_layers,
                "x_mean": dataset.x_mean.tolist(),
                "x_std": dataset.x_std.tolist(),
                "y_mean": dataset.y_mean.tolist(),
                "y_std": dataset.y_std.tolist(),
                "input_features": ["x", "y"],
                "output_features": ["perfect_rel_vx", "perfect_rel_vy"],
                "group_cols": ["model_id", "side"],
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
    print("Use this for testing:")
    print(
        "python3 scripts/test_gap_gru.py "
        f"--csv {args.csv} "
        f"--model {best_model_path} "
        f"--stats {stats_path} "
        f"--plot-name {args.run_name} "
        "--plot-dir plots "
        "--save-results-csv"
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv", required=True)
    parser.add_argument("--model-dir", required=True)

    # The script automatically appends: __data_<csv_file_name>
    parser.add_argument("--run-name", required=True)

    parser.add_argument("--seq-len", type=int, default=10)
    parser.add_argument("--hidden-size", type=int, default=64)
    parser.add_argument("--num-layers", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)

    parser.add_argument("--cpu", action="store_true")
    parser.add_argument("--overwrite", action="store_true")

    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()