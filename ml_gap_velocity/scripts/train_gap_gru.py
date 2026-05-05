#!/usr/bin/env python3

import os
import json
import argparse

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split


class GapSequenceDataset(Dataset):
    def __init__(self, csv_path, seq_len=10):
        self.seq_len = seq_len

        df = pd.read_csv(csv_path)

        # Keep only simplified rows, just in case the CSV ever has more.
        if "ns" in df.columns:
            df = df[df["ns"].astype(str).str.contains("simp", case=False, na=False)]

        df = df.dropna(subset=["time", "gap_id", "side", "x", "y", "vx", "vy"])
        df = df.sort_values(["gap_id", "side", "time"]).reset_index(drop=True)

        self.samples_x = []
        self.samples_y = []

        group_cols = ["gap_id", "side"]

        for _, group in df.groupby(group_cols):
            group = group.sort_values("time")

            positions = group[["x", "y"]].values.astype(np.float32)
            velocities = group[["vx", "vy"]].values.astype(np.float32)

            if len(group) < seq_len:
                continue

            for i in range(seq_len - 1, len(group)):
                x_seq = positions[i - seq_len + 1 : i + 1]
                y_label = velocities[i]

                self.samples_x.append(x_seq)
                self.samples_y.append(y_label)

        self.samples_x = np.array(self.samples_x, dtype=np.float32)
        self.samples_y = np.array(self.samples_y, dtype=np.float32)

        if len(self.samples_x) == 0:
            raise RuntimeError("No training sequences were created. Try lowering seq_len.")

        self.x_mean = self.samples_x.reshape(-1, 2).mean(axis=0)
        self.x_std = self.samples_x.reshape(-1, 2).std(axis=0) + 1e-8

        self.y_mean = self.samples_y.mean(axis=0)
        self.y_std = self.samples_y.std(axis=0) + 1e-8

        self.samples_x = (self.samples_x - self.x_mean) / self.x_std
        self.samples_y = (self.samples_y - self.y_mean) / self.y_std

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
            batch_first=True
        )

        self.head = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, x):
        # x shape: [batch, seq_len, 2]
        out, hidden = self.gru(x)

        # Use final GRU output.
        final_out = out[:, -1, :]

        # Predict [vx, vy].
        pred = self.head(final_out)
        return pred


def train(args):
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")

    dataset = GapSequenceDataset(args.csv, seq_len=args.seq_len)

    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size

    train_dataset, val_dataset = random_split(
        dataset,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False
    )

    model = GapVelocityGRU(
        input_size=2,
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        output_size=2
    ).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    best_val_loss = float("inf")

    os.makedirs(args.model_dir, exist_ok=True)

    best_model_path = os.path.join(args.model_dir, "gap_gru.pt")
    stats_path = os.path.join(args.model_dir, "norm_stats.json")

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

        print(
            f"epoch {epoch + 1:04d} | "
            f"train_loss={train_loss:.6f} | "
            f"val_loss={val_loss:.6f}"
        )

        if val_loss < best_val_loss:
            best_val_loss = val_loss

            # Save TorchScript model for runtime.
            model_cpu = model.to("cpu")
            model_cpu.eval()

            example_input = torch.zeros(1, args.seq_len, 2)
            traced = torch.jit.trace(model_cpu, example_input)
            traced.save(best_model_path)

            model.to(device)

            stats = {
                "seq_len": args.seq_len,
                "x_mean": dataset.x_mean.tolist(),
                "x_std": dataset.x_std.tolist(),
                "y_mean": dataset.y_mean.tolist(),
                "y_std": dataset.y_std.tolist(),
                "input_features": ["x", "y"],
                "output_features": ["vx", "vy"]
            }

            with open(stats_path, "w") as f:
                json.dump(stats, f, indent=4)

            print(f"saved best model to {best_model_path}")
            print(f"saved normalization stats to {stats_path}")

    print("done")
    print(f"best_val_loss={best_val_loss:.6f}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv", required=True)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--seq-len", type=int, default=10)
    parser.add_argument("--hidden-size", type=int, default=64)
    parser.add_argument("--num-layers", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--cpu", action="store_true")

    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()