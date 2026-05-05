#!/usr/bin/env python3

import json
import argparse

import numpy as np
import pandas as pd
import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--stats", required=True)
    parser.add_argument("--num-print", type=int, default=20)
    args = parser.parse_args()

    model = torch.jit.load(args.model)
    model.eval()

    with open(args.stats, "r") as f:
        stats = json.load(f)

    seq_len = stats["seq_len"]

    x_mean = np.array(stats["x_mean"], dtype=np.float32)
    x_std = np.array(stats["x_std"], dtype=np.float32)
    y_mean = np.array(stats["y_mean"], dtype=np.float32)
    y_std = np.array(stats["y_std"], dtype=np.float32)

    df = pd.read_csv(args.csv)

    if "ns" in df.columns:
        df = df[df["ns"].astype(str).str.contains("simp", case=False, na=False)]

    df = df.dropna(subset=["time", "gap_id", "side", "x", "y", "vx", "vy"])
    df = df.sort_values(["gap_id", "side", "time"]).reset_index(drop=True)

    printed = 0

    for key, group in df.groupby(["gap_id", "side"]):
        group = group.sort_values("time")

        positions = group[["x", "y"]].values.astype(np.float32)
        true_vels = group[["vx", "vy"]].values.astype(np.float32)

        if len(group) < seq_len:
            continue

        for i in range(seq_len - 1, len(group)):
            x_seq = positions[i - seq_len + 1 : i + 1]
            y_true = true_vels[i]

            x_norm = (x_seq - x_mean) / x_std
            x_tensor = torch.tensor(x_norm, dtype=torch.float32).unsqueeze(0)

            with torch.no_grad():
                y_pred_norm = model(x_tensor).numpy()[0]

            y_pred = y_pred_norm * y_std + y_mean

            print(
                f"gap={key} | "
                f"true vx,vy=({y_true[0]: .3f}, {y_true[1]: .3f}) | "
                f"pred vx,vy=({y_pred[0]: .3f}, {y_pred[1]: .3f})"
            )

            printed += 1
            if printed >= args.num_print:
                return


if __name__ == "__main__":
    main()