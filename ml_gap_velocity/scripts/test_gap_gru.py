#!/usr/bin/env python3

import os
import json
import argparse

import numpy as np
import pandas as pd
import torch
import matplotlib.pyplot as plt


def safe_name(name):
    return (
        str(name)
        .replace("/", "_")
        .replace(" ", "_")
        .replace("(", "")
        .replace(")", "")
        .replace(",", "")
        .replace("'", "")
        .replace('"', "")
    )


def choose_plot_group(df, seq_len, requested_gap_id=None, requested_side=None):
    valid_groups = []

    for key, group in df.groupby(["gap_id", "side"]):
        gap_id, side = key

        if requested_gap_id is not None and int(gap_id) != int(requested_gap_id):
            continue

        if requested_side is not None and str(side) != str(requested_side):
            continue

        if len(group) >= seq_len:
            valid_groups.append((key, len(group)))

    if len(valid_groups) == 0:
        raise RuntimeError("No valid group found for plotting.")

    # Pick the longest valid group if the user did not specify one.
    valid_groups.sort(key=lambda x: x[1], reverse=True)
    return valid_groups[0][0]


def run_model_on_group(model, group, seq_len, x_mean, x_std, y_mean, y_std):
    group = group.sort_values("time").reset_index(drop=True)

    time = group["time"].values.astype(np.float32)
    positions = group[["x", "y"]].values.astype(np.float32)
    true_vels = group[["vx", "vy"]].values.astype(np.float32)

    rows = []

    if len(group) < seq_len:
        return pd.DataFrame(rows)

    for i in range(seq_len - 1, len(group)):
        x_seq = positions[i - seq_len + 1 : i + 1]
        y_true = true_vels[i]

        x_norm = (x_seq - x_mean) / x_std
        x_tensor = torch.tensor(x_norm, dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            y_pred_norm = model(x_tensor).numpy()[0]

        y_pred = y_pred_norm * y_std + y_mean

        rows.append({
            "time": float(time[i]),
            "sample_idx": int(i),
            "true_vx": float(y_true[0]),
            "true_vy": float(y_true[1]),
            "pred_vx": float(y_pred[0]),
            "pred_vy": float(y_pred[1]),
        })

    return pd.DataFrame(rows)


def compute_mse(results_df):
    vx_mse = float(np.mean((results_df["true_vx"] - results_df["pred_vx"]) ** 2))
    vy_mse = float(np.mean((results_df["true_vy"] - results_df["pred_vy"]) ** 2))

    combined_mse = float(
        np.mean(
            (results_df["true_vx"] - results_df["pred_vx"]) ** 2
            + (results_df["true_vy"] - results_df["pred_vy"]) ** 2
        )
    )

    return vx_mse, vy_mse, combined_mse


def make_velocity_plots(results_df, output_path_base, plot_title, mse_text):
    x_axis = results_df["sample_idx"].values

    # vx plot
    plt.figure(figsize=(12, 5))
    plt.plot(x_axis, results_df["true_vx"].values, label="true vx")
    plt.plot(x_axis, results_df["pred_vx"].values, label="pred vx")
    plt.xlabel("sample index")
    plt.ylabel("vx")
    plt.title(f"{plot_title} | vx\n{mse_text}")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    vx_path = output_path_base + "_vx.png"
    plt.savefig(vx_path, dpi=200)
    plt.close()

    # vy plot
    plt.figure(figsize=(12, 5))
    plt.plot(x_axis, results_df["true_vy"].values, label="true vy")
    plt.plot(x_axis, results_df["pred_vy"].values, label="pred vy")
    plt.xlabel("sample index")
    plt.ylabel("vy")
    plt.title(f"{plot_title} | vy\n{mse_text}")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    vy_path = output_path_base + "_vy.png"
    plt.savefig(vy_path, dpi=200)
    plt.close()

    # velocity magnitude plot
    true_speed = np.sqrt(results_df["true_vx"].values ** 2 + results_df["true_vy"].values ** 2)
    pred_speed = np.sqrt(results_df["pred_vx"].values ** 2 + results_df["pred_vy"].values ** 2)

    plt.figure(figsize=(12, 5))
    plt.plot(x_axis, true_speed, label="true speed")
    plt.plot(x_axis, pred_speed, label="pred speed")
    plt.xlabel("sample index")
    plt.ylabel("speed magnitude")
    plt.title(f"{plot_title} | speed magnitude\n{mse_text}")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    speed_path = output_path_base + "_speed.png"
    plt.savefig(speed_path, dpi=200)
    plt.close()

    return vx_path, vy_path, speed_path


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--stats", required=True)

    parser.add_argument("--num-print", type=int, default=20)

    parser.add_argument("--plot-name", default="gap_gru_test")
    parser.add_argument("--plot-dir", default="plots")

    parser.add_argument("--gap-id", type=int, default=None)
    parser.add_argument("--side", choices=["left", "right"], default=None)

    parser.add_argument("--save-results-csv", action="store_true")

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

    # Gap IDs are globally unique in your setup, so grouping only by gap_id and side is fine.
    plot_key = choose_plot_group(
        df,
        seq_len=seq_len,
        requested_gap_id=args.gap_id,
        requested_side=args.side
    )

    plot_gap_id, plot_side = plot_key

    plot_group = df[
        (df["gap_id"] == plot_gap_id)
        & (df["side"].astype(str) == str(plot_side))
    ]

    results_df = run_model_on_group(
        model=model,
        group=plot_group,
        seq_len=seq_len,
        x_mean=x_mean,
        x_std=x_std,
        y_mean=y_mean,
        y_std=y_std
    )

    if len(results_df) == 0:
        raise RuntimeError("No prediction results were generated.")

    vx_mse, vy_mse, combined_mse = compute_mse(results_df)

    for _, row in results_df.head(args.num_print).iterrows():
        print(
            f"gap=({plot_gap_id}, '{plot_side}') | "
            f"true vx,vy=({row['true_vx']: .3f}, {row['true_vy']: .3f}) | "
            f"pred vx,vy=({row['pred_vx']: .3f}, {row['pred_vy']: .3f})"
        )

    print("")
    print(f"plot gap_id={plot_gap_id}, side={plot_side}")
    print(f"vx_mse={vx_mse:.8f}")
    print(f"vy_mse={vy_mse:.8f}")
    print(f"combined_mse={combined_mse:.8f}")

    os.makedirs(args.plot_dir, exist_ok=True)

    csv_base = os.path.splitext(os.path.basename(args.csv))[0]

    output_base_name = (
        f"{safe_name(args.plot_name)}"
        f"__data_{safe_name(csv_base)}"
        f"__gap_{plot_gap_id}_{plot_side}"
    )

    output_path_base = os.path.join(args.plot_dir, output_base_name)

    mse_text = (
        f"MSE: vx={vx_mse:.6f}, "
        f"vy={vy_mse:.6f}, "
        f"combined={combined_mse:.6f}"
    )

    plot_title = (
        f"{args.plot_name} | "
        f"data={csv_base} | "
        f"gap={plot_gap_id}, side={plot_side}"
    )

    vx_path, vy_path, speed_path = make_velocity_plots(
        results_df=results_df,
        output_path_base=output_path_base,
        plot_title=plot_title,
        mse_text=mse_text
    )

    print("") #
    print(f"saved vx plot: {vx_path}")
    print(f"saved vy plot: {vy_path}")
    print(f"saved speed plot: {speed_path}")

    if args.save_results_csv:
        results_path = output_path_base + "_results.csv"
        results_df.to_csv(results_path, index=False)
        print(f"saved results csv: {results_path}")


if __name__ == "__main__":
    main()