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


def build_features_for_group(group, input_dim):
    group = group.sort_values("sample_idx").reset_index(drop=True)

    if input_dim != 2:
        raise RuntimeError(
            f"This test script expects input_dim=2 for [x,y]. "
            f"Got input_dim={input_dim} from norm_stats.json."
        )

    x = group["x"].values.astype(np.float32)
    y = group["y"].values.astype(np.float32)

    features = np.stack([x, y], axis=1).astype(np.float32)

    true_vels = group[
        ["perfect_rel_vx", "perfect_rel_vy"]
    ].values.astype(np.float32)

    return group, features, true_vels


def run_model_on_all_gaps(model, df, seq_len, x_mean, x_std, y_mean, y_std):
    rows = []

    input_dim = len(x_mean)

    for key, group in df.groupby(["model_id", "side"]):
        model_id, side = key
        group = group.sort_values("sample_idx").reset_index(drop=True)

        if len(group) < seq_len:
            continue

        group, features, true_vels = build_features_for_group(group, input_dim)

        for i in range(seq_len - 1, len(group)):
            x_seq = features[i - seq_len + 1 : i + 1]
            y_true = true_vels[i]

            x_norm = (x_seq - x_mean) / x_std
            x_tensor = torch.tensor(x_norm, dtype=torch.float32).unsqueeze(0)

            with torch.no_grad():
                y_pred_norm = model(x_tensor).numpy()[0]

            y_pred = y_pred_norm * y_std + y_mean

            vx_error = float(y_pred[0] - y_true[0])
            vy_error = float(y_pred[1] - y_true[1])
            vector_error = float(np.sqrt(vx_error ** 2 + vy_error ** 2))

            rows.append({
                "model_id": int(model_id),
                "side": str(side),
                "sample_idx": int(group.loc[i, "sample_idx"]),

                "true_perfect_rel_vx": float(y_true[0]),
                "true_perfect_rel_vy": float(y_true[1]),
                "pred_perfect_rel_vx": float(y_pred[0]),
                "pred_perfect_rel_vy": float(y_pred[1]),

                "vx_error": vx_error,
                "vy_error": vy_error,
                "vector_error": vector_error,
                "squared_vector_error": float(vx_error ** 2 + vy_error ** 2),
            })

    return pd.DataFrame(rows)


def compute_global_metrics(results_df):
    vx_mse = float(np.mean(results_df["vx_error"] ** 2))
    vy_mse = float(np.mean(results_df["vy_error"] ** 2))

    vx_rmse = float(np.sqrt(vx_mse))
    vy_rmse = float(np.sqrt(vy_mse))

    vector_mse = float(np.mean(results_df["squared_vector_error"]))
    vector_rmse = float(np.sqrt(vector_mse))

    mean_vector_error = float(np.mean(results_df["vector_error"]))
    median_vector_error = float(np.median(results_df["vector_error"]))
    p90_vector_error = float(np.percentile(results_df["vector_error"], 90))
    p95_vector_error = float(np.percentile(results_df["vector_error"], 95))

    return {
        "vx_mse": vx_mse,
        "vy_mse": vy_mse,
        "vx_rmse": vx_rmse,
        "vy_rmse": vy_rmse,
        "vector_mse": vector_mse,
        "vector_rmse": vector_rmse,
        "mean_vector_error": mean_vector_error,
        "median_vector_error": median_vector_error,
        "p90_vector_error": p90_vector_error,
        "p95_vector_error": p95_vector_error,
    }


def compute_per_gap_metrics(results_df):
    per_gap = []

    for key, group in results_df.groupby(["model_id", "side"]):
        model_id, side = key

        vector_mse = float(np.mean(group["squared_vector_error"]))
        vector_rmse = float(np.sqrt(vector_mse))

        per_gap.append({
            "model_id": int(model_id),
            "side": str(side),
            "num_samples": int(len(group)),
            "vector_rmse": vector_rmse,
            "mean_vector_error": float(np.mean(group["vector_error"])),
            "median_vector_error": float(np.median(group["vector_error"])),
            "max_vector_error": float(np.max(group["vector_error"])),
        })

    per_gap_df = pd.DataFrame(per_gap)
    per_gap_df = per_gap_df.sort_values("vector_rmse", ascending=False).reset_index(drop=True)

    return per_gap_df


def make_all_gap_plots(results_df, per_gap_df, metrics, output_path_base, plot_title):
    metric_text = (
        f"global vector RMSE={metrics['vector_rmse']:.4f}, "
        f"mean error={metrics['mean_vector_error']:.4f}, "
        f"median={metrics['median_vector_error']:.4f}, "
        f"p90={metrics['p90_vector_error']:.4f}"
    )

    saved_paths = []

    # 1. Histogram of vector error over all predictions.
    plt.figure(figsize=(10, 5))
    plt.hist(results_df["vector_error"].values, bins=60)
    plt.xlabel("perfect relative velocity vector error magnitude")
    plt.ylabel("count")
    plt.title(f"{plot_title}\nAll prediction errors\n{metric_text}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = output_path_base + "_all_error_hist.png"
    plt.savefig(path, dpi=200)
    plt.close()
    saved_paths.append(path)

    # 2. Per-model RMSE, sorted worst to best.
    plt.figure(figsize=(14, 5))
    x_axis = np.arange(len(per_gap_df))
    plt.plot(x_axis, per_gap_df["vector_rmse"].values, marker="o", linewidth=1)
    plt.xlabel("model_id + side sequence sorted by vector RMSE, worst to best")
    plt.ylabel("per-sequence vector RMSE")
    plt.title(f"{plot_title}\nPer-sequence perfect relative velocity RMSE\n{metric_text}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = output_path_base + "_per_model_rmse.png"
    plt.savefig(path, dpi=200)
    plt.close()
    saved_paths.append(path)

    # 3. Predicted vs true perfect_rel_vx.
    plt.figure(figsize=(6, 6))
    plt.scatter(
        results_df["true_perfect_rel_vx"].values,
        results_df["pred_perfect_rel_vx"].values,
        s=5,
        alpha=0.4
    )
    min_val = min(
        results_df["true_perfect_rel_vx"].min(),
        results_df["pred_perfect_rel_vx"].min()
    )
    max_val = max(
        results_df["true_perfect_rel_vx"].max(),
        results_df["pred_perfect_rel_vx"].max()
    )
    plt.plot([min_val, max_val], [min_val, max_val], linestyle="--")
    plt.xlabel("true perfect_rel_vx")
    plt.ylabel("predicted perfect_rel_vx")
    plt.title(f"{plot_title}\nPredicted vs true perfect_rel_vx | RMSE={metrics['vx_rmse']:.4f}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = output_path_base + "_perfect_rel_vx_scatter.png"
    plt.savefig(path, dpi=200)
    plt.close()
    saved_paths.append(path)

    # 4. Predicted vs true perfect_rel_vy.
    plt.figure(figsize=(6, 6))
    plt.scatter(
        results_df["true_perfect_rel_vy"].values,
        results_df["pred_perfect_rel_vy"].values,
        s=5,
        alpha=0.4
    )
    min_val = min(
        results_df["true_perfect_rel_vy"].min(),
        results_df["pred_perfect_rel_vy"].min()
    )
    max_val = max(
        results_df["true_perfect_rel_vy"].max(),
        results_df["pred_perfect_rel_vy"].max()
    )
    plt.plot([min_val, max_val], [min_val, max_val], linestyle="--")
    plt.xlabel("true perfect_rel_vy")
    plt.ylabel("predicted perfect_rel_vy")
    plt.title(f"{plot_title}\nPredicted vs true perfect_rel_vy | RMSE={metrics['vy_rmse']:.4f}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = output_path_base + "_perfect_rel_vy_scatter.png"
    plt.savefig(path, dpi=200)
    plt.close()
    saved_paths.append(path)

    # 5. Worst model/side sequence plot.
    worst_gap = per_gap_df.iloc[0]
    worst_model_id = worst_gap["model_id"]
    worst_side = worst_gap["side"]

    worst_df = results_df[
        (results_df["model_id"] == worst_model_id)
        & (results_df["side"] == worst_side)
    ].sort_values("sample_idx")

    plt.figure(figsize=(12, 5))
    plt.plot(worst_df["sample_idx"], worst_df["true_perfect_rel_vx"], label="true perfect_rel_vx")
    plt.plot(worst_df["sample_idx"], worst_df["pred_perfect_rel_vx"], label="pred perfect_rel_vx")
    plt.plot(worst_df["sample_idx"], worst_df["true_perfect_rel_vy"], label="true perfect_rel_vy")
    plt.plot(worst_df["sample_idx"], worst_df["pred_perfect_rel_vy"], label="pred perfect_rel_vy")
    plt.xlabel("sample_idx")
    plt.ylabel("perfect relative velocity")
    plt.title(
        f"{plot_title}\nWorst sequence: model_id={worst_model_id}, side={worst_side}, "
        f"RMSE={worst_gap['vector_rmse']:.4f}"
    )
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = output_path_base + "_worst_model_timeseries.png"
    plt.savefig(path, dpi=200)
    plt.close()
    saved_paths.append(path)

    return saved_paths


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--csv", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--stats", required=True)

    parser.add_argument("--num-print", type=int, default=20)

    parser.add_argument("--plot-name", default="gap_gru_perfect_rel_test")
    parser.add_argument("--plot-dir", default="plots")

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

    results_df = run_model_on_all_gaps(
        model=model,
        df=df,
        seq_len=seq_len,
        x_mean=x_mean,
        x_std=x_std,
        y_mean=y_mean,
        y_std=y_std,
    )

    if len(results_df) == 0:
        raise RuntimeError("No prediction results generated. Check seq_len and CSV size.")

    metrics = compute_global_metrics(results_df)
    per_gap_df = compute_per_gap_metrics(results_df)

    print("")
    print("Global metrics over all model_id + side sequences:")
    print(f"num_predictions       = {len(results_df)}")
    print(f"num_sequences         = {len(per_gap_df)}")
    print(f"vx_mse                = {metrics['vx_mse']:.8f}")
    print(f"vy_mse                = {metrics['vy_mse']:.8f}")
    print(f"vx_rmse               = {metrics['vx_rmse']:.8f}")
    print(f"vy_rmse               = {metrics['vy_rmse']:.8f}")
    print(f"vector_mse            = {metrics['vector_mse']:.8f}")
    print(f"vector_rmse           = {metrics['vector_rmse']:.8f}")
    print(f"mean_vector_error     = {metrics['mean_vector_error']:.8f}")
    print(f"median_vector_error   = {metrics['median_vector_error']:.8f}")
    print(f"p90_vector_error      = {metrics['p90_vector_error']:.8f}")
    print(f"p95_vector_error      = {metrics['p95_vector_error']:.8f}")

    print("")
    print("Worst sequences:")
    print(per_gap_df.head(args.num_print).to_string(index=False))

    os.makedirs(args.plot_dir, exist_ok=True)

    csv_base = os.path.splitext(os.path.basename(args.csv))[0]

    output_base_name = (
        f"{safe_name(args.plot_name)}"
        f"__data_{safe_name(csv_base)}"
    )

    output_path_base = os.path.join(args.plot_dir, output_base_name)

    plot_title = f"{args.plot_name} | data={csv_base}"

    saved_paths = make_all_gap_plots(
        results_df=results_df,
        per_gap_df=per_gap_df,
        metrics=metrics,
        output_path_base=output_path_base,
        plot_title=plot_title,
    )

    print("")
    for path in saved_paths:
        print(f"saved plot: {path}")

    if args.save_results_csv:
        results_path = output_path_base + "_all_predictions.csv"
        per_gap_path = output_path_base + "_per_sequence_metrics.csv"

        results_df.to_csv(results_path, index=False)
        per_gap_df.to_csv(per_gap_path, index=False)

        print(f"saved all predictions csv: {results_path}")
        print(f"saved per-sequence metrics csv: {per_gap_path}")


if __name__ == "__main__":
    main()