#!/usr/bin/env python3
"""Convert MPPI sample scores to one CSV row per cycle, gap and iteration."""
import argparse
import csv
import itertools
import math
import sys

KEY = ("sim_time", "cycle", "gap", "left_id", "right_id", "iteration",
       "mode", "h_enabled", "w_h", "temperature")
COSTS = ("h", "obstacle", "goal", "path", "relative_velocity", "speed", "total")
METRICS = ("h_active_fraction", "h_share", "winner_changed", "weight_shift",
           "first_yaw_rate_delta")


def probabilities(costs, temperature):
    minimum = min(costs)
    weights = [math.exp(-(cost - minimum) / temperature) for cost in costs]
    total = sum(weights)
    return [w / total for w in weights]


def summarize(rows):
    feasible = [r for r in rows if r["feasible"] == "1"]
    result = {"samples": len(rows), "feasible": len(feasible)}
    if not feasible:
        return result
    for name in COSTS:
        result["mean_" + name] = sum(float(r[name]) for r in feasible) / len(feasible)
    h = [float(r["h"]) for r in feasible]
    costs = [float(r["total"]) for r in feasible]
    base = [c - penalty for c, penalty in zip(costs, h)]
    temp = float(feasible[0]["temperature"])
    full_weights = probabilities(costs, temp)
    base_weights = probabilities(base, temp)
    changes = [full - old for full, old in zip(full_weights, base_weights)]
    result.update(
        h_active_fraction=sum(value > 0 for value in h) / len(h),
        h_share=sum(h) / sum(costs) if sum(costs) else "",
        winner_changed=int(costs.index(min(costs)) != base.index(min(base))),
        weight_shift=sum(abs(delta) for delta in changes) / 2,
        first_yaw_rate_delta=sum(delta * float(r["first_yaw_rate"])
                                 for delta, r in zip(changes, feasible)),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", help="Completed samples_*.csv from one run")
    args = parser.parse_args()
    writer = csv.DictWriter(sys.stdout, fieldnames=[
        *KEY, "samples", "feasible", *("mean_" + c for c in COSTS), *METRICS])
    writer.writeheader()
    with open(args.samples, newline="") as stream:
        for key, rows in itertools.groupby(csv.DictReader(stream),
                                           key=lambda row: tuple(row[k] for k in KEY)):
            writer.writerow({**dict(zip(KEY, key)), **summarize(list(rows))})


if __name__ == "__main__":
    main()
