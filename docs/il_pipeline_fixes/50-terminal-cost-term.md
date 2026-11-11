# 50 — The gap scorer's pull toward the local waypoint was never trained

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `il_tools/il_model.py`, `il_tools/il_train.py`, `il_tools/il_train`, `il_tools/test/test_il_model.py`
**Behaviour change:** Models trained on schema-1 data now carry the linear term on DynamicGap's terminal cost (clearance weight 0).

## Problem
The scorer is `-(w_t · terminal_cost + w_c · clearance_cost) + network`, the first term being DynamicGap's rule (pull toward the local waypoint). All models so far were trained on `4_corners_data_1_ids`, recorded before fix 48 (schema 1, no costs), so `il_train` dropped the term (`residual: false`). The deployed `with_terminal_seed0` had no pull toward the goal at all, and in closed loop it drove with its momentum and never reached the goal.

Even with the term, its scale makes it negligible: the cost is `Q_f = 0.05` times a distance, so with `w_t = 1` a gap ending 1 m nearer the waypoint scores +0.05, far below the network's output. Training does not fix the scale: with Adam at lr 1e-3 and early stopping near epoch 15, `w_t` moves by about 0.1.

## What changed
- `il_model.dg_terms`: without recorded costs, the terminal cost is rebuilt as `Q_F · |terminal_rbt − local_waypoint|` (`TrajectoryEvaluator::terminalGoalCost`, both in the robot frame). The clearance cost cannot be rebuilt (needs the propagated scans) and is 0. It is then 0 for every candidate of the tick, so it does not change that tick's softmax.
- `featurize` returns `dg_clearance` (a recorded clearance cost exists). `il_train`: the term is kept whenever some candidate has a terminal cost; with no recorded clearance cost its weight is fixed at 0 (it would never train, yet the planner applies it).
- `il_train` stops with an error when the train or validation split is missing or empty (the validation split picks the kept epoch). `il_ablate` checks for DynamicGap's costs on featurized examples (it raised `KeyError: 'dg_avail'` on every dataset).
- `il_train --terminal-weight-init W` (default 1) and `--terminal-weight-min W` (floor kept through training, default none). `result.json` has `dg_weight`.
- The C++ scorer is unchanged: it already applies `dg_weight` to the live costs.

## Check
On `dgap_26-10-24_17-43-43_map_empty_goal_cursor_pilot` (schema 3, costs recorded): rebuilt vs recorded terminal cost over 431 candidates, correlation 0.9997, mean absolute error 0.002 (recorded mean 0.080).

## Results (4_corners_data_1_ids, seed 0)

| model | w_t | val top-1 | val top-1 at switches | test top-1 | picks gap nearest waypoint* | picks gap along velocity* |
|---|---|---|---|---|---|---|
| operator | | | | | 0.60 | 0.63 |
| `with_terminal_seed0` (no term) | – | 0.691 | 0.606 | 0.585 | 0.56 | 0.55 |
| `with_terminal_dgpull_seed0` | 1.08 | 0.682 | 0.581 | 0.589 | 0.57 | 0.57 |
| `with_terminal_pull20_seed0` | 20 | 0.661 | 0.587 | 0.603 | 0.65 | 0.55 |
| `with_terminal_floor20_seed0` | 20 (floor) | 0.657 | 0.594 | 0.603 | 0.66 | 0.55 |
| `with_terminal_pull80_seed0` | 80 | 0.631 | 0.548 | 0.589 | 0.65 | 0.55 |
| `linear_dgpull_seed0` (term only) | 3.83 | 0.614 | 0.613 | 0.599 | 1.00 | 0.46 |

\* val + test ticks with at least two selectable gaps (804).

`w_t = 20` (1 point of score per metre) gives up 3 points of val top-1 and gains 9 points of waypoint-seeking. Offline numbers reward the operator's momentum, so only a closed-loop run decides.

## Test
`test_il_model.py`: `test_costs_missing_before_schema_2` (rebuilt terminal cost, clearance 0), `test_train_save_load` (clearance weight fixed at 0). 12 + 29 Python tests pass.
