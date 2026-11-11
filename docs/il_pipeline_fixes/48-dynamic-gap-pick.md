# 48 — DynamicGap's own pick and costs not recorded (M-03, D4/D9)

**Status:** Fixed · **Repo(s):** dynamic_gap, arena-rosnav · **Files:** `src/Planner.cpp`, `include/dynamic_gap/Planner.h`, `include/dynamic_gap/utils/ManualSelection.h`, `test/planner_test_access.h`, `test/test_manual_selection.cpp`, `il_tools/il_dataset.py`, `il_tools/test/test_il_tools.py`; arena-rosnav `utils/ros/msgs/manual_gap_msgs/msg/ManualGapSample.msg`, `ManualGapCandidate.msg`
**Behaviour change:** None live. New fields in the sample (`SCHEMA_VERSION` 2) and in the dataset.

## Problem
In manual mode the planner computes every gap trajectory's cost (`generateGapTrajsV2` runs before the manual/autonomous branch) but skips `pickTraj`, so DynamicGap's own choice and its costs were never recorded. The gap scorer (design doc, Alternative 1) needs them:
- the go/no-go test: how often the operator disagreed with DynamicGap (under ~10% leaves little to learn);
- the baseline: top-1 accuracy of "pick the lowest cost" is what a learned scorer must beat;
- an optional input or residual base for the scorer.

The dataset also lacked fields the scorer needs: the robot's yaw (candidate geometry is in the odom frame), `shown`, `tube_len`, and whether a candidate is the gap followed on the previous tick (`is_current`).

## What changed
**Recording (dynamic_gap, arena-rosnav)**
- `ManualGapCandidate`: `dg_cost`, `dg_terminal_cost` (Q_f × distance from the terminal pose to the local waypoint), `dg_clearance_cost` (mean pose-wise cost against the propagated scans). `dg_cost` = terminal + clearance, inf if the path has fewer than 2 poses, NaN if not scored.
- `ManualGapSample`: `dg_pick_idx`, the position in `candidates[]` of DynamicGap's lowest-cost gap trajectory; -1 if none, or the tick did not plan (`newManualSample`).
- `SCHEMA_VERSION` 1 → 2.
- The pick is computed side-effect-free: `pickTraj` is called after `selectManualTrajectory` with local outputs (`dgTrajFlag`, `dgTrajIdx`), and the comparison block (`compareToCurrentTraj`) is not run. The operator's `trajFlag`, trajectory and selection are untouched.
- `dgTrajCost` / `meanPoseCost` (ManualSelection.h) hold the cost formula; `pickTraj` uses the same helper for gap trajectories, so the recorded cost and DynamicGap's ranking cannot drift apart.

The pick is DynamicGap's *preferred* gap (cost argmin), not what it would have executed: the comparison block depends on DynamicGap's own current trajectory, which in manual mode is the operator's.

**Dataset (`il_build_dataset`)**
- Per example: `robot_yaw_odom`, `schema_version`, `dg_pick` (None before schema 2).
- Per candidate: `shown`, `tube_len`, `is_current` (same model ids as the gap selected on the tick before the observation tick), `goal_rbt`, `terminal_rbt`, `path_rbt` (robot frame at the observation tick), `dg_cost`, `dg_terminal_cost`, `dg_clearance_cost` (None before schema 2; inf is written as JSON `Infinity`, read by Python's `json`).

## Effects
- Schema 1 sessions still build; their `dg_*` fields are None. Do not pool them into anything that uses DynamicGap's costs.
- On `sets/4_corners_data_1.yaml` (schema 1) the label is the `is_current` gap in 70% of examples.
- Planner cost per tick: one extra pass over the gap costs (`pickTraj`), negligible.

## Test
`test_manual_selection.cpp`: `ManualSelectionHelpers.DgTrajCost`, `ManualSelectionHelpers.NewSampleHasNoSelection` (dg_pick_idx -1), `ManualPlannerTest.RecordsDynamicGapCostsAndPick`, `ManualPlannerTest.CostsNotScoredAreNaN`. `test_il_tools.py`: `DatasetTest.test_dynamic_gap_costs_and_pick_exported`, `test_is_current_from_previous_tick`, `test_robot_frame_geometry`. 44 C++ and 29 Python tests pass.
