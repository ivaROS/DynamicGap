# 53: Raw enough to re-derive: per-scan gaps, full candidates, episode stages (schema 5)

**Status:** Done, verified in simulation 2026-11-11 · **Repo(s):** dynamic_gap, arena-rosnav
**Behaviour change:** None (recording only).

## Problem

An audit (2026-11-11) of whether the recording supports later design changes
without a new collection found:

1. **Gap-point estimators could not be run offline.** The GRU
   (`ml_gap_velocity/scripts/gru_gap_velocity_node.py` on `main_gru_v2*`) reads a
   10-step history of (x, y) per `(model_id, side)` of the *simplified* gaps, on
   every scan (25 Hz). The sample holds only the *raw* gaps, once per planning
   tick (~9 Hz). There is no offline gap detection.
2. **Episodes did not say which scenario they ran.** `params.yaml` is written at
   session start (first stage only); `episodes.csv` had no stage, scenario file or
   seed. `review_session.py` guessed from start and goal, which some files share.
3. **Static obstacles were not in the session.** Shelves come from the scenario
   JSON, not from `/map`.
4. **Candidates were thin.** Downsampled path (every 5th pose), no timing, no
   pose-wise costs, no gap tube, no estimator covariance.
5. **The lab-PC overlay was stale** (schema 3, built 2026-10-26): sessions from it
   would have had no edges (fix 52), which the gap scorer reads.

## Change

**Messages (arena-rosnav `manual_gap_msgs`, `ManualGapSample.SCHEMA_VERSION = 5`):**

- `ManualGapScanGaps` (new, own `SCHEMA_VERSION = 1`) on `<ns>/manual_gap_scan_gaps`,
  published by `Planner::laserScanCB` on every scan in manual and learned mode, also
  when colliding (`colliding = true`, no gaps): scan stamp and `scan_seq`, episode,
  robot pose and velocity, and `raw_edges` and `simplified_edges` (`ManualGapEdge`,
  same layout as `ManualGapSample.edges`) with ids, measurement, state and
  covariance after this scan's update.
- `ManualGapEdge.model_cov`: the estimator's 4×4 covariance (row-major) after its last update.
- `ManualGapCandidate`: `path_full` (every pose, odom frame), `path_time`,
  `dg_pose_costs` (one per pose), `tube` (`ManualGapTubeSegment[]`: start, lifespan,
  end condition, ids, gap points before and after manipulation, manipulated
  velocities, gap goal and its velocity and terminal position, robot frame).

**Recorder (`il_session_recorder`):** records `manual_gap_scan_gaps`;
`episodes.csv` gains `stage`, `stage_index`, `scenario_file`, `seed` (a reset row
describes the episode it starts); every scenario file used is copied to
`<session>/scenarios/`.

**Benchmark (`benchmark.py` `_start_stage`):** sets `/benchmark_stage`,
`/benchmark_stage_index` and `/benchmark_stage_seed` (the seed the RNG was
restarted from) before the reset that starts a stage.

**Suite:** `il_dataset_small_v1.yaml` has all 14 stages (26 episodes) again; the
2026-10-27 temporary comment-out is gone.

**Offline tools:** `il_bags.load_scan_gaps`; `il_check_session` adds three checks on
schema ≥ 5 sessions: scan gaps at ≥ 20 Hz sim, feasible candidates carry
`path_full`, `path_time` and `tube`, reset rows name scenario file or seed.

## Verification (2026-11-11)

Two-stage suite (CS1, 50ped v2; 40 s timeout), automatic goal-cursor operator,
port 11312, overlay rebuilt: all 11 `il_check_session` checks pass. 1152 scan-gaps
messages at 25.0 Hz, none skipped; every candidate model id of every planned tick
(3852) is in the simplified edges of the scan the tick used, so GRU histories can
be built per candidate; no NaN covariance on modelled edges; `dg_pose_costs` and
`path_time` match `path_full` on all 1926 feasible candidates. `episodes.csv` named
both stages with their files and seeds; both scenario files were copied. About
1.6 GB/h of bag. The dynamic_gap C++ tests (56) and il_tools tests pass (two
`il_goal_cursor_collect` port tests fail before and after, environment).

## Still not recorded

Propagated future scans (`DynamicScanPropagator`) and the planner's internal
costmaps; neither is an input the gap scorer or the GRU reads. Pedestrians step on
wall time while the robot runs on sim time, so their speed relative to the robot
varies with the real-time factor (readable from `episodes.csv` wall vs. sim time).
