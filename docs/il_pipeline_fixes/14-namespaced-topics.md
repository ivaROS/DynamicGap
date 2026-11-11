# 14 — Manual-mode topics were global

**Status:** Fixed · **Repo(s):** dynamic_gap, arena-rosnav · **Files:** dynamic_gap `src/Planner.cpp`, `rviz/arena.rviz`, `include/dynamic_gap/utils/manual_traj_selector.py`, `test/test_arena_rviz.py`; arena-rosnav `utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.{h,cpp}`
**Behaviour change:** Topic names (below).

## Problem
The planner subscribed to `/gap_selection_point`, `/traj_creation_point`, `/manual_traj_selection` and published `/manual_candidate_markers` (`Planner.cpp:184-192` before the change), all absolute. Every robot's planner would read the same cursor, and their markers would overwrite each other.

## Why it mattered for data collection
Multi-robot runs could not be recorded separately, and a recorded topic name did not say which robot it belonged to.

## What changed
Planner topics are relative to the planner's node handle, i.e. the robot namespace (`/rto/...` for the default robot):

| Topic | Type | Direction |
|---|---|---|
| `/rto/gap_selection_cursor` | `manual_gap_msgs/CursorState` | tool → planner (replaces `/gap_selection_point`) |
| `/rto/traj_creation_point` | `geometry_msgs/PointStamped` | tool → planner |
| `/rto/manual_traj_selection` | `std_msgs/Int32` | `manual_traj_selector.py` → planner |
| `/rto/manual_candidate_markers` | `visualization_msgs/MarkerArray` | planner → RViz |
| `/rto/manual_gap_sample` | `manual_gap_msgs/ManualGapSample` | planner → recorder (manual mode only) |

`/scenario_reset` and `/scenario_reset_begin` stay global (one task generator).

- The RViz tool has a "Topic Prefix" property (default `/rto/`, saved in `arena.rviz`); its two topics are `<prefix>gap_selection_cursor` and `<prefix>traj_creation_point`.
- `arena.rviz`: the candidate marker display reads `/rto/manual_candidate_markers`.
- `manual_traj_selector.py`: private param `~topic`, default `/rto/manual_traj_selection`.

## Effects
- Autonomous mode: no change (the topics are manual-mode only; the planner still subscribes but nothing publishes).
- Manual mode: same behaviour under new names.
- Downstream: `record_il.launch` (issue 07) records these names.

## Test
`test/test_arena_rviz.py` checks the tool prefix and marker topic in `arena.rviz`. `rviz_plugins/test/test_cursor_state.cpp` checks the prefix joining. The planner topics were checked in the sim smoke test (`rostopic list`).
