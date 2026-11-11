# IL fixes A and B — One candidate list and one sample per planning tick

**Status:** Fixed · **Repo(s):** dynamic_gap, arena-rosnav · **Files:** dynamic_gap `include/dynamic_gap/Planner.h`, `include/dynamic_gap/utils/ManualSelection.h`, `src/Planner.cpp`, `CMakeLists.txt`, `package.xml`, `test/test_manual_selection.cpp`; arena-rosnav `utils/ros/msgs/manual_gap_msgs/` (new)
**Behaviour change:** Manual mode only, see issues 16 and 17. Autonomous mode publishes no sample and is unchanged.

Plan sections 1.3 (fix A) and 1.4 (fix B) of `IL_DATA_COLLECTION_PLAN.md`. Closes DG-01, DG-02, DG-03, DG-08, DG-13, DG-14, DG-15, R-06, R-10, R-12, R-14, R-15, R-19, R-28; partly DG-04.

## Problem
- The operator's choice existed only inside `getCursorSelectedTrajectory` (DG-01). Candidates went out only as text markers and were then deleted (DG-02).
- The marker publisher and the selector each built their own candidate list with their own skip rules (DG-13), and the dense `display_id` differed from the sparse trajectory index (DG-14). Gap identity was not recorded (DG-15).
- Detection (25 Hz), selection (planning tick) and cursor (10 Hz) ran on different threads; a sample could only be rebuilt by a three-way join offline (DG-03).
- The goal was nowhere in the data (R-10).

## What changed
**Messages** (`manual_gap_msgs`, separate package so offline tools need not build the planner): `CursorState`, `ManualGapCandidate`, `ManualGapSample` (schema version 1). Field definitions and frames are in the `.msg` files.

**Fix A — one candidate list** (`selectManualTrajectory`):
- One entry per gap tube, in tube order, infeasible ones included. `traj_idx` = tube index; `display_id` = dense number over feasible (shown) candidates, -1 otherwise (`denseDisplayIds`).
- Identity: the left/right point estimator IDs of the tube's first gap; bearing of the gap centre as a second cue.
- The same list is drawn (`publishManualCandidateMarkers`), selected from (`nearestCandidate` on terminal poses, feasible only; since issue 43 restricted to the cursor's direction and sticky, see 43), and recorded. The selected index is the position in this list.

**Fix B — one sample per tick** on `/rto/manual_gap_sample` (manual mode only):
- Published on every return path of `runPlanningLoop` (a scope guard), with `status` = PLANNED, NOT_READY, COLLIDING or NO_GAPS; `tick_seq` increments every tick.
- Observation: the scan the gaps were computed from (copied under `gapMutex_` together with the gaps), its stamp, robot pose (odom) and velocity, goal (transformed with this tick's odom→robot transform) and local waypoint in the robot frame, all candidates with geometry, estimator states, gap goal, terminal pose, downsampled path (every 5th pose and the last), and cursor distance and bearing difference.
- Cursor: the last `CursorState` verbatim, its wall-clock receipt time, the odom point used, `cursor_valid` (usable, issue 29), `cursor_live` (usable, `pick_ok`, and the point was transformed to odom). On ticks that reach selection these are recorded inside `selectManualTrajectory` under the same lock as the selection, so a cursor message arriving later in the tick cannot change them.
- Choice: `source` (NONE, GAP_CURSOR, CLICK_TO_MOVE), `selected_idx`, selected model IDs, click target, `hold_still`.
- Episode: `episode_id` from `/scenario_reset`; `in_reset_window` while a reset is in progress (`/scenario_reset_begin` id ahead of `/scenario_reset`) and for 1 s wall time after it (`isInResetWindow`).
- A tick is a usable expert label when `source == SOURCE_GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0` (`il_bags.is_usable_label`; ticks that end before selection record `selected_idx = -1`, issue 45).

**Reset** (manual mode, plan P4 / R-26): on `/scenario_reset` the planner drops the previous gaps and their estimators (no inheritance across the teleport) and the click-to-move state. The stored selection, current trajectory and current model IDs belong to the planning thread; the callback only sets a flag and `selectManualTrajectory` clears them at the start of the next tick, so the callback never writes `currentTraj_` while a tick reads it. The chosen trajectory is copied inside `selectManualTrajectory`, under its lock. The cursor itself is not cleared: the tool re-projects the screen point after the teleport, and a still mouse keeps its selection.

## Effects
- Autonomous mode: no sample publisher, no change in behaviour (the candidate list is only built in manual mode).
- Manual mode: one `ManualGapSample` per tick (≈10 Hz).
- `ungap_would_apply` / `idle_would_apply` are reserved and always false (ungap and idling generation are disabled, PL-31).

## Test
`test/test_manual_selection.cpp`: display ids, nearest feasible candidate (ties to the lower index), cursor freshness, reset window, path downsampling, the scope guard firing on every return, the reset clearing manual state (and not in autonomous mode); `selectManualTrajectory` on three candidates (one infeasible): selected index, display ids, cursor distances, recorded cursor, chosen trajectory and lifespan; no cursor → no selection; a reset applied at the next tick; a cursor whose transform failed is used but not live. The sim smoke test checks that samples arrive with contiguous `tick_seq`, that cursor selections are recorded, and that recomputing the nearest endpoint from the recorded cursor and candidates reproduces `selected_idx` on every cursor tick.

```
cd $OV && catkin build --no-status dynamic_gap rviz_plugins --catkin-make-args run_tests && catkin_test_results build/dynamic_gap && catkin_test_results build/rviz_plugins
```
