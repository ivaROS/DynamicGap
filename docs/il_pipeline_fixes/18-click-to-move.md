# 18 — Click-to-move in autonomous mode; tracking ended early

**Status:** Fixed (planner part) · **Repo(s):** dynamic_gap · **Files:** `include/dynamic_gap/Planner.h`, `include/dynamic_gap/utils/ManualSelection.h`, `src/Planner.cpp`, `test/test_planner_manual_mode.cpp`
**Behaviour change:** Autonomous mode ignores `/traj_creation_point`. In manual mode a click-to-move trajectory is followed until the robot is within `xy_goal_tolerance` of the point, or for at most twice the estimated time.

## Problem
- The autonomous branch of `runPlanningLoop` handled `/traj_creation_point` (`Planner.cpp:2162` before the change): one click in RViz overrode the autonomous planner for a whole trajectory. The same branch read `hasTrajCreationPoint_` without `manualSelectionMutex_` (issue 28).
- Tracking ended after `dist / vx_absmax` (`Planner.cpp:77-81`), the time to cover the straight-line distance at full speed. The controller never drives that fast, so tracking stopped before the robot arrived.
- The RViz tool's status text names left-click as "toggle continuous gap selection" though left-click sends a click-to-move point. That text is fixed with the tool changes (issues 11, 29).

## Why it mattered for data collection
A stray click in an autonomous benchmark changed the run. In manual mode the robot stopped short of clicked points and went back to the cursor selection mid-way.

## What changed
- `trajCreationPointCB`: in autonomous mode the point is dropped and both click-to-move flags are cleared.
- `runPlanningLoop`: the autonomous branch calls `pickTraj` only. Click-to-move is handled in the manual branch, under the lock, as before.
- `tryHandleTrajCreationPoint`: the estimated time is stored in `createdTrajEstimatedTime_`; tracking ends when `isCreatedTrajDone` (in `ManualSelection.h`) says so: the robot is closer than `cfg_.goal.xy_global_goal_tolerance` (`xy_goal_tolerance`, 0.25 m for dgap) to the point, or tracking has lasted more than twice the estimate. The global-goal tolerance is used because the clicked point is a final target the controller stops at; the 0.1 m waypoint tolerance is tighter than the controller settles. The trajectory lifespan is set to the same 2× limit.

## Effects
- Autonomous mode: `/traj_creation_point` has no effect.
- Manual mode: click-to-move ends at the target or after 2× the estimate.
- Recorded data / metrics: autonomous runs can no longer contain operator input.

## Test
`test/test_planner_manual_mode.cpp`: `CreatedTrajDoneTest` (ends at the target, or only after twice the estimate); `TrajCreationPointIgnoredInAutonomousMode` (point dropped, tracking flag cleared); `TrajCreationPointKeptInManualMode`.

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
