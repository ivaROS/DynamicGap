# 37 — Manual mode drove under the planner's own control when in collision

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`, `test/test_manual_selection.cpp`
**Behaviour change:** Manual mode only: in collision or before the planner is ready, the robot stands still.

## Problem
Issue 16 made manual mode stand still when nothing is selected, but kept the not-ready and in-collision returns of `runPlanningLoop` on obstacle-avoidance control. In collision (closest scan range < robot radius + 7.5 mm) the scan is dropped, planning is skipped, `trajFlag` is `NONE`, and `ctrlGeneration` ran `obstacleAvoidanceControlLaw`, ignoring the operator's selection.

Seen in session `pilot_01` (2026-10-21): 12 `STATUS_COLLIDING` ticks; on the last 5 the robot was driven at 0.4–0.75 m/s and 1 rad/s while the operator held a selection. The operator reported the planner "fighting" the selection.

## Why it mattered for data collection
With the continuous gap selection tool on, the planner must never move the robot. Those ticks put planner motion into `cmd_vel` and into the observations of the following ticks.

## What changed
- `runPlanningLoop` sets `manualHoldStill_` in manual mode on the not-ready and in-collision returns, like the "no gaps" return.
- `ctrlGeneration` in manual mode returns a zero twist unless the tick has a `GAP` trajectory with poses (cursor gap or click-to-move). Obstacle-avoidance, idling and ungap control never run in manual mode, whatever path leads there.

Unchanged: the projection operator still adds its obstacle-repulsion term to the tracking command of the operator's trajectory (`projection_operator: true`), so close to obstacles the robot does not follow the selected path exactly.

## Effects
- Autonomous mode: no change.
- Manual mode: in collision the robot waits. If it stays in collision (pushed against a wall) it stays still until a pedestrian moves away or the episode times out; the operator cannot steer out, because no gaps are planned while in collision.
- Recorded data: `hold_still` is now also true on `STATUS_NOT_READY` and `STATUS_COLLIDING` ticks.

## Test
`test/test_manual_selection.cpp`: `CollidingHoldsStill`, `NotReadyHoldsStill`, `NoSelectionNeverRunsPlannerControl` (`NONE`, `IDLING`, `UNGAP`, and `GAP` with an empty trajectory all give a zero twist without touching the trajectory controller).

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
