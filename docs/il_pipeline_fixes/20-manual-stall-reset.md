# 20 — Low speed in manual mode triggered "Planning Failed"

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`, `test/test_planner_manual_mode.cpp`
**Behaviour change:** Manual mode only: a slow or still robot is not treated as a planning failure.

## Problem
`Planner::recordAndCheckVel` (`Planner.cpp:2670` before the change) keeps the last five command-velocity norms. If their sum is at most 1.0 while following a gap, it logs "Planning Failed", calls `reset()` (drops the current trajectory and model IDs) and returns false, so the plugin returns `FAILURE` to move_base_flex.

## Why it mattered for data collection
In manual mode the operator may hold the robot still, or pick a gap that needs a slow approach. That triggered resets and controller failures (and move_base_flex recovery) that had nothing to do with the operator's choice, and wiped the trajectory being followed.

## What changed
`recordAndCheckVel` still records the velocity, then returns `true` in manual mode before the stall check.

## Effects
- Autonomous mode: no change.
- Manual mode: no stall resets and no `FAILURE` from the stall check.
- Recorded data / metrics: manual episodes no longer contain recovery behaviour caused by the stall check.

## Test
`test/test_planner_manual_mode.cpp`: `StallFailsInAutonomousMode` (five zero commands with a gap flag → false) and `StallIsAcceptedInManualMode` (ten → always true).

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
