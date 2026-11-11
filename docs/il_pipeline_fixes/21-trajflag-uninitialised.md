# 21 — trajFlag uninitialised on early returns

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/DynamicGapPlanner.cpp`, `src/Planner.cpp`, `include/dynamic_gap/Planner.h`, `CMakeLists.txt`, `package.xml`, `test/`
**Behaviour change:** Both modes: ticks where the planner is not ready or the robot is in collision now always take the `NONE` path in `ctrlGeneration`.

## Problem
`DynamicGapPlanner::computeVelocityCommands` declared `int trajFlag;` without a value (`DynamicGapPlanner.cpp:98` before the change) and passed it to `Planner::runPlanningLoop`. Two early returns in `runPlanningLoop` did not set it: "not ready" (no laser scan, no global goal, or not initialised) and "in collision". `ctrlGeneration` and `recordAndCheckVel` then read an indeterminate value.

## Why it mattered for data collection
On those ticks the controller branch (idle, obstacle avoidance, feedback) and the stall check depended on whatever was on the stack. The robot's behaviour at episode start and in collision was not reproducible, so recorded `cmd_vel` on those ticks could not be trusted.

## What changed
- `DynamicGapPlanner.cpp`: `int trajFlag = NONE;`.
- `Planner.cpp`: both early returns set `trajFlag = NONE` next to `chosenTraj = Trajectory()`.
- Test support: `friend class PlannerTestAccess` in `Planner.h`; a gtest target in `CMakeLists.txt` with `<test_depend>rosunit</test_depend>`; the library now depends on `${PROJECT_NAME}_gencfg` so the test target builds on a clean build space.

## Effects
- Autonomous mode: not-ready and collision ticks use `NONE` (obstacle-avoidance control), as the third early return ("no gaps") already did.
- Manual mode: same.
- Recorded data / metrics: `cmd_vel` on those ticks is now deterministic.

## Test
`test/test_trajflag_uninitialised.cpp` (with `test/planner_test_access.h`, `test/fake_ros_master.h`, `test/test_main.cpp`) calls `runPlanningLoop` with a stale `trajFlag` for each early return (not initialised, no scan, no goal, colliding) and asserts it comes back as `NONE`. It fails on the old code.

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
