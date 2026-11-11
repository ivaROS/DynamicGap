# 16 — Manual mode with no cursor drove under obstacle avoidance

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `include/dynamic_gap/Planner.h`, `src/Planner.cpp`, `test/test_manual_selection.cpp`
**Behaviour change:** Manual mode only: with nothing selected, the robot stands still.

## Problem
In manual mode, when there was no cursor yet, or no feasible gap trajectory, `getCursorSelectedTrajectory` returned false with `trajFlag = NONE` (`Planner.cpp:869`, `:903` before the change). `ctrlGeneration` then ran obstacle-avoidance control, which moves the robot on its own. The "no gaps" early return did the same.

## Why it mattered for data collection
The robot moved without operator input, and those ticks looked like operator-driven motion in `cmd_vel`.

## What changed
- `manualHoldStill_` is reset at the start of each tick and set in manual mode when the selection yields nothing (no usable cursor — never received, or tool inactive, see issues 29 and 40 — no feasible candidate, and no click-to-move) and on the "no gaps" return.
- `ctrlGeneration` returns a zero twist in manual mode when `manualHoldStill_` is set.
- Not-ready and in-collision ticks were left on obstacle-avoidance control; issue 37 makes them hold still too.
- A still mouse is never "no cursor": the tool republishes every 0.1 s.

## Effects
- Autonomous mode: no change.
- Manual mode: the robot waits for a selection instead of wandering.
- Recorded data / metrics: `hold_still` in the sample marks these ticks.

## Test
`test/test_manual_selection.cpp`: `NoGapsHoldsStill` (a manual tick with no gaps sets hold-still and `NONE`), `HoldStillGivesZeroVelocity`.

```
cd $OV && catkin build --no-status dynamic_gap rviz_plugins --catkin-make-args run_tests && catkin_test_results build/dynamic_gap && catkin_test_results build/rviz_plugins
```
