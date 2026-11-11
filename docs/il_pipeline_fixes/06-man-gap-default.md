# 06 — Manual selection mode on by default

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `include/dynamic_gap/Planner.h`, `include/dynamic_gap/utils/ManualSelection.h`, `src/Planner.cpp`, `test/test_planner_manual_mode.cpp`
**Behaviour change:** Without a valid `/man_gap` the planner runs autonomously (it used to run in manual mode).

## Problem
`Planner.h:165` (before the change) initialised `manualSelectionMode_ = true`, and `Planner::initialize` read the parameter with `ros::param::get("/man_gap", manualSelectionMode_)`, ignoring the return value. If `/man_gap` was missing, or was not a bool (for example the string `"false"` from a hand-written launch), the planner stayed in manual mode and printed nothing.

## Why it mattered for data collection
An autonomous benchmark launched without `/man_gap` silently waited for a cursor and drove under obstacle avoidance; its results were filed as autonomous. Nothing in the logs showed which mode ran.

## What changed
- `Planner.h`: `manualSelectionMode_ = false`.
- New `include/dynamic_gap/utils/ManualSelection.h` with `parseManualModeParam(hasParam, value, mode)`: a bool is used as is; the strings `true`/`false` (any case) are parsed; anything else, or a missing parameter, gives `false`.
- `Planner::initialize` reads `/man_gap` as an `XmlRpcValue`, warns if it is missing, logs an error if it cannot be read, and always logs the final mode (`manual gap selection: on|off`).

## Effects
- Autonomous mode: unchanged when `/man_gap` is `false` (what `start_trials.launch` sets by default). Runs without the parameter are now autonomous.
- Manual mode: unchanged when `/man_gap` is `true`.
- Recorded data / metrics: the mode is in the planner log, and in `params.yaml` (issue 05).

## Test
`test/test_planner_manual_mode.cpp`, `ManualModeParamTest`: missing → off; bools used; "True"/"FALSE" parsed; "yes" and an int → off; a new `Planner` starts with manual mode off.

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
