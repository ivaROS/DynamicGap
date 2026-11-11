# 24 — scan_ newer than the gaps on collision ticks

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`, `test/test_planner_manual_mode.cpp`
**Behaviour change:** None, except that `scan_` always matches the current gaps.

## Problem
`laserScanCB` assigned `scan_ = scan` (`Planner.cpp:286` before the change), then returned early when the robot was in collision, before gap detection. While colliding, `scan_` was the newest scan but the gaps, models and ego-circles came from an older one.

## Why it mattered for data collection
Anything that pairs `scan_` with the gaps (the per-tick sample, issue B of the IL plan, and the static future-scan path in `runPlanningLoop` if a tick slips in as the flag flips) got a scan that did not produce those gaps.

## What changed
The collision test uses the incoming `scan`. `scan_` is assigned after it, so it is only replaced when gaps are computed from the same scan.

## Effects
- Autonomous mode: none. While colliding, `runPlanningLoop` returns early (issue 21) and nothing reads `scan_`; ego-circles were already not updated on collision ticks.
- Manual mode: same.
- Recorded data / metrics: `scan_` and the gaps always come from the same scan.

## Test
`test/test_planner_manual_mode.cpp`, `CollidingScanDoesNotReplaceScan`: a scan with a range below the inscribed radius sets `colliding_` and leaves `scan_` unset.

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
