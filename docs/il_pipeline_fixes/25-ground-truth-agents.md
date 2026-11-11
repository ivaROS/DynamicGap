# 25 — Ground-truth pedestrian maps never cleared

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `include/dynamic_gap/Planner.h`, `src/Planner.cpp`, `test/test_planner_manual_mode.cpp`
**Behaviour change:** Both modes: agents missing from the latest pedsim message, and all agents at an episode reset, are dropped from the ground-truth maps.

## Problem
`pedOdomCB` (`Planner.cpp:622,633` before the change) wrote each agent into `currentTrueAgentPoses_` / `currentTrueAgentVels_` by id and never removed one. Agents removed at a reset, or that left the simulation, stayed at their last pose forever. The maps were written without a lock while `laserScanCB` read them (under `gapMutex_`) in the gap estimators.

## Why it mattered for data collection
The perfect estimator and the pedestrian visualisation used agents that no longer existed, with stale poses that were also shifted relative to the robot after the reset teleport.

## What changed
- `pedOdomCB` builds new maps from the message and swaps them in under `gapMutex_`.
- New subscriber to `/scenario_reset` (`std_msgs/Int16`): `scenarioResetCB` clears both maps under `gapMutex_`.

## Effects
- Autonomous mode: estimators that use ground truth see only current agents. With the Kalman filter estimator (the default) the maps are only used for visualisation.
- Manual mode: same.
- Recorded data / metrics: none directly.
- The `/scenario_reset` subscriber is reused by the IL sample for the episode id.

## Test
`test/test_planner_manual_mode.cpp`, `AgentMapsFollowLatestMessageAndReset`: three agents, then a message with one, leaves one; a reset empties both maps.

```
cd $OV && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap
```
