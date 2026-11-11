# 17 — Cursor pick never set start time, lifespan or model IDs

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `include/dynamic_gap/Planner.h`, `src/Planner.cpp`
**Behaviour change:** Manual mode only: the planner's current gap (model IDs), tracking start time and lifespan follow the cursor-selected gap.

## Problem
`getCursorSelectedTrajectory` called `setCurrentTraj(nearestTraj)` every tick (`Planner.cpp:897` before the change) but never set the tracking start time, the lifespan or the current left/right gap model IDs. The autonomous path sets all of them in `changeTrajectoryHelper` (`:1764-1771`).

## Why it mattered for data collection
The planner's own record of which gap it was in (`currentLeftGapPtModelID`, `currentRightGapPtModelID`) stayed at whatever the last autonomous or reset state left, so it could not be used to check or label the operator's choice.

## What changed
In `selectManualTrajectory`, when the selected candidate's model-ID pair differs from the previous selection, the planner sets the tracking start time, the lifespan (trajectory timing span, as `changeTrajectoryHelper` does), the current left/right model IDs, and counts a trajectory change. Every tick it still tracks the freshest trajectory through the selected gap. Click-to-move and episode resets clear the stored selection.

## Effects
- Autonomous mode: no change.
- Manual mode: the robot follows the same trajectories; only the bookkeeping changes.
- Recorded data / metrics: `selected_left_model_id` / `selected_right_model_id` in the sample match the planner's current gap.

## Test
Covered by the sim smoke test (selected IDs recorded on every cursor tick) and code review; the identity comparison needs gap tubes with live estimators and is not unit tested.
