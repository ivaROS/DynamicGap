# 28 — Click-to-move handled without the lock in autonomous mode

**Status:** Fixed (with issue 18) · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`
**Behaviour change:** See issue 18. No separate change.

## Problem
The autonomous branch of `runPlanningLoop` read `hasTrajCreationPoint_` / `isTrackingCreatedTraj_` and called `tryHandleTrajCreationPoint` without `manualSelectionMutex_`, while `trajCreationPointCB` wrote them under it.

## Why it mattered for data collection
A click arriving during a planning tick could be half-applied.

## What changed
Issue 18 removes click-to-move from autonomous mode, so the autonomous branch no longer touches this state. The only remaining caller, `getCursorSelectedTrajectory` in the manual branch, holds the lock.

## Effects
- Autonomous mode: see issue 18.
- Manual mode: no change.
- Recorded data / metrics: none.

## Test
Covered by the issue 18 tests in `test/test_planner_manual_mode.cpp`.
