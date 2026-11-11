# 13 — Candidate markers never deleted

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`
**Behaviour change:** Manual mode only: candidate numbers from earlier ticks disappear.

## Problem
`publishManualCandidateMarkers` only added markers with ids 0..n-1 and never deleted any (`Planner.cpp:735-818` before the change). When the number of candidates fell from five to three, numbers 3 and 4 stayed on screen at their old positions.

## Why it mattered for data collection
The operator could aim at a number that no longer existed; the selection then went to whatever candidate was nearest.

## What changed
Every published `MarkerArray` starts with a `DELETEALL` marker (namespace `manual_candidate_ids`), so only this tick's numbers are shown.

## Effects
- Autonomous mode: no change (markers are manual mode only).
- Manual mode: the screen shows exactly the current candidates.

## Test
Checked in the sim smoke test (markers topic) and by code review; marker contents are not unit tested.
