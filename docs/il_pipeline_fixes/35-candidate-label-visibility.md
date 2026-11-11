# 35 — Candidate labels black on dark grey, 2 m tall

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`
**Behaviour change:** Manual mode only: numbers are white (selected: green) and 0.5 m tall.

## Problem
Unselected numbers were black (`r = g = b = 0`) with `scale.z = 2.0` (`Planner.cpp:776-790` before the change). The RViz background in `arena.rviz` is 48;48;48, so they were nearly invisible, and when visible they were 2 m tall and covered each other.

## What changed
Unselected numbers are white, the selected one green, `scale.z = 0.5`.

## Effects
- Autonomous mode: no change.
- Manual mode: readable numbers.

## Test
Visual; not unit tested.
