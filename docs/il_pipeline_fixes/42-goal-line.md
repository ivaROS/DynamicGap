# 42 — Robot-to-goal line for the operator

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/visualization/GoalVisualizer.cpp`, `include/dynamic_gap/visualization/GoalVisualizer.h`, `src/Planner.cpp`, `rviz/arena.rviz`
**Behaviour change:** Manual mode draws a faint yellow line (arrow) from the robot to the global goal in RViz. Autonomous mode: no change.

## Problem
The operator had no cue in RViz for where the goal is relative to the robot. A newer MGS version had this line, but it was never committed: it only exists in `stash@{1}` of dynamic_gap ("WIP on manual_gap_selection: f27f357", 2026-08-17).

## What changed
Ported from that stash:
- `GoalVisualizer::drawGoalArrow` publishes `/rto/goal_arrow` (`visualization_msgs/Marker`, ARROW, ns `goal_arrow`) from the robot position to the global goal, odom frame, in the global goal's yellow at alpha 0.2.
- `Planner::laserScanCB` draws it with the global goal, in manual mode only, on every scan.
- `rviz/arena.rviz`: Marker display "toGoal" on `/rto/goal_arrow`.

Not ported from the stash: it also set the global goal marker itself to alpha 0.2 (the goal stays opaque here; the line has its own colour), and RViz layout changes (duplicate Displays panels, window geometry, disabled namespaces).

## Test
Headless launch on master 11312 (`start_trials.launch man_gap:=true`, a cursor published at the goal): `/rto/goal_arrow` carried ns `goal_arrow`, frame `rto/odom`, start at the robot (3.18, 3.96), length 8.75 m, alpha 0.2. Unit tests: 38 pass.
