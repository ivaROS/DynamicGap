# 19 — RViz toolbar could teleport the robot, move the goal or force a reset

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `rviz/arena.rviz`, `test/test_arena_rviz.py`
**Behaviour change:** The three tools are gone from the arena.rviz toolbar.

## Problem
`arena.rviz` offered `rviz/SetInitialPose` (`/initialpose`, teleports the robot through the task generator's RViz UI module), `rviz/SetGoal` (`/goalpose`, moves the goal) and `rviz/PublishPoint` (`/clicked_point`, forces an episode reset). One stray click during collection changed the episode.

## Why it mattered for data collection
An operator using the gap selection tool could switch tools by shortcut or by mistake and silently teleport the robot or end the episode; the data would show it as an ordinary episode.

## What changed
The three tool entries are removed. Kept: Interact, MoveCamera, Select, FocusCamera, Measure, ContinuousGapSelectionTool (with `Topic Prefix: /rto/`, issue 14).

## Effects
- Autonomous mode: no change in the planner; the tools are no longer in the shipped RViz.
- Manual mode: same.
- They can still be added by hand in RViz if needed for debugging.

## Test
`test/test_arena_rviz.py` parses `arena.rviz` and checks the tool list.

```
cd src/planners/dynamic_gap && python3 -m unittest discover -s test -p "test_*.py" -v
```
