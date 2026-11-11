# 43 — Gap selection: cursor direction only, switch buffer

**Status:** Fixed · **Repo(s):** dynamic_gap, arena-rosnav · **Files:** `include/dynamic_gap/utils/ManualSelection.h`, `src/Planner.cpp`, `include/dynamic_gap/Planner.h`, `test/test_manual_selection.cpp`; arena-rosnav `arena_bringup/launch/start_trials.launch`, `task_generator/task_generator/tasks/modules/benchmark.py`, `task_generator/test/test_benchmark_man_gap_selection.py`
**Behaviour change:** Manual mode only. A gap is selectable only if its trajectory ends in the cursor's direction from the robot; with none there, nothing is selected and the robot stands still. A different gap replaces the followed one only when it is clearly nearer the cursor. Autonomous mode: no change.

## Problem
The selected gap changed too often for the operator to make progress. Each tick took the feasible candidate whose terminal point was nearest the cursor, with no memory of the previous choice, and could take a gap pointing away from the cursor when none pointed towards it.

Recorded sessions (2026-10-21/22, every tick present, `tick_seq` without gaps), switches of the selected gap between consecutive ticks:

| cause | 12-41 | 13-09 | 14-33 | pilot_01 | back within 1 s |
|---|---|---|---|---|---|
| close call: old gap still feasible, another terminal slightly nearer | 60% | 35% | 42% | 30% | 158 / 304 |
| old gap gone, next nearest elsewhere | 25% | 17% | 29% | 23% | 5 / 174 |
| mouse moved ≥ 10 px | 2% | 26% | 13% | 20% | 25 |
| mouse nudged < 10 px | 2% | 17% | 8% | 18% | 20 |
| same opening, new model ids | 10% | 2% | 6% | 6% | 1 |
| switches / usable ticks | 132 / 1314 | 223 / 494 | 264 / 568 | 115 / 469 | |

The later runs switched about every second tick. Close calls flip back and forth; the new terminal was a median 2–20% nearer than the old one.

## What changed
`selectManualTrajectory`, gap-cursor branch:
1. `inCursorDirection`: feasible candidates whose terminal bearing from the robot is within `/man_gap_max_bearing_deg` of the cursor bearing.
2. `nearestCandidate` over those.
3. `holdCurrentCandidate`: the followed gap (same left/right model ids, still selectable) is kept unless the nearest one's cursor distance is below `/man_gap_switch_ratio` times the followed one's.
4. No selectable gap: no selection (robot holds still, as with no cursor), and the followed gap is forgotten, so the next selected gap starts a new trajectory.

Parameters, global, set by `start_trials.launch` (args of the same name) and therefore saved in each session's `params.yaml`; a benchmark relaunch forwards them:
- `man_gap_max_bearing_deg`, default 60, in (0, 180]
- `man_gap_switch_ratio`, default 0.7, in (0, 1]; 1 is the old nearest-wins rule

`il_tools/il_check_session`'s "selection reproducible" check replays this rule (with the parameters from `params.yaml`) since the follow-up to this fix; before it, it recomputed plain nearest-feasible and failed on 15–19% of cursor ticks of every post-43 session (plan 3.3).

The per-tick sample is unchanged: `selected_idx` is the gap followed, `cursor_dist` and `cursor_bearing_diff` per candidate still allow recomputing either rule offline.

## IL data
Labels from before and after this fix follow different rules; keep the sessions apart, or filter on the two parameters in `params.yaml` (absent there = before this fix). The rule depends only on the tick's candidates, the cursor and the previous selection, so a policy given the previous selection can reproduce it.

## Choosing the defaults
Open-loop replay of the recorded ticks (the robot would have driven differently, so approximate): ratio 0.7 removes 25–35% of switches, lower ratios little more. ±60° leaves no selection on 1–15% of ticks, ±45° on 4–25%, ±90° rarely changes anything.

## Test
Unit tests (`ManualSelectionHelpers.InCursorDirection`, `.HoldCurrentCandidate`): 80 pass (`cd ~/il_overlay_ws && catkin build --no-status dynamic_gap --catkin-make-args run_tests && catkin_test_results build/dynamic_gap`). arena-rosnav: `test_benchmark_man_gap_selection.py` passes; `start_trials.launch` parses. Not yet tried in a live run.
