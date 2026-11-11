# 49 — Automatic operator: cursor held toward the goal

**Status:** New · **Repo(s):** dynamic_gap, arena-rosnav · **Files:** `il_tools/il_goal_cursor`; arena-rosnav `arena_bringup/launch/start_trials.launch`, `task_generator/task_generator/tasks/modules/benchmark.py`, `task_generator/task_generator/tasks/obstacles/random.py`, `arena_bringup/configs/benchmark/suites/il_goal_cursor.yaml`, `il_goal_cursor_pilot.yaml`
**Behaviour change:** None unless `il_goal_cursor:=true`.

## Problem
`with_terminal_seed0`, run in closed loop on `gru_learn_scenario_6`, followed its own momentum and never reached the goal (see `il_data/models/worldvel_goaldir_seed0/NOTES.md`). The operator data has about 4 goal directions, and in it "keep going" and "go to the goal" are nearly always the same choice, so the model cannot tell them apart. More episodes with varied start/goal pairs, where the goal-directed gap differs from the current heading, need many hours of driving.

## What changed
- `il_tools/il_goal_cursor` (ROS node): publishes `gap_selection_cursor` like the RViz ContinuousGapSelectionTool (10 Hz wall clock, `tool_active`, `pick_ok`, map frame, `last_move_wall` = every publish). The planner in manual mode selects exactly as for a person, so ticks are `SOURCE_GAP_CURSOR` labels and everything downstream (checks, dataset, training) is unchanged.
  - Cursor: `~lookahead` (3 m) from the robot toward the target, as a person hovers the mouse a little ahead. The planner's own rule (±`man_gap_max_bearing_deg` cone, nearest trajectory end, 0.7 switch ratio) then picks the gap.
  - Target `~target`: `goal`, `waypoint` (local waypoint of the global plan) or `auto` (default): the goal when the straight line to it (±0.3 m) is free in `/map`, else the waypoint, to go around walls. Obstacles spawned by the task generator are not in `/map`.
  - No gap within the cone of the target (e.g. an obstacle in front of the goal): the planner would select nothing and the robot would hold still until the timeout. The cursor is then put on the trajectory end whose direction is closest to the target's.
  - Robot pose, goal and waypoint come from the planner's `manual_gap_sample` (one tick old). Nothing is published before the first planned sample.
- `start_trials.launch`: `il_goal_cursor:=true` starts the node in the robot namespace (needs `man_gap:=true`); param `/il_goal_cursor`. The benchmark relaunch carries it (`_il_goal_cursor_args`).
- `obstacles/random.py`: the model weights are normalised to probabilities (`_probabilities`). Every model has weight 1, so with more than one model in `RANDOM.<kind>.models` numpy raised "probabilities do not sum to 1" and the task generator died before the first episode.
- Suites: `il_goal_cursor.yaml` (5 maps × 30 episodes: `map_empty`, `map_empty_medium`, `map1_small`, `experiment_rooms`, `small_warehouse`), `il_goal_cursor_pilot.yaml`. Random start and goal (`tm_robots: random`), no pedestrians, random static obstacles of the models that have a flatland YAML (shelf, long_shelf, table, pallet, office_desk, tree; `cuboid` and `cylinder` are Gazebo-only).

## Run
```
source /home/abdel/MGS_arena_has_issues/arena_ws/devel/setup.bash
roslaunch arena_bringup start_trials.launch man_gap:=true il_goal_cursor:=true \
    tm_modules:=benchmark benchmark_suite:=il_goal_cursor.yaml \
    record_il:=true il_data_root:=/home/abdel/il_data il_operator:=goal_cursor
```
Each map is its own session (`<runid>_<stage>`). Build with `--lag 0` (the node has no reaction time; its cursor is one tick old).

Unattended, with a restart when move_base_flex dies (`mbf_costmap_nav` segfault at a reset): `il_tools/il_goal_cursor_collect il_goal_cursor.yaml --data-root /home/abdel/il_data`. It launches one stage at a time on the port of `ROS_MASTER_URI` and refuses to start when a master, launch or pending benchmark relaunch is on that port (a process's `-p`, else its `ROS_MASTER_URI`). It watches and stops only the processes it started (marked in their environment), and its suite files (removed after each launch) and launch logs carry the port, so a run on another port is left alone. The planner counts as crashed after 60 s without `mbf_costmap_nav` and no pending relaunch (`delay_restart.py`). A stage counts as finished when its own benchmark run (found by the suite file in `configs/benchmark/logs`) logged `BENCHMARK COMPLETED SUCCESSFULLY`, also if move_base_flex exited first during that shutdown; any other end stops what is left of the launch, a pending benchmark relaunch (`delay_restart.py`) first, and restarts it for the episodes not finished (counted in that run's sessions, `<runid>_<stage>`), and three launches in a row without a finished episode skip it.

## Effects
- In a map with no obstacles the planner sees a single gap on every tick: pilot 1 (`map_empty`, 3 episodes, all reached the goal) had 431 usable ticks and `ticks_with_choice: 0`, nothing to learn. Hence the random obstacles.
- Episodes that time out (the robot can still be trapped) should be left out of a training set; their labels are not goal-directed.
