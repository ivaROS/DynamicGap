# 12 — Duplicate RViz node

**Status:** Fixed · **Repo(s):** arena-rosnav · **Files:** `arena_bringup/launch/start_trials.launch`, `arena_bringup/launch/start_arena.launch`, `arena_bringup/test/test_single_rviz.py`
**Behaviour change:** None (combinations that worked before start the same RViz; `headless:=0/1` with `show_rviz:=true`, which failed to launch, now launch)

## Problem
`start_trials.launch:172` always starts `<node name="rviz">` with `dynamic_gap/rviz/arena.rviz`.
It also passes `show_rviz` (default `true`) to the simulator include:
- `testing/simulators/flatland.launch:29-38`: when `headless` ≤ 1, includes `utils/rviz.launch`, which starts a second `<node name="rviz">` (no config) if `show_rviz` is true and `visualization == rviz`.
- `testing/simulators/gazebo.launch:32-34`: same, when `headless` ≤ 1 and `show_rviz` is true.

`start_arena.launch:115-131,161` has the same structure and the same clash.

`roslaunch --nodes` before the change, for both entry points and both simulators:

| headless | show_rviz | result |
|---|---|---|
| 0 | true | error: `roslaunch file contains multiple nodes named [/rviz]` |
| 1 | true | same error |
| 2 | true | one `/rviz` (arena.rviz) |
| 0, 1, 2 | false | one `/rviz` (arena.rviz) |

So `headless:=0` and `headless:=1` could not be launched with the default `show_rviz:=true`.

## Why it mattered for data collection
Runs with the simulator GUI (`headless:=0`) or with only RViz (`headless:=1`) failed at startup. Manual gap selection needs RViz with the arena.rviz tools, so the only working setup with default args was `headless:=2`.

## What changed
`start_trials.launch` and `start_arena.launch`:
- The flatland and gazebo includes get `show_rviz` = `false`. In both simulator files this arg only controls the plain RViz from `utils/rviz.launch`, so nothing else changes there.
- The arena.rviz node is unchanged. It is started for every `headless` and `show_rviz` value, as before, so `headless:=2` still opens arena.rviz.

`flatland.launch`, `gazebo.launch` and `utils/rviz.launch` are unchanged. No new params or topics. The `show_viz` param is still set from `show_rviz`.

After the change, for both entry points, both simulators, every `headless` value and both `show_rviz` values, exactly one `/rviz` with arena.rviz is started.

`show_rviz:=false` still does not turn arena.rviz off (same as before). Honouring it, by adding `if="$(arg show_rviz)"` to the arena.rviz node, is a possible follow-up.

Other includers:
- `training/flatland_simulator.launch` includes `utils/rviz.launch` with `show_rviz=false`: no RViz before or after (file unchanged).
- `launch/start_2ros_2rviz.py` runs `start_trials.launch` with default args (`headless:=2`, `show_rviz:=true`): one arena.rviz before and after.
- `task_generator/.../benchmark.py` relaunches the file named in `benchmark_launch_file` (`start_trials.launch` or `start_arena.launch`): covered above.

## Effects
- Autonomous mode: no change in the planner. Default launch (`headless:=2`, `show_rviz:=true`) starts the same nodes as before.
- Manual mode: arena.rviz (with the gap selection tool) is the only RViz in every `headless` mode.
- Recorded data / metrics: no change.
- Anything downstream: nothing must adapt.

## Test
`arena_bringup/test/test_single_rviz.py` runs `roslaunch --nodes` (no master needed) for `start_trials.launch` and `start_arena.launch` × `simulator` flatland/gazebo × `headless` 0/1/2 × `show_rviz` true/false. For every combination it asserts that roslaunch resolves the tree (return code 0), that the only RViz node is `/rviz`, and that `roslaunch --args /rviz` contains `dynamic_gap/rviz/arena.rviz`. It is skipped if `roslaunch` or the workspace is not available. It fails on the old launch files (8 of 24 cases: `headless` 0/1 with `show_rviz:=true`).

```
source /home/abdel/arena_ws/devel/setup.bash
cd src/arena/arena-rosnav/arena_bringup && python3 -m unittest discover -s test -v
```
