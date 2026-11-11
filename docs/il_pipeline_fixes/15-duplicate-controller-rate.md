# 15 — Duplicate controller rate

**Status:** Fixed · **Repo(s):** `src/arena/simulation-setup` · **Files:** `configs/mbf/local/dgap_local_planner_params.yaml`, `test/test_duplicate_controller_rate.py`
**Behaviour change:** None. All effective parameters are unchanged.

## Problem
Two files set the move_base_flex rates for dgap:

- `configs/mbf/move_base_params.yaml:10-11`: `controller_frequency: 10.0`, `controller_patience: 5.0`. Line 6 also sets `planner_frequency: 5.0`.
- `configs/mbf/local/dgap_local_planner_params.yaml:5-7` (before the change): `controller_frequency: 5.0`, `controller_patience: 15.0`, `planner_frequency: 0.0`.

`start_trials.launch` does not start move_base_flex itself. The task generator's `robot_manager.py` (`_launch_robot`, ~line 241) launches `arena_bringup/launch/testing/robot.launch` with `namespace:=/rto`. That file includes `simulation-setup/launch/mbf/mbf_nav/costmap_nav.launch` inside the `/rto` group. The load order in `costmap_nav.launch` is:

1. Line 15: include `mbf_dgap.launch`. It loads the dgap file and then `entities/robots/rto/configs/mbf/dgap_local_planner_params.yaml` into `move_base_flex` (lines 47-51) and `move_base_legacy_relay` (lines 58-62).
2. Line 44: the `move_base_flex` node loads `configs/mbf/move_base_params.yaml` as private params, then `entities/robots/rto/configs/mbf/move_base_params.yaml` (line 61). Lines 96-99 do the same for `move_base_legacy_relay`.

Later loads win, so the three keys in the dgap file never took effect. `roslaunch --dump-params arena_bringup robot.launch local_planner:=dgap model:=rto name:=rto namespace:=/rto frame:=rto/ inter_planner:=bypass` gives:

```
/rto/move_base_flex/controller_frequency: 10.0
/rto/move_base_flex/controller_patience: 5.0
/rto/move_base_flex/planner_frequency: 5.0
```

The same values appear under `/rto/move_base_legacy_relay/`. The rto dgap file has no key in common with either `move_base_params.yaml`.

Dumping `start_trials.launch` alone does not show these params, because the robot is launched later by the task generator.

## Why it mattered for data collection
The dgap file suggests a 5 Hz controller, a 15 s patience and no global replanning. The planner really ran at 10 Hz with a 5 s patience and replanned the global path at 5 Hz. Anyone reading the config to learn the control rate of recorded data would get the wrong number. Live runs agree with 10 Hz: `/rto/cmd_vel` measured 8.6-9.1 Hz over four 90 s runs. The rate is a little under 10 Hz, probably because some control cycles take longer than 100 ms or because of the sim real-time factor. This was not measured separately.

## What changed
`configs/mbf/local/dgap_local_planner_params.yaml`: removed `controller_frequency`, `controller_patience` and `planner_frequency`. A comment now says these come from `configs/mbf/move_base_params.yaml`, which is loaded after this file.

`configs/mbf/move_base_params.yaml` is unchanged. It is shared by every local planner, so it keeps its values.

No new params or topics.

## Effects
- Autonomous mode: none. The rates are the same: controller 10 Hz, patience 5 s, global planner 5 Hz.
- Manual mode: none.
- Recorded data / metrics: none. The effective dgap controller rate is 10 Hz (`/rto/cmd_vel` about 9 Hz measured).
- Downstream: to change the dgap rate, edit `configs/mbf/move_base_params.yaml`, which changes it for every planner. Setting it in the dgap file has no effect unless the load order in `costmap_nav.launch` changes. The full `--dump-params` output of `robot.launch` for dgap, teb, dwa, mpc and pgap, and of `start_trials.launch` for dgap, teb and dwa, is the same before and after the change.

## Test
`src/arena/simulation-setup/test/test_duplicate_controller_rate.py` (stdlib `unittest`, no ROS master needed).
- `TestSingleSourceInFiles`: of the four files loaded for dgap (generic and rto `move_base_params.yaml`, generic and rto `dgap_local_planner_params.yaml`), only `configs/mbf/move_base_params.yaml` defines `controller_frequency`, `controller_patience` and `planner_frequency`, with values 10.0, 5.0 and 5.0. This fails on the old dgap file.
- `TestEffectiveParams`: runs `roslaunch --dump-params arena_bringup robot.launch` with the task generator's arguments for dgap, teb and dwa. It checks that the three values are 10.0, 5.0 and 5.0 under `/rto/move_base_flex/` and `/rto/move_base_legacy_relay/`. Skipped if `roslaunch` is not on `PATH`.

Run:
```
source /home/abdel/arena_ws/devel/setup.bash
cd src/arena/simulation-setup && python3 -m unittest discover -s test -v
```
