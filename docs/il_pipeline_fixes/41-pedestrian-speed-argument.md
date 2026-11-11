# 41 — Pedestrian speed could not be set at launch

**Status:** Fixed (argument) · open (pedsim wall-clock stepping, below) · **Repo(s):** arena-rosnav · **Files:** `arena_bringup/launch/start_trials.launch`, `task_generator/task_generator/tasks/modules/benchmark.py`, `task_generator/test/test_benchmark_ped_vmax.py`
**Behaviour change:** New optional argument `ped_vmax:=<m/s>`. Without it: no change.

## Problem
The task generator gives each pedestrian `vmax = Pedsim.VMAX(...)`, read once from `/task_generator_node/configuration/pedsim/default_actor_config/VMAX` with a fallback of 0.5. `start_trials.launch` never loads `arena_bringup/configs/pedsim.yaml` (`VMAX: [0.3, 0.5]` and the force factors), so every pedestrian got 0.5 and there was no way to change it from the command line. The scenario JSON files of 50ped_daniel set no per-pedestrian `vmax`.

## What changed
- `start_trials.launch`: `ped_vmax` sets that parameter (and `/ped_vmax`) for every pedestrian.
- `benchmark.py`: a relaunch passes `ped_vmax:=` on when the first launch had it (`_ped_vmax_args`), like `force_factor`.
- `~/run_il_pilot.sh`: `PED_VMAX=0.8 ~/run_il_pilot.sh` passes it.

## Open: pedestrians move in wall time, the robot in sim time
`pedsim_simulator` steps with a `ros::WallRate` and a fixed time step (`simulator.cpp`, `runSimulation`), so pedestrians advance in wall-clock time while flatland and the robot run on `/clock`. Session `dgap_26-10-22_13-09-00_gru_learn_scenario_2`: the simulation ran at about 0.37 × real time (58 s of sim time in 155 s wall), the pedestrians' twist says ~0.5 m/s, but their positions moved a median 1.28 m per sim second. Relative to the robot, pedestrians were about 2.6 times faster than `vmax`, and that factor changes with the simulation load. `ped_vmax` scales the twist speed, not this factor. A fix belongs in pedsim_ros (step on sim time).

## Test
`task_generator/test/test_benchmark_ped_vmax.py` (forwarded when set, nothing otherwise). `roslaunch --dump-params arena_bringup start_trials.launch ped_vmax:=0.8` shows `/task_generator_node/configuration/pedsim/default_actor_config/VMAX: 0.8`; without the argument the parameter is absent.
