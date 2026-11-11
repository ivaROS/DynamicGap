# 39 — Benchmark stages ran the default scenario

**Status:** Fixed · **Repo(s):** arena-rosnav · **Files:** `task_generator/scripts/filewatcher.py`, `task_generator/test/test_filewatcher_config_file.py`
**Behaviour change:** With `tm_modules:=benchmark`, each stage now runs its own scenario and seed. Other launches: no change.

## Problem
The benchmark module applies a stage by writing its task config (scenario file, random seed) into a YAML file; `filewatcher.py` loads that file into `task_generator_server`, from which the scenario tasks read `SCENARIO_file`. Commit `85d2dd514` (2026-06-16, two simulations in parallel) made the benchmark write `task_generator_<port>.yaml` (and `resume_<port>.lock`) so that two masters do not overwrite each other, but the filewatcher kept reading `task_generator.yaml`. Every benchmark stage since then ran whatever `task_generator.yaml` held (`SCENARIO.file: 0.json`).

Seen on 2026-10-22: stage `gru_learn_scenario_2` of 50ped_daniel (`0_50ped_middle_start_v2.json`: 50 pedestrians, start (15, 12), goal (28, 5)) recorded 15 pedestrians, start (1.5, 2), goal (30, 2) — `0.json`. The session's `params.yaml` has `task_generator_server/SCENARIO_file: 0.json`.

## Why it mattered for data collection
Every stage of a suite was the same scenario, so a "suite" gave no variety, and the stage names in the recorded folders (IL sessions, `arena_evaluation` data) do not describe what ran. This likely affects every benchmark-suite run since 2026-06-16, unless `task_generator.yaml` was edited by hand to match.

## What changed
`task_config_name(tm_modules, ROS_MASTER_URI)` picks the file: with the `benchmark` module, `task_generator_<port>.yaml` with the port taken from `ROS_MASTER_URI` exactly as `benchmark.py` does; otherwise `task_generator.yaml`. Launches without the benchmark keep the shared file, so they do not pick up stage settings left in a per-port file. The filewatcher logs the file it reads.

## Effects
- Without the benchmark module: no change.
- With it: each stage runs its scenario. Per-port files exist for 11311 and 11312 only; the benchmark already needed them.
- Stages on the same map are not relaunched, so one IL session folder (issue 38) covers every stage until the map changes.

## Test
`task_generator/test/test_filewatcher_config_file.py`: per-port name for the benchmark (same rule as `benchmark.py`), shared file otherwise.

End to end (2026-10-22, master 11312, no display): after the benchmark relaunch the filewatcher read `task_generator_11312.yaml`, `task_generator_server/SCENARIO_file` was `0_50ped_middle_start_v2.json`, and the simulation had 50 pedestrians, the robot at (15.00, 12.00) and the goal at (28.0, 5.0).

```
cd src/arena/arena-rosnav/task_generator && python3 -m unittest discover -s test -v
```
