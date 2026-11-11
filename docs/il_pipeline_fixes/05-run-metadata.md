# 05 — params.yaml did not record the run settings

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/test/test_metrics_episode_results.py`
**Behaviour change:** None. `params.yaml` has six more keys.

## Problem
`Recorder.write_params()` wrote `model`, `map_file`, `scenario_file`, `inter_planner`, `local_planner`, `agent_name`, `namespace` and `time_unit`. Nothing said whether the run was manual (`/man_gap`), how often rows were written, whether sim time was used, or what timeout and goal tolerance the task generator applied.

## Why it mattered for data collection
Manual and autonomous runs could only be told apart by folder name. Metrics could not tell a timeout from a goal without knowing the timeout and tolerance (issue 03).

## What changed
`data_recorder_node`:
- `read_config()` runs before `write_params()` so the record interval is known.
- `params.yaml` gets:
  - `man_gap`: `/man_gap`
  - `use_sim_time`: `/use_sim_time`
  - `record_frequency`: from `data_recorder_config.yaml` (ms between rows)
  - `task_timeout`: `/task_generator_server/timeout` (s; negative means none)
  - `goal_radius`: `/task_generator_server/goal_radius`
  - `goal_tolerance_angle`: `/task_generator_server/goal_tolerance_angle`

  A param that is not set is written as `null`. Existing keys are unchanged.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: new folders carry the run settings. Metrics use `task_timeout`, `record_frequency`, `goal_radius` and `goal_tolerance_angle` (issue 03) and fall back to defaults for old folders.

## Test
`arena_evaluation/test/test_metrics_episode_results.py`, `RecorderParamsTest`: with the params set, `params.yaml` holds their values and the record interval from the config file; with none set, every new key is present and `null`. Both fail on the old recorder.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
