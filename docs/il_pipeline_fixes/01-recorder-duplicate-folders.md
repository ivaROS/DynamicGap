# 01 — Recorder duplicates across folders

**Status:** Fixed · **Repo(s):** `src/arena/evaluation` · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/test/test_recorder_duplicate_folders.py`
**Behaviour change:** After a `change_directory` call, the previous folder receives no more rows.

## Problem
The benchmark module of the task generator calls the recorder service `/<robot>/change_directory` for every new scenario. The handler `reconfigure()` (`data_recorder_node:275-279` before the change) did `del recorder` and then built a new `Recorder`. `del` only removes the name. The old object stays alive through its own subscribers: `DataCollector.subscriber` (line 52), `Recorder.clock_sub` (line 171) and `Recorder.scenario_reset_sub` (line 173). rospy keeps calling their callbacks, so every old recorder kept writing every `/clock` tick into its old folder.

## Why it mattered for data collection
Each scenario folder held its own episode plus every later one. In `data/dgap_26-07-23_16-43-39/dgap/` the folder `gru_learn_scenario_3` holds episodes 1–11, `gru_learn_scenario_4` holds 2–11, and so on down to `gru_learn_scenario_13` with only 11. Metrics computed per folder therefore counted the same episodes many times, mixed scenarios, and grew file I/O with every directory change (N live recorders after N changes).

## What changed
`data_recorder_node`:
- `DataCollector.shutdown()` unregisters the collector's subscriber.
- `Recorder.shutdown()` takes a new per-recorder lock, sets `_active = False`, unregisters `clock_sub` and `scenario_reset_sub`, and shuts down every data collector.
- `clock_callback` and `scenario_reset_callback` take the same lock and return at once when the recorder is no longer active. This covers callbacks that were already queued or running when `shutdown()` was called: a running write finishes before `shutdown()` returns, and a later one writes nothing. The old body of `clock_callback` is now `_record()`, unchanged.
- New module function `replace_recorder(recorder, result_dir)` shuts down the old recorder and returns a new one. `reconfigure()` uses it under a lock, so two overlapping service calls cannot leave a recorder running.

No new params or topics.

## Effects
- Autonomous mode: no change in what is recorded for the current folder.
- Manual mode: same as autonomous.
- Recorded data / metrics: each folder now holds only the episodes recorded while it was the active folder. Old folders recorded before this fix still contain the duplicates (see issue 36 for repair).
- Downstream: nothing must adapt. File names and columns are unchanged.

## Test
`src/arena/evaluation/arena_evaluation/test/test_recorder_duplicate_folders.py` loads the node with stubbed ROS modules and a fake `rospy.Subscriber`. It asserts that:
- after a directory change, the old folder gets no new rows in `episode.csv`, `scan.csv` or `start_goal.csv`, while the new folder does;
- every subscriber of the old recorder is unregistered and the new recorder's subscribers stay registered;
- a `/clock` or `/scenario_reset` callback delivered to the old recorder after the change writes nothing and does not change its episode.

The test fails on the old code (old folder grows from 3 to 7 rows) and passes on the new code. The env var `DATA_RECORDER_NODE` can point the test at another copy of the script.

Run:
```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
