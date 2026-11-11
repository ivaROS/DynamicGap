# 09 — One start_goal row per episode

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/arena_evaluation/metrics.py`, `arena_evaluation/arena_evaluation/metrics_static.py`, `arena_evaluation/test/test_start_goal_per_episode.py`, `arena_evaluation/test/test_recorder_duplicate_folders.py`
**Behaviour change:** `start_goal.csv` has one row per episode instead of one per tick. Metrics read both layouts.

## Problem
`Recorder.clock_callback` wrote a `start_goal` row on every recorded tick, next to the `episode` row. The values come from the `<ns>start` and `<ns>goal` params, which only change at an episode reset, so almost every row repeated the previous one.

## Why it mattered for data collection
`start_goal.csv` grew at the tick rate for data that changes once per episode. Its rows were also only correct if they lined up exactly with the other per-tick files, because metrics joined them by row position (`pd.concat(..., axis=1, join="inner")`).

## What changed
`data_recorder_node`:
- New `write_start_goal()` writes `[episode, start, goal]` and sets `start_goal_written`.
- `Recorder.scenario_reset_callback` calls it after it stores the new episode id. The task generator sets `start` and `goal` in `RobotManager.reset` before it publishes `/scenario_reset`, so the row holds the new episode's values.
- `Recorder._record` (run by `clock_callback`) calls it once, on the first recorded tick, for the case where no `/scenario_reset` has arrived yet. A later `/scenario_reset` for the same episode writes a newer row.

`metrics.py`, `metrics_static.py`:
- `start_goal.csv` is loaded on its own (`_load_start_goal`).
- `_is_start_goal_per_tick` tells the layouts apart: more than two rows per episode on average means the old per-tick file. Old files are joined by row as before.
- New files are merged by episode id (`_merge_start_goal`); the last row of an episode wins, and an episode with no row gets `[0, 0, 0]`, the recorder's default.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: an episode has one or two `start_goal` rows. Metrics give the same start and goal per episode as before for both old and new folders. An old folder whose episodes average two ticks or fewer would be read as new; such episodes are shorter than `MIN_EPISODE_LENGTH` and dropped anyway.

## Test
`arena_evaluation/test/test_start_goal_per_episode.py` loads `data_recorder_node` with stubbed `rospy` and checks that ticks within an episode write no extra rows, that a reset writes one row with the new params, and that the first tick writes a row when no reset arrived. It also checks the metrics merge for new files and the row join for old per-tick files. `test_recorder_duplicate_folders.py` was updated for the new row count.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
