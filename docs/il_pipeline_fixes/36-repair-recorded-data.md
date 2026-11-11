# 36 — Repair script for already-recorded folders

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/scripts/repair_recorded_data` (new), `arena_evaluation/test/test_repair_recorded_data.py`
**Behaviour change:** None (offline script; original folders are never modified).

## Problem
Folders recorded before fixes 01 and 02 have two faults:
- Duplicate episodes (issue 01): after `change_directory`, the old recorder kept writing every later episode into its old folder. In `data/dgap_26-07-23_16-43-39/dgap`, `gru_learn_scenario_2/rto` holds episodes 0–11, `gru_learn_scenario_3` holds 1–11, ..., `gru_learn_scenario_13` holds 11.
- Legacy time (issue 02): time was stored as `secs * 1e10 + nsecs`.

Metrics now decode the legacy time (issue 02) but cannot know which episodes in a folder are duplicates.

## What changed
`repair_recorded_data RUN_DIR [--dry-run] [--out DIR]` writes a repaired copy to `RUN_DIR_repaired` (refuses if it exists) and prints a YAML report per recording folder.

- Recording folders are the folders under `RUN_DIR` that contain `episode.csv` (at any depth, e.g. `scenario/rto/`). They are grouped into runs per robot `namespace` and per launch folder (`<planner>_<date>_<time>`, which the recorder's `auto:` prefix creates per launch; `RUN_DIR` itself if there is none): a later folder joins an earlier one when at least 5 and at least half of its rows appear in the earlier folder with the same time and the same odometry (the old and the new recorder wrote the same clock ticks and held the same messages; on `dgap_26-07-23_16-43-39` 84 of 85 row times and 99 % of shared odometry rows match). Sim time restarts at every launch, so times of different launches collide and tiny folders can share a row of the same start pose; folders of different launches are therefore never grouped.
- Within a run, folders are ordered by the time of their first row (decoded). Each keeps only the episode ids that are not in the next folder; the last keeps all. This is the ownership the fixed recorder would have produced.
- In each kept folder, rows of every `*.csv` with a `time` column are kept if their time belongs to a kept episode in `episode.csv`; `start_goal.csv` is filtered by its `episode` column (works for per-tick and per-episode files).
- Folders without `time_unit` in `params.yaml` get `time` and `msg_time` rewritten as nanoseconds (`t // 1e10 * 1e9 + t % 1e10`) and `time_unit: ns` added.
- `--dry-run` only prints the plan (episodes, keep, drop per folder).

Dry runs: on `data/dgap_26-07-23_16-43-39`, 12 recording folders in one run, all legacy time; each keeps exactly one episode (scenario_2 → 0, scenario_3 → 1, ..., scenario_13 → 11), 12 of 78 episode entries. On `data/attempt1` (14 short launches of one scenario), each launch keeps its own episode 0. On `data/ankit` (two runs in one parent): `dgap_26-06-19_10-12-07/.../empty_scenario_0` keeps 0–2 and drops 3–4, `empty_scenario_1` keeps 3–4, and the other run's `gru_learn_scenario_2` keeps its episode 0.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: metrics on a repaired copy give one result per real episode, with correct times.

## Test
`arena_evaluation/test/test_repair_recorded_data.py` builds a run with three scenario folders (one nested under `rto/`) in the old layout, each a copy of one recorder's timeline from its creation on: the plan keeps each folder's own episode; two independent runs with restarted ids and times are not merged; tiny runs of different launches that share a start-pose row are not merged; fewer than 5 shared rows do not group; runs are told apart by shared rows; `--dry-run` writes nothing; the copy has only the kept episodes in every file, nanosecond times and `time_unit: ns`, and the original is unchanged; new-format folders are not converted; an existing output is refused; `Metrics` reads a repaired folder with the right episode and duration.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
python3 scripts/repair_recorded_data --dry-run data/dgap_26-07-23_16-43-39
```
