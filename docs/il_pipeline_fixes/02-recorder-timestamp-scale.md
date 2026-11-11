# 02 — Recorder timestamp scale

**Status:** Fixed · **Repo(s):** `src/arena/evaluation` · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/arena_evaluation/utils.py`, `arena_evaluation/arena_evaluation/metrics.py`, `arena_evaluation/arena_evaluation/metrics_static.py`, `arena_evaluation/test/test_recorder_timestamp_scale.py`
**Behaviour change:** The `time` column is now true ROS time in integer nanoseconds, and rows are written every `record_frequency` ms of real sim time.

## Problem
`data_recorder_node:215` (before the change) built the timestamp as `clock.clock.secs * 10e9 + clock.clock.nsecs`. `10e9` is 1e10, not 1e9. Seconds were scaled ten times more than nanoseconds, so the value was not a time in any unit. Example from `data/temp/rto/episode.csv`: 5.995 s was written as `50995000000.0`, and 6.0 s as `60000000000.0`.

The same value drove the record-interval gate (`time_diff = (now - last) / 1e6`, compared with `record_frequency: 400`). Inside one second the gate worked (400 ms). Across a second boundary the value jumped by 9e9, so a row was written right away. In `data/temp/rto` rows are written at 5.995 s and again at 6.0 s, 5 ms apart.

`metrics.py:240` and `metrics_static.py:240` decoded with `episode["time"] /= 10**10`. That gives `secs + nsecs / 1e10`, so the sub-second part was ten times too small.

## Why it mattered for data collection
- Episode durations (`time_diff`) were wrong by up to 0.9 s per episode. `data/temp/rto` episode 0 reported 3.9005 s; the real duration is 3.005 s. In `data/dgap_26-09-21_00-05-29/dgap/empty_scenario_3/rto` episode 1 reported 180.96 s; the real duration is 180.6 s.
- The timeout check (`TIMEOUT_TRESHOLD = 180`) and every time-weighted pedestrian metric (`np.diff(episode["time"])`) used these wrong values.
- Row spacing was not uniform: rows near each full second could be a few ms apart, so per-row data (velocity, jerk, sample counts) was uneven.

## What changed
- `data_recorder_node`, `Recorder._record()`: timestamp is `secs * 10**9 + nsecs`, an integer in nanoseconds. The gate (`/ 1e6` → ms) is now true time, so rows are `record_frequency` ms apart. `write_params()` adds `time_unit: ns` to `params.yaml`.
- `utils.py`: new `Utils.time_to_seconds(time, time_unit=None)`. With `time_unit == "ns"` it returns `time / 10**9`. With no `time_unit` it decodes the old encoding: `time // 1e10 + (time % 1e10) / 1e9`. Any other value raises `ValueError`.
- `metrics.py`, `metrics_static.py`: `Metrics` reads `time_unit` from the folder's `params.yaml` (`_get_time_unit()`), and `_analyze_episode` converts the `time` column with `Utils.time_to_seconds` instead of dividing by `10**10`.

New `params.yaml` key: `time_unit` (value `ns`). Absent in folders recorded before this change.

## Effects
- Autonomous mode: planner unchanged. Recorder rows are now evenly spaced at `record_frequency` ms.
- Manual mode: same as autonomous.
- Recorded data / metrics: new folders hold integer ns in every `time` column. Metrics give correct durations for both new folders and old folders. Checked on real old folders, before → after: `data/temp/rto` episode 0: 3.9005 s → 3.005 s; `data/dgap_26-09-21_00-05-29/dgap/empty_scenario_3/rto` episodes 0/1/2: 181.0/180.96/76.0 s → 181.0/180.6/76.0 s. The new values match an independent decode of the raw `episode.csv` files. `metrics.py` and `metrics_static.py` give identical results.
- Downstream: any external tool that reads the raw `time` column must check `time_unit` in `params.yaml` (or use `Utils.time_to_seconds`). Inside `arena_evaluation` the only reader of the raw column is `Metrics`. `scripts/create_plots` (`resize_time`, line 270) reads the `time` field of `metrics.csv`, which is already in seconds; it is not affected by this change. The repair script (issue 36) converts old folders to ns and writes `time_unit: ns`.
- Unrelated, not changed: `scripts/create_plots` `resize_time` divides the metrics `time` field (already in seconds) by 1e9, so plot time axes were wrong before this fix and still are.

## Test
`src/arena/evaluation/arena_evaluation/test/test_recorder_timestamp_scale.py`:
- Loads the recorder with the stubbed ROS modules from `test_recorder_duplicate_folders.py`, sends clock ticks at 5.995 s, 6.0 s and then every 200 ms, and asserts the written `episode.csv` times are exactly `6400000000`, `6800000000`, `7200000000` (integer ns, 400 ms apart, no row 5 ms after the first tick).
- Asserts `params.yaml` contains `time_unit: ns`.
- Builds a synthetic folder in ns with `time_unit: ns` and a legacy folder (old encoding, no `time_unit`) and asserts both `metrics.Metrics` and `metrics_static.Metrics` report a duration of 1.005 s.

All four tests fail on the code before this change and pass after.

Run:
```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
