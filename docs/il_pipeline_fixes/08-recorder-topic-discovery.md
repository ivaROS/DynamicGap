# 08 — Startup topic discovery + stale rows

**Status:** Fixed · **Repo(s):** `src/arena/evaluation` · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/test/test_recorder_topic_discovery.py`, `arena_evaluation/test/test_recorder_duplicate_folders.py` (test stub only)
**Behaviour change:** Topic CSVs are always created and get a third column `msg_time`.

## Problem
`Recorder.__init__` (`data_recorder_node:142-166` before the change) called `rostopic.get_topic_list()` once and subscribed only to the published topics whose names matched the monitored list. A topic that was not advertised yet at that moment got no subscriber and no CSV file. Nothing retried later.

`/rto/cmd_vel` is published by `mbf_cmd_vel_relay`, a `topic_tools relay` that only advertises the topic after its first `cmd_vel_internal` message, and roslaunch does not order node start-up, so the recorder (`launch/testing/robot.launch:66`) usually starts before the topic exists. In the live run `smoke/fix22_auto` the planner published 796 `cmd_vel` messages, but the recorder log shows subscriptions only for `/clock`, `/pedsim_simulator/pedsim_agents_data`, `/rto/odom`, `/rto/scan` and `/scenario_reset`, and the folder `rec/rto` has no `cmd_vel.csv`. `data/temp/rto` shows the same.

`DataCollector.get_data()` (lines 114-118) returned the last held message on every 400 ms tick. A row gave no way to tell a fresh message from one repeated for many ticks.

## Why it mattered for data collection
- `cmd_vel.csv`, the action label for imitation learning, was missing depending on startup order. The same could happen to `scan`, `odom` or pedsim data if their publishers came up late.
- When a publisher stalled or stopped, the recorder kept writing its last message with a fresh row time. Downstream code could not detect or drop those stale rows.

## What changed
`data_recorder_node`:
- `get_topics_to_monitor()` now returns the explicit list `[topic, file name, class]`: `<ns>/scan` → `scan`, `<ns>/odom` → `odom`, `<ns>/cmd_vel` → `cmd_vel`, `/pedsim_simulator/pedsim_agents_data` → `pedsim_agents_data`. `<ns>` is `rospy.get_namespace()`, as before. The recorder subscribes to all four at startup, whether or not they are published yet. rospy connects when the publisher appears.
- The topic scan (`rostopic.get_topic_list`, the regex match, `get_class_for_topic_name`) is removed. The unused `re` and `rostopic` imports are removed. The never-matched `<ns>/scenario_reset` entry of the old list is dropped; `/scenario_reset` keeps its own subscriber.
- Each callback stores `(data, msg_time)` as one tuple via `DataCollector.hold()`. `msg_time` is the receive time from `rospy.get_rostime()` in ns (`secs * 10**9 + nsecs`), the same unit as the `time` column. Storing one tuple means a row never pairs new data with an old time.
- Topic CSV header is now `time,data,msg_time`. Rows are still written on every tick (sample and hold). Before the first message both `data` and `msg_time` are empty.
- File names are unchanged. `Recorder.shutdown()` from issue 01 still unregisters all four collectors.

No new params or topics.

## Effects
- Autonomous mode: no change in planning. All four topic CSVs now exist in every folder.
- Manual mode: same as autonomous.
- Recorded data / metrics: a row is stale when its `msg_time` is older than the previous row's `time` (the message was already written). `metrics.py` and `metrics_static.py` read the topic files by column name with pandas, so the extra column is ignored. Their `concat` keeps the first `msg_time` column (from `scan.csv`) and drops the others as duplicates; nothing in metrics uses it. No other script in `arena_evaluation` reads the raw topic files (`create_plots` reads `metrics.csv`; `process_data`, `get_metrics` go through `Metrics`).
- Downstream: code that wants per-topic freshness must read `msg_time` from each topic file directly. Old folders have no `msg_time` column.
- Known limit, unchanged by this fix: an empty `data` cell in `odom.csv` or `pedsim_agents_data.csv` (rows before the first message) makes the metrics converters fail (`json.loads("")`, `ast.literal_eval("")`). The old recorder wrote the same empty cells.

## Test
`src/arena/evaluation/arena_evaluation/test/test_recorder_topic_discovery.py` loads the node with the stubbed ROS modules from `test_recorder_duplicate_folders.py`. The stubbed topic list advertises only `scan` and `odom`. It asserts that:
- all four topics get a subscriber and a CSV file at startup;
- a `cmd_vel` message that arrives after startup is written, with header `time,data,msg_time`, an empty first row and the correct receive time;
- held rows repeat the data and keep the receive time of the held message, while a newer message updates it;
- `Recorder.shutdown()` unregisters every subscriber, including the four topic collectors;
- `Metrics` and `PedsimMetrics` in both `metrics.py` and `metrics_static.py` parse a folder written by the new recorder.

The shared stub in `test_recorder_duplicate_folders.py` gained `rospy.get_rostime`; the existing tests are otherwise unchanged. The new test fails on the old recorder (3 failures, 2 errors) and passes on the new one.

Run:
```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
