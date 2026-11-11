# 22 — Reset begin signal

**Status:** Fixed · **Repo(s):** arena-rosnav · **Files:** `task_generator/task_generator/task_generator_node.py`, `task_generator/test/test_reset_begin_signal.py`, `task_generator/test/test_reset_lock.py`
**Behaviour change:** None for existing consumers. One new latched topic, `/scenario_reset_begin`.

## Problem
`TaskGenerator.reset_task()` published only one signal, `/scenario_reset`, and only at the end of the reset (`task_generator_node.py:319` before the change): after `before_reset_task()` (pause), after `self._task.reset(...)` (robot teleport, new goal, pedestrians respawned) and after `after_reset_task()` (resume). Nothing marked the start of a reset.

## Why it mattered for data collection
A recorder or a rosbag cannot tell where the old episode ends. Messages sent while the reset runs (the teleport jump in odom, a scan of the new scene, pedestrians removed and respawned, the planner reacting to a new goal) are still attributed to the old episode, because the new episode id only arrives with `/scenario_reset`. Without a start marker these frames cannot be cut out of the old episode.

## What changed
`task_generator_node.py`:
- New publisher `scenario_reset_begin` (`std_msgs/Int16`, `queue_size=1`, `latch=True`), created next to `scenario_reset` in the same `if not self._train_mode` block. It resolves to `/scenario_reset_begin` for the default node namespace `/`. Train mode still creates no publishers.
- First statement of `reset_task()` publishes `self._number_of_resets`. This is the counter before it is incremented, so the value equals the id that the matching `/scenario_reset` publishes at the end of the same reset. The first reset sends `begin 0` … `scenario_reset 0`.
- `/scenario_reset` is unchanged (same point, same value, still latched).

Why latched: `/scenario_reset` is latched so that late subscribers learn the current episode, and the first reset runs in the constructor right after the publishers are created, before any subscriber can connect. A non-latched begin for episode 0 would be lost for everyone. Latching also lets `rosbag record --repeat-latched` repeat it at each bag split. A latched begin seen alone could look like a reset that never ended, so consumers must compare ids, not react to the arrival order of the two topics (on connect, the two latched messages can arrive in any order):
- last `scenario_reset_begin` id > last `/scenario_reset` id (or no `/scenario_reset` yet) → a reset is in progress;
- ids equal → the reset with that id has finished and the episode is running.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: nothing records the topic yet. The recorder and metrics are unchanged. Frames between `begin N` and `scenario_reset N` belong to neither episode and can be dropped by downstream tools.
- Downstream: the IL rosbag launch (issue 07) records `/scenario_reset_begin`. New consumers must use the id comparison above.

## Follow-up: one reset at a time

`reset_task()` had no lock: a `/reset_task` service call during a timer-driven reset published the same begin id twice. `TaskGenerator` now holds `_reset_lock` (a `threading.RLock`) for the whole of `reset_task()`. `_check_task_status` (the 0.5 s status timer) takes the lock without waiting and skips its tick if a reset is running, so it cannot start a second reset right after the first. A service call during a reset waits and then runs its own reset with the next id. Test: `task_generator/test/test_reset_lock.py` (the lock is held during the reset; a status tick during a reset is skipped; two concurrent resets publish ids 0 and 1).

## Test
`src/arena/arena-rosnav/task_generator/test/test_reset_begin_signal.py` loads `task_generator_node.py` with stubbed ROS and `task_generator` imports (it reuses the loader of `test_double_resume.py`). It asserts that:
- `reset_task()` publishes `scenario_reset_begin` once, as its first action, before `before_reset_task()` and before `/scenario_reset`;
- over three resets the begin id and the `/scenario_reset` id are equal (0, 1, 2);
- the constructor creates `scenario_reset_begin` as a latched `Int16` publisher outside train mode, and creates no publishers in train mode.

All checks except the train-mode one fail on the old code (no such publisher or publish call).

```
cd src/arena/arena-rosnav/task_generator && python3 -m unittest discover -s test -v
```
