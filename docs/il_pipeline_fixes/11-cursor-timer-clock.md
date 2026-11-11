# 11 — Cursor timer ran on sim time and republished the last-move stamp

**Status:** Fixed · **Repo(s):** arena-rosnav · **Files:** `utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.{h,cpp}`, `utils/plugins/rviz_plugins/src/cursor_state.h`, `utils/plugins/rviz_plugins/test/test_cursor_state.cpp`
**Behaviour change:** The tool publishes at 10 Hz of wall-clock time; each message carries its publish time.

## Problem
`ContinuousGapSelectionTool` published from a `ros::Timer` (`continuous_gap_selection_tool.cpp:29` before the change). With `use_sim_time` that timer runs on the simulation clock: it stops when the simulation is paused and speeds up or slows down with it. Each publish re-sent `lastKnownPos_` unchanged, so its stamp was the sim time of the last mouse move, not of the publish.

## Why it mattered for data collection
A paused or slow simulation starved the planner of cursor messages, so any receipt-age check would flap. Consumers that read the stamp as "time of publish" were wrong by however long the mouse had been still.

## What changed
- The timer is a `ros::WallTimer` (0.1 s), created stopped, started in `activate()`, stopped in `deactivate()`.
- Every publish builds a new `manual_gap_msgs/CursorState` (issue 29): `header.stamp` = `ros::Time::now()` (sim time, the planner's clock) at publish, `wall_stamp` = wall clock at publish, `last_move_wall` = wall clock of the last mouse move.
- The tool keeps publishing while active whether or not the mouse moves.

## Effects
- Autonomous mode: no change (the tool is only used in manual mode).
- Manual mode: the planner receives the cursor at a steady 10 Hz even when the simulation is paused or not real time.
- Recorded data / metrics: the cursor stamp is its publish time; the time of the last mouse move is a separate field.

## Test
`utils/plugins/rviz_plugins/test/test_cursor_state.cpp` checks that `makeCursorState` stamps the message with the given sim and wall times and the separate last-move time. The timer itself needs a running RViz and is not unit tested.

```
cd $OV && catkin build --no-status dynamic_gap rviz_plugins --catkin-make-args run_tests && catkin_test_results build/dynamic_gap && catkin_test_results build/rviz_plugins
```
