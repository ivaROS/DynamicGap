# 29 — Cursor stored a map point; drifted with the camera and held failed picks

**Status:** Fixed · **Repo(s):** arena-rosnav, dynamic_gap · **Files:** arena-rosnav `utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.{h,cpp}`, `utils/plugins/rviz_plugins/src/cursor_state.h`, `utils/plugins/rviz_plugins/{CMakeLists.txt,package.xml,plugin_description.xml}`, `utils/ros/msgs/manual_gap_msgs/` (new); dynamic_gap `include/dynamic_gap/Planner.h`, `src/Planner.cpp`, `test/test_manual_selection.cpp`
**Behaviour change:** Manual mode: the cursor is the ground point under the mouse at every publish; a deactivated tool removes the cursor (the robot holds still, issue 16).

## Problem
- The tool stored the 3D point from `SelectionManager::get3DPoint` on each mouse move (`continuous_gap_selection_tool.cpp:83-91` before the change) and re-sent it. `get3DPoint` hits rendered geometry, so it fails over empty background and the old point was kept. The shipped view follows `rto/base_link`, so as the robot drove, the scene scrolled under a still mouse while the published point stayed fixed in the map.
- Nothing told the planner the tool was switched off; the planner used the last point forever (`hasCursorPos_` was never cleared).
- The planner always applied `map2odom_` to the cursor (`Planner.cpp:721`), whatever its frame.

## Why it mattered for data collection
The recorded cursor did not show what the operator pointed at on screen, failed picks looked like confident holds, and a tool switched off still steered the robot.

## What changed
New message package `manual_gap_msgs` (arena-rosnav `utils/ros/msgs/manual_gap_msgs`) with `CursorState`, `ManualGapCandidate` and `ManualGapSample`, so offline tools can read the data without building the planner.

Tool:
- Stores the panel, viewport and screen position of the last mouse event. Every publish (issue 11) intersects the mouse ray with the ground plane z = 0 (`rviz::getPointOnPlaneFromWindowXY`) and publishes `CursorState` on `<prefix>gap_selection_cursor` (issue 14) with `tool_active = true` and `pick_ok` = the ray hits the ground and the pointer is inside the panel (checked with `QCursor::pos()`, since no event arrives once it has left). The point is only set when `pick_ok`.
- `deactivate()` publishes one message with `tool_active = false`.
- Click-to-move uses the same ground pick. The status text now reads "Hover: select the gap nearest the cursor. Left-Click: drive to this point." (issue 18b).

Planner (`cursorCB`, replaces `rvizPublishPointSubsciberCB` and `/gap_selection_point`):
- Stores the message and its wall-clock receipt time.
- `tool_active = false` → no cursor (`hasCursorPos_ = false`).
- `pick_ok = false` → the last point is kept (a still mouse keeps its selection).
- Otherwise the point is transformed from the message's own `frame_id` to the odom frame through `tfBuffer_`, at the message stamp, else at the latest transform; if neither exists the last good point is kept, a throttled warning is logged, and the sample marks the cursor as not live until a transform succeeds.
- If the cursor stamp and planner time differ by more than 2 s, a throttled error says RViz is probably not on sim time (DG-12).
- The cursor is usable for selection when a point exists, the tool is active, however old the last message is (`isCursorUsable`, `isCursorHeld`; issue 40 removed a 0.5 s timeout). The tool publishes every 0.1 s whatever the mouse does, so this never requires mouse movement.

## Effects
- Autonomous mode: no change.
- Manual mode: the cursor follows the screen under a still mouse while the camera follows the robot; switching tools stops the robot (issue 16); an RViz that stops publishing keeps the last selection (issue 40).
- Recorded data / metrics: the sample (issue A/B doc) carries the cursor message verbatim, its receipt time, the odom point used, `cursor_valid` and `cursor_live` (usable and `pick_ok`).
- Downstream: anything publishing `/gap_selection_point` (`geometry_msgs/PointStamped`) must publish `manual_gap_msgs/CursorState` on `/rto/gap_selection_cursor` instead.

## Test
- `utils/plugins/rviz_plugins/test/test_cursor_state.cpp`: topic prefix joining; a failed pick has no point; an inactive tool is never `pick_ok`.
- `dynamic_gap/test/test_manual_selection.cpp`: a cursor in `map` is transformed with the map→odom transform; a failed pick keeps the last point; `tool_active = false` removes the cursor; an unknown frame is ignored; `CursorFreshnessIgnoresMouseMovement`.
- The ray–plane pick needs a running RViz and is not unit tested.

```
cd $OV && catkin build --no-status dynamic_gap rviz_plugins --catkin-make-args run_tests && catkin_test_results build/dynamic_gap && catkin_test_results build/rviz_plugins
```
