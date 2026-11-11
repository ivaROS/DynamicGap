# 40 — Cursor dropped after 0.5 s without a cursor message

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`, `include/dynamic_gap/Planner.h`, `include/dynamic_gap/utils/ManualSelection.h`, `test/test_manual_selection.cpp`, `test/planner_test_access.h`
**Behaviour change:** Manual mode keeps selecting the gap nearest the last known cursor position however long ago the last cursor message arrived. Only switching the tool off (or never using it) ends the selection.

## Problem
`isCursorUsable` also required the last `CursorState` to be less than 0.5 s old in wall time (`cursorTimeout_`, `isCursorFresh`). Whenever RViz did not publish for 0.5 s (render stall, crash, frozen window), the planner treated the cursor as gone and the robot held still, although the operator had not changed the selection.

Operator report, 2026-10-22: "MGS felt like the bot didn't want to follow my direction", with the request that the last known cursor position stays the valid cursor position.

## What changed
- `isCursorUsable` = a point exists, a message exists, and the last message has `tool_active` (`isCursorHeld`). No age check; `cursorTimeout_` is removed.
- The point itself is unchanged: a failed pick or failed transform keeps the last good point in the odom frame (issue 29), so with RViz gone the robot keeps heading for the gap nearest that fixed point.
- Deactivating the tool still publishes `tool_active = false`, which stops the selection as before.

## Effects
- Live: a stalled or crashed RViz no longer stops the robot; it keeps following the last selection.
- Recorded data: those ticks now have `cursor_valid` and `cursor_live` true and count as usable. `cursor_rx_wall` is still recorded, so ticks whose cursor message is older than 0.5 s can be found offline (`il_bags.CURSOR_FRESH_S`).

## Test
`test_manual_selection.cpp`: `CursorHeldWithoutNewMessages` (only the active flag counts) and `ManualPlannerTest.OldCursorStillSelects` (a cursor received 60 s ago still selects its gap, `trajFlag == GAP`, live label). All 38 tests pass.
