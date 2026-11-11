# 45 — Ticks without planning recorded selected_idx = 0

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `src/Planner.cpp`, `include/dynamic_gap/utils/ManualSelection.h`, `test/test_manual_selection.cpp`
**Behaviour change:** None live. Recorded samples of ticks that end before selection now carry `selected_idx = -1` and selected model ids -1.

## Problem
`runPlanningLoop` built the tick's `ManualGapSample` with only `status` set. `selected_idx` and the selected model ids are set to -1 in `selectManualTrajectory`, which ticks that return early (not ready, colliding, no gaps) never reach, so they kept the message default 0.

Found in session `dgap_26-10-22_15-13-41_gru_learn_scenario_2`: the 11 status 2 (colliding) ticks 189–200 had no candidates but `selected_idx = 0`. A reader that takes `selected_idx >= 0` as "a gap was chosen" counts them as choices. `source` was `SOURCE_NONE`, so the usable-label count was not affected.

## What changed
- `newManualSample()` (ManualSelection.h) returns the sample a tick starts with: `STATUS_PLANNED`, `SOURCE_NONE`, `selected_idx = -1`, selected model ids -1.
- `runPlanningLoop` starts from it; every early return keeps "nothing selected".

## Effects
- Recorded data: early-return ticks read as no selection. Already recorded sessions are unchanged; for them, treat `selected_idx` as valid only when `status == STATUS_PLANNED`.

## Test
`test_manual_selection.cpp`: `ManualSelectionHelpers.NewSampleHasNoSelection`.
