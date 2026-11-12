# 54: Episode sets can cut the end of an episode (to_tick)

**Status:** Done 2026-11-11 · **Repo(s):** dynamic_gap
**Behaviour change:** None (dataset building only).

## Problem

An episode-set file (`il_build_dataset --set`) could only drop the start of an
episode (`from_tick`). In the first schema-5 pass (2026-11-11), part 3 episode 9
collided both right after the spawn (0.8-5 s) and near the goal (53.5 s): keeping
its 46 clean seconds needs the end cut as well. Moving the goal instead would
label the operator's choices with a goal they were not driving to.

## Change

`il_dataset.load_set` reads an optional `to_tick` per entry; selections are now
`{session dir: {episode: (from_tick, to_tick)}}`. `build_examples` drops label
ticks after `to_tick` (counted as `cut`). Observations precede their labels, so
they never pass it.

## Verification

New test `test_episode_set_cuts_label_ticks_after_to_tick`; the il_tools tests
(37) pass. Used by `~/il_data/sets/s5_pass1_cut.yaml`.
