# 47 — Datasets from chosen episodes (episode sets)

**Status:** Fixed · **Repo(s):** dynamic_gap · **Files:** `il_tools/il_dataset.py`, `il_tools/il_build_dataset`, `il_tools/test/test_il_tools.py`
**Behaviour change:** None live. New option `il_build_dataset --set FILE`.

## Problem
`il_build_dataset` took whole session folders. A session holds several episodes, some with collisions, so a hand-picked set (for example one clean episode per corner) could not be built without the other episodes of the same sessions.

## What changed
- `il_dataset.load_set(path)` reads an episode set file: `root` (folder of the sessions) and `episodes`, a list or groups of lists of `{session, episode}`; other fields are notes.
- `build_dataset(..., selection=)` / `--set FILE` uses only the listed episodes (count `not_in_set` for the rest). Listed episodes are kept even if they collided: the set is an explicit choice. Their colliding ticks are never usable labels, and every example keeps `collision_episode` to filter on.

Example: `il_data/sets/4_corners_data_1.yaml` (2026-10-23, gru_learn, v2/v4/v6/v8 corners, 7 episodes, 1975 examples).

## Test
`test_il_tools.py`: `DatasetTest.test_episode_set_selects_and_keeps_collisions`.
