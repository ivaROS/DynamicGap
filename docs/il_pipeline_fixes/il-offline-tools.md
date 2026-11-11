# IL offline tools — bag checks, dataset build, replay viewer

**Status:** Done · **Repo(s):** dynamic_gap · **Files:** `il_tools/il_bags.py`, `il_tools/il_dataset.py`, `il_tools/il_check_session`, `il_tools/il_build_dataset`, `il_tools/il_replay_viewer`, `il_tools/test/test_il_tools.py`
**Behaviour change:** None (offline tools).

Plan sections 1.6 (automatic checks), 1.7 (offline processing) and R-38 (looking at the labels). They read the session folders written by `record_il.launch` (issue 07) and need only `rosbag` and the generated `manual_gap_msgs` Python messages.

## il_check_session
```
il_tools/il_check_session SESSION_DIR [...] [--namespace rto] [--yaml results.yaml]
```
Runs the checks of plan 1.6 and exits with 1 if one fails:
- bags present, no `.bag.active` files;
- `tick_seq` contiguous (a planner restart, where it starts again at 0, is reported but not a failure);
- the planner's selection rule, replayed from the recorded cursor, robot pose and candidates, gives `selected_idx` (`il_bags.selection_mismatches`). Sessions before issue 43 (no `man_gap_max_bearing_deg` / `man_gap_switch_ratio` in `params.yaml`): every `SOURCE_GAP_CURSOR` tick selects the nearest feasible terminal pose. From issue 43 on: direction cone, nearest, then the sticky switch against the gap followed before, which is replayed tick by tick (set by each gap-cursor selection; cleared by click-to-move, by a selection with no gap in the cursor's direction, and by an episode change). A gap with a gap point without a model (id -1) has no identity, as in the planner (`hasModelIDs`): it is never the followed gap. Checked are gap-cursor ticks and `PLANNED` ticks with `cursor_valid` but no selection, outside reset windows; after a hole in `tick_seq` the followed gap is unknown and the next selection is not checked. The robot pose is recorded at publish, not at selection, so a choice within 2° of the cone edge or 0.02 of the switch ratio counts as reproduced;
- one clock: on ticks whose cursor message is fresh (received less than 0.5 s of wall time before the tick; the tool may have been switched off), the cursor stamp is within 5 s of the sample stamp; and stamps do not go backwards within an episode;
- `episodes.csv` is present, and every episode in it has samples and an outcome (the last episode may still be running).

It prints a report in decisions rather than ticks (plan 1.7): ticks, usable label ticks (`source == GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0`), decisions (changes of the selected gap identity between usable ticks with mouse movement in between), usable ticks with more than one feasible candidate, click-to-move and hold-still ticks, and episode results.

Episode outcome: `GOAL_REACHED` if the goal (robot frame) at the episode's last tick is within the goal radius plus the distance covered in one 0.5 s goal check, `TIMEOUT` if the episode lasted the task timeout (within 1 s), else `ABORTED`. The duration is taken from `episodes.csv` (the episode's `reset` to the next `begin`, sim time), because the planner can stop ticking before the episode ends; the samples are used only for an episode still running when recording stopped; `collision` if any scan range came within the robot radius + 1 cm. The timeout comes from the episode's `reset` row in `episodes.csv` (IL fix 51: stages on one map share a session but not their timeout), else from `params.yaml` (`task_generator_server`); the goal radius from `params.yaml`: the global `goal_radius`, which the robot manager reads at startup, else (unset in every launch) the task generator's, its fallback.

## il_build_dataset
```
il_tools/il_build_dataset --out DIR [--lag S] [--max-cursor-dist M] [--max-bearing-diff RAD]
                          [--min-tie-margin M] [--keep-collision-episodes] [--seed S] SESSION_DIR [...]
il_tools/il_build_dataset --out DIR --set SET_FILE [options]
```
An episode set file (`il_data/sets/*.yaml`: `root`, then `episodes` grouped under any keys,
each `{session, episode}`) uses only those episodes, collision episodes included. An entry's
optional `from_tick: N` drops that episode's ticks before `tick_seq` N, as labels and as
observations (counted as `cut`), e.g. a collision right after the spawn; the gap followed
before is then unknown for the first kept observation.
Writes `train/val/test.jsonl.gz` (one example per line) and `dataset.yaml` (options, per-session counts of every drop reason — threshold, collision episode, no/too old/invalid observation tick, identity lost, duplicate — and reports). For each usable label tick t:
- thresholds (from the pilot): cursor distance and bearing difference to the selected candidate, and the margin to the second-nearest candidate;
- reaction lag (DG-09): the observation is the latest tick of the same episode at least `--lag` seconds of wall time before t; it is dropped if it is more than `max_lag_error_s` (0.25 s) older than t − lag (a hole in the samples), or if it is not a planned tick or lies in a reset window; the label is the index, in that tick's candidates, of the gap with the same identity (left and right model ids) as the one selected at t; if the gap does not exist there, or has no identity (a model id -1), the example is dropped; with `--lag 0` the observation is the label tick and the label is its recorded `selected_idx`; two label ticks that give the same observation tick and label produce one example;
- collision episodes are dropped (or kept and tagged with `--keep-collision-episodes`);
- splits are by episode, deterministic from a hash of session, episode and seed (default 80/10/10); never by tick.

An example holds the scan, robot velocity and pose, goal and local waypoint (robot frame), every candidate's geometry, estimator states, gap goal, terminal pose and path, and the label.

## il_replay_viewer
```
rosbag play --clock SESSION_DIR/session_*.bag
il_tools/il_replay_viewer _namespace:=rto
```
Publishes `/il_replay/markers`: candidate paths (grey), the selected one (green if the tick is a usable label, orange otherwise), the cursor point (blue if live), and a line with episode, tick, source and flags.

## Test
`il_tools/test/test_il_tools.py` writes synthetic sessions with rosbag: a good session passes every check; split bags are read in order; a missing tick, a wrong label, a wall-clock cursor stamp, a `.bag.active` file, a missing `episodes.csv` and an episode without samples are each detected; a switched-off tool is not a clock error; the outcome duration comes from `episodes.csv`; collisions are flagged per episode. The dataset tests check labels, pairing by identity under a lag when the candidate order changes, dropping of lost identities, unusable ticks, thresholds, collision episodes, reset-window and non-planned observations, stale observations after a hole, duplicates, and that splits are deterministic per episode.

```
source <workspace>/devel/setup.bash
cd src/planners/dynamic_gap/il_tools && python3 -m unittest discover -s test -v
```
