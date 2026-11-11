# IL collection protocol (template)

Plan section 1.8 of `IL_DATA_COLLECTION_PLAN.md`. Fill in every **TODO**
before the first real session; the tooling below already exists. The
current system is described in `COMPLETE_OVERVIEW_IL_DOC.md`.

## 1. Decisions to fill in

| Item | Value |
|---|---|
| Instruction given to operators (what to optimise: reach the goal quickly? safely? like a person would?) | **TODO** |
| Operators, and their warm-up (e.g. 5 min unrecorded on `map_empty`) | **TODO** |
| Session length and breaks | **TODO** (suggest ≤ 20 min, then a break) |
| Maps, scenarios and pedestrian densities to cover (and `ped_vmax`) | Designed dataset: benchmark suite `il_dataset_small_v1.yaml` on `gru_learn` (CS1–CS6, smartgap, 0_50ped_middle_start variants; held-out variants listed in the suite). Sessions up to 2026-10-26 (pilot, `50ped_daniel.yaml`) are preliminary |
| Target size, in decisions (not ticks) per map/density | **TODO** |
| RViz config (`view.rviz` is saved with each session; plan R-11, PL-26) | **TODO**: shipped `arena.rviz`, or a cut-down view matching the model inputs (D5). Either way decide whether the candidate-number display (`/rto/manual_candidate_markers`) is on; it is off in `arena.rviz` today. |
| `data_root` (outside every git repo, D7) | **TODO** (pilot: `/home/abdel/il_data`) |
| Selection parameters `man_gap_max_bearing_deg`, `man_gap_switch_ratio` | **TODO** (defaults 60°, 0.7; check in the pilot) |
| Thresholds and reaction lag (from the pilot, plan Phase 2) | **TODO** |

Settings in effect until changed:
- D1: the cursor is what is under the mouse on screen, re-projected every
  publish (fix 29).
- D2: direction cone plus sticky switch (fix 43): only gaps whose trajectory
  ends within ±60° of the robot-to-cursor bearing; the followed gap is kept
  unless another is clearly nearer the cursor (distance below 0.7× the
  followed one's).
- D3: the robot holds still whenever nothing is selected (fixes 16, 37); a
  still mouse keeps its selection indefinitely, only switching the tool off
  ends it (fix 40).
- D4: the autonomous planner's pick and costs are recorded (fix 48,
  `dg_pick_idx`, `dg_*`), never executed.
- D6: gaps only.
- The robot drives holonomic (fix 44): it strafes and keeps its heading.

## 2. Running a session

After C++ changes, rebuild the overlay:
`cd ~/il_overlay_ws && catkin build --no-status dynamic_gap rviz_plugins manual_gap_msgs`.

```
source ~/il_overlay_ws/devel/setup.bash
roslaunch arena_bringup start_trials.launch man_gap:=true \
    record_il:=true il_data_root:=<data_root> il_operator:=<id> \
    [il_session:=<name>] [ped_vmax:=<m/s>] \
    [tm_modules:=benchmark benchmark_suite:=<suite>.yaml] \
    [man_gap_max_bearing_deg:=60 man_gap_switch_ratio:=0.7]
```

**Designed dataset:** `~/run_il_dataset.sh [session]` (copy in
`.claude/skills/monitor-il-run/run_il_dataset.sh.example`) runs the
`il_dataset_small_v1.yaml` suite with data root `/home/abdel/il_data`, operator
`ewan` and RViz under gdb. The pilot script `~/run_il_pilot.sh [session]`
(optional `PED_VMAX=<m/s>`) does the same with the `50ped_daniel.yaml` suite
(preliminary data). With a benchmark the first launch relaunches itself to load
the suite's map and records into `<contest>_<date_time>_<first stage>`; the
short folder written before the relaunch holds no driving. All stages on that
map run in this one session, each with its own scenario, seed and timeout from
its first episode on (fix 51); the session's `episodes.csv` has each episode's
timeout. A stage that changes the map, robot or task mode relaunches into a new
session. To run only some stages, comment the others out in the suite file.

If move_base_flex dies (`[rto/move_base_flex-5] process has died`, exit -11;
a crash inside move_base_flex, not the planner), the robot keeps the last
velocity command until the episode ends: stop the run, note it in
`notes.txt`, and treat the episodes from the crash on as unusable. Random starts on `map1_small` can spawn the robot
in collision; prefer a benchmark suite with fixed starts.

A session name can be used once; the recorder refuses a non-empty folder.

In RViz press `g` (ContinuousGapSelectionTool). Hover the mouse over the gap
to take, in its direction from the robot; hold the mouse still to keep it.
If the candidate-number display is on, the followed gap's number is green.
Left-click drives to a point (click-to-move; those ticks are not labels).
Switching tools stops the robot. Do not use other RViz tools during a
session.

After the session, write anything odd into
`<data_root>/il_gap_selection/<session>/notes.txt`.

## 3. Checks before a long session (replaces plan 1.10)

Run a 2-minute test session, then:

```
il_tools/il_check_session <data_root>/il_gap_selection/<session>
```

- [ ] All automatic checks pass.
- [ ] `usable_ticks` is most of `cursor_ticks`; `ticks_with_choice` > 0.
- [ ] Samples arrive at ~9–10 Hz with non-empty `candidates` and a goal.
- [ ] Playback with `il_tools/il_replay_viewer` shows the selected path where
      the robot went.
- [ ] Mouse held still for 10 s: the selection stays (no hold-still ticks).
- [ ] Tool switched off: the robot stops within 0.5 s.
- [ ] Pointer moved off the map: the robot keeps the last selection; those
      ticks have `cursor_live = false`.
- [ ] After an episode reset, `episode_id` increments and ticks within 1 s
      of the reset have `in_reset_window = true`.
- [ ] A click-to-move produces `SOURCE_CLICK_TO_MOVE` ticks.
- [ ] `cursor.header.frame_id` is `map` and no clock warning is logged by the
      planner ("is RViz on sim time?").

**At launch:**
- [ ] With a benchmark, the relaunch carries `record_il:=true` and
      `il_session:=<runid>_<stage>`.
- [ ] The session folder exists, `session_0.bag.active` grows and
      `rosbag record` runs.
- [ ] `versions.yaml`: dynamic_gap at the branch head and not dirty.
- [ ] `status = COLLIDING` on every tick means the robot spawned in
      collision; restart.

**While running:** RViz exiting with code -11 is a known segfault; the sim
and recorder keep running but no new cursor messages arrive (the last
selection is held). Note it in `notes.txt`.

**After each session:** also run
`.claude/skills/monitor-il-run/review_session.py <session>` (scenario per
episode, goal consistency, rates, `cmd_vel` without a selection, unusable
ticks by cause, gap switches with the mouse still).

## 4. After collection

```
il_tools/il_check_session <sessions...> --yaml checks.yaml
il_tools/il_build_dataset --out <dataset_dir> --lag <s> \
    --max-cursor-dist <m> --max-bearing-diff <rad> --min-tie-margin <m> <sessions...>
```

Keep the session folders unchanged; the dataset can always be rebuilt from
them with other thresholds. Do not pool sessions recorded before and after
fixes 43 (selection rule) or 44 (holonomic) unmarked; see overview II.10.
