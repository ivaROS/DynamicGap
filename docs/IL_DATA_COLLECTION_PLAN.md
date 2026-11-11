# Imitation Learning Data Collection — Plan and Issue Register

One file holding both the plan (Part 1) and every known issue (Part 2).
It merges and supersedes `IL_DATA_COLLECTION_PLAN.md` (original issue list),
`IL_DATA_COLLECTION_DESIGN.md` (first fix design) and
`IL_DATA_COLLECTION_REVIEW.md` (review of both).

**Status (2026-10-22, dynamic_gap `5ecc15a`, fixes up to 45).** Phases 1,
3, 4 (code) and 6 (tools) are implemented: fixes A–D, the message package
`manual_gap_msgs`, `record_il.launch`, and the offline tools in `il_tools/`
(checks, dataset build, replay viewer). All PL issues are fixed except the
four kept as docs only (23, 26, 30, 31). Each change has a doc in
`docs/il_pipeline_fixes/`, unit tests, and was checked in a headless sim with
a fake cursor (manual and autonomous). Open: R-25 and R-27 (need a real
session to look at), the pilot (Phase 2), the collection protocol (Phase 5,
`IL_COLLECTION_PROTOCOL.md` is a template), and an autonomous benchmark
regression run.

**This file keeps the history.** Part 1 is the plan as designed; several
parts were changed while implementing it. The current system is described
in `COMPLETE_OVERVIEW_IL_DOC.md`; where the two disagree, the overview
reflects the code. Deviations from Part 1:

- **Cursor validity (1.5 P2, P4).** The cursor is never ignored for lack of
  mouse movement or for lack of new messages: the `last_move_wall` rule and
  the 0.5 s `cursorTimeout` were both dropped (fix 40; user requirement: a
  still mouse keeps its selection). Only `tool_active = false` ends the
  selection. Resets are flagged in the sample instead (`in_reset_window`).
- **Behaviour (D3, 1.1 principles).** "Flag only" was overridden: in manual
  mode the robot holds still whenever nothing is selected, and the planner's
  own control laws never run (fixes 16, 37).
- **Selection rule (D2).** Hover-nearest was replaced by a direction cone
  plus a sticky switch (fix 43, `/man_gap_max_bearing_deg`,
  `/man_gap_switch_ratio`).
- **Rate.** The planning tick is 10 Hz configured, ≈9 Hz measured, not 5 Hz
  (fix 15).
- **Usable label.** The rule also requires `selected_idx >= 0`
  (`il_bags.is_usable_label`); early-return ticks record `selected_idx = -1`
  (fix 45).
- **Control.** The robot drives holonomic since fix 44; sessions before it
  are non-holonomic.

**Basis.** Line references are pinned to these commits (R-34):

| Repo | Commit | Working tree |
|---|---|---|
| `planners/dynamic_gap` | `f27f357bb9804e7d57b0115e60001c6b577c0fd0` | `rviz/arena.rviz` modified |
| `arena/arena-rosnav` | `8088929cbd2fd6df50ffd5474580a6c579285e4a` | config files modified; RViz tool clean |

The DG issues and their file/line evidence come from the original plan and
were re-checked against these commits. Code sketches and member names in the
fixes are still unverified until implemented.

**Changes from the first design.** This plan differs from
`IL_DATA_COLLECTION_DESIGN.md` in six ways, each driven by the review:

1. The goal is added to the sample.
2. The cursor is carried in a custom `CursorState` message.
3. A dead cursor is flagged, not acted on. The robot's behaviour changes only
   at episode reset. The `HELD` mode is dropped.
4. The autonomous planner's pick and cost are dropped from version 1.
5. Cursor liveness is judged in wall time.
6. Build order starts with a thin end-to-end slice and a pilot.

---

# Part 1 — The plan

## 1.1 Intent

Collect demonstrations of a human choosing which gap the robot should take,
to train a model that predicts that choice. One training sample is:

```
(scene observation, goal, variable-length candidate gap set)  ->  index into that set
```

The model does not detect gaps and does not produce trajectories. The
candidate set is part of the observation and must be recorded exactly as the
human saw it.

### What good data means

1. **Complete.** Each sample holds the options, the scene, the goal and the choice.
2. **Consistent.** The set the human saw is the set the label indexes into.
3. **Honest.** A label counts only when the human was really choosing.
4. **Self-contained.** A sample can be read without re-joining streams offline.
5. **Re-derivable.** Enough raw data is kept to recompute labels under another rule.
6. **Matched.** The human decides from the same information the model will get.

### Principles

- The sample is defined at the planning tick (10 Hz configured, ≈9 Hz
  measured; first written here as 5 Hz, see PL-15) and stamped in-process.
- Record facts and flags at collection time. Filter and shift offline.
- Do not change robot behaviour unless the data cannot be made honest otherwise.
- Every new line in the planner is guarded by `manualSelectionMode_`.

## 1.2 Decisions

The "Default assumed here" column is the plan as written; "Current state"
is what the code does as of 2026-10-22.

| # | Decision | Default assumed here | Current state |
|---|---|---|---|
| D1 | Does a still mouse mean a map spot or a direction from the robot? | Whatever is under the cursor on screen (see 1.5, T2). The RViz view then decides which it is. With the shipped view (top-down, following `rto/base_link`) a still mouse means a fixed offset from the robot. | As assumed; re-projected every publish (fix 29). |
| D2 | Hover-nearest, or latch a gap until the operator points elsewhere? | Hover-nearest for version 1; revisit after the pilot. | **Decided:** direction cone + sticky switch (fix 43). Thresholds to check in the pilot. |
| D3 | On a dead cursor: change robot behaviour, or only flag? | Flag only. | **Overridden:** the robot holds still without a selection (fixes 16, 37); a still mouse is not a dead cursor (fix 40). |
| D4 | Include the autonomous planner's pick? | Not in version 1. | **Reversed:** recorded since fix 48 (`dg_pick_idx`, per-candidate `dg_*` costs), without acting on it. |
| D5 | What is the model given as input? | Scan, robot velocity, goal, candidate set. RViz is cut down to match. | Inputs as assumed; RViz is **not** cut down yet (PL-26). |
| D6 | Gaps only, or also "go straight" and "wait"? | Gaps only; forced-choice ticks are flagged. | Gaps only; ungap and idling are off in both modes (PL-31), so the flags are always false. |
| D7 | Where bags are stored (`data_root`). | Not set. | Pilot uses `/home/abdel/il_data`; not fixed in the protocol. |

## 1.3 Fix A — one candidate list per tick

Build one `std::vector<ManualCandidate>` once per tick in `runPlanningLoop`.
`publishManualCandidateMarkers` and `getCursorSelectedTrajectory` both read it.

- Every tube is listed, including infeasible ones, in `traj_idx` order.
- The recorded label is the position in this array.
- The dense training index is derived offline by masking on `shown`.
- Gap identity over time is `(left_model_id, right_model_id)`. IDs are
  unique for the planner's lifetime (1.11, R-21), so `episode_id` is not
  needed for uniqueness but is kept for segmentation. Gap bearing is
  recorded as a second continuity cue.
- Selection runs first, then markers are published, and the highlight is
  driven by the cursor selection, chosen by identity.

Closes DG-13, DG-14, DG-15, R-19.

## 1.4 Fix B — one sample message per tick

Messages live in a small separate package so offline tools do not need to
build the planner. `tick_seq` increments on every tick and a sample is
published on every exit path of the planning loop, with an empty candidate
set if need be. There are three early returns before candidates exist
(not ready, colliding, no gaps; 1.11 R-28).

The sketches below are the design. The implemented messages are in
arena-rosnav `utils/ros/msgs/manual_gap_msgs/msg/` (`SCHEMA_VERSION = 2` since fix 48)
and add to `ManualGapSample`: `status` (`PLANNED`, `NOT_READY`, `COLLIDING`,
`NO_GAPS`), `cursor_valid` (a cursor point was available for selection) and
`hold_still` (zero velocity commanded this tick). `episode_id` is -1 before
the first `/scenario_reset`.

### `CursorState.msg` (published by the RViz tool)

```
Header  header                 # sim stamp; frame_id = RViz fixed frame
time    wall_stamp             # wall clock at publish
bool    tool_active            # false in the one message sent on deactivate
bool    pick_ok                # the current screen position hits the ground plane
geometry_msgs/Point point      # in header.frame_id; valid only if pick_ok
int32   screen_x
int32   screen_y
time    last_move_wall         # wall clock of the last real mouse move
```

### `ManualGapCandidate.msg`

```
int32   traj_idx               # index into gapTrajs / gapTubes (sparse)
int32   display_id             # number shown to the human (dense); -1 if not shown
int32   left_model_id
int32   right_model_id
bool    feasible               # non-empty trajectory
bool    shown                  # drawn as selectable this tick
uint16  tube_len               # number of gaps in the tube
# gap geometry: robot frame at the gap snapshot stamp
float32 left_range
float32 left_theta
float32 right_range
float32 right_theta
float32 bearing                # gap centre bearing
# models: robot frame at the gap snapshot stamp
geometry_msgs/Vector3 left_model_pos
geometry_msgs/Vector3 left_model_vel
geometry_msgs/Vector3 right_model_pos
geometry_msgs/Vector3 right_model_vel
# odom frame at the tick
geometry_msgs/Point   goal_pos
geometry_msgs/Pose    terminal_pose
geometry_msgs/Point[] path     # downsampled candidate trajectory
float32 cursor_dist            # cursor to terminal pose; NaN if none
float32 cursor_bearing_diff    # bearing(cursor) - bearing(terminal), from robot; NaN if none
```

### `ManualGapSample.msg` (published by the planner)

```
uint8 SOURCE_NONE          = 0   # no cursor has ever arrived, or no candidates
uint8 SOURCE_GAP_CURSOR    = 1   # a gap was selected from the cursor
uint8 SOURCE_CLICK_TO_MOVE = 2   # following a click-created trajectory
uint8 SOURCE_MODEL         = 3   # reserved: a learned model chose

uint16  schema_version
Header  header                   # planning tick, sim time
time    wall_stamp
uint32  tick_seq
int32   episode_id               # scenario_reset counter
bool    in_reset_window          # tick is close to an episode reset

# observation
sensor_msgs/LaserScan scan       # copied with the gaps under gapMutex_
time    gap_snapshot_stamp
time    odom_stamp
geometry_msgs/Pose    robot_pose_odom
geometry_msgs/Twist   robot_vel
geometry_msgs/Point   goal_robot_frame
geometry_msgs/Point   local_waypoint_robot_frame
ManualGapCandidate[]  candidates
bool    ungap_would_apply        # forced-choice tick (D6)
bool    idle_would_apply

# cursor
CursorState cursor               # latest message received, verbatim
time    cursor_rx_wall           # planner wall clock at receipt
geometry_msgs/Point cursor_odom  # the point actually used for selection
bool    cursor_live              # see 1.5, P2

# choice
uint8   source
int32   selected_idx             # position in candidates[]; -1 if none
int32   selected_left_model_id
int32   selected_right_model_id
geometry_msgs/Point click_point_odom   # for CLICK_TO_MOVE
```

**A tick is a usable expert label when**
`source == GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0`
(`il_bags.is_usable_label`; the `selected_idx` term was added with fix 45).

The manual branch also sets the planner's current left/right model IDs, as
the autonomous path does.

Closes DG-01, DG-02, DG-03, DG-08; partly DG-04; R-06, R-10, R-12, R-14,
R-15, R-16, R-28.

## 1.5 Fix C — cursor validity

The tool's last-known-position publisher stays. A still mouse is a deliberate
hold. Nothing here requires the operator to keep moving the mouse.

### Tool side

- **T1. Publish `CursorState`** at 10 Hz while active, and one final message
  with `tool_active = false` on `deactivate()`.
- **T2. Re-project every publish.** Store the mouse's screen position and
  viewport, not the 3D point. On each timer tick, intersect the mouse ray
  with the ground plane:

  ```cpp
  #include <rviz/geometry.h>

  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  Ogre::Vector3 hit;
  msg.pick_ok = rviz::getPointOnPlaneFromWindowXY(viewport_, ground,
                                                  screenX_, screenY_, hit);
  ```

  What is published is then always what is under the cursor on screen, even
  when the camera follows the robot. This is required, not optional: the
  shipped view follows the robot (1.11, R-17).
- **T3. Pointer leaves the window:** set `pick_ok = false`.
- **T4. Camera control:** the tool never forwards events to the view
  controller, so the camera cannot be moved while it is active, and
  switching tools calls `deactivate()` (1.11, R-22). With the following
  top-down view this is acceptable; forwarding wheel/zoom events is optional.

### Planner side

- **P1. Transform properly.** Use the message's own `frame_id` and stamp
  through the tf buffer; reject and warn on failure. Record the transformed
  point.
- **P2. Compute `cursor_live`**, without changing behaviour:

  ```
  cursor_live = tool_active && pick_ok
             && (wall_now - cursor_rx_wall) < cursorTimeout   // ~0.5 s wall
             && last_move_wall > last_reset_wall
  ```

  *As implemented (fix 40):* `cursor_live = tool_active && pick_ok && tf ok`.
  Neither the age term nor the `last_move_wall` term is used: the last known
  cursor is held until `tool_active = false`. Old cursor messages are found
  offline from `cursor_rx_wall` (`il_bags.CURSOR_FRESH_S`, 0.5 s).
- **P3. Clock check.** If the cursor's sim stamp and the planner clock differ
  by seconds, log an error (RViz is not on sim time).
- **P4. Reset — the one behaviour change.** On `scenario_reset`, clear
  `hasCursorPos_`, `cursorPosOdomFrame_`, `selectedManualCandidateId_`,
  `hasTrajCreationPoint_`, `isTrackingCreatedTraj_`, record
  `last_reset_wall`, and ignore the cursor until `last_move_wall` is later
  than it. Subscribe to `scenario_reset` directly: `Planner::reset()` is
  never called on an episode reset (1.11, R-04). Also clear gap estimator
  state, which does survive the teleport (1.11, R-26).

  *As implemented:* the reset clears the previous gaps and estimators, the
  click-to-move state and the selection (on the planning thread), and the
  ground-truth agent maps (PL-25). The cursor itself is **not** cleared and
  is not ignored until the mouse moves: the tool re-projects the screen
  point after the teleport, and a still mouse keeps its selection. Reset
  ticks are excluded by `in_reset_window` instead.
- **P5. No distance gate in the planner.** `cursor_dist` and
  `cursor_bearing_diff` are recorded per candidate; gating is done offline
  with thresholds set from the pilot.

Closes DG-05 (offline), DG-06, DG-07, DG-10, DG-11, DG-12, DG-20, R-02,
R-03, R-05, R-17, R-20, R-24.

## 1.6 Fix D — recording

**Format.** rosbag, via a new `record_il.launch`.

**Topics.** The sample topic, the `CursorState` topic, `scan`, `odom`, `acc`,
`cmd_vel`, `/tf`, `/tf_static`, the goal, the global plan, the static map,
`scenario_reset`, per-episode task-generator output, the pedsim topic,
collision/bumper output, `/traj_creation_point`. No marker topics.

**Options.** lz4 compression; split by size; latched topics repeated into
each split (check that your `rosbag record` supports `--repeat-latched`).

**Layout.**

```
<data_root>/il_gap_selection/
  2026-10-05_session01/
    session_0.bag, session_1.bag, ...
    params.yaml        all rosparams at start
    versions.yaml      git hash and git diff of each repo
    view.rviz          the RViz config used
    notes.txt          operator ID, instruction given, anything odd
```

*As implemented* (`il_session_recorder`, fix 07): also `versions/<repo>.diff`
(uncommitted changes) and `episodes.csv` (one row per
`/scenario_reset_begin` and `/scenario_reset`, with wall and sim time, start
and goal); bags split at 2048 MB; the recorder refuses an unset, relative or
in-repo `data_root` and a non-empty session folder, and is `required`, so a
refusal stops the launch. A benchmark relaunch keeps recording (PL-38).

`data_root` is a launch argument and sits outside the git repo (D7).

**Automatic checks, run on every bag.**

- `tick_seq` is contiguous.
- Recomputing nearest-endpoint from the recorded cursor and candidates
  reproduces `selected_idx` on every `GAP_CURSOR` tick.
- All sim stamps are on one clock.
- Every episode has an outcome (success, collision or timeout).
- No `.bag.active` files remain.

**During sessions.** Watch free disk space; the recorder stops quietly when
it runs low.

Closes DG-16, DG-17, DG-18, DG-19, R-07, R-08, R-09, R-13, R-36.

## 1.7 Offline processing

The bags are the raw record. A script reads them and writes the training set.

- **Label filter.** Keep ticks that pass the rule in 1.4. Apply distance,
  bearing and near-tie thresholds chosen from the pilot.
- **Reaction lag (DG-09).** For lag *k* (set in wall time, converted with the
  recorded real-time factor): take the selected identity at tick *t*, find
  the same identity in tick *t − k*, pair that index with the observation
  from *t − k*. Drop the sample if the identity does not exist there.
- **Episode filter.** Drop or separately tag episodes that end in collision.
- **Splits.** By episode, ideally by scenario or map. Never by tick.
- **Reporting.** Per session: ticks, usable ticks, decisions (identity
  switches backed by mouse movement), ticks with more than one candidate.

*As implemented:* `il_tools/il_check_session` (checks and report) and
`il_tools/il_build_dataset` (filters, lag, split 80/10/10 by episode hash);
see `il_pipeline_fixes/il-offline-tools.md` and overview II.8. The lag is set
in wall seconds and matched through `wall_stamp`, not converted with the
real-time factor. Episode outcomes (`GOAL_REACHED`, `TIMEOUT`, `ABORTED`,
plus a collision flag) are derived offline by `il_bags.episode_outcomes`;
the task generator publishes none.

## 1.8 Collection protocol (to be written before collecting)

One page covering: the instruction given to operators (what to optimise);
who operates and a warm-up period; session length and breaks; the maps,
scenarios and pedestrian densities to cover; the target in decisions, not
ticks; the fixed RViz config.

## 1.9 Order of work

| Phase | Work | Done when |
|---|---|---|
| 0 | Code checks: R-04, R-17, R-19, R-21, R-22, R-25, R-26, R-27. Pin repo commits. Settle D1–D7. | **Code checks done (1.11).** R-25, R-27 need a running sim. D1–D7 open. |
| 1 | Thin slice: message package, single candidate list (A), minimal sample (B), `record_il.launch`, offline loader. | **Done.** A recorded sim session loads and passes the automatic checks (`il_tools/il_check_session`). |
| 2 | Pilot: ten minutes of collection. Measure tie rate, switch rate, trivial-tick fraction, identity lifetimes, cursor liveness. | Thresholds and D2 are decided from data. **In progress.** Pilot runs (`~/run_il_pilot.sh`, benchmark suite `50ped_daniel.yaml`, data root `/home/abdel/il_data`) led to fixes 37–45; D2 is decided (fix 43). Thresholds and reaction lag are still to be set. `il_check_session` reports decisions and choice ticks; tie margins and cursor distances are in every candidate. |
| 3 | Cursor work (C): tool and planner. | The checklist in 1.10 passes. **Done in code** (issues 11, 14, 16, 29); the RViz-side items of 1.10 need a run with a real mouse. |
| 4 | Recording hardening, metadata, replay viewer. Regression run of the autonomous planner. | Benchmark unchanged with manual mode off. **Done** except a full benchmark. Short check (headless, `map1_small`, random scenarios with seed 42, 60 s timeout, 8 min per planner, scored with the fixed metrics): planner at `f27f357` 10 episodes, 2 goal / 5 collision / 3 timeout, 1.50 collisions per episode, mean speed 0.45 m/s; current planner 9 episodes, 2 / 3 / 4, 1.56, 0.46 m/s. No sign of a regression, but too few episodes to call it unchanged. |
| 5 | Write the protocol; collect. | **Open.** Template: `IL_COLLECTION_PROTOCOL.md`. |
| 6 | Offline processing and dataset build. | **Tools done** (`il_tools/il_build_dataset`); thresholds and lag from the pilot. |

## 1.10 Checks before a long session

Superseded by section 3 of `IL_COLLECTION_PROTOCOL.md` (written after fixes
15 and 40), which is the list to use. The original list is kept below; its
5 Hz rate and its reset rule ("`cursor_live` false until the mouse moves")
no longer hold.

- [ ] Samples arrive at 5 Hz with non-empty `candidates` and a goal.
- [ ] Cursor sim stamp and sample stamp are on the same clock.
- [ ] `cursor.header.frame_id` is `map`.
- [ ] `selected_idx` matches the gap the robot visibly follows.
- [ ] Mouse held still for 10 s: `cursor_live` stays true.
- [ ] Tool deactivated: `cursor_live` goes false within 0.5 s.
- [ ] Pointer moved out of the window: `cursor_live` goes false.
- [ ] After a reset: `episode_id` increments; `cursor_live` is false until
      the mouse moves.
- [ ] A click-to-move produces `CLICK_TO_MOVE` ticks.
- [ ] The automatic bag checks in 1.6 pass on the test bag.

## 1.11 Phase 0 results (2026-10-07)

Static reading of the code at the pinned commits. "Runtime" means it needs a
running sim to settle.

| ID | Result | Finding |
|---|---|---|
| R-04 | **Confirmed, worse** | `Planner::reset()` is called only from `recordAndCheckVel` (`Planner.cpp:2681`) on a planning failure. The call in `DynamicGapPlanner.cpp:105` is commented out. Nothing in the planner reacts to an episode reset. |
| R-17 | **Confirmed** | `arena.rviz:823-833`: view is `rviz/TopDownOrtho` with `Target Frame: rto/base_link`, so the camera follows the robot. The tool stores a 3D map point (`continuous_gap_selection_tool.cpp:83-91`), so under a still mouse the published point stays put in the map while the screen scrolls. |
| R-19 | **Confirmed, different** | Markers are published before selection (`Planner.cpp:2138-2152`). The green highlight compares `display_id` with `selectedManualCandidateId_`, which only the old numeric topic `/manual_traj_selection` sets (`:706-711`). In cursor mode the highlight never reflects the cursor pick at all. |
| R-21 | **Confirmed, mostly good** | IDs come from `currentModelIdx_` (`Planner.h:550`), a counter that only increments (`GapPointAssociator.cpp:174`). IDs are never reused and never reset, so they are unique for the planner's lifetime, across episodes. Copies, manipulation and propagation keep the ID (`RotatingFrameCartesianKalmanFilter.cpp:95,140`). Association is per point, nearest within `assoc_thresh` (1.0 m), so a split or merge changes the pair, and a point can inherit a nearby edge's ID. Raw and simplified gaps draw from the same counter. |
| R-22 | **Confirmed** | `processMouseEvent` returns `0` and never calls the view controller, so the camera cannot move while the tool is active. Switching tools calls `deactivate()`, which stops publishing; the planner keeps using the last point (DG-06). |
| R-26 | **Confirmed** | Nothing clears `prevRawGaps_` / `prevSimplifiedGaps_` or their models on a reset. After a teleport, any new gap point within 1.0 m (robot frame) of an old one inherits the old model, velocity estimate included. |
| R-28 | **Confirmed** | `runPlanningLoop` returns before candidates exist in three places: not ready (`:1973`), colliding (`:1980`), no gaps (`:2002`). `laserScanCB` also returns early when colliding (`:294`), leaving gaps from the previous scan. |
| DG-10 | **Low risk, keep the check** | `arena.rviz:798` sets `Fixed Frame: map`. The planner still assumes it. |
| DG-12 | **Low risk, keep the check** | `use_sim_time` (`start_trials.launch:77`) and RViz (`:172`) are in the same launch; roslaunch sets params before starting nodes. Only a separately started RViz would break it. |
| R-25 | Runtime | Needs a sim run to see whether a tick can mix pre- and post-reset data. |
| R-27 | Runtime | Needs a sim run to see whether the sim clock jumps backwards at reset. |
| R-34 | **Done** | Commits pinned at the top of this file. |

**New observation.** The tool sets `lastKnownPos_.header.stamp` only on a
mouse move and the timer re-sends that message unchanged, so the current
`/gap_selection_point` stamp is already "sim time of last move". Not a bug,
but any code reading it as "time of publish" is wrong.

---

# Part 2 — Issue register

**IDs.** `DG-xx` are from the original plan (DG-20 was added during design).
`R-xx` are from the review.
**Severity.** BLOCKER: good data is impossible without a change. HIGH: data
will be silently wrong or unusable. MED: degrades quality; workaround exists.
LOW: annoyance or future-proofing.
**Tag.** DOC: follows from the documents. CHECK: needed a code or setup
check; the result is in 1.11.
**Status.** As of 2026-10-21 the fixes in Part 1 are implemented; an issue is closed by the section in "Plan" except where noted in the header (R-25, R-27, R-11/D5, R-29/R-32 protocol, R-33 benchmark). "Plan" gives the section that addresses it.

## Index

| ID | Sev | Tag | Title | Plan |
|---|---|---|---|---|
| DG-01 | BLOCKER | DOC | Selection never leaves `getCursorSelectedTrajectory` | 1.4 |
| DG-02 | BLOCKER | DOC | Candidates exist only as markers, then are deleted | 1.4 |
| DG-03 | BLOCKER | DOC | Detection, selection and cursor run at three rates | 1.4 |
| R-10 | BLOCKER | DOC | The goal is not in the sample | 1.4 |
| DG-04 | HIGH | DOC | Nearest-endpoint is a proxy for intent; ties invisible | 1.4, 1.7 |
| DG-05 | HIGH | DOC | No abstain and no gate on the nearest match | 1.5 P5, 1.7 |
| DG-06 | HIGH | DOC | Cursor state never cleared; leaks across resets | 1.5 P2, P4 |
| DG-07 | HIGH | DOC | No staleness check; failed picks hold the old point | 1.5 T2, P2 |
| DG-08 | HIGH | DOC | Click-to-move looks like a gap selection | 1.4 |
| DG-09 | HIGH | DOC | Human reaction lag unmodelled | 1.7 |
| DG-10 | HIGH | DOC | Cursor frame assumed to be `map` | 1.5 P1 |
| DG-13 | HIGH | DOC | Two definitions of the candidate set | 1.3 |
| DG-14 | HIGH | DOC | Dense `display_id` vs sparse trajectory index | 1.3 |
| DG-15 | HIGH | DOC | Stable gap identity not recorded | 1.3 |
| DG-20 | HIGH | DOC | Tool republishes the pre-reset point after a reset | 1.5 P4 |
| R-01 | HIGH | DOC | Autonomous pick and cost are not free to record | 1.2 D4 |
| R-04 | HIGH | CHECK ✔ | Reset is never called on an episode reset | 1.5 P4 |
| R-05 | HIGH | DOC | First design changed robot behaviour needlessly | 1.5 |
| R-07 | HIGH | DOC | Record list was incomplete | 1.6 |
| R-11 | HIGH | CHECK | The human sees more than the model will | 1.2 D5, 1.8 |
| R-13 | HIGH | DOC | No episode outcome recorded | 1.6, 1.7 |
| R-17 | HIGH | CHECK ✔ | A still mouse does not point where the operator sees it | 1.5 T2 |
| R-18 | HIGH | DOC | Choice is matched against a set the human has not seen | 1.2 D2, 1.7 |
| R-21 | HIGH | CHECK ✔ | The model-ID pair: unique, but changes on split/merge | 1.3 |
| R-25 | HIGH | CHECK (runtime) | Samples straddle the reset | 1.4 |
| R-26 | HIGH | CHECK ✔ | Gap estimators survive the teleport | 1.5 P4 |
| R-29 | HIGH | DOC | Ticks are not independent samples | 1.7, 1.8 |
| R-32 | HIGH | DOC | No collection protocol | 1.8 |
| R-36 | HIGH | DOC | No automatic consistency checks | 1.6 |
| DG-11 | MED | DOC | Cursor transformed at latest time, not its stamp | 1.5 P1 |
| DG-12 | MED | DOC | RViz sim-time assumed; fails silently | 1.5 P3 |
| DG-16 | MED | DOC | Planner topics are namespaced, cursor topics global | 1.6 |
| DG-17 | MED | DOC | No rosbag infrastructure in the repo | 1.6 |
| DG-18 | MED | DOC | Episode boundaries and metadata not wired in | 1.4, 1.6 |
| R-02 | MED | DOC | Ground-plane pick can still fail | 1.5 T2, T3 |
| R-03 | MED | DOC | Cursor timeout should use wall time | 1.5 P2 |
| R-06 | MED | DOC | Message fields had no frames or times | 1.4 |
| R-08 | MED | DOC | Session-level metadata is not enough | 1.6 |
| R-09 | MED | CHECK | rosbag details glossed over | 1.6 |
| R-12 | MED | DOC | Candidate paths not recorded | 1.4 |
| R-14 | MED | DOC | No wall time or real-time factor | 1.4 |
| R-15 | MED | DOC | No schema version; messages inside the planner package | 1.4 |
| R-16 | MED | DOC | Action space differs from the deployed planner | 1.2 D6, 1.4 |
| R-19 | MED | CHECK ✔ | Highlight ignores the cursor pick entirely | 1.3 |
| R-20 | MED | DOC | A distance gate conflicts with pointing in a direction | 1.5 P5 |
| R-22 | MED | CHECK ✔ | Camera cannot move while the tool is active | 1.5 T4 |
| R-24 | MED | DOC | A bare `PointStamped` forces workarounds | 1.4, 1.5 T1 |
| R-27 | MED | CHECK (runtime) | Sim clock behaviour at reset unknown | Phase 0 |
| R-30 | MED | DOC | Expert-only data and drift | 1.4 |
| R-31 | MED | DOC | No split rule | 1.7 |
| R-33 | MED | DOC | No regression check on the autonomous planner | 1.9 Phase 4 |
| R-35 | MED | DOC | Build order hid problems until late | 1.9 |
| R-37 | MED | DOC | No pilot | 1.9 Phase 2 |
| R-38 | MED | DOC | No way to look at the labels | 1.9 Phase 4 |
| DG-19 | LOW | DOC | Bag volume over long sessions | 1.6 |
| R-23 | LOW | DOC | Near-ties measurable but no rule set | 1.7 |
| R-28 | LOW | CHECK ✔ | Ticks with no sample | 1.4 |
| R-34 | LOW | DOC ✔ | Line references are not pinned to a commit | Header |

✔ = checked against the code; see 1.11.

## 2.1 The label is lost

**DG-01 — Selection never leaves `getCursorSelectedTrajectory`** (BLOCKER)
- *Problem.* The function (`Planner.cpp:854`) picks the nearest trajectory
  and sets it current, but never assigns `lowestCostTrajIdx`. The manual
  branch returns early at `:2183`, so the current left/right model IDs (set
  only at `:1720`, `:1724`, `:1744`, all non-manual) are never updated.
- *Effect.* The label exists only as a local variable. No recording of
  existing topics recovers it.
- *Fix.* Publish it in `ManualGapSample`; set the model IDs in the manual branch.

**DG-02 — Candidates exist only as markers, then are deleted** (BLOCKER)
- *Problem.* Gap geometry, models, goals and feasibility go out only as
  `visualization_msgs` markers. The `Gap *` / `GapTube *` objects are deleted
  at `Planner.cpp:2186-2190`.
- *Effect.* The model's input is destroyed every tick; markers carry no IDs
  or flags.
- *Fix.* Structured `candidates[]` copied before the delete.

**DG-03 — Three rates** (BLOCKER)
- *Problem.* Gap detection runs in `laserScanCB` at 25 Hz, selection in the
  planning loop (10 Hz sim; first written as 5 Hz, see PL-15), the cursor at
  10 Hz wall, on different threads.
- *Effect.* A sample is only well defined at the planning tick; anything
  else needs an offline three-way join.
- *Fix.* Define and stamp the sample at the tick; copy scan and gaps
  together under `gapMutex_`.

**DG-08 — Click-to-move looks like a gap selection** (HIGH)
- *Problem.* `tryHandleTrajCreationPoint` returns `GAP` while tracking a
  click-created trajectory (`Planner.cpp:49-103`, called at `:862`).
- *Effect.* Those ticks are identical to real selections at the output.
- *Fix.* `source = CLICK_TO_MOVE`; excluded from labels.

## 2.2 Candidate set and identity

**DG-13 — Two definitions of the candidate set** (HIGH)
- *Problem.* The marker publisher (`:735`) and the selector each build their
  own list, with separate skip rules and hardcoded flags (`:740-741`).
- *Effect.* They agree by coincidence; drift mislabels the whole dataset.
- *Fix.* One list per tick, read by both.

**DG-14 — Dense `display_id` vs sparse trajectory index** (HIGH)
- *Problem.* `gapTrajs[i]` maps to `gapTubes[i]` (`:1230`) with empty
  entries for infeasible tubes; `display_id` skips those holes.
- *Effect.* The two indices differ whenever a candidate is infeasible.
- *Fix.* Array position is canonical; both indices are recorded.

**DG-15 — Stable gap identity not recorded** (HIGH)
- *Problem.* Per-point model IDs exist (`GapPointAssociator.cpp:125-139`)
  but are not saved; raw indices reshuffle every tick.
- *Effect.* "Held one gap for 2 s" and "switched ten times" look the same.
- *Fix.* Record the ID pair and gap bearing per candidate.

**R-21 — The model-ID pair** (HIGH, CHECK ✔)
- *Finding.* IDs come from a lifetime counter: never reused, never reset
  between episodes, kept through copy, manipulation and propagation.
  Association is per point within 1.0 m, so splits and merges change the
  pair for the same opening, and a point can inherit a neighbour's ID.
- *Effect.* A pair change is not proof the human switched. The lag shift
  (DG-09) depends on this.
- *Fix.* Key on the pair; use bearing as a second cue; count switches only
  when backed by mouse movement.

**R-19 — Highlight ignores the cursor pick** (MED, CHECK ✔)
- *Finding.* Markers are published before selection, and the highlight is
  driven by `selectedManualCandidateId_`, which only `/manual_traj_selection`
  sets. In cursor mode the operator never sees which gap is selected except
  through `curr_exec_dg_traj`.
- *Fix.* Select first, then publish; highlight the cursor pick by identity.

## 2.3 Cursor validity

**DG-06 — Cursor state never cleared** (HIGH)
- *Problem.* `hasCursorPos_` is set at `:722` and never reset.
  `Planner::reset()` (`:2611`) does not clear any manual-mode state.
- *Effect.* The planner uses the last cursor forever after the tool is
  switched off, and across episode resets.
- *Fix.* `cursor_live` flag; clear on `scenario_reset`.

**DG-07 — No staleness check; failed picks hold the old point** (HIGH)
- *Problem.* The tool updates its stored point only on a `MouseMove` whose
  `get3DPoint` succeeded (`continuous_gap_selection_tool.cpp:83`).
- *Effect.* Hovering over empty space silently keeps the old point.
- *Fix.* Ground-plane pick re-projected every publish; `pick_ok` flag.

**DG-20 — Tool republishes the pre-reset point** (HIGH)
- *Problem.* After a reset the tool resends the old point within 100 ms,
  undoing any clearing in the planner.
- *Fix.* Ignore the cursor until `last_move_wall` is later than the reset.
- *As implemented.* Not ignored: the tool re-projects the held screen point
  every publish (PL-29), so after the teleport it is again what is under the
  mouse; reset ticks are excluded by `in_reset_window`.

**DG-05 — No abstain and no gate** (HIGH)
- *Problem.* The nearest-candidate search (`:869-890`) has no threshold.
- *Effect.* A cursor far from everything still yields a confident label.
- *Fix.* Record distance and bearing difference; gate offline.

**R-17 — A still mouse does not point where the operator sees it** (HIGH, CHECK ✔)
- *Finding.* The shipped view follows `rto/base_link`. The tool stores a 3D
  map point, so as the robot drives the screen scrolls under a still mouse
  but the published point stays fixed in the map.
- *Fix.* Store the screen position; re-project each publish.

**R-02 — Ground-plane pick can still fail** (MED)
- *Problem.* The ray misses the plane above the horizon, and no event
  arrives once the pointer leaves the window.
- *Fix.* `pick_ok = false` in both cases.

**R-03 — Cursor timeout should use wall time** (MED)
- *Problem.* A sim-time timeout flaps if the sim does not run at real time.
- *Fix.* Wall clock for liveness.

**R-20 — A distance gate conflicts with pointing in a direction** (MED)
- *Problem.* Pointing well past a gap along its bearing is a clear choice
  that a metric gate rejects.
- *Fix.* Record both distance and bearing difference; decide offline.

**R-22 — Camera cannot move while the tool is active** (MED, CHECK ✔)
- *Finding.* The tool never forwards mouse events to the view controller.
  Switching tools to move the camera deactivates it.
- *Fix.* With the following top-down view, accept it. Optionally forward
  wheel events for zoom.

**R-24 — A bare `PointStamped` forces workarounds** (MED)
- *Problem.* Tool state had to be inferred from timeouts and point equality.
- *Fix.* `CursorState` message.

**R-05 — First design changed robot behaviour needlessly** (HIGH)
- *Problem.* Timeout, distance gate and `HELD` all altered control, against
  the "record facts, filter offline" principle, and `HELD` relied on
  unverified ID stability.
- *Fix.* Flag only; behaviour changes at reset alone.
- *Later.* Clean data did need behaviour changes: hold still without a
  selection (PL-16, PL-37) and a sticky selection (PL-43). No timeout or
  distance gate was added.

## 2.4 What the label means

**DG-04 — Nearest-endpoint is a proxy for intent** (HIGH)
- *Problem.* The cursor is matched to a trajectory endpoint
  (`:869-890`), which depends on the goal placer and trajectory generator.
- *Effect.* Changing the pipeline shifts label meaning; near-ties are
  indistinguishable from confident picks.
- *Fix.* Record the cursor, all distances and full geometry so labels can be
  recomputed.

**R-18 — Choice is matched against a set the human has not seen** (HIGH)
- *Problem.* Tick *t*'s cursor is matched to tick *t*'s candidates before
  they are drawn. A jumping endpoint flips the selection with no input.
- *Options.* Latch by identity; hysteresis; or keep and filter switches with
  no mouse movement behind them (possible via `last_move_wall`).
- *Fix.* Version 1 keeps hover-nearest and filters offline; decide D2 after
  the pilot.
- *Outcome.* Hover-nearest switched gaps about every second tick, so D2 was
  decided before the pilot: direction cone plus a sticky switch (PL-43).

**DG-09 — Reaction lag unmodelled** (HIGH)
- *Problem.* The cursor reflects an earlier rendered frame plus human
  reaction time.
- *Effect.* Pairing observation(t) with action(t) leaks the future.
- *Fix.* Offline shift by identity (1.7).

**R-23 — Near-ties: no rule set** (LOW)
- *Fix.* Threshold chosen from pilot data.

## 2.5 What the model needs to see

**R-10 — The goal is not in the sample** (BLOCKER)
- *Problem.* Neither earlier document records where the robot is going.
- *Effect.* The choice cannot be predicted without it.
- *Fix.* Goal and local waypoint in the sample.

**R-11 — The human sees more than the model will** (HIGH, CHECK)
- *Problem.* RViz may show the map, global plan and pedestrians outside
  sensor view.
- *Effect.* Choices driven by that become noise.
- *Fix.* Fix the model's inputs (D5); cut RViz to match; save the config.

**R-12 — Candidate paths not recorded** (MED)
- *Fix.* Downsampled path and tube length per candidate.

**R-16 — Action space differs from the deployed planner** (MED)
- *Problem.* "Go straight" and "wait" are disabled, so the human must always
  pick a gap.
- *Correction (2026-10-17, PL-31).* They are disabled in both modes: ungap
  and idling trajectory generation is commented out (`Planner.cpp:2117-2128`),
  so the autonomous planner is gap-only too.
- *Fix.* Flag ticks where either would have applied (D6).

**R-01 — Autonomous pick and cost are not free** (HIGH)
- *Problem.* Manual mode returns before costs are computed; the scoring
  block has side effects.
- *Fix.* Dropped from version 1 (D4). Since fix 48 `pickTraj` runs in
  manual mode into local outputs, without the comparison block, and the
  costs come from the trajectories already generated; see
  [doc 48](il_pipeline_fixes/48-dynamic-gap-pick.md).

## 2.6 Frames and time

**DG-10 — Cursor frame assumed to be `map`** (HIGH)
- *Problem.* The tool stamps with RViz's fixed frame; the planner always
  applies map-to-odom (`:721`). The shipped config uses `map`.
- *Fix.* Transform from the message's own frame.

**DG-11 — Transform taken at latest time** (MED)
- *Problem.* `tfCB` (`:690`) looks up at `ros::Time(0)`.
- *Fix.* Look up at the cursor's stamp; record the result.

**DG-12 — RViz sim time assumed** (MED)
- *Problem.* If RViz starts before `use_sim_time` is set
  (`start_trials.launch:77`), cursor stamps are wall time. Low risk when
  RViz is started by that launch file.
- *Fix.* Clock check in the planner; test-bag check.

**R-06 — Message fields had no frames or times** (MED)
- *Fix.* Stated per field in 1.4.

**R-14 — No wall time or real-time factor** (MED)
- *Problem.* Human lag is in wall time; all stamps were sim time.
- *Fix.* Wall stamp in each sample.

## 2.7 Reset and episodes

**R-04 — Reset is never called on an episode reset** (HIGH, CHECK ✔)
- *Finding.* `Planner::reset()` runs only after a planning failure. The
  call from the plugin is commented out.
- *Fix.* Subscribe to `scenario_reset` directly.

**R-25 — Samples straddle the reset** (HIGH, CHECK, runtime)
- *Problem.* A tick can mix a pre-reset scan with a post-reset pose.
- *Fix.* `in_reset_window` flag; excluded from labels.

**R-26 — Gap estimators survive the teleport** (HIGH, CHECK ✔)
- *Finding.* Previous gaps and their models are never cleared on reset. Any
  new point within 1.0 m of an old one inherits its model.
- *Fix.* Clear previous gaps on `scenario_reset` (manual mode only, so the
  benchmark path is unchanged).

**R-27 — Sim clock at reset** (MED, CHECK, runtime)
- *Problem.* A backwards jump clears tf buffers and breaks bag ordering.

**DG-18 — Episode boundaries and metadata** (MED)
- *Problem.* `scenario_reset` (`task_generator_node.py:116`, latched
  `std_msgs/Int16`) and run parameters are not captured.
- *Fix.* `episode_id` in the sample; metadata files per session.

**R-13 — No episode outcome** (HIGH)
- *Fix.* Record collision and goal-reached events.

**R-28 — Ticks with no sample** (LOW, CHECK ✔)
- *Finding.* Three early returns before candidates exist.
- *Fix.* Publish on every exit path; `tick_seq` always increments.

## 2.8 Recording

**DG-17 — No rosbag infrastructure** (MED)
- *Problem.* The only recorder is a CSV logger in `arena_evaluation`.
- *Fix.* `record_il.launch`.

**DG-16 — Namespacing** (MED)
- *Problem.* Planner topics are robot-relative (`Planner.h:486`); cursor
  topics are global (`:187`, `:189`).
- *Fix.* Record list names both; cursor topic names become parameters.

**DG-19 — Bag volume** (LOW)
- *Fix.* No markers; compression; splitting.

**R-07 — Record list was incomplete** (HIGH)
- *Fix.* Goal, global plan, `acc`, map, collisions, task info added.

**R-08 — Session-level metadata is not enough** (MED)
- *Fix.* Per-episode task info in the bag; `git diff` saved.

**R-09 — rosbag details** (MED, CHECK)
- *Problem.* Split needs a size; latched topics land only in the first
  split; crashes leave `.bag.active`; low disk stops recording quietly; a
  slow recorder drops messages.
- *Fix.* Options and checks in 1.6.

**R-15 — No schema version** (MED)
- *Fix.* `schema_version`; separate message package.

## 2.9 Learning-side

**R-29 — Ticks are not independent samples** (HIGH)
- *Problem.* One held gap produces many near-identical ticks; many ticks
  have a single candidate.
- *Fix.* Targets and reports in decisions.

**R-32 — No collection protocol** (HIGH)
- *Problem.* No fixed operator instruction, coverage or targets.
- *Fix.* Protocol in 1.8.

**R-30 — Expert-only data and drift** (MED)
- *Fix.* `SOURCE_MODEL` reserved for later corrective collection.

**R-31 — No split rule** (MED)
- *Fix.* Split by episode.

**R-33 — No regression check on the autonomous planner** (MED)
- *Fix.* Guard new code; benchmark before and after.

## 2.10 Process

**R-34 — Line references not pinned** (LOW) — done, see header.

**R-35 — Build order hid problems** (MED)
- *Fix.* Thin slice first.

**R-36 — No automatic consistency checks** (HIGH)
- *Fix.* Checks in 1.6.

**R-37 — No pilot** (MED)
- *Fix.* Phase 2.

**R-38 — No way to look at the labels** (MED)
- *Fix.* Replay viewer in Phase 4.

---

# System map (as implemented, 2026-10-22)

Full version with rates, topics and parameters: `COMPLETE_OVERVIEW_IL_DOC.md`
(I.3–I.10, appendix B).

**Producer — `arena/arena-rosnav/utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.cpp`**

- Shortcut key `g`. While active, a 10 Hz wall timer publishes
  `manual_gap_msgs/CursorState` on `<prefix>gap_selection_cursor` (prefix
  property, default `/rto/`): the held screen position re-projected onto the
  ground plane at each publish, `pick_ok`, `tool_active`; one final message
  with `tool_active = false` on `deactivate()` (issues 11, 29).
- Left-click publishes one point on `<prefix>traj_creation_point`
  (click-to-move).

**Consumer — `planners/dynamic_gap/src/Planner.cpp`**

- `/man_gap` rosparam gates `manualSelectionMode_` (default off, issue 06;
  set by `arena_bringup/launch/start_trials.launch`).
- `/man_gap_max_bearing_deg` (60) and `/man_gap_switch_ratio` (0.7) set the
  selection rule (issue 43).
- `cursorCB` transforms the cursor from its own frame to odom (issue 29); the
  last known cursor is held until `tool_active = false` (issue 40).
- Gap detection, association and estimation run in `laserScanCB`.
- `runPlanningLoop` (manual mode): `selectManualTrajectory` builds the one
  candidate list, selects from click-to-move or the cursor (direction cone,
  nearest terminal point, sticky switch; issue 43) and fills the sample; then
  `publishManualCandidateMarkers`; a `ManualGapSample` is published on
  `<ns>/manual_gap_sample` on every return path (fixes A, B). Nothing
  selected → zero velocity (issues 16, 37).
- Control is holonomic (`holonomic: true` in simulation-setup
  `configs/mbf/local/dgap_local_planner_params.yaml`, issue 44).
- Recording: `arena_bringup/launch/record_il.launch` (fix D, issue 07);
  offline: `il_tools/`.

**Rates (robot `rto`)**

| Stream | Rate | Source |
|---|---|---|
| Laser scan | 25 Hz | `rto/model_params.yaml:39` |
| Planning loop | 10 Hz configured, ≈9 Hz measured in sim (PL-15) | `configs/mbf/move_base_params.yaml:10` |
| Cursor | 10 Hz wall clock | `continuous_gap_selection_tool.cpp` (`createWallTimer`) |
| Odometry | 50 Hz | `rto.model.yaml:25` |
| Global path replan (NavfnROS) | 5 Hz | `move_base_params.yaml:6` |

Scan: 512 beams, ±pi, `max_range` 5.0 m.

---

# Part 3 — Pipeline review issues (2026-10-17)

A review of the whole collection pipeline, beyond the planner: RViz tool,
planner, task generator, launch files, mbf config, and the
`arena_evaluation` recorder and metrics. The recorder and metrics produced
the 80% (manual) and 50–60% (autonomous) success figures, so treat those
figures as unreliable until PL-01 to PL-04 are fixed.

**Fix workflow.** Each fix lives on branch `imitation_learning` in the
repo it touches and is pushed to the Udog-ILLINOIS fork (fixes up to PL-51
were made on `il-data-pipeline-fixes`, where `imitation_learning` starts). Each fixed issue has
a doc in `docs/il_pipeline_fixes/NN-*.md` (what changed, why, effects, test)
and its own test.

**Status values.** Fixed: committed with a passing test and sim smoke test.
In progress: work in the tree, not committed. Open: not started. Docs only:
out of scope for this pass by decision; documented, not changed.

## 3.1 Status

| ID | Sev | Issue | Repo | Status |
|---|---|---|---|---|
| PL-01 | CRIT | Old recorders keep writing into earlier folders after `change_directory` | evaluation | Fixed `edaa1dc`, [doc](il_pipeline_fixes/01-recorder-duplicate-folders.md) |
| PL-02 | CRIT | Recorder time is `secs*1e10+nsecs`; metrics divide by 1e10 | evaluation | Fixed `473ac6e`, [doc](il_pipeline_fixes/02-recorder-timestamp-scale.md) |
| PL-03 | CRIT | Success never checks the goal; timeouts can score as success | evaluation | Fixed `61d1c5c`, [doc](il_pipeline_fixes/03-success-definition.md) |
| PL-04 | CRIT | Metrics episode loop starts at 0 and stops at the first gap | evaluation | Fixed `61d1c5c`, [doc](il_pipeline_fixes/04-episode-loop.md) |
| PL-05 | CRIT | `params.yaml` does not record `man_gap` | evaluation | Fixed `61d1c5c`, [doc](il_pipeline_fixes/05-run-metadata.md) |
| PL-06 | CRIT | `manualSelectionMode_` defaults to true; bad `/man_gap` is ignored | dynamic_gap | Fixed `0996314`, [doc](il_pipeline_fixes/06-man-gap-default.md) |
| PL-07 | CRIT | No IL topics recorded (cursor, candidates, selection, goal) | arena-rosnav | Fixed `fd011986f` (arena-rosnav), [doc](il_pipeline_fixes/07-il-recording.md) |
| PL-08 | HIGH | Recorder misses topics advertised late (`cmd_vel`); stale rows | evaluation | Fixed `95548ae`, [doc](il_pipeline_fixes/08-recorder-topic-discovery.md) |
| PL-09 | MED | `start_goal.csv` gets one row per tick | evaluation | Fixed `416afc6`, [doc](il_pipeline_fixes/09-start-goal-per-episode.md) |
| PL-10 | LOW | `after_reset_task()` called twice per reset | arena-rosnav | Fixed `37829ef80`, [doc](il_pipeline_fixes/10-double-resume.md) |
| PL-11 | MED | Cursor timer runs on sim time; republishes the last-move stamp | arena-rosnav | Fixed `393e248df` (arena-rosnav), [doc](il_pipeline_fixes/11-cursor-timer-clock.md) |
| PL-12 | MED | Two nodes named `/rviz` when `headless` ≤ 1 | arena-rosnav | Fixed `7d65f68dc`, [doc](il_pipeline_fixes/12-duplicate-rviz.md) |
| PL-13 | HIGH | Candidate markers never deleted; ghost numbers on screen | dynamic_gap | Fixed `57e5709`, [doc](il_pipeline_fixes/13-ghost-candidate-markers.md) |
| PL-14 | MED | Manual-mode topics are global; shared across robots | dynamic_gap, arena-rosnav | Fixed `57e5709`, `393e248df`, [doc](il_pipeline_fixes/14-namespaced-topics.md) |
| PL-15 | MED | `controller_frequency` / patience defined twice | simulation-setup | Fixed `832aed2` (simulation-setup), [doc](il_pipeline_fixes/15-duplicate-controller-rate.md) |
| PL-16 | HIGH | Manual mode with no cursor drives under obstacle avoidance | dynamic_gap | Fixed `57e5709`, [doc](il_pipeline_fixes/16-no-cursor-hold-still.md) |
| PL-17 | HIGH | Cursor pick never sets start time, lifespan or model IDs | dynamic_gap | Fixed `57e5709`, [doc](il_pipeline_fixes/17-cursor-pick-bookkeeping.md) |
| PL-18 | HIGH | Click-to-move active in autonomous mode; wrong status text; short lifespan | dynamic_gap, arena-rosnav | Fixed `0996314`, `393e248df`, [doc](il_pipeline_fixes/18-click-to-move.md) |
| PL-19 | HIGH | RViz toolbar can teleport the robot, move the goal or force a reset | dynamic_gap | Fixed `57e5709`, [doc](il_pipeline_fixes/19-dangerous-rviz-tools.md) |
| PL-20 | HIGH | Low speed in manual mode triggers "Planning Failed" and FAILURE | dynamic_gap | Fixed `0996314`, [doc](il_pipeline_fixes/20-manual-stall-reset.md) |
| PL-21 | HIGH | `trajFlag` uninitialised on early returns | dynamic_gap | Fixed `bf42455`, [doc](il_pipeline_fixes/21-trajflag-uninitialised.md) |
| PL-22 | HIGH | `/scenario_reset` published after the teleport | arena-rosnav | Fixed `9859a394a`, [doc](il_pipeline_fixes/22-reset-begin-signal.md) |
| PL-23 | HIGH | Collisions do not end episodes | arena-rosnav | Docs only (decision 2026-10-17), [doc](il_pipeline_fixes/23-collisions-do-not-end-episodes.md) |
| PL-24 | MED | `scan_` newer than the gaps on collision ticks | dynamic_gap | Fixed `0996314`, [doc](il_pipeline_fixes/24-scan-gap-sync.md) |
| PL-25 | HIGH | Ground-truth pedestrian maps never cleared | dynamic_gap | Fixed `0996314`, [doc](il_pipeline_fixes/25-ground-truth-agents.md) |
| PL-26 | HIGH | Operator sees ground truth in RViz | dynamic_gap | Docs only (decision 2026-10-17), [doc](il_pipeline_fixes/26-operator-sees-ground-truth.md) |
| PL-27 | MED | Highlight follows the numeric selector, not the cursor pick | dynamic_gap | Fixed `57e5709`, `7b373c5`, [doc](il_pipeline_fixes/27-highlight-follows-cursor.md) |
| PL-28 | LOW | Click-to-move handled without the lock in autonomous mode | dynamic_gap | Fixed with PL-18 `0996314`, [doc](il_pipeline_fixes/28-click-to-move-lock.md) |
| PL-29 | HIGH | Cursor stores a map point; drifts with the camera, holds failed picks | arena-rosnav, dynamic_gap | Fixed `393e248df`, `57e5709`, `7b373c5`, [doc](il_pipeline_fixes/29-cursor-reprojection.md) |
| PL-30 | HIGH | GRU feature logger only on other branches | dynamic_gap | Docs only (decision 2026-10-17), [doc](il_pipeline_fixes/30-gru-logger-other-branches.md) |
| PL-31 | MED | Ungap and idling disabled in both modes | dynamic_gap | Docs only (decision 2026-10-17), [doc](il_pipeline_fixes/31-ungap-idle-disabled.md) |
| PL-32 | LOW | Velocity metric computed from positions | evaluation | Fixed `61d1c5c`, [doc](il_pipeline_fixes/32-velocity-metric.md) |
| PL-33 | MED | Collisions checked only on 400 ms samples | evaluation | Fixed `9aecac3`, [doc](il_pipeline_fixes/33-collisions-between-samples.md) |
| PL-34 | LOW | Pedestrian heading from force, no velocity recorded | evaluation | Fixed `9aecac3`, [doc](il_pipeline_fixes/34-pedestrian-heading.md) |
| PL-35 | LOW | Candidate labels black on dark grey, 2 m tall | dynamic_gap | Fixed `57e5709`, [doc](il_pipeline_fixes/35-candidate-label-visibility.md) |
| PL-36 | — | Repair script for already-recorded folders (dedupe, time fix) | evaluation | Fixed `34e9f59`, [doc](il_pipeline_fixes/36-repair-recorded-data.md) |
| PL-37 | HIGH | Manual mode drove under obstacle-avoidance control in collision | dynamic_gap | Fixed `6d2bfac`, [doc](il_pipeline_fixes/37-manual-never-planner-control.md) |
| PL-38 | HIGH | Benchmark relaunch dropped IL recording | arena-rosnav | Fixed `c99dd2017`, [doc](il_pipeline_fixes/38-benchmark-relaunch-drops-il-recording.md) |
| PL-39 | HIGH | Benchmark stages ran the default scenario (per-port config not read) | arena-rosnav | Fixed `7dcb61a57`, [doc](il_pipeline_fixes/39-benchmark-scenario-not-applied.md) |
| PL-40 | MED | Cursor dropped after 0.5 s without a cursor message | dynamic_gap | Fixed `94b09eb`, [doc](il_pipeline_fixes/40-cursor-held-without-messages.md) |
| PL-41 | MED | Pedestrian speed not settable at launch; pedsim steps in wall time (open) | arena-rosnav | Fixed `5605de45b` (argument), [doc](il_pipeline_fixes/41-pedestrian-speed-argument.md) |
| PL-42 | LOW | No robot-to-goal cue for the operator (line only in an uncommitted stash) | dynamic_gap | Fixed `ac447be`, [doc](il_pipeline_fixes/42-goal-line.md) |
| PL-43 | HIGH | Selected gap switches too often (close calls flip back and forth); gaps pointing away from the cursor get selected | dynamic_gap, arena-rosnav | Fixed `3c2ef1c`, arena-rosnav `2b1ecdea2` (arguments), [doc](il_pipeline_fixes/43-directional-sticky-selection.md) |
| PL-44 | MED | dgap ran non-holonomic on the holonomic rto (heading turned, no strafing) | simulation-setup | Fixed `7c1f551` (simulation-setup), [doc](il_pipeline_fixes/44-holonomic.md) |
| PL-45 | LOW | Ticks without planning (not ready, colliding, no gaps) recorded selected_idx 0 instead of -1 | dynamic_gap | Fixed `8d75c2a`, [doc](il_pipeline_fixes/45-early-return-selected-idx.md) |
| PL-46 | MED | Collision episodes not detected: colliding ticks carry no scan | dynamic_gap | Fixed `a4ceba5`, [doc](il_pipeline_fixes/46-collision-from-status.md) |
| PL-47 | LOW | Datasets only from whole sessions; no way to pick episodes | dynamic_gap | Fixed `8f5f49f`, [doc](il_pipeline_fixes/47-episode-sets.md) |
| PL-48 | HIGH | DynamicGap's own pick and costs not recorded (M-03); dataset lacks yaw, `is_current`, `shown`, `tube_len` | dynamic_gap, arena-rosnav | Fixed `50e7acd`, `1debace` (dataset), arena-rosnav `e9054b511`, [doc](il_pipeline_fixes/48-dynamic-gap-pick.md) |
| PL-49 | MED | Operator data has ~4 goal directions; no way to collect goal-directed choices without hours of driving | dynamic_gap, arena-rosnav | Done `66b8d9e`, arena-rosnav `6d642c737`, `949731ac7`; review fixes (collector) `acf0089`…`703a5f1`, [doc](il_pipeline_fixes/49-goal-cursor-operator.md) |
| PL-50 | HIGH | Gap scorer's terminal-cost term dropped on schema-1 data, so no pull toward the goal | dynamic_gap | Fixed `2afe89f`; review fixes (empty splits, ablation, identity) `acf0089`…`703a5f1`, [doc](il_pipeline_fixes/50-terminal-cost-term.md) |
| PL-51 | HIGH | Benchmark stages on the same map ran one episode late (first stage twice, last never) | arena-rosnav | Fixed arena-rosnav `caa8417c4`; review fixes (stage timeout and seed, run stops on a failed config, per-episode timeout in `episodes.csv`) `45e93f7a3`…`fb7afaeeb`; pipeline test 2026-10-26 (CS1–CS4 in order, collect → check → dataset → train → learned scorer in closed loop), [doc](il_pipeline_fixes/51-benchmark-stage-one-episode-late.md) |

Approved behaviour changes and the cursor-continuity requirement (a still
mouse keeps its selection; nothing may require mouse movement) are recorded
in each issue's doc as it is fixed.

## 3.2 Issues from the IL model design (M-xx)

From `IL_MODEL_AND_DATA_DESIGN.md`. All open except M-03 (fix 48).

| ID | Sev | Issue |
|---|---|---|
| M-01 | BLOCKER (MoG route) | No teleop or continuous human action data exists |
| M-02 | HIGH | The GRU does not run inside the planner; its tracks cannot be recorded |
| M-03 | HIGH | DynamicGap's own pick is never computed in manual mode. Fixed (PL-48) |
| M-04 | HIGH | The 80% human result had no `changeTrajectory()` checks or hysteresis |
| M-05 | HIGH | No HG-DAgger infrastructure (arbiter, takeover, `/control_owner`) |
| M-06 | MED | `cursor_live` cannot tell watching from intervening |
| M-07 | MED | The frozen GRU checkpoint is not pinned |
| M-08 | MED | Design pools 360 beams; the scan has 512 |
| M-09 | LOW | Policy rate 10 Hz vs planner tick 5 Hz. The tick is 10 Hz (PL-15), so the rates already match; keep the deployment rate a separate setting |

Decisions D8–D11 (teleop or cursor-only, record DynamicGap's pick, a
safety-checks-on condition, which GRU) are open; see the design doc.

## 3.3 Other findings

| Finding | Source | Status |
|---|---|---|
| `scripts/create_plots` `resize_time` divided seconds by 1e9, and metrics truncated row times to whole seconds, so plot time axes were zero | PL-02 work | Fixed `34e9f59` (row times are float seconds) |
| `time_diff` comment in the metrics files said ns; the value is seconds | PL-02 work | Fixed `34e9f59` |
| `Recorder._record` treated a time of 0 as unset | PL-02 work | Fixed `34e9f59` |
| Metrics crashed on an empty data cell (row written before the first odom/pedsim message) | PL-08 work | Fixed with PL-04 `61d1c5c` |
| If `Recorder()` raised during `change_directory`, recording stopped until the next call | PL-01 work | Fixed `34e9f59` (the old recorder keeps recording) |
| `reset_task()` had no lock; a manual `/reset_task` during a timer reset duplicated the begin id | PL-22 work | Fixed `5efff57db`, see [doc 22](il_pipeline_fixes/22-reset-begin-signal.md) |
| `show_rviz:=false` does not turn arena.rviz off (behaviour change, needs a decision) | PL-12 work | Open (decision) |
| The effective controller rate is 10 Hz (`move_base_params.yaml`), not dgap's 5 Hz | PL-15 work | Documented in PL-15; system map corrected |
| `time_in_personal_space` and the gaze totals in `PedsimMetrics` are cast to integers, truncating seconds | metrics review | Open |
| `arena_bringup` is a git repo nested in arena-rosnav (both track its files); `versions.yaml` records both | PL-07 work | Noted |
| In a manual sim run (fake cursor), mbf stopped calling the planner 22 s before the episode's timeout, 2.9 m from the goal; `NavfnROS/plan` had published only empty plans since the episode began. mbf output is not captured (robot nodes are started by the task generator), so the abort reason is unknown. The episode outcome is still right (duration from `episodes.csv`), but such episodes have a stretch without samples | e2e2 session, review of PL-07 | Open (investigate: capture mbf logs) |
| The candidate-number display (`/rto/manual_candidate_markers`) is `Enabled: false` in `arena.rviz` (`:786-788`), so the operator sees no numbers or green highlight unless they turn it on; `view.rviz` is copied at start, so a display toggled during a session is not recorded | overview review | Open (decision: enable in `arena.rviz`) |
| Pedsim steps on wall time while the robot runs on sim time, so pedestrians move faster than `vmax` relative to the robot when the sim runs below real time (~2.6× at RTF 0.37) | PL-41 work | Open; fix belongs in pedsim_ros |
| RViz segfaults (exit -11, NVIDIA 470 GL); cursor messages stop and the last selection is held. `rviz_prefix` (gdb) is not passed to benchmark relaunches | pilot runs | Open, not caused by the tool |
| move_base_flex segfaults (exit -11, in `libmbf_abstract_server` / `libmbf_costmap_server`) at a reset, mid-episode or at a relaunch's start; the robot then coasts on the last `cmd_vel` and can drift into the goal | 2026-10-24 goal-cursor runs, 2026-10-26 pipeline test | Open, inside move_base_flex; drop the episodes from the crash on |
| Random starts on `map1_small` can spawn the robot in collision: every tick `COLLIDING`, robot holds still | pilot runs | Workaround: a benchmark suite with fixed starts (`50ped_daniel.yaml`) |
| Gap switches with the mouse still (near-ties; association can pass an ID to a nearby edge) | pilot runs | Reduced by PL-43; recheck in the pilot |
| `ManualGapSample.msg` comment says `cursor_live` needs a "recent" message; since PL-40 there is no age check | PL-40 work | Open (comment to update) |
| `il_check_session`'s "selection reproducible" check recomputed plain nearest-feasible (`il_bags.nearest_feasible`), not the PL-43 rule (cone + sticky switch), so it failed on every post-43 session (56/385, 119/623 and 55/405 cursor ticks of the three pilot sessions) | doc update 2026-10-22 | Fixed: the check replays the PL-43 rule with the parameters from `params.yaml` (`il_bags.selection_mismatches`); 0 mismatches on all six pilot sessions, [doc](il_pipeline_fixes/il-offline-tools.md) |

# Unfiled / new issues

_Append below. Give an ID and move into a section once triaged._
