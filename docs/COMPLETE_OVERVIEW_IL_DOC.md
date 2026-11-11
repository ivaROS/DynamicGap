# Manual Gap Selection (MGS) — Complete Overview

Status: complete as of 2026-10-22 (dynamic_gap `5ecc15a`, fixes up to 45).
Where this file and the plan disagree, this file reflects the code; the plan
keeps the history.

Sources: `docs/IL_DATA_COLLECTION_PLAN.md`, `docs/IL_MODEL_AND_DATA_DESIGN.md`,
`docs/IL_COLLECTION_PROTOCOL.md`, `docs/il_pipeline_fixes/`,
`include/dynamic_gap/utils/ManualSelection.h`, `src/Planner.cpp`,
`src/DynamicGapPlanner.cpp`, `rviz/arena.rviz`, `il_tools/`,
arena-rosnav `utils/plugins/rviz_plugins/`, `utils/ros/msgs/manual_gap_msgs/`,
`arena_bringup/launch/{start_trials,record_il}.launch`,
`arena_bringup/scripts/il_session_recorder`, `task_generator/`,
simulation-setup `configs/mbf/`, `arena_evaluation/scripts/data_recorder_node`,
`.claude/skills/monitor-il-run/`.

---

# Part I — What MGS is and how it works

## I.1 Purpose

The operator picks which gap dynamic_gap follows, in place of the planner's
own lowest-cost pick. dynamic_gap still detects the gaps, builds a
trajectory for each and tracks the chosen one; the human only chooses. With
manual selection the success rate was about 80%, against 50–60% for the
autonomous planner (measured without `changeTrajectory()` checks, see
model design M-04), so the gap is in decision-making. Recording those
choices gives imitation-learning (IL) data for a learned gap chooser.

## I.2 Background: dynamic_gap

A **gap** is an opening between two obstacle edges in the LiDAR scan that the
robot could drive through. It has a left and a right point; each point is
tracked over time by an estimator with a stable ID (`left_model_id`,
`right_model_id`).

The planner does its work in two places that run at different rates on
different threads:

**Gap detection — `Planner::laserScanCB` (`Planner.cpp:316`), on every scan,
25 Hz sim time.**

1. Preprocess the scan. If the closest range is inside the robot radius
   (+7.5 mm), set `colliding_` and stop.
2. Detect raw gaps (`gapDetector_->gapDetection`).
3. Associate gap points with the previous scan's points, so an edge keeps its
   ID (`gapPointAssociator_`).
4. Update the estimators (`updateModels`): the Kalman filter that estimates
   each gap point's position and velocity. The GRU is meant to replace this
   velocity step; it does not run in the planner yet (model design M-02).
5. Simplify gaps (merge small ones), then associate and estimate again.
6. Store `currSimplifiedGaps_` together with `scan_` under `gapMutex_`.

It chooses nothing and commands nothing; it keeps the current gap picture.

**Planning tick — `Planner::runPlanningLoop` (`Planner.cpp:2307`), 10 Hz
configured, about 9 Hz measured, sim time.** move_base_flex calls
`DynamicGapPlanner::computeVelocityCommands` (`DynamicGapPlanner.cpp:78`),
which calls `runPlanningLoop` (line 99). Each tick:

1. Create the tick's `ManualGapSample`; a `ScopeExit` publishes it on every
   return path (manual mode only).
2. Early returns: not ready (no scan, goal or init), colliding, no gaps. In
   manual mode the robot holds still; the sample records the status.
3. Deep-copy the latest gaps, and the scan they came from, into the tick.
4. Gap manipulation → propagation into **gap tubes** (how each gap evolves
   over the horizon) → gap goal placement → feasibility check.
5. Future scan propagation and **trajectory generation per gap tube**, with
   pose costs.
6. Selection: manual mode `selectManualTrajectory` (`:978`, see I.6);
   autonomous mode `pickTraj` (lowest cost).
7. Draw candidate numbers; the controller turns the trajectory into
   `cmd_vel`.

One tick is one decision: one observation, one choice, one command. That is
why it is the IL sample unit.

The rate is 10 Hz, not 5 Hz: `dgap_local_planner_params.yaml` used to say
`controller_frequency: 5.0`, but `move_base_params.yaml` is loaded later and
sets 10.0 (fix 15). `/rto/cmd_vel` measured 8.6–9.1 Hz in live runs. The
global path (NavfnROS) is replanned at 5 Hz.

## I.3 Components

| Component | Where | Role |
|---|---|---|
| RViz `ContinuousGapSelectionTool` | arena-rosnav `utils/plugins/rviz_plugins/src/continuous_gap_selection_tool.cpp` | Publishes the cursor (`CursorState`) at 10 Hz wall clock; left click publishes `traj_creation_point` (click-to-move). Toggled with `g`. |
| Planner manual mode | `src/Planner.cpp`, `/man_gap` → `manualSelectionMode_` | `cursorCB` stores the cursor; `selectManualTrajectory` picks the candidate; publishes the sample and markers |
| `manual_gap_msgs` | arena-rosnav `utils/ros/msgs/manual_gap_msgs/msg/` | `CursorState`, `ManualGapCandidate`, `ManualGapSample` (II.3) |
| Candidate markers | `publishManualCandidateMarkers` → `manual_candidate_markers` | Numbers at each shown candidate's end point; green = followed this tick |
| IL recorder | arena-rosnav `arena_bringup/scripts/il_session_recorder`, `record_il.launch` | rosbag session (II.6) |
| Offline tools | `il_tools/` | Checks, dataset build, replay viewer (II.8) |

**RViz** is ROS's 3D viewer: the window with the map, scan, pedestrians and
candidate trajectories. A **tool** is an RViz plugin that takes over the
mouse inside that window.

## I.4 Data flow, one tick

```
LiDAR 25 Hz ──► laserScanCB ──► gaps + tracked gap points (gapMutex_)
                                        │
RViz mouse ──► cursor tool 10 Hz wall ──► cursorCB ──► cursor point, odom frame
                                        │
move_base_flex 10 Hz ──► runPlanningLoop ──► latest gaps + latest cursor
                                        ├─► candidate trajectories
                                        ├─► selection (I.6)
                                        ├─► markers in RViz
                                        ├─► cmd_vel
                                        └─► ManualGapSample (1 per tick) ──► rosbag
```

The three streams are not synchronised. The tick reads whatever gaps and
cursor are latest and records them in the sample, so no offline join is
needed (DG-03, fix A/B).

## I.5 The cursor

**RViz side** (`continuous_gap_selection_tool.cpp`):
- While active, a 0.1 s **wall timer** (`:40`) publishes `CursorState` on
  `<ns>/gap_selection_cursor`, also when the mouse is still: a still mouse is
  a deliberate hold.
- Each message re-projects the held screen position onto the ground plane
  (`pickGround`), so the point is what is under the mouse while the camera
  follows the robot (fix 29).
- Fields: ground point, `pick_ok` (pointer over the map and the pick hit the
  ground), `tool_active`, screen position, `last_move_wall`.
- Deactivating sends one message with `tool_active = false`.
- Left click publishes `traj_creation_point` (click-to-move) instead.

**Planner side** (`Planner::cursorCB`, `Planner.cpp:821`):
- Stores the message and its wall-clock arrival (`cursorRxWall_`).
- Transforms the point into the odom frame through tf with the message's own
  frame and stamp (falls back to the latest transform).
- Warns if the cursor's sim stamp is more than 2 s from planner time (RViz
  not on sim time).
- A failed pick or transform keeps the last good point.
- The cursor never expires (fix 40): only `tool_active = false` ends the
  selection. `cursor_live` in the sample is `usable && pick_ok && tf ok`
  (`Planner.cpp:1198`); it has no age check, despite the comment in the
  `.msg`. Old cursor messages are found offline from `cursor_rx_wall`
  (`il_bags.CURSOR_FRESH_S`, 0.5 s).

The cursor runs on wall time because it follows the operator's hand and the
screen, which keep running whatever the simulation speed.

## I.6 The selection rule

See II.2.4. In code: `selectManualTrajectory` (`Planner.cpp:978`) →
`inCursorDirection` (cone) → `nearestCandidate` → `holdCurrentCandidate`
(sticky, `manualSwitchRatio_`). Click-to-move is checked first
(`tryHandleTrajCreationPoint`) and records `SOURCE_CLICK_TO_MOVE`.

## I.7 Control in manual mode

Each tick ends in one of three ways (`Planner::ctrlGeneration`,
`Planner.cpp:2813`):

| Case | What drives | Sample |
|---|---|---|
| A gap is selected from the cursor | The freshest trajectory through that gap (rebuilt every tick, `setCurrentTraj`), tracked by the controller | `source = GAP_CURSOR`, `hold_still = false` |
| A click-to-move target is active | A trajectory to the clicked point, until within `xy_goal_tolerance` (0.25 m) or 2× its estimated time (fix 18) | `source = CLICK_TO_MOVE` |
| Anything else | Nothing: zero `cmd_vel` | `hold_still = true` |

"Anything else" covers no cursor yet, tool switched off, no gap in the
direction cone, no gaps, collision and not ready (`manualHoldStill_`, fixes
16, 37). The planner's own control laws (obstacle avoidance, idling, ungap)
never run in manual mode, and the stall check is off (fix 20), so a still
robot is never treated as a planning failure.

**Holonomic control (fix 44).** `holonomic: true` in simulation-setup
`configs/mbf/local/dgap_local_planner_params.yaml`. The rto strafes:
`cmd_vel` has `linear.x` and `linear.y` and `angular.z = 0`, so the heading
stays fixed (the 360° laser loses no view). The control law is constant speed
(`vx_absmax`) towards the lookahead pose, then the projection operator
(`projection_operator: true`) and clipping to `max_vel_x/y` (0.75 m/s).

The projection operator still adds an obstacle-repulsion term to the tracking
command, so close to obstacles the robot does not follow the selected path
exactly (fix 37). In collision no gaps are planned, so the operator cannot
steer out; the robot waits until a pedestrian moves away or the episode times
out.

## I.8 Operator interface

RViz opens `dynamic_gap/rviz/arena.rviz` (`start_trials.launch:189`, the only
RViz node, fix 12).

- **View:** `TopDownOrtho`, target frame `rto/base_link` (the camera follows
  the robot), fixed frame `map`. The camera cannot be moved while the gap tool
  is active (the tool consumes mouse events).
- **Shown by default:** map, pedsim walls, the processed scan
  (`/rto/modified_scan`), the robot marker, raw and simplified gaps with their
  estimator positions and velocities, candidate trajectories
  (`/rto/candidate_tube_trajectories`), the trajectory being executed
  (`/rto/curr_exec_dg_traj`), the subgoal, ground-truth pedestrian histories
  (`/rto/pedestrian_histories`, fix 26) and a faint yellow robot-to-goal line
  (`/rto/goal_arrow`, fix 42).
- **Off by default:** the candidate numbers (`/rto/manual_candidate_markers`,
  display `Enabled: false` at `arena.rviz:786-788`), pedsim tracked persons
  and groups, the goal marker, costmaps, TF. Turn the candidate numbers on by
  hand to see them; see II.12.
- **Candidate numbers** (when shown): one per shown candidate at its
  trajectory end point, `display_id`, white, 0.5 m tall; green = the gap the
  robot follows this tick (fixes 27, 35). None is green while holding still or
  in click-to-move. Numbers from earlier ticks are deleted every tick (fix 13).
- **Toolbar:** Interact, MoveCamera, Select, FocusCamera, Measure,
  ContinuousGapSelectionTool. Set-pose, set-goal and publish-point were
  removed because one stray click teleported the robot, moved the goal or
  reset the episode (fix 19).

**Controls** (ContinuousGapSelectionTool):

| Action | Effect |
|---|---|
| `g` | Activate the tool; the cursor starts publishing (10 Hz wall) |
| Hover over the ground | Select the gap nearest the cursor, in its direction (I.6) |
| Hold the mouse still | Keep the selection (the point under the mouse is re-projected every publish) |
| Pointer off the map | `pick_ok = false`; the last good point is kept, ticks are not `cursor_live` |
| Left click | Click-to-move to that point (not an IL label) |
| Switch to another tool | Sends `tool_active = false`; the robot holds still |

The tool's status bar reads "Hover: select the gap nearest the cursor.
Left-Click: drive to this point." with the ground coordinates.

## I.9 Parameters

Global rosparams set by `start_trials.launch` (lines 105-107), read once at
planner start (`Planner.cpp:124-150`) and saved in each session's
`params.yaml`:

| Parameter | Default | Valid | Meaning |
|---|---|---|---|
| `/man_gap` | `false` | `true`/`false` | Manual gap selection. Unset or not a bool → autonomous (fix 06). |
| `/man_gap_max_bearing_deg` | 60 | (0, 180] | Half-width of the direction cone around the robot-to-cursor bearing (fix 43) |
| `/man_gap_switch_ratio` | 0.7 | (0, 1] | Another gap replaces the followed one only if its cursor distance is below this fraction of the followed one's (fix 43) |
| `/ped_vmax` | 0.5 (task generator) | m/s | Pedestrian max speed; set only if the launch arg is given (fix 41) |

Out-of-range values log an error and fall back to the default.

Planner and controller settings that matter for IL (simulation-setup
`configs/mbf/`): `controller_frequency` 10 Hz and `planner_frequency` 5 Hz in
`move_base_params.yaml` (fix 15); `holonomic: true`, `projection_operator:
true`, `xy_goal_tolerance` 0.25 m in `local/dgap_local_planner_params.yaml`;
`max_vel_x/y` 0.75 m/s, `max_vel_theta` 1.0 rad/s, `robot_radius` 0.225 m and
scan `max_range` 5.0 m in `entities/robots/rto/configs/mbf/dgap_local_planner_params.yaml`.

**`start_trials.launch` arguments** used for collection:

| Argument | Default | Meaning |
|---|---|---|
| `man_gap`, `man_gap_max_bearing_deg`, `man_gap_switch_ratio` | `false`, 60, 0.7 | See above |
| `record_il` | `false` | Include `record_il.launch` (II.6) |
| `il_data_root` | — | Absolute folder outside every git repo; required with `record_il` |
| `il_session` | date and time | Session folder name |
| `il_operator` | — | Written to `notes.txt` |
| `ped_vmax` | — | Pedestrian max speed (m/s) |
| `tm_modules`, `benchmark_suite` | — | `benchmark` + a suite file for fixed scenarios |
| `map_file` | `map1_small` | Map when not using a benchmark |
| `timeout`, `desired_resets` | 180 s, 10 | Episode timeout and number of episodes |
| `rviz_prefix` | — | e.g. gdb, for an RViz crash backtrace; not passed to benchmark relaunches |

`record_il.launch` also takes `instruction`, `rviz_config`, `split_size_mb`,
`min_free_gb` and `namespace`; `start_trials.launch` does not pass these
through, so they keep their defaults unless `record_il.launch` is started on
its own.

## I.10 Rates and clocks

**Sim time** is the simulator's clock (`/clock`); **wall time** is the
computer's real clock. They match only at real-time factor 1; at 0.8×, one
wall second is 0.8 sim seconds. Robot, sensors and planner run on sim time;
the cursor tool runs on wall time.

| Stream | Rate | Clock | Source |
|---|---|---|---|
| LiDAR scan (512 beams, ±π, 5 m) | 25 Hz | sim | `rto/model_params.yaml:39` |
| Odometry | 50 Hz | sim | `rto.model.yaml:25` |
| Gap detection (`laserScanCB`) | 25 Hz, once per scan | sim | planner |
| Planning tick, selection, `cmd_vel` | 10 Hz configured, ≈9 Hz measured | sim | `configs/mbf/move_base_params.yaml:10` |
| Global path replan (NavfnROS) | 5 Hz | sim | `move_base_params.yaml:6` |
| Cursor (`CursorState`) | 10 Hz | wall | `continuous_gap_selection_tool.cpp:40` |
| `ManualGapSample` | 1 per planning tick | sim header + `wall_stamp` | `Planner.cpp:2313` |
| `arena_evaluation` CSV recorder | every 400 ms | sim | `data_recorder_config.yaml:3` |

Consequences:
- Cursor (10 Hz wall) and tick (10 Hz sim) are not aligned one to one; the
  tick uses the latest cursor.
- The reaction lag `lag_s` is set in wall seconds and matched through
  `wall_stamp` (II.8).
- At ~9 Hz, one hour of driving is about 32–36k ticks; an episode (60 s
  timeout) is up to ~550 ticks.

---

# Part II — Application: imitation-learning data pipeline

## II.1 Goal and what one sample is

`(scan, robot velocity, goal, candidate gap set) → index into that set`

The model only chooses. It does not detect gaps or plan trajectories, so the
candidate set is part of the input and must be recorded exactly as the
operator saw it (plan 1.1, D5).

### Runs vs. ticks

Sessions are recorded as whole episodes (spawn to goal, collision or
timeout), but behaviour cloning trains on single ticks: each (observation →
label) pair is an independent example, shuffled into batches. The model does
not see a run as a sequence; its only memory is what the inputs carry (the
estimator, later the GRU's 15-frame history).

The episode still matters for:
1. **Labels.** For the future waypoint model, the label at tick *t* is the
   robot's odometry over the next 2 s, taken from the same episode, padded and
   masked at the episode end.
2. **Filtering.** Collision episodes are dropped as a whole (II.8).
3. **Splits.** By episode, never by tick: neighbouring ticks are nearly
   identical, so a tick split leaks training data into validation.

### What good data means

From plan 1.1, with how the pipeline meets each today:

| Property | Meaning | How it is met |
|---|---|---|
| Complete | Each sample holds the options, the scene, the goal and the choice | One `ManualGapSample` carries scan, pose, velocity, goal, candidates, cursor and choice (II.3) |
| Consistent | The set the operator saw is the set the label indexes into | Markers, selection and label come from one candidate list per tick; `selected_idx` indexes `candidates[]` (fix A) |
| Honest | A label counts only when the operator was really choosing | Usable-label rule: `GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0`; hold-still and click-to-move ticks are flagged |
| Self-contained | A sample can be read without joining streams offline | Stamped and filled in the planner on every return path (fix B) |
| Re-derivable | Enough raw data to recompute labels under another rule | Per-candidate `cursor_dist` / `cursor_bearing_diff`, the raw cursor, and raw odom, scan and tf in the bag |
| Matched | The operator decides from what the model will get | **Not met:** RViz shows ground-truth pedestrian histories the model does not get (fix 26, II.2.7) |

## II.2 Critical design decisions

### II.2.1 Action space: gaps only
Ungap and idling are off in both modes, so "go straight" and "wait" cannot
be labelled. `ungap_would_apply` / `idle_would_apply` stay in the schema so
they can be enabled later (fix 31, D6).

### II.2.2 One candidate list and one sample per tick (fixes A and B)
- Markers, selection and the recorded label all come from one candidate
  list, built once per planning tick (10 Hz, I.10). Infeasible candidates
  are included, in tube order. `display_id` numbers only shown candidates,
  -1 for the rest.
- Gap identity is `(left_model_id, right_model_id)`, unique for the
  planner's lifetime; gap bearing is a second cue.
- One `ManualGapSample` per tick, stamped in the planner, published on every
  return path (`ScopeExit`). No offline join of detection, selection and
  cursor streams.
- Ticks that end before selection (not ready, colliding, no gaps) record
  `selected_idx = -1` (`newManualSample`, fix 45).

### II.2.3 Cursor meaning and validity (fix C)
- Continuous selection: a still mouse is a deliberate hold. The tool
  republishes `CursorState` at 10 Hz and re-projects the held screen
  position onto the ground each time, so the cursor stays "what is under the
  mouse" while the camera follows the robot (fix 29).
- The last known cursor position never expires — not for a still mouse, not
  when RViz stops publishing. Only `tool_active = false` ends the selection
  (`isCursorHeld`, fix 40). Replaces the planned 0.5 s timeout (plan P2).
- The cursor is transformed with its own frame and stamp through tf; a
  failed transform or pick keeps the last good point (P1).

### II.2.4 Selection rule (fix 43)
1. Direction cone: only gaps whose trajectory ends within
   ±`man_gap_max_bearing_deg` (default 60°) of the robot-to-cursor bearing.
2. Nearest terminal point to the cursor among those.
3. Sticky: the followed gap is kept unless another is clearly nearer,
   `dist < man_gap_switch_ratio × dist_current` (default 0.7). A gap is
   recognised by its left and right model ids; one with a gap point without
   a model (id -1) cannot be, so it is never kept and each tick on it counts
   as a new tracked trajectory (`hasModelIDs`; the offline tools do the same).

Replaces the plan's v1 hover-nearest (D2), which switched gaps about every
second tick. Both parameters are saved in `params.yaml`. Per-candidate
`cursor_dist` and `cursor_bearing_diff` allow recomputing either rule
offline (P5).

### II.2.5 The planner never drives in manual mode
- Nothing selected → zero velocity: no cursor, nothing in the cone, no gaps,
  collision, not ready (`manualHoldStill_`, fixes 16, 37). Obstacle
  avoidance, idling and ungap control never run in manual mode.
- Stall check off in manual mode (fix 20).
- Click-to-move is manual-only, tracked to tolerance or 2× estimated time
  (fixes 18, 28).
- Overrides the plan's D3 ("flag only"): clean data needed the behaviour
  change.

### II.2.6 Collection principles
- Record facts and flags live; filter and shift offline. Keep raw data so
  labels can be re-derived.
- Every planner change is guarded by `manualSelectionMode_`; autonomous mode
  is unchanged. `/man_gap` is off unless it clearly says true (fix 06).
- Episode boundaries come from `/scenario_reset` directly (`Planner::reset()`
  is not called on a reset). Ground-truth agent maps and selection state are
  cleared on it (fix 25).
- The green highlight is the gap the robot follows this tick (fix 27).

### II.2.7 Known open tensions
- The operator sees ground-truth pedestrians the scan may not see; this
  breaks "matched". Kept for now, `view.rviz` saved per session (fix 26).
- The robot now drives holonomic (fix 44); earlier sessions are
  non-holonomic and must not be mixed in unmarked.

## II.3 Message schema

All in arena-rosnav `utils/ros/msgs/manual_gap_msgs/msg/`. `SCHEMA_VERSION = 2` (fix 48).

### `ManualGapSample` — one planning tick

Published on `<ns>/manual_gap_sample`, once per tick, manual mode only,
including ticks that end before candidates exist.

| Part | Fields | Meaning |
|---|---|---|
| Bookkeeping | `schema_version`, `header` (tick, sim time, odom frame), `wall_stamp`, `tick_seq` (+1 every tick), `episode_id` (last `/scenario_reset`, -1 before), `in_reset_window`, `status` (`PLANNED`, `NOT_READY`, `COLLIDING`, `NO_GAPS`) | Which tick, which episode, whether it planned |
| Observation | `scan` (the scan the gaps came from), `gap_snapshot_stamp`, `odom_stamp`, `robot_pose_odom`, `robot_vel` (robot frame), `goal_robot_frame`, `local_waypoint_robot_frame`, `candidates[]`, `ungap_would_apply` / `idle_would_apply` (reserved, false) | What the operator saw |
| Cursor | `cursor` (latest `CursorState`, verbatim), `cursor_rx_wall`, `cursor_odom` (point used for selection), `cursor_valid`, `cursor_live` | Where the mouse was and whether it was in use |
| Choice | `source` (`NONE`, `GAP_CURSOR`, `CLICK_TO_MOVE`, `MODEL` reserved), `selected_idx` (position in `candidates[]`, -1 if none), `selected_left/right_model_id`, `click_point_odom`, `hold_still` | What was chosen |
| DynamicGap | `dg_pick_idx` (position in `candidates[]` of DynamicGap's lowest-cost gap, -1 if none; recorded, not executed; fix 48) | What the planner would have preferred |

A tick is a **usable label** when
`source == GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0`
(`il_bags.is_usable_label`): a gap was picked by the cursor, the cursor was
in use, and it was not the moment of a reset.

### `ManualGapCandidate` — one candidate of a tick

- Identity and display: `traj_idx` (sparse index into the tick's tubes),
  `display_id` (number drawn, -1 if not shown), `left_model_id`,
  `right_model_id`, `feasible`, `shown`, `tube_len`.
- Gap geometry, robot frame, at the gap snapshot: `left/right_range`,
  `left/right_theta`, `bearing`.
- Estimator states, robot frame: `left/right_model_pos`, `left/right_model_vel`
  (Kalman filter today, not the GRU).
- Trajectory, odom frame: `goal_pos`, `terminal_pose`, `path` (downsampled).
- Cursor relation: `cursor_dist` (cursor to terminal pose, m),
  `cursor_bearing_diff` (rad); NaN if no cursor or not feasible.
- DynamicGap's cost (fix 48): `dg_cost` = `dg_terminal_cost` (Q_f × distance
  to the local waypoint) + `dg_clearance_cost` (mean pose-wise cost); inf if
  the path has < 2 poses, NaN if not scored. Lower is better.

`selected_idx` indexes this array, so the label always refers to what was on
screen that tick.

### `CursorState` — from the RViz tool

`header` (sim stamp, RViz fixed frame), `wall_stamp`, `tool_active`,
`pick_ok`, `point` (valid only if `pick_ok`), `screen_x`, `screen_y`,
`last_move_wall`.

## II.4 Running a session

**Build.** The C++ packages `manual_gap_msgs`, `rviz_plugins` and
`dynamic_gap` are built in the overlay `~/il_overlay_ws` (extends
`~/arena_ws/devel`; its sources are symlinks into `src`). After C++ changes:
`cd ~/il_overlay_ws && catkin build --no-status dynamic_gap rviz_plugins manual_gap_msgs`.
Python packages (arena_bringup, task_generator, arena_evaluation) run from
`src` without a build.

**Minimal recorded run** (random scenarios on `map1_small`):
```
source ~/il_overlay_ws/devel/setup.bash
roslaunch arena_bringup start_trials.launch man_gap:=true \
    record_il:=true il_data_root:=<data_root> il_operator:=<id> [il_session:=<name>]
```

**Designed dataset** (`~/run_il_dataset.sh [session]`): the same with
`tm_modules:=benchmark benchmark_suite:=il_dataset_small_v1.yaml`, data root
`/home/abdel/il_data`, operator `ewan`, RViz under gdb.

**Pilot run** (`~/run_il_pilot.sh [session]`, optional `PED_VMAX=<m/s>`; its
sessions are preliminary): the same with the `50ped_daniel.yaml` suite. The 50ped_daniel suite uses the
`gru_learn` map with a fixed start and goal per scenario, which avoids the
spawn-in-collision problem of random starts on `map1_small` (II.12).

**Scenarios and episodes.** The task generator places robot, goal and
pedestrians at each reset, and `/scenario_reset` carries the new episode id.
- Random tasks (`tm_robots`, `tm_obstacles` `random`): new start, goal and
  pedestrians each episode; `desired_resets` episodes, `timeout` each.
- Benchmark (`tm_modules:=benchmark`): stages from the suite, each with its own
  scenario file and seed (fix 39). The first launch relaunches itself
  (`benchmark_resume:=true`) to load the suite's map; the relaunch keeps IL
  recording and records into `<contest>_<date_time>_<first stage>` (fix 38).
  It relaunches again only when the map, robot or task mode changes, so one
  session folder covers every following stage on that map. Since fix 51 each
  such stage gets its scenario, seed and timeout before the reset that starts
  it (before, every stage ran one episode late), and `episodes.csv` records
  each episode's timeout. The short folder written before the first
  relaunch holds no driving.
- Pedestrian speed: `ped_vmax:=<m/s>` (fix 41). Pedestrians step on wall time
  while the robot runs on sim time, so relative to the robot they move faster
  than `vmax` whenever the sim runs below real time (II.12).

**Operator.** In RViz press `g`, hover over the gap to take, hold still to
keep it (I.8). Do not use other RViz tools during a session; switching tools
stops the robot. Afterwards write anything odd into `notes.txt`.

A session name can be used once; the recorder refuses a non-empty folder.

## II.5 Episodes and resets

- An episode runs from one `/scenario_reset` to the next. The task generator
  teleports the robot and sets a new goal; the planner clears cursor,
  selection and estimator state on the reset (it subscribes directly;
  `Planner::reset()` is not called).
- `in_reset_window` marks ticks between `/scenario_reset_begin` and shortly
  after `/scenario_reset`; they are never labels.
- Collisions do not end episodes (PL-23); a collided run continues to goal or
  timeout and is flagged.
- The task generator publishes no outcome; it is derived offline (II.8).

**Episode loop.** The task generator checks the task every 0.5 s
(`_check_task_status`). An episode ends when the robot is within
`goal_radius` of the goal (`RobotManager._is_goal_reached`) or the timeout
passes; it then calls `reset_task()`:
1. publish `/scenario_reset_begin <id>` (latched, fix 22);
2. teleport the robot, set the new start and goal, respawn pedestrians;
3. publish `/scenario_reset <id>` (latched), with the same id.

Consumers compare ids rather than arrival order: begin id > reset id means a
reset is in progress; equal ids mean episode `<id>` is running. Resets are
serialised by a lock, so a manual `/reset_task` during a timed reset gets the
next id. `il_session_recorder` writes one `episodes.csv` row per begin and per
reset, with wall and sim time, start, goal and (fix 51) the task generator's
timeout.

**Success.** IL sessions get their outcome from `il_bags.episode_outcomes`
(II.8): `GOAL_REACHED`, `TIMEOUT` or `ABORTED`, plus a collision flag. The
`arena_evaluation` metrics use the same idea (fix 03): `TIMEOUT` first, then
`COLLISION`, then `GOAL_REACHED` only if the last pose is at the goal, else
`ABORTED`. Before fix 03 a timeout could score as a success, so the 80% /
50–60% figures in I.1 are unreliable until re-measured.

**Start and goal per episode.** In IL sessions they come from `episodes.csv`
and from the goal topic in the bag. In `arena_evaluation` folders
`start_goal.csv` now has one row per episode instead of one per tick (fix 09).

## II.6 Recording

`record_il:=true` on `start_trials.launch` includes `record_il.launch`, which
starts `il_session_recorder` (fix 07). It:

- **Refuses to start** if `data_root` is unset, not absolute, or inside a git
  repo, if `session` is not a plain folder name, or if the session folder is
  not empty. The node is `required`, so a refusal stops the launch instead of
  running unrecorded.
- **Writes** `<data_root>/il_gap_selection/<session>/` (empty `session` →
  date and time):
  ```
  session_0.bag, session_1.bag, ...  rosbag, lz4, split at split_size_mb (2048)
  params.yaml       every rosparam at start
  versions.yaml     commit, branch, dirty flag per repo
  versions/<repo>.diff  uncommitted changes
  view.rviz         the RViz config used
  notes.txt         operator, instruction, start and stop time
  episodes.csv      one row per /scenario_reset_begin and /scenario_reset
                    (wall and sim time, start, goal, timeout)
  ```
- **Records** (for `namespace:=rto`): `/rto/scan`, `/rto/odom`, `/rto/acc`,
  `/rto/cmd_vel`, `/tf`, `/tf_static`, `/map`, `/map_metadata`,
  `/rto/move_base_simple/goal`, `/rto/move_base_flex/NavfnROS/plan`,
  `/scenario_reset_begin`, `/scenario_reset`, `/scenario_finished`,
  `/pedsim_simulator/pedsim_agents_data`, `/rto/gap_selection_cursor`,
  `/rto/traj_creation_point`, `/rto/manual_traj_selection`,
  `/rto/manual_gap_sample`, `/rto/curr_exec_dg_traj`. No markers.
  `~extra_topics` adds more. Latched topics (map) are repeated into every
  split (`--repeat-latched`).
- Topics are recorded at their native rates (I.10); only
  `manual_gap_sample` is tick-aligned. Raw odom is kept so waypoint labels can
  be built later.
- **Checks** free disk every 30 s (warning below `min_free_gb`, error below a
  fifth of it, error if rosbag exited). Stops rosbag with SIGINT on shutdown,
  SIGHUP or SIGTERM, and rosbag dies with the recorder (`PR_SET_PDEATHSIG`),
  so no `.bag.active` files are left.

Launch arguments: `data_root` (required), `session`, `namespace` (rto),
`rviz_config`, `split_size_mb` (2048), `min_free_gb` (5), `operator`,
`instruction`.

### The `arena_evaluation` recorder is separate

`arena_evaluation/scripts/data_recorder_node` is Arena's older evaluation
logger, not part of IL recording. It keeps the latest `scan`, `odom`,
`cmd_vel` and pedsim message and, on `/clock`, writes one CSV row per topic
every 400 ms of sim time (`record_frequency: 400`), new message or not
(sample and hold; `msg_time` marks stale rows, fix 08). `metrics.py` reads
these CSVs for success, collisions and path length; the 80% / 50–60% figures
came from it. At 2.5 Hz, with no gaps, cursor or choice, it cannot produce IL
samples.

## II.7 Checks before and during a session

**Before a long session** (protocol section 3; replaces plan 1.10, which
predates fixes 15 and 40). Record a 2-minute test session, run
`il_tools/il_check_session` on it, then check:

- [ ] All automatic checks pass; `usable_ticks` is most of `cursor_ticks`;
      `ticks_with_choice` > 0.
- [ ] Samples arrive at ~9–10 Hz with non-empty `candidates` and a goal.
- [ ] Playback with `il_replay_viewer` shows the selected path where the robot
      went.
- [ ] Mouse held still for 10 s: the selection stays (no hold-still ticks).
- [ ] Tool switched off: the robot stops within 0.5 s.
- [ ] Pointer moved off the map: the robot keeps the last selection; those
      ticks have `cursor_live = false`.
- [ ] After a reset, `episode_id` increments and ticks within ~1 s of it have
      `in_reset_window = true`.
- [ ] A click-to-move produces `SOURCE_CLICK_TO_MOVE` ticks.
- [ ] `cursor.header.frame_id` is `map`; the planner logs no clock warning
      ("is RViz on sim time?").

**At launch:**
- Arguments include `man_gap:=true`, `record_il:=true` and `il_data_root`.
  Without `il_data_root` the required recorder exits and the launch stops.
- With a benchmark, wait for the relaunch and check it carries
  `record_il:=true` and `il_session:=<runid>_<stage>`.
- The session folder exists, `session_0.bag.active` grows and `rosbag record`
  runs.
- `versions.yaml`: dynamic_gap at the branch head and not dirty.
- Optional: listen to `/rto/manual_gap_sample` for 5 s and count
  `(status, source, hold_still)`. `status = COLLIDING` on every tick means the
  robot spawned in collision.

**While running:** RViz exiting with code -11 is a known segfault (II.12).
The sim and recorder keep running, but no new cursor messages arrive. A
`REQUIRED process has died` line ends the launch; the reason is in
`~/.ros/log/<run_id>/`.

**After the session:** `il_check_session <session>` and
`.claude/skills/monitor-il-run/review_session.py <session>`. The second adds
per-episode scenario matching (all episodes matching `0.json` means the stage
config was not applied, fix 39), goal consistency, rate and speed sanity,
ticks with nonzero `cmd_vel` but no selection (must be 0, fix 37), unusable
ticks by cause, and gap switches with the mouse still.

The `monitor-il-run` Claude Code skill runs these checks around a live
session.

## II.8 Offline processing

Tools in `il_tools/` (doc: `il_pipeline_fixes/il-offline-tools.md`).

### `il_check_session SESSION_DIR [...]`
- Bags present, no `.bag.active`.
- `tick_seq` contiguous (a planner restart is reported, not failed).
- Replaying the selection rule reproduces `selected_idx`
  (`il_bags.selection_mismatches`): nearest feasible terminal pose for
  sessions before fix 43; from fix 43 on, the direction cone and the sticky
  switch with the parameters from `params.yaml`, the followed gap replayed
  tick by tick. A choice within 2° of the cone edge or 0.02 of the switch
  ratio counts as reproduced (the robot pose is recorded at publish, not at
  selection).
- One clock: on ticks with a fresh cursor (< 0.5 s wall), cursor stamp within
  5 s of the sample stamp; stamps do not go backwards within an episode.
- `episodes.csv` present; every episode has samples and an outcome.
- Report in decisions, not ticks: ticks, usable label ticks, decisions
  (identity changes between usable ticks with mouse movement), ticks with
  more than one feasible candidate, click-to-move and hold-still ticks,
  episode results.

**Episode outcome** (`il_bags.episode_outcomes`): `GOAL_REACHED` if the goal
at the last tick is within the goal radius plus one 0.5 s goal check of
travel; `TIMEOUT` if the episode lasted the task timeout (±1 s); else
`ABORTED`. `collision` if any scan range came within robot radius + 1 cm.
Duration comes from `episodes.csv`, because the planner can stop ticking
before the episode ends.

### `il_build_dataset --out DIR [options] SESSION_DIR [...]`

Writes `train/val/test.jsonl.gz` and `dataset.yaml` (options, per-session
drop counts, reports). Ticks are not hand-picked: every usable label tick is
kept unless a filter drops it (`il_dataset.build_examples`):

| Step | Option (default) | Drops |
|---|---|---|
| Usable label | — | Ticks that are not `is_usable_label` |
| Thresholds | `--max-cursor-dist` (∞), `--max-bearing-diff` (∞), `--min-tie-margin` (0) | Unclear choices: cursor far from the selection, off in bearing, or near tie. Defaults keep everything until the pilot sets them. |
| Collision episodes | `--keep-collision-episodes` to keep and tag | Whole episodes with a collision |
| Reaction lag | `--lag` (0 s, wall) | Observation = latest tick ≥ lag before the label tick. Dropped if > 0.25 s older than t − lag (a hole in the samples), not `PLANNED`, or in a reset window |
| Identity | — | The selected gap (by model IDs) not present in the observation tick |
| Duplicates | — | Two label ticks mapping to the same observation and label |
| Split | `--seed` | By episode, hash of session, episode, seed; 80/10/10 |

Not filtered today: **timeout episodes**. Runs where the operator hesitated
until the timeout may teach hesitation; tag them or try a build without them.

An example holds the scan, robot velocity, pose and yaw, goal and local
waypoint (robot frame), every candidate's features, DynamicGap's pick
(`dg_pick`) and the label index. Candidate features include `shown`,
`tube_len`, `is_current` (followed on the previous tick), robot-frame goal,
terminal and path, and the `dg_*` costs (None before schema 2; fix 48).

### `il_replay_viewer`
`rosbag play --clock SESSION_DIR/session_*.bag` plus
`il_tools/il_replay_viewer _namespace:=rto` draws candidate paths, the
selection (green = usable label, orange = not), the cursor (blue if live) and
a status line.

### Evaluation metrics and old data (not IL bags)

These apply to `arena_evaluation` CSV folders (II.6), used for success rates,
not to IL sessions.
- **Repair** (fix 36): `arena_evaluation/scripts/repair_recorded_data RUN_DIR
  [--dry-run] [--out DIR]` writes a repaired copy (never touches the
  original). It removes episodes duplicated across folders by the old
  recorder (fix 01) and rewrites old `time` values to nanoseconds (fix 02).
- **Velocity** (fix 32): the odometry linear velocity (vx, vy), not
  differences of positions; acceleration and jerk change with it.
- **Collisions** (fix 33): from `scan_min.csv`, the smallest range of every
  scan since the previous row, so contacts between 400 ms rows are no longer
  missed.
- **Pedestrians** (fix 34): `pedsim_agents_data.csv` has a `velocity` field and
  `theta` follows the walking direction.

## II.9 Learning side

Full reasoning: `docs/IL_MODEL_AND_DATA_DESIGN.md`.

### Two models
- **Gap-selection scorer (Alternative 1)** — what `il_build_dataset` builds
  today. Input: the scene's edges and their links (schema 4, see
  `il_pipeline_fixes/52-scene-edges.md`), each candidate's points and ground
  velocities, robot velocity, goal, local waypoint; no scan, no DynamicGap
  costs by default; output: a score per candidate. Cross-entropy over the
  tick's candidates; up-weight gap switches. Defaults: hidden 64, dropout 0.1.
  First measure how often the human disagreed with DynamicGap's pick; under
  ~10% means little to learn.
- **MoG BC over waypoints (recommended long-term)** — Mixture-of-Gaussians
  behaviour cloning: 4–6 trajectory modes (diagnostics default K = 3–5), each 8
  robot-frame waypoints 0.25 s apart (2 s horizon), tracked by DynamicGap's
  controller. Inputs: LiDAR, GRU tracks (6 nearest × x, y, vx, vy, valid,
  branch dropout 25–50%), goal, ego velocity; no previous command. Loss:
  winner-take-all NLL plus mode cross-entropy. Inference: highest-weight
  mode's mean, never the mixture mean. Then HG-DAgger, 3–5 rounds of 30–50
  episodes, interventions ~50% of each batch, 1–2 s before each takeover
  dropped.

### What the MoG route still needs
- M-02: the GRU must run online and be recorded (today only `.pt` files in
  `ml_gap_velocity/models/`).
- D8: cursor-only data or teleop. No teleop data exists; odometry waypoint
  labels from cursor runs imitate DynamicGap's generator steered by the
  human's gap choice.
- Policy rate (10 Hz in the design) is set separately from the sample rate
  (M-09).

### Sample selection and weighting
- Keep everything that passes the II.8 filters; do not hand-pick "good"
  ticks — that biases the data and removes the ordinary states the policy
  needs to stay stable.
- Most ticks are easy (one obvious gap). Down-weight them, and stationary
  samples, rather than dropping them; up-weight decision ticks (gap
  switches).
- Validation split by episode; select checkpoints by closed-loop Arena
  success, not validation loss; report 3–5 seeds.

### How much data
- Gap scorer: 30–60 min of selection logs (design doc).
- MoG BC (estimate, not from a source in this repo): about 200–400
  successful episodes, ~2–4 h, spread over maps, crowd densities and seeds;
  in line with robomimic's ~200 human demos per task.
- What counts is the number of distinct decision points (forks), not ticks.
  Too few and the mixture collapses to one mode. Check decisions per episode
  in `il_check_session`; if low, collect denser scenes rather than longer
  runs.
- HG-DAgger rounds add targeted data from the policy's own failure states,
  usually worth more per minute than more demonstrations.

## II.10 Data compatibility across versions

`SCHEMA_VERSION` is 2 since fix 48 (DynamicGap's pick and costs added); 1 from
fix B until then.
Several fixes changed what the fields mean or how the robot behaves, so
sessions from before and after them must not be pooled unmarked. Tell them
apart by the dynamic_gap (and simulation-setup) commit in each session's
`versions.yaml`, or by the params in `params.yaml`.

| Fix | Commit (date) | Changes | How to tell |
|---|---|---|---|
| A, B, 16, 17, 27, 29 | dynamic_gap `57e5709` (2026-10-21) | First sessions with `ManualGapSample`; earlier recordings have no IL sample at all | Topic present |
| 40 | dynamic_gap `94b09eb` (2026-10-22) | Cursor no longer dropped after 0.5 s without a message; selection kept while RViz is silent | Commit |
| 43 | dynamic_gap `3c2ef1c` (2026-10-22) | Selection rule: direction cone + sticky switch, instead of hover-nearest. Labels, switch rate and hold-still ticks change | `/man_gap_max_bearing_deg` and `/man_gap_switch_ratio` in `params.yaml` |
| 44 | simulation-setup `7c1f551` (2026-10-22) | Holonomic driving: `cmd_vel.linear.y` used, heading fixed. Trajectories and odometry differ | `holonomic: true` under the dgap params in `params.yaml` |
| 45 | dynamic_gap `8d75c2a` (2026-10-22) | Early-return ticks record `selected_idx = -1` instead of 0. Old sessions: filter on `status == PLANNED` | Commit |
| 48 | dynamic_gap `50e7acd`, arena-rosnav `e9054b511` (2026-10-23) | `dg_pick_idx` and per-candidate `dg_*` costs added; behaviour unchanged | `schema_version == 2` |
| 15 | simulation-setup `832aed2` | None: the rate was always 10 Hz; only the config was cleaned up | — |

Rules:
- Never pool pre-43 and post-43 labels: they come from different selection
  rules.
- Never pool pre-44 and post-44 sessions for anything that uses motion
  (waypoint labels, velocities).
- `il_build_dataset` already requires `status == PLANNED` for observation
  ticks, so pre-45 early-return ticks do not become examples. Their
  `selected_idx = 0` is still wrong in raw analysis.
- When a field is added or changed, bump `SCHEMA_VERSION`.

## II.11 Collection protocol

A template exists: `docs/IL_COLLECTION_PROTOCOL.md` (plan 1.8). The tooling
and the session and check sections are written. Still **TODO** before the
first real session:
- the instruction given to operators (reach the goal quickly? safely? like a
  person?);
- operators and their warm-up;
- session length and breaks (suggested ≤ 20 min);
- maps, scenarios and pedestrian densities to cover;
- target size in decisions per map and density;
- the RViz config: shipped `arena.rviz` or a view cut down to the model inputs
  (fix 26);
- `data_root` (D7);
- thresholds and reaction lag from the pilot.

The template's "plan defaults in effect" line still lists D2 hover-nearest and
D3 flag only. Both are superseded (fix 43, II.2.5); update it when filling in
the rest.

## II.12 Open decisions and issues

### Plan decisions D1–D7 (plan 1.2)

| # | Decision | Current state |
|---|---|---|
| D1 | Still mouse: map spot or direction from the robot? | What is under the mouse on screen, re-projected every publish (fix 29). With the robot-following view this is a fixed offset from the robot. |
| D2 | Hover-nearest or latch a gap? | **Decided:** direction cone + sticky switch (fix 43). Thresholds to be checked in the pilot. |
| D3 | Dead cursor: change behaviour or only flag? | **Overridden:** the robot holds still without a selection (fixes 16, 37); a still mouse is not a dead cursor (fix 40). |
| D4 | Record the autonomous planner's pick? | Recorded since fix 48 (`dg_pick_idx`, `dg_*` costs), computed without acting on it (M-03, D9). |
| D5 | Model inputs | Scan, robot velocity, goal, candidate set. RViz is **not** cut down to match yet (fix 26). |
| D6 | Gaps only, or also "go straight" / "wait"? | Gaps only; ungap and idling are off in both modes (fix 31). |
| D7 | `data_root` | Pilot uses `/home/abdel/il_data`; not fixed in the protocol. |

Model-design decisions D8–D11 (teleop or cursor-only, record DynamicGap's
pick, a safety-checks-on condition, which GRU) are open; see
`IL_MODEL_AND_DATA_DESIGN.md`. Its issues M-01 to M-09 are all open. Note:
M-09 states the planner tick as 5 Hz; it is 10 Hz (I.10), so the policy and
tick rates already match.

### Open issues

| Issue | Effect | Status |
|---|---|---|
| Candidate numbers display off in `arena.rviz` (`/rto/manual_candidate_markers`, `Enabled: false`) | The operator does not see numbers or the green highlight unless they turn the display on by hand. `view.rviz` copies the file at start, so a display toggled during the session is not recorded. The protocol assumes the number turns green. | New, needs a decision (enable in `arena.rviz`) |
| Operator sees ground-truth pedestrians (fix 26) | Breaks "matched" | Docs only; part of the protocol's RViz decision |
| Pedsim steps on wall time (fix 41) | Pedestrians move faster than `vmax` relative to the robot when the sim runs below real time (~2.6× at RTF 0.37) | Open; fix belongs in pedsim_ros |
| RViz segfault (exit -11, NVIDIA 470 GL) | Cursor messages stop; the last selection is held | Open, not caused by the tool; `rviz_prefix` (gdb) is not passed to benchmark relaunches |
| Spawn in collision with random starts on `map1_small` | Every tick `COLLIDING`, robot holds still | Use a benchmark suite with fixed starts |
| move_base_flex crashes (exit -11, segfault in `libmbf_abstract_server` / `libmbf_costmap_server`) | Seen at a reset, mid-episode and at a relaunch's start (2026-10-24 goal-cursor runs, 2 of 3 learned-scorer test runs on 2026-10-26). The planner stops; flatland keeps applying the last `cmd_vel`, so on straight scenarios the robot can coast into the goal and the episode looks reached. Samples and `cmd_vel` stop in the bag | Open, inside move_base_flex; stop the run and drop the episodes from the crash on; `il_goal_cursor_collect` restarts the stage |
| mbf stops calling the planner | In one fake-cursor run, 22 s before the timeout and 2.9 m from the goal, with only empty global plans. The episode has a stretch without samples. Outcome is still right (duration from `episodes.csv`) and `max_lag_error_s` drops observations across the hole | Open; mbf logs are not captured |
| Gap switches with the mouse still | Near-ties and gap ID churn (association can pass an ID to a nearby edge) | Reduced by fix 43; recheck in the pilot |
| Collisions do not end episodes (fix 23) | Ticks after a collision come from non-expert states | Docs only; collision episodes are dropped offline |
| `cursor_live` comment in `ManualGapSample.msg` says "recent" | Since fix 40 there is no age check (I.5); stale cursors are found offline from `cursor_rx_wall` | Comment to update |
| Plan 1.1 / 1.10 say the tick is 5 Hz | Wrong; 10 Hz configured, ≈9 Hz measured | Docs to update |
| `show_rviz:=false` does not turn `arena.rviz` off | — | Open (decision) |
| `PedsimMetrics` casts personal-space times to integers | Truncated seconds in metrics | Open |

---

# Appendix

## A. Fix index

One doc per fix in `docs/il_pipeline_fixes/`; status and commits in plan 3.1.
"Manual" = manual mode only; "Metrics" = `arena_evaluation` only.

| Fix | Title | Repo | Behaviour change |
|---|---|---|---|
| A, B | One candidate list and one sample per tick | dynamic_gap, arena-rosnav | Manual: `ManualGapSample` published |
| 01 | Recorder duplicates across folders | evaluation | Metrics: old folder stops receiving rows |
| 02 | Recorder timestamp scale | evaluation | Metrics: `time` in ns |
| 03 | Success never checked the goal | evaluation | Metrics: new `ABORTED` |
| 04 | Metrics episode loop stopped at the first gap | evaluation | Metrics: every episode analysed |
| 05 | `params.yaml` did not record run settings | evaluation | None (more keys) |
| 06 | Manual mode on by default | dynamic_gap | Autonomous unless `/man_gap` is true |
| 07 | No IL topics recorded | arena-rosnav | `record_il:=true` records a session |
| 08 | Startup topic discovery, stale rows | evaluation | Metrics: `msg_time` column |
| 09 | One start_goal row per episode | evaluation | Metrics |
| 10 | Double resume | arena-rosnav | None |
| 11 | Cursor timer on sim time | arena-rosnav | Cursor at 10 Hz wall, stamped at publish |
| 12 | Duplicate RViz node | arena-rosnav | None |
| 13 | Candidate markers never deleted | dynamic_gap | Manual: old numbers disappear |
| 14 | Manual-mode topics were global | dynamic_gap, arena-rosnav | Topics under `/rto/` |
| 15 | Duplicate controller rate | simulation-setup | None (10 Hz confirmed) |
| 16 | No cursor drove under obstacle avoidance | dynamic_gap | Manual: holds still without a selection |
| 17 | Cursor pick never set start time, lifespan, model IDs | dynamic_gap | Manual: bookkeeping follows the cursor pick |
| 18 | Click-to-move in autonomous mode; ended early | dynamic_gap | Manual only; ends at tolerance or 2× estimate |
| 19 | Dangerous RViz tools | dynamic_gap | Three tools removed |
| 20 | Low speed triggered "Planning Failed" | dynamic_gap | Manual: no stall check |
| 21 | `trajFlag` uninitialised on early returns | dynamic_gap | Both: `NONE` path |
| 22 | Reset begin signal | arena-rosnav | New `/scenario_reset_begin` |
| 23 | Collisions do not end episodes | arena-rosnav | Docs only |
| 24 | `scan_` newer than the gaps on collision ticks | dynamic_gap | Scan matches the gaps |
| 25 | Ground-truth pedestrian maps never cleared | dynamic_gap | Both: cleared on reset |
| 26 | Operator sees ground truth | dynamic_gap | Docs only |
| 27 | Highlight followed the numeric selector | dynamic_gap | Manual: green = cursor pick |
| 28 | Click-to-move without the lock | dynamic_gap | With 18 |
| 29 | Cursor stored a map point | arena-rosnav, dynamic_gap | Manual: re-projected each publish |
| 30 | GRU feature logger only on other branches | dynamic_gap | Docs only |
| 31 | Ungap and idling disabled | dynamic_gap | Docs only |
| 32 | Velocity metric from positions | evaluation | Metrics |
| 33 | Collisions only on 400 ms samples | evaluation | Metrics: `scan_min.csv` |
| 34 | Pedestrian heading from force | evaluation | Metrics |
| 35 | Candidate labels unreadable | dynamic_gap | Manual: white / green, 0.5 m |
| 36 | Repair script for recorded folders | evaluation | Offline script |
| 37 | Manual mode drove under planner control in collision | dynamic_gap | Manual: holds still in collision and before ready |
| 38 | Benchmark relaunch dropped IL recording | arena-rosnav | Relaunch keeps recording |
| 39 | Benchmark stages ran the default scenario | arena-rosnav | Each stage runs its own scenario |
| 40 | Cursor dropped after 0.5 s without a message | dynamic_gap | Manual: last cursor held until tool off |
| 41 | Pedestrian speed argument | arena-rosnav | New `ped_vmax`; wall-clock stepping open |
| 42 | Robot-to-goal line | dynamic_gap | Manual: yellow line in RViz |
| 43 | Directional, sticky selection | dynamic_gap, arena-rosnav | Manual: new selection rule |
| 44 | Holonomic control | simulation-setup | Both: robot strafes, heading fixed |
| 45 | Early-return `selected_idx` 0 | dynamic_gap | Recorded `-1` on early returns |
| 48 | DynamicGap's pick and costs | dynamic_gap, arena-rosnav | Recorded only (`dg_pick_idx`, `dg_*`); schema 2 |
| — | Offline tools | dynamic_gap | `il_check_session`, `il_build_dataset`, `il_replay_viewer` |

## B. System map

**Repos** (branch `imitation_learning` in each, started from `il-data-pipeline-fixes`; commits per session in
`versions.yaml`):

| Repo | Path under `src/` | IL parts |
|---|---|---|
| dynamic_gap | `planners/dynamic_gap` | Planner manual mode, `rviz/arena.rviz`, `il_tools/`, docs |
| arena-rosnav | `arena/arena-rosnav` | RViz tool, `manual_gap_msgs`, task generator, `start_trials` / `record_il` launch, `il_session_recorder` |
| arena_bringup | `arena/arena-rosnav/arena_bringup` | Nested git repo inside arena-rosnav; both track its files |
| simulation-setup | `arena/simulation-setup` | mbf configs (rates, holonomic), robot params, worlds, benchmark scenarios |
| arena-evaluation | `arena/evaluation` | CSV recorder and metrics (not IL) |

**Nodes and IL topics** (robot namespace `/rto`):

```
task_generator ──/scenario_reset_begin, /scenario_reset, goal──────────────┐
pedsim_simulator ──/pedsim_simulator/pedsim_agents_data──┐                 │
flatland ──/rto/scan (25 Hz), /rto/odom (50 Hz), /tf─────┤                 │
                                                         ▼                 ▼
RViz ContinuousGapSelectionTool ──/rto/gap_selection_cursor (10 Hz wall)──► dynamic_gap
                                └─/rto/traj_creation_point (click)────────► (in move_base_flex,
                                                                             10 Hz tick)
dynamic_gap ──/rto/manual_gap_sample, /rto/curr_exec_dg_traj──► il_session_recorder (rosbag)
            ──/rto/manual_candidate_markers, /rto/goal_arrow──► RViz
move_base_flex ──/rto/cmd_vel──► flatland
```

The recorder's topic list is in II.6. Rates and clocks are in I.10.

## C. Glossary

| Term | Meaning |
|---|---|
| Gap | Opening between two obstacle edges in the scan; left and right points tracked by estimators with stable IDs |
| Gap tube | A gap and how it evolves over the planning horizon (propagation) |
| Candidate | One gap tube with its trajectory, offered to the operator in a tick |
| Terminal point | Last pose of a candidate's trajectory; the cursor is matched against it |
| Tick | One call of `runPlanningLoop` (≈9–10 Hz); one sample, one decision |
| Episode | From one `/scenario_reset` to the next |
| Usable label | `GAP_CURSOR && cursor_live && !in_reset_window && selected_idx >= 0` |
| Decision | A change of the selected gap identity between usable ticks, with mouse movement in between |
| Hold still | Manual mode commands zero velocity because nothing is selected |
| Cursor live / held | Tool active, pick ok, tf ok; the last point is held indefinitely (fix 40) |

General terms (ROS, timing, learning) are in D.

## D. Definitions

### Time and rates

| Term | Meaning |
|---|---|
| Wall clock / wall time | The computer's real clock, the time on the wall. One wall second is one real second, whatever the simulation does. The RViz cursor timer and `wall_stamp` use it. |
| Sim time | The simulator's own clock, published on `/clock`. Nodes with `use_sim_time` read this instead of the wall clock. If the sim slows down, sim time slows down with it. |
| Real-time factor (RTF) | Sim seconds per wall second. 1.0 = real time; 0.8 = the sim runs at 80% speed. |
| Hz | Times per second. 10 Hz = every 0.1 s. |
| Stamp / timestamp | The time written into a message's header, saying when its data was valid. |
| Timer | Code that runs a function at a fixed interval. A wall timer uses the wall clock, a ROS timer uses sim time. |
| Reaction lag | Delay between what the operator saw and when their choice shows up. Offline, the label is paired with an earlier observation to undo it. |
| Sample and hold | Write the last received value on every recording step, whether or not a new one arrived. Repeated rows are stale. |

### ROS and the simulator

| Term | Meaning |
|---|---|
| ROS (Noetic) | Robot Operating System, the middleware all parts run on. Noetic is the version (Ubuntu 20.04, Python 3.8). |
| Node | One running ROS program, e.g. the planner, the recorder, RViz. |
| Topic | A named channel nodes publish messages to and subscribe from, e.g. `/rto/scan`. |
| Message (`.msg`) | The data type sent on a topic, defined in a `.msg` file, e.g. `ManualGapSample`. |
| Publish / subscribe | Send messages on a topic / receive them. |
| Callback (CB) | A function ROS calls whenever a message arrives, e.g. `laserScanCB`, `cursorCB`. |
| Latched topic | A topic whose last message is kept and given to every new subscriber, e.g. the map. |
| Namespace (`rto`) | A prefix that groups a robot's topics: `/rto/scan`, `/rto/odom`. `rto` is the robot model. |
| rosparam / `params.yaml` | Named settings on the ROS parameter server, e.g. `/man_gap`. The recorder saves all of them per session. |
| Launch file (`roslaunch`) | XML that starts a set of nodes with their arguments, e.g. `start_trials.launch`. |
| rosbag (`.bag`) | ROS's recording format: every message on chosen topics, with timestamps, replayable with `rosbag play`. `.bag.active` = a bag still open (recording not closed cleanly). |
| lz4 | A fast compression used inside the bags. |
| Split (bag) | Starting a new bag file when the current one reaches a size, `session_0.bag`, `session_1.bag`, … |
| tf | ROS's system for coordinate frames and the transforms between them over time. |
| Frame | A coordinate system. `map`: fixed world. `odom`: the robot's odometry world, smooth but drifts. `base_link` / robot frame: attached to the robot, x forward. |
| Transform | Converting a point from one frame to another, e.g. the cursor from the RViz frame to odom. |
| Odometry (odom) | The robot's own estimate of its pose and velocity from its motion. |
| LiDAR / laser scan | A sensor that measures distances in a circle of beams. Here 512 beams over 360°, range 5 m. |
| move_base_flex (mbf) | The ROS navigation framework that calls the local planner every control cycle and sends its `cmd_vel` to the robot. |
| Local planner / global planner | Local: plans the next seconds around obstacles (dynamic_gap). Global: plans the whole route on the map (NavfnROS). |
| `cmd_vel` | The velocity command sent to the robot (linear x, y and angular z). |
| Flatland / Arena | Arena (arena-rosnav) is the benchmark around a 2D simulator; Flatland is its physics/sensor sim. |
| pedsim | The pedestrian simulator that moves the people in Arena. |
| Task generator | Arena node that runs episodes: places robot and goal, resets, counts timeouts. |
| Scenario reset | The task generator starting a new episode (`/scenario_reset`). |
| RViz | ROS's 3D viewer. The operator sees map, scan, people and candidates there and moves the cursor in it. |
| RViz tool / plugin | Code loaded into RViz that handles the mouse, here `ContinuousGapSelectionTool`. |
| Ground-plane pick | Turning the mouse's screen position into a point on the floor of the 3D view. |
| Ground truth | The simulator's exact values (true pedestrian positions), which the real robot would not have. |
| Holonomic | A robot that can move sideways as well as forward. Non-holonomic (like a car) cannot. |

### Code and system

| Term | Meaning |
|---|---|
| Thread | A part of a program that runs at the same time as others. Scan callback, planning loop and cursor callback run on different threads. |
| Mutex / lock | A lock that stops two threads from using the same data at once, e.g. `gapMutex_`. |
| Deep copy | A full copy of data, so later changes to the original do not affect it. |
| `ScopeExit` | A helper that runs code whenever a function returns, by any path. Used to always publish the sample. |
| Early return | Leaving a function before the end, e.g. "not ready" in the planning loop. |
| Schema version | A number in the message that changes when its fields change, so old data can be told apart. |
| SIGINT / SIGTERM / SIGHUP | Signals that ask a program to stop: Ctrl-C / kill / closed terminal. |
| Git commit / dirty / diff | A saved version of the code / uncommitted changes exist / the text of those changes. |
| `.jsonl.gz` | Gzip-compressed text file with one JSON record (one example) per line. |
| Hash | A fixed-length fingerprint of some text; the same input always gives the same hash. Used to assign episodes to splits. |

### Planning

| Term | Meaning |
|---|---|
| Kalman filter (KF) | A classic estimator that combines a motion model with noisy measurements to track position and velocity. Tracks the gap points today. |
| Estimator / model (gap point) | The KF instance tracking one gap point; its ID is the `model_id`. |
| Association | Matching this scan's points to last scan's, so a tracked point keeps its ID. |
| Propagation | Predicting how gaps move over the next seconds. |
| Feasibility | Whether a gap can be reached and passed in time. |
| Cost | A score of how good a trajectory is (clearance, goal progress); autonomous mode takes the lowest. |
| Trajectory | A sequence of future robot poses over time. |
| Waypoint | One point on a path or trajectory. |
| Horizon | How far ahead in time a plan or prediction goes (2 s for the waypoint model). |
| Hysteresis / sticky | Keeping the current choice unless a new one is clearly better, to avoid flicker. |

### Learning

| Term | Meaning |
|---|---|
| Imitation learning (IL) | Learning a policy from an expert's (here the operator's) demonstrations. |
| Policy | The learned function from observation to action. |
| Behaviour cloning (BC) | IL as plain supervised learning: predict the expert's action from each observation. |
| Observation / label | The model's input / the correct answer for it (here the selected gap). |
| Example / sample | One (observation, label) pair. |
| Episode / demonstration | One run from start to end. |
| Gaussian / mixture of Gaussians (MoG) | A bell-shaped probability distribution / a weighted sum of several, so the output can have several peaks ("left" and "right"). |
| Mode | One peak of the distribution; one distinct option. |
| Mode collapse | All weight ends up on one mode, so the mixture acts like a single Gaussian. |
| NLL (negative log-likelihood) | Loss that measures how improbable the expert's action is under the model's distribution. |
| Cross-entropy | Loss for choosing among classes, e.g. which gap. |
| Label smoothing / soft labels | Giving a little probability to the other classes instead of 100% on one, for noisy labels. |
| Winner-take-all | Training only the mode closest to the expert's action on each example. |
| Weighting (sample weights) | Making some examples count more in the loss, e.g. gap switches or interventions. |
| Batch | The group of examples used for one training step. |
| Train / validation / test split | Data for learning / for tuning and picking checkpoints / for the final score. |
| Leakage | Near-identical data in both train and validation, which makes validation look better than it is. |
| Seed | The number that fixes random choices, so a run can be repeated. |
| Checkpoint | A saved copy of the model's weights during training. |
| Open loop / closed loop | Scored on recorded data / scored by letting the model drive in the sim. |
| Compounding error | Small policy mistakes lead to states the expert never showed, causing bigger mistakes. |
| DAgger | Dataset Aggregation: run the learner, have an expert label the states it visits, add them, retrain. |
| HG-DAgger | Human-gated DAgger: the policy drives, the human takes over when needed, and the takeovers become training data. |
| Intervention / takeover | The human taking control from the policy. |
| GRU | Gated Recurrent Unit, a neural network for sequences; here trained to estimate gap point velocities from a 15-frame history. |
| Dropout (branch dropout) | Randomly zeroing inputs or units during training so the model does not depend on them. |
| Ablation | Removing one input or part to measure how much it matters. |
| Pilot | A short trial collection to set thresholds before the real one. |
