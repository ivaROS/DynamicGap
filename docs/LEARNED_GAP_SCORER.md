# Learned gap scorer in the planner

The gap scorer trained by `il_tools` (`il_model.py`, `il_train`) runs inside
the planner in C++ and replaces DynamicGap's own gap selection. Data in, choice
out: each planning tick the planner builds the tick's `ManualGapSample` (the
same message the IL recorder publishes and the training set is built from),
the scorer scores every candidate gap, and the robot follows the highest one.
It switches as soon as another gap scores higher; there is no previous-choice
input and no extra hysteresis.

DynamicGap's gap detection, propagation, feasibility, trajectory generation and
trajectory controller are unchanged. Only the choice of gap changes: `pickTraj`
and the trajectory comparison (`changeTrajectory`) are not used to drive.

## Export a model

`il_train` writes `gap_scorer.json` next to `model.pt`. For a model trained
before that:

```
il_tools/il_export_scorer /home/abdel/il_data/models/<run>
```

The file holds the feature lists, the model's inputs and flags (network,
attention, linear term), the cost caps and every weight. Any model of the
`GapScorer` family loads without code changes: attention on or off, linear term
on or off, any input groups, any layer sizes.

## Launch

```
source /home/abdel/MGS_arena_has_issues/arena_ws/devel/setup.bash
roslaunch arena_bringup start_trials.launch \
    learned_gap_scorer:=true \
    learned_gap_scorer_path:=/home/abdel/il_data/models/with_terminal_seed0/gap_scorer.json \
    [tm_modules:=benchmark benchmark_suite:=<suite>.yaml] \
    [record_il:=true il_data_root:=<data_root> il_operator:=model il_session:=<name>]
```

| Argument | Default | Meaning |
|---|---|---|
| `learned_gap_scorer` | `false` | the scorer selects the gap (params `/learned_gap_scorer`) |
| `learned_gap_scorer_path` | `""` | the exported `gap_scorer.json` (`/learned_gap_scorer_path`) |

With both `learned_gap_scorer:=true` and `man_gap:=true`, the scorer selects
and manual selection is turned off (warning at start).

After C++ or message changes, rebuild in the main workspace:
`cd /home/abdel/MGS_arena_has_issues/arena_ws && catkin build dynamic_gap rviz_plugins manual_gap_msgs`,
then check `rospack find arena_bringup` still resolves (a subset build once left
`devel/.catkin` listing only the built packages).

A model trained on data without DynamicGap's costs (schema < 2) must not carry
the linear term: in training it was zero, while the planner always has the
costs and would apply it. `il_train` now leaves it out by itself; older runs are
exported with `il_export_scorer RUN_DIR --without-dg-costs` (refused for a linear model,
which is that term alone). A training run needs non-empty train and validation
splits; with few episodes pick an `il_build_dataset --seed` that puts one in `val`.

Tested in closed loop on 2026-10-26 (pipeline test, `il_dataset_small_v1` CS1–CS4,
a model trained on one goal-cursor episode): every tick `SOURCE_MODEL`, 4 of 4 goals
reached. Two other runs lost move_base_flex to its own segfault (overview, known
issues), not related to the scorer.

## When the scorer has no usable choice: the robot holds still

With the scorer on, DynamicGap's own pick is never followed. The robot
commands zero velocity (as manual mode's `hold_still`) when:

- the model did not load: missing path, unreadable file, not a scorer export,
  or feature lists that differ from the planner's (a model exported by another
  `il_model` version). `ROS_ERROR` at start and, throttled, every tick;
- no candidate is selectable: feasible, shown, and with a finite DynamicGap
  cost. DynamicGap never executes a trajectory it costs infinite (a pose inside
  the inflated robot), so neither does the scorer;
- a score is not finite (throttled warning);
- the planner is not ready, in collision, or sees no gaps (as manual mode).

The planner's stall check does not reset the planner while the scorer holds
the robot still.

## Recording

With the scorer on, every tick publishes `manual_gap_sample` (recorded with
`record_il:=true`). Schema 3 adds `model_score` per candidate (NaN when the
scorer did not run or the candidate is not selectable).

| Tick | `source` | `selected_idx` | `hold_still` |
|---|---|---|---|
| scorer chose | `SOURCE_MODEL` | the chosen candidate | false |
| scorer ran, no usable choice | `SOURCE_MODEL` | -1 | true |
| model not loaded | `SOURCE_NONE` | -1 | true |

`dg_pick_idx` still records DynamicGap's own pick, for comparison. Model ticks
are not training labels (`il_bags.is_usable_label` needs the cursor);
`il_bags.session_report` (in a dataset's `dataset.yaml`) counts them as `model_ticks`.

## Evaluate on held-out scenarios (time to goal, collisions, video)

`il_tools/il_eval_model` runs one or more models in closed loop on a benchmark suite and
records the RViz window. Default suite `il_eval_heldout_v1.yaml`: the gru_learn scenarios
left out of `il_dataset_small_v1` (50 pedestrians, middle starts; v3, v5, v7, v9, v10, v11
and smartgap 11), one episode each, timeout 120 s.

```
source ~/il_overlay_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312        # a port no other run uses
il_tools/il_eval_model --model full=/home/abdel/il_data/models/il_small_v1_full_seed0 \
    --model nocoll=/home/abdel/il_data/models/il_small_v1_no_collisions_seed0 \
    --out /home/abdel/il_data/evals/<name>
il_tools/il_eval_model --report-only --out /home/abdel/il_data/evals/<name>   # metrics and videos again
```

Per model, one benchmark launch with `learned_gap_scorer:=true` and IL recording, on a
virtual display of its own (Xvfb, software OpenGL; needs `sudo apt install xvfb`) that
ffmpeg records, so nothing appears on the desktop. RViz's "Noetic end-of-life" dialog is
switched off (`DISABLE_ROS1_EOL_WARNINGS`). It refuses a port in use and stops only the
processes it started (`il_launch.py`, shared with `il_goal_cursor_collect`).

Writes `<out>/<model>/`: `screen.mp4` (the whole run), `videos/epNN_<stage>.mp4` (each
episode, cut by `episodes.csv` wall times, 1 s before to 1 s after), `metrics.csv`,
`run.yaml`, `sessions/` (the IL session) and `launch.log`; `<out>/summary.csv` has one row
per model. Metrics per episode (`il_eval.py`): outcome, time to goal (if reached), path
length, contacts (runs of `STATUS_COLLIDING` ticks), colliding ticks and contact time,
closest scan reading, closest pedestrian (centre distance), the longest planner stall. An
episode in which the planner did not tick for more than 2 s (move_base_flex stalled or
crashed) is `valid: false` and left out of the summary: the robot then coasts or stands.
The benchmark writes `task_generator_<port>.yaml`, `resume_<port>.lock`, its log and an
`arena_evaluation/data` folder, as any benchmark run on that port.

## Inputs: same as in training

`LearnedGapScorer::featurize` computes `il_model.featurize` on the sample: gap
edge points from range and angle, width, bearing, the edge estimators' position
and velocity, the trajectory's end point converted to the robot frame at
`robot_pose_odom` (as `il_dataset.py` does), robot velocity, goal and local
waypoint, and DynamicGap's two cost terms with the same caps. The robot state
is taken when the scorer runs and the recorded sample keeps those values.

A model whose feature lists differ from the planner's is refused, not run.
Changing `il_model.CANDIDATE_FEATURES`/`CONTEXT_FEATURES` or `featurize` needs
the same change in `LearnedGapScorer.cpp`.

## Runtime and tests

The forward pass is native Eigen (no ONNX Runtime or LibTorch): the network is
small (tens of thousands of weights, a few candidates per tick), Eigen is
already a dependency, and the export is read with yaml-cpp (JSON is YAML).

- `test/test_learned_gap_scorer.cpp`: the C++ inputs, scores and choice against
  `il_model` on the fixtures in `test/fixtures/` (random models with attention,
  without attention and a subset of inputs, and the linear model), refusal of
  foreign models, and the planner holding still without a usable choice.
- Fixtures: `il_tools/il_scorer_fixtures` (rewrite after changing `il_model`).
  To check a trained model too:
  `il_scorer_fixtures --model <run>/gap_scorer.json --name mine --out /tmp/fx`
  then run the test with `LEARNED_GAP_SCORER_FIXTURES=/tmp/fx`.
