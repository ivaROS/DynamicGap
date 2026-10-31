# Scope and provenance

This is the basic GRU gap-sector density implementation with RViz visuals, from
`fda9d417b2144abb52f4b2199ff7f9c94109f5e3`, not the current checkout. The first density
commit is `d5f2ac9`; its parent `5910442` is the density patch base. History then adds
angular membership, raw-point visualization, dynamic-only density, CSV collection,
simulator ground-truth labels, six-feature training, inference, text visualization,
trajectory cost integration, and the idling exemption. Stopping at `fda9d41` avoids
the subsequent multi-trajectory merge and `5c28ecf` density-change/aspect-ratio work.
The user-supplied `89b23d9` baseline predates the prerequisite GRU velocity work;
see comparison_to_pre_gru.txt for the distinction. No stash or uncommitted content
is part of this bundle. All paths below are relative to final_files/ unless stated.

# Behavior and data flow

1. Planner scan processing detects raw gaps, simplifies them, associates endpoint
   models across time, and updates estimates. `scanIdxInsideGapRange()` tests an
   inclusive, wrapping angular scan-index interval from right to left.
2. `computeSimplifiedGapSectorDensity()` walks BOTH endpoints of every raw gap
   contained in the simplified gap interval. Radius is the farthest contained
   endpoint, including static points. Dynamic count uses endpoint speed after
   `Estimator::isolateGapDynamics()`, with speed >= 0.05 m/s. It counts endpoints,
   not unique pedestrians. The simplified gap boundary points are included.
3. Circular index width times scan angle increment gives theta (radians).
   Sector area A = 0.5*r*r*theta (m²), density = dynamic endpoint count / A.
   Invalid indices, missing scan/gap, or nonpositive geometry return zero-valued
   initialized results. `gapPointDensityRadius_ = 3.0` is a historical field;
   this sector calculation uses the farthest endpoint rather than that radius.
4. `computeSimplifiedGapGroundTruthSectorDensity()` uses the same geometry but
   classifies endpoints by the nearest simulator pedestrian, with match distance
   strictly less than 0.6 m and true speed >= 0.05 m/s. This is a ground-truth
   classification of observed endpoints, not a count of all simulator pedestrians.
5. `updateModel()` writes sector fields alongside the existing velocity CSV row.
   `publishGapFeatureObservation()` publishes only the LEFT simplified endpoint's
   row so each sector is observed once. Its model ID is the temporal identity;
   gap_index is useful for display but is not a stable tracking key. The midpoint
   of the simplified gap is the visualization anchor.
6. `gru_gap_feature_node.py` buffers samples by `(model_id, side)`, retains only
   side `left`, normalizes in the ordered feature list from the stats JSON,
   evaluates a TorchScript GRU on [1, seq_len, 6], and denormalizes the output.
   The target is current `gt_sector_density`, not future occupancy or a forecast.
7. `TrajectoryEvaluator::gruGapFeatureDensityCB()` validates output name/value
   sizes, the valid flag, exact output name `gt_sector_density`, and finiteness,
   then stores estimates by model ID under a mutex.
8. `evaluateTrajectory(..., scanIdx, densityModelID)` preserves posewise obstacle
   cost and adds `5.0 * max(0, prediction)` once to the existing terminal goal cost.
   The caller combines terminal cost with mean posewise cost. A fresh lookup is
   made each evaluation, including reevaluation of a reduced running trajectory.
   The new constructor takes `ros::NodeHandle&` in addition to configuration.
   `getGapLeftModelIDForDensityCost()` selects a gap's left model. Idling and
   trajectories without an applicable gap use -1 to omit learned density cost.

# Interfaces and configuration

| Interface | Reference behavior |
| --- | --- |
| GapFeatureObservation.msg | Header, gap_index, model_id, side, midpoint vis_x/vis_y, parallel named feature/target arrays |
| GapFeaturePrediction.msg | Copied observation header/identity, parallel named outputs, valid, seq_len_used |
| Observation publisher | Relative `gap_feature_observation`, normally `/rto/gap_feature_observation` |
| Prediction consumer | C++ hard-coded absolute `/rto/gru_gap_feature_prediction`, queue 100 |
| Contained point markers | Relative `contained_raw_gap_points`, normally `/rto/contained_raw_gap_points` |
| GRU text markers | Python configurable `~marker_topic`, set to `/rto/gru_gap_feature_prediction_markers` to match RViz |
| Simulator labels | `Planner::pedOdomCB`, pedsim_msgs/AgentStates on cfg_.ped_topic |
| Logging | Session CSV, named `dgap_<timestamp>.csv`; no service interface added |

The six inputs, in training order, are `sector_density`,
`sector_dynamic_raw_gap_point_count`, `contained_raw_gap_point_count`, `sector_area`,
`sector_angle_rad`, `sector_radius`. Output is `gt_sector_density`. Observation
messages also carry `gt_sector_dynamic_raw_gap_point_count` as a target. Names and
normalization arrays are part of the model contract; do not reorder independently.

| Setting | Reference default and location |
| --- | --- |
| Density cost enabled | true, hard-coded evaluator constructor |
| Density cost weight | 5.0, hard-coded evaluator constructor |
| Prediction maximum age | 3.0 s, hard-coded evaluator constructor |
| Dynamic threshold | 0.05 m/s, Planner.h |
| Ground-truth endpoint match threshold | 0.6 m, Planner.h |
| CSV enabled | true; stream-open failure disables logging |
| Python ~model_path, ~stats_path | Required, no defaults |
| ~seq_len | 10; must match trained sequence length |
| ~device | cpu |
| ~max_buffer_time_gap | 0.5 s |
| ~observation_topic | relative gap_feature_observation |
| ~prediction_topic | relative gru_gap_feature_prediction |
| ~marker_topic | relative gru_gap_feature_prediction_markers |
| ~text_marker_height, ~text_marker_z | 0.28 m, 0.35 m |
| Training sequence / hidden / layers | 10 / 64 / 2 |
| Training batch / epochs / learning rate | 64 / 100 / 0.001 |
| Training seed / loss | 42 / mse |

There is no density YAML or dynamic-reconfigure interface in this snapshot.
Other trajectory settings come from the destination's existing configuration;
do not copy the holonomic settings just to add density. The complete historical
configuration files are supporting context, not changes in this density patch.

# Frames, units and transformations

Gap endpoint positions and the marker anchors are expressed in cfg_.robot_frame_id
(typically `rto/base_link`). Positions and radius are metres, area m², angles radians,
speeds m/s, density dynamic endpoints/m², and timestamps ROS time. Pedestrian poses
are transformed as PoseStamped and velocities as Vector3Stamped into the robot
frame in pedOdomCB, using TF lookup at latest time (`ros::Time(0)`). Vector transforms
rotate velocity and do not translate it. The reference model loader configures
`rto/odom`, `rto/base_link`, and `rto/hokuyo_link` for `/model == rto`.

Endpoint estimator velocity is robot-relative in the existing pipeline.
`isolateGapDynamics()` compensates ego motion before the dynamic speed threshold.
The older simplified-gap learned velocity override is separate from learned density;
raw-point density in this implementation relies on the active raw endpoint estimators
(normally Kalman). Do not add ego velocity twice or replace non-holonomic state
estimation conventions with assumptions from this snapshot. Verify scan index
orientation, wrapping and sensor origin against the destination.

# Fallback and error behavior

The Python node publishes valid=false during sequence warmup; negative timestamp
increments or gaps > 0.5 s clear that track buffer. It rejects non-left observations.
Callback exceptions are throttled logs and may publish nothing. Missing required
stats keys/features and TorchScript loading errors prevent successful inference;
there is no alternative model fallback. The metadata file defines normalization
means/stds and input/output names; the network file alone is insufficient.

Missing, invalid, mismatched, nonfinite or too-old predictions add zero density
penalty; baseline goal/obstacle evaluation continues. Staleness uses ABSOLUTE time
difference from observation stamp to ros::Time::now(), so future stamps can also
be rejected. A negative finite output is clamped to zero by C++, although the text
marker can show the raw negative output. An invalid incoming message is ignored
rather than clearing an existing cached valid prediction; the old value can remain
usable until its three-second age limit. Cache/buffer entries are not comprehensively
evicted for dead model IDs. These are historical behaviors, not guarantees to copy.

No positive sector radius or angle means zero density. Absent simulator ground truth
can produce zero label counts, which does not establish an actually empty sector.
A missing CSV directory or stream error disables logging; the code does not create
the directory. Empty trajectories and some size mismatches get infinite terminal
cost; exception handlers log, and are not a comprehensive safe-output policy.

# Dependencies, build and run

ROS1/catkin, C++17, Eigen3, Boost, TF2, sensor/nav/geometry/visualization messages,
roscpp/rospy, pluginlib, base_local_planner, nav_core/mbf_costmap_core, pedsim_msgs,
and dynamic_reconfigure are existing runtime/build integration dependencies.
Use package.xml and CMakeLists.txt to inspect message_generation/message_runtime,
generate_messages and add_dependencies on exported generated targets. The package
has no dedicated Python dependency lockfile. Inference needs NumPy, PyTorch and
rospy; training additionally needs pandas. Retain destination versions/toolchain.

In a configured ROS1 workspace containing this package, build with the workspace's
catkin tooling (for example `catkin build dynamic_gap`), then source devel/setup.bash.
The bundle is a focused reference, not a complete standalone build tree. No build
or ROS runtime was executed while packaging, and no model weights are included.
Launch the destination's existing Dynamic GAP/Arena navigation setup. This historical
snapshot does not add a density-specific launch file. Check resolved topic names.
From repository root, after supplying a compatible trained model and stats:

```bash
python3 ml_gap_velocity/scripts/gru_gap_feature_node.py \
  _model_path:=models/basic_density_gap_gru.pt \
  _stats_path:=models/basic_density_norm_stats.json \
  _seq_len:=10 _device:=cpu \
  _observation_topic:=/rto/gap_feature_observation \
  _prediction_topic:=/rto/gru_gap_feature_prediction \
  _marker_topic:=/rto/gru_gap_feature_prediction_markers
```

Paths above are example artifact names; use the filenames emitted by training.
The CSV directory in the complete Planner snapshot has been made repository-relative
for portability. Original code used a developer-home workspace path. This is the
only source-content privacy normalization; see portability_redactions.json. Choose
a writable destination log location during the approved port. The patch does not
change that pre-existing path in the recorded base and contains no personal paths.

Collect representative CSV rows with ground-truth pedestrian topics and valid TF,
then train from repository root:

```bash
python3 ml_gap_velocity/scripts/train_gap_gru.py \
  --csv ml_gap_velocity/data/session.csv \
  --model-dir ml_gap_velocity/models --run-name basic_density \
  --seq-len 10 --hidden-size 64 --num-layers 2 --epochs 100 --cpu
```

Training emits a traced TorchScript best model, normalization statistics with named
features/outputs, configuration and loss history. It filters to left rows, groups by
model_id, orders by sample_idx and uses a sliding sequence ending at the current
label. Statistics are computed over the dataset before random train/validation
splitting. Overlapping windows can cross the split, so validation can be optimistic;
training does not reproduce the runtime timestamp-gap reset. Do not concatenate
sessions with reused model IDs without accounting for session identity. The older
`test_gap_gru.py` and plotting script expect velocity inputs [x,y], not this density
model, and are intentionally omitted. There are no dedicated density unit tests in
this snapshot; validate normalization parity, fallback, model identity, cost scaling,
frames, and end-to-end navigation in the destination.

# Debugging and RViz

Open the bundled arena.rviz for the historical view or add the two density displays
to the destination RViz config. Both are visualization_msgs/MarkerArray displays:

- `/rto/contained_raw_gap_points`: per-gap palette cycling through six colors;
  raw LEFT endpoints are sphere lists, raw RIGHT endpoints are cube lists,
  scale 0.18 m, z=0.15 m. Each publication starts with DELETEALL on this marker
  topic, then publishes current lists, even if empty. IDs are simplified gap indices.
- `/rto/gru_gap_feature_prediction_markers`: green TEXT_VIEW_FACING markers at
  each gap midpoint, namespace `gru_sector_density_text`, showing gap index,
  GRU density and GT density (or N/A). Lifetime is 0.35 s. IDs are a Python hash
  of model_id/side, stable only within a process and potentially colliding.

The patch adds only those displays. The complete historical RViz snapshot also
contains camera/layout preferences and other displays, including an unused empty
Odometry display; these are not porting requirements. RViz TF/fixed-frame settings
must match the destination. Model text appears only after valid inference; raw
endpoint markers do not require a loaded GRU. C++ named logs include
GRUGapFeatureDensityCost and TrajectoryEvaluator; Python logs input/output feature
names, loading, callback errors and throttled predicted-vs-GT values.

# Limitations and destination-specific porting

This is density correction from recent sector measurements, not learned velocity
replacement, unique-person density estimation, a future-density predictor, or a
collision-avoidance guarantee. Sector endpoint counting includes boundaries and
may count two endpoints associated with one pedestrian. Kalman noise can misclassify
static points. Learned density defaults to zero extra penalty when unavailable.
Long prediction-age tolerance and scalar cost weight were experiment settings, not
validated universal defaults. Ground truth is simulation-dependent; deployment
must distinguish unavailable labels from true zero density.

The source comment says cost-driven trajectory switching was already commented
out. Adding a cost does not necessarily cause immediate switching to a lower-cost
trajectory. Preserve destination switching policy and explain its practical effect.
The running trajectory's model identity and the incoming candidate's identity must
remain distinct. With multiple destination trajectories per gap, carry the same
sector's stable left model ID to EVERY candidate and exclude sentinel IDs. The later
multi-trajectory merge fixes are outside this basic historical snapshot.

Do not port holonomic trajectory generation, idling restoration, ungap policy,
GRU velocity override, controller behavior or unrelated logging as new features.
The patch includes idling integration because it establishes the -1 density exemption;
adapt that exemption to existing destination idling behavior. Keep its non-holonomic
kinematics, turning constraints, CBF/social costs, collision tests and cost scales.
Make namespaces configurable where the destination requires it; avoid hard-coding
/rto into a multi-robot system. Compare destination build generation against the
pre-GRU-baseline prerequisite notes before assuming generated messages already exist.

Excluded experiments: aspect ratio, density change/rate and future-density targets;
post-snapshot multi-trajectory work; MPPI and its performance report; disabled gap
tubes/idling experiments in the current checkout; boundary-exclusion/revert trials;
backup src/delete_Planner.cpp; inert scan debug loop; empty --argument files;
editor settings/locks; all maps, plots, CSV data, weights, build outputs and stashes.
No source checkout files were modified to produce this reference.

Patch curation also omits the unrelated comment/layout-only rewrite of the
pre-existing computePerfectGapVelocityLabel() velocity-label function.
