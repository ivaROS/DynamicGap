# Dynamic Gap: Safe Gap-based Navigation in Dynamic Environments

<p align="center">
    <a href="https://arxiv.org/abs/2210.05022">arXiv</a> •
    <a href="https://www.youtube.com/watch?v=EyW7pQFC8cQ">Video</a> •
    <a href="#citation">Citation</a>

</p>

<p>
    <img align="center" width="250" src="./assets/dgap_empty.gif" alt="empty"> 
    <img align="center" width="250" src="./assets/dgap_factory.gif" alt="empty"> 
    <img align="center" width="250" src="./assets/dgap_hospital.gif" alt="empty"> 
</p>

Dynamic gap is a local motion planner designed to run in dynamic, structured environments. That means that this planner can excel in both empty regions full of moving agents as well as crowded, dense regions with corridors, obstacles, and rooms. In this proposed work, we extend the family of perception-informed gap-based planners to dynamic settings by modeling how gaps evolve over time. This gap evolution informs feasibility analysis, trajectory generation, and trajectory selection.

### MPPI default

MPPI is enabled by default inside Dynamic Gap and uses `gap_guided` mode. To disable it, set the planner's private parameter before launching or restarting the planner:

```bash
rosparam set /rto/move_base_flex/DynamicGapPlannerROS/mppi/enabled false
```

The setting is read when the planner initializes. If you load `config/mppi.yaml`, set the parameter after loading that file so the explicit opt-out takes effect. To re-enable MPPI, set the same parameter to `true` before restarting.

## <a name="Installation"></a>Installation

This planner has been tested on ROS Noetic on Ubuntu 20.04.

We implement this planner within the [Arena-Rosnav](https://arena-rosnav.readthedocs.io/en/latest/) benchmarking environment.

We forked our own version of Arena-Rosnav to solve some pre-existent problems in their repositories. In order to install our version, please visit this [repository](https://github.com/ivaROS/arena-rosnav/tree/patch) and follow the installation instructions within the README. If you encounter any problems related to installation, feel free to leave an issue on this Github and we will respond as quickly as possible.

## <a name="Quick Start"></a>Quick Start

To quickly deploy a planner with Arena-rosnav, you can select the desired planner in the `local_planner` argument and the desired map in the `map_file` argument within the `start_arena.launch` file in the `arena_bringup` ROS package:

<pre>
📦arena-rosnav  
 ┣ 📂arena-bringup
   ┣ 📂configs
   ┣ 📂launch
     ┣ 📂testing
     ┣ 📂training
     ┣ 📂utils
     ┣ <b>start_arena.launch</b>
   ┣ 📂params
   ┣ 📂rviz
   ┣ 📂scripts
 ┣ 📂task_generator
 ┣ 📂testing
 ┣ 📂training
 ┣ 📂utils
</pre>

Then, you can run
```
roslaunch arena_bringup start_arena.launch
```
## <a name="Benchmarking"></a>Benchmarking
If you would like to run more systematic tests, then you can set the `config.yaml` file to include your desired scenarios in the suite section and your desired set of planners in the contest section:

<pre>
📦arena-rosnav  
 ┣ 📂arena-bringup
   ┣ 📂configs
     ┣ 📂benchmark
       ┣ 📂contests
         ┣ <b> Set your desired set of planners here! </b> 
       ┣ 📂logs
       ┣ 📂suites
         ┣ <b> Set your desired set of scenarios here! </b> 
       ┣ <b>config.yaml</b>
     ┣ 📂parametrized
     ┣ 📂robot_setup
     ┣ 📂training
   ┣ 📂launch
   ┣ 📂params
   ┣ 📂rviz
   ┣ 📂scripts
 ┣ 📂task_generator
 ┣ 📂testing
 ┣ 📂training
 ┣ 📂utils
</pre>

Then, you can run
```
roslaunch arena_bringup start_arena.launch tm_modules:=benchmark
```
## <a name="Citation"></a>Citation
If would like to cite this work, please use the following format:
```
@article{asselmeier2024dynamicgap,
  title     ={Dynamic Gap: Safe Gap-based Navigation in Dynamic Environments},
  author    ={Max Asselmeier, Dhruv Ahuja, Abdel Zaro, Ahmad Abuaish, Ye Zhao, Patricio A. Vela},
  journal   = {arXiv},
  year      = {2024},
  month     = {September},
}
```
### Gap density with MPPI

Ground-truth density is now the default test source
(`gap_density/source: ground_truth`). It uses the same observed endpoint sector geometry and simulator
classification as the GRU training labels, adding `weight * gt_sector_density`
once per candidate. No GRU model, stats, or inference node is needed. Start before
navigation and restart the planner if it is already running:

```bash
roslaunch dynamic_gap gap_density.launch robot_namespace:=rto
```

The existing pedestrian-state topic and TF must be available. Missing GT clears
previous costs immediately at the next scan; cached GT sector values expire after
`gap_density/max_ground_truth_age` (default 0.5 s). The existing 0.5 s pedestrian
freshness check also applies. Ground-truth text appears on the raw endpoint marker
topic in RViz. This is the reference's density of observed dynamic endpoints per
sector area, rather than a count of every simulator pedestrian in the sector.

To return to learned scoring, set `gap_density/source: gru`, or launch with
`source:=gru` and the model/statistics arguments below.

### Learned density option

The density port uses the archive's six-feature model contract independently of
GRU velocity replacement. Raw endpoint Kalman estimates supply isolated obstacle
speeds; each simplified sector publishes one left-model observation. Inputs are
`sector_density`, `sector_dynamic_raw_gap_point_count`,
`contained_raw_gap_point_count`, `sector_area`, `sector_angle_rad`, `sector_radius`.
The output is current `gt_sector_density` (dynamic endpoints/m²), with no forecast
or aspect-ratio inputs. Sector boundaries are inclusive, wrapping is supported,
and static endpoints contribute to the maximum sector radius. Endpoints are
counted, so this is not a count of unique people.

The native terminal cost receives `weight * max(0, prediction)` once per gap
candidate, including every candidate associated with the same left endpoint ID.
MPPI's internal common rollout objective stays unchanged: sector density is
constant within a gap and enters its final candidate score for cross-gap selection.
The exported terminal cost includes it, so `pickTraj` preserves it when reconstructing
scores. Running-path reevaluation fetches the latest prediction by its stored left
ID. Ungap, idling and gap-free final-approach paths use the invalid ID `-1` and get
no density penalty. Existing kinematics, collision/CBF checks and switching policy
are retained. In particular, the existing lower-cost switching branch calls the
switch helper but still returns its maintained path; this port does not change
that behavior, so density alone does not guarantee an immediate trajectory switch.
Standard MPPI produces one candidate associated with its existing nearest-goal gap;
density does not create additional alternatives in that mode.

Missing, stale, invalid, nonfinite, wrong-frame or incompatible predictions add
zero cost. Invalid predictions clear the cached value immediately. Dead model IDs
are pruned, scan time rewinds clear cached predictions, and inference resets on
backward/duplicate timestamps or gaps above 0.5 s. GRU warmup publishes invalid
predictions. Density settings in [config/gap_density.yaml](config/gap_density.yaml)
load under the same **private planner namespace** as `mppi.yaml`; topics resolve
under the **robot namespace**, normally `/rto`. Defaults enable density scoring
with weight 5. GRU predictions have maximum age 3 s; GT sector values use 0.5 s.
In GRU mode, missing or unusable predictions leave baseline scoring active. Density does not require
enabling `use_gru_gap_velocity`; set that existing parameter to false for a
Kalman-velocity/density-only experiment.

No trained density model or normalization statistics are included in the reference.
After supplying a compatible TorchScript model and its stats JSON, launch this
before navigation (adjust the planner namespace to your navigation node/plugin):

```bash
source /home/azaro/arena_ws/devel/setup.bash
roslaunch dynamic_gap gap_density.launch robot_namespace:=rto source:=gru \
  planner_namespace:=move_base_flex/DynamicGapPlannerROS \
  model_path:=/absolute/path/basic_density.pt \
  stats_path:=/absolute/path/basic_density_norm_stats.json
```

The inference node runs separately on CPU by default, using one Torch thread
(`~num_threads`). `seq_len` defaults to 10 and must match the stats metadata.
Stats names, array shapes and finite values are validated; inference uses exactly
the saved normalization scales, including the trainer's epsilon. In RViz,
[rviz/arena.rviz](rviz/arena.rviz) adds raw endpoint markers and GRU-vs-GT text at
`/rto/contained_raw_gap_points` and `/rto/gru_gap_feature_prediction_markers`.
Change those display topics for another robot namespace. Marker positions and
observations use the configured robot frame, following the existing pipeline's
coincident planar laser/base origin convention.

For training collection, set `gap_density/csv_path` to a **new writable file** in
the private planner namespace before initialization, or pass `csv_path` to the
launch file. Logging is off by default and refuses to overwrite an existing file.
Simulator GT labels are published/logged only after a pedestrian-state update no
older than 0.5 s. Unavailable GT produces omitted message targets and blank CSV
labels, not zero labels. A received empty pedestrian array is valid empty GT.
GT classification matches the nearest pedestrian within 0.6 m and uses true
speed at least 0.05 m/s, with poses and velocity vectors in the robot frame.

```bash
python3 ml_gap_velocity/scripts/train_gap_density_gru.py \
  --csv /absolute/path/session.csv --model-dir /absolute/path/models \
  --run-name basic_density --seq-len 10 --hidden-size 64 --num-layers 2 --cpu
```

The existing velocity trainer is preserved. The separate density trainer drops
missing/nonfinite labels and splits sequences at timestamp reset/gap boundaries
when the CSV contains `stamp`. Use separate training sessions when model IDs may
repeat. As in the reference, overlapping windows are randomly split and statistics
use the full dataset, so reported validation loss can be optimistic; use independent
session data to assess a trained model. Live navigation and performance with trained
weights still require an Arena run; `/tmp/mppi_perf_full.json` is a pre-port baseline.

Regression checks (after building/generated messages):

```bash
catkin run_tests dynamic_gap --workspace /home/azaro/arena_ws
python3 test/test_gap_density_python.py
```
