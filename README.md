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

MPPI is enabled by default in Dynamic Gap. To disable it, set the ROS parameter
before the planner initializes:

```bash
rosparam set /rto/move_base_flex/DynamicGapPlannerROS/mppi/enabled false
```

Dynamic Gap reads this setting during initialization, so restart the planner
after changing it.

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

### Collecting MPPI score data

Set `mppi/log_scores: true` in the planner's ROS parameters before launch.
Each planner instance writes a timestamped sample CSV under
`mppi_scores/` in its ROS run log directory. Set `mppi/score_log_dir` to
a persistent directory to retain collections independently of ROS log cleanup.
The logger flushes after every gap-planning cycle. A 30-minute run at 10 Hz
and 96 proposals per cycle can produce about 1.7 million rows per iteration.

Each row represents one sampled control sequence. Group by cycle, gap and
iteration within one file. Simulation time and endpoint IDs support joins
with episode and tracking data. Cost columns are weighted contributions to
the optimizer's total. Obstacle and h are averages over the prediction
horizon. Infeasible proposals have infinite total cost and empty component
columns. Cycles without MPPI sampling produce no rows.

Generate a compact time series with:

```sh
python3 scripts/summarize_mppi_scores.py samples_RUN.csv > gap_iterations.csv
```

Plot the mean component columns against simulation time, or plot
`h_share` (sum of h divided by sum of total cost over feasible samples)
alongside `weight_shift` (fraction of normalized sampling weight redistributed
by h). A large common penalty can raise the share without changing steering.
`winner_changed` compares the cheapest proposal with and without h.
`first_yaw_rate_delta` estimates the change in the weighted first yaw rate
in rad/s before update-fraction scaling, bounds and fallback checks. These
comparisons hold proposals, feasibility and scoring context fixed. They do
not simulate subsequent iterations, gap selection or the executed command.
