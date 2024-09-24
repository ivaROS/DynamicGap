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