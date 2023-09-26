### Dependencies and Installation
This planner is designed to run on ROS Noetic / Ubuntu 20.04.

First, install [ROS Noetic](http://wiki.ros.org/noetic/Installation).

Once that is completed, install the ROS Navigation stack:

``` sudo apt-get install ros-noetic-navigation```

We run this planner in the STDR Simulator (we need the second_order branch):

``` git clone -b second_order https://github.com/ivaROS/stdr_simulator.git```

We also need a custom package to allow for detection of other agents in the simulator:

``` git clone https://github.com/max-assel/modify_stdr_scan.git```

Within the planner, we use the OsqpEigen library to generate trajectories. You can follow the installation instructions [here](https://github.com/robotology/osqp-eigen).

Lastly, we use the `navigation_test` repository to set up and queue experiments:

``` git clone https://github.com/max-assel/navigation_test.git```

### Building

Navigate to your ROS workspace and build:

``` catkin build ```

### Running
To run experiments, you can run

``` rosrun nav_scripts stdr_master.py```

And you can further modify experiments within `stdr_master.py`, including what planners to use, how many experiments, what maps, what obstacles, etc.

### TODOs for Max
- Make sure all nodes die correctly
- add flag for static/dynamic to toggle propagation
- add move_base_virtual, egocircle
