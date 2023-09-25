### Dependencies and Installation
This planner is designed to run on ROS Noetic / Ubuntu 20.04.

First, install [ROS Noetic](http://wiki.ros.org/noetic/Installation).

Once that is completed, install the ROS Navigation stack:

``` sudo apt-get install ros-noetic-navigation```

We run this planner in the STDR Simulator (we need the second_order branch):

``` git clone -b second_order https://github.com/ivaROS/stdr_simulator.git```

We also need a custom package to allow for detection of other agents in the simulator:

``` git clone https://github.com/max-assel/modify_stdr_scan.git```

Lastly, we use the OsqpEigen library. Follow the installation instructions [here](https://github.com/robotology/osqp-eigen).

### Building

Navigate to your ROS workspace and build:

``` catkin build ```

### TODOs for Max
- Make sure all nodes die correctly
- add flag for static/dynamic to toggle propagation
- add move_base_virtual, egocircle