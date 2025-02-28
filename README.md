# Dependencies and Installation

Dependency | Version |
--- | --- | 
ROS | Noetic |
CMake | 3.16 |
OSQP | v0.6.2 |
OsqpEigen | v0.8.0 |
`stdr_simulator` | Current |
`modify_stdr_scan` | Current |
`navigation_test` | Current |

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

# Building

Navigate to your ROS workspace and build:

``` catkin build ```

# Running
First, turn on the turtlebot, should hear a beep and power light should turn green.

Second, confirm that your ROS networking variables are correctly set:

```
export ROS_MASTER_URI=http://localhost:11311

export ROS_IP=localhost

export ROS_HOSTNAME=localhost
```

## Teleop
Start up turtlebot:

``` roslaunch turtlebot_bringup minimal.launch```

To start teleop:

```roslaunch turtlebot_teleop keyboard_teleop.launch```

## move_base
Make sure laser + turtlebot cord are connected to laptop

To start up turtlebot + laser + move_base/dynamic_gap, run

```roslaunch dynamic_gap move_base_hardware_test.launch```

Then, give the planner a desired goal position
