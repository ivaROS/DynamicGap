# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;nav_msgs;geometry_msgs;visualization_msgs;plan_visualization;mapping;path_search;traj_planner;plan_msgs;laser_geometry".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lintermediate_planner".split(';') if "-lintermediate_planner" != "" else []
PROJECT_NAME = "intermediate_planner"
PROJECT_SPACE_DIR = "/home/az/arena_ws/install"
PROJECT_VERSION = "0.0.0"
