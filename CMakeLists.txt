cmake_minimum_required(VERSION 3.10)
project(dynamic_gap)

add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

set(CATKIN_PACKAGE_DEPENDENCIES
  base_local_planner
  pluginlib
  roscpp
  rospy
  mod_tb_scan
)

find_package(catkin REQUIRED COMPONENTS
          ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

#generate_dynamic_reconfigure_options(
#  cfg/dg.cfg
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  # DEPENDS OpenMP 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/config/DynamicGapConfig.cpp
  src/gap_detection/GapDetector.cpp
  src/gap_estimation/GapAssociator.cpp
  src/gap_estimation/PerfectEstimator.cpp
  src/gap_estimation/RotatingFrameCartesianKalmanFilter.cpp
  src/gap_feasibility/GapFeasibilityChecker.cpp
  src/global_plan_management/GlobalPlanManager.cpp
  src/scan_processing/DynamicScanPropagator.cpp
  src/trajectory_generation/GapTrajectoryGenerator.cpp
  src/trajectory_generation/GapManipulator.cpp
  src/trajectory_generation/NavigableGapGenerator.cpp
  src/trajectory_evaluation/TrajectoryEvaluator.cpp
  src/trajectory_tracking/TrajectoryController.cpp
  src/utils/Utils.cpp
  src/visualization/GapVisualizer.cpp
  src/visualization/GoalVisualizer.cpp
  src/visualization/TrajectoryVisualizer.cpp
  src/Planner.cpp
  src/DynamicGapPlanner.cpp
  ) 

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PYTHON_LIBRARIES}
	OsqpEigen::OsqpEigen
)

#catkin_install_python(PROGRAMS
#  scripts/vis_teb_obsts.py 
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# target_include_directories(dynamic_gap PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})