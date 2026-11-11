cmake_minimum_required(VERSION 3.10)
project(dynamic_gap)

add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

set(CATKIN_PACKAGE_DEPENDENCIES
  base_local_planner
  pluginlib
  roscpp
  rospy
  pedsim_msgs
  dynamic_reconfigure
  manual_gap_msgs
)

find_package(catkin REQUIRED COMPONENTS
          ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# reads the learned gap scorer's export (JSON)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED IMPORTED_TARGET yaml-cpp)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/ControlParameters.cfg
  cfg/EstimationParameters.cfg
)

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
  src/gap_association/Associator.cpp
  src/gap_association/GapPointAssociator.cpp
  src/gap_association/GapAssociator.cpp
  src/gap_estimation/PerfectEstimator.cpp
  src/gap_estimation/RotatingFrameCartesianKalmanFilter.cpp
  src/gap_feasibility/GapFeasibilityChecker.cpp
  src/gap_propagation/GapPropagator.cpp
  src/global_plan_management/GlobalPlanManager.cpp
  src/scan_processing/DynamicScanPropagator.cpp
  src/trajectory_generation/GapTrajectoryGenerator.cpp
  src/trajectory_generation/UngapTrajectoryGenerator.cpp
  src/trajectory_generation/GapManipulator.cpp
  src/trajectory_generation/GapGoalPlacer.cpp
  src/trajectory_evaluation/TrajectoryEvaluator.cpp
  src/trajectory_tracking/TrajectoryController.cpp
  src/ungap_feasibility/UngapFeasibilityChecker.cpp
  src/visualization/GapVisualizer.cpp
  src/visualization/GoalVisualizer.cpp
  src/visualization/AgentVisualizer.cpp
  src/visualization/TrajectoryVisualizer.cpp
  src/learned_selection/LearnedGapScorer.cpp
  src/Planner.cpp
  src/DynamicGapPlanner.cpp
  src/TimeKeeper.cpp
  ) 

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PYTHON_LIBRARIES}
	PkgConfig::YAML_CPP
)

# Generated dynamic_reconfigure headers must exist before the library compiles
# (otherwise building only the test target fails on a clean build space).
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
# manual_gap_msgs headers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/test_trajflag_uninitialised.cpp
    test/test_planner_manual_mode.cpp
    test/test_manual_selection.cpp
    test/test_learned_gap_scorer.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
    # Use RPATH (not RUNPATH) so the freshly built libdynamic_gap.so is loaded even when
    # LD_LIBRARY_PATH lists an underlay workspace that has its own (older) copy.
    set_target_properties(${PROJECT_NAME}_test PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
    # fixtures written by il_tools/il_scorer_fixtures
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE SCORER_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()
endif()
