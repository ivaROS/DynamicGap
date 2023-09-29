cmake_minimum_required(VERSION 3.10)
project(dynamic_gap)

add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

set(CATKIN_PACKAGE_DEPENDENCIES
  base_local_planner
  pluginlib
  dynamic_reconfigure
  # egocircle
  # geometry_msgs
  # nav_msgs
  nav_configs
  roscpp
  rospy
  modify_stdr_scan
  # std_msgs
  # benchmarking_tools
  # pips_trajectory_msgs
  dynamic_reconfigure
  # message_generation
)

find_package(catkin REQUIRED COMPONENTS
          ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

generate_dynamic_reconfigure_options(
  cfg/dg.cfg
)

##############
## messages ##
##############

add_message_files(DIRECTORY msg
  FILES
  TrajPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
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
  SYSTEM 
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/planner.cpp
  src/dynamic_gap.cpp
  src/gap_utils.cpp
  src/dynamicgap_config.cpp
  src/visualization.cpp
  src/goal_selector.cpp
  src/trajectory_scoring.cpp
  src/gap_trajectory_generator.cpp
  src/gap_manip.cpp
  src/trajectory_controller.cpp
  src/gap_associator.cpp
  src/estimation/staticEstimator.cpp
  src/estimation/rot_frame_kf.cpp
  src/gap_feasibility.cpp
  ) 

add_dependencies(${PROJECT_NAME} 
              ${PROJECT_NAME}_gencfg
              ${${PROJECT_NAME}_EXPORTED_TARGETS} 
              ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PYTHON_LIBRARIES}
	# ${OpenMP_LIBS}
	# osqp::osqp
	OsqpEigen::OsqpEigen
)


catkin_install_python(PROGRAMS
  scripts/vis_teb_obsts.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# target_include_directories(dynamic_gap PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )