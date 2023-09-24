cmake_minimum_required(VERSION 3.10)
project(dynamic_gap)

add_compile_options(-std=c++17)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

#find_package(OpenMP REQUIRED)
#if(OPENMP_FOUND)
#  message(STATUS "OPENMP FOUND")
#  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
#  set(OpenMP_LIBS gomp)
#endif()

set(CATKIN_PACKAGE_DEPENDENCIES
  base_local_planner
  pluginlib
  dynamic_reconfigure
  # egocircle
  # geometry_msgs
  # nav_msgs
  roscpp
  rospy
  # std_msgs
  # benchmarking_tools
  # pips_trajectory_msgs
  dynamic_reconfigure
  # message_generation
)

find_package(catkin REQUIRED COMPONENTS
          ${CATKIN_PACKAGE_DEPENDENCIES}
        )

# find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

generate_dynamic_reconfigure_options(
  cfg/dg.cfg
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  # ${osqp_INCLUDE_DIR}
)

add_message_files(DIRECTORY msg
  FILES
  TrajPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  #  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  # DEPENDS OpenMP 

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
  # src/mp_model.cpp
  src/cart_model.cpp
  src/gap_feasibility.cpp
  ) 

catkin_install_python(PROGRAMS
  scripts/vis_teb_obsts.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(dynamic_gap ${PROJECT_NAME}_gencfg)
add_dependencies(dynamic_gap ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_compile_options(dynamic_gap PRIVATE ${OpenMP_FLAGS})

# target_include_directories(dynamic_gap PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

target_link_libraries(dynamic_gap
	${catkin_LIBRARIES}
	${PYTHON_LIBRARIES}
	# ${OpenMP_LIBS}
	# osqp::osqp
	OsqpEigen::OsqpEigen
)
# target_link_libraries(PRIVATE )
