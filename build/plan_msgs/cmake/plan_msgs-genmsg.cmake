# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "plan_msgs: 2 messages, 2 services")

set(MSG_I_FLAGS "-Iplan_msgs:/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(plan_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" "geometry_msgs/Vector3:geometry_msgs/Quaternion:geometry_msgs/Twist:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" "geometry_msgs/Vector3:geometry_msgs/Quaternion:geometry_msgs/Twist:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:plan_msgs/RobotState"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_custom_target(_plan_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_msgs" "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:nav_msgs/Path:geometry_msgs/PoseStamped"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_msg_cpp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)

### Generating Services
_generate_srv_cpp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)
_generate_srv_cpp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
)

### Generating Module File
_generate_module_cpp(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(plan_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_cpp _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_gencpp)
add_dependencies(plan_msgs_gencpp plan_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_msg_eus(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)

### Generating Services
_generate_srv_eus(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)
_generate_srv_eus(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
)

### Generating Module File
_generate_module_eus(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(plan_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_eus _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_geneus)
add_dependencies(plan_msgs_geneus plan_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_msg_lisp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)

### Generating Services
_generate_srv_lisp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)
_generate_srv_lisp(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
)

### Generating Module File
_generate_module_lisp(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(plan_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_lisp _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_genlisp)
add_dependencies(plan_msgs_genlisp plan_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_msg_nodejs(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)

### Generating Services
_generate_srv_nodejs(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)
_generate_srv_nodejs(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
)

### Generating Module File
_generate_module_nodejs(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(plan_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_nodejs _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_gennodejs)
add_dependencies(plan_msgs_gennodejs plan_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_msg_py(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)

### Generating Services
_generate_srv_py(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)
_generate_srv_py(plan_msgs
  "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
)

### Generating Module File
_generate_module_py(plan_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(plan_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(plan_msgs_generate_messages plan_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/msg/RobotStateStamped.msg" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/Subgoal.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/arena-rosnav/utils/ros/msgs/plan_msgs/srv/MakeGlobalPlan.srv" NAME_WE)
add_dependencies(plan_msgs_generate_messages_py _plan_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_msgs_genpy)
add_dependencies(plan_msgs_genpy plan_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(plan_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(plan_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(plan_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(plan_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(plan_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(plan_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(plan_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(plan_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(plan_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(plan_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(plan_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(plan_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(plan_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(plan_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(plan_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(plan_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs)
  install(CODE "execute_process(COMMAND \"/home/az/.cache/pypoetry/virtualenvs/arena-rosnav-fVe40LB9-py3.8/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(plan_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(plan_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(plan_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(plan_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
