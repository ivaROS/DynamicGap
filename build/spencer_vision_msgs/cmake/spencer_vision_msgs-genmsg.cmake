# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "spencer_vision_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ispencer_vision_msgs:/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(spencer_vision_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_custom_target(_spencer_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_vision_msgs" "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" "sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_custom_target(_spencer_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_vision_msgs" "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" "spencer_vision_msgs/PersonImage:sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_custom_target(_spencer_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_vision_msgs" "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" "sensor_msgs/RegionOfInterest"
)

get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_custom_target(_spencer_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_vision_msgs" "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" "sensor_msgs/RegionOfInterest:spencer_vision_msgs/PersonROI:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_cpp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg"
  "${MSG_I_FLAGS}"
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_cpp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_cpp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(spencer_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(spencer_vision_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(spencer_vision_msgs_generate_messages spencer_vision_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_cpp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_cpp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_cpp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_cpp _spencer_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_vision_msgs_gencpp)
add_dependencies(spencer_vision_msgs_gencpp spencer_vision_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_vision_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_eus(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg"
  "${MSG_I_FLAGS}"
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_eus(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_eus(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(spencer_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(spencer_vision_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(spencer_vision_msgs_generate_messages spencer_vision_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_eus _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_eus _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_eus _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_eus _spencer_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_vision_msgs_geneus)
add_dependencies(spencer_vision_msgs_geneus spencer_vision_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_vision_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_lisp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg"
  "${MSG_I_FLAGS}"
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_lisp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_lisp(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(spencer_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(spencer_vision_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(spencer_vision_msgs_generate_messages spencer_vision_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_lisp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_lisp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_lisp _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_lisp _spencer_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_vision_msgs_genlisp)
add_dependencies(spencer_vision_msgs_genlisp spencer_vision_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_vision_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_nodejs(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg"
  "${MSG_I_FLAGS}"
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_nodejs(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_nodejs(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(spencer_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(spencer_vision_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(spencer_vision_msgs_generate_messages spencer_vision_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_nodejs _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_nodejs _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_nodejs _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_nodejs _spencer_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_vision_msgs_gennodejs)
add_dependencies(spencer_vision_msgs_gennodejs spencer_vision_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_vision_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_py(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg"
  "${MSG_I_FLAGS}"
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_py(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
)
_generate_msg_py(spencer_vision_msgs
  "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(spencer_vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(spencer_vision_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(spencer_vision_msgs_generate_messages spencer_vision_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImage.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_py _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonImages.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_py _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROI.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_py _spencer_vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/az/arena_ws/src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_messages/spencer_vision_msgs/msg/PersonROIs.msg" NAME_WE)
add_dependencies(spencer_vision_msgs_generate_messages_py _spencer_vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_vision_msgs_genpy)
add_dependencies(spencer_vision_msgs_genpy spencer_vision_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_vision_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_vision_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(spencer_vision_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(spencer_vision_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(spencer_vision_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_vision_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(spencer_vision_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(spencer_vision_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(spencer_vision_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_vision_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(spencer_vision_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(spencer_vision_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(spencer_vision_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_vision_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(spencer_vision_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(spencer_vision_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(spencer_vision_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs)
  install(CODE "execute_process(COMMAND \"/home/az/.cache/pypoetry/virtualenvs/arena-rosnav-fVe40LB9-py3.8/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_vision_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(spencer_vision_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(spencer_vision_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(spencer_vision_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
