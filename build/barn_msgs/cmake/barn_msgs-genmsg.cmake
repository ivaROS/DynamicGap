# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barn_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ibarn_msgs:/home/az/arena_ws/src/planners/trail/barn_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barn_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_custom_target(_barn_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barn_msgs" "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barn_msgs
  "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barn_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(barn_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barn_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barn_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barn_msgs_generate_messages barn_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_dependencies(barn_msgs_generate_messages_cpp _barn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barn_msgs_gencpp)
add_dependencies(barn_msgs_gencpp barn_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barn_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barn_msgs
  "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barn_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(barn_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barn_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barn_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barn_msgs_generate_messages barn_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_dependencies(barn_msgs_generate_messages_eus _barn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barn_msgs_geneus)
add_dependencies(barn_msgs_geneus barn_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barn_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barn_msgs
  "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barn_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(barn_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barn_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barn_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barn_msgs_generate_messages barn_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_dependencies(barn_msgs_generate_messages_lisp _barn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barn_msgs_genlisp)
add_dependencies(barn_msgs_genlisp barn_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barn_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barn_msgs
  "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barn_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barn_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barn_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barn_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barn_msgs_generate_messages barn_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_dependencies(barn_msgs_generate_messages_nodejs _barn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barn_msgs_gennodejs)
add_dependencies(barn_msgs_gennodejs barn_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barn_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barn_msgs
  "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barn_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(barn_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barn_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barn_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barn_msgs_generate_messages barn_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/az/arena_ws/src/planners/trail/barn_msgs/msg/BARN_data.msg" NAME_WE)
add_dependencies(barn_msgs_generate_messages_py _barn_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barn_msgs_genpy)
add_dependencies(barn_msgs_genpy barn_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barn_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barn_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barn_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barn_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barn_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barn_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barn_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barn_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barn_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barn_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barn_msgs)
  install(CODE "execute_process(COMMAND \"/home/az/.cache/pypoetry/virtualenvs/arena-rosnav-fVe40LB9-py3.8/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barn_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barn_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barn_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
