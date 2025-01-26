# Install script for directory: /home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/az/arena_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE PROGRAM FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE PROGRAM FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/setup.bash;/home/az/arena_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE FILE FILES
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/setup.bash"
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/setup.sh;/home/az/arena_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE FILE FILES
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/setup.sh"
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/setup.zsh;/home/az/arena_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE FILE FILES
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/setup.zsh"
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/az/arena_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/az/arena_ws/install" TYPE FILE FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/srv" TYPE FILE FILES
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/srv/CheckPoint.srv"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/srv/CheckPose.srv"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/srv/CheckPath.srv"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/srv/FindValidPose.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/action" TYPE FILE FILES
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/action/GetPath.action"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/action/GetInterPath.action"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/action/ExePath.action"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/action/Recovery.action"
    "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/action/MoveBase.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathAction.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathActionGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathActionResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathActionFeedback.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetPathFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathAction.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathActionGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathActionResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathActionFeedback.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/GetInterPathFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathAction.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathActionGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathActionResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathActionFeedback.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/ExePathFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryAction.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryActionGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryActionResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryActionFeedback.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/RecoveryFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/msg" TYPE FILE FILES
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseAction.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseActionGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseActionResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseActionFeedback.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseGoal.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseResult.msg"
    "/home/az/arena_ws/devel/.private/mbf_msgs/share/mbf_msgs/msg/MoveBaseFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/mbf_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/az/arena_ws/devel/.private/mbf_msgs/include/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/az/arena_ws/devel/.private/mbf_msgs/share/roseus/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/az/arena_ws/devel/.private/mbf_msgs/share/common-lisp/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/az/arena_ws/devel/.private/mbf_msgs/share/gennodejs/ros/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/az/.cache/pypoetry/virtualenvs/arena-rosnav-fVe40LB9-py3.8/bin/python3" -m compileall "/home/az/arena_ws/devel/.private/mbf_msgs/lib/python3/dist-packages/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/az/arena_ws/devel/.private/mbf_msgs/lib/python3/dist-packages/mbf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/mbf_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/mbf_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs/cmake" TYPE FILE FILES
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/mbf_msgsConfig.cmake"
    "/home/az/arena_ws/build/mbf_msgs/catkin_generated/installspace/mbf_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_msgs" TYPE FILE FILES "/home/az/arena_ws/src/arena/utils/move_base_flex/core/mbf_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/az/arena_ws/build/mbf_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/az/arena_ws/build/mbf_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
