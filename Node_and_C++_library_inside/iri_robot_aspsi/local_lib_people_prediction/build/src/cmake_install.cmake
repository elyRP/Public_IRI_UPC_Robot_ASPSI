# Install script for directory: /home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/iridrivers" TYPE SHARED_LIBRARY FILES "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/lib/libpeople_prediction.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iridrivers/libpeople_prediction.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iridrivers/people_prediction" TYPE FILE FILES
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/iri_geometry.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_abstract.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/prediction_bhmip.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/prediction_behavior.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_sim.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iridrivers/people_prediction/random" TYPE FILE FILES "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/random/rand_gmm.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iridrivers/people_prediction/scene_elements" TYPE FILE FILES
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_elements/person_abstract.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_elements/person_bhmip.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_elements/person_behavior.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_elements/robot.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/scene_elements/person_virtual.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iridrivers/people_prediction/nav" TYPE FILE FILES
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/nav/force_reactive_robot_companion.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/nav/plan_local_nav.h"
    "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/nav/plan_local_nav_person_companion.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iridrivers/people_prediction/learning_parameters" TYPE FILE FILES "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/learning_parameters/learnninglinearregression.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/share/cmake-3.16/Modules/Findpeople_prediction.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/share/cmake-3.16/Modules" TYPE FILE FILES "/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/src/../Findpeople_prediction.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/noetic/ros_ws/src/AKP_WS/navigation/iri_robot_aspsi/local_lib_people_prediction/build/src/examples/cmake_install.cmake")

endif()

