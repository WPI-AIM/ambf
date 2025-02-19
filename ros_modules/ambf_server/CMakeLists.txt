#  Software License Agreement (BSD License)
#  Copyright (c) 2020-2024, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev: $

cmake_minimum_required(VERSION 3.10)
project(ambf_server)

set (BUILD_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/ambf_server")
make_directory(${BUILD_INCLUDE_DIR})

set (ROS1_depends geometry_msgs ambf_msgs tf2 message_generation)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (AMBF_ROS_VERSION 1)
  catkin_package(
    INCLUDE_DIRS include "${CMAKE_CURRENT_BINARY_DIR}/include"
    LIBRARIES ambf_server
    CATKIN_DEPENDS ${ROS1_depends}
    )
  include_directories (include ${catkin_INCLUDE_DIRS})

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (AMBF_ROS_VERSION 2)
    set (AMBF_ROS_DISTRO_STRING $ENV{ROS_DISTRO})
    if ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "galactic")
      set (AMBF_ROS_DISTRO 7) # g is seventh letter
    elseif ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "humble")
      set (AMBF_ROS_DISTRO 8)
    elseif ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "jazzy")
      set (AMBF_ROS_DISTRO 9)
    endif ()

    set (ROS2_depends rclcpp std_msgs geometry_msgs sensor_msgs diagnostic_msgs tf2 tf2_ros tf2_msgs tf2_geometry_msgs ambf_msgs)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()
    include_directories (include)
    link_directories (${ament_cmake_LIBRARY_DIRS})
  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

configure_file (ambf_ral_config.h.in "${BUILD_INCLUDE_DIR}/ambf_ral_config.h")
include_directories (${BUILD_INCLUDE_DIR}/..)

###########
## Build ##
###########

set (HDR_FILES
    include/${PROJECT_NAME}/ambf_ral.h
    include/${PROJECT_NAME}/RosComBase.h
    include/${PROJECT_NAME}/CmdWatchDog.h
    include/${PROJECT_NAME}/Actuator.h
    include/${PROJECT_NAME}/ActuatorRosCom.h
    include/${PROJECT_NAME}/Camera.h
    include/${PROJECT_NAME}/CameraRosCom.h
    include/${PROJECT_NAME}/Light.h
    include/${PROJECT_NAME}/LightRosCom.h
    include/${PROJECT_NAME}/Object.h
    include/${PROJECT_NAME}/ObjectRosCom.h
    include/${PROJECT_NAME}/RigidBody.h
    include/${PROJECT_NAME}/RigidBodyRosCom.h
    include/${PROJECT_NAME}/GhostObject.h
    include/${PROJECT_NAME}/GhostObjectRosCom.h
    include/${PROJECT_NAME}/Sensor.h
    include/${PROJECT_NAME}/SensorRosCom.h
    include/${PROJECT_NAME}/Vehicle.h
    include/${PROJECT_NAME}/VehicleRosCom.h
    include/${PROJECT_NAME}/World.h
    include/${PROJECT_NAME}/WorldRosCom.h)

set (SRC_FILES
    src/ambf_ral.cpp
    src/RosComBase.cpp
    src/Actuator.cpp
    src/ActuatorRosCom.cpp
    src/Camera.cpp
    src/CameraRosCom.cpp
    src/Light.cpp
    src/LightRosCom.cpp
    src/Object.cpp
    src/ObjectRosCom.cpp
    src/RigidBody.cpp
    src/RigidBodyRosCom.cpp
    src/GhostObject.cpp
    src/GhostObjectRosCom.cpp
    src/Sensor.cpp
    src/SensorRosCom.cpp
    src/Vehicle.cpp
    src/VehicleRosCom.cpp
    src/World.cpp
    src/WorldRosCom.cpp
    )

## Declare a C++ library
add_library(ambf_server SHARED ${HDR_FILES} ${SRC_FILES})

if (AMBF_ROS_VERSION EQUAL 1)
  add_dependencies (ambf_server ${catkin_EXPORTED_TARGETS})
  target_link_libraries (ambf_server ${catkin_LIBRARIES})

else ()
  ament_target_dependencies (
    ambf_server
    ${ROS2_depends})

  install (
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
    )

  install (
    DIRECTORY ${BUILD_INCLUDE_DIR}
    DESTINATION include
    )

  install (
    TARGETS ambf_server
    EXPORT ambf_server
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

  ament_export_targets (ambf_server HAS_LIBRARY_TARGET)
  ament_export_dependencies (${ROS2_depends})
  ament_package ()

endif()
