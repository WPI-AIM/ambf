#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2021, AMBF.
#  (http://practicepoint.wpi.edu)
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
#  $Author: Adnan Munawar, Shreyas Chandra Sekhar$
#  $Date:  $
#  $Rev: 1.0$

cmake_minimum_required (VERSION 3.10)
project (ambf_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin QUIET COMPONENTS
  geometry_msgs
  ambf_msgs
  tf
  message_generation)

if (catkin_FOUND)
  add_definitions (-DROS1)
  catkin_python_setup ()
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES ambf_client_cpp
    CATKIN_DEPENDS ambf_msgs message_runtime
    #  DEPENDS system_lib
    )

  include_directories( include ${catkin_INCLUDE_DIRS})

else ()

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    add_definitions (-DROS2)
    set (ROS_VERSION 2)

    # For now, only build the Python client
    # find_package (ambf_msgs REQUIRED)
    # find_package (rclcpp REQUIRED)
    # find_package (std_msgs REQUIRED)
    # find_package (geometry_msgs REQUIRED)
    # find_package (tf2 REQUIRED)
    # find_package (rosidl_default_generators REQUIRED)
    # # Add other ROS2 dependencies as needed

    # ament_export_include_directories(include)
    # ament_export_libraries(ambf_client_cpp)
    # ament_export_dependencies(ambf_msgs rclcpp std_msgs geometry_msgs tf2 rosidl_default_generators)

    find_package (ament_cmake_python REQUIRED)
    ament_python_install_package (ambf_client
      PACKAGE_DIR ${ambf_client_SOURCE_DIR}/python/ambf_client)
    ament_python_install_package (ros_abstraction_layer
      PACKAGE_DIR ${ambf_client_SOURCE_DIR}/python/ros_abstraction_layer)

    ament_package()
  endif (ament_cmake_FOUND)
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set (ENV_HDR_FILES
  include/${PROJECT_NAME}/ambf_client.h
  include/${PROJECT_NAME}/Object.h
  include/${PROJECT_NAME}/RosComBase.h
  include/${PROJECT_NAME}/ObjectRosCom.h
  include/${PROJECT_NAME}/Actuator.h
  include/${PROJECT_NAME}/Camera.h
  include/${PROJECT_NAME}/Light.h
  include/${PROJECT_NAME}/Object.h
  include/${PROJECT_NAME}/RigidBody.h
  include/${PROJECT_NAME}/Sensor.h
  include/${PROJECT_NAME}/Vehicle.h
  include/${PROJECT_NAME}/RosComBase.h
  include/${PROJECT_NAME}/ActuatorRosCom.h
  include/${PROJECT_NAME}/CameraRosCom.h
  include/${PROJECT_NAME}/LightRosCom.h
  include/${PROJECT_NAME}/ObjectRosCom.h
  include/${PROJECT_NAME}/RigidBodyRosCom.h
  include/${PROJECT_NAME}/SensorRosCom.h
  include/${PROJECT_NAME}/VehicleRosCom.h
  include/${PROJECT_NAME}/World.h
  include/${PROJECT_NAME}/WorldRosCom.h
  include/${PROJECT_NAME}/RosComBase.h
  include/${PROJECT_NAME}/CmdWatchDog.h
  )

set (ENV_SRC_FILES
  src/ambf_client.cpp
  src/Object.cpp
  src/ObjectRosCom.cpp
  src/Actuator.cpp
  src/Camera.cpp
  src/Light.cpp
  src/Object.cpp
  src/RigidBody.cpp
  src/Sensor.cpp
  src/Vehicle.cpp
  src/ActuatorRosCom.cpp
  src/CameraRosCom.cpp
  src/LightRosCom.cpp
  src/ObjectRosCom.cpp
  src/RigidBodyRosCom.cpp
  src/SensorRosCom.cpp
  src/VehicleRosCom.cpp
  src/World.cpp
  src/WorldRosCom.cpp
  src/RosComBase.cpp
  )

if (ROS_VERSION EQUAL 1)
  # Declare a C++ library
  add_library (ambf_client_cpp ${ENV_HDR_FILES} ${ENV_SRC_FILES})
  add_dependencies (ambf_client_cpp ${catkin_EXPORTED_TARGETS})
  # Specify libraries to link a library or executable target against
  target_link_libraries (ambf_client_cpp ${catkin_LIBRARIES})
endif (ROS_VERSION EQUAL 1)

set (AMBF_CLIENT_INCLUDE_DIRS include)
set (AMBF_CLIENT_LIBRARIES ambf_client_cpp)
