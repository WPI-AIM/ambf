#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2024, AMBF.
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
#  $Author: Adnan Munawar $
#  $Date:  $
#  $Rev: $

cmake_minimum_required (VERSION 3.10)
project (ambf_msgs)

find_package (catkin QUIET COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

set (MESSAGES
  ObjectState.msg
  ObjectCmd.msg
  ActuatorState.msg
  ActuatorCmd.msg
  SensorState.msg
  SensorCmd.msg
  ContactData.msg
  ContactEvent.msg
  ContactSensorState.msg
  ContactSensorCmd.msg
  GhostObjectState.msg
  GhostObjectCmd.msg
  CameraState.msg
  CameraCmd.msg
  LightState.msg
  LightCmd.msg
  RigidBodyState.msg
  RigidBodyCmd.msg
  VehicleState.msg
  VehicleCmd.msg
  WorldState.msg
  WorldCmd.msg
  )

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
else (catkin_FOUND)
  # look for ROS2
  find_package(ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    find_package (rosidl_default_generators REQUIRED)
    find_package (std_msgs REQUIRED)
    find_package (geometry_msgs REQUIRED)
  endif (ament_cmake_FOUND)
endif(catkin_FOUND)

if (ROS_VERSION EQUAL 1)
  add_message_files (
    FILES
    ${MESSAGES}
  )
  generate_messages (
    DEPENDENCIES
    geometry_msgs
    std_msgs
  )
  catkin_package (
    CATKIN_DEPENDS message_runtime
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

else (ROS_VERSION EQUAL 2)

  set (MESSAGES_ROS2 "")
  foreach (_msg ${MESSAGES})
    list (APPEND MESSAGES_ROS2  "msg/${_msg}")
  endforeach ()

  rosidl_generate_interfaces (
    ambf_msgs
    ${MESSAGES_ROS2}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)

  ament_package ()
endif ()
