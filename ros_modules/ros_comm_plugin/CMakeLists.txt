#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2024, AMBF.
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
#  $Rev:  $


cmake_minimum_required(VERSION 3.10)
project (ros_comm_plugin)

set (ROS1_depends ambf_server cv_bridge image_transport)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  find_package (Boost COMPONENTS filesystem REQUIRED)
  set (ROS_VERSION 1)
  include_directories(include ${catkin_INCLUDE_DIRS})
  catkin_package (
    LIBRARIES depth_streamer_plugin object_comm_plugin video_streamer_plugin world_comm_plugin
    CATKIN_DEPENDS ambf_server
    )
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp
      std_msgs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs
      ambf_msgs ambf_server
      cv_bridge image_transport)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach()

    include_directories(include)

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

###########
## Build ##
###########

find_package (AMBF REQUIRED)

add_definitions (${AMBF_DEFINITIONS})

include_directories (${AMBF_INCLUDE_DIRS})

add_library (depth_streamer_plugin SHARED DepthStreamerPlugin.cpp)
add_library (object_comm_plugin SHARED ObjectCommPlugin.cpp)
add_library (video_streamer_plugin SHARED VideoStreamerPlugin.cpp)
add_library (world_comm_plugin SHARED WorldCommPlugin.cpp)

set (ROS_COMM_PLUGINS depth_streamer_plugin object_comm_plugin video_streamer_plugin world_comm_plugin)

find_package (OpenCV QUIET)
if (OpenCV_FOUND)
  message (STATUS "-- *** FOUND OPENCV")
  if (cv_bridge_FOUND AND image_transport_FOUND)
    message(STATUS "-- *** FOUND ROS_CV_BRIDGE and IMAGE_TRANSPORT, ENABLING VIDEO PUBLISHING SUPPORT")
    add_definitions (-DAF_ENABLE_OPEN_CV_SUPPORT)
    include_directories (${OpenCV_INCLUDE_DIRS})
    ## NEED to fix this target_link_libraries (ros_comm_plugin ${OpenCV_LIBRARIES})
  else()
    message (STATUS "-- *** ROS CV_BRIDGE and IMAGE_TRANSPORT NOT FOUND")
  endif ()
else ()
  message (STATUS "-- *** OPENCV NOT FOUND")
endif ()

if (ROS_VERSION EQUAL 1)

  foreach (_plugin ${ROS_COMM_PLUGINS})
    add_dependencies (${_plugin} ${catkin_EXPORTED_TARGETS})
    target_link_libraries (${_plugin} ${catkin_LIBRARIES})
  endforeach ()

else ()

  install (
    TARGETS ${ROS_COMM_PLUGINS}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    )

  foreach (_plugin ${ROS_COMM_PLUGINS})
    ament_target_dependencies (${_plugin} ambf_server ${ROS2_depends})
  endforeach ()
  ament_package ()

endif()
