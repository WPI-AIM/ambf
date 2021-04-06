cmake_minimum_required(VERSION 3.0.2)
project(ambf_client)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  roscpp
  tf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ambf_client
#  CATKIN_DEPENDS ambf_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
