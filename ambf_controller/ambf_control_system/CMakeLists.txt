cmake_minimum_required(VERSION 3.0.2)
project(ambf_control_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_model)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_server)
