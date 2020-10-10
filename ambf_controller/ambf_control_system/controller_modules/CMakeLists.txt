cmake_minimum_required(VERSION 3.0.2)
project(controller_modules)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  message_generation
  tf
  roscpp
)



FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)

include_directories( include ${catkin_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})
include_directories("../rbdl_model/include/")
include_directories("../rbdl_server/include/")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controller_modules
  CATKIN_DEPENDS ambf_msgs geometry_msgs  tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



set(SRC_FILES
   src/PDController.cpp
   src/ControllerNode.cpp)

set (HDR_FILES
   include/${PROJECT_NAME}/PDController.h
   include/${PROJECT_NAME}/ControllerNode.h)

# target_link_libraries (PDController  )

## Declare a C++ library
add_library(${PROJECT_NAME} ${SRC_FILES} ${HRD_FILES} )
