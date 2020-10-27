cmake_minimum_required(VERSION 3.0.2)
project(controller_modules_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_modules
)

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
find_package(GTest REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( INCLUDE_DIRS include
  LIBRARIES controller_modules_test controller_modules
  CATKIN_DEPENDS roscpp
  DEPENDS controller_modules
)

include_directories(${controller_modules_INCLUDE_DIRS})
include_directories( include ${GTEST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${controller_modules_INCLUDE_DIRS} )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} src/controller_manager_test.cpp)
target_link_libraries (${PROJECT_NAME} ${GLFW_LIBRARIES} ${CONTROLLER_MODULES_LIBRARIES} controller_modules)

add_executable (manager_test src/manager_test.cpp)
target_link_libraries (manager_test ${GLFW_LIBRARIES} ${CONTROLLER_MODULES_LIBRARIES} controller_modules)