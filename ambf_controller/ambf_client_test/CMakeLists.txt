cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(ambf_client_test)

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
)

catkin_package( INCLUDE_DIRS include
  LIBRARIES ambf_client_test
  CATKIN_DEPENDS ambf_msgs message_runtime
#  DEPENDS system_lib
)


# Locate GTest
find_package(GTest REQUIRED)
include_directories( include ${GLFW_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set (SOURCE_FILES
    src/ambf_client_test.cpp)

set (HDR_FILES
    include/${PROJECT_NAME}/ambf_client_test.h)

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread ambf_client)
