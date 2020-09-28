cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(ambf_client_app)

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
  LIBRARIES ambf_client_app
  CATKIN_DEPENDS ambf_msgs message_runtime
#  DEPENDS system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS})

set(SOURCE_FILES 
    src/main.cpp)

set (IK_HDR_FILES
    include/${PROJECT_NAME}/ik_test.h
    include/${PROJECT_NAME}/psm_fk.h
    include/${PROJECT_NAME}/dh.h
    include/${PROJECT_NAME}/psm_ik.h
    include/${PROJECT_NAME}/utilities.h)

set(IK_SOURCE_FILES
    src/ik_test.cpp
    src/psm_fk.cpp
    src/dh.cpp
    src/psm_ik.cpp
    src/utilities.cpp)


find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ambf_client)

add_executable (ik_test ${IK_HDR_FILES} ${IK_SOURCE_FILES})
target_link_libraries (ik_test ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ambf_client)

