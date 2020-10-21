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
find_package(GTest REQUIRED)

include_directories( include ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})
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
 include
  ${catkin_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)



set(SRC_FILES
   src/PDController.cpp
   src/ControllerNode.cpp)

set (HDR_FILES
   include/${PROJECT_NAME}/ControllerBase.h
   include/${PROJECT_NAME}/PDController.h
   include/${PROJECT_NAME}/ControllerNode.h)

# target_link_libraries (PDController  )

## Declare a C++ library
add_library(${PROJECT_NAME} ${SRC_FILES} ${HRD_FILES} )
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

# add_executable (control_node src/control_node.cpp)
# target_link_libraries (control_node ${PROJECT_NAME} ${Eigen_LIBRARIES} ambf_client)


add_executable (PDcontroller_test src/PDtesting_controller.cpp)
target_link_libraries (PDcontroller_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GLFW_LIBRARIES} pthread )

add_executable (controlnode_test src/controlnode_testing.cpp)
target_link_libraries (controlnode_test ${PROJECT_NAME} ${AMBF_LIBRARIES} ${GTEST_LIBRARIES} ${GLFW_LIBRARIES} pthread ambf_client )


