cmake_minimum_required(VERSION 3.0.2)
project(rbdl_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
FIND_PACKAGE (yaml-cpp REQUIRED)
#FIND_PACKAGE (RBDL REQUIRED)
#find_package(RBDL REQUIRED PATHS "/home/shreyas/rbdl/examples/simple/")

find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
tf
message_generation
)


add_service_files(
  FILES
  RBDLModel.srv
  RBDLForwardDynamics.srv
  RBDLInverseDynamics.srv
  RBDLJacobian.srv
  RBDLKinimatics.srv
  RBDLBodyNames.srv
  
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rbdl_server
 CATKIN_DEPENDS roscpp
 # DEPENDS system_lib
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${YAML_CPP_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})



set(RBDL_HRD_FILES
  include/${PROJECT_NAME}/RBDLServer.h

)

set(RBDL_SRC_FILES
  src/RBDLServer.cpp

)

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} ${RBDL_SRC_FILES} ${RBDL_HRD_FILES}
)

# add_library(${PROJECT_NAME}
#   src/RBDLServer.cpp  include/${PROJECT_NAME}/RBDLServer.h
# )
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES}  ${AMBF_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ambf_client)

set (RBDL_SERVER_INCLUDE_DIRS include)
set (RBDL_SERVER_LIBRARIES rbdl_server)


set(SOURCE_FILES 
    src/rbdl_server_node.cpp)

add_executable (rbdl_server_node ${SOURCE_FILES})
target_link_libraries (rbdl_server_node ${PROJECT_NAME})


add_executable (rbdl_main src/main.cpp)
target_link_libraries (rbdl_main ${PROJECT_NAME})
