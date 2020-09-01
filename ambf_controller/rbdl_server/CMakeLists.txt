cmake_minimum_required(VERSION 3.0.2)
project(rbdl_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
tf
message_generation
)

add_service_files(
  FILES
  RBDLModel.srv
  RBDLDynamics.srv
  RBDLJacobian.srv
  
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rbdl_server
 CATKIN_DEPENDS roscpp
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
)


set(RBDL_HRD_FILES
  include/${PROJECT_NAME}/RBDLServer.h
)

set(RBDL_SRC_FILES
  src/RBDLServer.cpp
)
## Declare a C++ library
add_library(rbdl_server ${RBDL_SRC_FILES} ${RBDL_HRD_FILES}
)

# add_library(${PROJECT_NAME}
#   src/RBDLServer.cpp  include/${PROJECT_NAME}/RBDLServer.h
# )
add_dependencies(rbdl_server ${catkin_EXPORTED_TARGETS})

find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify libraries to link a library or executable target against
target_link_libraries(rbdl_server ${Boost_LIBRARIES} ${catkin_LIBRARIES})

set (RBDL_SERVER_INCLUDE_DIRS include)
set (RBDL_SERVER_LIBRARIES rbdl_server)

#add_executable(rbdl_server_node src/rbdl_server_node.cpp)
#target_link_libraries(rbdl_server_node RBDLServer  ${catkin_LIBRARIES})

set(SOURCE_FILES 
    src/rbdl_server_node.cpp)

add_executable (rbdl_server_node src/rbdl_server_node.cpp)
target_link_libraries (rbdl_server_node rbdl_server)
    