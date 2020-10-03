cmake_minimum_required(VERSION 3.0.2)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(rbdl_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  message_generation
  roscpp
  tf
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
   std_msgs
   geometry_msgs
 )

catkin_package(
 INCLUDE_DIRS include ${RBDL_MODEL_INCLUDE_DIRS}

 LIBRARIES rbdl_server
 CATKIN_DEPENDS roscpp
 # DEPENDS system_lib
 # DEPENDS Eigen
)

include_directories( include ${catkin_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})
include_directories("../rbdl_model/include/")

set(SOURCE_FILES
    src/main.cpp)

set (RBDL_SERVER_HDR_FILES
    include/${PROJECT_NAME}/RBDLServer.h)

set(RBDL_SERVER_SOURCE_FILES
    src/rbdl_server_node.cpp)


find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

## Declare a C++ library
#add_library(${PROJECT_NAME} ${RBDL_SRC_FILES} ${RBDL_HRD_FILES}
#)

add_executable (${PROJECT_NAME} ${RBDL_SERVER_HDR_FILES} ${RBDL_SERVER_SOURCE_FILES})
#add_dependencies(rbdl_server $rbdl_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries (rbdl_server ${catkin_LIBRARIES} ${RBDL_MODEL_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} rbdl_model)
target_link_libraries (rbdl_server ${YAML_CPP_LIBRARIES} ${RBDL_MODEL_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${RBDL_LIBRARY} rbdl_model)

#add_executable (rbdl_main src/main.cpp)
#target_link_libraries (rbdl_main ${PROJECT_NAME})

#include_directories(include ${catkin_INCLUDE_DIRS})




#SET (RBDL_DIR "/home/shreyas/rbdl/examples/simple/")
#find_package(RBDL REQUIRED)
#include_directories(${RBDL_INCLUDE_DIRS})

#FIND_PACKAGE (yaml-cpp REQUIRED)
#FIND_PACKAGE (RBDL REQUIRED)
#FIND_PACKAGE (rbdl_model REQUIRED)
## FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)


##find_package(RBDL REQUIRED PATHS "/home/shreyas/rbdl/examples/simple/")
#include_directories(include ${YAML_CPP_INCLUDE_DIR} ${RBDL_INCLUDE_DIR} ${RBDL_MODEL_INCLUDE_DIRS})

#find_package (GLFW REQUIRED)
#find_package (Boost COMPONENTS program_options filesystem REQUIRED)
#include_directories (${GLFW_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})


#set (RBDL_SERVER_INCLUDE_DIRS include)
#set (RBDL_SERVER_LIBRARIES rbdl_server)

#set(HDR_FILES
#    include/${PROJECT_NAME}/RBDLServer.h)

#set(SOURCE_FILES
#    src/rbdl_server_node.cpp)

#add_executable (rbdl_server_node ${HRD_FILES} ${SOURCE_FILES})
#target_sources(rbdl_server_node PRIVATE ${HDR_FILES} ${SOURCE_FILES})
#target_include_directories(rbdl_server_node PUBLIC include ${RBDL_MODEL_INCLUDE_DIRS} rbdl_model)
#target_include_directories(rbdl_server_node PUBLIC ${INTERNAL_INCLUDES})

#target_link_libraries (rbdl_server_node ${PROJECT_NAME} ${YAML_CPP_LIBRARIES} ${RBDL_MODEL_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${RBDL_LIBRARY} rbdl_model)
##target_link_libraries (${PROJECT_NAME} ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ambf_client)

