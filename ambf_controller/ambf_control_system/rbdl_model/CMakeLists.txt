cmake_minimum_required(VERSION 3.0.2)
project(rbdl_model)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  ambf_msgs
#  geometry_msgs
#  message_generation
#  tf
)

catkin_package( INCLUDE_DIRS include
  LIBRARIES rbdl_model
#  CATKIN_DEPENDS ambf_msgs message_runtime
#  DEPENDS system_lib
)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/shreyas/rbdl/CMake/")

#SET (RBDL_DIR "/home/shreyas/rbdl/examples/simple/")
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(RBDL REQUIRED)
include_directories(${RBDL_INCLUDE_DIRS})

FIND_PACKAGE (yaml-cpp REQUIRED)
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(RBDL REQUIRED PATHS "/home/shreyas/rbdl/examples/simple/")
include_directories(include ${YAML_CPP_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})


#set(EXECUTABLE_NAME ${PROJECT_NAME})
#set(EXECUTABLE_NAME rbdl_model)


set (RBDL_MODEL_INCLUDE_DIRS include)
set (RBDL_MODEL_LIBRARIES rbdl_model)


#add_executable(${EXECUTABLE_NAME})

set (HDR_FILES
    include/${PROJECT_NAME}/Utilities.h
    include/${PROJECT_NAME}/BodyParam.h
    include/${PROJECT_NAME}/JointParam.h
    include/${PROJECT_NAME}/BuildRBDLModel.h
)

set(SOURCE_FILES
    src/Utilities.cpp
    src/BodyParam.cpp
    src/JointParam.cpp
    src/BuildRBDLModel.cpp
#    src/main.cpp
    )

## Declare a C++ library
add_library(rbdl_model ${HDR_FILES} ${SOURCE_FILES})

add_dependencies(rbdl_model ${catkin_EXPORTED_TARGETS})

#add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(rbdl_model ${catkin_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${YAML_CPP_LIBRARIES} pthread)

set (RBDL_MODEL_INCLUDE_DIRS include)
set (RBDL_MODEL_LIBRARIES rbdl_model)


#find_package (GLFW REQUIRED)
#find_package (Boost COMPONENTS program_options filesystem REQUIRED)
#include_directories (${GLFW_INCLUDE_DIRS})

#target_sources(${EXECUTABLE_NAME} PUBLIC ${HDR_FILES} ${SOURCE_FILES})
#target_include_directories(${EXECUTABLE_NAME} PUBLIC include)
#target_link_libraries(${EXECUTABLE_NAME} ${AMBF_LIBRARIES} ${YAML_CPP_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ambf_client)

#set_target_properties(${EXECUTABLE_NAME} PROPERTIES
#  CXX_STANDARD 14
#  CXX_STANDARD_REQUIRED YES
#  )

#add_dependencies(${EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS})


### Specify libraries to link a library or executable target against
#target_link_libraries(${EXECUTABLE_NAME} ${catkin_LIBRARIES})


#set (RBDL_MODEL_INCLUDE_DIRS include)
#set (RBDL_MODEL_LIBRARIES rbdl_model)

