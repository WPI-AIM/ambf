cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(rbdl_model_test)

find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  message_generation
  tf
  rbdl_model
)

catkin_package( INCLUDE_DIRS include
  LIBRARIES rbdl_model_test rbdl_model
  CATKIN_DEPENDS ambf_msgs geometry_msgs
  DEPENDS rbdl_model
)

include_directories(${rbdl_model_INCLUDE_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})

FIND_PACKAGE (yaml-cpp REQUIRED)
#FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories(include ${YAML_CPP_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})




include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(RBDL_MODEL_INCLUDE_DIRS)

set (HDR_FILES
    include/${PROJECT_NAME}/rbdl_model_test.h)

set(SOURCE_FILES
    src/rbdl_model_test.cpp
    src/main.cpp)



find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${YAML_CPP_LIBRARIES} pthread rbdl_model ${RBDL_MODEL_LIBRARIES})
