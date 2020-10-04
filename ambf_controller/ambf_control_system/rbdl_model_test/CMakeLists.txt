cmake_minimum_required(VERSION 3.0.2)
project(rbdl_model_test)

find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  message_generation
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rbdl_model_test
#  CATKIN_DEPENDS ambf_msgs geometry_msgs message_generation tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories( include ${catkin_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})
include_directories("../rbdl_model/include/")

#set(SOURCE_FILES
#    src/ambf_client_test.cpp)

#set (HDR_FILES
#    include/${PROJECT_NAME}/ambf_client_test.h)

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

#add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
#target_link_libraries (${PROJECT_NAME} ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread ambf_client)
