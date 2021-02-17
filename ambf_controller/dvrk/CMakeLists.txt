cmake_minimum_required(VERSION 2.8.3)
project(dvrk_ambf_extensions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  HomogenousTransform.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputeIK.srv
#   Service2.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  LIBRARIES tf_function transformations
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
