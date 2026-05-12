cmake_minimum_required(VERSION 3.10)
project(ambf_ral)

set(BUILD_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/ambf_ral")
make_directory(${BUILD_INCLUDE_DIR})

set(ROS1_depends geometry_msgs roscpp sensor_msgs std_msgs tf)
find_package(catkin QUIET COMPONENTS ${ROS1_depends})

if (catkin_FOUND)
  set(AMBF_ROS_VERSION 1)

  catkin_package(
    INCLUDE_DIRS include "${CMAKE_CURRENT_BINARY_DIR}/include"
    CATKIN_DEPENDS ${ROS1_depends}
  )

else ()
  find_package(ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set(AMBF_ROS_VERSION 2)
    set(AMBF_ROS_DISTRO_STRING $ENV{ROS_DISTRO})
    if ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "galactic")
      set(AMBF_ROS_DISTRO 7)
    elseif ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "humble")
      set(AMBF_ROS_DISTRO 8)
    elseif ("${AMBF_ROS_DISTRO_STRING}" STREQUAL "jazzy")
      set(AMBF_ROS_DISTRO 9)
    endif ()

    set(ROS2_depends rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)
    foreach (_dep ${ROS2_depends})
      find_package(${_dep} REQUIRED)
    endforeach ()
  endif ()
endif ()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ambf_ral_config.h.in "${BUILD_INCLUDE_DIR}/ambf_ral_config.h")

if (AMBF_ROS_VERSION EQUAL 1)
  install(
    DIRECTORY include/ambf_ral
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(
    DIRECTORY ${BUILD_INCLUDE_DIR}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

else ()
  ament_export_include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/include)
  ament_export_dependencies(${ROS2_depends})

  install(
    DIRECTORY include/ambf_ral
    DESTINATION include
  )

  install(
    DIRECTORY ${BUILD_INCLUDE_DIR}
    DESTINATION include
  )

  ament_package()
endif ()