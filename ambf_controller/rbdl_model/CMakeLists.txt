cmake_minimum_required(VERSION 3.0.2)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(rbdl_model)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_package( INCLUDE_DIRS include
  LIBRARIES rbdl_model
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
find_package(yaml-cpp REQUIRED)
FIND_PACKAGE (RBDL REQUIRED)
#find_package(RBDL REQUIRED PATHS "/home/shreyas/rbdl/examples/simple/")
include_directories(${YAML_CPP_INCLUDE_DIR})

set(EXECUTABLE_NAME ${PROJECT_NAME})
add_executable(${EXECUTABLE_NAME})

set (HDR_FILES
    include/${PROJECT_NAME}/Utilities.h
    include/${PROJECT_NAME}/Body.h
    include/${PROJECT_NAME}/Joint.h
    include/${PROJECT_NAME}/BuildRBDLModel.h
#    include/${PROJECT_NAME}/dh.h
#    include/${PROJECT_NAME}/psm_ik.h
#    include/${PROJECT_NAME}/utilities.h
)

set(SOURCE_FILES
    src/Utilities.cpp
    src/Body.cpp
    src/Joint.cpp
    src/BuildRBDLModel.cpp
    src/main.cpp
)

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

target_sources(${EXECUTABLE_NAME} PRIVATE ${HDR_FILES} ${SOURCE_FILES})
target_include_directories(${EXECUTABLE_NAME} PRIVATE include)
target_link_libraries(${EXECUTABLE_NAME} "${RBDL_LIBRARIES}" ${YAML_CPP_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
)





#########################################################################################

#set(RBDL_API_PATH
#    /home/shreyas/softwares/rbdl
#)

##set(RBDL_INCLU_PATH
##    /home/shreyas/softwares/rbdl-orb
##)

##include_directories( include ${catkin_INCLUDE_DIRS} "${RBDL_API_PATH}/include/rbdl/Body.h")
#include_directories( include ${catkin_INCLUDE_DIRS} "${RBDL_API_PATH}")

#set (HDR_FILES
##    include/${PROJECT_NAME}/ik_test.h
##    include/${PROJECT_NAME}/psm_fk.h
##    include/${PROJECT_NAME}/dh.h
##    include/${PROJECT_NAME}/psm_ik.h
##    include/${PROJECT_NAME}/utilities.h
#)

#set(SOURCE_FILES
#    src/main.cpp
#)


#find_package (GLFW REQUIRED)
#find_package (Boost COMPONENTS program_options filesystem REQUIRED)
#include_directories (${GLFW_INCLUDE_DIRS})


##target_include_directories(${PROJECT_NAME} PUBLIC ${RBDL_INCLUDE_DIRS})
##target_compile_features(rbdl PUBLIC cxx_std_11)

#find_library(RBDL librbdl.so ${RBDL_API_PATH}/build/)
#add_executable (${PROJECT_NAME} ${HRD_FILES} ${SOURCE_FILES})
#target_link_libraries (${PROJECT_NAME} ${RBDL} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})


###find_path(RBDL_INCLUDE_PATH /home/shreyas/softwares/rbdl-orb/include/rbdl/Body.h)
##find_path(RBDL_INCLUDE_PATH NAMES Body.h PATHS /home/shreyas/softwares/rbdl-orb/include/rbdl/)
#find_path(RBDL_INCLUDE_PATH NAMES Body.h PATHS /home/shreyas/softwares/rbdl/include/rbdl/)
##find_path(RBDL_INCLUDE_PATH NAMES Body.h PATHS /home/shreyas/softwares/rbdl/)

#if (RBDL_INCLUDE_PATH)
#    target_include_directories(${PROJECT_NAME} PRIVATE ${RBDL_INCLUDE_PATH})
#endif()
