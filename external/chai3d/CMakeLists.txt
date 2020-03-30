#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2016-11-01 19:39:15 +0100 (Tue, 01 Nov 2016) $
#  $Rev: 2152 $


#
# configuration
#

cmake_minimum_required (VERSION 3.0.0)
project (CHAI3D)

# version info
file (READ ${PROJECT_SOURCE_DIR}/src/version VERSION)
STRING (REGEX REPLACE ".*MAJOR=([0-9]+).*" "\\1" MAJOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*MINOR=([0-9]+).*" "\\1" MINOR_VERSION "${VERSION}")
STRING (REGEX REPLACE ".*RELEASE=([0-9]+).*" "\\1" RELEASE_VERSION "${VERSION}")
set (PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${RELEASE_VERSION})

# platform detection
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (OS "win")
  if (${CMAKE_CL_64})
    set (ARCH "x64")
  else ()
    set (ARCH "Win32")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
  set (OS "lin")
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i686")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set (OS "mac")
  set (ARCH "x86_64")
endif ()

# enforce build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif()

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})

# OpenGL dependency
find_package (OpenGL REQUIRED)

# ROS Dependency
if (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      dvrk_arm
      chai_env
    )
    if (${chai_env_FOUND})
        message("-- ***FOUND CHAI ENV SUPPORT")
        add_definitions(-DC_ENABLE_AMBF_DVRK_DEVICE_SUPPORT)
        set (CHAI3D_DEFINITIONS -DC_ENABLE_AMBF_DVRK_DEVICE_SUPPORT)
    endif()
    find_package(razer_hydra)
    if (${razer_hydra_FOUND})
        message("-- ***Found Razer Hydra")
        add_definitions(-DC_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT)
        set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${razer_hydra_INCLUDE_DIRS})
        set(catkin_LIBRARIES ${catkin_LIBRARIES} ${razer_hydra_LIBRARIES})
    else()
        message("Didn't find Razer Hydra")
    endif()
    include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories (OPENGL_INCLUDE_DIR)
# on non-Windows, DHD dependency
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Windows)
  include_directories (${PROJECT_SOURCE_DIR}/../DHD/include)
  if(${CMAKE_GENERATOR} STREQUAL Xcode)
    set (DHD_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/../DHD/lib/${OS})
  else()
    set (DHD_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/../DHD/lib/${OS}-${ARCH})
  endif()
  link_directories (${DHD_LIBRARY_DIRS})
endif ()

# Windows global build options
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)

  # VisualStudio compiler
  if (MSVC)
    add_definitions (-D_CRT_SECURE_NO_DEPRECATE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP")
    if (${CMAKE_CL_64})
      add_definitions (-DWIN64)
    else ()
      add_definitions (-DWIN32)
    endif ()

  # MinGW compiler
  elseif (MINGW)
    add_definitions (-DWIN32)
    add_definitions (-DHAVE_GCC_DESTRUCTOR)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated -std=c++0x")
    set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -march=native -Wno-deprecated")
  endif ()

# Linux global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)
  add_definitions (-DLINUX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=native -Wno-deprecated -std=c++0x")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC -march=native -Wno-deprecated")

# Mac OS X global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  add_definitions (-DMACOSX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Qunused-arguments -Wno-deprecated")
endif ()


#
# static library
#

# header search path
include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/../Eigen)
include_directories (${PROJECT_SOURCE_DIR}/../glew/include)
include_directories (${PROJECT_SOURCE_DIR}/../giflib/include)
include_directories (${PROJECT_SOURCE_DIR}/../lib3ds/include)
include_directories (${PROJECT_SOURCE_DIR}/../libjpeg/include)
include_directories (${PROJECT_SOURCE_DIR}/../libpng/include)
include_directories (${PROJECT_SOURCE_DIR}/../openal/include)
include_directories (${PROJECT_SOURCE_DIR}/../openal/OpenAL32/Include)
include_directories (${PROJECT_SOURCE_DIR}/../openal/Alc)
include_directories (${PROJECT_SOURCE_DIR}/../pugixml/include)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/include/theoraplayer)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/external/ogg/include)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/external/theora/include)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/external/vorbis/include)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/external/vorbis/lib)
include_directories (${PROJECT_SOURCE_DIR}/../theoraplayer/src/Theora)

# static library source files
file (GLOB_RECURSE source RELATIVE ${CHAI3D_SOURCE_DIR} ${CHAI3D_SOURCE_DIR}/src/*.cpp ${CHAI3D_SOURCE_DIR}/src/*.h)

# static library external dependencies source files
file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../Eigen/Eigen/*)
file (GLOB_RECURSE source_gif       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../giflib/*.c       ${PROJECT_SOURCE_DIR}/../giflib/*.h)
file (GLOB_RECURSE source_glew      RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../glew/*.c         ${PROJECT_SOURCE_DIR}/../glew/*.h)
file (GLOB_RECURSE source_3ds       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../lib3ds/*.c       ${PROJECT_SOURCE_DIR}/../lib3ds/*.h)
file (GLOB_RECURSE source_jpeg      RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../libjpeg/*.c      ${PROJECT_SOURCE_DIR}/../libjpeg/*.h)
file (GLOB_RECURSE source_png       RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../libpng/*.c       ${PROJECT_SOURCE_DIR}/../libpng/*.h)
file (GLOB_RECURSE source_pugixml   RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../pugixml/*.c      ${PROJECT_SOURCE_DIR}/../pugixml/*.h)
file (GLOB_RECURSE source_openal    RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../openal/*.c       ${PROJECT_SOURCE_DIR}/../openal/*.h)
file (GLOB_RECURSE source_theora    RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../theoraplayer/*.c ${PROJECT_SOURCE_DIR}/../theoraplayer/*.cpp ${PROJECT_SOURCE_DIR}/external/theoraplayer/*.h)

# platform-specific source files adjustments
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  file (GLOB_RECURSE source_theora_exclude RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../theoraplayer/external/theora/lib/x86/*.c)
else ()
  file (GLOB_RECURSE source_theora_exclude RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../theoraplayer/external/theora/lib/x86_vc/*.c)
endif ()
list (REMOVE_ITEM source_theora ${source_theora_exclude})

# group source files (MSVC likes this)
foreach (FILE ${source} ${source_eigen} ${source_gif} ${source_glew} ${source_3ds} ${source_jpeg} ${source_png} ${source_pugixml} ${source_openal} ${source_theora})
  get_filename_component (PARENT_DIR "${FILE}" PATH)
  string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
  string (REPLACE "/" "\\" GROUP "${GROUP}")
  source_group ("${GROUP}" FILES "${FILE}")
endforeach ()

# build flags
add_definitions (${PROJECT_DEFINITIONS} -DGLEW_STATIC -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY -DAL_LIBTYPE_STATIC -D__THEORA -D_LIB)

# static library
add_library (chai3d STATIC ${source} ${source_eigen} ${source_gif} ${source_glew} ${source_3ds} ${source_jpeg} ${source_png} ${source_pugixml} ${source_openal} ${source_theora})
# library exports
set (CHAI3D_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/../Eigen" "${PROJECT_SOURCE_DIR}/../glew/include" "${OPENGL_INCLUDE_DIR}")
set (CHAI3D_LIBRARIES chai3d ${OPENGL_LIBRARIES})
# ROS CATKIN INCLUDES AND LIBRARIES
if (${CMAKE_SYSTEM_NAME} MATCHES Linux)
    set (CHAI3D_INCLUDE_DIRS "${CHAI3D_INCLUDE_DIRS}" "${catkin_INCLUDE_DIRS}")
    set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} ${catkin_LIBRARIES})
endif()
set (CHAI3D_DEFINITIONS ${CHAI3D_DEFINITIONS} ${PROJECT_DEFINITIONS})
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} drd)
  set (CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${DHD_LIBRARY_DIRS})
else ()
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} winmm)
endif ()
if (${OS} MATCHES lin)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)
elseif (${OS} MATCHES mac)
  set (CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} "-framework CoreFoundation" "-framework IOKit" "-framework CoreServices" "-framework CoreAudio" "-framework AudioToolbox" "-framework AudioUnit")
endif ()
set (CHAI3D_SOURCE_DIR ${PROJECT_SOURCE_DIR})


#
# optional extras
#

# GLFW
if (EXISTS ${PROJECT_SOURCE_DIR}/extras/GLFW)
  add_subdirectory (${PROJECT_SOURCE_DIR}/extras/GLFW)
endif ()


#
# executables
#

# examples
if (EXISTS ${PROJECT_SOURCE_DIR}/examples)
  add_subdirectory (${PROJECT_SOURCE_DIR}/examples)
endif ()

# utilities
if (EXISTS ${PROJECT_SOURCE_DIR}/utils)
  add_subdirectory (${PROJECT_SOURCE_DIR}/utils)
endif ()


#
# export package
#

# export package for use from the build tree
export (TARGETS chai3d FILE ${PROJECT_BINARY_DIR}/CHAI3DTargets.cmake)
export (PACKAGE CHAI3D)

# package definitions
set (CONF_INCLUDE_DIRS ${CHAI3D_INCLUDE_DIRS})
set (CONF_LIBRARIES    ${CHAI3D_LIBRARIES})
set (CONF_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS})
set (CONF_DEFINITIONS  ${CHAI3D_DEFINITIONS})
set (CONF_SOURCE_DIR   ${CHAI3D_SOURCE_DIR})

# package configuration and version files
configure_file (CHAI3DConfig.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfig.cmake" @ONLY)
configure_file (CHAI3DConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfigVersion.cmake" @ONLY)
