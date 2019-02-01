#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
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


cmake_minimum_required (VERSION 3.0.0)
project (application-GLUT)

# chai3d dependency
find_package (CHAI3D 3.2 REQUIRED)
include_directories (${CHAI3D_INCLUDE_DIRS})
link_directories (${CHAI3D_LIBRARY_DIRS})
add_definitions (${CHAI3D_DEFINITIONS})

# GLUT dependency
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  find_package (FreeGLUT REQUIRED)
  include_directories (${FREEGLUT_INCLUDE_DIRS})
  set (GLUT_LIBRARIES ${FREEGLUT_LIBRARIES})
else ()
  find_package (GLUT REQUIRED)
endif ()

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

# enforce build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif()

# set output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${CHAI3D_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${CHAI3D_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${CHAI3D_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${CHAI3D_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CHAI3D_SOURCE_DIR}/bin/${OS}-${ARCH})

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
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated -std=c++0x")
  endif ()

# Linux global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux)

  add_definitions (-DLINUX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wno-deprecated -std=c++0x")

# Mac OS X global build options
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)

  add_definitions (-DMACOSX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -Wno-deprecated -std=c++0x -stdlib=libc++")

endif ()

# target executable
add_executable (${PROJECT_NAME} application.cpp)
target_link_libraries (${PROJECT_NAME} ${CHAI3D_LIBRARIES} ${GLUT_LIBRARIES})

# OS specific adjustments
if (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD COMMAND Rez -append ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../resources/icons/chai3d.rsrc -o ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} COMMAND SetFile -a C ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} VERBATIM)
endif ()
