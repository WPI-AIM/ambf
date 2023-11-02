#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2021, AMBF.
#  (https://github.com/WPI-AIM/ambf)
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
#  * Neither the name of authors nor the names of its contributors may
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
#  $Author: Adnan Munawar $
#  $Date:  $

find_package(yaml-cpp QUIET)

if (NOT yaml-cpp_FOUND)
    message("-- *** YAML-CPP NOT FOUND, ADDING IT AS AN EXTERNAL PROJECT")
    include(../cmake/ExternalYamlCpp.cmake)
endif()

include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${AMBF_INCLUDE_DIRS})

file (GLOB_RECURSE source_adf_loader *.cpp *.h)

add_library(adf_loader ${source_adf_loader})

if(NOT yaml-cpp_FOUND)
  add_dependencies(adf_loader YAML_CPP_EXTERNAL_PROJECT)
endif()

target_link_libraries (adf_loader ${YAML_CPP_LIBRARIES} ${AMBF_LIBRARIES})

set(ADF_LOADER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR} PARENT_SCOPE)
set(ADF_LOADER_LIBRARIES adf_loader PARENT_SCOPE)
