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
#  $Date: 2016-05-12 16:00:10 +0200 (Thu, 12 May 2016) $
#  $Rev: 2019 $


# common settings required to build examples
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

# build all targets
foreach (example 01-bullet-cube 02-bullet-pool 03-bullet-mesh 04-bullet-tool 05-bullet-dental 06-bullet-exploration 07-bullet-articulation 08-bullet-coordination 09-bullet-vehicle 10-bullet-gripper)

  file (GLOB source ${example}/*.cpp)
  add_executable (${example} ${source})
  target_link_libraries (${example} ${CHAI3D-BULLET_LIBRARIES} ${CHAI3D_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

  # OS specific adjustments
  if (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    add_custom_command (TARGET ${example} POST_BUILD COMMAND Rez -append ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../resources/icons/chai3d.rsrc -o ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${example} COMMAND SetFile -a C ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${example} VERBATIM)
  endif ()
  
endforeach ()
