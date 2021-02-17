#  Software License Agreement (BSD License)
#  Copyright (c) 2020, AMBF.
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
#  $Author: Adnan Munawar, Melody Su, Nathaniel Goldfarb, Shreyas Chandra Sekhar$$
#  $Date:  April, 2019 $
#  $Rev: 1.0$

# common settings required to build examples
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

include_directories (${GLFW_INCLUDE_DIRS}
                     raven2/include
                     dvrk/cpp_controller/include
)

add_executable (ambf_controller raven2/include/ambf_defines.h
                                raven2/include/ambf_controller.h
                                raven2/include/ambf_motion_planner.h
                                raven2/src/ambf_motion_planner.cpp
                                raven2/src/ambf_controller.cpp
                                raven2/src/ambf_defines.cpp
                                raven2/src/main.cpp )



target_link_libraries (ambf_controller ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

set (PSM_IK_HDR_FILES
    dvrk/cpp_controller/include/psm_ik_test.h
    dvrk/cpp_controller/include/DH.h
    dvrk/cpp_controller/include/Utilities.h)

set(PSM_IK_SOURCE_FILES
    dvrk/cpp_controller/src/psm_ik_test.cpp
    dvrk/cpp_controller/src/DH.cpp
    dvrk/cpp_controller/src/Utilities.cpp)

add_executable (psm_ik_test  ${PSM_IK_HDR_FILES} ${PSM_IK_SOURCE_FILES})
target_link_libraries (psm_ik_test ${AMBF_LIBRARIES} ${AMBF_CLIEMT_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ambf_client_cpp)


set (ECM_IK_HDR_FILES
    dvrk/cpp_controller/include/ecm_ik_test.h
    dvrk/cpp_controller/include/DH.h
    dvrk/cpp_controller/include/Utilities.h)

set(ECM_IK_SOURCE_FILES
    dvrk/cpp_controller/src/ecm_ik_test.cpp
    dvrk/cpp_controller/src/DH.cpp
    dvrk/cpp_controller/src/Utilities.cpp)

add_executable (ecm_ik_test  ${ECM_IK_HDR_FILES} ${ECM_IK_SOURCE_FILES})
target_link_libraries (ecm_ik_test ${AMBF_LIBRARIES} ${AMBF_CLIEMT_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ambf_client_cpp)


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dvrk)
