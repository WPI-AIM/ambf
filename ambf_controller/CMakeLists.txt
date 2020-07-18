#  Software License Agreement (BSD License)
#  Copyright (c) 2019, AMBF.
#  (www.aimlab.wpi.edu)
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
#  $Author: Melody Su $
#  $Date:  April, 2019 $
#  $Rev:  $


# common settings required to build examples
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

include_directories (${GLFW_INCLUDE_DIRS}
                     raven2/include 
		     dvrk/include )

add_executable (ambf_controller raven2/include/ambf_defines.h
                                raven2/include/ambf_controller.h
                                raven2/include/ambf_motion_planner.h
                                raven2/src/ambf_motion_planner.cpp
                                raven2/src/ambf_controller.cpp 
                                raven2/src/ambf_defines.cpp
                                raven2/src/main.cpp )

add_executable (ambf_client    
	dvrk/include/CmdWatchDog.h
	dvrk/include/RosComBase.h

	dvrk/include/ActuatorRosCom.h
	dvrk/include/Actuator.h

	dvrk/include/Camera.h
	dvrk/include/CameraRosCom.h

	dvrk/include/LightRosCom.h
	dvrk/include/Light.h

	dvrk/include/ObjectRosCom.h
	dvrk/include/Object.h

	dvrk/include/RigidBodyRosCom.h
	dvrk/include/RigidBody.h

	dvrk/include/SensorRosCom.h
	dvrk/include/Sensor.h

	dvrk/include/VehicleRosCom.h
	dvrk/include/Vehicle.h

	dvrk/include/WorldRosCom.h
	dvrk/include/World.h

	dvrk/include/ambf_client.h
	#dvrk/include/BaseObject.h


	dvrk/src/RosComBase.cpp

	dvrk/src/ActuatorRosCom.cpp
	dvrk/src/Actuator.cpp

	dvrk/src/CameraRosCom.cpp
	dvrk/src/Camera.cpp

	dvrk/src/LightRosCom.cpp
	dvrk/src/Light.cpp

	dvrk/src/ObjectRosCom.cpp 
	dvrk/src/Object.cpp

	dvrk/src/RigidBodyRosCom.cpp
	dvrk/src/RigidBody.cpp

	dvrk/src/SensorRosCom.cpp
	dvrk/src/Sensor.cpp

	dvrk/src/VehicleRosCom.cpp
	dvrk/src/Vehicle.cpp

	dvrk/src/WorldRosCom.cpp
	dvrk/src/World.cpp

	dvrk/src/ambf_client.cpp
	dvrk/src/main.cpp
	#dvrk/src/BaseObject.cpp
	)

target_link_libraries (ambf_controller ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries (ambf_client ${AMBF_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})
