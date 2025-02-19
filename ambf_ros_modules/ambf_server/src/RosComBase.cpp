//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
*/
//==============================================================================


#include "ambf_server//RosComBase.h"
#include "ambf_msgs/ActuatorCmd.h"
#include "ambf_msgs/ActuatorState.h"
#include "ambf_msgs/CameraState.h"
#include "ambf_msgs/CameraCmd.h"
#include "ambf_msgs/LightState.h"
#include "ambf_msgs/LightCmd.h"
#include "ambf_msgs/ObjectCmd.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/RigidBodyCmd.h"
#include "ambf_msgs/RigidBodyState.h"
#include "ambf_msgs/GhostObjectCmd.h"
#include "ambf_msgs/GhostObjectState.h"
#include "ambf_msgs/SensorCmd.h"
#include "ambf_msgs/SensorState.h"
#include "ambf_msgs/ContactSensorState.h"
#include "ambf_msgs/ContactSensorCmd.h"
#include "ambf_msgs/VehicleCmd.h"
#include "ambf_msgs/VehicleState.h"
#include "ambf_msgs/WorldCmd.h"
#include "ambf_msgs/WorldState.h"

bool afROSNode::s_initialized;
ros::NodeHandle* afROSNode::s_nodePtr;
unsigned int afROSNode::s_registeredInstances = 0;

template<class T_state, class T_cmd>
///
/// \brief RosComBase::cleanUp
///
void RosComBase<T_state, T_cmd>::cleanUp(){
    m_pub.shutdown();
    m_sub.shutdown();
}

template<class T_state, class T_cmd>
RosComBase<T_state, T_cmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out)
{
    m_name = a_name;
    m_namespace = a_namespace;
    m_enableComm = false;

    m_freq_min = a_freq_min;
    m_freq_max = a_freq_max;
    nodePtr = afROSNode::getNodeAndRegister();
    aspinPtr.reset(new ros::AsyncSpinner(1));
    nodePtr->setCallbackQueue(&m_custom_queue);
    m_watchDogPtr.reset(new CmdWatchDog(a_freq_min, a_freq_max, time_out));
}

template void RosComBase<ambf_msgs::ActuatorState, ambf_msgs::ActuatorCmd>::cleanUp();
template void RosComBase<ambf_msgs::CameraState, ambf_msgs::CameraCmd>::cleanUp();
template void RosComBase<ambf_msgs::LightState, ambf_msgs::LightCmd>::cleanUp();
template void RosComBase<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>::cleanUp();
template void RosComBase<ambf_msgs::RigidBodyState, ambf_msgs::RigidBodyCmd>::cleanUp();
template void RosComBase<ambf_msgs::GhostObjectState, ambf_msgs::GhostObjectCmd>::cleanUp();
template void RosComBase<ambf_msgs::SensorState, ambf_msgs::SensorCmd>::cleanUp();
template void RosComBase<ambf_msgs::ContactSensorState, ambf_msgs::ContactSensorCmd>::cleanUp();
template void RosComBase<ambf_msgs::VehicleState, ambf_msgs::VehicleCmd>::cleanUp();
template void RosComBase<ambf_msgs::WorldState, ambf_msgs::WorldCmd>::cleanUp();

template RosComBase<ambf_msgs::ActuatorState, ambf_msgs::ActuatorCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::CameraState, ambf_msgs::CameraCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::LightState, ambf_msgs::LightCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::RigidBodyState, ambf_msgs::RigidBodyCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::GhostObjectState, ambf_msgs::GhostObjectCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::SensorState, ambf_msgs::SensorCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::ContactSensorState, ambf_msgs::ContactSensorCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::VehicleState, ambf_msgs::VehicleCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<ambf_msgs::WorldState, ambf_msgs::WorldCmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
