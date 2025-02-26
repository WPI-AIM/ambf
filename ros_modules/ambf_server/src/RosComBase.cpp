//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
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

#include <ambf_server/RosComBase.h>

std::mutex afROSNode::s_mutex;

bool afROSNode::s_initialized;
#if AMBF_ROS1
size_t afROSNode::s_registeredInstances = 0;
ambf_ral::ral * afROSNode::s_ral = nullptr;
#elif AMBF_ROS2
std::map<std::string, ambf_ral::ral*> afROSNode::s_rals;
#endif

template<class T_state, class T_cmd>
///
/// \brief RosComBase::cleanUp
///
void RosComBase<T_state, T_cmd>::cleanUp()
{
    ambf_ral::publisher_shutdown(m_pubPtr);
    ambf_ral::subscriber_shutdown(m_subPtr);
}

template<class T_state, class T_cmd>
RosComBase<T_state, T_cmd>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out)
{
    m_name = a_name;
    m_namespace = a_namespace;
    m_enableComm = false;

    m_freq_min = a_freq_min;
    m_freq_max = a_freq_max;
    m_nodePtr = afROSNode::getNodeAndRegister(a_namespace + a_name);
#if AMBF_ROS1
    m_nodePtr->setCallbackQueue(&m_custom_queue);
#endif
    m_watchDogPtr.reset(new CmdWatchDog(m_nodePtr, a_freq_min, a_freq_max, time_out));
}

template void RosComBase<AMBF_RAL_MSG(ambf_msgs, ActuatorState), AMBF_RAL_MSG(ambf_msgs, ActuatorCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, CameraState),  AMBF_RAL_MSG(ambf_msgs, CameraCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, LightState),  AMBF_RAL_MSG(ambf_msgs, LightCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, ObjectState),  AMBF_RAL_MSG(ambf_msgs, ObjectCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, RigidBodyState),  AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, SensorState),  AMBF_RAL_MSG(ambf_msgs, SensorCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, ContactSensorState),  AMBF_RAL_MSG(ambf_msgs, SensorCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, GhostObjectState),  AMBF_RAL_MSG(ambf_msgs, SensorCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, VehicleState),  AMBF_RAL_MSG(ambf_msgs, VehicleCmd)>::cleanUp();
template void RosComBase<AMBF_RAL_MSG(ambf_msgs, WorldState),  AMBF_RAL_MSG(ambf_msgs, WorldCmd)>::cleanUp();

template RosComBase<AMBF_RAL_MSG(ambf_msgs, ActuatorState),  AMBF_RAL_MSG(ambf_msgs, ActuatorCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, CameraState),  AMBF_RAL_MSG(ambf_msgs, CameraCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, LightState),  AMBF_RAL_MSG(ambf_msgs, LightCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, ObjectState),  AMBF_RAL_MSG(ambf_msgs, ObjectCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, RigidBodyState),  AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, SensorState),  AMBF_RAL_MSG(ambf_msgs, SensorCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, ContactSensorState),  AMBF_RAL_MSG(ambf_msgs, ContactSensorCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, GhostObjectState),  AMBF_RAL_MSG(ambf_msgs, GhostObjectCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, VehicleState),  AMBF_RAL_MSG(ambf_msgs, VehicleCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
template RosComBase<AMBF_RAL_MSG(ambf_msgs, WorldState),  AMBF_RAL_MSG(ambf_msgs, WorldCmd)>::RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
