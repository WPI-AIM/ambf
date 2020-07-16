//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (www.aimlab.wpi.edu)

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

    \author    <http://www.aimlab.wpi.edu>
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar
    \version   $
*/
//==============================================================================


#include "BaseObject.h"
//#include "ambf_msgs/ActuatorCmd.h"
//#include "ambf_msgs/ActuatorState.h"
//#include "ambf_msgs/CameraState.h"
//#include "ambf_msgs/CameraCmd.h"
//#include "ambf_msgs/LightState.h"
//#include "ambf_msgs/LightCmd.h"
#include "ambf_msgs/ObjectCmd.h"
#include "ambf_msgs/ObjectState.h"
//#include "ambf_msgs/RigidBodyCmd.h"
//#include "ambf_msgs/RigidBodyState.h"
//#include "ambf_msgs/SensorCmd.h"
//#include "ambf_msgs/SensorState.h"
//#include "ambf_msgs/VehicleCmd.h"
//#include "ambf_msgs/VehicleState.h"
//#include "ambf_msgs/WorldCmd.h"
//#include "ambf_msgs/WorldState.h"

//template <class T_state, class T_cmd>
//void BaseObject<T_state, T_cmd>::destroy_at(T_state* t_s, T_cmd* t_c) {
//    t_s->~T_state();
//    t_c->~T_cmd();
//}


//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::ActuatorState, ambf_msgs::ActuatorCmd>::cleanUp(){
//    RosComBase<ambf_msgs::ActuatorState, ambf_msgs::ActuatorCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::CameraState, ambf_msgs::CameraCmd>::cleanUp(){
//    RosComBase<ambf_msgs::CameraState, ambf_msgs::CameraCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::LightState, ambf_msgs::LightCmd>::cleanUp(){
//    RosComBase<ambf_msgs::LightState, ambf_msgs::LightCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

//template<>
///
/// \brief RosComBase::cleanUp
///
///

//BaseObject<ambf_msgs::ObjectCmd, ambf_msgs::ObjectState>::cleanUp(){
//void RosComBase<ambf_msgs::ObjectCmd, ambf_msgs::ObjectState>::cleanUp();
//void BaseObject<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>::cleanUp(){
//        RosComBase<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>::cleanUp();
//    m_pub.shutdown();
//    m_sub.shutdown();
//}

//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::RigidBodyState, ambf_msgs::RigidBodyCmd>::cleanUp(){
//    RosComBase<ambf_msgs::RigidBodyState, ambf_msgs::RigidBodyCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}


//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::SensorState, ambf_msgs::SensorCmd>::cleanUp(){
//    RosComBase<ambf_msgs::SensorState, ambf_msgs::SensorCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::VehicleState, ambf_msgs::VehicleCmd>::cleanUp(){
//    RosComBase<ambf_msgs::VehicleState, ambf_msgs::VehicleCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

//template<>
/////
///// \brief RosComBase::cleanUp
/////
//void BaseObject<ambf_msgs::WorldState, ambf_msgs::WorldCmd>::cleanUp(){
//    RosComBase<ambf_msgs::WorldState, ambf_msgs::WorldCmd>::cleanUp();
////    m_pub.shutdown();
////    m_sub.shutdown();
//}

