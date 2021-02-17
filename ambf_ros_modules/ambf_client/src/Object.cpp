//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
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
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar
    \version   1.0$
*/
//==============================================================================

#include "ambf_client/Object.h"
namespace ambf_client{

Object::Object(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): ObjectRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

bool Object::is_joint_idx_valid(int joint_idx) {
    int n_joints = get_num_of_children();


    return joint_idx <= n_joints;
}

int Object::get_joint_idx_from_name(std::string joint_name) {
    std::vector<std::string> joint_names = m_State.joint_names;

    std::vector<std::string>::iterator it = std::find(joint_names.begin(), joint_names.end(), joint_name);

    if(it != joint_names.end()) {
        return std::distance(joint_names.begin(), it);
    }

    return -1;
}

std::string Object::get_joint_name_from_id(int joint_index) {
    std::vector<std::string> joint_names = m_State.joint_names;
    if(joint_index < joint_names.size()) {
        return joint_names[joint_index];
    }

    return NULL;
}


tf::Vector3 Object::get_position() {
    double px = m_State.pose.position.x;
    double py = m_State.pose.position.y;
    double pz = m_State.pose.position.z;

    return tf::Vector3(px, py, pz);
}

tf::Quaternion Object::get_orientation() {
    tf::Quaternion rot_quat;

    tf::quaternionMsgToTF(m_State.pose.orientation, rot_quat);
    return rot_quat;
}

tf::Pose Object::get_pose() {
    tf::Pose pose;

    tf::poseMsgToTF(m_State.pose, pose);
    return pose;
}

///
/// \brief Object::get_joint_force
/// \param fx
/// \param fy
/// \param fz
///
tf::Vector3 Object::get_joint_force(){
    tf::Vector3 f(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_State.wrench.torque, f);
    return f;
}


///
/// \brief Object::get_joint_torque
/// \param nx
/// \param ny
/// \param nz
///
tf::Vector3 Object::get_joint_torque(){
    tf::Vector3 n(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_State.wrench.torque, n);
    return n;
}


extern "C"{

Object* create_object(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Object(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_object(Object* obj){
    delete obj;
}

}

}
