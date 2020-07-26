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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

#include "ambf_comm/RigidBody.h"
namespace ambf_comm{

RigidBody::RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RigidBodyRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

///
/// \brief RigidBody::cur_position
/// \param px
/// \param py
/// \param pz
///
void RigidBody::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_State.pose.position.x = px;
    m_State.pose.position.y = py;
    m_State.pose.position.z = pz;
}


///
/// \brief RigidBody::cur_orientation
/// \param roll
/// \param pitch
/// \param yaw
///
void RigidBody::cur_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}


///
/// \brief RigidBody::cur_orientation
/// \param qx
/// \param qy
/// \param qz
/// \param qw
///
void RigidBody::cur_orientation(double qx, double qy, double qz, double qw){
    tf::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}


///
/// \brief RigidBody::cur_force
/// \param fx
/// \param fy
/// \param fz
///
void RigidBody::cur_force(double fx, double fy, double fz){
    tf::Vector3 f(fx, fy, fz);
    tf::vector3TFToMsg(f, m_State.wrench.force);
}


///
/// \brief RigidBody::cur_torque
/// \param nx
/// \param ny
/// \param nz
///
void RigidBody::cur_torque(double nx, double ny, double nz){
    tf::Vector3 n(nx, ny, nz);
    tf::vector3TFToMsg(n, m_State.wrench.torque);
}


///
/// \brief RigidBody::cur_linear_velocity
/// \param vx
/// \param vy
/// \param vz
///
void RigidBody::cur_linear_velocity(double vx, double vy, double vz){
    tf::Vector3 v(vx, vy, vz);
    tf::vector3TFToMsg(v, m_State.twist.linear);
}


///
/// \brief RigidBody::cur_angular_velocity
/// \param ax
/// \param ay
/// \param az
///
void RigidBody::cur_angular_velocity(double ax, double ay, double az){
    tf::Vector3 a(ax, ay, az);
    tf::vector3TFToMsg(a, m_State.twist.angular);
}


///
/// \brief RigidBody::get_command
/// \return
///
ambf_msgs::RigidBodyCmd RigidBody::get_command(){
    ambf_msgs::RigidBodyCmd temp_cmd = m_Cmd;
    int joint_commands_size = m_Cmd.joint_cmds.size();
    temp_cmd.joint_cmds_types.resize(joint_commands_size);
    for(size_t idx = 0; idx < joint_commands_size ; idx++){
        if (idx < m_Cmd.joint_cmds_types.size()){
            temp_cmd.joint_cmds_types[idx] = m_Cmd.joint_cmds_types[idx];
        }
        else{
            temp_cmd.joint_cmds_types[idx] = 0;
        }
    }
    return temp_cmd;
}


///
/// \brief RigidBody::set_wall_time
/// \param a_sec
///
void RigidBody::set_wall_time(double a_sec){
    m_State.wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
}


///
/// \brief RigidBody::set_children_names
/// \param children_names
///
void RigidBody::set_children_names(std::vector<std::string> children_names){
    m_State.children_names = children_names;
}


///
/// \brief RigidBody::set_joint_names
/// \param joint_names
///
void RigidBody::set_joint_names(std::vector<std::string> joint_names){
    m_State.joint_names = joint_names;
}


///
/// \brief RigidBody::set_joint_positions
/// \param joint_positions
///
void RigidBody::set_joint_positions(std::vector<float> joint_positions){
    if (m_State.joint_positions.size() != joint_positions.size()){
        m_State.joint_positions.resize(joint_positions.size());
    }
    m_State.joint_positions = joint_positions;
}


///
/// \brief RigidBody::set_joint_velocities
/// \param joint_velocities
///
void RigidBody::set_joint_velocities(std::vector<float> joint_velocities){
    if (m_State.joint_velocities.size() != joint_velocities.size()){
        m_State.joint_velocities.resize(joint_velocities.size());
    }
    m_State.joint_velocities = joint_velocities;
}


///
/// \brief RigidBody::set_joint_efforts
/// \param joint_efforts
///
void RigidBody::set_joint_efforts(std::vector<float> joint_efforts){
    if (m_State.joint_efforts.size() != joint_efforts.size()){
        m_State.joint_efforts.resize(joint_efforts.size());
    }
    m_State.joint_efforts = joint_efforts;
}

extern "C"{

RigidBody* create_RigidBody(std::string a_name, std::string a_namespace="/ambf_comm/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new RigidBody(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_rigid_body(RigidBody* obj){
    delete obj;
}

}

}
