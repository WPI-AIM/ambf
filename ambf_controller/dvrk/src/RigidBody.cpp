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

#include "RigidBody.h"
namespace ambf_client{

RigidBody::RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RigidBodyRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

bool RigidBody::is_joint_idx_valid(int joint_idx) {
    int n_joints = m_State.joint_positions.size();
    if(joint_idx < n_joints) return true;

    std::cerr << "ERROR! Requested Joint Idx of " << std::to_string(joint_idx) << " outside valid range [0 - " << std::to_string(n_joints - 1) << "]" << std::endl;

    return false;
}

tf::Vector3 RigidBody::get_joint_position() {
    double px = m_State.pose.position.x;
    double py = m_State.pose.position.y;
    double pz = m_State.pose.position.z;

    std::vector<float> joint_positions = m_State.joint_positions;
    ROS_INFO("joint_position-size(): %d", joint_positions.size());
    for(float joint_position : joint_positions) {
        ROS_INFO("joint_position: %f", joint_position);
    }

    return tf::Vector3(px, py, pz);
}

//tf::Quaternion RigidBody::get_ryp() {
//}

tf::Quaternion RigidBody::get_joint_orientation() {
    tf::Quaternion rot_quat;

    tf::quaternionMsgToTF(m_State.pose.orientation, rot_quat);
    return rot_quat;
}

tf::Pose RigidBody::get_joint_pose() {
    tf::Pose pose;

    tf::poseMsgToTF(m_State.pose, pose);
    return pose;
}

tf::Vector3 RigidBody::get_linear_velocity() {
    tf::Vector3 v(0, 0, 0);
    tf::vector3MsgToTF(m_State.twist.linear, v);

    return v;
}

tf::Vector3 RigidBody::get_principal_inertia() {
    tf::Vector3 I(0, 0, 0);
    tf::pointMsgToTF(m_State.pInertia, I);

    return I;
}

///
/// \brief RigidBody::get_angular_velocity
/// \param ax
/// \param ay
/// \param az
///
tf::Vector3 RigidBody::get_angular_velocity(){
    tf::Vector3 a(0, 0, 0);
    tf::vector3MsgToTF(m_State.twist.angular, a);

    return a;
}


///
/// \brief RigidBody::get_joint_force
/// \param fx
/// \param fy
/// \param fz
///
tf::Vector3 RigidBody::get_joint_force(){
    tf::Vector3 f(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_State.wrench.torque, f);
    return f;
}


///
/// \brief RigidBody::get_joint_torque
/// \param nx
/// \param ny
/// \param nz
///
tf::Vector3 RigidBody::get_joint_torque(){
    tf::Vector3 n(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_State.wrench.torque, n);
    return n;
}


///
/// \brief RigidBody::set_position
/// \param px
/// \param py
/// \param pz
///
void RigidBody::set_joint_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_Cmd.pose.position.x = px;
    m_Cmd.pose.position.y = py;
    m_Cmd.pose.position.z = pz;
}

///
/// \brief RigidBody::set_orientation
/// \param roll
/// \param pitch
/// \param yaw
///
void RigidBody::set_joint_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_Cmd.pose.orientation);
}


///
/// \brief RigidBody::set_orientation
/// \param qx
/// \param qy
/// \param qz
/// \param qw
///
void RigidBody::set_joint_orientation(double qx, double qy, double qz, double qw){
    tf::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_Cmd.pose.orientation);
}


void RigidBody::set_joint_pose(const tf::Pose pose) {
     tf::poseTFToMsg(pose, m_Cmd.pose);
}

///
/// \brief RigidBody::set_linear_velocity
/// \param vx
/// \param vy
/// \param vz
///
void RigidBody::set_linear_velocity(double vx, double vy, double vz){
    tf::Vector3 v(vx, vy, vz);
    tf::vector3TFToMsg(v, m_Cmd.twist.linear);
}

///
/// \brief RigidBody::set_angular_velocity
/// \param ax
/// \param ay
/// \param az
///
void RigidBody::set_angular_velocity(double ax, double ay, double az){
    tf::Vector3 a(ax, ay, az);
    tf::vector3TFToMsg(a, m_Cmd.twist.angular);
}

///
/// \brief RigidBody::set_joint_velocities
/// \param joint_velocities
///
void RigidBody::set_joint_velocities(std::vector<float> joint_velocities){
    if (m_Cmd.joint_cmds.size() != joint_velocities.size()){
        m_State.joint_velocities.resize(joint_velocities.size());
    }
    m_State.joint_velocities = joint_velocities;
}


///
/// \brief RigidBody::set_joint_force
/// \param fx
/// \param fy
/// \param fz
///
void RigidBody::set_joint_force(double fx, double fy, double fz){
    tf::Vector3 f(fx, fy, fz);
    tf::vector3TFToMsg(f, m_Cmd.wrench.force);
}


///
/// \brief RigidBody::set_joint_torque
/// \param nx
/// \param ny
/// \param nz
///
void RigidBody::set_joint_torque(double nx, double ny, double nz){
    tf::Vector3 n(nx, ny, nz);
    tf::vector3TFToMsg(n, m_Cmd.wrench.torque);
}

extern "C"{

RigidBody* create_RigidBody(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new RigidBody(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_rigid_body(RigidBody* obj){
    delete obj;
}

}

}
