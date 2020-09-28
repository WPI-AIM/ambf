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

#include "ambf_client/RigidBody.h"
namespace ambf_client{

RigidBody::RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RigidBodyRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

bool RigidBody::is_joint_idx_valid(int joint_idx) {
    int n_joints = m_State.joint_positions.size();
    if(joint_idx < n_joints) return true;

    std::cerr << "ERROR! Requested Joint Idx of " << std::to_string(joint_idx) << " outside valid range [0 - " << std::to_string(n_joints - 1) << "]" << std::endl;

    return false;
}

tf::Vector3 RigidBody::get_linear_vel() {
    tf::Vector3 v(0, 0, 0);
    tf::vector3MsgToTF(m_State.twist.linear, v);

    return v;
}

///
/// \brief RigidBody::get_angular_velocity
/// \param ax
/// \param ay
/// \param az
///
tf::Vector3 RigidBody::get_angular_vel(){
    tf::Vector3 a(0, 0, 0);
    tf::vector3MsgToTF(m_State.twist.angular, a);

    return a;
}

int RigidBody::get_joint_idx_from_name(std::string joint_name) {
    std::vector<std::string> joint_names = m_State.joint_names;
    std::vector<std::string>::iterator it = std::find(joint_names.begin(), joint_names.end(), joint_name);

    if(it != joint_names.end()) {
        int joint_idx = std::distance(joint_names.begin(), it);
        return joint_idx;
    }

    std::cerr << "ERROR! Requested Joint " << joint_name << " not found in list of joints" << std::endl;
    return -1;
}


std::string RigidBody::get_joint_name_from_idx(int joint_idx) {
    if(is_joint_idx_valid(joint_idx))
        return m_State.joint_names[joint_idx];

    return NULL;
}

template <>
float RigidBody::get_joint_pos(int joint_idx){
    if(is_joint_idx_valid(joint_idx))
        return m_State.joint_positions[joint_idx];

    std::cerr << "Invalid joint credentials" << std::endl;
    return FLT_MIN;
}

template <>
float RigidBody::get_joint_pos(std::string joint_name){

    int joint_idx = get_joint_idx_from_name(joint_name);

    return get_joint_pos(joint_idx);
}


template <>
float RigidBody::get_joint_vel(int joint_idx){
    if(is_joint_idx_valid(joint_idx))
        return m_State.joint_velocities[joint_idx];

    std::cerr << "Invalid joint credentials" << std::endl;
    return FLT_MIN;
}

template <>
float RigidBody::get_joint_vel(std::string joint_name){

    int joint_idx = get_joint_idx_from_name(joint_name);

    return get_joint_vel(joint_idx);
}

template <>
float RigidBody::get_joint_effort(int joint_idx){
    if(is_joint_idx_valid(joint_idx))
        return m_State.joint_efforts[joint_idx];

    std::cerr << "Invalid joint credentials" << std::endl;
    return FLT_MIN;
}

template <>
float RigidBody::get_joint_effort(std::string joint_name){

    int joint_idx = get_joint_idx_from_name(joint_name);

    return get_joint_effort(joint_idx);
}

tf::Vector3 RigidBody::get_inertia() {
    tf::Vector3 I(0, 0, 0);
    tf::pointMsgToTF(m_State.pInertia, I);

    return I;
}

tf::Vector3 RigidBody::get_pos_command() {
    double px = m_Cmd.pose.position.x;
    double py = m_Cmd.pose.position.y;
    double pz = m_Cmd.pose.position.z;
    return tf::Vector3(px, py, pz);
}

tf::Quaternion RigidBody::get_rot_command() {
    tf::Quaternion rot_quat(0.0, 0.0, 0.0, 0.0);
    tf::quaternionMsgToTF(m_Cmd.pose.orientation, rot_quat);

    return rot_quat;
}

tf::Vector3 RigidBody::get_force_command() {
    tf::Vector3 f(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_Cmd.wrench.torque, f);
    return f;
}


tf::Vector3 RigidBody::get_torque_command() {
    tf::Vector3 t(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_Cmd.wrench.torque, t);
    return t;
}

tf::Vector3 RigidBody::get_linear_velocity_command() {
    tf::Vector3 l(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_Cmd.twist.linear, l);
    return l;
}

tf::Vector3 RigidBody::get_angular_velocity_command() {
    tf::Vector3 a(0.0, 0.0, 0.0);
    tf::vector3MsgToTF(m_Cmd.twist.angular, a);
    return a;
}

///
/// \brief RigidBody::set_force
/// \param fx
/// \param fy
/// \param fz
///
void RigidBody::set_force(double fx, double fy, double fz){
    tf::Vector3 f(fx, fy, fz);
    tf::vector3TFToMsg(f, m_Cmd.wrench.force);
}

///
/// \brief RigidBody::set_torque
/// \param nx
/// \param ny
/// \param nz
///
void RigidBody::set_torque(double nx, double ny, double nz){
    tf::Vector3 n(nx, ny, nz);
    tf::vector3TFToMsg(n, m_Cmd.wrench.torque);
}

void RigidBody::set_wrench(tf::Vector3 f, tf::Vector3 n) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_FORCE;

    tf::vector3TFToMsg(f, m_Cmd.wrench.force);
    tf::vector3TFToMsg(n, m_Cmd.wrench.torque);
}

///
/// \brief RigidBody::set_position
/// \param px
/// \param py
/// \param pz
///
void RigidBody::set_position(double px, double py, double pz){
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
void RigidBody::set_rpy(double roll, double pitch, double yaw) {
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
void RigidBody::set_rot(tf::Quaternion rot_quat) {
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_Cmd.pose.orientation);
}

void RigidBody::set_pose(const tf::Pose pose) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_POSITION;

    tf::poseTFToMsg(pose, m_Cmd.pose);
}

///
/// \brief RigidBody::set_linear_vel
/// \param vx
/// \param vy
/// \param vz
///
void RigidBody::set_linear_vel(double vx, double vy, double vz){
    tf::Vector3 v(vx, vy, vz);
    tf::vector3TFToMsg(v, m_Cmd.twist.linear);
}

///
/// \brief RigidBody::set_angular_vel
/// \param ax
/// \param ay
/// \param az
///
void RigidBody::set_angular_vel(double ax, double ay, double az){
    tf::Vector3 a(ax, ay, az);
    tf::vector3TFToMsg(a, m_Cmd.twist.angular);
}

void RigidBody::set_twist(tf::Vector3 v, tf::Vector3 a) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_VELOCITY;

    tf::vector3TFToMsg(v, m_Cmd.twist.linear);
    tf::vector3TFToMsg(a, m_Cmd.twist.angular);
}

void RigidBody::wrench_command(double fx, double fy, double fz, double nx, double ny, double nz) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_FORCE;

    tf::Vector3 f(fx, fy, fz);
    tf::vector3TFToMsg(f, m_Cmd.wrench.force);

    tf::Vector3 n(nx, ny, nz);
    tf::vector3TFToMsg(n, m_Cmd.wrench.torque);
}

void RigidBody::pose_command(double px, double py, double pz, double qx, double qy, double qz, double qw) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_POSITION;

    m_Cmd.pose.position.x = px;
    m_Cmd.pose.position.y = py;
    m_Cmd.pose.position.z = pz;

    tf::Quaternion rot_quat(qx, qy, qz, qw);
    tf::quaternionTFToMsg(rot_quat, m_Cmd.pose.orientation);
}

void RigidBody::velocity_command(double vx, double vy, double vz, double ax, double ay, double az) {
    m_Cmd.cartesian_cmd_type = m_Cmd.TYPE_VELOCITY;

    tf::Vector3 v(vx, vy, vz);
    tf::vector3TFToMsg(v, m_Cmd.twist.linear);

    tf::Vector3 a(ax, ay, az);
    tf::vector3TFToMsg(a, m_Cmd.twist.angular);
}

void RigidBody::set_joint_control(int joint_idx, float command, int control_type) {
    if(!is_joint_idx_valid(joint_idx)) return;
    int n_jnts = get_num_joints();


    if(m_Cmd.joint_cmds.size() != n_jnts) {
        m_Cmd.joint_cmds.resize(n_jnts, 0.0);
        m_Cmd.joint_cmds_types.resize(n_jnts, control_type);
    }

    m_Cmd.joint_cmds[joint_idx] = command;
    m_Cmd.joint_cmds_types[joint_idx] = control_type;

}

template<>
void RigidBody::set_joint_effort(int joint_idx, float effort) {
    set_joint_control(joint_idx, effort, m_Cmd.TYPE_FORCE);
}

template<>
void RigidBody::set_joint_effort(std::string joint_name, float effort) {
    int joint_idx = get_joint_idx_from_name(joint_name);
    set_joint_effort(joint_idx, effort);
}


//template<>
//void RigidBody::set_joint_pos<int>(int joint_idx, float pos) {
//    set_joint_control(joint_idx, pos, m_Cmd.TYPE_POSITION);
//}


//template<>
//void RigidBody::set_joint_pos<std::string>(std::string joint_idx, float pos) {
////    int joint_idx = get_joint_idx_from_name(joint_name);
////    set_joint_pos(joint_idx, pos);
//}


template<>
void RigidBody::set_joint_pos(int joint_idx, float pos) {
    set_joint_control(joint_idx, pos, m_Cmd.TYPE_POSITION);
}

template<>
void RigidBody::set_joint_pos(std::string joint_name, float pos) {
    int joint_idx = get_joint_idx_from_name(joint_name);
    set_joint_pos(joint_idx, pos);
}

template<>
void RigidBody::set_joint_vel(int joint_idx, float vel) {
    set_joint_control(joint_idx, vel, m_Cmd.TYPE_VELOCITY);
}

template<>
void RigidBody::set_joint_vel(std::string joint_name, float vel) {
    int joint_idx = get_joint_idx_from_name(joint_name);
    set_joint_vel(joint_idx, vel);
}

tf::Vector3 RigidBody::get_joint_position() {
    double px = m_State.pose.position.x;
    double py = m_State.pose.position.y;
    double pz = m_State.pose.position.z;

    return tf::Vector3(px, py, pz);
}

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


tf::Vector3 RigidBody::get_principal_inertia() {
    tf::Vector3 I(0, 0, 0);
    tf::pointMsgToTF(m_State.pInertia, I);

    return I;
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
