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

#ifndef AFRIGIDBODYCOMM_H
#define AFRIGIDBODYCOMM_H

#include "ambf_client/RigidBodyRosCom.h"
#include <boost/lexical_cast.hpp>
#include <string>
#include <type_traits>
#include <iostream>
#include <numeric>

namespace ambf_client{

class RigidBody: public RigidBodyRosCom {
public:
    RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);

    // Getters Common to Object class
    bool is_active(); //TBD - its always active
    inline int get_sim_step() { return m_State.sim_step; } //TBT - To be tested
    inline int get_num_of_children(){return m_State.children_names.size();}
    inline std::vector<std::string> get_children_names(){return m_State.children_names;}
    tf::Vector3 get_pos(); //TBT
    tf::Quaternion get_rot(); //TBT
    tf::Vector3 get_rpy(); //TBT
    tf::Pose get_pose(); //TBT
    tf::Vector3 get_pos_command(); //TBT
    tf::Quaternion get_rot_command(); //TBT
    inline std::string get_name() { return boost::lexical_cast<std::string>(m_State.name); } //TBT

    // Setters Common to Object class
    void set_active(bool flag); //TBD
    void set_pos(double px, double py, double pz); //TBT
    void set_rpy(double roll, double pitch, double yaw); //TBT
    void set_rot(tf::Quaternion rot_quat); //TBT


    // Getters w.r.t Rigid Body Class
    bool is_joint_idx_valid(int joint_idx); //TBT
    tf::Vector3 get_linear_vel(); //TBT
    tf::Vector3 get_angular_vel(); //TBT
    int get_joint_idx_from_name(std::string joint_name); //TBT
    std::string get_joint_name_from_idx(int joint_idx); //TBT

    template<typename T> //TBT
    float get_joint_pos(T t);

    template<typename T> //TBT
    float get_joint_vel(T t);

    template<typename T> //TBT
    float get_joint_effort(T t);

    inline std::vector<float> get_all_joint_pos() { return m_State.joint_positions; } //TBT
    inline std::vector<float> get_all_joint_vel() { return m_State.joint_velocities; } //TBT
    inline std::vector<float> get_all_joint_effort() { return m_State.joint_efforts; } //TBT
    inline int get_num_joints() { return m_State.joint_positions.size(); } //TBT
    inline std::vector<std::string> get_joint_names() { return m_State.joint_names; } //TBT
    tf::Vector3 get_inertia();



    tf::Vector3 get_force_command(); //TBT
    tf::Vector3 get_torque_command(); //TBT
    tf::Vector3 get_linear_velocity_command(); //TBT
    tf::Vector3 get_angular_velocity_command(); //TBT

    inline bool get_publish_children_name_flag() { return m_Cmd.publish_children_names; } //TBT
    inline bool get_publish_joint_names_flag() { return m_Cmd.publish_joint_names; } //TBT
    inline bool get_publish_joint_positions_flag() { return m_Cmd.publish_joint_positions; } //TBT

    // Setters w.r.t Rigid Body Class
    inline void set_publish_children_names_flag(bool publish_children_names) { m_Cmd.publish_children_names = publish_children_names; } //TBT
    inline void set_publish_joint_names_flag(bool publish_joint_names) { m_Cmd.publish_joint_names = publish_joint_names; } //TBT
    inline void set_publish_joint_positions_flag(bool publish_joint_positions) { m_Cmd.publish_joint_positions= publish_joint_positions; } //TBT

    void set_force(double fx, double fy, double fz);  //TBT
    void set_torque(double nx, double ny, double nz);  //TBT
    void set_wrench(tf::Vector3 f, tf::Vector3 n);  //TBT

//    void set_position(double px, double py, double pz);
//    void set_rpy(double roll, double pitch, double yaw);
//    void set_rot(tf::Quaternion rot_quat);
    void set_pose(tf::Pose pose); //TBT
    void set_linear_vel(double vx, double vy, double vz); //TBT
    void set_angular_vel(double ax, double ay, double az); //TBT
    void set_twist(tf::Vector3 v, tf::Vector3 a);  //TBT
    void set_twist(geometry_msgs::Twist twist);  //TBT

    void wrench_command(double fx, double fy, double fz, double nx, double ny, double nz);  //TBT
    void pose_command(double px, double py, double pz, double qx, double qy, double qz, double qw);  //TBT
    void velocity_command(double vx, double vy, double vz, double ax, double ay, double az);  //TBT

    template<typename T>
    void set_joint_pos(T t, float pos); //TBT
    void set_multiple_joint_pos(std::vector<int> joints_idx, std::vector<float> joints_pos); //TBT

    template<typename T>
    void set_joint_vel(T t, float vel); //TBT
    void set_multiple_joint_vel(std::vector<int> joints_idx, std::vector<float> joints_vel); //TBT

    template<typename T>
    void set_joint_effort(T t, float effort); //TBT
    void set_multiple_joint_effort(std::vector<int> joints_idx, std::vector<float> joints_effort); //TBT

//    template<typename T>
//    int get_joint_pos(T t) {
//        int joint_idx = -1;

//        if(std::is_same<T, std::string>::value) {
////            joint_idx = get_joint_idx_from_name(std::to_string(t).c_str()); //compilation error on conversion
//            std::cerr << "String type: " << t << std::endl;
//        } else if(std::numeric_limits<T>::is_integer) {
//            std::cerr << "Integer type: " << t << std::endl;
////            joint_idx = convert<int>(t); //compilation error on conversion
//        } else {
//            return joint_idx;
//        }

//        return m_State.joint_positions[joint_idx];
//    }


private:
    void set_joint_control(int joint_idx, float command, int control_type);
    void set_multiple_joint_control(std::vector<int> joints_idx, std::vector<float> joints_pos, int control_type);
};


}

#endif
