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

#include <string>
#include "ambf_client/RigidBodyRosCom.h"
#include <type_traits>
#include <iostream>

namespace ambf_client{

class RigidBody: public RigidBodyRosCom {
public:
    RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);

    bool is_active(); //TBD
    int get_sim_step(); //TBT

    bool is_joint_idx_valid(int joint_idx);
    tf::Vector3 get_linear_vel();
    tf::Vector3 get_angular_vel();
    int get_joint_idx_from_name(std::string joint_name);
    std::string get_joint_name_from_idx(int joint_idx);

    template<typename T>
    float get_joint_pos(T t);

    template<typename T>
    float get_joint_vel(T t);

    template<typename T>
    float get_joint_effort(T t);

    inline std::vector<float> get_all_joint_pos() { return m_State.joint_positions; }
    inline std::vector<float> get_all_joint_vel() { return m_State.joint_velocities; }
    inline std::vector<float> get_all_joint_effort() { return m_State.joint_efforts; }
    inline int get_num_joints() { return m_State.joint_positions.size(); }
    inline std::vector<std::string> get_joint_names() { return m_State.joint_names; }
    tf::Vector3 get_inertia();

    tf::Vector3 get_pos_command();
    tf::Quaternion get_rot_command();
    tf::Vector3 get_force_command();
    tf::Vector3 get_torque_command();
    tf::Vector3 get_linear_velocity_command();
    tf::Vector3 get_angular_velocity_command();

    inline bool get_publish_children_name_flag() { return m_Cmd.publish_children_names; }
    inline bool get_publish_joint_names_flag() { return m_Cmd.publish_joint_names; }
    inline bool get_publish_joint_positions_flag() { return m_Cmd.publish_joint_positions; }

    inline void set_publish_children_names_flag(bool publish_children_names) { m_Cmd.publish_children_names = publish_children_names; }
    inline void set_publish_joint_names_flag(bool publish_joint_names) { m_Cmd.publish_joint_names = publish_joint_names; }
    inline void set_publish_joint_positions_flag(bool publish_joint_positions) { m_Cmd.publish_joint_positions= publish_joint_positions; }

    void set_force(double fx, double fy, double fz);
    void set_torque(double nx, double ny, double nz);
    void set_wrench(tf::Vector3 f, tf::Vector3 n);
    void set_position(double px, double py, double pz);
    void set_rpy(double roll, double pitch, double yaw);
    void set_rot(tf::Quaternion rot_quat);
    void set_pose(tf::Pose pose);
    void set_linear_vel(double vx, double vy, double vz);
    void set_angular_vel(double ax, double ay, double az);
    void set_twist(tf::Vector3 v, tf::Vector3 a);

    void wrench_command(double fx, double fy, double fz, double nx, double ny, double nz);
    void pose_command(double px, double py, double pz, double qx, double qy, double qz, double qw);
    void velocity_command(double vx, double vy, double vz, double ax, double ay, double az);

    void set_joint_control(int joint_idx, float command, int control_type);

    template<typename T>
    void set_joint_effort(T t, float effort);

    template<typename T>
    void set_joint_pos(T t, float pos);


    template<typename T>
    void set_joint_vel(T t, float vel);





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

    void getBackground();

    inline int get_num_of_children(){return m_State.children_names.size();}
    inline std::vector<std::string> get_children_names(){return m_State.children_names;}
    //    inline std::string get_name() { return m_State.name.c_str(); } - TBD
    std::string get_parent_name(); //TBD



    tf::Vector3 get_joint_position();
    tf::Vector3 get_joint_rpy(); //TBD
    tf::Quaternion get_joint_orientation();
    tf::Pose get_joint_pose();
    tf::Vector3 get_principal_inertia();
    inline double get_joint_mass(){return m_State.mass;}




    tf::Vector3 get_joint_force();
    tf::Vector3 get_joint_torque();

    void set_joint_position(double px, double py, double pz);
    void set_joint_orientation(double roll, double pitch, double yaw);
    void set_joint_orientation(double qx, double qy, double qz, double qw);
    void set_joint_pose(tf::Pose pose);

    void set_linear_velocity(double vx, double vy, double vz);
    void set_angular_velocity(double ax, double ay, double az);
    void set_joint_velocities(std::vector<float> joint_velocities);

    void set_joint_efforts(std::vector<float> joint_efforts); //TBD
    void set_joint_force(double fx, double fy, double fz);
    void set_joint_torque(double nx, double ny, double nz);

};


}

#endif
