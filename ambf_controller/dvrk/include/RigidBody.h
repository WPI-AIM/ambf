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
#include "RigidBodyRosCom.h"

namespace ambf_client{

class RigidBody: public RigidBodyRosCom{
public:
    RigidBody(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    inline int get_num_of_children(){return m_State.children_names.size();}
    inline std::vector<std::string> get_children_names(){return m_State.children_names;}
    //    inline std::string get_name() { return m_State.name.c_str(); } - TBD
    std::string get_parent_name(); //TBD

    bool is_joint_idx_valid(int joint_idx);
    tf::Vector3 get_joint_position();
    tf::Vector3 get_joint_rpy(); //TBD
    tf::Quaternion get_joint_orientation();
    tf::Pose get_joint_pose();
    tf::Vector3 get_principal_inertia();
    inline double get_joint_mass(){return m_State.mass;}

    tf::Vector3 get_linear_velocity();
    tf::Vector3 get_angular_velocity();

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
