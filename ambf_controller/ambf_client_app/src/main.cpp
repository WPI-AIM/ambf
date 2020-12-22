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
    \author    <schandrasekhar@wpi.edu>
    \author    Shreyas Chandra Sekhar
    \version   1.0$
*/
//==============================================================================
#include "ambf_client/ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

void printVector3(std::string caption, tf::Vector3 v) {
    ROS_INFO("%s: , %f, %f, %f", caption.c_str(), v[0], v[1], v[2]);
}

template<typename T>
void printVector(std::string caption, std::vector<T> ts) {
    ROS_INFO("%s", caption.c_str());

    for(T t: ts) {
        std::cout << t << std::endl;
    }
}

int main(int argc, char* argv[])
{
     Client client;
    client.connect();

//    client.printSummary();
//    vector<string> object_names;

//     std::vector<std::string> object_names = client.getRigidBodyNames();
////     usleep(20000);
//     std::cout << "main - list of object names:" << std::endl;
//     for(std::string object_name : object_names) {
//         std::cout << object_name << std::endl;
//     }

    std::string link_name = "base";
    rigidBodyPtr link_handler = client.getARigidBody(link_name, true);
    usleep(250000);
    link_handler->set_joint_effort(0, 0.0);

//    tf::Vector3 pos = link_handler->get_pos();
//    std::cout << "pos: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;


    link_handler->set_pos(0, 0, 0);
//    tf::Quaternion rot_quat(0.8, 0.8, 0.8, 0.8);
////    rot_quat = link_handler->get_rot();
//    link_handler->set_rot(rot_quat);
////    link_handler->set_rpy(0.8, 0.8, 0.8);
//    rot_quat = link_handler->get_rot();
//    std::cout << "rot_quat: " << rot_quat[0] << ", " << rot_quat[1] << ", " << rot_quat[2] << ", " << rot_quat[3] << std::endl;

    rigidBodyPtr link1_handler = client.getARigidBody("link4", true);

//    link1_handler->set_force(3000.0, 3000.0, 3000.0);
    link1_handler->set_angular_vel(30.0, 30.0, 30.0);
//    std::cout << "get_linear_vel: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;

//    std::cout << "get_name: " << link_handler->get_name() << std::endl;

//    std::vector<float> joints_command = {10.2, 50.2, 100.2, 130.2, 120.2, 120.2, 150.2};
//    for(int i = 0; i < 5; i++) {
//        link_handler->set_all_joint_effort(joints_command);
//        usleep(250000);

//        std::cout << "link_handler->get_num_of_children(): " << link_handler->get_num_of_children() << std::endl;
//    }
//    link_handler->set_joint_pos<int>(0, 100.0); // works now
//    link_handler->set_joint_pos<std::string>("link1-link2", 100.0);

//    link_handler->set_joint_vel<int>(1, 0.1);
//    link_handler->set_joint_vel<std::string>("link6-link7", 0.1);

//    link_handler->set_joint_effort<int>(1, 0.1);
//    link_handler->set_joint_effort<std::string>("link6-link7", 0.1);

//    std::vector<float> joints_command = {10.2, 50.2, 100.2, 130.2, 120.2, 120.2, 150.2};
//    link_handler->set_all_joint_pos(joints_command);
//    link_handler->set_all_joint_vel(joints_command);
//    link_handler->set_all_joint_effort(joints_command);

//     string psm_baselink = "psm/baselink";
//    // cout << "psm_baselink: " << psm_baselink << "\n";
//     rigidBodyPtr psm_baselink_handler = client.getARigidBody(psm_baselink, true);
//     usleep(1000000);

//    std::cout << "psm_baselink_handler->is_joint_idx_valid(0): " << psm_baselink_handler->is_joint_idx_valid(0) << std::endl;
//     print(psm_handle.get_rot())
//     psm_handle.set_joint_pos(0, 0)
//    psm_baselink_handler->get_joint_rpy();
//     for(int i = 0; i < 100; i++) {

////         psm_baselink_handler->get_joint_rpy();
////         usleep(20000);
//         psm_baselink_handler->set_joint_pos(0, 0);
//         usleep(20000);
//         int n_children = psm_baselink_handler->get_num_of_children();

//         cout << "get_num_of_children(): " << n_children << "\n";
//     }
//     std::vector<std::string> base_children = psm_baselink_handler->get_children_names();


//    for(string name : base_children) {
//        cout << "name: " << name << "\n";
//    }

//    ROS_INFO("is_joint_idx_valid(0): %d", psm_baselink_handler->is_joint_idx_valid(0));
//    printVector3("get_linear_vel()", psm_baselink_handler->get_linear_vel());
//    printVector3("get_angular_vel()", psm_baselink_handler->get_angular_vel());

//    if(base_children.size() < 1) {
//        client.cleanUp();
//        return 0;
//    }
//    string joint_name = ("baselink-" + base_children[0]).c_str();
//    ROS_INFO("get_joint_idx_from_name(%s): %d", joint_name.c_str(), psm_baselink_handler->get_joint_idx_from_name(joint_name));
//    ROS_INFO("get_joint_name_from_idx(%d): %s", 0, psm_baselink_handler->get_joint_name_from_idx(0).c_str());

//    ROS_INFO("is_joint_idx_valid(%d): %d", 0, psm_baselink_handler->is_joint_idx_valid(0));

//    ROS_INFO("get_joint_pos<int>(%d): %f", 0, psm_baselink_handler->get_joint_pos<int>(0));
//    ROS_INFO("get_joint_pos<std::string>(%s): %f", joint_name.c_str(), psm_baselink_handler->get_joint_pos<std::string>(joint_name));

//    ROS_INFO("get_joint_vel<int>(%d): %f", 0, psm_baselink_handler->get_joint_vel<int>(0));
//    ROS_INFO("get_joint_vel<std::string>(%s): %f", joint_name.c_str(), psm_baselink_handler->get_joint_vel<std::string>(joint_name));

//    ROS_INFO("get_joint_effort<int>(%d): %f", 0, psm_baselink_handler->get_joint_effort<int>(0));
//    ROS_INFO("get_joint_effort<std::string>(%s): %f", joint_name.c_str(), psm_baselink_handler->get_joint_effort<std::string>(joint_name));

//    printVector("psm_baselink_handler->get_all_joint_pos()", psm_baselink_handler->get_all_joint_pos());
//    printVector("psm_baselink_handler->get_all_joint_vel()", psm_baselink_handler->get_all_joint_vel());
//    printVector("psm_baselink_handler->get_all_joint_effort()", psm_baselink_handler->get_all_joint_effort());

//    tf::Vector3 joint_position = psm_baselink_handler->get_joint_position();
//    printVector3("joint_position", joint_position);
//    client.printSummary();
    client.cleanUp();
//    client.~Client();

	return 0;
}
