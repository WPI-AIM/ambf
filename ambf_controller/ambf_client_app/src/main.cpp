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

    client.printSummary();
    std::vector<std::string> object_names = client.getRigidBodyNames();
    std::cout << "main - list of object names:" << std::endl;
    for(std::string object_name : object_names) {
     std::cout << object_name << std::endl;
    }

    std::string link_name = "base";
    rigidBodyPtr link_handler = client.getARigidBody(link_name, true);
    usleep(250000);
    link_handler->set_joint_effort(0, 0.0);


    link_handler->set_pos(0, 0, 0);
    rigidBodyPtr link1_handler = client.getARigidBody("link4", true);

    link1_handler->set_angular_vel(30.0, 30.0, 30.0);

    link_handler->set_joint_pos<int>(0, 1.0);
    link_handler->set_joint_pos<std::string>("link1-link2", 100.0);

    client.printSummary();
    client.cleanUp();
	return 0;
}
