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

#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include<geometry_msgs/WrenchStamped.h>

#include "World.h"
#include "BaseObject.h"
#include "Object.h"
#include "Light.h"
#include "RigidBody.h"

#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>
#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/LightCmd.h>
#include<ambf_msgs/LightState.h>
#include<ambf_msgs/RigidBodyCmd.h>
#include<ambf_msgs/RigidBodyState.h>

#include <unordered_map>
#include <memory>
#include <vector>
#include <iostream>
#include <string>

using namespace std;
using namespace ambf_client;


struct Observation{
public:
    Observation();

    geometry_msgs::PoseStamped m_nextState;
    double m_reward;
    double m_done;
};

class Client{
public:
    Client();
    ~Client(void);

    void connect();
    void create_objs_from_rostopics();

//    void add_object(std::string name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5);
//    ambf_client::Object* get_object_handle(std::string name);
//    bool object_cur_position(std::string name, double px, double py, double pz);
//    bool object_cur_orientation(std::string name, double roll, double pitch, double yaw);
//    bool object_cur_force(std::string name, double fx, double fy, double fz);
//    bool object_cur_torque(std::string name, double nx, double ny, double nz);
    void clean_up();

private:
    ros::master::V_TopicInfo ros_topics_;
//    vector<string> sub_list_;
//    std::unordered_map<string, ObjectClient *> objects_map_;

//    std::unordered_map<string, IBaseObject *> objects_map_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> > objects_map_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> >::iterator itr_;
    std::unordered_map<string, IBaseObject *>::iterator ptr_;
//    vector <shared_ptr<BaseObject>> As;
//    vector<IBaseObject *> list_;
//    vector <shared_ptr<IBaseObject *>> list_;

    float rate_ = 1000;
    string world_name_ = "";
    string a_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded
//    string client_name_ = "";
//WorldClient *world_handle_ = NULL;
    World *world_handle_ = NULL;
//    BaseObject<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd> *object_handle_ = NULL;
//    BaseObject *object_handle_ = NULL;

    int a_freq_min_ = 50;
    int a_freq_max_ = 100;
    double time_out_ = 10.0;


    bool getPublishedTopics();
    bool endsWith(const std::string& stack, const std::string& needle);


    void refresh();
    void start();

    string get_common_namespace();

//    bool object_exists(std::string name);
//    static const int max_obj_size=10;
//    int m_numObjects;
//    std::map<std::string, boost::shared_ptr<ambf_client::Object> > m_objectMap;
//    std::map<std::string, boost::shared_ptr<ambf_client::Object> >::iterator m_objectIt;
//    boost::shared_ptr<ambf_client::Object> m_Objects[max_obj_size];
};


#endif
