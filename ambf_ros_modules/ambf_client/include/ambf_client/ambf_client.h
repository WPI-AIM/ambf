
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

#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include<geometry_msgs/WrenchStamped.h>

#include "ambf_client/Actuator.h"
#include "ambf_client/Camera.h"
#include "ambf_client/World.h"
#include "ambf_client/RosComBase.h"
#include "ambf_client/Object.h"
#include "ambf_client/Light.h"
#include "ambf_client/RigidBody.h"
#include "ambf_client/Sensor.h"
#include "ambf_client/Vehicle.h"

#include<ambf_msgs/ActuatorCmd.h>
#include<ambf_msgs/ActuatorState.h>
#include<ambf_msgs/CameraCmd.h>
#include<ambf_msgs/CameraState.h>
#include<ambf_msgs/LightCmd.h>
#include<ambf_msgs/LightState.h>
#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/RigidBodyCmd.h>
#include<ambf_msgs/RigidBodyState.h>
#include<ambf_msgs/SensorCmd.h>
#include<ambf_msgs/SensorState.h>
#include<ambf_msgs/VehicleCmd.h>
#include<ambf_msgs/VehicleState.h>
#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>

#include <unordered_map>
#include <memory>
#include <vector>
#include <iostream>
#include <string>
#include "ros/topic_manager.h"

using namespace std;
using namespace ambf_client;

//------------------------------------------------------------------------------
typedef IBaseObject* iBaseObjectPtr;
typedef std::unordered_map<string, iBaseObjectPtr> iBaseObjectMap;

typedef Actuator* actuatorPtr;
typedef Camera* cameraPtr;
typedef Light* lightPtr;
typedef Object* objectPtr;
typedef RigidBody* rigidBodyPtr;
typedef Sensor* sensorPtr;
typedef Vehicle* vehiclePtr;
typedef World* worldPtr;
//------------------------------------------------------------------------------
struct Observation{
public:
    Observation();

    geometry_msgs::PoseStamped m_nextState;
    double m_reward;
    double m_done;
};

class Client{
public:
    Client(std::string a_name="");
    ~Client(void);

    void connect();
    void sys_run();
    void createObjsFromRostopics();
    void printSummary();

    vector<string> getActuatorNames();
    vector<string> getCameraNames();
    vector<string> getLightNames();
    vector<string> getObjectNames();
    vector<string> getRigidBodyNames();
    vector<string> getSensorNames();
    vector<string> getVehicleNames();
    vector<string> getWorldNames();

    actuatorPtr getActuator(std::string a_name, bool suppress_warning);
    cameraPtr getCamera(std::string a_name, bool suppress_warning);
    lightPtr getLight(std::string a_name, bool suppress_warning);
    objectPtr getObject(std::string a_name, bool suppress_warning);
    rigidBodyPtr getRigidBody(std::string a_name, bool suppress_warning);
    sensorPtr getSensor(std::string a_name, bool suppress_warning);
    vehiclePtr getVehicle(std::string a_name, bool suppress_warning);
    worldPtr getWorld(std::string a_name, bool suppress_warning);


    void cleanUp();

private:
    ros::master::V_TopicInfo ros_topics_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> > objects_map_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> >::iterator itr_;
    std::unordered_map<string, IBaseObject *>::iterator ptr_;


    const float rate_{1000};
    string world_name_ = "";
    string a_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded


    const int a_freq_min_{50};
    const int a_freq_max_{100};
    const double time_out_{10.0};
    const int loop_rate_{1000};

    bool getPublishedTopics();
    bool endsWith(const std::string& stack, const std::string& needle);


    void refresh();
    void start();

    string getCommonNamespace();

    template <typename T, typename TMap>
    T getObject(std::string a_name, TMap* a_map, bool suppress_warning);
    bool checkMessageType(std::string msg_type);
    void getObjectNames(string msg_type, vector<string>& object_names);
};


#endif
