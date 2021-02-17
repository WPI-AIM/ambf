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

#include "ambf_client/WorldRosCom.h"

///
/// \brief PointCloundHandler::init
/// \param a_node
/// \param a_topic_name
///
void PointCloundHandler::init(boost::shared_ptr<ros::NodeHandle> a_node, std::string a_topic_name){
    m_pcSub = a_node->subscribe(a_topic_name, 5, &PointCloundHandler::sub_cb, this);
}


///
/// \brief PointCloundHandler::sub_cb
/// \param msg
///
void PointCloundHandler::sub_cb(sensor_msgs::PointCloudPtr msg){
    m_StatePtr = msg;
}


sensor_msgs::PointCloudPtr PointCloundHandler::get_point_cloud(){
    return m_StatePtr;
}


void PointCloundHandler::remove(){
    m_pcSub.shutdown();
    m_StatePtr->points.clear();
    m_StatePtr->channels.clear();
}


///
/// \brief WorldRosCom::WorldRosCom
/// \param a_name
/// \param a_namespace
/// \param a_freq_min
/// \param a_freq_max
/// \param time_out
///
WorldRosCom::WorldRosCom(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RosComBase(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    init();
}


///
/// \brief WorldRosCom::init
///
void WorldRosCom::init(){
    m_State.sim_step = 0;
    m_enableSimThrottle = false;
    m_stepSim = true;

    m_pub = nodePtr->advertise<ambf_msgs::WorldCmd>("/" + m_namespace + "/" + m_name + "/Command", 10);
    m_sub = nodePtr->subscribe("/" + m_namespace + "/" + m_name + "/State", 10, &WorldRosCom::sub_cb, this);


    m_thread = boost::thread(boost::bind(&WorldRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
}


///
/// \brief WorldRosCom::~WorldRosCom
///
WorldRosCom::~WorldRosCom(){
    ros::shutdown();
    usleep(100000);
    std::cerr << "Thread Shutdown: " << m_name << std::endl;
}


///
/// \brief WorldRosCom::reset_cmd
///
void WorldRosCom::reset_cmd(){
    m_enableSimThrottle = false;
    m_stepSim = true;
}


///
/// \brief WorldRosCom::sub_cb
/// \param msg
///

void WorldRosCom::sub_cb(ambf_msgs::WorldStateConstPtr msg){
    m_StatePrev = m_State;
    m_State = *msg;
    m_watchDogPtr->acknowledge_wd();
}
