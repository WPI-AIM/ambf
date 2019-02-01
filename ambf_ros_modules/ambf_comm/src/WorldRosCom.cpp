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
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   $
*/
//==============================================================================

#include "ambf_comm/WorldRosCom.h"

WorldRosCom::WorldRosCom(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max): RosComBase(a_name, a_namespace, a_freq_min, a_freq_max){
    init();
}

void WorldRosCom::init(){
    m_State.sim_step = 0;
    m_enableSimThrottle = false;
    m_stepSim = true;

    m_pub = nodePtr->advertise<ambf_msgs::WorldState>("/" + m_ambf_namespace + "/" + m_name + "/State", 10);
    m_sub = nodePtr->subscribe("/" + m_ambf_namespace + "/" + m_name + "/Command", 10, &WorldRosCom::sub_cb, this);

    m_thread = boost::thread(boost::bind(&WorldRosCom::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
}

WorldRosCom::~WorldRosCom(){
    ros::shutdown();
    std::cerr << "Thread Shutdown: " << m_name << std::endl;
}

void WorldRosCom::reset_cmd(){
    m_enableSimThrottle = false;
    m_stepSim = true;
}

void WorldRosCom::sub_cb(ambf_msgs::WorldCmdConstPtr msg){
    m_CmdPrev = m_Cmd;
    m_Cmd = *msg;
    m_num_skip_steps = m_Cmd.n_skip_steps;
    m_enableSimThrottle = (bool)m_Cmd.enable_step_throttling;
    if (m_enableSimThrottle){
        if(!m_stepSim){
            m_stepSim = (bool)m_Cmd.step_clock ^ (bool)m_CmdPrev.step_clock;
        }
    }
    else{
            m_stepSim = true;
    }
    m_watchDogPtr->acknowledge_wd();
}
