//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#include <ambf_server/SensorRosCom.h>

SensorRosCom::SensorRosCom(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): SensorRosComBase(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    init();
}

void SensorRosCom::init(){
    m_State.name.data = m_name;
    m_State.sim_step = 0;

    ambf_ral::create_publisher<AMBF_RAL_MSG(ambf_msgs, SensorState)>
      (m_pubPtr,
       m_nodePtr,
       "/" + m_namespace + "/" + m_name + "/State",
       10, false);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(ambf_msgs, SensorCmd), SensorRosCom>
      (m_subPtr,
       m_nodePtr,
       "/" + m_namespace + "/" + m_name + "/Command",
       10,
       &SensorRosCom::sub_cb, this);

    m_thread = std::thread(std::bind(&SensorRosCom::run_publishers, this));
    std::cerr << "INFO! Thread Joined: " << m_name << std::endl;
}

void SensorRosCom::reset_cmd(){
    for (int i = 0 ; i < m_Cmd.actuate.size() ; i++){
        m_Cmd.actuate[i] = false;
    }
}

void SensorRosCom::sub_cb(const AMBF_RAL_MSG(ambf_msgs, SensorCmd) & msg){
    m_Cmd = msg;
    m_watchDogPtr->acknowledge_wd();
}


ContactSensorRosCom::ContactSensorRosCom(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): SensorRosComBase(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    init();
}

void ContactSensorRosCom::init(){
    m_State.name.data = m_name;
    m_State.sim_step = 0;
    
    ambf_ral::create_publisher<AMBF_RAL_MSG(ambf_msgs, ContactSensorState)>
      (m_pubPtr,
       m_nodePtr,
       "/" + m_namespace + "/" + m_name + "/State",
       10, false);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(ambf_msgs, ContactSensorCmd), ContactSensorRosCom>
      (m_subPtr,
       m_nodePtr,
       "/" + m_namespace + "/" + m_name + "/Command",
       10,
       &ContactSensorRosCom::sub_cb, this);

    m_thread = std::thread(std::bind(&ContactSensorRosCom::run_publishers, this));
    std::cerr << "INFO! Thread Joined: " << m_name << std::endl;
}

void ContactSensorRosCom::reset_cmd(){

}

void ContactSensorRosCom::sub_cb(const AMBF_RAL_MSG(ambf_msgs, ContactSensorCmd) & msg){
    m_Cmd = msg;
    m_watchDogPtr->acknowledge_wd();
}

template<class T_state, class T_cmd>
SensorRosComBase<T_state, T_cmd>::SensorRosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RosComBase<T_state, T_cmd>(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

//template SensorRosComBase<ambf_msgs::ContactSensorState, ambf_msgs::ContactSensorCmd>::SensorRosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
