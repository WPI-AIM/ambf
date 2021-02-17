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

#include "ambf_client/Sensor.h"
namespace ambf_client{

Sensor::Sensor(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): SensorRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}


bool Sensor::is_triggered(int idx) {
    if(idx < 0 || idx >= m_State.triggered.size()) {
        std::cerr << "Invalid Index" << std::endl;

        return false;
    }

    return m_State.triggered[idx];
}

float Sensor::get_range(int idx) {
    if(idx < 0 || idx >= m_State.range.size()) {
        std::cerr << "Invalid Index" << std::endl;

        return -1;
    }

    return m_State.range[idx];
}

float Sensor::get_measurement(int idx) {
    if(idx < 0 || idx >= m_State.measurement.size()) {
        std::cerr << "Invalid Index" << std::endl;

        return -1;
    }

    return m_State.measurement[idx];
}

tf::Vector3 Sensor::get_position() {
    double px = m_State.pose.position.x;
    double py = m_State.pose.position.y;
    double pz = m_State.pose.position.z;

    return tf::Vector3(px, py, pz);
}

tf::Quaternion Sensor::get_orientation() {
    tf::Quaternion rot_quat;

    tf::quaternionMsgToTF(m_State.pose.orientation, rot_quat);
    return rot_quat;
}

std::vector<bool> Sensor::get_triggers(){
    std::vector<bool> triggered;
    for (int i = 0 ; i < m_State.triggered.size() ; i++){
        triggered[i] = m_State.triggered[i];
    }

    return triggered;
}

std::vector<double> Sensor::get_ranges(){
    std::vector<double> ranges;

    for (int i = 0 ; i < m_State.range.size() ; i++){
        ranges[i] = m_State.range[i];
    }
    return ranges;
}

std::vector<double> Sensor::get_measurements(){
    std::vector<double> measurement;

    for (int i = 0 ; i < m_State.measurement.size() ; i++){
        measurement[i] = m_State.measurement[i];
    }
    return measurement;
}

std::vector<std::string> Sensor::get_sensed_objects(){
    std::vector<std::string> sensed_objects;

    for (int i = 0 ; i < m_State.sensed_objects.size() ; i++){
        sensed_objects[i] = m_State.sensed_objects[i].data;
    }
}

std::vector<int> Sensor::get_sensed_objects_map(){
    std::vector<int> sensed_objects_map;

    for (int i = 0 ; i < m_State.sensed_objects_map.size() ; i++){
        sensed_objects_map[i] = m_State.sensed_objects_map[i];
    }
    return sensed_objects_map;
}

extern "C"{

Sensor* create_sensor(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Sensor(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_sensor(Sensor* obj){
    delete obj;
}

}
}
