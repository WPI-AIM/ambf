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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

#include "ambf_comm/Sensor.h"
namespace ambf_comm{

Sensor::Sensor(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): SensorRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

void Sensor::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_State.pose.position.x = px;
    m_State.pose.position.y = py;
    m_State.pose.position.z = pz;
}

void Sensor::cur_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}

void Sensor::cur_orientation(double qx, double qy, double qz, double qw){
    tf::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}

void Sensor::set_wall_time(double a_sec){
    m_State.wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
}

void Sensor::set_trigger(bool triggered){
    if (m_State.triggered.size() == 0){
        m_State.triggered.resize(1);
    }
    m_State.triggered[0] = triggered;

}

void Sensor::set_triggers(std::vector<bool> triggered){
    if (m_State.triggered.size() != triggered.size()){
        m_State.triggered.resize(triggered.size());
    }

    for (int i = 0 ; i < triggered.size() ; i++){
        m_State.triggered[i] = triggered[i];
    }
}

void Sensor::set_range(double range){
    if (m_State.range.size() == 0){
        m_State.range.resize(1);
    }
    else{
        m_State.range[0] = range;
    }
}

void Sensor::set_ranges(std::vector<double> range){
    if (m_State.range.size() != range.size()){
        m_State.range.resize(range.size());
    }

    for (int i = 0 ; i < range.size() ; i++){
        m_State.range[i] = range[i];
    }
}

void Sensor::set_measurement(double measurement){
    if (m_State.measurement.size() == 0){
        m_State.measurement.resize(1);
    }
    m_State.measurement[0] = measurement;
}

void Sensor::set_measurements(std::vector<double> measurement){
    if (m_State.measurement.size() != measurement.size()){
        m_State.measurement.resize(measurement.size());
    }

    for (int i = 0 ; i < measurement.size() ; i++){
        m_State.measurement[i] = measurement[i];
    }
}

void Sensor::set_sensed_object(std::string sensed_object){
    if (m_State.sensed_objects.size() == 0){
        m_State.sensed_objects.resize(1);
    }
     m_State.sensed_objects[0].data = sensed_object;
}

void Sensor::set_sensed_objects(std::vector<std::string> sensed_objects){
    if (m_State.sensed_objects.size() != sensed_objects.size()){
        m_State.sensed_objects.resize(sensed_objects.size());
    }

    for (int i = 0 ; i < sensed_objects.size() ; i++){
        m_State.sensed_objects[i].data = sensed_objects[i];
    }
}

void Sensor::set_sensed_object_map(int sensed_objects_map){
    if (m_State.sensed_objects_map.size() == 0){
        m_State.sensed_objects_map.resize(1);
    }
    m_State.sensed_objects_map[0] = 0;
}

void Sensor::set_sensed_objects_map(std::vector<int> sensed_objects_map){
    if (m_State.sensed_objects_map.size() != sensed_objects_map.size()){
        m_State.sensed_objects_map.resize(sensed_objects_map.size());
    }

    for (int i = 0 ; i < sensed_objects_map.size() ; i++){
        m_State.sensed_objects_map[i] = sensed_objects_map[i];
    }
}


void Sensor::set_type(std::string type){
    m_State.type.data = type;
}

extern "C"{

Sensor* create_sensor(std::string a_name, std::string a_namespace="/ambf_comm/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Sensor(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_sensor(Sensor* obj){
    delete obj;
}

}
}
