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

#include "ambf_client/Vehicle.h"
namespace ambf_client{

Vehicle::Vehicle(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): VehicleRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
}

ambf_msgs::VehicleCmd Vehicle::get_command(){
    ambf_msgs::VehicleCmd temp_cmd = m_Cmd;
    return temp_cmd;
}

tf::Vector3 Vehicle::get_position() {
    double px = m_State.pose.position.x;
    double py = m_State.pose.position.y;
    double pz = m_State.pose.position.z;

    return tf::Vector3(px, py, pz);
}

tf::Quaternion Vehicle::get_orientation() {
    tf::Quaternion rot_quat;

    tf::quaternionMsgToTF(m_State.pose.orientation, rot_quat);
    return rot_quat;
}

tf::Vector3 Vehicle::get_principal_inertia() {
    tf::Vector3 I(0, 0, 0);
    tf::pointMsgToTF(m_State.pInertia, I);

    return I;
}

extern "C"{

Vehicle* create_Vehicle(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Vehicle(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_Vehicle(Vehicle* obj){
    delete obj;
}

}
}
