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

#ifndef AFSENSORCOMM_H
#define AFSENSORCOMM_H

#include <string>
#include "SensorRosCom.h"

namespace ambf_client{
class Sensor: public SensorRosCom{
public:
    Sensor(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
//    inline void set_name(std::string name){m_State.name.data = name;}
    tf::Vector3 get_position();
    tf::Quaternion get_orientation();
//    void set_wall_time(double a_sec);
//    inline void set_parent_name(std::string parent_name){m_State.parent_name.data = parent_name;}
//    inline void set_sim_time(double a_sec){ m_State.sim_time = a_sec;}
//    inline void increment_sim_step(){m_State.sim_step++;}
//    inline void set_sim_step(uint step){m_State.sim_step = step;}
    inline void set_count(int count){m_State.count = count;}

//    void set_trigger(bool triggered);
    inline bool is_triggered() { return m_State.triggered[0]; }
    std::vector<bool> get_triggers();

    double get_range() {m_State.range[0];}
    std::vector<double> get_ranges();

    double get_measurement() { return m_State.measurement[0]; }
    std::vector<double> get_measurements();

    std::string get_sensed_object(); //TBD
    std::vector<std::string> get_sensed_objects();

//    void set_type(std::string type);

    // We may have multiple individual sensor elements belonging to this
    // sensor comm. And groups of sensors may be in contact with different
    // sets of objects. This method is thus used to specify the mapping
    // of each sensor element w.r.t. to the sensed_objects list of string.
    std::vector<int> get_sensed_objects_map();

    int get_sensed_object_map() { return m_State.sensed_objects_map[0]; }

};
}

#endif
