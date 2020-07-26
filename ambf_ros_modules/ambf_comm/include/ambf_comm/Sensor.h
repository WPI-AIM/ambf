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

#ifndef AFSENSORCOMM_H
#define AFSENSORCOMM_H

#include <string>
#include "ambf_comm/SensorRosCom.h"

namespace ambf_comm{
class Sensor: public SensorRosCom{
public:
    Sensor(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    inline void set_name(std::string name){m_State.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void set_wall_time(double a_sec);
    inline void set_parent_name(std::string parent_name){m_State.parent_name.data = parent_name;}
    inline void set_sim_time(double a_sec){ m_State.sim_time = a_sec;}
    inline void increment_sim_step(){m_State.sim_step++;}
    inline void set_sim_step(uint step){m_State.sim_step = step;}
    inline void set_count(int count){m_State.count = count;}

    void set_trigger(bool triggered);
    void set_triggers(std::vector<bool> triggered);

    void set_range(double range);
    void set_ranges(std::vector<double> ranges);

    void set_measurement(double measurements);
    void set_measurements(std::vector<double> measurements);

    void set_sensed_object(std::string sensed_object);
    void set_sensed_objects(std::vector<std::string> sensed_objects);

    void set_type(std::string type);

    // We may have multiple individual sensor elements belonging to this
    // sensor comm. And groups of sensors may be in contact with different
    // sets of objects. This method is thus used to specify the mapping
    // of each sensor element w.r.t. to the sensed_objects list of string.
    void set_sensed_objects_map(std::vector<int> sensed_objects_map);

    void set_sensed_object_map(int sensed_objects_map);

};
}

#endif
