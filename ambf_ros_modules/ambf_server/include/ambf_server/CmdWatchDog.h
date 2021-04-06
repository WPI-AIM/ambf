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

#ifndef CMDWATCHDOG_H
#define CMDWATCHDOG_H

#include <ros/rate.h>

class CmdWatchDog{
public:
    CmdWatchDog(const int &a_freq_min, const int &a_freq_max , const double &time_out): m_freq_min(a_freq_min), m_freq_max(a_freq_max), m_time_out(time_out){
        m_expire_duration.fromSec(m_time_out);
        m_initialized = false;
        _m_minRatePtr.reset(new ros::Rate(m_freq_min));
        _m_maxRatePtr.reset(new ros::Rate(m_freq_max));
        m_ratePtr = _m_minRatePtr;
    }
    void acknowledge_wd(){
        if (m_initialized == false){ m_ratePtr = _m_maxRatePtr;}
        m_initialized = true;
        m_next_cmd_expected_time= ros::Time::now() + m_expire_duration;
    }
    bool is_wd_expired(){
        bool expired = (ros::Time::now() > m_next_cmd_expected_time && m_initialized) ? true : false;
        if(expired) m_ratePtr = _m_minRatePtr;
        return expired;
    }
    void consolePrint(std::string class_name){
        if(m_initialized){
            m_initialized = false;
            std::cerr << "WatchDog expired, Resetting \"" << class_name << "\" command" << std::endl;
        }
    }
    boost::shared_ptr<ros::Rate> m_ratePtr;

protected:
    int m_freq_min, m_freq_max;
    double m_time_out;

private:
    boost::shared_ptr<ros::Rate> _m_minRatePtr, _m_maxRatePtr;
    ros::Time m_next_cmd_expected_time;
    ros::Duration m_expire_duration;
    bool m_initialized;
};
#endif
