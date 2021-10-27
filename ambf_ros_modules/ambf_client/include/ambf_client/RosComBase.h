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

#ifndef ROSCOMBASE_H
#define ROSCOMBASE_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "ambf_client/CmdWatchDog.h"

class IBaseObject
{
  // list of cell methods
public:
    virtual ~IBaseObject() = default;
};

template <class T_cmd, class T_state>
class RosComBase : public IBaseObject {
public:
    RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out)
    {
        m_name = a_name;
        m_namespace = a_namespace;

        m_freq_min = a_freq_min;
        m_freq_max = a_freq_max;

        int argc = 0;
        char **argv = 0;
        ros::init(argc, argv, "ambf_client");
        nodePtr.reset(new ros::NodeHandle);
        aspinPtr.reset(new ros::AsyncSpinner(1));

        nodePtr->setCallbackQueue(&m_custom_queue);


        m_watchDogPtr.reset(new CmdWatchDog(a_freq_min, a_freq_max, time_out));


    }

    RosComBase(T_cmd m_Cmd_val, T_state m_State_val) : m_Cmd(m_Cmd_val), m_State(m_State_val){}

    virtual void init() = 0;
    virtual void run_publishers();
    virtual void cleanUp();
    virtual T_cmd get_command(){return m_Cmd;}

    int m_freq_min;
    int m_freq_max;

protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_namespace;
    std::string m_name;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_state m_StatePrev;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;


    boost::thread m_thread;
    ros::CallbackQueue m_custom_queue;


    virtual void reset_cmd() = 0;

};

template <class T_cmd, class T_state>
void RosComBase<T_cmd, T_state>::run_publishers(){
    while(nodePtr->ok()){
        m_pub.publish(m_Cmd);

        m_custom_queue.callAvailable();
        ros::MultiThreadedSpinner spinner(0);
        spinner.spin(&m_custom_queue);

        if(m_watchDogPtr->is_wd_expired()){
            m_watchDogPtr->consolePrint(m_name);
            reset_cmd();
        }

        m_watchDogPtr->m_ratePtr->sleep();

    }
}


#endif
