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

#ifndef ROSCOMBASE_H
#define ROSCOMBASE_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "ambf_server/CmdWatchDog.h"
#include "mutex"

class afROSNode{
  public:
    static ros::NodeHandle* getNodeAndRegister(){
        s_nodePtr = getNode();
        s_registeredInstances++;
        return s_nodePtr;
    }

    static ros::NodeHandle* getNode(){
        if (s_initialized == false){
            int argc = 0;
            char **argv = 0;
            ros::init(argc, argv, "ambf_comm_node", ros::init_options::AnonymousName);
            s_nodePtr = new ros::NodeHandle;
            s_initialized = true;
            std::cerr << "INFO! INITIALIZING ROS NODE HANDLE\n";
        }
        return s_nodePtr;
    }

    static void destroyNode(){
        if (s_initialized){
            s_initialized = false;

            std::cerr << "INFO! TOTAL ACTIVE COMM INSTANCES: " << s_registeredInstances << std::endl;
            std::cerr << "INFO! WAITING FOR ALL COMM INSTANCES TO UNREGISTER ... \n";
            while(s_registeredInstances > 0){
                std::cerr << "\tINFO! REMAINING ACTIVE COMMs: " << s_registeredInstances << std::endl;
                usleep(10000);
            }

            std::cerr << "INFO! DESTROYING ROS NODE HANDLE\n";
            ros::shutdown();
            delete s_nodePtr;
        }
    }

    static bool isNodeActive(){
        return s_initialized;
    }

    static void unRegister(){
        s_registeredInstances--;
    }

private:
    static bool s_initialized;
    static unsigned int s_registeredInstances;
    static ros::NodeHandle* s_nodePtr;
};

template <class T_state, class T_cmd>
class RosComBase{
public:
    RosComBase(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);

    ~RosComBase();

    virtual void init() = 0;

    virtual void run_publishers();

    virtual void cleanUp();

    inline void enableComm(){
        m_enableComm = true;
    }

    inline void disableComm(){
        m_enableComm = false;
    }

    virtual T_cmd get_command(){
        return m_Cmd;
    }

    virtual void set_state(T_state& state){
        m_writeMtx.lock();
        m_State = state;
        m_writeMtx.unlock();
    }

    virtual T_state& get_state(){
        return m_State;
    }

    inline void set_name(std::string name){
        m_State.name.data = name;
    }

    inline void set_identifier(std::string identifier){
        m_State.identifier.data = identifier;
    }

    inline void set_time_stamp(double a_sec){
        m_State.header.stamp.fromSec(a_sec);
    }

    inline void set_wall_time(double a_sec){
        m_State.wall_time = a_sec;
    }

    inline void set_sim_time(double a_sec){
        m_State.sim_time = a_sec;
        increment_sim_step();
    }

    virtual void increment_sim_step(){
        m_State.sim_step++;
    }

public:
    std::mutex m_writeMtx;

    int m_freq_min;

    int m_freq_max;

protected:
    ros::NodeHandle* nodePtr;

    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;

    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_namespace;

    std::string m_name;

    ros::Publisher m_pub;

    ros::Subscriber m_sub;

    tf::Transform m_trans;

    T_state m_State;

    T_cmd m_Cmd;

    T_cmd m_CmdPrev;

    boost::thread m_thread;

    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;

private:
    T_state m_StateCopy;

    // Flag to enable communication thread
    bool m_enableComm;

    void copyState(){
        m_writeMtx.lock();
        m_StateCopy = m_State;
        m_writeMtx.unlock();
    }
};

template<class T_state, class T_cmd>
void RosComBase<T_state, T_cmd>::run_publishers(){
    while(afROSNode::isNodeActive()){
        if (m_enableComm){
            // Call callbacks
            m_custom_queue.callAvailable();
            if(m_watchDogPtr->is_wd_expired()){
                m_watchDogPtr->consolePrint(m_name);
                reset_cmd();
            }
            // Update and publish state
            copyState();
            m_pub.publish(m_StateCopy);
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
    afROSNode::unRegister();
}

template<class T_state, class T_cmd>
RosComBase<T_state, T_cmd>::~RosComBase(){
    std::cerr << "INFO! Thread ShutDown: " << m_name << std::endl;
}


#endif
