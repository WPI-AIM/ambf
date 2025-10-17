//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
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
#include <thread>
#include <ambf_server/ambf_ral.h>
#include <ambf_server/CmdWatchDog.h>
#include <unistd.h>
#include <mutex>
#include <cstdint>

/*! Class afROSNode.  Singleton that owns the ROS node(s) and allow to
  access them from anywhere, including plugins.  ROS1 implementation
  creates a single node and uses custom queues to manage difference
  spin rates using callAvailable.  ROS 2 allows to create multiple
  nodes per process so it's easier to manage.  For ROS 2, the nodes
  are managed throught the RAL class found in ambf_ral and stored in a
  std::map so they can be retrieved by name. To create a new node, use
  getNodeAndRegister.  To retrieve an existing node, use getNode.  Any
  class calling getNodeAndRegister should also call destroyNode at the
  end (cleanUp, destructor...). */
class afROSNode {
public:
    static ambf_ral::node_ptr_t getNodeAndRegister(const std::string & node_name) {
        ambf_ral::node_ptr_t result = nullptr;
        s_mutex.lock();
#if AMBF_ROS1
        s_registeredInstances++;
        if (s_initialized == false) {
            s_ral = new ambf_ral::ral("AMBF");
            s_initialized = true;
            std::cerr << "INFO! INITIALIZING ROS 1 NODE HANDLE FOR AMBF" << std::endl;
        }
        result = s_ral->node();
#elif AMBF_ROS2
        std::string _real_name = node_name;
        ambf_ral::clean_nodename(_real_name);
        // find in map
        auto found = s_rals.find(_real_name);
        if (found != s_rals.end()) {
            std::cerr << "WARNING! Trying to create a new ROS2 node with an existing name: "
                      << _real_name << " (based on user provided name " << node_name << ")" << std::endl;
            std::cerr << "WARNING! Returning the found node instead " << std::endl;
            result = s_rals[_real_name]->node();
        } else {
            std::cerr << "INFO! Created a new ROS2 node for: "
                      << _real_name << " (based on user provided name " << node_name << ")" << std::endl;
            ambf_ral::ral * _ral = new ambf_ral::ral(_real_name);
            s_rals[_real_name] = _ral;
            result = _ral->node();
            s_initialized = true;
        }
#endif
        s_mutex.unlock();
        return result;
    }

public:

    static ambf_ral::node_ptr_t getNode(const std::string & node_name) {
        s_mutex.lock();
        ambf_ral::node_ptr_t result = nullptr;
#if AMBF_ROS1
        if (s_initialized) {
            result = s_ral->node();
        } else {
            std::cerr << "ERROR! getNode CALLED BEFORE getNodeAndRegister FOR: " << node_name << std::endl;
        }
#elif AMBF_ROS2
        std::string _real_name = node_name;
        ambf_ral::clean_nodename(_real_name);
        // find in map
        auto found = s_rals.find(_real_name);
        if (found != s_rals.end()) {
            result = found->second->node();
        } else {
            std::cerr << "INFO! getNode CALLED BEFORE getNodeAndRegister FOR: "
                      << _real_name << " (based on user provided name " << node_name << ")" << std::endl;
        }
#endif
        s_mutex.unlock();
        return result;
    }

    static bool destroyNode(const std::string & node_name) {
        s_mutex.lock();
        s_initialized = false;
        bool success = false;
#if AMBF_ROS1
        if (s_registeredInstances == 0) {
          std::cerr << "WARNING: TRYING TO DESTROY MORE COMM INSTANCES THAN REGISTERED FOR: " << node_name << std::endl;
          return false;
        }
        s_registeredInstances--;
        std::cerr << "INFO! TOTAL ACTIVE COMM INSTANCES AFTER destroyNode for "
                  << node_name << ": " << s_registeredInstances << std::endl;
        if (s_registeredInstances == 0) {
            std::cerr << "INFO! DESTROYING ROS 1 NODE HANDLE\n";
            ambf_ral::shutdown();
            delete s_ral;
        }
        success = true;
#elif AMBF_ROS2
        std::string _real_name = node_name;
        ambf_ral::clean_nodename(_real_name);
        // find in map
        auto found = s_rals.find(_real_name);
        if (found != s_rals.end()) {
            std::cerr << "INFO! DESTROYING ROS 2 NODE ROS: "
                      << _real_name << " (based on user provided name " << node_name << ")" << std::endl;
            delete(found->second);
            s_rals.erase(found);
            success = true;
        } else {
            std::cerr << "INFO! destroyNode couldn't find ROS 2 node for: "
                      << _real_name << " (based on user provided name " << node_name << ")" << std::endl;
        }
        if (s_rals.size() == 0) {
            ambf_ral::shutdown();
        }
#endif
        s_mutex.unlock();
        return success;
    }

    static bool isNodeActive(void) {
        return s_initialized;
    }

private:
    static std::mutex s_mutex;
    static bool s_initialized;
#if AMBF_ROS1
    static size_t s_registeredInstances;
    static ambf_ral::ral * s_ral;
#elif AMBF_ROS2
    static std::map<std::string, ambf_ral::ral*> s_rals;
#endif
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
        m_State.header.stamp = ambf_ral::time_from_seconds(a_sec);
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
    ambf_ral::node_ptr_t m_nodePtr;

    std::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_namespace;

    std::string m_name;

    tf2::Transform m_trans;

    AMBF_RAL_PUBLISHER_PTR(T_state) m_pubPtr;
    AMBF_RAL_SUBSCRIBER_PTR(T_cmd) m_subPtr;

    T_state m_State;

    T_cmd m_Cmd;

    T_cmd m_CmdPrev;

    std::thread m_thread;

#if AMBF_ROS1
    ros::CallbackQueue m_custom_queue;
#endif

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
void RosComBase<T_state, T_cmd>::run_publishers(void) {
    while (afROSNode::isNodeActive()) {
        if (m_enableComm) {
            // Call callbacks
#if AMBF_ROS1
            m_custom_queue.callAvailable();
#elif AMBF_ROS2
            try {
                rclcpp::spin_some(m_nodePtr);
            } catch (const std::exception & e) {
                std::cerr << "ERROR! Exception caught while spinning node "
                          << m_namespace + m_name << ": " << e.what() << std::endl;
            }
            // rclcpp::spin_some(m_nodePtr);
#endif
            if (m_watchDogPtr->is_wd_expired()) {
                m_watchDogPtr->consolePrint(m_name);
                reset_cmd();
            }
            // Update and publish state
            copyState();
            m_pubPtr->publish(m_StateCopy);
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
}

template<class T_state, class T_cmd>
RosComBase<T_state, T_cmd>::~RosComBase(){
    m_thread.join();
    cleanUp();
    afROSNode::destroyNode(m_namespace + m_name);
    std::cerr << "INFO! Thread ShutDown: " << m_namespace + m_name << std::endl;
}

#endif
