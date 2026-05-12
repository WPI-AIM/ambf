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

#ifndef CDVRK_BRIDGEH
#define CDVRK_BRIDGEH

#include <ambf_ral/ambf_ral.h>
#include "FootPedals.h"
#include "Console.h"
#include "string.h"
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

using namespace boost::placeholders;

#include "dvrk_arm/States.h"
#include "FcnHandle.h"
#include "dvrk_arm/Timing.h"

class DVRK_Bridge: public States, public DVRK_FootPedals{
public:
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name, int bridge_frequnce = 1000);
    ~DVRK_Bridge();

    void servo_cp(const geometry_msgs::PoseStamped &pose);
    void servo_cf(const geometry_msgs::Wrench &wrench);
    void servo_jp(const sensor_msgs::JointState &jnt_state);
    void set_cur_mode(const std::string &state, bool lock_ori);

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    static void get_arms_from_rostopics(std::vector<std::string> &arm_names);

    bool _start_pubs;
    bool _gripper_closed;

    typedef ambf_ral::node_ptr_t NodePtr;
    typedef ambf_ral::rate_ptr_t RatePtr;

    bool shutDown();

    FcnHandle<const AMBF_RAL_MSG(geometry_msgs, PoseStamped)&> poseFcnHandle;
    FcnHandle<const AMBF_RAL_MSG(sensor_msgs, JointState)&> jointFcnHandle;
    FcnHandle<const AMBF_RAL_MSG(geometry_msgs, WrenchStamped)&> wrenchFcnHandle;
    FcnHandle<const AMBF_RAL_MSG(sensor_msgs, JointState)&> gripperFcnHandle;

private:
    std::string arm_name;

    std::shared_ptr<ambf_ral::ral> m_ral;
    NodePtr n;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(geometry_msgs, WrenchStamped)) servo_cf_pub;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(std_msgs, Bool)) force_orientation_lock_pub;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(std_msgs, String)) state_pub;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(geometry_msgs, PoseStamped)) servo_cp_pub;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(sensor_msgs, JointState)) servo_jp_pub;
    AMBF_RAL_PUBLISHER_PTR(AMBF_RAL_MSG(std_msgs, Bool)) gravity_comp_ena_pub;

    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(geometry_msgs, PoseStamped)) measured_cp_sub;
    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(sensor_msgs, JointState)) measured_js_sub;
    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(std_msgs, String)) state_sub;
    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(geometry_msgs, WrenchStamped)) measured_cf_sub;
    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(std_msgs, Bool)) gripper_event_sub;
    AMBF_RAL_SUBSCRIBER_PTR(AMBF_RAL_MSG(sensor_msgs, JointState)) gripper_measured_js_sub;
    RatePtr run_loop_rate, wrench_loop_max_rate;
    int _freq;

    double scale;
    std::vector<std::string> valid_arms;
    void init();
    void state_cb(const AMBF_RAL_MSG(std_msgs, String) &msg);
    void measured_cp_cb(const AMBF_RAL_MSG(geometry_msgs, PoseStamped) &msg);
    void measured_js_cb(const AMBF_RAL_MSG(sensor_msgs, JointState) &msg);
    void measured_cf_cb(const AMBF_RAL_MSG(geometry_msgs, WrenchStamped) &msg);
    void gripper_sub_cb(const AMBF_RAL_MSG(std_msgs, Bool) &gripper);
    void gripper_measured_js_cb(const AMBF_RAL_MSG(sensor_msgs, JointState) &state);
    void _rate_sleep();
    void run();
    std::shared_ptr<boost::thread> loop_thread;

    AMBF_RAL_MSG(geometry_msgs, PoseStamped) cur_pose, pre_pose, cmd_pose;
    AMBF_RAL_MSG(sensor_msgs, JointState) cur_joint, pre_joint, cmd_joint;
    AMBF_RAL_MSG(std_msgs, String) cur_state, state_cmd;
    AMBF_RAL_MSG(geometry_msgs, WrenchStamped) cur_wrench, cmd_wrench;
    bool _on;
};

#endif
