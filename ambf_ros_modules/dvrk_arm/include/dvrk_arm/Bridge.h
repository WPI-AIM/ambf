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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/WrenchStamped.h"
#include "FootPedals.h"
#include "Console.h"
#include "string.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "ros/callback_queue.h"
#include "dvrk_arm/States.h"
#include "FcnHandle.h"
#include "dvrk_arm/Timing.h"
#include "boost/thread.hpp"

class DVRK_Bridge: public States, public DVRK_FootPedals{
public:
    friend class DVRK_FootPedals;
    friend class DVRK_Console;

    DVRK_Bridge(const std::string &arm_name, int bridge_frequnce = 1000);
    ~DVRK_Bridge();

    void set_cur_pose(const geometry_msgs::PoseStamped &pose);
    void set_cur_wrench(const geometry_msgs::Wrench &wrench);
    void set_cur_joint(const sensor_msgs::JointState &jnt_state);
    void set_cur_mode(const std::string &state, bool lock_ori);

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    static void get_arms_from_rostopics(std::vector<std::string> &arm_names);

    bool _start_pubs;
    bool _gripper_closed;

    typedef std::shared_ptr<ros::NodeHandle> NodePtr;
    typedef std::shared_ptr<ros::Rate> RatePtr;
    typedef std::shared_ptr<ros::AsyncSpinner> AspinPtr;

    bool shutDown();

    FcnHandle<const geometry_msgs::PoseStamped&> poseFcnHandle;
    FcnHandle<const sensor_msgs::JointState&> jointFcnHandle;
    FcnHandle<const geometry_msgs::WrenchStamped&> wrenchFcnHandle;
    FcnHandle<const sensor_msgs::JointState&> gripperFcnHandle;

private:
    std::string arm_name;

    NodePtr n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_lock_pub;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
    ros::Publisher joint_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Subscriber wrench_sub;
    ros::Subscriber gripper_sub;
    ros::Subscriber gripper_angle_sub;
    ros::CallbackQueue cb_queue;
    RatePtr run_loop_rate, wrench_loop_max_rate;
    int _freq;

    double scale;
    std::vector<std::string> valid_arms;
    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void wrench_sub_cb(const geometry_msgs::WrenchStampedConstPtr &wrench);
    void gripper_sub_cb(const std_msgs::BoolConstPtr &gripper);
    void gripper_state_sub_cb(const sensor_msgs::JointStateConstPtr &state);
    void timer_cb(const ros::TimerEvent&);
    void _rate_sleep();
    void run();
    std::shared_ptr<boost::thread> loop_thread;

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint, cmd_joint;
    std_msgs::String cur_state, state_cmd;
    geometry_msgs::WrenchStamped cur_wrench, cmd_wrench;
    bool _on;
};

#endif
