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

#include "dvrk_arm/Bridge.h"


DVRK_Bridge::DVRK_Bridge(const std::string &arm_name, int bridge_frequency): _freq(bridge_frequency){
    valid_arms.push_back("MTML");
    valid_arms.push_back("MTMR");
    valid_arms.push_back("PSM1");
    valid_arms.push_back("PSM2");
    valid_arms.push_back("PSM3");

    bool _valid_arm = false;

    for(size_t i = 0; i < valid_arms.size(); i ++){
        if (strcmp(arm_name.c_str(), valid_arms[i].c_str()) == 0){
            this->arm_name = valid_arms[i];
           _valid_arm = true;
        }
    }

    if(_valid_arm){
        std::cerr<<"CONNECTING TO: " << arm_name << std::endl;
        init();
    }
    else{
        std::cerr<<"INVALID ARM SPECIFIED: " << arm_name << std::endl;
    }
}

void DVRK_Bridge::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, arm_name + "_interface_node");
    n.reset(new ros::NodeHandle);
    n->setCallbackQueue(&cb_queue);
    run_loop_rate.reset(new ros::Rate(1000));
    wrench_loop_max_rate.reset(new ros::Rate(1000));

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_Bridge::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_Bridge::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/state_joint_current", 10, &DVRK_Bridge::joint_sub_cb, this);
    wrench_sub = n->subscribe("/dvrk/" + arm_name + "/wrench_body_current", 10, &DVRK_Bridge::wrench_sub_cb, this);
    gripper_sub = n->subscribe("/dvrk/" + arm_name + "/gripper_closed_event", 10, &DVRK_Bridge::gripper_sub_cb, this);
    gripper_angle_sub = n->subscribe("/dvrk/" + arm_name + "/state_gripper_current", 10, &DVRK_Bridge::gripper_state_sub_cb, this);

    joint_pub = n->advertise<sensor_msgs::JointState>("/dvrk/" + arm_name + "/set_position_joint", 10);
    pose_pub  = n->advertise<geometry_msgs::Pose>("/dvrk/" + arm_name + "/set_position_cartesian", 10);
    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_lock_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);

    activeState = DVRK_UNINITIALIZED;
    _gripper_closed = false;

    cmd_pose.pose.position.x = 0; cmd_pose.pose.position.y = 0; cmd_pose.pose.position.z = 0;
    cmd_pose.pose.orientation.x = 0; cmd_pose.pose.orientation.y = 0; cmd_pose.pose.orientation.z = 0; cmd_pose.pose.orientation.w = 1;
    cmd_wrench.wrench.force.x = 0; cmd_wrench.wrench.force.y = 0; cmd_wrench.wrench.force.z = 0;
    cmd_wrench.wrench.torque.x = 0; cmd_wrench.wrench.torque.y = 0; cmd_wrench.wrench.torque.z = 0;

    init_footpedals(n);
    loop_thread.reset(new boost::thread(boost::bind(&DVRK_Bridge::run, this)));
    _start_pubs = false;
    _on = true;
    usleep(300000);
    scale = 0.1;
}

void DVRK_Bridge::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
    if(jointFcnHandle._is_set){
        jointFcnHandle.fcn_handle(cur_joint);
    }
}

void DVRK_Bridge::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;
    if(poseFcnHandle._is_set){
        poseFcnHandle.fcn_handle(cur_pose);
    }
}

void DVRK_Bridge::wrench_sub_cb(const geometry_msgs::WrenchStampedConstPtr &msg){
    cur_wrench = *msg;
    if(wrenchFcnHandle._is_set){
        wrenchFcnHandle.fcn_handle(cur_wrench);
    }
}

void DVRK_Bridge::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
    for(std::map<ARM_STATES, std::string>::iterator it = stateMap.begin(); it != stateMap.end() ; ++it){
        if(strcmp(cur_state.data.c_str(), it->second.c_str()) == 0){
            activeState = it->first;
        }
    }
}

void DVRK_Bridge::gripper_sub_cb(const std_msgs::BoolConstPtr &gripper){
    _gripper_closed = gripper->data;
}

void DVRK_Bridge::gripper_state_sub_cb(const sensor_msgs::JointStateConstPtr &state){
    if(gripperFcnHandle._is_set){
        gripperFcnHandle.fcn_handle(*state);
    }
}

void DVRK_Bridge::run(){
    while (n->ok() && _on){
        cb_queue.callAvailable();
        run_loop_rate->sleep();
        if(_start_pubs == true){
            switch (activeState) {
            case DVRK_POSITION_JOINT:
                joint_pub.publish(cmd_joint);
                break;
            case DVRK_POSITION_CARTESIAN:
                pose_pub.publish(cmd_pose.pose);
                break;
            case DVRK_EFFORT_CARTESIAN:
                force_pub.publish(cmd_wrench.wrench);
                break;
            default:
                break;
            }
        }
    }
}

void DVRK_Bridge::set_cur_mode(const std::string &state, bool lock_ori){
    for(std::map<ARM_STATES, std::string>::iterator it = stateMap.begin(); it != stateMap.end() ; ++it){
        if(strcmp(state.c_str(), it->second.c_str()) == 0){
            state_cmd.data = state;
            state_pub.publish(state_cmd);
            if(it->first == DVRK_EFFORT_CARTESIAN){
                std_msgs::Bool lock;
                lock.data = lock_ori;
                force_orientation_lock_pub.publish(lock);
            }
            usleep(100000);
        }
    }
    _start_pubs = false;
}

void DVRK_Bridge::set_cur_pose(const geometry_msgs::PoseStamped &pose){
    cmd_pose = pose;
    activeState = DVRK_POSITION_CARTESIAN;
    _start_pubs = true;
}

void DVRK_Bridge::set_cur_wrench(const geometry_msgs::Wrench &wrench){
    cmd_wrench.wrench = wrench;
    activeState = DVRK_EFFORT_CARTESIAN;
    _start_pubs = true;
    wrench_loop_max_rate->sleep();
}

void DVRK_Bridge::set_cur_joint(const sensor_msgs::JointState &jnt_state){
    cmd_joint = jnt_state;
    activeState = DVRK_POSITION_JOINT;
    _start_pubs = true;
}

void DVRK_Bridge::_rate_sleep(){
    run_loop_rate->sleep();
}

bool DVRK_Bridge::_is_available(){
    if (pose_sub.getNumPublishers() > 0){
        return true;
    }
    else{
        return false;
    }

}

void DVRK_Bridge::get_arms_from_rostopics(std::vector<std::string> &arm_names){
    ros::M_string s;
    ros::init(s, "dvrk_arm_node");
    if (ros::master::check()){
        std::string armR, armL, checkR, checkL;
        armR = "MTMR";
        armL = "MTML";
        checkR = std::string("/dvrk/" + armR + "/status");
        checkL = std::string("/dvrk/" + armL + "/status");
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        for(int i = 0 ; i < topics.size() ; i++){
            if(strcmp(topics[i].name.c_str(), checkR.c_str()) == 0){
                arm_names.push_back(armR);
            }
            if(strcmp(topics[i].name.c_str(), checkL.c_str()) == 0){
                arm_names.push_back(armL);
            }
        }
    }
}

bool DVRK_Bridge::_in_effort_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_effort_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::_in_cart_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_cart_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::_in_jnt_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_jnt_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Bridge::shutDown(){
    _on = false;
//    ros::shutdown();
    usleep(100000);
    loop_thread->interrupt();
//    loop_thread.reset();

    std::cerr<<"Shutdown called for: "<< arm_name <<std::endl;
    return true;
}

DVRK_Bridge::~DVRK_Bridge(){
    ros::shutdown();
    std::cerr << "CLOSING DVRK_BRIDGE" << std::endl;
}
