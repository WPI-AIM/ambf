//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (www.aimlab.wpi.edu)

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

    \author:    Melody Su
    \date:      April, 2019
    \version:   $
*/
//===========================================================================

#ifndef AMBFCONTROLLER_H
#define AMBFCONTROLLER_H

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
#include <map>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"

enum AMBFCmdType {_jp, _jw, _cp, _cw};

class AMBFController{

private:
    int lr_;
    int n_joints;
    ros::NodeHandle nh_; 

    vector<ros::Publisher>  raven_pubs; // command
    vector<ros::Subscriber> raven_subs; // command

    vector<float> js_command_L;   // raven joint space command (position or wrench: depending on AMBFCmdType)
    vector<float> js_command_R;

    tf::Vector3  cf_command_L;        // raven cartesian force command      
    tf::Vector3  cf_command_R;
    tf::Vector3  ct_command_L;        // raven cartesian torque command      
    tf::Vector3  ct_command_R;

    vector<float> jp_state_L;     // raven joint space position state
    vector<float> jp_state_R;

    tf::Transform cp_state_L;        // raven catesian space position state
    tf::Transform cp_state_R;
    tf::Vector3  cf_state_L;        // raven cartesian force state     
    tf::Vector3  cf_state_R;
    tf::Vector3  ct_state_L;        // raven cartesian torque state      
    tf::Vector3  ct_state_R;

    tf::Vector3      zero_vec;
    vector<float>  zero_joints;
    vector<unsigned char>   true_joints;
    vector<unsigned char>   false_joints;

    string sub_append;
    string pub_append;  
    string raven_append;

    string L_append;
    string R_append;

    bool state_L_updated;
    bool state_R_updated;
    bool command_L_updated;
    bool command_R_updated;

    AMBFCmdType command_type;


public:
    AMBFController(int, char**);

    bool init_sys();
    bool init_ros(int, char**);
    bool sys_run();
    bool raven_command_pb();
    bool raven_motion_planning();
    void raven_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>&,  const string& );
    bool reset_commands();

    ~AMBFController();
};

#endif