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

#include "ambf_defines.h"

class AMBFController{

private:

    int lr_;
    int n_arms;
    int n_joints;
    mutex _mutex;
    ros::NodeHandle nh_; 

    vector<ros::Publisher>  raven_pubs;  // command publisher
    vector<ros::Subscriber> raven_subs;  // state subscriber

    vector<AMBFCmd> raven_command;       // raven command structure
    vector<AMBFSta> raven_state;         // raven state structure

    string          sub_append;          // place holder for namescpace strings
    string          pub_append;  
    string          raven_append;
    vector<string>  arm_append;          // left arm 0 & right arm 1

    tf::Vector3             zero_vec;    // place holder for frequently used arrays
    vector<float>           zero_joints;
    vector<unsigned char>   true_joints;
    vector<unsigned char>   false_joints;

public:

    AMBFController(int, char**);

    bool init_sys();
    bool init_ros(int, char**);
    bool sys_run();
    bool raven_command_pb();
    bool raven_motion_planning();
    void raven_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>&,  const string& );
    bool reset_command();

    ~AMBFController();
};

#endif