//===========================================================================
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

    \author:    Melody Su
    \date:      April, 2019
    \version:   1.0$
*/
//===========================================================================

#ifndef AMBFCONTROLLER_H
#define AMBFCONTROLLER_H

#include "ambf_motion_planner.h"

class AMBFController{

private:

    mutex _mutex;
    bool print_menu;
    bool debug_mode;
    ros::NodeHandle nh_;

    vector<ros::Publisher>     raven_pubs;      // raven command publisher
    vector<ros::Subscriber>    raven_subs;      // raven state subscriber
    vector<AMBFRavenPlanner>   raven_planner;   // raven motion planner

    vector<ros::Publisher>     camera_pubs;     // camera command publisher
    vector<ros::Subscriber>    camera_subs;     // camera state subscriber
    vector<AMBFCameraPlanner>  camera_planner;  // camera motion planner

public:

    AMBFController(int, char**);

    bool init_sys();
    bool init_ros(int, char**);
    void sys_run(); // the system process function
    void csl_run(); // the console process function

    bool raven_first_pb();
    bool raven_command_pb();
    bool camera_command_pb();
    void raven_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>&,  const string& );
    void camera_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>&,  const string& );

    bool motion_planning();
    bool reset_command();

    int get_key();
    bool show_menu();

    ~AMBFController();
};

#endif
