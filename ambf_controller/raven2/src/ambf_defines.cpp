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

#include "ambf_defines.h"

const int             AMBFDef::V                      = -1;
const int             AMBFDef::camera_count           = 3;
const int             AMBFDef::raven_joints           = 7;
const int             AMBFDef::raven_arms             = 2;
const int             AMBFDef::raven_iksols           = 8;
const int             AMBFDef::loop_rate              = 1000;                            // Hz
const float           AMBFDef::safe_ori_incr          = M_PI / (6 * AMBFDef::loop_rate); // rad
const float           AMBFDef::safe_pos_incr          = 0.5 / AMBFDef::loop_rate;  // m
// The loop rate of the AMBF simulator:     2000 Hz
// The loop rate of the AMBF python client: 1000 Hz
// The loop rate of the Raven source code:  1000 Hz

const string          AMBFDef::env_append             = "/ambf/env/";
const string          AMBFDef::sub_append             = "/State";                           // place holder for namescpace strings
const string          AMBFDef::pub_append             = "/Command";
const string          AMBFDef::raven_append           = "raven_2";
const vector<string>  AMBFDef::arm_append             = {"/base_link_L", "/base_link_R"};   // left arm 0 & right arm 1
const vector<string>  AMBFDef::cam_append             = {"cam_1", "cam_2", "cam_3"};        // each camera name
const vector<string>  AMBFDef::axes_name              = {"X", "Y", "Z"};
const vector<string>  AMBFDef::sign_name              = {"-", "+"};

const tf::Vector3             AMBFDef::zero_vec             = tf::Vector3(0,0,0);           // place holder for frequently used arrays
const vector<float>           AMBFDef::zero_joints          = {          0,           0,           0,        0,       0,        0,         0};
const vector<float>           AMBFDef::max_joints           = {       M_PI,        M_PI,        0.10,     M_PI,       2, M_PI*3/4,  M_PI*3/4};
const vector<float>           AMBFDef::min_joints           = {      -M_PI,       -M_PI,       -0.17,    -M_PI,      -2,        0,         0};
const vector<float>           AMBFDef::home_joints          = {   M_PI*1/3,    M_PI*3/5,       -0.09, M_PI*3/4,       0,   M_PI/6,    M_PI/6};
const vector<float>           AMBFDef::dance_scale_joints   = {        0.3,         0.3,        0.06,      0.3,     1.2,   M_PI/6,    M_PI/6};
const vector<unsigned char>   AMBFDef::true_joints          = {          1,           1,           1,        1,       1,        1,         1};
const vector<unsigned char>   AMBFDef::false_joints         = {          0,           0,           0,        0,       0,        0,         0};

const vector<vector<float>>   AMBFDef::raven_joint_limit    = {{         0,    M_PI*1/4,       -0.17,  -M_PI*2,      -2,       -2,        -2},
                                                               {  M_PI*1/2,    M_PI*3/4,        0.10,   M_PI*2,       2,        2,         2}};  // TODO: finalize this
const vector<vector<float>>   AMBFDef::raven_dh_alpha       = {{         0, -75 Deg2Rad, 128 Deg2Rad,        0,  M_PI/2,   M_PI/2,         0},
                                                               {      M_PI,  75 Deg2Rad,  52 Deg2Rad,        0,  M_PI/2,   M_PI/2,         0}};
const vector<vector<float>>   AMBFDef::raven_dh_a           = {{         0,           0,           0,        0,       0,    0.013,         0},
                                                               {         0,           0,           0,        0,       0,    0.013,         0}};
const vector<vector<float>>   AMBFDef::raven_dh_d           = {{         0,           0,           V,    -0.47,       0,        0,         0},
                                                               {         0,           0,           V,    -0.47,       0,        0,         0}};
const vector<vector<float>>   AMBFDef::raven_dh_theta       = {{         V,           V,      M_PI/2,        V,       V,        V,         0},
                                                               {         V,           V,     -M_PI/2,        V,       V,        V,         0}};

const vector<float>           AMBFDef::raven_ikin_param     = { (float)sin(AMBFDef::raven_dh_alpha[0][1]),     // 0: GM1
                                                                (float)cos(AMBFDef::raven_dh_alpha[0][1]),     // 1: GM2
                                                                (float)sin(AMBFDef::raven_dh_alpha[1][2]),     // 2: GM3
                                                                (float)cos(AMBFDef::raven_dh_alpha[1][2]),     // 3: GM4
                                                                AMBFDef::raven_dh_d[0][3]                ,     // 4: d4 = -0.47
                                                                AMBFDef::raven_dh_a[0][5]               };     // 5: Lw = 0.013

const vector<tf::Transform>   AMBFDef::raven_T_B0           = { tf::Transform(tf::Matrix3x3(0,0,1,0,-1,0,1,0,0),tf::Vector3(0.30071, 0.061, -0.007)),
                                                                tf::Transform(tf::Matrix3x3(0,0,-1,0,1,0,1,0,0),tf::Vector3(-0.30071, 0.061, -0.007))};
const tf::Transform           AMBFDef::raven_T_CB           =   tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-M_PI/2),tf::Vector3(0,0,0));


// Raven joints:
// joint -: 0_link-base_link_L:             fixed
// joint 0: base_link_L-link1_L:            revolute        (shoulder)              range: -pi~pi
// joint 1: link1_L-link2_L:                revolute        (elbow)                 range: -pi~pi
// joint 2: link2_L-link3_L:                prismatic       (tool plate up/down)    range: -0.17~0.1
// joint 3: link3_L-instrument_shaft_L:     continuous      (tool shaft roll)       range: no limit
// joint 4: instrument_shaft_L-wrist_L:     revolute        (tool wrist)            range: -2~2
// joint 5: wrist_L-grasper1_L:             revolute        (grasper left)          range: -2~2
// joint 6: wrist_L-grasper2_L:             revolute        (grasper right)         range: -2~2
