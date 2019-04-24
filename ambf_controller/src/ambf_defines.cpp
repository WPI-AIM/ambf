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

#include "ambf_defines.h"

const int             AMBFDef::raven_joints           = 7;
const int             AMBFDef::raven_arms             = 2;
const int             AMBFDef::loop_rate              = 1000; // Hz
// The loop rate of the AMBF simulator:     2000 Hz
// The loop rate of the AMBF python client: 1000 Hz
// The loop rate of the Raven source code:  1000 Hz
                                                     
const string          AMBFDef::sub_append             = "/State";                           // place holder for namescpace strings
const string          AMBFDef::pub_append             = "/Command";  
const string          AMBFDef::raven_append           = "/ambf/env/raven_2";
const vector<string>  AMBFDef::arm_append             = {"/base_link_L", "/base_link_R"};     // left arm 0 & right arm 1

const tf::Vector3             AMBFDef::zero_vec       = tf::Vector3(0,0,0);           // place holder for frequently used arrays
const vector<float>           AMBFDef::zero_joints    = {0,0,0,0,0,0,0};
const vector<unsigned char>   AMBFDef::true_joints    = {1,1,1,1,1,1,1};
const vector<unsigned char>   AMBFDef::false_joints   = {0,0,0,0,0,0,0};

