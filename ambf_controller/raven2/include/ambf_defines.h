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
#ifndef AMBFDEFINES_H
#define AMBFDEFINES_H

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
#include <thread>
#include <map>
#include <termios.h>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"

#define Deg2Rad   * M_PI/180
#define Rad2Deg   *180/M_PI
#define Eps       1.0e-05

enum AMBFCmdType {_jp, _jw, _cp, _cw, _null};
enum AMBFCmdMode {freefall, homing, dancing, cube_tracing}; // cube_tracing: only for raven

struct AMBFCmd
{
	bool updated;		// raven command updated
	AMBFCmdType type;	// raven command type
  tf::Transform cp;
	vector<float> js;  	// raven joint space command (position or wrench: depending on AMBFCmdType)
	tf::Vector3   cf;	// raven cartesian force command
	tf::Vector3   ct;	// raven cartesian torque command
	bool cn_flag;		// raven state: children name flag
	bool jn_flag;		// raven state: joint name flag
	bool jp_flag;		// raven state: joint position flag
};

struct AMBFSta
{
	bool updated;			 // raven state updated

	vector<float>   jp;        // raven joint space position state
    tf::Transform cp;        // raven catesian space position state
    tf::Vector3   cf;        // raven cartesian force state
    tf::Vector3   ct;        // raven cartesian torque state
};


class AMBFDef {

public:

  static const int         V;
  static const int         camera_count;
  static const int 			   raven_joints;
  static const int 			   raven_arms;
  static const int         raven_iksols;
  static const int 			   loop_rate;
  static const float       safe_ori_incr;
  static const float       safe_pos_incr;
  static const string          sub_append;          // place holder for namescpace strings
  static const string          pub_append;
  static const string          raven_append;
  static const string          env_append;
  static const vector<string>  arm_append;          // left arm 0 & right arm 1
  static const vector<string>  cam_append;          // each camera name
  static const vector<string>  axes_name;
  static const vector<string>  sign_name;

  static const tf::Vector3             zero_vec;    // place holder for frequently used arrays
  static const vector<float>           zero_joints;
  static const vector<float>           max_joints;
  static const vector<float>           min_joints;
  static const vector<float>           home_joints;
  static const vector<float>           dance_scale_joints;
  static const vector<unsigned char>   true_joints;
  static const vector<unsigned char>   false_joints;

  static const vector<vector<float>>   raven_joint_limit;
  static const vector<vector<float>>   raven_dh_alpha;
  static const vector<vector<float>>   raven_dh_a;
  static const vector<vector<float>>   raven_dh_d;
  static const vector<vector<float>>   raven_dh_theta;
  static const vector<float>           raven_ikin_param;
  static const vector<tf::Transform>   raven_T_B0;  // raven base frame to raven zero frame (have seperate ones for each arm)
  static const tf::Transform           raven_T_CB;  // raven crtk frame to base frame (have seperate ones for each arm)
};

#endif
