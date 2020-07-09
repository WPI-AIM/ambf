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

#ifndef AMBFMOTIONPLANNER_H
#define AMBFMOTIONPLANNER_H

#include "ambf_defines.h"

class AMBFCameraPlanner{

private:
	bool           homed;
	bool 		   found_home;
	tf::Transform  home_pose;

public:

	AMBFCmd command;       // camera command structure
    AMBFSta state;  	   // camera state structure
 	AMBFCmdMode mode;      // camera command mode

    AMBFCameraPlanner();
    ~AMBFCameraPlanner();
    bool set_home();
    bool go_home(bool,int);   		// cp command will be used
    bool wander_dance(bool, int);  	// cp command will be used
};


//===========================================================================
class AMBFRavenPlanner{

private:
	bool homed;

public:

	AMBFCmd command;       // raven command structure
    AMBFSta state;  	   // raven state structure
 	AMBFCmdMode mode;      // Raven command mode

    AMBFRavenPlanner();
    ~AMBFRavenPlanner();

    bool fwd_kinematics(int, vector<float>, tf::Transform&);
    tf::Transform fwd_trans(int, int, vector<float>, vector<float>, vector<float>, vector<float>);
    bool inv_kinematics(int, tf::Transform&, float, vector<float>&);
    bool apply_joint_limits(vector<float>& , bool&);
    bool find_best_solution(vector<float>,vector<vector<float>>,vector<bool>,int&,float&);
	bool kinematics_show(int, bool);
	bool joint_to_dhvalue(vector<float>, vector<float>&, int);
	bool dhvalue_to_joint(vector<float>, vector<float>&, float, int);
	bool check_incr_safety(vector<float>, vector<float>&, int);

    bool go_home(bool,int);   			// jp command will be used
    bool sine_dance(bool, int);     	// jp command will be used
    bool trace_cube(bool, int, bool);   // cp command will be used

};


// Raven Note:
//
// if(command.type == _jp || command.type == _jw)
// update command.js
//
// if(command.type == _cp)
// do inverse kinematics
// update command.js
//
// if(command.type == _cw)
// update command.cf
// update command.ct
//
// Remember to set:
// command.type 	= _???;
// command.updated = true;
// state.updated   = false;

#endif
