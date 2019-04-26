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


#include "ambf_motion_planner.h"

/**
 * @brief      Constructs the AMBFPLanner object.
 */
AMBFPlanner::AMBFPlanner()
{
	homed = false;
	mode  = AMBFCmdMode::freefall;
	state.updated = false;
	command.updated = false;
	command.type = _null;

	state.jp.resize(AMBFDef::raven_joints);
	command.js.resize(AMBFDef::raven_joints);
}


/**
 * @brief      Destroys the AMBFPlanner object.
 */
AMBFPlanner::~AMBFPlanner()
{

}



/**
 * @brief      Go back to home pose.
 *
 * @return     homed check.
 */
bool AMBFPlanner::go_home(bool first_entry, int arm)
{
	static int count = 0;
	static vector<vector<float>> start_jp(AMBFDef::raven_arms, vector<float>(AMBFDef::raven_joints));
	static vector<vector<float>> delta_jp(AMBFDef::raven_arms, vector<float>(AMBFDef::raven_joints));

	if(first_entry)
	{
		for(int i=0; i<AMBFDef::raven_joints; i++)
		{
			start_jp[arm][i] = state.jp[i];
			delta_jp[arm][i] = AMBFDef::home_joints[i] - state.jp[i];
		}
		count = 0;
	}

	float duration = 10;  // seconds
	int iterations = duration * AMBFDef::raven_arms * AMBFDef::loop_rate;
	float scale = min((double)(1.0*count/iterations),(double)1.0);

	vector<float> diff_jp(AMBFDef::raven_joints);
	for(int i=0; i<AMBFDef::raven_joints; i++)
	{
		command.js[i] = scale * delta_jp[arm][i] + start_jp[arm][i];
		diff_jp[i] = fabs(AMBFDef::home_joints[i] - state.jp[i]);
	}

	float max_value = *max_element(diff_jp.begin(), diff_jp.end());
	if(max_value < 0.1) homed = true;
	else				homed = false;

	command.type 	= _jp;
	command.updated = true;
	state.updated   = false;	
	count ++;

	return homed;
}



/**
 * @brief      Do a little sinosoidal dance move.
 *
 * @param[in]  arm   The arm
 *
 * @return     success
 */
bool AMBFPlanner::sine_dance(bool first_entry, int arm)
{
	// approach 1: (best performed)
	static int count = 0;
	float scale = 0.3;
	float speed = 1.0/AMBFDef::loop_rate;

	for(int i=0; i<AMBFDef::raven_joints; i++)
	{
		float offset = (i+arm)*M_PI/2;
			
		if(i == 2)
		{
			command.js[i] = 0.2*scale*sin(count*speed+offset)+AMBFDef::home_joints[i];
		}
		else if(i == 4)
		{
			command.js[i] = 3.0*scale*sin(count*speed+offset)+AMBFDef::home_joints[i];	
		}
		else
		{
			command.js[i] = scale*sin(count*speed+offset)+AMBFDef::home_joints[i];
		}
	}

	/* approach 2: (stable but not practical under joint limit constraints)
	static int count = 0;
	static vector<vector<int>> offset(AMBFDef::raven_arms, vector<int>(AMBFDef::raven_joints));
	static vector<float> delta_jp(AMBFDef::raven_joints);

	int sign = arm * 2 - 1;
	float scale = 0.1/AMBFDef::loop_rate;

	if(first_entry)
	{
		for(int i=0; i<AMBFDef::raven_joints; i++)
		{
			delta_jp[i] = AMBFDef::max_joints[i] - AMBFDef::min_joints[i];
			offset[arm][i] = asin(((state.jp[i]-AMBFDef::min_joints[i])*2/delta_jp[i])-1)/scale;
		}
		count = 0;
	}

	for(int i=0; i<2; i++)
	{
		command.js[i] = (sin((count*sign+offset[arm][i])*scale)+1)*delta_jp[i]/2 + AMBFDef::min_joints[i];
		command.js[i] = min((double)command.js[i],(double)0.9*AMBFDef::max_joints[i]);
		command.js[i] = max((double)command.js[i],(double)0.9*AMBFDef::min_joints[i]);
	}*/

	/* approach 3: (not great, sometimes unstable)	
	float scale = 1.0;
	float duration = 10;  // seconds
	int iterations = duration * AMBFDef::raven_arms * AMBFDef::loop_rate;
	static int count = 0;
	static vector<vector<float>> start_jp(AMBFDef::raven_arms, vector<float>(AMBFDef::raven_joints));
	if(first_entry)
	{
		for(int i=0; i<AMBFDef::raven_joints; i++)
		{
			start_jp[arm][i] = state.jp[i];
		}
		count = 0;
	}

	for(int i=0; i<AMBFDef::raven_joints; i++)
	{
		command.js[i] = sin(count/iterations)*scale + start_jp[arm][i];
		command.js[i] = min((double)command.js[i],(double)AMBFDef::max_joints[i]);
		command.js[i] = max((double)command.js[i],(double)AMBFDef::min_joints[i]);
	}*/

	count ++;
	command.type 	= _jp;
	command.updated = true;
	state.updated   = false;

	return true;

}