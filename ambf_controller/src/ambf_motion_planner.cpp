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
 * @brief      Constructs the AMBFRavenPlanner object.
 */
AMBFRavenPlanner::AMBFRavenPlanner()
{
	homed = false;
	mode  = AMBFCmdMode::freefall;
	state.updated = false;
	command.updated = false;
	command.type = AMBFCmdType::_null;

	state.jp.resize(AMBFDef::raven_joints);
	command.js.resize(AMBFDef::raven_joints);
}


/**
 * @brief      Destroys the AMBFRavenPlanner object.
 */
AMBFRavenPlanner::~AMBFRavenPlanner()
{

}



/**
 * @brief      Go back to home pose.
 *
 * @return     homed check.
 */
bool AMBFRavenPlanner::go_home(bool first_entry, int arm)
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

	command.type 	= AMBFCmdType::_jp;
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
bool AMBFRavenPlanner::sine_dance(bool first_entry, int arm)
{
	static int count = 0;
	static vector<int> rampup_count = {0,0};

	float speed        = 1.00/AMBFDef::loop_rate;
	float rampup_speed = 0.05/AMBFDef::loop_rate;

	if(first_entry || !homed)
	{
		if(first_entry)
		{
			count = 0;
			rampup_count[arm] = 0;
		}
		go_home(first_entry,arm);
	}
	else
	{
		// start actual sinosoid dance
		for(int i=0; i<AMBFDef::raven_joints; i++)
		{
			float offset = (i+arm)*M_PI/2;
			float rampup = min((double)rampup_speed*rampup_count[arm],(double)1.0);
			command.js[i] = rampup*AMBFDef::dance_scale_joints[i]*sin(speed*count+offset)+AMBFDef::home_joints[i];
			rampup_count[arm] ++;
		}

		count ++;
		command.type 	= AMBFCmdType::_jp;
		command.updated = true;
		state.updated   = false;
	}

	return true;

}

//===========================================================================

/**
 * @brief      Constructs the AMBFCameraPlanner object.
 */
AMBFCameraPlanner::AMBFCameraPlanner()
{
	homed = false;
	found_home = false;
	mode  = AMBFCmdMode::freefall;
	state.updated = false;
	command.updated = false;
	command.type = AMBFCmdType::_null;
}



/**
 * @brief      Sets the home pose for the camera.
 *
 * @return     success
 */
bool AMBFCameraPlanner::set_home()
{
	if(!found_home)
	{
		home_pose.setOrigin(state.cp.getOrigin());
	   	home_pose.setRotation(state.cp.getRotation());
		found_home = true;
	}
	return found_home;
}


/**
 * @brief      Destroys the AMBFCameraPlanner object.
 */
AMBFCameraPlanner::~AMBFCameraPlanner()
{

}



/**
 * @brief      Go back to home pose.
 *
 * @return     homed check.
 */
bool AMBFCameraPlanner::go_home(bool first_entry, int cam)
{
	static int count = 0;
	static vector<tf::Transform> start_cp(AMBFDef::camera_count);

	if(first_entry)
	{
		start_cp[cam].setOrigin(state.cp.getOrigin());
		start_cp[cam].setRotation(state.cp.getRotation());
		count = 0;
	}

	float duration = 10;  // seconds
	int iterations = duration * AMBFDef::camera_count * AMBFDef::loop_rate;
	float scale = min((double)(1.0*count/iterations),(double)1.0);

	command.cp.setOrigin((1-scale)*start_cp[cam].getOrigin() + scale*home_pose.getOrigin());
	command.cp.setRotation(start_cp[cam].getRotation().slerp(home_pose.getRotation(),scale));


	float pos_diff = (state.cp.getOrigin() - home_pose.getOrigin()).length();
	float ori_diff = fabs(state.cp.getRotation().angleShortestPath(home_pose.getRotation()));

	if(pos_diff < 0.1 && ori_diff < 0.1) 
	{
		homed = true;
		mode = AMBFCmdMode::freefall; // change back to static mode
		command.type 	= AMBFCmdType::_null;
		command.updated = false;
		state.updated   = false;
		ROS_INFO("Camera%d back to home. (Stopped sending motion commands!)",cam+1);
	}
	else
	{
		homed = false;
		command.type 	= AMBFCmdType::_cp;
		command.updated = true;
		state.updated   = false;	
		count ++;
	}

	return homed;
}



/**
 * @brief      Do a little randomly wandering dance move.
 *
 * @param[in]  arm   The arm
 *
 * @return     success
 */
bool AMBFCameraPlanner::wander_dance(bool first_entry, int cam)
{
/*	static int count = 0;
	static vector<int> rampup_count = {0,0};

	float speed        = 1.00/AMBFDef::loop_rate;
	float rampup_speed = 0.05/AMBFDef::loop_rate;

	if(first_entry || !homed)
	{
		if(first_entry)
		{
			count = 0;
			rampup_count[arm] = 0;
		}
		go_home(first_entry,arm);
	}
	else
	{
		// start actual sinosoid dance
		for(int i=0; i<AMBFDef::raven_joints; i++)
		{
			float offset = (i+arm)*M_PI/2;
			float rampup = min((double)rampup_speed*rampup_count[arm],(double)1.0);
			command.js[i] = rampup*AMBFDef::dance_scale_joints[i]*sin(speed*count+offset)+AMBFDef::home_joints[i];
			rampup_count[arm] ++;
		}

		count ++;
		command.type 	= _jp;
		command.updated = true;
		state.updated   = false;
	}
*/
	return true;

}