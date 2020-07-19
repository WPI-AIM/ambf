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
 * @brief      change raw joint values to dh convension
 *
 * @param[in]  joint    The joint   (length 7)
 * @param      dhvalue  The dhvalue (length 6)
 * @param[in]  arm      The arm
 *
 * @return     success
 */
bool AMBFRavenPlanner::joint_to_dhvalue(vector<float> joint, vector<float>& dhvalue, int arm)
{
	bool success = false;
	if(arm < 0 || arm >= AMBFDef::raven_arms || joint.size() != AMBFDef::raven_joints)
	{
		ROS_ERROR("Invalid input to function joint to dhvalue.");
		return success;
	}

	for(int i=0; i<AMBFDef::raven_joints-1; i++)
	{
		if(i != 2)
		{
			if(i == 5)
			{
				if(arm == 0)
					dhvalue[i] =  (joint[i] - joint[i+1]);  // red(left) - black(right)
				else
					dhvalue[i] = -(joint[i] - joint[i+1]);  // black(left) - red(right)
			}
			else
				dhvalue[i] = joint[i];

			while(dhvalue[i] > M_PI)   dhvalue[i] -= 2*M_PI;
			while(dhvalue[i] < -M_PI)  dhvalue[i] += 2*M_PI;
		}
		else
			dhvalue[i] = joint[i];
	}

	success = true;
	return success;
}




/**
 * @brief      change dh convension to raw joint values
 *
 * @param[in]  dhvalue  The dhvalue (length 6)
 * @param      joint    The joint   (length 7)
 * @param[in]  gangle   The gangle
 * @param[in]  arm      The arm
 *
 * @return     limited  The joint values were saturated
 */
bool AMBFRavenPlanner::dhvalue_to_joint(vector<float> dhvalue, vector<float>& joint, float gangle, int arm)
{
	bool limited;

	// set desired joint positions
	for(int i=0; i<AMBFDef::raven_joints-1; i++)
	{
		if(i != 2)
		{
			if(i == 5)
			{
				if(arm == 0)
				{
					joint[i+1] =  (-dhvalue[i] + gangle) / 2;  // black (right)
					joint[i] =    (dhvalue[i] + gangle) / 2;  // red (left)
				}
				else
				{
					joint[i] =    (-dhvalue[i] + gangle) / 2; // red (right)
					joint[i+1] =  (dhvalue[i] + gangle) / 2;  // black (left)
				}

			}
			else
				joint[i] = dhvalue[i];

			while(joint[i] > M_PI)   joint[i] -= 2*M_PI;
			while(joint[i] < -M_PI)  joint[i] += 2*M_PI;
		}
		else
			joint[i] = dhvalue[i];
	}

	// check joint limits for saturating
	apply_joint_limits(joint, limited);

	return limited;
}




/**
 * @brief      Check safety constraints for joint level increments
 *
 * @param[in]  curr_raw  The curr raw joint position
 * @param      next_raw  The next raw joint position
 *
 * @return     success
 */
bool AMBFRavenPlanner::check_incr_safety(vector<float> curr_raw, vector<float>& next_raw, int length)
{
	bool success = false;
	vector<float> curr_jp(AMBFDef::raven_joints);
	vector<float> next_jp(AMBFDef::raven_joints);

	if(length == AMBFDef::raven_joints-1)
	{
		dhvalue_to_joint(curr_raw, curr_jp, 0, 0);
		dhvalue_to_joint(next_raw, next_jp, 0, 0);
	}
	else if(length == AMBFDef::raven_joints)
	{
		for(int i=0;i<AMBFDef::raven_joints;i++)
		{
			curr_jp[i] = curr_raw[i];
			next_jp[i] = next_raw[i];
		}
	}

	for(int i=0; i<length; i++)
	{
		if(i == 2)
		{
			if(next_jp[i] > curr_jp[i])
			{
				next_jp[i] = curr_jp[i] + min(fabs(next_jp[i]-curr_jp[i]),AMBFDef::safe_pos_incr);
			}
			else
			{
				next_jp[i] = curr_jp[i] - min(fabs(next_jp[i]-curr_jp[i]),AMBFDef::safe_pos_incr);
			}
		}
		else
		{
			if(next_jp[i] > curr_jp[i])
			{
				next_jp[i] = curr_jp[i] + min(fabs(next_jp[i]-curr_jp[i]),AMBFDef::safe_ori_incr);
			}
			else
			{
				next_jp[i] = curr_jp[i] - min(fabs(next_jp[i]-curr_jp[i]),AMBFDef::safe_ori_incr);
			}
		}

	}

	if(length == AMBFDef::raven_joints-1)
	{
		joint_to_dhvalue(next_jp, next_raw, 0);
		success = true;
	}
	else if(length == AMBFDef::raven_joints)
	{
		success = true;
	}

	return success;
}




/**
 * @brief      Raven forward kinematics calculation
 *
 * @param[in]  arm        The arm
 * @param[in]  input_jp   The input joitn position
 * @param      output_cp  The output cartesian end-effector pose
 *
 * @return     success
 */
bool AMBFRavenPlanner::fwd_kinematics(int arm, vector<float> input_jp, tf::Transform& output_cp)
{
	bool success = false;

	vector<float> dh_alpha(6);
	vector<float> dh_theta(6);
	vector<float> dh_a(6);
	vector<float> dh_d(6);

	// convert the joint angles to DH theta convention
	vector<float> jp_dh(6);

	if(!joint_to_dhvalue(input_jp, jp_dh, arm))
	{
		ROS_ERROR("Something went wrong in joint to dhvalue conversion.");
		return success;
	}

	for(int i=0; i<AMBFDef::raven_joints-1; i++)
	{
		if(i == 2)
		{
			dh_d[i] = jp_dh[i];
			dh_theta[i] = AMBFDef::raven_dh_theta[arm][i];
		}
		else
		{
			dh_d[i] = AMBFDef::raven_dh_d[arm][i];
			dh_theta[i] = jp_dh[i];
		}

		dh_alpha[i] = AMBFDef::raven_dh_alpha[arm][i];
		dh_a[i] = AMBFDef::raven_dh_a[arm][i];
	}

	// computes forward kinematics
	output_cp = AMBFDef::raven_T_CB * AMBFDef::raven_T_B0[arm] * fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d);

	success = true;
	return success;
}



/**
 * @brief      computes the accumulative multiplication of homogeneous matrices
 *             under the dh convension from frame a to frame b.
 *
 * @param[in]  a         start frame
 * @param[in]  b         goal frame
 * @param      dh_alpha  The dh parameter alpha
 * @param      dh_theta  The dh parameter theta
 * @param      dh_a      The dh parameter a
 * @param      dh_d      The dh parameter d
 *
 * @return     The resultant transformation matrix
 */
tf::Transform AMBFRavenPlanner::fwd_trans(int a, int b, vector<float> dh_alpha, vector<float> dh_theta, vector<float> dh_a, vector<float> dh_d)
{
	if ((b <= a) || b == 0)
		ROS_ERROR("Invalid start/end indices.");

	double xx = cos(dh_theta[a]),
	       xy = -sin(dh_theta[a]),
	       xz = 0;

	double yx = sin(dh_theta[a]) * cos(dh_alpha[a]),
	       yy = cos(dh_theta[a]) * cos(dh_alpha[a]),
	       yz = -sin(dh_alpha[a]);

	double zx = sin(dh_theta[a]) * sin(dh_alpha[a]),
	       zy = cos(dh_theta[a]) * sin(dh_alpha[a]),
	       zz = cos(dh_alpha[a]);

	double px = dh_a[a];
	double py = -sin(dh_alpha[a]) * dh_d[a];
	double pz = cos(dh_alpha[a]) * dh_d[a];

	tf::Transform xf;
	xf.setBasis(tf::Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz));
	xf.setOrigin(tf::Vector3(px, py, pz));

	// recursively find transforms for following links
	if (b > a + 1)
		xf *= fwd_trans(a + 1, b, dh_alpha, dh_theta, dh_a, dh_d);

	return xf;
}


/**
 * @brief      Raven inverse kinematics calculation
 *
 * @return     success
 */
bool AMBFRavenPlanner::inv_kinematics(int arm, tf::Transform& input_cp, float input_gangle, vector<float>& output_jp)
{
	bool success = false;

	tf::Transform xf =  (AMBFDef::raven_T_CB * AMBFDef::raven_T_B0[arm]).inverse() * input_cp;

	vector<vector<float>> iksol(AMBFDef::raven_iksols, vector<float>(AMBFDef::raven_joints-1));
	vector<bool>ikcheck(AMBFDef::raven_iksols);

	vector<float> dh_alpha(6);
	vector<float> dh_theta(6);
	vector<float> dh_a(6);
	vector<float> dh_d(6);

	for(int i=0; i<AMBFDef::raven_joints-1;i++)
	{
		dh_alpha[i] = AMBFDef::raven_dh_alpha[arm][i];
		dh_theta[i] = AMBFDef::raven_dh_theta[arm][i];
		dh_a[i] = AMBFDef::raven_dh_a[arm][i];
		dh_d[i] = AMBFDef::raven_dh_d[arm][i];
	}

	for (int i = 0; i < AMBFDef::raven_iksols; i++)
	{
		iksol[i] = AMBFDef::zero_joints;
		ikcheck[i] = true;
	}

	//  Step 1, Compute P5
	tf::Transform T60 = xf.inverse();
	tf::Vector3 p6rcm = T60.getOrigin();
	tf::Vector3 p05[8];

	p6rcm[2] = 0;  // take projection onto x-y plane
	for (int i = 0; i < 2; i++)
	{
		tf::Vector3 p65 = (-1 + 2 * i) * AMBFDef::raven_ikin_param [5] * p6rcm.normalize();
		p05[4 * i] = p05[4 * i + 1] = p05[4 * i + 2] = p05[4 * i + 3] = xf * p65;
	}

	//  Step 2, compute displacement of prismatic joint d3
	for (int i = 0; i < AMBFDef::raven_iksols/4; i++)
	{
		float insertion = 0;
		insertion += p05[4 * i].length();

		if (insertion <= AMBFDef::raven_ikin_param[5])
		{
		  ROS_ERROR("WARNING: Raven mechanism at RCM singularity (Lw: %f, ins: %f). IK failing.",
		            AMBFDef::raven_ikin_param[5], insertion);

		  ikcheck[4 * i + 0] = ikcheck[4 * i + 1] = false;
		  ikcheck[4 * i + 2] = ikcheck[4 * i + 3] = false;
		  success = false;
		  break;
		}
		iksol[4 * i + 0][2] = iksol[4 * i + 1][2] = -AMBFDef::raven_ikin_param[4] - insertion;
		iksol[4 * i + 2][2]= iksol[4 * i + 3][2] = -AMBFDef::raven_ikin_param[4] + insertion;
	}

	//  Step 3, calculate theta 2
	for (int i = 0; i < AMBFDef::raven_iksols; i += 2)  // p05 solutions
	{
		float z0p5 = p05[i][2];

		float cth2 = 0;
		float d = iksol[i][2] + AMBFDef::raven_ikin_param[4];
		float cth2_nom = (( z0p5 / d) + AMBFDef::raven_ikin_param[1] * AMBFDef::raven_ikin_param[3]);
		float cth2_den = (AMBFDef::raven_ikin_param[0] * AMBFDef::raven_ikin_param[2]);

		cth2 = -cth2_nom / cth2_den;

		// Smooth roundoff errors at +/- 1.
		if (cth2 > 1 && cth2 < 1 + Eps)
		  cth2 = 1;
		else if (cth2 < -1 && cth2 > -1 - Eps)
		  cth2 = -1;

		if (cth2 > 1 || cth2 < -1)
		{
		  ikcheck[i] = ikcheck[i + 1] = false;
		}
		else
		{
		  iksol[i][1] = acos(cth2);
		  iksol[i + 1][1] = -acos(cth2);
		}
	}

	//  Step 4: Compute theta 1
	for (int i = 0; i < AMBFDef::raven_iksols; i++)
	{
		if (ikcheck[i] == false) continue;

		float cth2 = cos(iksol[i][1]);
		float sth2 = sin(iksol[i][1]);
		float d = iksol[i][2] + AMBFDef::raven_ikin_param[4];
		float BB1 = sth2 * AMBFDef::raven_ikin_param[2];
		float BB2 = 0;
		tf::Matrix3x3 Bmx;  // using 3 vector and matrix bullet types for convenience.
		tf::Vector3 xyp05(p05[i]);
		xyp05[2] = 0;

		BB2 = cth2 * AMBFDef::raven_ikin_param[1]*AMBFDef::raven_ikin_param[2]
		      - AMBFDef::raven_ikin_param[0]*AMBFDef::raven_ikin_param[3];

		if (arm == 0)
		  Bmx.setValue(BB1, BB2, 0, -BB2, BB1, 0, 0, 0, 1);
		else
		  Bmx.setValue(BB1, BB2, 0,  BB2, -BB1, 0, 0, 0, 1);

		tf::Vector3 scth1 = Bmx.inverse() * xyp05 * (1 / d);
		iksol[i][0] = atan2(scth1[1], scth1[0]);
	}

	//  Step 5: get theta 4, 5, 6
	for (int i = 0; i < AMBFDef::raven_iksols; i++)
	{
		if (ikcheck[i] == false) continue;

		// compute T03:
		dh_theta[0] = iksol[i][0];
		dh_theta[1] = iksol[i][1];
		dh_d[2] = iksol[i][2];

		tf::Transform T03 = fwd_trans(0,3, dh_alpha, dh_theta, dh_a, dh_d);
		tf::Transform T36 = T03.inverse() * xf;

		float c5 = -T36.getBasis()[2][2];
		float s5 = (T36.getOrigin()[2] - AMBFDef::raven_ikin_param[4]) / AMBFDef::raven_ikin_param[5];

		// Compute theta 4:
		float c4, s4;
		if (fabs(c5) > Eps)
		{
			c4 = T36.getOrigin()[0] / (AMBFDef::raven_ikin_param[5] * c5);
			s4 = T36.getOrigin()[1] / (AMBFDef::raven_ikin_param[5] * c5);
		}
		else
		{
			c4 = T36.getBasis()[0][2] / s5;
			s4 = T36.getBasis()[1][2] / s5;
		}
		iksol[i][3] = atan2(s4, c4);

		// Compute theta 5:
		iksol[i][4] = atan2(s5, c5);

		// Compute theta 6:
		float s6, c6;
		if (fabs(s5) > Eps)
		{
			c6 = T36.getBasis()[2][0] / s5;
			s6 = -T36.getBasis()[2][1] / s5;
		}
		else
		{
			dh_theta[3] = iksol[i][3];
			dh_theta[4] = iksol[i][4];
			tf::Transform T05 = T03 * fwd_trans(3,5, dh_alpha, dh_theta, dh_a, dh_d);
			tf::Transform T56 = T05.inverse() * xf;
			c6 = T56.getBasis()[0][0];
			s6 = T56.getBasis()[2][0];
		}
		iksol[i][5] = atan2(s6, c6);
	}

	vector<float> jp_dh(6); // current joint angles

	if(!joint_to_dhvalue(state.jp, jp_dh, arm))
	{
		ROS_ERROR("Something went wrong in joint to dhvalue conversion.");
		return success;
	}

	int sol_idx;
	float sol_err;

	if(find_best_solution(jp_dh, iksol, ikcheck, sol_idx, sol_err))
	{
		bool limited = dhvalue_to_joint(iksol[sol_idx], output_jp, input_gangle, arm);

		// adjust desired cartesian positions if necessary
		if(limited)
		{
		  fwd_kinematics(arm, output_jp, xf);
		  input_cp = xf;
		}

		success = true;
	}
	else
		ROS_ERROR("Raven IK calculation failed.");

	return success;
}



/**
 * @brief      Apply Raven joint limit constraints.
 *
 * @param      joint  The input joint value vector
 *
 * @return     success
 */
bool AMBFRavenPlanner::apply_joint_limits(vector<float>& joint, bool& changed)
{
	bool success = false;
	changed = false;

	if(joint.size() != AMBFDef::raven_joints)
	{
		ROS_ERROR("Wrong input vector length to apply joint limit check.");
		return success;
	}

	for(int i=0; i<AMBFDef::raven_joints; i++)
	{
		if(i != 2)
		{
			while(joint[i] >  M_PI)  joint[i] -= 2*M_PI;
			while(joint[i] < -M_PI)  joint[i] += 2*M_PI;
		}

		if(joint[i] < AMBFDef::raven_joint_limit[0][i])
		{
			joint[i] = AMBFDef::raven_joint_limit[0][i];
			changed = true;
		}
		else if(joint[i] > AMBFDef::raven_joint_limit[1][i])
		{
			joint[i] = AMBFDef::raven_joint_limit[1][i];
			changed = true;
		}
	}

	success = true;
	return success;
}





/**
 * @brief      Find the IK solution that matches the current pose.
 *
 * @param[in]  curr_jp  The curr jp
 * @param[in]  iksol    The iksol
 * @param[in]  ikcheck  The ikcheck
 * @param      sol_idx  The sol index
 * @param      sol_err  The sol error
 *
 * @return     success
 */
bool AMBFRavenPlanner::find_best_solution(vector<float> curr_jp, vector<vector<float>> iksol, vector<bool> ikcheck, int& sol_idx, float& sol_err)
{
	bool success = false;
	float best_err = 1e+10;
	float best_idx = -1;

	for(int i=0;i<AMBFDef::raven_iksols;i++)
	{
		float error = 0;
		if(ikcheck[i] == true)
		{
			for(int j=0;j<AMBFDef::raven_joints-1;j++)
			{
				if(j == 2)
					error += 100*pow(iksol[i][j] - curr_jp[j],2);
				else
				{
					float diff = iksol[i][j] - curr_jp[j];
					while(diff >  M_PI) diff -= 2*M_PI;
					while(diff < -M_PI) diff += 2*M_PI;

					error += pow(diff,2);
				}
			}
			if(error < best_err)
			{
				best_err = error;
				best_idx = i;
				success = true;
			}
		}
	}

	sol_err = best_err;
	sol_idx = best_idx;

	return success;
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




/**
 * @brief      Commands Raven to trace a random cube in Cartesian space)
 *
 * @param[in]  first_entry  The first entry
 * @param[in]  arm          The arm
 *
 * @return     success
 */
bool AMBFRavenPlanner::trace_cube(bool first_entry, int arm, bool debug_mode)
{
	int idx;
	bool success = false;
	int duration_count = 2 * AMBFDef::loop_rate;
	float motion_scale = 0.10;  // (m)

	static vector<int> last_idx(AMBFDef::raven_arms);
	static vector<int> count(AMBFDef::raven_arms);

	static vector<tf::Transform> start_pose(AMBFDef::raven_arms);
	static vector<tf::Vector3>   next_loc(AMBFDef::raven_arms);
	static vector<tf::Vector3>   prev_loc(AMBFDef::raven_arms);

	if(first_entry || !homed)
	{
		last_idx[arm] = -1;
		count[arm] = 0;
		next_loc[arm] = (arm == 0) ? tf::Vector3(1,1,0):tf::Vector3(1,0,0);
		start_pose[arm].setRotation(state.cp.getRotation());
		start_pose[arm].setOrigin(state.cp.getOrigin() - motion_scale*next_loc[arm]);

		go_home(first_entry,arm);
		kinematics_show(arm,debug_mode);

		success = true;
		return success;
	}

	if(count[arm] % duration_count == 0)
	{
		count[arm] = 0;

		do idx = rand() % 3; while(idx == last_idx[arm]);

		prev_loc[arm] = next_loc[arm];
		next_loc[arm][idx] = ((int)next_loc[arm][idx]+1) % 2;

		last_idx[arm] = idx;

		if(debug_mode)
		{
			ROS_INFO("Cube tracing update: arm%d start moving in %s%s direction. (x,y,z = %f,%f,%f)",
				arm, AMBFDef::sign_name[int(next_loc[arm][idx])].c_str(), AMBFDef::axes_name[idx].c_str(),
				state.cp.getOrigin().x(),state.cp.getOrigin().y(),state.cp.getOrigin().z());
		}
	}

	float ratio = 0.5-0.5*cos(M_PI*count[arm]/duration_count);

	command.cp = start_pose[arm];
	command.cp.setOrigin(command.cp.getOrigin() + motion_scale*(prev_loc[arm]*(1-ratio)+next_loc[arm]*ratio));

	inv_kinematics(arm, command.cp, 0, command.js);

	count[arm] ++;
	command.type 	= AMBFCmdType::_cp;
	command.updated = true;
	state.updated   = false;

	success = true;
	return success;
}




/**
 * @brief      A test cript for inverse kinematics
 *
 * @param[in]  arm   The arm
 *
 * @return     success
 */
bool AMBFRavenPlanner::kinematics_show(int arm, bool debug_mode)
{
	static vector<int> count = {0,0};
	bool success = false;

	tf::Transform 	cp_trn = state.cp;
	tf::Vector3 	cp_pos = cp_trn.getOrigin();
	tf::Quaternion 	cp_ori = cp_trn.getRotation();

	if(debug_mode && count[arm] % 1000 == 0)
	{
		ROS_INFO("arm%d:",arm);
		ROS_INFO("          jp = ( %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f)",
			state.jp[0],state.jp[1],state.jp[2],state.jp[3],state.jp[4],state.jp[5],state.jp[6]);

		ROS_INFO("after FK: cp = ( %f,\t%f,\t%f), ori = ( %f,\t%f,\t%f,\t%f)",
			cp_pos.x(),cp_pos.y(),cp_pos.z(),cp_ori.x(),cp_ori.y(),cp_ori.z(),cp_ori.w());



		vector<float> new_jp = AMBFDef::zero_joints;
		inv_kinematics(arm, cp_trn, state.jp[5]+state.jp[6], new_jp);
		cp_pos = cp_trn.getOrigin();
		cp_ori = cp_trn.getRotation();

		ROS_INFO("after IK: jp = ( %f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%f)",
			new_jp[0],new_jp[1],new_jp[2],new_jp[3],new_jp[4],new_jp[5],new_jp[6]);
/*		ROS_INFO("          cp = (\t%f,\t%f,\t%f), ori = (\t%f,\t%f,\t%f,\t%f)",
			cp_pos.x(),cp_pos.y(),cp_pos.z(),cp_ori.x(),cp_ori.y(),cp_ori.z(),cp_ori.w());*/

		fwd_kinematics(arm, new_jp, cp_trn);
		cp_pos = cp_trn.getOrigin();
		cp_ori = cp_trn.getRotation();

		ROS_INFO("again FK: cp = ( %f,\t%f,\t%f), ori = ( %f,\t%f,\t%f,\t%f)\n\n",
			cp_pos.x(),cp_pos.y(),cp_pos.z(),cp_ori.x(),cp_ori.y(),cp_ori.z(),cp_ori.w());

		count[arm] = 0;
	}

	count[arm] ++;

	success = true;
	return success;
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
	static int count = 0;
	static float duration = rand() % 6 + 5; // 5-10 seconds
	static vector<tf::Transform> start_cp(AMBFDef::camera_count);
	static vector<tf::Transform> goal_cp(AMBFDef::camera_count);

	if(first_entry)
	{
		start_cp[cam].setOrigin(state.cp.getOrigin());
		start_cp[cam].setRotation(state.cp.getRotation());

		tf::Vector3 move_vec = tf::Vector3(rand()%3-1,rand()%3-1,rand()%3-1);
		while(move_vec.length() == 0.0) move_vec = tf::Vector3(rand()%3-1,rand()%3-1,rand()%3-1);

		tf::Vector3 goal_pos = start_cp[cam].getOrigin()+move_vec;
		tf::Quaternion goal_oir = start_cp[cam].getRotation()*tf::Quaternion(move_vec,M_PI/6);
		goal_cp[cam].setOrigin(goal_pos);
		goal_cp[cam].setRotation(state.cp.getRotation());

		count = 0;
	}

	float iterations = duration * AMBFDef::camera_count * AMBFDef::loop_rate;
	float scale = 0.5*cos(1.0*count/iterations)-0.5;

	command.cp.setOrigin((1-scale)*start_cp[cam].getOrigin() + scale*goal_cp[cam].getOrigin());
	command.cp.setRotation(start_cp[cam].getRotation().slerp(goal_cp[cam].getRotation(),scale));


	command.type 	= AMBFCmdType::_cp;
	command.updated = true;
	state.updated   = false;
	count ++;

	return true;

}
