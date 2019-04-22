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

# include "ambf_controller.h"


/**
 * @brief      Constructs the AMBFController object.
 *
 * @param[in]  arguements passed from main function
 */
AMBFController::AMBFController(int argc, char** argv)
{
	ROS_INFO("AMBF Controller Initializing...");

	init_sys();

	if(!init_ros(argc,argv)) // initialize ros
	{	
		ROS_ERROR("Initialization Error. System Shutting Down.");
		exit(1);
	}
}


/**
 * @brief      initialize system parameters
 *
 * @return     success
 */
bool AMBFController::init_sys()
{
	// The loop rate of the AMBF simulator: 	2000 Hz
	// The loop rate of the AMBF python client: 1000 Hz
	// The loop rate of the Raven source code:  1000 Hz
	lr_ = 1000;  // the loop rate (Hz)
	n_joints = 7;
	n_arms = 2;

	// system variables
	raven_append = "/ambf/env/raven_2";
	sub_append = "/State";
	pub_append = "/Command";

	arm_append.push_back("/base_link_L");
	arm_append.push_back("/base_link_R");

	for(int i=0; i<n_joints; i++)
	{
		zero_joints.push_back(0);
		true_joints.push_back(1);
		false_joints.push_back(0);
	}
	zero_vec = tf::Vector3(0,0,0);

	// joint -: 0_link-base_link_L: 			fixed
	// joint 0: base_link_L-link1_L: 			revolute		(shoulder)				range: -pi~pi
	// joint 1: link1_L-link2_L: 				revolute		(elbow)					range: -pi~pi
	// joint 2: link2_L-link3_L: 				prismatic 		(tool plate up/down)	range: -0.17~0.1
	// joint 3: link3_L-instrument_shaft_L: 	continuous		(tool shaft roll)		range: no limit
	// joint 4: instrument_shaft_L-wrist_L: 	revolute		(tool wrist)			range: -2~2
	// joint 5: wrist_L-grasper1_L: 			revolute		(grasper left)			range: -2~2
	// joint 6: wrist_L-grasper2_L: 			revolute		(grasper right)			range: -2~2

	raven_state.resize(n_arms);
	raven_command.resize(n_arms);
	for(int i=0; i<n_arms; i++)
	{
		raven_state[i].updated = false;
		raven_command[i].updated = false;
		raven_command[i].type = _jp; 		// _null

		raven_state[i].jp.resize(n_joints);
		raven_command[i].js.resize(n_joints);
	}
	reset_command();
	return true;
}



/**
 * @brief      initialize ROS published and subscribed topics
 *
 * @param[in]  arguements passed from main function
 * @param      argv  The argv
 *
 * @return     success
 */
bool AMBFController::init_ros(int argc, char** argv)
{

	while(!ros::ok())
		ROS_ERROR("Something is wrong with ROS. Will keep trying...");

	
	// setup rostopic communication
	/*
	Publications: 
	 * /ambf/env/World/State [ambf_msgs/WorldState]
	 * /default_camera/State [ambf_msgs/ObjectState]
	 * /light_1/State [ambf_msgs/ObjectState]
	 * /light_2/State [ambf_msgs/ObjectState]


	Subscriptions: 
	 * /ambf/env/World/Command [unknown type]
	 * /default_camera/Command [unknown type]
	 * /light_1/Command [unknown type]
	 * /light_2/Command [unknown type]
	*/
	string topic;

	for(int i=0; i<n_arms; i++)
	{
		topic = raven_append + arm_append[i] + sub_append;

		raven_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic,1,
				               boost::bind(&AMBFController::raven_state_cb, this, _1, arm_append[i])));

		topic = raven_append + arm_append[i] + pub_append;
		raven_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic,1));
	}

	return true;	
}



/**
 * @brief      callback function for all the state messages
 *
 * @param[in]  event       The ROS message event
 * @param[in]  topic_name  The topic name
 */
void AMBFController::raven_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>& event,  const std::string& topic_name)
{
  lock_guard<mutex> _mutexlg(_mutex);

  static int count = 0;
  int i = -1;

  for(int i=0; i<n_arms; i++)
  {
  	if(topic_name == arm_append[i])
  	{
  		const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

	  	geometry_msgs::Pose cp = msg->pose;
	  	raven_state[i].cp.setOrigin(tf::Vector3(cp.position.x,cp.position.y,cp.position.z));
	   	raven_state[i].cp.setRotation(tf::Quaternion(cp.orientation.x,cp.orientation.y,cp.orientation.z,cp.orientation.w));

	   	geometry_msgs::Wrench wr = msg->wrench;
	   	raven_state[i].ct = tf::Vector3(wr.torque.x,wr.torque.y,wr.torque.z);
	  	raven_state[i].cf = tf::Vector3(wr.force.x,wr.force.y,wr.force.z);

	   	if(msg->joint_positions.size() == n_joints)
	   	{
		  	for(int j = 0; j< n_joints; j++)
		  		raven_state[i].jp[j] = msg->joint_positions[j]; 

	  		ROS_INFO("Received raven topic: %s count=%d",topic_name.c_str(),count);
		  	raven_state[i].updated = true;	
		  	count++;	
	   	}
	   	else
	   	{
	   		ROS_ERROR("Received a corrupted raven topic: %s.",topic_name.c_str());
	   	}
  	}
  }
} 


/**
 * @brief      AMBFController::Plans raven motion whenever a new raven state info is received
 *
 * @return     success
 */
bool AMBFController::raven_motion_planning()
{
	lock_guard<mutex> _mutexlg(_mutex);

	for(int i=0; i<n_arms; i++)
	{
		if(raven_state[i].updated && raven_command[i].type != _null)
		{
			if(raven_command[i].type == _jp || raven_command[i].type == _jw)
			{
				// TODO: update raven_command[i].js
				raven_command[i].updated = true;
				raven_state[i].updated   = false;
			}
			else if(raven_command[i].type == _cp)
			{
				// TODO: set desired cartesian position
				//		 do inverse kinematics
				//		 update raven_command[i].js

				raven_command[i].updated = true;
				raven_state[i].updated   = false;
			}
			else if(raven_command[i].type == _cw)
			{
				// TODO: update raven_command[i].cf
				//       update raven_command[i].ct
				raven_command[i].updated = true;
				raven_state[i].updated   = false;
			}
		}		
	}

	return true;
}


/**
 * @brief      run the system process
 *
 * @return     check if ros is running fine
 */
bool AMBFController::sys_run()
{
	bool check = ros::ok();
	
	ros::Rate loop_rate(lr_);
    while(check){

		raven_motion_planning();
        raven_command_pb();
        reset_command();

        ros::spinOnce();
        loop_rate.sleep();

        check = ros::ok();
    }

    return check;
}


/**
 * @brief      the publish function for all the command ROS topics
 *
 * @return     success
 */
bool AMBFController::raven_command_pb()
{ 
	lock_guard<mutex> _mutexlg(_mutex);
	
	/*
	This is the ObjectCmd content:

	Header header
	bool enable_position_controller  (default as false)
	geometry_msgs/Pose pose
	geometry_msgs/Wrench wrench
	float32[] joint_cmds
	bool[] position_controller_mask 
	*/
	
	static int count = 0;
	float sleep_time =  1/(float)lr_;

	for(int i=0; i<n_arms; i++)
	{
		ambf_msgs::ObjectCmd msg;

		if(raven_command[i].updated && raven_command[i].type != _null)
		{
			if(raven_command[i].type == _jp || raven_command[i].type == _cp)
			{
				msg.joint_cmds = raven_command[i].js;
				msg.enable_position_controller = true;

				for(int j=0; j<n_joints; j++)
				{
					msg.position_controller_mask = false_joints;
					msg.position_controller_mask[j] = true; // need to publish one joint at a time

					raven_pubs[i].publish(msg);
					ros::Duration(sleep_time).sleep();  // wait for the topic to be published
				}
			}
			else if(raven_command[i].type == _jw)
			{
				msg.joint_cmds = raven_command[i].js;
				msg.position_controller_mask = false_joints;

				raven_pubs[i].publish(msg);
			}
			else if (raven_command[i].type == _cw)
			{
				msg.wrench.force.x  = raven_command[i].cf.x();
				msg.wrench.force.y  = raven_command[i].cf.y();
				msg.wrench.force.z  = raven_command[i].cf.z();
				msg.wrench.torque.x = raven_command[i].ct.x();
				msg.wrench.torque.y = raven_command[i].ct.y();
				msg.wrench.torque.z = raven_command[i].ct.z();

				raven_pubs[i].publish(msg);
			}

			count ++;
			raven_command[i].updated = false;
	    	ROS_INFO("publish count: %s",to_string(count).c_str());
		}
	}
	
    return true;
}


/**
 * @brief      reset all the commands to zero
 *
 * @return     success
 */
bool AMBFController::reset_command()
{
    lock_guard<mutex> _mutexlg(_mutex);

    for(int i=0; i<n_arms; i++)
    {
	 	raven_command[i].js = zero_joints;   	// raven joint space command
	    raven_command[i].cf = zero_vec;        // raven cartesian force command      
	    raven_command[i].ct = zero_vec;        // raven cartesian torque command         	
    }

	return true;
}

/**
 * @brief      Destroys the AMBFContreller object.
 */
AMBFController::~AMBFController()
{
	ROS_INFO("okay bye");
}