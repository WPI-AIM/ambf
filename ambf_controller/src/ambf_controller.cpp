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
	// system variables
	raven_append = "/ambf/env/raven_2";
	sub_append = "/State";
	pub_append = "/Command";

	L_append = "/base_link_L";
	R_append = "/base_link_R";

	// The loop rate of the AMBF simulator: 	2000 Hz
	// The loop rate of the AMBF python client: 1000 Hz
	// The loop rate of the Raven source code:  1000 Hz
	lr_ = 1000;  // the loop rate (Hz)
	n_joints = 7;
	zero_vec = tf::Vector3(0,0,0);

	for(int i=0; i<n_joints; i++)
	{
		zero_joints.push_back(0);
		true_joints.push_back(1);
		false_joints.push_back(0);
	}

	command_type = _jp;

	// joint -: 0_link-base_link_L: 			fixed
	// joint 0: base_link_L-link1_L: 			revolute		(shoulder)				range: -pi~pi
	// joint 1: link1_L-link2_L: 				revolute		(elbow)					range: -pi~pi
	// joint 2: link2_L-link3_L: 				prismatic 		(tool plate up/down)	range: -0.17~0.1
	// joint 3: link3_L-instrument_shaft_L: 	continuous		(tool shaft roll)		range: no limit
	// joint 4: instrument_shaft_L-wrist_L: 	revolute		(tool wrist)			range: -2~2
	// joint 5: wrist_L-grasper1_L: 			revolute		(grasper left)			range: -2~2
	// joint 6: wrist_L-grasper2_L: 			revolute		(grasper right)			range: -2~2

	js_command_L.reserve(n_joints);
	js_command_R.reserve(n_joints);
	jp_state_L.reserve(n_joints);
	jp_state_R.reserve(n_joints);

	reset_commands();

	state_L_updated = false;
	state_R_updated = false;
	command_L_updated = false;
	command_R_updated = false;

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

	string topic_L, topic_R;

	topic_L = raven_append + L_append + sub_append;
	topic_R = raven_append + R_append + sub_append;

	raven_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic_L,1,
			               boost::bind(&AMBFController::raven_state_cb, this, _1, L_append)));
	raven_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic_R,1,
			               boost::bind(&AMBFController::raven_state_cb, this, _1, R_append)));

	topic_L = raven_append + L_append + pub_append;
	topic_R = raven_append + R_append + pub_append;

	raven_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic_L,1));
	raven_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic_R,1));

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

  static int count_L = 0;
  static int count_R = 0;
  const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

  geometry_msgs::Pose cp = msg->pose;

  if(topic_name == L_append)
  {
  	cp_state_L.setOrigin(tf::Vector3(cp.position.x,cp.position.y,cp.position.z));
   	cp_state_L.setRotation(tf::Quaternion(cp.orientation.x,cp.orientation.y,cp.orientation.z,cp.orientation.w));

   	if(msg->joint_positions.size() == n_joints)
   	{
	  	for(int i = 0; i< n_joints; i++)
	  		jp_state_L[i] = msg->joint_positions[i]; 

  		ROS_INFO("Received raven topic: %s count=%d",topic_name.c_str(),count_L);
	  	state_L_updated = true;	
	  	count_L++;	
   	}
   	else
   	{
   		ROS_ERROR("Received a corrupted raven topic: %s.",topic_name.c_str());
   		state_L_updated = false;
   	}

  	

  }
  else if(topic_name == R_append)
  {
  	cp_state_R.setOrigin(tf::Vector3(cp.position.x,cp.position.y,cp.position.z));
  	cp_state_R.setRotation(tf::Quaternion(cp.orientation.x,cp.orientation.y,cp.orientation.z,cp.orientation.w));
   	
   	if(msg->joint_positions.size() == n_joints)
   	{
	  	for(int i = 0; i< n_joints; i++)
	  		jp_state_L[i] = msg->joint_positions[i];


  		ROS_INFO("Received raven topic: %s count=%d",topic_name.c_str(),count_R);
	  	state_R_updated = true;
	  	count_R ++;
	}
   	else
   	{
   		ROS_ERROR("Received a corrupted raven topic: %s.",topic_name.c_str());
   		state_R_updated = false;
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
	bool check = true;

	if(command_type == _jp)
	{
		if(state_L_updated)
		{
			// TODO: update js_command_L
			command_L_updated = true;
		}
		if(state_R_updated)
		{
			// TODO: update js_command_R
			command_R_updated = true;
		}
	}
	else if(command_type == _jw)
	{
		if(state_L_updated)
		{
			// TODO: update js_command_L

			command_L_updated = true;
		}
		if(state_R_updated)
		{
			// TODO: update js_command_R

			command_R_updated = true;
		}	
	}
	else if(command_type == _cp)
	{
		if(state_L_updated)
		{
			// TODO: set desired cartesian position
			//		 do inverse kinematics
			//		 update js_command_L

			command_L_updated = true;
		}
		if(state_R_updated)
		{
			// TODO: set desired cartesian position
			//		 do inverse kinematics
			//		 update js_command_R
			
			command_R_updated = true;
		}		
	}
	else if(command_type == _cw)
	{
		if(state_L_updated)
		{
			// TODO: update cf_command_L
			//       update ct_command_L
			command_L_updated = true;
		}
		if(state_R_updated)
		{
			// TODO: update cf_command_R
			//       update ct_command_R
			command_R_updated = true;
		}		
	}
	else
		check = false;

	return check;

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
        reset_commands();

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

	ambf_msgs::ObjectCmd msg_L, msg_R;
	static int count_L = 0;
	static int count_R = 0;

	if(command_type == _jp || command_type == _cp || command_type == _jw)
	{
		if(command_L_updated)
		{
			msg_L.joint_cmds = js_command_L;
			msg_L.position_controller_mask = false_joints;
		}
		if(command_R_updated)
		{
			msg_R.joint_cmds = js_command_R;
			msg_R.position_controller_mask = false_joints;
		}
	}
	else if (command_type == _cw)
	{
		if(command_L_updated)
		{
			msg_L.wrench.force.x = cf_command_L.x();
			msg_L.wrench.force.y = cf_command_L.y();
			msg_L.wrench.force.z = cf_command_L.z();
			msg_L.wrench.torque.x = ct_command_L.x();
			msg_L.wrench.torque.y = ct_command_L.y();
			msg_L.wrench.torque.z = ct_command_L.z();
		}
		if(command_R_updated)
		{
			msg_R.wrench.force.x = cf_command_R.x();
			msg_R.wrench.force.y = cf_command_R.y();
			msg_R.wrench.force.z = cf_command_R.z();
			msg_R.wrench.torque.x = ct_command_R.x();
			msg_R.wrench.torque.y = ct_command_R.y();
			msg_R.wrench.torque.z = ct_command_R.z();
		}		
	}

	if(command_L_updated)
	{
		if(command_type == _jp || command_type == _cp)
		{
			for(int i=0; i<n_joints; i++)
			{
				msg_L.position_controller_mask = false_joints;
				msg_L.position_controller_mask[i] = true; // need to publish one joint at a time
				msg_L.enable_position_controller = true;

				raven_pubs[0].publish(msg_L);
				ros::Duration(0.001).sleep();
			}
		}
		else
		{
			raven_pubs[0].publish(msg_L);
		}
		command_L_updated = false;

		count_L ++;
    	ROS_INFO("publish count: %s",to_string(count_L).c_str());
	}
	if(command_R_updated)
	{
		if(command_type == _jp || command_type == _cp)
		{
			for(int i=0; i<n_joints; i++)
			{
				msg_R.position_controller_mask = false_joints;
				msg_R.position_controller_mask[i] = true;
				msg_R.enable_position_controller = true;

				raven_pubs[1].publish(msg_R); // need to publish one joint at a time

				ros::Duration(0.001).sleep();
			}
		}
		else
		{
			raven_pubs[0].publish(msg_R);
		}
		command_R_updated = false;

		count_R ++;
    	ROS_INFO("publish count: %s",to_string(count_R).c_str());
	} 

    return true;
}


/**
 * @brief      reset all the commands to zero
 *
 * @return     success
 */
bool AMBFController::reset_commands()
{
    lock_guard<mutex> _mutexlg(_mutex);

	js_command_L = zero_joints;   	// raven joint space command
	js_command_R = zero_joints;

    cf_command_L = zero_vec;        // raven cartesian force command      
    cf_command_R = zero_vec;
    ct_command_L = zero_vec;        // raven cartesian torque command      
    ct_command_R = zero_vec;

	return true;
}

/**
 * @brief      Destroys the AMBFContreller object.
 */
AMBFController::~AMBFController()
{
	ROS_INFO("okay bye");
}