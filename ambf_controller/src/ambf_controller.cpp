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

	// joint -: 0_link-base_link_L: 			fixed
	// joint 0: base_link_L-link1_L: 			revolute		(shoulder)				range: -pi~pi
	// joint 1: link1_L-link2_L: 				revolute		(elbow)					range: -pi~pi
	// joint 2: link2_L-link3_L: 				prismatic 		(tool plate up/down)	range: -0.17~0.1
	// joint 3: link3_L-instrument_shaft_L: 	continuous		(tool shaft roll)		range: no limit
	// joint 4: instrument_shaft_L-wrist_L: 	revolute		(tool wrist)			range: -2~2
	// joint 5: wrist_L-grasper1_L: 			revolute		(grasper left)			range: -2~2
	// joint 6: wrist_L-grasper2_L: 			revolute		(grasper right)			range: -2~2

	raven_planner.resize(AMBFDef::raven_arms);
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

	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		topic = AMBFDef::raven_append + AMBFDef::arm_append[i] + AMBFDef::sub_append;

		raven_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic,1,
				               boost::bind(&AMBFController::raven_state_cb, this, _1, AMBFDef::arm_append[i])));

		topic = AMBFDef::raven_append + AMBFDef::arm_append[i] + AMBFDef::pub_append;

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

  for(int i=0; i<AMBFDef::raven_arms; i++)
  {
  	if(topic_name == AMBFDef::arm_append[i])
  	{
  		const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

	  	geometry_msgs::Pose cp = msg->pose;
	  	raven_planner[i].state.cp.setOrigin(tf::Vector3(cp.position.x,cp.position.y,cp.position.z));
	   	raven_planner[i].state.cp.setRotation(tf::Quaternion(cp.orientation.x,cp.orientation.y,cp.orientation.z,cp.orientation.w));

	   	geometry_msgs::Wrench wr = msg->wrench;
	   	raven_planner[i].state.ct = tf::Vector3(wr.torque.x,wr.torque.y,wr.torque.z);
	  	raven_planner[i].state.cf = tf::Vector3(wr.force.x,wr.force.y,wr.force.z);

		if(msg->joint_positions.size() == AMBFDef::raven_joints)
	   	{
		  	for(int j = 0; j< AMBFDef::raven_joints; j++)
		  		raven_planner[i].state.jp[j] = msg->joint_positions[j]; 

	  		ROS_INFO("Received raven topic: %s count=%d",topic_name.c_str(),count);
		  	raven_planner[i].state.updated = true;	
		  	count++;	
	   	}
	   	else if(msg->joint_positions.size() == 0)
	   	{
	   		raven_first_pb(); // first time
	   	}
	   	else
	   	{
	   		ROS_ERROR("Received a corrupted raven topic: %s. ",topic_name.c_str());
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

	static int count = 0;

	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		if(raven_planner[i].state.updated && raven_planner[i].command.type != _null)
		{
			if(raven_planner[i].command.type == _jp || raven_planner[i].command.type == _jw)
			{
				// TODO: update raven_planner[i].command.js
				raven_planner[i].command.js[0] = 0.3*sin(count*0.01/M_PI);
				raven_planner[i].command.js[1] = 0.3*sin(count*0.01/M_PI+i*M_PI/2);

				raven_planner[i].command.updated = true;
				raven_planner[i].state.updated   = false;

				count ++;
			}
			else if(raven_planner[i].command.type == _cp)
			{
				// TODO: set desired cartesian position
				//		 do inverse kinematics
				//		 update raven_planner[i].command.js

				raven_planner[i].command.updated = true;
				raven_planner[i].state.updated   = false;
			}
			else if(raven_planner[i].command.type == _cw)
			{
				// TODO: update raven_planner[i].command.cf
				//       update raven_planner[i].command.ct
				raven_planner[i].command.updated = true;
				raven_planner[i].state.updated   = false;
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
	
	ros::Rate loop_rate(AMBFDef::loop_rate);
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
 * @brief      Publish the first command to set state message preferences
 *
 * @return     success
 */
bool AMBFController::raven_first_pb()
{
	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		ambf_msgs::ObjectCmd msg;
		msg.header.stamp = ros::Time::now();
		msg.publish_children_names  = raven_planner[i].command.cn_flag;
		msg.publish_joint_names	    = raven_planner[i].command.jn_flag;
	    msg.publish_joint_positions = raven_planner[i].command.jp_flag;
		raven_pubs[i].publish(msg);	
	}
	return true;
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
	float sleep_time =  1.0/AMBFDef::loop_rate;

	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		ambf_msgs::ObjectCmd msg;
		msg.header.stamp = ros::Time::now();
		msg.publish_children_names  = raven_planner[i].command.cn_flag;
		msg.publish_joint_names	    = raven_planner[i].command.jn_flag;
	    msg.publish_joint_positions = raven_planner[i].command.jp_flag;

		if(raven_planner[i].command.updated && raven_planner[i].command.type != _null)
		{
			if(raven_planner[i].command.type == _jp || raven_planner[i].command.type == _cp)
			{
				msg.joint_cmds = raven_planner[i].command.js;
				msg.position_controller_mask = AMBFDef::true_joints;

				raven_pubs[i].publish(msg);
			}
			else if(raven_planner[i].command.type == _jw)
			{
				msg.joint_cmds = raven_planner[i].command.js;
				msg.position_controller_mask = AMBFDef::false_joints;

				raven_pubs[i].publish(msg);
			}
			else if (raven_planner[i].command.type == _cw)
			{
				msg.wrench.force.x  = raven_planner[i].command.cf.x();
				msg.wrench.force.y  = raven_planner[i].command.cf.y();
				msg.wrench.force.z  = raven_planner[i].command.cf.z();
				msg.wrench.torque.x = raven_planner[i].command.ct.x();
				msg.wrench.torque.y = raven_planner[i].command.ct.y();
				msg.wrench.torque.z = raven_planner[i].command.ct.z();

				raven_pubs[i].publish(msg);
			}

			count ++;
			raven_planner[i].command.updated = false;
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

    for(int i=0; i<AMBFDef::raven_arms; i++)
    {
	 	raven_planner[i].command.js = AMBFDef::zero_joints;   	// raven joint space command
	    raven_planner[i].command.cf = AMBFDef::zero_vec;         // raven cartesian force command      
	    raven_planner[i].command.ct = AMBFDef::zero_vec;         // raven cartesian torque command   

	    raven_planner[i].command.cn_flag = false;		// raven state: children name flag
	    raven_planner[i].command.jn_flag = false;		// raven state: joint name flag
	    raven_planner[i].command.jp_flag = true;		// raven state: joint position flag

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