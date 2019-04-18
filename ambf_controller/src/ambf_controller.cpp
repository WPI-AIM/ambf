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


bool AMBFController::init_sys()
{
	// system variables
	raven_append = "/ambf/env/raven_2";
	sub_append = "/State";
	pub_append = "/Command";

	raven_parts.push_back("/0_link");					// 0

	raven_parts.push_back("/base_link_L");				// 1
	raven_parts.push_back("/link1_L");					// 2
	raven_parts.push_back("/link2_L");					// 3
	raven_parts.push_back("/link3_L");					// 4
	raven_parts.push_back("/instrument_shaft_L");		// 5
	raven_parts.push_back("/wrist_L");					// 6
	raven_parts.push_back("/grasper1_L");				// 7
	raven_parts.push_back("/grasper2_L");				// 8

	raven_parts.push_back("/base_link_R");				// 9
	raven_parts.push_back("/link1_R");					// 10
	raven_parts.push_back("/link2_R");					// 11
	raven_parts.push_back("/link3_R");					// 12
	raven_parts.push_back("/instrument_shaft_R");		// 13
	raven_parts.push_back("/wrist_R");					// 14
	raven_parts.push_back("/grasper1_R");				// 15
	raven_parts.push_back("/grasper2_R");				// 16

	lr_ = 1000;  // TODO: figure out appropreate value
	n_parts = raven_parts.size();

	for(int i=0; i<n_parts; i++)
	{
		raven_parts_map.insert(pair<string,int>(raven_parts[i],i));
	}
	
	return true;
}



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

	for(int i=0; i<n_parts; i++)
	{
		topic = raven_append + raven_parts[i] + sub_append;

		state_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic,1,
			               boost::bind(&AMBFController::state_callback, this, _1, raven_parts[i])));

		topic = raven_append + raven_parts[i] + pub_append;
		command_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic,1));
	}
	raven_pose.reserve(n_parts);
	


	return true;	
}



void AMBFController::state_callback(const ros::MessageEvent<ambf_msgs::ObjectState const>& event,  const std::string& topic_name)
{
  int i = raven_parts_map.find(topic_name)->second;

  const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

  raven_pose[i] = msg->pose;

  ROS_INFO("received topic: %s, x=%f",  topic_name.c_str(), raven_pose[i].position.x);
} 


bool AMBFController::sys_run()
{
	bool check = ros::ok();
	
	ros::Rate loop_rate(lr_);
    while(check){

        publish_command();

        ros::spinOnce();
        loop_rate.sleep();

        check = ros::ok();
    }

    return check;
}


bool AMBFController::publish_command()
{
	/*
	This is the ObjectCmd content:

	Header header
	bool enable_position_controller
	geometry_msgs/Pose pose
	geometry_msgs/Wrench wrench
	float32[] joint_cmds
	bool[] position_controller_mask
	*/

	static int count = 0;

    for(int i=0; i<n_parts; i++)   // TODO: try to make robot hold still
    {
    	ambf_msgs::ObjectCmd msg;

    	msg.pose.position.x=raven_pose[i].position.x;
    	msg.pose.position.y=raven_pose[i].position.y;
    	msg.pose.position.z=raven_pose[i].position.z;

    	msg.pose.orientation.x=raven_pose[i].orientation.x;
    	msg.pose.orientation.y=raven_pose[i].orientation.y;
    	msg.pose.orientation.z=raven_pose[i].orientation.z;
    	msg.pose.orientation.w=raven_pose[i].orientation.w;

    	command_pubs[i].publish(msg);
    }
    
    count ++;
    ROS_INFO("publish count: %s",to_string(count).c_str());

    return true;
}



AMBFController::~AMBFController()
{
	ROS_INFO("okay bye");
}