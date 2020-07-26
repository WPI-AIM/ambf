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
	raven_planner.resize(AMBFDef::raven_arms);
	camera_planner.resize(AMBFDef::camera_count);

	reset_command();
	print_menu = true;
	debug_mode = false;
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

	string topic;

	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		topic = AMBFDef::env_append + AMBFDef::raven_append + AMBFDef::arm_append[i] + AMBFDef::sub_append;
		raven_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic,1,
				               boost::bind(&AMBFController::raven_state_cb, this, _1, AMBFDef::arm_append[i])));

		topic = AMBFDef::env_append + AMBFDef::raven_append + AMBFDef::arm_append[i] + AMBFDef::pub_append;
		raven_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic,1));
	}

	for(int i=0; i<AMBFDef::camera_count; i++)
	{
		topic = AMBFDef::env_append + AMBFDef::cam_append[i] + AMBFDef::sub_append;
		camera_subs.push_back(nh_.subscribe<ambf_msgs::ObjectState>(topic,1,
				               boost::bind(&AMBFController::camera_state_cb, this, _1, AMBFDef::cam_append[i])));

		topic = AMBFDef::env_append + AMBFDef::cam_append[i] + AMBFDef::pub_append;
		camera_pubs.push_back(nh_.advertise<ambf_msgs::ObjectCmd>(topic,1));
	}

	return true;
}


/**
 * @brief      callback function for all the raven state messages
 *
 * @param[in]  event       The ROS message event
 * @param[in]  topic_name  The topic name
 */
void AMBFController::raven_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>& event,  const std::string& topic_name)
{
  lock_guard<mutex> _mutexlg(_mutex);

  for(int i=0; i<AMBFDef::raven_arms; i++)
  {
  	if(topic_name == AMBFDef::arm_append[i])
  	{
  		const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

	   	geometry_msgs::Wrench wr = msg->wrench;
	   	raven_planner[i].state.ct = tf::Vector3(wr.torque.x,wr.torque.y,wr.torque.z);
	  	raven_planner[i].state.cf = tf::Vector3(wr.force.x,wr.force.y,wr.force.z);

		if(msg->joint_positions.size() == AMBFDef::raven_joints)
	   	{
		  	for(int j = 0; j< AMBFDef::raven_joints; j++)
		  		raven_planner[i].state.jp[j] = msg->joint_positions[j];

		  	// set the carteisian end effector pose
		  	raven_planner[i].fwd_kinematics(i, raven_planner[i].state.jp, raven_planner[i].state.cp);
		  	raven_planner[i].state.updated = true;
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
 * @brief      callback function for all the camera state messages
 *
 * @param[in]  event       The ROS message event
 * @param[in]  topic_name  The topic name
 */
void AMBFController::camera_state_cb(const ros::MessageEvent<ambf_msgs::ObjectState const>& event,  const std::string& topic_name)
{
  static vector<bool> found_home = {false,false,false};

  lock_guard<mutex> _mutexlg(_mutex);

  for(int i=0; i<AMBFDef::camera_count; i++)
  {
  	if(topic_name == AMBFDef::cam_append[i])
  	{
  		const ambf_msgs::ObjectStateConstPtr msg = event.getConstMessage();

	  	geometry_msgs::Pose cp = msg->pose;

	  	if(!(cp.orientation.x == 0.0 && cp.orientation.y == 0.0 && cp.orientation.z == 0.0))
	  	{
	  		camera_planner[i].state.cp.setOrigin(tf::Vector3(cp.position.x,cp.position.y,cp.position.z));
		   	camera_planner[i].state.cp.setRotation(tf::Quaternion(cp.orientation.x,cp.orientation.y,cp.orientation.z,cp.orientation.w));

		   	if(!found_home[i])
		   		found_home[i] = camera_planner[i].set_home();

		   	camera_planner[i].state.updated = true;
	  	}
  	}
  }
}


/**
 * @brief      Plans raven and camera motion whenever a new state info is received
 *
 * @return     success
 */
bool AMBFController::motion_planning()
{
	lock_guard<mutex> _mutexlg(_mutex);

	// raven motion planning
	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		if(raven_planner[i].state.updated)
		{
			switch(raven_planner[i].mode)
			{
				case AMBFCmdMode::freefall:	// do nothing
					raven_planner[i].kinematics_show(i,debug_mode);
					break;

				case AMBFCmdMode::homing:
					raven_planner[i].go_home(false,i);
					raven_planner[i].kinematics_show(i,debug_mode);
					break;

				case AMBFCmdMode::dancing:
					raven_planner[i].sine_dance(false,i);
					raven_planner[i].kinematics_show(i,debug_mode);
					break;

				case AMBFCmdMode::cube_tracing:
					raven_planner[i].trace_cube(false,i,debug_mode);
					break;
			}
		}
	}

	// camera motion planning
	for(int i=0; i<AMBFDef::camera_count; i++)
	{
		if(camera_planner[i].state.updated && camera_planner[i].command.type != _null)
		{
			switch(camera_planner[i].mode)
			{
				case AMBFCmdMode::freefall:	// do nothing
					break;

				case AMBFCmdMode::homing:
					camera_planner[i].go_home(false,i);
					break;

				case AMBFCmdMode::dancing:
					camera_planner[i].wander_dance(false,i);
					break;
			}
		}
	}

	return true;
}


/**
 * @brief      run the system process (for system thread)
 *
 */
void AMBFController::sys_run()
{
	bool check = ros::ok();

	ros::Rate loop_rate(AMBFDef::loop_rate);
    while(check){

		motion_planning();

        raven_command_pb();
        camera_command_pb();

        reset_command();

        ros::spinOnce();
        loop_rate.sleep();

        check = ros::ok();
    }
    ROS_ERROR("System thread is shutting down.");
}



/**
 * @brief      run the console process (for console thread)
 *
 */
void AMBFController::csl_run()
{

	int key;
	bool check = ros::ok();

	ros::Rate loop_rate(AMBFDef::loop_rate);
    while(check){

    	show_menu();
		key = get_key();

		for(int i=0;i<AMBFDef::raven_arms; i++)
		{
			switch(key)
			{
				case '0':
					if(i == 0) ROS_INFO("0: Entered Raven freefall mode. No commands sent to either arm.");
					raven_planner[i].mode = AMBFCmdMode::freefall;
					raven_planner[i].command.type = _null;
					print_menu = true;
					break;

				case '1':
					if(i == 0) ROS_INFO("1: Entered Raven homing mode. Both arms moving to home.");
					raven_planner[i].mode = AMBFCmdMode::homing;
					raven_planner[i].command.type = _jp;
					raven_planner[i].go_home(true,i);
					print_menu = true;
					break;

				case '2':
					if(i == 0) ROS_INFO("2: Entered Raven dancing mode. Enjoy a little dance!");
					raven_planner[i].mode = AMBFCmdMode::dancing;
					raven_planner[i].command.type = _jp;
					raven_planner[i].sine_dance(true,i);
					print_menu = true;
					break;

				case '3':
					if(i == 0) ROS_INFO("3: Entered Raven cube_tracing mode. Enjoy a little dance!");
					raven_planner[i].mode = AMBFCmdMode::cube_tracing;
					raven_planner[i].command.type = _cp;
					raven_planner[i].trace_cube(true,i,debug_mode);
					print_menu = true;
					break;
			}
		}

		for(int i=0;i<AMBFDef::camera_count; i++)
		{
			switch(key)
			{
				case 'a':
				case 'A':
					if(i == 0) ROS_INFO("a: Entered Camera static mode. All cameras stay fixed.");
					camera_planner[i].mode = AMBFCmdMode::freefall;
					camera_planner[i].command.type = _null;
					print_menu = true;
					break;

				case 'b':
				case 'B':
					if(i == 0) ROS_INFO("b: Entered Camera wandering mode. All cameras start a random walk!");
					camera_planner[i].mode = AMBFCmdMode::dancing;
					camera_planner[i].command.type = _cp;
					camera_planner[i].wander_dance(true,i);
					print_menu = true;
					break;

				case 'c':
				case 'C':
					if(i == 0) ROS_INFO("c: Entered Camera homing mode. All cameras move back to home pose.");
					camera_planner[i].mode = AMBFCmdMode::homing;
					camera_planner[i].command.type = _cp;
					camera_planner[i].go_home(true,i);
					print_menu = true;
					break;
			}
		}

		if(key == 'x' || key == 'X')
		{
			debug_mode = !debug_mode;
			ROS_INFO("x: Entered Toggle console debug mode. Setting changed.");
			print_menu = true;
		}

        ros::spinOnce();
        loop_rate.sleep();

        check = ros::ok();
    }
    ROS_ERROR("Console thread is shutting down.");
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

	float sleep_time =  1.0/AMBFDef::loop_rate;

	for(int i=0; i<AMBFDef::raven_arms; i++)
	{
		ambf_msgs::ObjectCmd msg;
		msg.header.stamp = ros::Time::now();
		msg.publish_children_names  = raven_planner[i].command.cn_flag;
		msg.publish_joint_names	    = raven_planner[i].command.jn_flag;
	    msg.publish_joint_positions = raven_planner[i].command.jp_flag;

		if(raven_planner[i].command.updated && raven_planner[i].command.type != AMBFCmdType::_null)
		{
			switch(raven_planner[i].command.type)
			{
				case AMBFCmdType::_jp:
				case AMBFCmdType::_cp:
					raven_planner[i].check_incr_safety(raven_planner[i].state.jp, raven_planner[i].command.js, AMBFDef::raven_joints);
					msg.joint_cmds = raven_planner[i].command.js;
					msg.position_controller_mask = AMBFDef::true_joints;

					break;

				case AMBFCmdType::_jw:
					msg.joint_cmds = raven_planner[i].command.js;
					msg.position_controller_mask = AMBFDef::false_joints;
					break;

				case AMBFCmdType::_cw:
					msg.wrench.force.x  = raven_planner[i].command.cf.x();
					msg.wrench.force.y  = raven_planner[i].command.cf.y();
					msg.wrench.force.z  = raven_planner[i].command.cf.z();
					msg.wrench.torque.x = raven_planner[i].command.ct.x();
					msg.wrench.torque.y = raven_planner[i].command.ct.y();
					msg.wrench.torque.z = raven_planner[i].command.ct.z();
					break;
			}

			raven_pubs[i].publish(msg);
			raven_planner[i].command.updated = false;
		}
	}

    return true;
}



/**
 * @brief      the publish function for all the command ROS topics
 *
 * @return     success
 */
bool AMBFController::camera_command_pb()
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

	float sleep_time =  1.0/AMBFDef::loop_rate;

	for(int i=0; i<AMBFDef::camera_count; i++)
	{
		ambf_msgs::ObjectCmd msg;
		msg.header.stamp = ros::Time::now();

		if(camera_planner[i].command.updated && camera_planner[i].command.type == AMBFCmdType::_cp)
		{
			msg.pose.position.x = camera_planner[i].command.cp.getOrigin().x();
			msg.pose.position.y = camera_planner[i].command.cp.getOrigin().y();
			msg.pose.position.z = camera_planner[i].command.cp.getOrigin().z();

			msg.pose.orientation.x = camera_planner[i].command.cp.getRotation().x();
			msg.pose.orientation.y = camera_planner[i].command.cp.getRotation().y();
			msg.pose.orientation.z = camera_planner[i].command.cp.getRotation().z();
			msg.pose.orientation.w = camera_planner[i].command.cp.getRotation().w();

			msg.enable_position_controller = true;

			camera_pubs[i].publish(msg);
			camera_planner[i].command.updated = false;
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

    for(int i=0; i<AMBFDef::raven_arms; i++)
    {
    	camera_planner[i].command.cp = tf::Transform();
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



/**
 * @brief      Gets the keyboard input.
 *
 * @return     The key.
 */
int AMBFController::get_key()
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking
    	//   returns EOF (-1) if no character is available
    	character = fgetc(stdin);

   	// restore the original terminal attributes
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}


/**
 * @brief      Shows the menu.
 *
 * @return     success
 */
bool AMBFController::show_menu()
{
	string s;
	string s_false = "    ";
	string s_true  = " (v)";

	// detect if camera mode has changed.
	static AMBFCmdMode last_cam_mode = AMBFCmdMode::freefall;

	AMBFCmdMode cam_mode = AMBFCmdMode::freefall;

	for(int i=0; i<AMBFDef::camera_count; i++)
	{
		if(camera_planner[i].mode == AMBFCmdMode::homing)
		{
			cam_mode = AMBFCmdMode::homing;
			break;
		}
		else if(camera_planner[i].mode == AMBFCmdMode::dancing)
		{
			cam_mode = AMBFCmdMode::dancing;
			break;
		}
	}
	if(last_cam_mode != cam_mode)
		print_menu = true;

	last_cam_mode = cam_mode;

	// print the user menu
	if(print_menu)
	{
		ROS_INFO("\n\nWelcome to the AMBF Raven Simulator");
		ROS_INFO("-----------------------------------------------------");
		ROS_INFO("Please choose a mode:");

		// for raven
		if(raven_planner[0].mode == AMBFCmdMode::freefall) 		s = s_true;
		else											   		s = s_false;
		ROS_INFO("%s0: Raven freefall mode",s.c_str());

		if(raven_planner[0].mode == AMBFCmdMode::homing) 		s = s_true;
		else											 		s = s_false;
		ROS_INFO("%s1: Raven homing mode",s.c_str());

		if(raven_planner[0].mode == AMBFCmdMode::dancing) 		s = s_true;
		else											  		s = s_false;
		ROS_INFO("%s2: Raven dancing mode",s.c_str());

		if(raven_planner[0].mode == AMBFCmdMode::cube_tracing) 	s = s_true;
		else											       	s = s_false;
		ROS_INFO("%s3: Raven cube_tracing mode",s.c_str());

		// for camera
		if(cam_mode == AMBFCmdMode::freefall) 	s = s_true;
		else									s = s_false;
		ROS_INFO("%sa: Camera static mode",s.c_str());

		if(cam_mode == AMBFCmdMode::dancing) 	s = s_true;
		else									s = s_false;
		ROS_INFO("%sb: Camera wandering mode",s.c_str());

		if(cam_mode == AMBFCmdMode::homing) 	s = s_true;
		else									s = s_false;
		ROS_INFO("%sc: Camera homing mode",s.c_str());

		if(debug_mode)							s = s_true;
		else									s = s_false;
		ROS_INFO("%sx: Toggle console debug mode",s.c_str());

		ROS_INFO("-----------------------------------------------------\n\n");

		print_menu = false;
	}

	return true;
}
