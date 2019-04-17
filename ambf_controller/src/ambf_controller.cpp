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

	if(!init_ros(argc,argv)) // initialize ros
	{	
		ROS_ERROR("Initialization Error. System Shutting Down.");
		exit(1);
	}
}



bool AMBFController::init_ros(int argc, char** argv)
{
	while(!ros::ok())
		ROS_ERROR("Something is wrong with ROS. Will keep trying...");

	// setup rostopic communication
	cmd_pub = nh_.advertise<std_msgs::String>("chatter", 1000);
	// sta_sub = nh_.subscribe;
	
	// ros variables
	lr_ = 500;

	return true;	
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
	static int count = 0;

	std::stringstream ss;
    ss << "Hello World!" << count;
    std_msgs::String msg;
    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());

    cmd_pub.publish(msg);
    count ++;

    return true;
}



AMBFController::~AMBFController()
{
	ROS_INFO("okay bye");
}