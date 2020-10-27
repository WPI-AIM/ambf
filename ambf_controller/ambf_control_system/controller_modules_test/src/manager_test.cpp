#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "Eigen/Core"
#include "boost/shared_ptr.hpp"
#include "controller_modules/JointControl.h"

int main(int argc, char* argv[])
{
    
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<controller_modules::JointControl>("CalcTau");
   controller_modules::JointControl msg;

   msg.request.desired.positions = std::vector<double>{5.0,5.0};
   msg.request.actual.positions = std::vector<double>{0.0,0.0};

   msg.request.desired.velocities = std::vector<double>{5.0,5.0};
   msg.request.actual.velocities = std::vector<double>{0.0,0.0};
   msg.request.controller_name = "PD";

   if (client.call(msg))
   {
        ROS_INFO("it worked");
        
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    } 

   ros::spin();
   return 0;
}
