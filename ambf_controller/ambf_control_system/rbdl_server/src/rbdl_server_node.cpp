#include <ros/ros.h>
#include "rbdl_server/RBDLServer.h"

RBDLServer::RBDLServer(ros::NodeHandle *nodehandle) {

}



int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_server");
    ros::NodeHandle the_potato;
    RBDLServer my_server(&the_potato);
    ros::Rate loop_rate(1000);
    while(ros::ok)
    {

        loop_rate.sleep();
        ros::spin();
        // ROS_INFO("starting");
        // ros::ServiceClient client_model = the_potato.serviceClient<rbdl_server::RBDLModel>("CreateModel");
        // ROS_INFO("In ");
        // rbdl_server::RBDLModel my_model;
        // my_model.request.model = "hello";
        // if (client_model.call(my_model))
        // {
        //     ROS_INFO("Did the things");
        // }
        // else
        // {
        //     ROS_ERROR("Failed to call service add_two_ints");
        //     return 1;
        // }

    }
    

}
