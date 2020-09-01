#include <ros/ros.h>
#include "rbdl_server/RBDLServer.h"

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_server");
    ros::NodeHandle the_potato;
    RBDLServer my_server(&the_potato);
    

}
