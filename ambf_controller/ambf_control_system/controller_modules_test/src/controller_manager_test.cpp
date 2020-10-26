#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "Eigen/Core"
#include "boost/shared_ptr.hpp"

int main(int argc, char* argv[])
{
    
    ros::init(argc, argv, "talker");
    Eigen::MatrixXd Kp(2,2);
    Kp(0,0) = 5;
    Kp(1,0) = 10;
    Kp(0,1) = 10;
    Kp(1,1) = 16;
    Eigen::MatrixXd Kd(2,2);
    Kd(0,0) = 7;
    Kd(1,0) = 6;
    Kd(0,1) = 6;
    Kd(1,1) = 7;
    //PDController controller(Kp,Kd);
    
    ros::NodeHandle n;
    std::cout<<"ch0"<<std::endl;
    ControllerManager manager(&n);
    std::cout<<"ch1"<<std::endl;
    boost::shared_ptr<ControllerBase> my_controller(new PDController(Kp,Kd));
    manager.addController("PD",my_controller);
    


    ros::spin();
    return 0;
}
