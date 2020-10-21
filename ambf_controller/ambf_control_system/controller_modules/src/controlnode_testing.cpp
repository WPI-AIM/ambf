#include "gtest/gtest.h"
#include "controller_modules/PDController.h"
#include "controller_modules/ControllerNode.h"
#include "Eigen/Core"
#include <ambf_client/ambf_client.h>
#include <ros/ros.h>
#include <ros/master.h>
#include "trajectory_generator/trajectory.h"
#include "sensor_msgs/JointState.h"

class ControllerNode_Test: public ::testing::Test
{
    protected:
        ControllerNode_Test();
        Client client_;
        rigidBodyPtr psm_baselink_handler_;
};

ControllerNode_Test::ControllerNode_Test()
{
    client_.connect();
    string psm_baselink = "psm/baselink";
    cout << "psm_baselink: " << psm_baselink << "\n";
    psm_baselink_handler_ = client_.getARigidBody(psm_baselink, true);
    usleep(1000000);
}


//TEST_F(ControllerNode_Test, TestRunning )
//{
//    Eigen::MatrixXd Kp(7,7);
//    Kp(0,0) = 5;
//    Kp(1,0) = 10;
//    Kp(0,1) = 10;
//    Kp(1,1) = 16;
//    Eigen::MatrixXd Kd(7,7);
//    Kd(0,0) = 7;
//    Kd(1,0) = 6;
//    Kd(0,1) = 6;
//    Kd(1,1) = 7;
//    PDController pdcontroller(Kp,Kd);
//    ros::NodeHandle n;
//    ControllerNode controller(psm_baselink_handler_, &n, pdcontroller);

//    controller.startController();
//    ASSERT_TRUE(controller.isRunning());

//}


TEST_F(ControllerNode_Test, TestTraj )
{
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
    PDController pdcontroller(Kp,Kd);
    ros::NodeHandle n;
    ControllerNode controller(psm_baselink_handler_, &n, pdcontroller);
    trajectory_generator::trajectory msg;
    sensor_msgs::JointState s1, s2, s3;
    std::vector<double> p1, p2, p3, v1, v2, v3;

    for( unsigned int i= 0; i<7; i++)
    {
        p1.push_back(1);
        p2.push_back(1);
        p3.push_back(1);
        v1.push_back(0.0);
        v2.push_back(0.0);
        v3.push_back(0.0);
    }

    s1.position = p1;
    s2.position = p2;
    s3.position = p3;
    s1.position = v1;
    s2.position = v2;
    s3.position = v3;

    msg.traj.push_back(s1);
    msg.traj.push_back(s2);
    msg.traj.push_back(s3);
    controller.startController();
    controller.updataPath(msg);


}


//TEST_F(ControllerNode_Test, TestPath )
//{
//    Eigen::MatrixXd Kp(2,2);
//    Kp(0,0) = 5;
//    Kp(1,0) = 10;
//    Kp(0,1) = 10;
//    Kp(1,1) = 16;
//    Eigen::MatrixXd Kd(2,2);
//    Kd(0,0) = 7;
//    Kd(1,0) = 6;
//    Kd(0,1) = 6;
//    Kd(1,1) = 7;
//    PDController pdcontroller(Kp,Kd);
//    ros::NodeHandle n;
//    ControllerNode controller(psm_baselink_handler_, &n, pdcontroller);

//    controller.startController();
//    ASSERT_TRUE(controller.isRunning());

//}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

