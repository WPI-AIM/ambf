#include "controller_modules/ControllerNode.h"

#include <ambf_client/ambf_client.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Eigen/Core"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Main");
  ros::NodeHandle n;
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kd;
  Client client_;
  rigidBodyPtr handler_;
  client_.connect();
  handler_ = client_.getARigidBody("name", true);
  ControllerNode cnt(handler_, &n, Kp, Kd);

}
