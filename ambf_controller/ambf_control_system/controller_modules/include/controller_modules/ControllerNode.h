#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H

#include <Eigen/Core>
#include <ambf_client/ambf_client.h>
#include <ambf_client/RigidBody.h>
#include "rbdl_server/RBDLForwardDynamics.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLBodyNames.h"
#include "ros/ros.h"
#include "controller_modules/PDController.h"
#include <vector>
#include "trajectory_generator/trajectory.h"
#include <boost/thread/thread.hpp>
#include "std_msgs/Empty.h"

class ControllerNode
{

    public:
        ControllerNode(rigidBodyPtr, ros::NodeHandle*, const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::MatrixXd>&);
        void setGain(const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::MatrixXd>&);
        void updataPath(const trajectory_generator::trajectory&);
        bool startController();

   private:
        Eigen::VectorXd VectToEigen(const std::vector<double> &msg);
        Eigen::VectorXd VectToEigen(const std::vector<float> &msg);
        void startControllerCallback(const std_msgs::Empty );
        void stopControllerCallback(const std_msgs::Empty );
        void control();
        ros::Subscriber start_controller;
        ros::Subscriber stop_controller;
        rigidBodyPtr handle;
        bool have_path;
        bool running;
        int path_index;
        int path_length;
        ros::NodeHandle n;
        ros::ServiceClient client_ID; //nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
        //
        PDController controller;
        Eigen::VectorXd desired_pos; //=  VectToEigen(pos_vec);
        Eigen::VectorXd desired_vel;
        Eigen::VectorXd desired_accel ;// =  VectToEigen(vel_vec);
        trajectory_generator::trajectory path;
        boost::thread run;
};

#endif // CONTROLLERNODE_H
