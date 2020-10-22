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
#include "sensor_msgs/JointState.h"

class ControllerNode
{

    public:
        ControllerNode(rigidBodyPtr, ros::NodeHandle*, PDController&);
        void setGain(const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::MatrixXd>&);
        void updataPath(const trajectory_generator::trajectory&);
        bool startController();
        bool isRunning();
        int getStepCount();
        int getPathLength();
        Eigen::VectorXd getDesiredPos(); //=  VectToEigen(pos_vec);
        Eigen::VectorXd getDesiredVel();
        Eigen::VectorXd getDesiredAccel();
        void step();


   private:

        template<typename T, typename A>
        Eigen::VectorXd VectToEigen(std::vector<T,A> const& msg );
        ros::Publisher desired_pub;
        boost::mutex mtx_;
        void startControllerCallback(const std_msgs::Empty );
        void stopControllerCallback(const std_msgs::Empty );
        void controlloop();

        void updateState();
        std::vector<double> calcTorque(const std::vector<double>, const std::vector<double>);
        ros::Subscriber start_controller;
        ros::Subscriber stop_controller;
        rigidBodyPtr handle;
        bool have_path;
        bool running;
        int step_count;
        int path_length;
        ros::NodeHandle n;
        ros::ServiceClient client_ID; //nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
        PDController controller;
        std::vector<double> curr_pos, curr_vel;
        Eigen::VectorXd desired_pos; //=  VectToEigen(pos_vec);
        Eigen::VectorXd desired_vel;
        Eigen::VectorXd desired_accel ;// =  VectToEigen(vel_vec);
        trajectory_generator::trajectory path;
        boost::thread run;
};

#endif // CONTROLLERNODE_H
