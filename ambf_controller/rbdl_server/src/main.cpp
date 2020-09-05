
#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include "rbdl_server/RBDLServer.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLForwardDynamics.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLKinimatics.h"
#include <vector>
#include <array>


using namespace RigidBodyDynamics::Math;


void print(std::vector <double> const &a) 
{
   std::cout << "The vector elements are : ";

   for(int i=0; i < a.size(); i++)
   {
        ROS_INFO( "%f", a.at(i) );
   }
}

void msgToEigen(const std_msgs::Float64MultiArray& msg, Eigen::MatrixXd& new_mat)
{
	float dstride0 = msg.layout.dim[0].stride;
	float dstride1 = msg.layout.dim[1].stride;
	float h = msg.layout.dim[0].size;
	float w = msg.layout.dim[1].size;

	// Below are a few basic Eigen demos:
	std::vector<double> data = msg.data;
	new_mat = Eigen::Map<Eigen::MatrixXd>(data.data(), h, w);
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_main");
    ros::NodeHandle nh;
    ROS_INFO("starting");
    ros::ServiceClient client_model = nh.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_FD = nh.serviceClient<rbdl_server::RBDLForwardDynamics>("ForwardDynamics");
    ros::ServiceClient client_ID = nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_Jac = nh.serviceClient<rbdl_server::RBDLJacobian>("Jacobian");
    ros::ServiceClient client_kin = nh.serviceClient<rbdl_server::RBDLKinimatics>("ForwardKinimatics");


    rbdl_server::RBDLModel model_msg; 
    rbdl_server::RBDLForwardDynamics Fordny_msg; 
    rbdl_server::RBDLInverseDynamics Invdny_msg;
    rbdl_server::RBDLJacobian Jac_msg; 
    rbdl_server::RBDLKinimatics Kin_msg;  
    
    const int dof = 3;
    std::vector<double> q{{0.5, 0.1, 1.0}};
    std::vector<double> qd{{0.0, 0.0, 0.0}};
    std::vector<double> qdd{{0.0, 0.0, 0.0}};
    std::vector<double> tau{{0.0, 0.0, 0.0}};
    
    if(ros::ok)
    {
      
        model_msg.request.model = "hello";
        if (client_model.call(model_msg))
        {
            ROS_INFO("built the model");
        }
        else
        {
            ROS_ERROR("Failed to call service model");
            return 1;
        }

        
        Fordny_msg.request.q = q;
        Fordny_msg.request.qd = qd;
        Fordny_msg.request.tau = tau;
        if (client_FD.call(Fordny_msg))
        {
            print(Fordny_msg.response.qdd); 
        }
        else
        {
            ROS_ERROR("Failed to call service FD");
            return 1;
        }

        Invdny_msg.request.q = q;
        Invdny_msg.request.qd = qd;
        Invdny_msg.request.qdd = qdd;
        if (client_ID.call(Invdny_msg))
        {
            print(Invdny_msg.response.tau);
        }
        else
        {
            ROS_ERROR("Failed to call service ID");
            return 1;
        }

        Jac_msg.request.q = q;
        Jac_msg.request.body_name = "bodyB";
        Jac_msg.request.point.x = 0.0;
        Jac_msg.request.point.y = 0.0;
        Jac_msg.request.point.z = 0.0;
        if (client_Jac.call(Jac_msg))
        {
            Eigen::MatrixXd  mat;
            msgToEigen(Jac_msg.response.jacobian, mat);
            std::cout << mat << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call service Jac");
            return 1;
        }

        Kin_msg.request.q = q;
    
        if (client_kin.call(Kin_msg))
        {
            for(auto name: Kin_msg.response.names)
            {
                std::cout << name << std::endl;
            }
            for(auto point: Kin_msg.response.points)
            {
                std::cout << point.x << std::endl;
            }
            
        }
        else
        {
            ROS_ERROR("Failed to call service kin");
            return 1;
        }

    }



    

}
