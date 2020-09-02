
#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include "rbdl_server/RBDLServer.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLDynamics.h"
#include "rbdl_server/RBDLModel.h"
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

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_main");
    ros::NodeHandle nh;
    ROS_INFO("starting");
    ros::ServiceClient client_model = nh.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_FD = nh.serviceClient<rbdl_server::RBDLDynamics>("ForwardDynamics");
    ros::ServiceClient client_ID = nh.serviceClient<rbdl_server::RBDLDynamics>("InverseDynamics");
    ros::ServiceClient client_Jac = nh.serviceClient<rbdl_server::RBDLJacobian>("Jacobian");


    rbdl_server::RBDLModel model_msg; 
    rbdl_server::RBDLDynamics Fordny_msg; 
    rbdl_server::RBDLDynamics Invdny_msg;
    rbdl_server::RBDLJacobian Jac_msg;  
    
    const int dof = 3;
    std::vector<double> q{{0.0, 0.0, 0.0}};
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
        Jac_msg.request.body_name = "bodyA";
        if (client_Jac.call(Jac_msg))
        {
            std::cout<<"helle " <<std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call service Jac");
            return 1;
        }

    }



    

}
