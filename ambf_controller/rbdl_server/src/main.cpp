
#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include "rbdl_server/RBDLServer.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLDynamics.h"
#include "rbdl_server/RBDLModel.h"
#include <vector>
#include <array>


using namespace RigidBodyDynamics::Math;


void print(std::vector <double> const &a) {
   std::cout << "The vector elements are : ";

   for(int i=0; i < a.size(); i++)
   std::cout << a.at(i) << ' ';
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_main");
    ros::NodeHandle nh;
    ROS_INFO("starting");
    ros::ServiceClient client_model = nh.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_FD = nh.serviceClient<rbdl_server::RBDLDynamics>("ForwardDynamics");

    rbdl_server::RBDLModel model_msg; 
    rbdl_server::RBDLDynamics dny_msg;  
    
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
            ROS_ERROR("Failed to call service add_two_ints");
            return 1;
        }

                model_msg.request.model = "hello";

        
        dny_msg.request.q = q;
        dny_msg.request.qd = qd;
        dny_msg.request.tau = tau;
        if (client_FD.call(dny_msg))
        {
            print(dny_msg.response.qdd); 
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            return 1;
        }

    }
    

}
