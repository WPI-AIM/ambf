#include "ambf_client/RBDLServer.h"

RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

//   model = new RigidBodyDynamics::Model();
   model = new RigidBodyDynamics::Model();
   ROS_INFO("in class constructor of ExampleRosClass");

}

bool RBDLServer::ForwardDynamics_srv(ambf_client::RBDLDynamicsRequestConstPtr req, ambf_client::RBDLDynamicsResponseConstPtr  res)
{
    RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Zero (model->q_size);
  //    VectorNd QDot = VectorNd::Zero (model->qdot_size);
//    VectorNd Tau = VectorNd::Zero (model->qdot_size);
//    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
//    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
}
