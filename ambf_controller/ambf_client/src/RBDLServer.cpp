#include "ambf_client/RBDLServer.h"


RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

    model = new RigidBodyDynamics::Model();
    //ros::ServiceServer service = nh_.advertiseService("ForwardDynamics", RBDLServer::ForwardDynamics_srv );
}

bool RBDLServer::ForwardDynamics_srv(ambf_client::RBDLDynamicsRequestConstPtr req, ambf_client::RBDLDynamicsResponseConstPtr  res)
{
//    RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Zero (model->q_size);
//    RigidBodyDynamics::Math::VectorNd QDot = RigidBodyDynamics::Math::VectorNd::Zero (model->qdot_size);
//    RigidBodyDynamics::Math::VectorNd Tau = RigidBodyDynamics::Math::VectorNd::Zero (model->qdot_size);
//    RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model->qdot_size);
//    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
    return true;
}
