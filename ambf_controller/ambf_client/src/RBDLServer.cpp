#include "ambf_client/RBDLServer.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

     model = new Model();
     this->FD_srv = nh_.advertiseService("ForwardDynamics", &RBDLServer::ForwardDynamics_srv, this);
     this->ID_srv = nh_.advertiseService("InverseDynamics", &RBDLServer::InverseDynamics_srv, this);
}

RBDLServer::~RBDLServer()
{
    delete model;
}

bool RBDLServer::ForwardDynamics_srv(ambf_client::RBDLDynamicsRequest& req, ambf_client::RBDLDynamicsResponse&  res)
{
    VectorNd Q = VectorNd::Zero (model->q_size);
    VectorNd QDot = VectorNd::Zero (model->qdot_size);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
    return true;
}

bool RBDLServer::InverseDynamics_srv(ambf_client::RBDLDynamicsRequest& req, ambf_client::RBDLDynamicsResponse&  res)
{
    VectorNd Q = VectorNd::Zero (model->q_size);
    VectorNd QDot = VectorNd::Zero (model->qdot_size);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    InverseDynamics(*model, Q, QDot, Tau, QDDot);
    return true;
}
