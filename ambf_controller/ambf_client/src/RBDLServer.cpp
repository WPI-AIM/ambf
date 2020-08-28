#include "ambf_client/RBDLServer.h"


RBDLServer::RBDLServer()
{
  //model = new RigidBodyDynamics::Model();
}

bool RBDLServer::ForwardDynamics(ambf_client::RBDLDynamicsRequestConstPtr req, ambf_client::RBDLDynamicsResponseConstPtr  res)
{

    VectorNd Q = VectorNd::Zero (model->q_size);
    VectorNd QDot = VectorNd::Zero (model->qdot_size);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    RigidBodyDynamics::ForwardDynamics (*model, Q, QDot, Tau, QDDot);
}
