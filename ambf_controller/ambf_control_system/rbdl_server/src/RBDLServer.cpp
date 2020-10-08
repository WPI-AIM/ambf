#include "rbdl_server/RBDLServer.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    
     model = new Model();
     have_model = false;
     FD_srv = nh_.advertiseService("ForwardDynamics", &RBDLServer::ForwardDynamics_srv, this);
     ID_srv = nh_.advertiseService("InverseDynamics", &RBDLServer::InverseDynamics_srv, this);
     MD_srv = nh_.advertiseService("CreateModel", &RBDLServer::CreateModel_srv, this);
     Jac_srv = nh_.advertiseService("Jacobian", &RBDLServer::Jacobian_srv, this);
     Kin_srv = nh_.advertiseService("ForwardKinimatics", &RBDLServer::ForwardKinimatics_srv, this);
     ROS_INFO("RBDL server running");
}

RBDLServer::~RBDLServer()
{
    delete model;
}

///
/// \brief RBDLServer::CreateModel_srv
/// \param req
/// \param res
///
bool RBDLServer::CreateModel_srv(rbdl_server::RBDLModelRequest& req, rbdl_server::RBDLModelResponse& res) //parses the AMBF model into  rbdl model
{

    // std::string actuator_config_file;
    // BuildRBDLModel  buildRBDLModel(actuator_config_file);
    // body_ids = buildRBDLModel.getRBDLBodyToIDMap();
    // *model = buildRBDLModel.getRBDLModel();
    // buildRBDLModel.cleanUp();

    return true;
}


///
/// \brief RBDLServer::ForwardDynamics_srv
/// \param req
/// \param res
///
bool  RBDLServer::ForwardDynamics_srv(rbdl_server::RBDLForwardDynamicsRequest& req, rbdl_server::RBDLForwardDynamicsResponse&  res )
{
    // Need to add some checks on the size of the inputs to make sure they are correct
    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (model->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
    if (model->qdot_size != req.qd.size())
    {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }
    if (model->qdot_size != req.tau.size() )
    {
        ROS_INFO("Joint torque (tau) not correct size");
        return false;
    }

    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd Tau = VectToEigen(req.tau);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
    std::vector<double> qdd(&QDDot[0], QDDot.data()+QDDot.cols()*QDDot.rows());
    res.qdd = qdd;
    return true;
}

///
/// \brief RBDLServer::InverseDynamics_srv
/// \param req
/// \param res
///
bool RBDLServer::InverseDynamics_srv(rbdl_server::RBDLInverseDynamicsRequest& req, rbdl_server::RBDLInverseDynamicsResponse&  res)
{
    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (model->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
    if (model->qdot_size != req.qd.size())
    {
        ROS_INFO("Joint velocity length (qd) not correct size");
        return false;
    }
    if (model->qdot_size != req.qdd.size() )
    {
        ROS_INFO("Joint accel (qdd) not correct size");
        return false;
    }

    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd QDDot = VectToEigen(req.qdd);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    InverseDynamics(*model, Q, QDot, QDDot, Tau );
    std::vector<double> tau(&Tau[0], Tau.data()+Tau.cols()*Tau.rows());
    res.tau = tau;

    return true;
}


///
/// \brief RBDLServer::ForwardKinimatics_srv
/// \param req
/// \param res
///
bool RBDLServer::ForwardKinimatics_srv(rbdl_server::RBDLKinimaticsRequest& req, rbdl_server::RBDLKinimaticsResponse& res)
{
    std::string key;
    int id;
    VectorNd Q = VectorNd::Zero(model->q_size);
    Vector3d point(0,0,0);
    geometry_msgs::Point current_point;
    std::vector<geometry_msgs::Point> points;
    std::vector<std::string> names;
    Vector3d fk;
    geometry_msgs::Pose pose;
    int size = res.points.size();

    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (model->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
    
    
    for(std::pair<std::string, int> body : body_ids)
    {
        key = body.first;
        id = body.second;
        fk = CalcBodyToBaseCoordinates(*model, Q, id, point, false);
        current_point.x = fk(0);
        current_point.y = fk(1);
        current_point.z = fk(2);
        points.push_back(current_point);
        names.push_back(key);
    }

    res.names = names;
    res.points = points;
    return true;

}

///
/// \brief RBDLServer::Jacobian_srv
/// \param req
/// \param res
///
bool RBDLServer::Jacobian_srv(rbdl_server::RBDLJacobianRequest& req, rbdl_server::RBDLJacobianResponse& res)
{
    
    std::vector<std::string> names;
    int id;
    std_msgs::Float64MultiArray msg;
    MatrixNd G (MatrixNd::Zero (6, model->dof_count));
    VectorNd Q = VectToEigen(req.q);
    Vector3d point(req.point.x, req.point.y, req.point.z);

    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }
    if (model->q_size != req.q.size())
    {
        ROS_INFO("Joint length (q) not correct size");
        return false;
    }
      

    if (body_ids.find(req.body_name) != body_ids.end()) 
    {
		id = body_ids[req.body_name];
	} 
    else 
    {
        ROS_INFO("That is not a body, the current bodies are");
        GetNames(names);
        for(std::string name: names)
        {
            ROS_INFO("%s", name);
        }

		return false;
	}

    
    CalcPointJacobian6D(*model, Q, id, point, G, false);
    tf::matrixEigenToMsg(G, msg);
    res.jacobian = msg;
    return true;
    
}

///
/// \brief RBDLServer::GetNames_srv
/// \param req
/// \param res
///
bool RBDLServer::GetNames_srv(rbdl_server::RBDLBodyNamesRequest& req, rbdl_server::RBDLBodyNamesResponse& res)
{
    
    std::vector<std::string> names;
    //check to see if the model is active
    if(!have_model)
    {
        ROS_INFO("Model not set");
        return false;
    }

    GetNames(names);
    res.names = names;

    return true;
}

///
/// \brief RBDLServer::VectToEigen
/// \param msg
///
VectorNd RBDLServer::VectToEigen(const std::vector<double> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    VectorNd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}

///
/// \brief RBDLServer::GetNames
/// \param names
///
void RBDLServer::GetNames(std::vector<std::string>& names)
{
    //loop through the names and add them to the vect
    for(std::pair<std::string, int> body : body_ids)
    {
        names.push_back(body.first);

    }

}



