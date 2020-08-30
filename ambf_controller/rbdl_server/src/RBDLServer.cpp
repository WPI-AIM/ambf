#include "rbdl_server/RBDLServer.h"


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RBDLServer::RBDLServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

     model = new Model();
     this->FD_srv = nh_.advertiseService("ForwardDynamics", &RBDLServer::ForwardDynamics_srv, this);
     this->ID_srv = nh_.advertiseService("InverseDynamics", &RBDLServer::InverseDynamics_srv, this);
     this->create_model = nh_.advertiseService("CreateModel", &RBDLServer::CreateModel_srv, this);
}

RBDLServer::~RBDLServer()
{
    delete model;
}

bool RBDLServer::CreateModel_srv(rbdl_server::RBDLModelRequest& req, rbdl_server::RBDLModelResponse& res) //parses the AMBF model into  rbdl model
{

    model->gravity = Vector3d (0., -9.81, 0.);

	Body body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
		Joint joint_a = Joint( JointTypeRevolute, Vector3d (0., 0., 1.)
	);
	
	int body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
	
	Body body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
		Joint joint_b = Joint ( JointTypeRevolute, Vector3d (0., 0., 1.)
	);
	
    int body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);
	
	Body body_c = Body (0., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
		Joint joint_c = Joint ( JointTypeRevolute, Vector3d (0., 0., 1.)
	);
	
	int body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

    body_ids["bodyA"] = body_a_id;
    body_ids["bodyB"] = body_b_id;
    body_ids["bodyC"] = body_c_id;

    return true;
}
VectorNd RBDLServer::VectToEigen(const std::vector<double> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    VectorNd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}

bool RBDLServer::ForwardDynamics_srv(rbdl_server::RBDLDynamicsRequest& req, rbdl_server::RBDLDynamicsResponse&  res)
{
    // Need to add some checks on the size of the inputs to make sure they are correct

    if (model->q_size != req.q.size()){return false;}
    if (model->qdot_size != req.qd.size()){return false;}
    if (model->qdot_size != req.tau.size() ){return false;}

    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd Tau = VectToEigen(req.tau);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);
    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
    std::vector<double> qdd(&QDDot[0], QDDot.data()+QDDot.cols()*QDDot.rows());
    res.qdd = qdd;
    return true;
}

bool RBDLServer::InverseDynamics_srv(rbdl_server::RBDLDynamicsRequest& req, rbdl_server::RBDLDynamicsResponse&  res)
{
    if (model->q_size != req.q.size()){return false;}
    if (model->qdot_size != req.qd.size()){return false;}
    if (model->qdot_size != req.qdd.size() ){return false;}

    VectorNd Q =  VectToEigen(req.q);
    VectorNd QDot = VectToEigen(req.qd);
    VectorNd QDDot = VectToEigen(req.qdd);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    InverseDynamics(*model, Q, QDot, QDDot, Tau );
    std::vector<double> tau(&Tau[0], Tau.data()+Tau.cols()*Tau.rows());
    res.tau = tau;

    return true;
}



bool RBDLServer::ForwardKinmatics_srv()
{
    std::string key;
    int id;
    VectorNd Q = VectorNd::Zero(model->q_size);
    Vector3d point(0,0,0);
    geometry_msgs::PoseArray points;
    std::vector<std::string> names;
    Vector3d fk;
    geometry_msgs::Pose pose;


    for(auto& body : body_ids)
    {
        key = body.first;
        id = body.second;
        fk = CalcBodyToBaseCoordinates(*model, Q, id, point, false);
        pose.position.x = fk(0);
        pose.position.y = fk(1);
        pose.position.z = fk(2);
        points.poses.push_back(pose);
        names.push_back(key);
    }

    return true;

}


bool RBDLServer::Jacobian_srv()
{
    MatrixNd G;
    VectorNd Q = VectorNd::Zero(model->q_size);
    Vector3d point(0,0,0);
    int id = 0;
    CalcPointJacobian6D(*model, Q, id, point, G, false);

//    for (int i = 0; i < nrow; ++i) {
//      for (int j = 0; j < ncol; ++j) {
//        std::cout << arr2[i][j] << " ";
//      }
//      std::cout << "\n";
//    }

}



