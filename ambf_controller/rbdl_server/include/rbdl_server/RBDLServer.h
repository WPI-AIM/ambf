#ifndef AMBFRBDLServer_H
#define AMBFRBDLServer_H


#include <ros/ros.h>
#include "rbdl_server/RBDLDynamics.h"
#include <rbdl/rbdl.h>
#include <boost/bind.hpp>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
class RBDLServer
{

	private:
        ros::NodeHandle nh_;
        RigidBodyDynamics::Model *model = NULL;
        ros::ServiceServer FD_srv, ID_srv;
        VectorNd VectToEigen(const std::vector<double>&);
        bool CreateModel();
        bool CheckSize(int);
        bool ForwardDynamics_srv(rbdl_server::RBDLDynamicsRequest&, rbdl_server::RBDLDynamicsResponse&  );
        bool InverseDynamics_srv(rbdl_server::RBDLDynamicsRequest&, rbdl_server::RBDLDynamicsResponse&  );

	public:
        RBDLServer(ros::NodeHandle* nodehandle);
        RBDLServer();
        ~RBDLServer();


};

#endif
