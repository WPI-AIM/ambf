#ifndef AMBFRBDLServer_H
#define AMBFRBDLServer_H


#include <ros/ros.h>
#include "rbdl_server/RBDLDynamics.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class RBDLServer
{

	private:
        ros::NodeHandle nh_;
        RigidBodyDynamics::Model *model = NULL;
        std::map<std::string, int> body_ids; //body ids
        ros::ServiceServer FD_srv, ID_srv;
        VectorNd VectToEigen(const std::vector<double>&);
        bool CreateModel(); //parses the AMBF model into  rbdl model
        bool CheckSize(int); //need to implement this to find way of checking the msg field sizes
        bool ForwardDynamics_srv(rbdl_server::RBDLDynamicsRequest&, rbdl_server::RBDLDynamicsResponse&  );
        bool InverseDynamics_srv(rbdl_server::RBDLDynamicsRequest&, rbdl_server::RBDLDynamicsResponse&  );
        bool ForwardKinmatics_srv();
        bool Jacobian_srv();
	public:
        RBDLServer(ros::NodeHandle* nodehandle);
        RBDLServer();
        ~RBDLServer();


};

#endif
