#ifndef AMBFRBDLServer_H
#define AMBFRBDLServer_H


#include <ros/ros.h>
#include "ambf_client/RBDLDynamics.h"
#include <rbdl/rbdl.h>
#include <boost/bind.hpp>

class RBDLServer
{

	private:
        ros::NodeHandle nh_;
        ros::ServiceServer FD_srv, ID_srv;
        void CreateModel();
        bool ForwardDynamics_srv(ambf_client::RBDLDynamicsRequest&, ambf_client::RBDLDynamicsResponse&  );
        bool InverseDynamics_srv(ambf_client::RBDLDynamicsRequest&, ambf_client::RBDLDynamicsResponse&  );

	public:
        RigidBodyDynamics::Model *model = NULL;
        RBDLServer(ros::NodeHandle* nodehandle);
        RBDLServer();
        ~RBDLServer();


};

#endif
