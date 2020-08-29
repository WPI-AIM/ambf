#ifndef AMBFRBDLServer_H
#define AMBFRBDLServer_H


#include <ros/ros.h>
#include "ambf_client/RBDLDynamics.h"
#include <rbdl/rbdl.h>


class RBDLServer
{

	private:
        ros::NodeHandle nh_;
        void CreateModel();
        bool ForwardDynamics_srv(ambf_client::RBDLDynamicsRequestConstPtr, ambf_client::RBDLDynamicsResponseConstPtr  );

	public:
        RigidBodyDynamics::Model *model = NULL;

        RBDLServer(ros::NodeHandle* nodehandle);


};

#endif
