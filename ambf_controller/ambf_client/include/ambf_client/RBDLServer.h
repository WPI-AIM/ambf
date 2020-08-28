#ifndef RBDLServer_H
#define RBDLServer_H


#include <rbdl/rbdl.h>
#include "ambf_client/RBDLDynamics.h"


using namespace RigidBodyDynamics::Math;


class RBDLServer
{

	private:
        RigidBodyDynamics::Model* model = NULL;
        void CreateModel();
        bool ForwardDynamics(ambf_client::RBDLDynamicsRequestConstPtr, ambf_client::RBDLDynamicsResponseConstPtr  );

	public:
	    RBDLServer();

};

#endif // RBDLModel_H
