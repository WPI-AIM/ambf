#ifndef RBDLServer_H
#define RBDLServer_H


#include <rbdl/rbdl.h>
#include "ambf_client/RBDLDynamics.h"

using namespace RigidBodyDynamics;

using namespace RigidBodyDynamics::Math;


class RBDLServer
{

	private:
        Model* model = NULL;
        void CreateModel();

	public:
	    RBDLServer();

};

#endif // RBDLModel_H
