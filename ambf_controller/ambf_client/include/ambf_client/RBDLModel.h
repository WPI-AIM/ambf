#ifndef RBDLModel_H
#define RBDLModel_H


#include <rbdl/rbdl.h>


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class RBDLParser
{

	private:
        Model* model = NULL;
        void CreateModel();

	public:
	    RBDLParser();

};

#endif // RBDLModel_H
