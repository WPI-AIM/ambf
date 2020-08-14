#ifndef RBDLPARSER_H
#define RBDLPARSER_H


#include <rbdl/rbdl.h>


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class RBDLParser
{

	private:
        Model* model = NULL;
		void CreateModel(ambf_client::RBDlInit::Request  &req, ambf_client::RBDlInit::Response &res);
	    void FowardDynamics(ambf_client::RBDLDyn::Request  &req, ambf_client::RBDLDyn::Response &res);
	    void InverseDynamics(ambf_client::RBDLDyn::Request  &req, ambf_client::RBDLDyn::Response &res);
        void CalcPointJacobian6D(void);
        void CalcPointJacobian(void);
        void CalcBodyToBaseCoordinates(void);

	public:
	    RBDLParser();

};

#endif // RBDLPARSER_H
