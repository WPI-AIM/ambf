//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu, nagoldfarb@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar, Nathaniel Goldfarb
    \version   1.0$
*/
//==============================================================================

#ifndef AMBFRBDLServer_H
#define AMBFRBDLServer_H


#include <ros/ros.h>

#include "rbdl_server/RBDLForwardDynamics.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLBodyNames.h"
#include <unordered_map>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <rbdl/rbdl.h>
#include <eigen_conversions/eigen_msg.h>
// #include "rbdl_model/BuildRBDLModel.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class RBDLServer
{
	private:
                ros::NodeHandle nh_;
                RigidBodyDynamics::Model *model = NULL;
                bool have_model;
                std::unordered_map<std::string, unsigned int> body_ids; //body ids
                ros::ServiceServer FD_srv, ID_srv, MD_srv, Jac_srv, Kin_srv;
                VectorNd VectToEigen(const std::vector<double>&);
                RigidBodyDynamics::Model* getModel();
                bool CreateModel_srv(rbdl_server::RBDLModelRequest&, rbdl_server::RBDLModelResponse& ); //parses the AMBF model into  rbdl model
                bool CheckSize(int); //need to implement this to find way of checking the msg field sizes
                bool ForwardDynamics_srv(rbdl_server::RBDLForwardDynamicsRequest&, rbdl_server::RBDLForwardDynamicsResponse&  );
                bool InverseDynamics_srv(rbdl_server::RBDLInverseDynamicsRequest&, rbdl_server::RBDLInverseDynamicsResponse&  );
                bool ForwardKinimatics_srv(rbdl_server::RBDLKinimaticsRequest&, rbdl_server::RBDLKinimaticsResponse&);
                bool Jacobian_srv(rbdl_server::RBDLJacobianRequest&, rbdl_server::RBDLJacobianResponse&);
                bool GetNames_srv(rbdl_server::RBDLBodyNamesRequest&, rbdl_server::RBDLBodyNamesResponse&);
                void GetNames(std::vector<std::string>&);
	
        public:
                RBDLServer(ros::NodeHandle* nodehandle);
                RBDLServer();
              ~RBDLServer();


};

#endif
