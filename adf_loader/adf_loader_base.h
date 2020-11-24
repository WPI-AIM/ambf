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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_ADF_LOADER_BASE_H
#define AF_ADF_LOADER_BASE_H
//------------------------------------------------------------------------------

#include "afAttributes.h"

using namespace ambf;


///
/// \brief The ADFLoader class
///
class ADFLoaderBase{
public:
    ADFLoaderBase();

    // Load rigid body from the ADF file with the name of the body specified
    virtual bool loadRigidBody(std::string rb_config_file, std::string node_name, afRigidBodyAttributes* attribs);

    // Load rigid body from a YAML::Node
    virtual bool loadRigidBody(YAML::Node* rb_node, afRigidBodyAttributes* attribs);

    // Load soft body from the ADF file with the name of the body specified
    virtual bool loadSoftBody(std::string sb_config_file, std::string node_name, afSoftBodyAttributes* attribs);

    // Load soft body from a YAML::Node
    virtual bool loadSoftBody(YAML::Node* sb_node, afSoftBodyAttributes* attribs);

    // Load joint from the ADF file with the name of the body specified
    virtual bool loadJoint(std::string jnt_config_file, std::string node_name, afJointAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadJoint(YAML::Node* jnt_node, afJointAttributes* attribs);

    // Load sensor from the ADF file with the name of the sensor specified
    virtual bool loadSensor(std::string sen_config_file, std::string node_name, afSensorAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadSensor(YAML::Node* sen_node, afSensorAttributes* attribs);

    // Load actuator from the ADF file with the name of the actuator specified
    virtual bool loadActutator(std::string act_config_file, std::string node_name, afActuatorAttributes* attribs);

    // Load actuator from a YAML::Node
    virtual bool loadActutator(YAML::Node* act_node, afActuatorAttributes* attribs);

    // Load sensor from the ADF file with the name of the sensor specified
    virtual bool loadVehicle(std::string vh_config_file, std::string node_name, afVehicleAttributes* attribs);

    // Load sensor from a YAML::Node
    virtual bool loadVehicle(YAML::Node* vh_node, afVehicleAttributes* attribs);

    // Load multibody from ADF file
    virtual bool loadMultiBody(std::string mb_config_file, afMultiBodyAttributes* attribs);

    // Load world from ADF file
    virtual bool loadWorld(std::string wd_config_file, afWorldAttributes* attribs);

public:
    static unsigned int getVersion(){return m_adfVersion;}

protected:

    static unsigned int m_adfVersion;

};
#endif
