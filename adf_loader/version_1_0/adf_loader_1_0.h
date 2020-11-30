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
#ifndef AF_ADF_LOADER_1_0_H
#define AF_ADF_LOADER_1_0_H
//------------------------------------------------------------------------------

#include "adf_loader_interface.h"

using namespace ambf;

namespace adf_loader_1_0{

class ADFUtils: public ADFUtilsBase{
public:
    virtual bool getCartControllerAttribsFromNode(YAML::Node* a_node, afCartesianControllerAttributes* attribs);

    virtual bool getCollisionAttribsFromNode(YAML::Node* a_node, afCollisionAttributes* attribs);

    virtual bool getCommunicationAttribsFromNode(YAML::Node* a_node, afCommunicationAttributes* attribs);

    virtual bool getHierarchyAttribsFromNode(YAML::Node* a_node, afHierarchyAttributes* attribs);

    virtual bool getIdentificationAttribsFromNode(YAML::Node* a_node, afIdentificationAttributes* attribs);

    virtual bool getInertialAttrisFromNode(YAML::Node* a_node, afInertialAttributes* attribs);

    virtual bool getJointControllerAttribsFromNode(YAML::Node* a_node, afJointControllerAttributes* attribs);

    virtual bool getKinematicAttribsFromNode(YAML::Node* a_node, afKinematicAttributes* attribs);

    virtual bool getMatrialFromNode(YAML::Node* a_node, cMaterial* mat);

    virtual bool getShaderAttribsFromNode(YAML::Node* a_node, afShaderAttributes* attribs);

    virtual bool getVisualAttribsFromNode(YAML::Node* a_node, afVisualAttributes* attribs);

    virtual bool getSurfaceAttribsFromNode(YAML::Node* a_node, afSurfaceAttributes* attribs);

    static afJointType getJointTypeFromString(const std::string & a_joint_str);

    static afPrimitiveShapeType getShapeTypeFromString(const std::string & a_shape_str);

    // Copy data specified via ADF node
    static bool copyShapeOffsetData(YAML::Node* offsetNode, afPrimitiveShapeAttributes* attribs);

    // Copy data specified via ADF node
    static bool copyPrimitiveShapeData(YAML::Node* shapeNode, afPrimitiveShapeAttributes* attribs);

    template <typename T>
    ///
    /// \brief toXYZ
    /// \param node
    /// \return
    ///
    static T toXYZ(YAML::Node* node);


    template <typename T>
    ///
    /// \brief toRPY
    /// \param node
    /// \param v
    /// \return
    ///
    static T toRPY(YAML::Node* node);
};


///
/// \brief The ADFLoader class
///
class ADFLoader_1_0: public ADFLoaderBase{
public:
    ADFLoader_1_0();

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

    // Load sensor from the ADF file with the name of the sensor specified
    virtual bool loadRayTracerSensor(std::string sen_config_file, std::string node_name, afRayTracerSensorAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadRayTracerSensor(YAML::Node* sen_node, afRayTracerSensorAttributes* attribs);

    // Load sensor from the ADF file with the name of the sensor specified
    virtual bool loadResistanceSensor(std::string sen_config_file, std::string node_name, afResistanceSensorAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadResistanceSensor(YAML::Node* sen_node, afResistanceSensorAttributes* attribs);

    // Load actuator from the ADF file with the name of the actuator specified
    virtual bool loadActuator(std::string act_config_file, std::string node_name, afActuatorAttributes* attribs);

    // Load actuator from a YAML::Node
    virtual bool loadActuator(YAML::Node* act_node, afActuatorAttributes* attribs);

    // Load actuator from the ADF file with the name of the actuator specified
    virtual bool loadConstraintActuator(std::string act_config_file, std::string node_name, afConstraintActuatorAttributes* attribs);

    // Load actuator from a YAML::Node
    virtual bool loadConstraintActuator(YAML::Node* act_node, afConstraintActuatorAttributes* attribs);

    // Load sensor from the ADF file with the name of the sensor specified
    virtual bool loadVehicle(std::string vh_config_file, std::string node_name, afVehicleAttributes* attribs);

    // Load sensor from a YAML::Node
    virtual bool loadVehicle(YAML::Node* vh_node, afVehicleAttributes* attribs);

    // Load multibody from ADF file
    virtual bool loadMultiBody(std::string mb_config_file, afMultiBodyAttributes* attribs);

    // Load world from ADF file
    virtual bool loadWorld(std::string wd_config_file, afWorldAttributes* attribs);

public:

};
}
#endif
