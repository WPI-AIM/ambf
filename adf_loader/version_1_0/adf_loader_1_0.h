//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_ADF_LOADER_1_0_H
#define AF_ADF_LOADER_1_0_H
//------------------------------------------------------------------------------

#include "adf_loader_base.h"

using namespace ambf;
using namespace std;

namespace adf_loader_1_0{

class ADFUtils{
public:
    static bool getCartControllerAttribsFromNode(YAML::Node* a_node, afCartesianControllerAttributes* attribs);

    static bool getCollisionAttribsFromNode(YAML::Node* a_node, afCollisionAttributes* attribs);

    static bool getCommunicationAttribsFromNode(YAML::Node* a_node, afCommunicationAttributes* attribs);

    static bool getHierarchyAttribsFromNode(YAML::Node* a_node, afHierarchyAttributes* attribs);

    static bool getIdentificationAttribsFromNode(YAML::Node* a_node, afIdentificationAttributes* attribs);

    static bool getInertialAttrisFromNode(YAML::Node* a_node, afInertialAttributes* attribs);

    static bool getJointControllerAttribsFromNode(YAML::Node* a_node, afJointControllerAttributes* attribs);

    static bool getKinematicAttribsFromNode(YAML::Node* a_node, afKinematicAttributes* attribs);

    static bool getColorAttribsFromNode(YAML::Node* a_node, afColorAttributes* a_color);

    static bool getShaderAttribsFromNode(YAML::Node* a_node, afShaderAttributes* attribs);

    static bool getVisualAttribsFromNode(YAML::Node* a_node, afVisualAttributes* attribs);

    static bool getSurfaceAttribsFromNode(YAML::Node* a_node, afSurfaceAttributes* attribs);

    static bool getWheelAttribsFromNode(YAML::Node* a_node, afWheelAttributes* attribs);

    static bool getPluginAttribsFromNode(YAML::Node* a_node, vector<afPluginAttributes>* attribs);

    static afActuatorType getActuatorTypeFromString(const string & a_str);

    static afJointType getJointTypeFromString(const string & a_str);

    static afSensorType getSensorTypeFromString(const string & a_str);

    static afPrimitiveShapeType getPrimitiveShapeTypeFromString(const string & a_shape_str);

    static afCollisionMeshShapeType getCollisionMeshShapeTypeFromString(const string & a_shape_str);

    static afControlType getControlTypeFromString(const string & a_control_str);

    // Copy data specified via ADF node
    static bool copyShapeOffsetData(YAML::Node* offsetNode, afPrimitiveShapeAttributes* attribs);

    // Copy data specified via ADF node
    static bool copyPrimitiveShapeData(YAML::Node* shapeNode, afPrimitiveShapeAttributes* attribs);

    static afVector3d positionFromNode(YAML::Node* node);

    static afMatrix3d rotationFromNode(YAML::Node* node);

    static void saveRawData(YAML::Node* node, afBaseObjectAttributes* a_attribs);

    static void saveRawData(YAML::Node* node, afLaunchAttributes* a_attribs);
};


///
/// \brief The ADFLoader class
///
class ADFLoader_1_0: public ADFLoaderBase{
public:
    ADFLoader_1_0();

    // Must set this version string in the constructor of each loader.
    using ADFLoaderBase::m_version;

    virtual string getLoaderVersion();

    virtual bool loadObjectAttribs(YAML::Node* a_node, string a_objName, afType a_objType, afBaseObjectAttributes* a_objAttribs);

    // Load Light
    virtual bool loadLightAttribs(YAML::Node* a_node, afLightAttributes* attribs);

    // Load Camera
    virtual bool loadCameraAttribs(YAML::Node* a_node, afCameraAttributes* attribs);

    // Load rigid body from a YAML::Node
    virtual bool loadRigidBodyAttribs(YAML::Node* a_node, afRigidBodyAttributes* attribs);

    // Load soft body from a YAML::Node
    virtual bool loadSoftBodyAttribs(YAML::Node* a_node, afSoftBodyAttributes* attribs);

    // Load ghost object from a YAML::Node
    virtual bool loadGhostObjectAttribs(YAML::Node* a_node, afGhostObjectAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadJointAttribs(YAML::Node* a_node, afJointAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadSensorAttribs(YAML::Node* a_node, afSensorAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadRayTracerSensorAttribs(YAML::Node* a_node, afRayTracerSensorAttributes* attribs);

    // Load joint from a YAML::Node
    virtual bool loadResistanceSensorAttribs(YAML::Node* a_node, afResistanceSensorAttributes* attribs);

    // Load actuator from a YAML::Node
    virtual bool loadActuatorAttribs(YAML::Node* a_node, afActuatorAttributes* attribs);

    // Load actuator from a YAML::Node
    virtual bool loadConstraintActuatorAttribs(YAML::Node* a_node, afConstraintActuatorAttributes* attribs);

    // Load sensor from a YAML::Node
    virtual bool loadVehicleAttribs(YAML::Node* a_node, afVehicleAttributes* attribs);

    // Load sensor from a YAML::Node
    virtual bool loadVolumeAttribs(YAML::Node* a_node, afVolumeAttributes* attribs);

    // Load Input Device Attributes
    virtual bool loadInputDeviceAttribs(YAML::Node* a_node, afInputDeviceAttributes *attribs);

    // Load Simulated Device Attributes
    virtual bool loadSimulatedDeviceAttribs(YAML::Node* a_node, afSimulatedDeviceAttribs *attribs);

    // Load all the input device attributes
    virtual bool loadTeleRoboticUnitsAttribs(string, vector<afTeleRoboticUnitAttributes> *attribs, vector<int> dev_indexes);

    // Load model from ADF file
    virtual bool loadModelAttribs(string, afModelAttributes* attribs);

    // Load world from ADF file
    virtual bool loadWorldAttribs(string, afWorldAttributes* attribs);

    // Load Launch File Attribs
    virtual bool loadLaunchFileAttribs(string, afLaunchAttributes* attribs);

    // Load all the input device attributes
    bool loadTeleRoboticUnitsAttribs(YAML::Node* a_node, string a_filepath, vector<afTeleRoboticUnitAttributes>* attribs, vector<int> dev_indexes);

    // Load model from ADF file
    bool loadModelAttribs(YAML::Node* a_node, afModelAttributes* attribs);

    // Load world from ADF file
    bool loadWorldAttribs(YAML::Node* a_node, afWorldAttributes* attribs);

    // Load Launch File Attribs
    bool loadLaunchFileAttribs(YAML::Node* a_node, afLaunchAttributes* attribs);
};
}
#endif
