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
#ifndef AF_ATTRIBUTES_H
#define AF_ATTRIBUTES_H
//------------------------------------------------------------------------------
#include <yaml-cpp/yaml.h>
#include <afEnums.h>
#include <chai3d.h>
#include <btBulletDynamicsCommon.h>
#include <boost/filesystem/path.hpp>

using namespace chai3d;


namespace ambf {

///
/// \brief The afKinematicAttributes struct
///
struct afKinematicAttributes{
public:
    afKinematicAttributes(){}

    cTransform m_location;
    double m_scale;
};


///
/// \brief The afPrimitiveShapeAttributes struct
///
struct afPrimitiveShapeAttributes{
public:

    afPrimitiveShapeAttributes();

    // Helper methods for primitive shapes
    // Required variables for creating a Plane
    void setPlaneData(double normal_x, double normal_y, double normal_z, double plane_constant);

    // Required variables for creating a Box
    void setBoxData(double dimension_x, double dimension_y, double dimension_z);

    // Required variables for creating a Sphere
    void setSphereData(double radius);

    // Required variables for creating a Capsule
    void setCapsuleData(double radius, double height, afAxisType axis);

    // Required variables for creating a Cone
    void setConeData(double radius, double height, afAxisType axis);

    // Pos Offset of the Shape
    void setPosOffset(double px, double py, double pz);

    // Rot Offset of the Shape
    void setRotOffset(double roll, double pitch, double yaw);

    inline void setShapeType(afPrimitiveShapeType shapeType){m_shapeType = shapeType;}

    inline void setAxisType(afAxisType axisType){m_axisType = axisType;}

    void setShapeScale(double a_scale){m_shapeScale = a_scale;}

    inline cVector3d getDimensions() const {return m_dimensions * m_shapeScale;}

    inline double getRadius() const {return m_radius * m_shapeScale;}

    inline double getHeight() const {return m_height * m_shapeScale;}

    inline double getPlaneConstant() const {return m_planeConstant * m_shapeScale;}

    inline cVector3d getPlaneNormal() const {return m_planeNormal;}

    inline afPrimitiveShapeType getShapeType() const {return m_shapeType;}

    inline afAxisType getAxisType() const {return m_axisType;}

    inline cVector3d getPosOffset() const {return m_posOffset;}

    inline cMatrix3d getRotOffset() const {return m_rotOffset;}

public:

    double m_radius = 0;

    double m_height = 0;

    cVector3d m_dimensions;

    cVector3d m_planeNormal;

    double m_planeConstant = 0;

    afPrimitiveShapeType m_shapeType;

    afAxisType m_axisType;

    cVector3d m_posOffset;

    cMatrix3d m_rotOffset;

private:
    double m_shapeScale = 1.0;
};


///
/// \brief The afRayAttributes struct
///
struct afRayAttributes{
    // Direction rel to parent that this sensor is looking at
    cVector3d m_direction;

    // Range of this sensor, i.e. how far can it sense
    double m_range;

    // Based on the location, direciton and range, calculate
    // start and end points for the ray tracing in Local Frame
    cVector3d m_rayFromLocal;
    cVector3d m_rayToLocal;
};


///
/// \brief The afTaxonomyAttributes struct
///
struct afIdentificationAttributes{
public:
    afIdentificationAttributes(){}

    std::string m_name;
    std::string m_namespace;
    afObjectType m_objectType;
};


///
/// \brief The afCollisionAttributes struct
///
struct afCollisionAttributes{
public:
    afCollisionAttributes(){}

    std::string m_meshName;
    boost::filesystem::path m_path;
    double m_margin;
    afGeometryType m_geometryType;
    std::vector<afPrimitiveShapeAttributes> m_primitiveShapes;
    std::vector<uint> m_groups;
};


///
/// \brief The afCommunicationAttributes struct
///
struct afCommunicationAttributes{
public:
    afCommunicationAttributes(){}

    uint m_minPublishFreq;
    uint m_maxPublishFreq;
    bool m_passive;
};


struct afCartesianControllerAttributes{
public:
    afCartesianControllerAttributes(){}

    // PID Controller Gains for Linear and Angular Controller
    double P_lin, I_lin, D_lin;
    double P_ang, I_ang, D_ang;

    // The default output type is velocity
    afControlType m_positionOutputType = afControlType::VELOCITY;

    // The default output type is velocity
    afControlType m_orientationOutputType = afControlType::VELOCITY;
};


///
/// \brief The afHeirarcyAttributes struct
///
struct afHierarchyAttributes{
public:
    afHierarchyAttributes(){}

    std::string m_parentName;
    std::string m_childName;
};


///
/// \brief The afSurfaceAttributes struct
///
struct afSurfaceAttributes{
public:
    afSurfaceAttributes(){
        m_linearDamping = 0.04;
        m_angularDamping = 0.1;
        m_staticFriction = 0.5;
        m_dynamicFriction = 0.5;
        m_rollingFriction = 0.01;
        m_restitution = 0.1;
    }
    double m_linearDamping;
    double m_angularDamping;
    double m_staticFriction;
    double m_dynamicFriction;
    double m_rollingFriction;
    double m_restitution;
};


///
/// \brief The afInertialAttributes struct
///
struct afInertialAttributes{
public:
    afInertialAttributes(){}

    double m_mass;
    btVector3 m_inertia;
    bool m_estimateInertia = false;
    btTransform m_inertialOffset;
    afSurfaceAttributes m_surfaceAttribs;
};


///
/// \brief The afJointControllerAttributes struct
///
struct afJointControllerAttributes{
public:
    afJointControllerAttributes(){}

    afControlType m_outputType = afControlType::VELOCITY;

    double P;
    double I;
    double D;

    double max_impulse;
};


///
/// \brief The afVisualAttributes struct
///
struct afVisualAttributes{
    afVisualAttributes(){}

    std::string m_meshName;
    boost::filesystem::path m_path;
    afGeometryType m_geometryType;
    std::vector<afPrimitiveShapeAttributes> m_primitiveShapes;
    cMaterial m_material;
};


///
/// \brief The afShaderAttributes struct
///
struct afShaderAttributes{
public:
    afShaderAttributes(){}

    bool m_shaderDefined;
    boost::filesystem::path m_path;
    boost::filesystem::path m_vtxShader;
    boost::filesystem::path m_fragShader;
};


struct afBaseObjectAttributes{
    // Base Struct that can be used to later cast specific Object Attributes
    afIdentificationAttributes m_identificationAttribs;
};


///
/// \brief The afActuatorAttributes struct
///
struct afActuatorAttributes: public afBaseObjectAttributes
{
public:
    afActuatorAttributes(){}

    afActuatorType m_actuatorType;

    afCommunicationAttributes m_communicationAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
};


///
/// \brief The afConstraintActuatorAttributes struct
///
struct afConstraintActuatorAttributes: public afActuatorAttributes{
public:
    afConstraintActuatorAttributes(){}

    bool m_visible;
    float m_visibleSize;
    float m_maxImpulse;
    float m_tau;
};



///
/// \brief The afCameraAttributes struct
///
struct afCameraAttributes: public afBaseObjectAttributes
{
public:
    afCameraAttributes(){}

    cVector3d m_lookAt;
    cVector3d m_up;
    float m_nearPlane;
    float m_farPlane;
    float m_fieldViewAngle;
    float m_orthoViewWidth;
    bool m_orthographic;
    bool m_stereo;
    float m_stereoEyeSeparation;
    float m_stereFocalLength;
    std::vector<std::string> m_controllingDeviceNames;
    uint m_monitorNumber;
    bool m_publishImage;
    bool m_publishDepth;
    uint m_publishImageInterval;
    uint m_publishDepthInterval;
    bool m_multiPass;

    afHierarchyAttributes m_hierarchyAttribs;
    afKinematicAttributes m_kinematicAttribs;
};


///
/// \brief The afLightAttributes struct
///
struct afLightAttributes: public afBaseObjectAttributes
{
public:
    afLightAttributes(){}

    float m_spotExponent;
    float m_cuttoffAngle;
    cVector3d m_direction;

    uint m_shadowQuality;

    afHierarchyAttributes m_hierarchyAttribs;
    afKinematicAttributes m_kinematicAttribs;
};



///
/// \brief The afJointAttributes struct
///
struct afJointAttributes: public afBaseObjectAttributes
{

public:
    afJointAttributes();

    cVector3d m_parentPivot;
    cVector3d m_childPivot;
    cVector3d m_parentAxis;
    cVector3d m_childAxis;
    cTransform m_transformInParent;
    bool m_enableMotor;
    bool m_enableFeedback;
    uint m_maxMotorImpulse;
    float m_lowerLimit;
    float m_upperLimit;
    float m_erp;
    float m_cfm;
    float m_offset;
    float m_damping;
    float m_stiffness;
    afJointType m_jointType;
    bool m_enableLimits;
    bool m_ignoreInterCollision;
    double m_equilibriumPoint;

    afHierarchyAttributes m_hierarchyAttribs;
    afCommunicationAttributes m_communicationAttribs;
    afJointControllerAttributes m_controllerAttribs;
};


///
/// \brief The afRigidBodyAttributes struct
///
struct afRigidBodyAttributes: public afBaseObjectAttributes
{
public:
    afRigidBodyAttributes(){}

    bool m_publishChildrenNames;
    bool m_publishJointNames;
    bool m_publishJointPositions;

    afCommunicationAttributes m_communicationAttribs;
    afCollisionAttributes m_collisionAttribs;
    afCartesianControllerAttributes m_controllerAttribs;
    afInertialAttributes m_inertialAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afVisualAttributes m_visualAttribs;
    afShaderAttributes m_shaderAttribs;
    afSurfaceAttributes m_surfaceAttribs;
};

///
/// \brief The afSoftBodyAttributes struct
///
struct afSoftBodyAttributes: public afBaseObjectAttributes
{
public:
    afSoftBodyAttributes(){}

    float m_kLST;
    float m_kAST;
    float m_kVST;
    float m_kVCF;
    float m_kDP;
    float m_kDG;
    float m_kLF;
    float m_kPR;
    float m_kVC;
    float m_kDF;
    float m_kMT;
    float m_kCHR;
    float m_kKHR;
    float m_kSHR;
    float m_kAHR;
    float m_kSRHR_CL;
    float m_kSKHR_CL;
    float m_kSSHR_CL;
    float m_kSR_SPLT_CL;
    float m_kSK_SPLT_CL;
    float m_kSS_SPLT_CL;
    float m_maxVolume;
    float m_timeScale;
    uint m_vIterations;
    uint m_pIterations;
    uint m_dIterations;
    uint m_cIterations;
    uint m_flags;
    uint m_bendingConstraint;
    uint m_clusters;
    bool m_randomizeConstraints;
    std::vector<uint> m_fixedNodes;

    afCommunicationAttributes m_communicationAttribs;
    afCollisionAttributes m_collisionAttribs;
    afCartesianControllerAttributes m_controllerAttribs;
    afInertialAttributes m_inertialAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afVisualAttributes m_visualAttribs;
    afShaderAttributes m_shaderAttribs;
};


///
/// \brief The afWheelAttributes struct
///
struct afWheelAttributes{
public:
    afWheelAttributes(){}

    double m_width;
    double m_radius;
    double m_friction;

    double m_rollInfluence;
    cVector3d m_downDirection;
    cVector3d m_axelDirection;
    cVector3d m_offset;
    bool m_isFront = false;
    double m_steeringLimitMin = 0.0;
    double m_steeringLimitMax = 0.0;
    double m_enginePowerMax = 0.0;
    double m_brakePowerMax = 0.0;

    afVisualAttributes m_visualAttribs;
    std::string m_wheelBodyName;
    afWheelRepresentationType m_representationType;

    struct afSuspensionAttributes{
        double m_stiffness;
        double m_damping;
        double m_compression;
        double m_restLength;
    };

    afSuspensionAttributes m_suspensionAttribs;


};

///
/// \brief The afVehicleAttributes struct
///
struct afVehicleAttributes: public afBaseObjectAttributes
{
public:
    afVehicleAttributes(){}
    std::string m_chassisBodyName;
    boost::filesystem::path m_wheelsVisualPath;
    std::vector<afWheelAttributes> m_wheelAttribs;
};


///
/// \brief The afSensorAttributes struct
///
struct afSensorAttributes: public afBaseObjectAttributes
{
public:
    afSensorAttributes(){}

    afSensorType m_sensorType;

    afCommunicationAttributes m_communicationAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
};



///
/// \brief The afResistanceSensorAttributes struct
///

struct afRayTracerSensorAttributes: public afSensorAttributes{
public:
    afRayTracerSensorAttributes(){}

    bool m_visible;
    float m_visibleSize;
    float m_maxImpulse;
    float m_tau;
    float m_range;

    std::string m_contourMesh;
    afSensactorSpecificationType m_specificationType;
    std::vector<afRayAttributes> m_raysAttribs;
};


///
/// \brief The afResistanceSensorAttributes struct
///

struct afResistanceSensorAttributes: public afRayTracerSensorAttributes{
public:
    afResistanceSensorAttributes(){}

    double m_contactNormalStiffness;
    double m_contactNormalDamping;
    double m_staticContactFriction;
    double m_staticContactDamping;
    double m_dynamicFriction;
    double m_contactArea;
    bool m_useVariableCoeff;
};



///
/// \brief The afMultiBodyAttributes struct
///
struct afMultiBodyAttributes: public afBaseObjectAttributes{
public:
    afMultiBodyAttributes(){}

    boost::filesystem::path m_visualMeshesPath;
    boost::filesystem::path m_collisionMeshesPath;

    std::string m_namespace;

    std::vector <afRigidBodyAttributes> m_rigidBodyAttribs;
    std::vector <afSoftBodyAttributes> m_softBodyAttribs;
    std::vector <afVehicleAttributes> m_vehicleAttribs;
    std::vector <afJointAttributes> m_jointAttribs;
    std::vector <afSensorAttributes> m_sensorAttribs;
    std::vector <afActuatorAttributes> m_actuatorAttribs;

    bool m_ignoreInterCollision;
};


struct afInputDeviceAttributes: public afIdentificationAttributes{
public:
    afInputDeviceAttributes();

    std::string m_hardwareName;

    afCartesianControllerAttributes m_IIDControllerAttribs;
    afCartesianControllerAttributes m_SDEControllerAttribs;

    bool m_enableSDEJointControl;
    double m_deadBand;
    double m_maxForce;
    double m_maxJerk;
    double m_workspaceScale;

    boost::filesystem::path m_sdeFilepath;
    std::string m_rootLink;

    bool m_sdeDefined = false;
    bool m_rootLinkDefined = false;

    afKinematicAttributes m_kinematicAttribs;
    cTransform m_orientationOffset;

    bool m_visible;
    double m_visibleSize;
    std::vector<std::string> m_pairedCamerasNames;

    struct afButtons{
        int A1; // Action 1 Button
        int A2; // Action 2 Button
        int G1; // Gripper 1 Button
        int NEXT_MODE; // Next Mode Button
        int PREV_MODE; // Prev Mode Button
    };

    afButtons m_buttons;
};


struct afTeleRoboticUnitAttributes{

    boost::filesystem::path m_path;
    std::vector <afInputDeviceAttributes> m_inputDeviceAttribs;
};



///
/// \brief The afWorldAttributes struct
///
struct afWorldAttributes: public afBaseObjectAttributes{
public:
    afWorldAttributes(){}

    std::vector<afLightAttributes> m_lightAttribs;
    std::vector<afCameraAttributes> m_cameraAttribs;

    cVector3d m_gravity;

    uint m_maxIterations;

    boost::filesystem::path m_environmentFilePath;

    struct afSkyBoxAttributes{

        boost::filesystem::path m_path;
        std::string m_leftImage;
        std::string m_frontImage;
        std::string m_rightImage;
        std::string m_backImage;
        std::string m_topImage;
        std::string m_bottomImage;

        afShaderAttributes m_shaderAttribs;

        bool m_use = false;
    };

    struct afEnclosure{
        double m_width;
        double m_height;
        double m_length;

        bool m_use = false;
    };

    afEnclosure m_enclosure;

    afSkyBoxAttributes m_skyBoxAttribs;

    std::string m_namespace;

    afShaderAttributes m_shaderAttribs;
};


struct afLaunchAttributes{
    boost::filesystem::path m_path;
    boost::filesystem::path m_colorFilepath;
    std::vector<boost::filesystem::path> m_multiBodyFilepaths;
    boost::filesystem::path m_worldFilePath;
    boost::filesystem::path m_inputDevicesFilepath;
};

}


#endif
