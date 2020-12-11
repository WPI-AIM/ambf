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
#include <afEnums.h>
#include <afMath.h>
#include <afPath.h>
#include <afUtils.h>

typedef unsigned int uint;

namespace ambf {

///
/// \brief The afKinematicAttributes struct
///
struct afKinematicAttributes{
public:
    afKinematicAttributes(){}

    afTransform m_location;
    double m_scale;
};


///
/// \brief The afPrimitiveShapeAttributes struct
///
struct afPrimitiveShapeAttributes{
public:

    afPrimitiveShapeAttributes(){
        m_posOffset.set(0, 0, 0);
        m_rotOffset.setIdentity();
        m_shapeType = afPrimitiveShapeType::INVALID;
        m_axisType = afAxisType::Z;
    }

    // Helper methods for primitive shapes
    // Required variables for creating a Plane
    void setPlaneData(double normal_x, double normal_y, double normal_z, double plane_constant){
        m_planeNormal.set(normal_x, normal_y, normal_z);
        m_planeConstant = plane_constant;
        m_shapeType = afPrimitiveShapeType::PLANE;
    }

    // Required variables for creating a Box
    void setBoxData(double dimension_x, double dimension_y, double dimension_z){
        m_dimensions.set(dimension_x, dimension_y, dimension_z);
        m_shapeType = afPrimitiveShapeType::BOX;
    }

    // Required variables for creating a Sphere
    void setSphereData(double radius){
        m_radius = radius;
        m_shapeType = afPrimitiveShapeType::SPHERE;
    }

    // Required variables for creating a Capsule
    void setCapsuleData(double radius, double height, afAxisType axis){
        m_radius = radius;
        m_height = height;
        m_axisType = axis;
        m_shapeType = afPrimitiveShapeType::CAPSULE;
    }

    // Required variables for creating a Cone
    void setConeData(double radius, double height, afAxisType axis){
        m_radius = radius;
        m_height = height;
        m_axisType = axis;
        m_shapeType = afPrimitiveShapeType::CONE;
    }

    // Pos Offset of the Shape
    void setPosOffset(double px, double py, double pz){
        m_posOffset.set(px, py, pz);
    }

    // Rot Offset of the Shape
    void setRotOffset(double roll, double pitch, double yaw){
        m_rotOffset.setRPY(roll, pitch, yaw);
    }

    inline void setShapeType(afPrimitiveShapeType shapeType){m_shapeType = shapeType;}

    inline void setAxisType(afAxisType axisType){m_axisType = axisType;}

    void setShapeScale(double a_scale){m_shapeScale = a_scale;}

    inline afVector3d getDimensions() const {return m_dimensions * m_shapeScale;}

    inline double getRadius() const {return m_radius * m_shapeScale;}

    inline double getHeight() const {return m_height * m_shapeScale;}

    inline double getPlaneConstant() const {return m_planeConstant * m_shapeScale;}

    inline afVector3d getPlaneNormal() const {return m_planeNormal;}

    inline afPrimitiveShapeType getShapeType() const {return m_shapeType;}

    inline afAxisType getAxisType() const {return m_axisType;}

    inline afVector3d getPosOffset() const {return m_posOffset;}

    inline afMatrix3d getRotOffset() const {return m_rotOffset;}

public:

    double m_radius = 0;

    double m_height = 0;

    afVector3d m_dimensions;

    afVector3d m_planeNormal;

    double m_planeConstant = 0;

    afPrimitiveShapeType m_shapeType;

    afAxisType m_axisType;

    afVector3d m_posOffset;

    afMatrix3d m_rotOffset;

private:
    double m_shapeScale = 1.0;
};


///
/// \brief The afRayAttributes struct
///
struct afRayAttributes{
    // Direction rel to parent that this sensor is looking at
    afVector3d m_direction;

    // Range of this sensor, i.e. how far can it sense
    double m_range;

    // Based on the location, direciton and range, calculate
    // start and end points for the ray tracing in Local Frame
    afVector3d m_rayFromLocal;
    afVector3d m_rayToLocal;
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
    afPath m_path;
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
    afVector3d m_inertia;
    bool m_estimateInertia = false;
    afTransform m_inertialOffset;
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


struct afColorAttributes{
    afColorAttributes(){}

    struct afRGBA{
        double m_R, m_G, m_B;
    };
    afRGBA m_specular;
    afRGBA m_diffuse;
    afRGBA m_emission;
    double m_ambient = 1.0;
    double m_alpha = 1.0;
    unsigned int m_shininiess = 64;

};


///
/// \brief The afVisualAttributes struct
///
struct afVisualAttributes{
    afVisualAttributes(){}

    std::string m_meshName;
    afPath m_path;
    afGeometryType m_geometryType;
    std::vector<afPrimitiveShapeAttributes> m_primitiveShapes;
    afColorAttributes m_colorAttribs;
};


///
/// \brief The afShaderAttributes struct
///
struct afShaderAttributes{
public:
    afShaderAttributes(){}

    bool m_shaderDefined;
    afPath m_path;
    std::string m_vtxShaderFileName;
    std::string m_fragShaderFileName;
};


struct afBaseObjectAttributes{
    // Base Struct that can be used to later cast specific Object Attributes
    afIdentificationAttributes m_identificationAttribs;

    virtual void resolveRelativeNamespace(string a_parentNamespace){
        m_identificationAttribs.m_namespace = afUtils::mergeNamespace(a_parentNamespace, m_identificationAttribs.m_namespace);
    }

    virtual void resolveRelativePathAttribs(afPath a_parentPath){}
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

    afVector3d m_lookAt;
    afVector3d m_up;
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
    afVector3d m_direction;

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
    afJointAttributes(){}

    afVector3d m_parentPivot;
    afVector3d m_childPivot;
    afVector3d m_parentAxis;
    afVector3d m_childAxis;
    afTransform m_transformInParent;
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

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_collisionAttribs.m_path.resolvePath(a_parentPath);
        m_visualAttribs.m_path.resolvePath(a_parentPath);
        m_shaderAttribs.m_path.resolvePath(a_parentPath);
    }
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

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_collisionAttribs.m_path.resolvePath(a_parentPath);
        m_visualAttribs.m_path.resolvePath(a_parentPath);
        m_shaderAttribs.m_path.resolvePath(a_parentPath);
    }
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
    afVector3d m_downDirection;
    afVector3d m_axelDirection;
    afVector3d m_offset;
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
    afPath m_wheelsVisualPath;
    std::vector<afWheelAttributes> m_wheelAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_wheelsVisualPath.resolvePath(a_parentPath);
    }
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

    virtual void resolveRelativePathAttribs(afPath a_parentPath){

    }
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


struct afInputDeviceAttributes: public afBaseObjectAttributes{
public:
    afInputDeviceAttributes(){}

    bool m_enableSDEJointControl;
    double m_deadBand;
    double m_maxForce;
    double m_maxJerk;
    double m_workspaceScale;
    bool m_sdeDefined = false;
    bool m_rootLinkDefined = false; bool m_visible;
    double m_visibleSize;

    std::string m_hardwareName;
    std::string m_rootLink;
    afPath m_sdeFilepath;
    std::vector<std::string> m_pairedCamerasNames;

    afCartesianControllerAttributes m_IIDControllerAttribs;
    afCartesianControllerAttributes m_SDEControllerAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afTransform m_orientationOffset;

    struct afButtons{
        int A1; // Action 1 Button
        int A2; // Action 2 Button
        int G1; // Gripper 1 Button
        int NEXT_MODE; // Next Mode Button
        int PREV_MODE; // Prev Mode Button
    };

    afButtons m_buttons;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_sdeFilepath.resolvePath(a_parentPath);
    }
};


struct afFileObjectAttributes{
public:
    afFileObjectAttributes(){}

    afPath m_path;
};


///
/// \brief The afMultiBodyAttributes struct
///
struct afMultiBodyAttributes: public afBaseObjectAttributes, public afFileObjectAttributes{
public:
    afMultiBodyAttributes(){}

    afPath m_visualMeshesPath;
    afPath m_collisionMeshesPath;

    std::vector <afRigidBodyAttributes> m_rigidBodyAttribs;
    std::vector <afSoftBodyAttributes> m_softBodyAttribs;
    std::vector <afVehicleAttributes> m_vehicleAttribs;
    std::vector <afJointAttributes> m_jointAttribs;
    std::vector <afSensorAttributes> m_sensorAttribs;
    std::vector <afActuatorAttributes> m_actuatorAttribs;

    bool m_ignoreInterCollision;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_path.resolvePath(a_parentPath);
    }
};


// Struct for multiple input devices
struct afAllInputDevicesAttributes: public afFileObjectAttributes{
public:
    afAllInputDevicesAttributes(){}
    std::vector <afInputDeviceAttributes> m_inputDeviceAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){

    }
};



///
/// \brief The afWorldAttributes struct
///
struct afWorldAttributes: public afBaseObjectAttributes, public afFileObjectAttributes{
public:
    afWorldAttributes(){}

    std::vector<afLightAttributes> m_lightAttribs;
    std::vector<afCameraAttributes> m_cameraAttribs;

    afVector3d m_gravity;
    uint m_maxIterations;
    afPath m_environmentFilePath;
    afShaderAttributes m_shaderAttribs;
    std::string m_namespace;

    struct afSkyBoxAttributes{
        afPath m_path;
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

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_skyBoxAttribs.m_path.resolvePath(a_parentPath);
        m_shaderAttribs.m_path.resolvePath(a_parentPath);
        m_environmentFilePath.resolvePath(a_parentPath);
    }
};


struct afLaunchAttributes: public afFileObjectAttributes{
    afPath m_colorFilepath;
    std::vector<afPath> m_multiBodyFilepaths;
    afPath m_worldFilePath;
    afPath m_inputDevicesFilepath;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        m_colorFilepath.resolvePath(a_parentPath);
        m_worldFilePath.resolvePath(a_parentPath);
        m_inputDevicesFilepath.resolvePath(a_parentPath);

        for (int i = 0 ; i < m_multiBodyFilepaths.size() ; i++){
            m_multiBodyFilepaths[i].resolvePath(a_parentPath);
        }
    }
};

}


#endif
