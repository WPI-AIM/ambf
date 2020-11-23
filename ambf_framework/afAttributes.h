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
#include "btBulletDynamicsCommon.h"
#include <boost/filesystem/path.hpp>

using namespace chai3d;


namespace ambf {

///
/// \brief The afPrimitiveShapeAttributes struct
///
struct afPrimitiveShapeAttributes{
public:

    afPrimitiveShapeAttributes();

    // Copy data specified via ADF node
    bool copyShapeOffsetData(YAML::Node* offsetNode);

    // Copy data specified via ADF node
    bool copyPrimitiveShapeData(YAML::Node* shapeNode);

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

    void setScale(double a_scale);

    inline cVector3d getDimensions() const {return m_dimensions * m_scale;}

    inline double getRadius() const {return m_radius * m_scale;}

    inline double getHeight() const {return m_height * m_scale;}

    inline double getPlaneConstant() const {return m_planeConstant * m_scale;}

    inline cVector3d getPlaneNormal() const {return m_planeNormal;}

    inline afPrimitiveShapeType getShapeType() const {return m_shapeType;}

    inline afAxisType getAxisType() const {return m_axisType;}

    inline cVector3d getPosOffset() const {return m_posOffset;}

    inline cMatrix3d getRotOffset() const {return m_rotOffset;}

private:
    double m_radius = 0;

    double m_height = 0;

    cVector3d m_dimensions;

    cVector3d m_planeNormal;

    double m_planeConstant = 0;

    afPrimitiveShapeType m_shapeType;

    afAxisType m_axisType;

    cVector3d m_posOffset;

    cMatrix3d m_rotOffset;

    double m_scale = 1.0;
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
struct afTaxonomyAttributes{
public:
    afTaxonomyAttributes(){}

    std::string m_name;
    std::string m_namespace;
};


///
/// \brief The afCollisionAttributes struct
///
struct afCollisionAttributes{
public:
    afCollisionAttributes(){}

    boost::filesystem::path m_collisionMeshFilePath;
    double m_collisionMargin;
    afGeometryType m_collisionGeometryType;
    std::vector<afPrimitiveShapeAttributes> m_collisionPrimitiveShapes;
    std::vector<uint> m_collisionGroups;
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
        m_linear_damping = 0.04;
        m_angular_damping = 0.1;
        m_static_friction = 0.5;
        m_dynamic_friction = 0.5;
        m_rolling_friction = 0.01;
        m_restitution = 0.1;
    }
    double m_linear_damping;
    double m_angular_damping;
    double m_static_friction;
    double m_dynamic_friction;
    double m_rolling_friction;
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
/// \brief The afKinematicAttributes struct
///
struct afKinematicAttributes{
public:
    afKinematicAttributes(){}

    cTransform m_location;
    double m_scale;
};


///
/// \brief The afVisualAttributes struct
///
struct afVisualAttributes{
    afVisualAttributes(){}

    std::string m_meshName;
    boost::filesystem::path m_visualMeshFilePath;
    afGeometryType m_visualGeometryType;
    std::vector<afPrimitiveShapeAttributes> m_visualPrimitiveShapes;
    cMaterial m_material;
};


///
/// \brief The afShaderAttributes struct
///
struct afShaderAttributes{
public:
    afShaderAttributes(){}

    bool m_shaderDefined;
    boost::filesystem::path m_vtxShaderFilePath;
    boost::filesystem::path m_fragShaderFilePath;
};


///
/// \brief The afActuatorAttributes struct
///
struct afActuatorAttributes:
        public afTaxonomyAttributes,
        public afCommunicationAttributes,
        public afKinematicAttributes,
        public afHierarchyAttributes
{
public:
    afActuatorAttributes(){}

    afActuatorType m_actuatorType;
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
struct afCameraAttributes:
        public afTaxonomyAttributes,
        public afHierarchyAttributes,
        public afKinematicAttributes
{
public:
    afCameraAttributes(){}

    cVector3d m_lookAt;
    cVector3d m_up;
    float m_nearPlane;
    float m_farPlane;
    float m_fieldViewAngle;
    float m_orthoViewWidth;
    bool m_stereo;
    std::vector<std::string> m_controllingDeviceNames;
    uint m_monitor;
    bool m_publishImage;
    bool m_publishDesph;
    uint m_publishImageInterval;
    uint m_publishDepthInterval;
    bool m_multiPass;
};


///
/// \brief The afLightAttributes struct
///
struct afLightAttributes:
        public afTaxonomyAttributes,
        public afHierarchyAttributes,
        public afKinematicAttributes
{
public:
    afLightAttributes(){}

    float m_spotExponent;
    float m_cuttoffAngle;
    cVector3d m_direction;

    afShadowQualityType m_shadowQuality;
};



///
/// \brief The afJointAttributes struct
///
struct afJointAttributes:
        public afTaxonomyAttributes,
        public afCommunicationAttributes,
        public afJointControllerAttributes,
        public afHierarchyAttributes
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
    float m_limitLow;
    float m_limitHigh;
    float m_erp;
    float m_cfm;
    float m_offset;
    float m_damping;
    float m_stiffness;
    afJointType m_jointType;
    afJointControllerAttributes m_controllerAttribs;
    bool m_ignoreInterCollision;
};


///
/// \brief The afRigidBodyAttributes struct
///
struct afRigidBodyAttributes:
        public afTaxonomyAttributes,
        public afCommunicationAttributes,
        public afCollisionAttributes,
        public afCartesianControllerAttributes,
        public afInertialAttributes,
        public afKinematicAttributes,
        public afVisualAttributes,
        public afShaderAttributes
{
public:
    afRigidBodyAttributes(){}

    bool m_publishChildrenNames;
    bool m_publishJointNames;
    bool m_publishJointPositions;
};

///
/// \brief The afSoftBodyAttributes struct
///
struct afSoftBodyAttributes:
        public afTaxonomyAttributes,
        public afCommunicationAttributes,
        public afCollisionAttributes,
        public afCartesianControllerAttributes,
        public afInertialAttributes,
        public afKinematicAttributes,
        public afVisualAttributes
{
public:
    afSoftBodyAttributes(){}

    float m_kLST;
    float mkAST;
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
    std::vector<uint> m_fixedNodes;

};

///
/// \brief The afVehicleAttributes struct
///
struct afVehicleAttributes:
        public afTaxonomyAttributes,
        public afVisualAttributes
{
public:
    afVehicleAttributes(){}



};


///
/// \brief The afSensorAttributes struct
///
struct afSensorAttributes:
        public afTaxonomyAttributes,
        public afCommunicationAttributes,
        public afKinematicAttributes,
        public afHierarchyAttributes
{
public:
    afSensorAttributes(){}

    bool m_visible;
    float m_visibleSize;
    float m_maxImpulse;
    float m_tau;
    float m_range;

    afSensorType m_sensorType;
};



///
/// \brief The afResistanceSensorAttributes struct
///

struct afRayTracerSensorAttributes: public afSensorAttributes{
public:
    afRayTracerSensorAttributes(){}

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
struct afMultiBodyAttributes: public afTaxonomyAttributes{
public:
    afMultiBodyAttributes(){}

    boost::filesystem::path m_highResPath;
    boost::filesystem::path m_lowResPath;

    std::vector <afRigidBodyAttributes> m_rigidBodyAttribs;
    std::vector <afSoftBodyAttributes> m_softBodyAttribs;
    std::vector <afVehicleAttributes> m_vehicleAttribs;
    std::vector <afJointAttributes> m_jointAttribs;
    std::vector <afSensorAttributes> m_sensorAttribs;
    std::vector <afActuatorAttributes> m_actuatorAttribs;

    bool m_ignoreInterCollision;
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
    double m_suspensionStiffness;
    double m_suspensionDamping;
    double m_suspensionCompression;
    double m_suspensionRestLength;
    double m_rollInfluence;
    cVector3d m_downDirection;
    cVector3d m_axelDirection;
    cVector3d m_offset;
    bool m_isFront = false;
    double m_high_steering_lim = 0.0;
    double m_low_steering_lim = 0.0;
    double m_max_engine_power = 0.0;
    double m_max_brake_power = 0.0;

    afWheelRepresentationType m_wheelRepresentationType;

};



///
/// \brief The afWorldAttributes struct
///
struct afWorldAttributes{
public:
    afWorldAttributes(){}
};

}


#endif
