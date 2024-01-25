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
#ifndef AF_ATTRIBUTES_H
#define AF_ATTRIBUTES_H
//------------------------------------------------------------------------------
#include <afEnums.h>
#include <afMath.h>
#include <afPath.h>
#include <afUtils.h>

typedef unsigned int uint;

namespace ambf {

enum class afStatusFlag{
    UNDEFINED,
    TRUE,
    FALSE,
};

///
/// \brief The afKinematicAttributes struct
///
struct afKinematicAttributes{
public:
    afKinematicAttributes(){
        m_scale = 1.0;
    }

    afTransform m_location;
    double m_scale;
};


///
/// \brief The afPrimitiveShapeAttributes struct
///
struct afPrimitiveShapeAttributes{
public:

    afPrimitiveShapeAttributes(){
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

    inline afTransform getOffset() const{return m_offset;}

public:

    double m_radius = 0;

    double m_height = 0;

    afVector3d m_dimensions;

    afVector3d m_planeNormal;

    double m_planeConstant = 0;

    afPrimitiveShapeType m_shapeType;

    afAxisType m_axisType;

    afTransform m_offset;

private:
    double m_shapeScale = 1.0;
};


///
/// \brief The afRayAttributes struct
///
struct afRayAttributes{
    // Direction rel to parent that this sensor is pointing to
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
    afIdentificationAttributes(){
        m_objectType = afType::INVALID;
    }

    string m_name;
    string m_namespace;
    afType m_objectType;
};


///
/// \brief The afCollisionAttributes struct
///
struct afCollisionAttributes{
public:
    afCollisionAttributes(){
        m_margin = 0.001;
        m_meshShapeType = afCollisionMeshShapeType::CONCAVE_MESH;
    }

    double m_margin;
    afPath m_meshFilepath;
    afCollisionMeshShapeType m_meshShapeType;
    afGeometryType m_geometryType;
    vector<afPrimitiveShapeAttributes> m_primitiveShapes;
    vector<uint> m_groups;
};


///
/// \brief The afCommunicationAttributes struct
///
struct afCommunicationAttributes{
public:
    afCommunicationAttributes(){
        m_minPublishFreq = 50;
        m_maxPublishFreq = 1000;
        m_passive = false;
    }

    uint m_minPublishFreq;
    uint m_maxPublishFreq;
    bool m_passive;
};


struct afCartesianControllerAttributes{
public:
    afCartesianControllerAttributes(){
        P_lin = 10;
        I_lin = 0;
        D_lin = 1;
        P_ang = 10;
        I_ang = 0;
        D_ang = 1;

        m_positionOutputType = afControlType::VELOCITY;
        m_orientationOutputType = afControlType::VELOCITY;
    }

    // PID Controller Gains for Linear and Angular Controller
    double P_lin, I_lin, D_lin;
    double P_ang, I_ang, D_ang;

    // The default output type is velocity
    afControlType m_positionOutputType;

    // The default output type is velocity
    afControlType m_orientationOutputType;
};


///
/// \brief The afHeirarcyAttributes struct
///
struct afHierarchyAttributes{
public:
    afHierarchyAttributes(){}

    string m_parentName;
    string m_childName;
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
    afInertialAttributes(){
        m_mass = 1.0;
        m_overrideGravity = false;
        m_estimateInertia = true;
        m_estimateInertialOffset = true;
    }

    bool m_estimateInertialOffset;

    double m_mass;
    afVector3d m_inertia;
    afVector3d m_gravity;
    bool m_overrideGravity;
    bool m_estimateInertia;
    afTransform m_inertialOffset;
    afSurfaceAttributes m_surfaceAttribs;
};


///
/// \brief The afJointControllerAttributes struct
///
struct afJointControllerAttributes{
public:
    afJointControllerAttributes(){
        m_P = 10;
        m_I = 0;
        m_D = 1;
        m_outputType = afControlType::VELOCITY;
        m_maxImpulse = 0.05;
    }

    afControlType m_outputType;

    double m_P;
    double m_I;
    double m_D;

    double m_maxImpulse;
};


struct afColorAttributes{
    afColorAttributes(){
        m_ambient.set(0.3, 0.3, 0.3);
        m_diffuse.set(0.7, 0.7, 0.7);
        m_emission.set(0.0, 0.0, 0.0);
        m_specular.set(1.0, 1.0, 1.0);

        m_alpha = 1.0;
        m_shininess = 64;
        m_useMaterial = true;
    }

    afVector3d m_ambient;
    afVector3d m_diffuse;
    afVector3d m_specular;
    afVector3d m_emission;
    double m_alpha;
    unsigned int m_shininess;
    bool m_useMaterial; // use the color attribs defined above
};


///
/// \brief The afVisualAttributes struct
///
struct afVisualAttributes{
    afVisualAttributes(){
        m_visible = true;
        m_meshRemoveDuplicates = afStatusFlag::UNDEFINED;
    }

    afPath m_meshFilepath;
    afStatusFlag m_meshRemoveDuplicates;
    afGeometryType m_geometryType;
    vector<afPrimitiveShapeAttributes> m_primitiveShapes;
    afColorAttributes m_colorAttribs;
    bool m_visible;
};


///
/// \brief The afShaderAttributes struct
///
struct afShaderAttributes{
public:
    afShaderAttributes(){
        m_shaderDefined = false;
    }

    bool m_shaderDefined;
    afPath m_vtxFilepath;
    afPath m_fragFilepath;
};


///
/// \brief The afPluginAttributes struct
///
struct afPluginAttributes{
public:
    string m_name;
    string m_filename;
    afPath m_path;

    void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_path.c_str().empty() == false){
            m_path.resolvePath(a_parentPath);
        }
    }
};

struct afSpecificationData{
    string m_type;
    string m_rawData;

    friend std::ostream& operator << (std::ostream& out, const afSpecificationData& data){
        out << "Specification Type: " << data.m_type << "\nRaw Data: \n" << data.m_rawData;
        return out;
    }
};


///
/// \brief The afBaseObjectAttributes struct
///
struct afBaseObjectAttributes{
public:
    afBaseObjectAttributes(){
        m_pathsResolved = false;
        m_namespaceResolved = false;
    }

    virtual void setSpecificationData(const afSpecificationData& a_data){
        m_specificationData = a_data;
    }

    virtual afSpecificationData getSpecificationData(){
        return m_specificationData;
    }

    // Identification attribs. Is used to determine what type of object this is (Rigid Body, Soft Body, Sensor, etc.)
    afIdentificationAttributes m_identificationAttribs;

    afCommunicationAttributes m_communicationAttribs;

    vector<afPluginAttributes> m_pluginAttribs;

    virtual void resolveRelativeNamespace(string a_parentNamespace){
        if (m_namespaceResolved == false){
            m_identificationAttribs.m_namespace = afUtils::mergeNamespace(a_parentNamespace, m_identificationAttribs.m_namespace);
            m_namespaceResolved = true;
        }
    }

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        for (int i = 0 ; i < m_pluginAttribs.size() ; i++){
            m_pluginAttribs[i].resolveRelativePathAttribs(a_parentPath);
        }
    }

    inline bool areNamespaceAttribsResolved(){ return m_namespaceResolved;}

    inline bool arePathAttribsResolved(){ return m_pathsResolved;}

    string m_identifier;

protected:
    bool m_pathsResolved;
    bool m_namespaceResolved;

private:
    afSpecificationData m_specificationData;
};


///
/// \brief The afActuatorAttributes struct
///
struct afActuatorAttributes: public afBaseObjectAttributes
{
public:
    afActuatorAttributes(){}

    afActuatorType m_actuatorType;
    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
};


///
/// \brief The afConstraintActuatorAttributes struct
///
struct afConstraintActuatorAttributes: public afActuatorAttributes{
public:
    afConstraintActuatorAttributes(){
        m_visible = false;
        m_visibleSize = 0.02;
        m_maxImpulse = 10.0;
        m_tau = 1.0;
    }

    bool m_visible;
    double m_visibleSize;
    double m_maxImpulse;
    double m_tau;
};

///
/// \brief The afImageResolution struct
///
struct afImageResolutionAttribs{
    afImageResolutionAttribs(){
        m_width = 640;
        m_height = 480;
    }
    double m_width;
    double m_height;
};


struct afNoiseModelAttribs{
    afNoiseModelAttribs(){
        m_enable = false;
        m_mean = 0.1;
        m_std_dev = 0.001;
        m_bias = 0.0;
    }
    bool m_enable;
    double m_mean;
    double m_std_dev;
    double m_bias;
};

struct afMouseControlScales{
public:
    afMouseControlScales(){
        m_pan = 0.01;
        m_rotate = 0.3;
        m_arcball = 0.03;
        m_scroll = 0.1;
    }

    double m_pan;
    double m_rotate;
    double m_scroll;
    double m_arcball;
};

///
/// \brief The afCameraAttributes struct
///
struct afCameraAttributes: public afBaseObjectAttributes
{
public:
    afCameraAttributes(){
        m_up.set(0.0, 0.0, 1.0);
        m_nearPlane = 0.1;
        m_farPlane = 10.0;
        m_fieldViewAngle = 0.7;
        m_orthographic = false;
        m_stereo = false;
        m_stereFocalLength = 0.5;
        m_stereoEyeSeparation = 0.2;
        m_monitorNumber = 0;
        m_visible = true;
        m_publishImage = false;
        m_publishDepth = false;
        m_publishImageInterval = 1;
        m_publishDepthInterval = 10;
        m_multiPass = false;
    }

    afVector3d m_lookAt;
    afVector3d m_up;
    double m_nearPlane;
    double m_farPlane;
    double m_fieldViewAngle;
    double m_orthoViewWidth;
    bool m_orthographic;
    bool m_stereo;
    double m_stereoEyeSeparation;
    double m_stereFocalLength;
    vector<string> m_controllingDeviceNames;
    uint m_monitorNumber;
    bool m_visible;
    bool m_publishImage;
    bool m_publishDepth;
    uint m_publishImageInterval;
    uint m_publishDepthInterval;
    afShaderAttributes m_preProcessShaderAttribs;
    afShaderAttributes m_depthComputeShaderAttribs;
    afMouseControlScales m_mouseControlScales;
    bool m_multiPass;

    afImageResolutionAttribs m_publishImageResolution;
    afImageResolutionAttribs m_publishDephtResolution;
    afNoiseModelAttribs m_depthNoiseAttribs;

    afHierarchyAttributes m_hierarchyAttribs;
    afKinematicAttributes m_kinematicAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
            m_preProcessShaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
            m_preProcessShaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
            m_depthComputeShaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
            m_depthComputeShaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};


///
/// \brief The afLightAttributes struct
///
struct afLightAttributes: public afBaseObjectAttributes
{
public:
    afLightAttributes(){
        m_spotExponent = 0.7;
        m_cuttoffAngle = 0.7;
    }

    double m_spotExponent;
    double m_cuttoffAngle;
    afVector3d m_direction;

    afShadowQualityType m_shadowQuality;

    afHierarchyAttributes m_hierarchyAttribs;
    afKinematicAttributes m_kinematicAttribs;
};



///
/// \brief The afJointAttributes struct
///
struct afJointAttributes: public afBaseObjectAttributes
{

public:
    afJointAttributes(){
        m_enableMotor = true;
        m_enableFeedback = false;
        m_enableLimits = false;
        m_maxMotorImpulse = 0.1;
        m_jointOffset = 0.0;
        m_childOffset = 0.0;
        m_damping = 0.0;
        m_stiffness = 0.0;
        m_equilibriumPoint = 0.0;
        m_ignoreInterCollision = true;
        m_erp = 0.1;
        m_cfm = 0.1;
    }

    struct afConeTwistLimits{
        double m_Z = 0.7;
        double m_Y = 0.7;
        double m_X = 0.7;
    };

    struct afSixDofLimits{
        afSixDofLimits(){
            for (int i = 0 ; i < 6 ; i++){
                m_lowerLimit[i] = -1.2;
                m_upperLimit[i] = 1.2;
            }
        }
        double m_lowerLimit[6];
        double m_upperLimit[6];
    };

    struct afSixDofSpringAttribs{
        afSixDofSpringAttribs(){
            for (int i = 0 ; i < 6 ; i++){
                m_stiffness[i] = 1.0;
                m_equilibriumPoint[i] = 0.0;
                m_damping[i] = 0.7;
            }
        }
        double m_stiffness[6];
        double m_equilibriumPoint[6];
        double m_damping[6];
    };

    afVector3d m_parentPivot;
    afVector3d m_childPivot;
    afVector3d m_parentAxis;
    afVector3d m_childAxis;
    afTransform m_transformInParent;
    double m_maxMotorImpulse;
    double m_lowerLimit;
    double m_upperLimit;
    afConeTwistLimits m_coneTwistLimits;
    afSixDofLimits m_sixDofLimits;
    afSixDofSpringAttribs m_sixDofSpringAttribs;
    double m_erp;
    double m_cfm;
    // Rotational offset of joint along the free joint axis
    double m_jointOffset;
    // Rotation offset of child along the free joint axis
    double m_childOffset;
    double m_damping;
    double m_stiffness;
    double m_equilibriumPoint;
    afJointType m_jointType;

    bool m_enableMotor;
    bool m_enableFeedback;
    bool m_enableLimits;
    bool m_ignoreInterCollision;

    afHierarchyAttributes m_hierarchyAttribs;
    afJointControllerAttributes m_controllerAttribs;
};


///
/// \brief The afRigidBodyAttributes struct
///
struct afRigidBodyAttributes: public afBaseObjectAttributes
{
public:
    afRigidBodyAttributes(){
        m_publishJointNames = true;
        m_publishChildrenNames = false;
        m_publishJointPositions = true;
    }

    bool m_publishChildrenNames;
    bool m_publishJointNames;
    bool m_publishJointPositions;

    afCollisionAttributes m_collisionAttribs;
    afCartesianControllerAttributes m_controllerAttribs;
    afInertialAttributes m_inertialAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afVisualAttributes m_visualAttribs;
    afShaderAttributes m_shaderAttribs;
    afSurfaceAttributes m_surfaceAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
            m_collisionAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_visualAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
            m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};

///
/// \brief The afSoftBodyAttributes struct
///
struct afSoftBodyAttributes: public afBaseObjectAttributes
{
public:
    afSoftBodyAttributes(){
        m_useMaterial = false;
        m_useBendingConstraints = false;
        m_usePoseMatching = false;
        m_useConstraintRandomization = false;

        // Initialize Defaults here
        m_kVCF = 1;
        m_kDG = 0;
        m_kLF = 0;
        m_kDP = 0;
        m_kPR = 0;
        m_kVC = 0;
        m_kDF = 0.2;
        m_kMT = 0;
        m_kCHR = 1.0;
        m_kKHR = 0.1;
        m_kSHR = 1.0;
        m_kAHR = 0.7;
        m_kSRHR_CL = 0.1;
        m_kSKHR_CL = 1;
        m_kSSHR_CL = 0.5;
        m_kSR_SPLT_CL = 0.5;
        m_kSK_SPLT_CL = 0.5;
        m_kSS_SPLT_CL = 0.5;
        m_maxVolume = 1;
        m_timeScale = 1;

        m_vIterations = 0;
        m_pIterations = 1;
        m_dIterations = 0;
        m_cIterations = 4;

        m_useMaterial = false;
        m_kLST = 1;
        m_kAST = 1;
        m_kVST = 1;

        m_flags = 0x0001;
        m_collisions = 0x0001;

        m_useBendingConstraints = false;
        m_bendingConstraint = 0;

        m_useClusters = false;
        m_clusters = 0;

        m_usePoseMatching = false;

        m_useConstraintRandomization = false;
    }

    double m_kVCF;
    double m_kDG;
    double m_kLF;
    double m_kDP;
    double m_kPR;
    double m_kVC;
    double m_kDF;
    double m_kMT;
    double m_kCHR;
    double m_kKHR;
    double m_kSHR;
    double m_kAHR;
    double m_kSRHR_CL;
    double m_kSKHR_CL;
    double m_kSSHR_CL;
    double m_kSR_SPLT_CL;
    double m_kSK_SPLT_CL;
    double m_kSS_SPLT_CL;
    double m_maxVolume;
    double m_timeScale;

    uint m_vIterations;
    uint m_pIterations;
    uint m_dIterations;
    uint m_cIterations;

    bool m_useMaterial;
    double m_kLST;
    double m_kAST;
    double m_kVST;

    int m_flags;
    int m_collisions;

    bool m_useBendingConstraints;
    uint m_bendingConstraint;

    bool m_useClusters;
    uint m_clusters;

    bool m_usePoseMatching;

    bool m_useConstraintRandomization;
    vector<uint> m_fixedNodes;

    afCollisionAttributes m_collisionAttribs;
    afCartesianControllerAttributes m_controllerAttribs;
    afInertialAttributes m_inertialAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afVisualAttributes m_visualAttribs;
    afShaderAttributes m_shaderAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
            m_collisionAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_visualAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
            m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};


///
/// \brief The afGhostObjectAttributes struct
///
struct afGhostObjectAttributes: public afBaseObjectAttributes
{
public:
    afGhostObjectAttributes(){
    }

    afCollisionAttributes m_collisionAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
    afVisualAttributes m_visualAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
            m_collisionAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_visualAttribs.m_meshFilepath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};


///
/// \brief The afWheelAttributes struct
///
struct afWheelAttributes{
public:
    afWheelAttributes(){
        m_isFront = false;
        m_steeringLimitMin = 0.0;
        m_steeringLimitMax = 0.0;
        m_enginePowerMax = 0.0;
        m_brakePowerMax = 0.0;
    }

    double m_width;
    double m_radius;
    double m_friction;

    double m_rollInfluence;
    afVector3d m_downDirection;
    afVector3d m_axelDirection;
    afVector3d m_offset;
    bool m_isFront ;
    double m_steeringLimitMin;
    double m_steeringLimitMax;
    double m_enginePowerMax;
    double m_brakePowerMax;

    afVisualAttributes m_visualAttribs;
    string m_wheelBodyName;
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
    string m_chassisBodyName;
    afPath m_wheelsVisualPath;
    vector<afWheelAttributes> m_wheelAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
            m_wheelsVisualPath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};

enum class afVolumeSpecificationType{
    INVALID,
    MULTI_IMAGE
};


///
/// \brief The afMultiImagesAttribs struct
///
struct afMultiImagesAttributes{
public:
    afMultiImagesAttributes(){
        m_count = 0;
    }
    afPath m_path;
    string m_prefix;
    string m_format;
    uint m_count;

    void resolveRelativePathAttribs(afPath a_parentPath){
        m_path.resolvePath(a_parentPath);
    }
};

///
/// \brief The afVolumeAttributes struct
///
struct afVolumeAttributes: public afBaseObjectAttributes{
   public:
    afVolumeAttributes(){
        m_dimensions.set(1.0, 1.0, 1.0);
        m_isosurfaceValue = 0.5;
        m_opticalDensity = 1.0;
        m_quality = 0.5;
    }

    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
    afVolumeSpecificationType m_specificationType;
    afMultiImagesAttributes m_multiImageAttribs;
    afShaderAttributes m_shaderAttribs;
    afColorAttributes m_colorAttribs;
    afVector3d m_dimensions;
    double m_isosurfaceValue;
    double m_opticalDensity;
    double m_quality;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
        m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
        m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
        m_multiImageAttribs.resolveRelativePathAttribs(a_parentPath);
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
    afKinematicAttributes m_kinematicAttribs;
    afHierarchyAttributes m_hierarchyAttribs;
};



///
/// \brief The afResistanceSensorAttributes struct
///

struct afRayTracerSensorAttributes: public afSensorAttributes{
public:
    afRayTracerSensorAttributes(){
        m_visible = false;
        m_visibleSize = 0.02;
        m_maxImpulse = 0.5;
        m_tau = 0.5;
        m_range = 0.5;
    }

    bool m_visible;
    double m_visibleSize;
    double m_maxImpulse;
    double m_tau;
    double m_range;

    afPath m_contourMeshFilepath;
    afSensactorSpecificationType m_specificationType;
    vector<afRayAttributes> m_raysAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        if (m_pathsResolved == false){
            m_contourMeshFilepath.resolvePath(a_parentPath);
            m_pathsResolved = true;
        }
    }
};


///
/// \brief The afResistanceSensorAttributes struct
///

struct afResistanceSensorAttributes: public afRayTracerSensorAttributes{
public:
    afResistanceSensorAttributes(){
        m_useVariableCoeff = false;
        m_staticContactFriction = 0;
        m_dynamicFriction = 0;
        m_contactArea = 0.1;
        m_staticContactDamping = 0.1;
        m_contactNormalStiffness = 0;
        m_contactNormalDamping = 0;
    }

    double m_contactNormalStiffness;
    double m_contactNormalDamping;
    double m_staticContactFriction;
    double m_staticContactDamping;
    double m_dynamicFriction;
    double m_contactArea;
    bool m_useVariableCoeff;
};


struct afFileObjectAttributes{
public:
    afFileObjectAttributes(){}
    afPath m_filePath;
};

///
/// \brief The afMultiBodyAttributes struct
///
struct afModelAttributes: public afBaseObjectAttributes, public afFileObjectAttributes{
public:
    afModelAttributes(){
        m_ignoreInterCollision = false;
        m_enableComm = true;
        m_overrideGravity = false;
    }
    ~afModelAttributes(){
//        for (int i = 0 ; i < m_sensorAttribs.size() ; i++){
//            delete m_sensorAttribs[i];
//        }
//        for (int i = 0 ; i < m_actuatorAttribs.size() ; i++){
//            delete m_actuatorAttribs[i];
//        }
    }

    afPath m_visualMeshesPath;
    afPath m_collisionMeshesPath;
    afShaderAttributes m_shaderAttribs;

    bool m_enableComm;
    bool m_overrideGravity;
    afVector3d m_gravity;

    vector <afRigidBodyAttributes> m_rigidBodyAttribs;
    vector <afSoftBodyAttributes> m_softBodyAttribs;
    vector <afGhostObjectAttributes> m_ghostObjectAttribs;
    vector <afVehicleAttributes> m_vehicleAttribs;
    vector <afVolumeAttributes> m_volumeAttribs;
    vector <afJointAttributes> m_jointAttribs;
    vector <afSensorAttributes*> m_sensorAttribs;
    vector <afActuatorAttributes*> m_actuatorAttribs;
    vector <afCameraAttributes> m_cameraAttribs;
    vector <afLightAttributes> m_lightAttribs;
    vector <afPluginAttributes> m_pluginAttribs;

    bool m_ignoreInterCollision;

    virtual bool resolveRelativeNamespace(){
        if (m_namespaceResolved == false){
            string a_parentNamespace = m_identificationAttribs.m_namespace;

            for (int i = 0 ; i < m_rigidBodyAttribs.size() ; i++){
                m_rigidBodyAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_softBodyAttribs.size() ; i++){
                m_softBodyAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_ghostObjectAttribs.size() ; i++){
                m_ghostObjectAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_jointAttribs.size() ; i++){
                m_jointAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_vehicleAttribs.size() ; i++){
                m_vehicleAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_volumeAttribs.size() ; i++){
                m_volumeAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_sensorAttribs.size() ; i++){
                m_sensorAttribs[i]->resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_actuatorAttribs.size() ; i++){
                m_actuatorAttribs[i]->resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_cameraAttribs.size() ; i++){
                m_cameraAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_lightAttribs.size() ; i++){
                m_lightAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            m_namespaceResolved = true;
        }
        return true;
    }

    virtual bool resolveRelativePathAttribs(){
        if (m_pathsResolved == false){
            afPath a_parentPath = m_filePath.parent_path();

            for (int i = 0 ; i < m_rigidBodyAttribs.size() ; i++){
                m_rigidBodyAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_softBodyAttribs.size() ; i++){
                m_softBodyAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_ghostObjectAttribs.size() ; i++){
                m_ghostObjectAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_vehicleAttribs.size() ; i++){
                m_vehicleAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_sensorAttribs.size() ; i++){
                m_sensorAttribs[i]->resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_actuatorAttribs.size() ; i++){
                m_actuatorAttribs[i]->resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_volumeAttribs.size() ; i++){
                m_volumeAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_cameraAttribs.size() ; i++){
                m_cameraAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_lightAttribs.size() ; i++){
                m_lightAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (int i = 0 ; i < m_pluginAttribs.size() ; i++){
                m_pluginAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            if (m_shaderAttribs.m_shaderDefined){
                m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
                m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);
            }

            m_pathsResolved = true;
        }
        return true;
    }
};

struct afSimulatedDeviceAttribs: public afBaseObjectAttributes, public afFileObjectAttributes{
    afSimulatedDeviceAttribs(){
        m_enableJointControl = true;
        m_sdeDefined = false;
        m_rootLinkDefined = false;
        m_overrideLocation = false;
        m_overrideController = false;
    }
    bool m_enableJointControl;
    bool m_sdeDefined = false;
    bool m_rootLinkDefined = false;
    bool m_overrideLocation;
    bool m_overrideController;

    string m_rootLinkName;


    afCartesianControllerAttributes m_controllerAttribs;
    afKinematicAttributes m_kinematicAttribs;
    afModelAttributes m_modelAttribs;

    virtual void resolveRelativePathAttribs(afPath a_parentPath){
        afBaseObjectAttributes::resolveRelativePathAttribs(a_parentPath);
        if (m_pathsResolved == false){
            m_modelAttribs.m_filePath = a_parentPath;
            m_modelAttribs.resolveRelativePathAttribs();
        }
    }
};


struct afInputDeviceAttributes: public afBaseObjectAttributes{
public:
    afInputDeviceAttributes(){
        m_workspaceScale = 1.0;
        m_visible = false;
        m_visibleSize = 1.0;
    }


    double m_deadBand;
    double m_maxForce;
    double m_maxJerk;
    double m_workspaceScale;
    double m_visibleSize;


    bool m_visible;

    string m_hardwareName;
    afCartesianControllerAttributes m_controllerAttribs;
    afTransform m_orientationOffset;

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
public:
    afSimulatedDeviceAttribs m_sdeAttribs;
    afInputDeviceAttributes m_iidAttribs;
    vector<string> m_pairedCamerasNames;
};



///
/// \brief The afSkyBoxAttributes struct
///
struct afSkyBoxAttributes{
    afPath m_leftImageFilepath;
    afPath m_frontImageFilepath;
    afPath m_rightImageFilepath;
    afPath m_backImageFilepath;
    afPath m_topImageFilepath;
    afPath m_bottomImageFilepath;
    afShaderAttributes m_shaderAttribs;

    bool resolveRelativePathAttribs(afPath a_parentPath){
        m_backImageFilepath.resolvePath(a_parentPath);
        m_bottomImageFilepath.resolvePath(a_parentPath);
        m_frontImageFilepath.resolvePath(a_parentPath);
        m_leftImageFilepath.resolvePath(a_parentPath);
        m_rightImageFilepath.resolvePath(a_parentPath);
        m_topImageFilepath.resolvePath(a_parentPath);

        m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
        m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);

        return true;
    }

    bool m_use = false;
};


///
/// \brief The afWorldAttributes struct
///
struct afWorldAttributes: public afBaseObjectAttributes, public afFileObjectAttributes{
public:
    afWorldAttributes(){
        m_maxIterations = 10;
        m_gravity.set(0, 0, -9.8);
        m_showGUI = true;
        m_identificationAttribs.m_name = "World";
    }

    vector<afLightAttributes> m_lightAttribs;
    vector<afCameraAttributes> m_cameraAttribs;
    vector<afPluginAttributes> m_pluginAttribs;

    afVector3d m_gravity;
    uint m_maxIterations;
    afShaderAttributes m_shaderAttribs;
    string m_namespace;

    bool m_showGUI;

    struct afEnclosure{
        double m_width;
        double m_height;
        double m_length;
        bool m_use = false;
    };

    struct afEnvironmentModel{
        afModelAttributes m_modelAttribs;
        bool m_use = false;
    };

    afEnclosure m_enclosure;
    afEnvironmentModel m_environmentModel;
    afSkyBoxAttributes m_skyBoxAttribs;

    virtual bool resolveRelativeNamespace(){
        if (m_namespaceResolved == false){
            string a_parentNamespace = m_identificationAttribs.m_namespace;

            for (int i = 0 ; i < m_lightAttribs.size() ; i++){
                m_lightAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }

            for (int i = 0 ; i < m_cameraAttribs.size() ; i++){
                m_cameraAttribs[i].resolveRelativeNamespace(a_parentNamespace);
            }
            m_namespaceResolved = true;
        }
        return true;
    }

    virtual bool resolveRelativePathAttribs(){
        if (m_pathsResolved == false){
            afPath a_parentPath = m_filePath.parent_path();
            m_skyBoxAttribs.resolveRelativePathAttribs(a_parentPath);

            m_shaderAttribs.m_vtxFilepath.resolvePath(a_parentPath);
            m_shaderAttribs.m_fragFilepath.resolvePath(a_parentPath);

            for (uint i = 0 ; i < m_lightAttribs.size() ; i++){
                m_lightAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (uint i = 0 ; i < m_cameraAttribs.size() ; i++){
                m_cameraAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            for (uint i = 0 ; i < m_pluginAttribs.size() ; i++){
                m_pluginAttribs[i].resolveRelativePathAttribs(a_parentPath);
            }

            m_pathsResolved = true;
        }
        return true;
    }
};


struct afLaunchAttributes: public afFileObjectAttributes{
public:
    afLaunchAttributes(){
    }
    afPath m_colorFilepath;
    afPath m_worldFilepath;
    afPath m_inputDevicesFilepath;
    vector<afPath> m_modelFilepaths;
    vector<afPluginAttributes> m_pluginAttribs;

    virtual void setSpecificationData(const afSpecificationData& a_data){
        m_specificationData = a_data;
    }

    virtual afSpecificationData getSpecificationData(){
        return m_specificationData;
    }

    virtual bool resolveRelativePathAttribs(){
        afPath a_parentPath = m_filePath.parent_path();

        m_colorFilepath.resolvePath(a_parentPath);
        m_worldFilepath.resolvePath(a_parentPath);
        m_inputDevicesFilepath.resolvePath(a_parentPath);

        for (uint i = 0 ; i < m_modelFilepaths.size() ; i++){
            m_modelFilepaths[i].resolvePath(a_parentPath);
        }

        for (uint i = 0 ; i < m_pluginAttribs.size() ; i++){
            m_pluginAttribs[i].resolveRelativePathAttribs(a_parentPath);
        }

        return true;
    }
private:
    afSpecificationData m_specificationData;
};

}


#endif
