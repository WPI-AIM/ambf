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
#ifndef AF_LIBRARY_H
#define AF_LIBRARY_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "afSoftMultiMesh.h"
#include "CBullet.h"
#include "chai3d.h"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/path.hpp>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <thread>
#include <fstream>
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#endif

//-----------------------------------------------------------------------------
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
#include "ambf_comm/Actuator.h"
#include "ambf_comm/Camera.h"
#include "ambf_comm/Light.h"
#include "ambf_comm/Object.h"
#include "ambf_comm/RigidBody.h"
#include "ambf_comm/Sensor.h"
#include "ambf_comm/Vehicle.h"
#include "ambf_comm/World.h"
#endif

// Support for Depth Image to PointCloud2
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#endif
//-----------------------------------------------------------------------------


//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class afMultiBody;
class afRigidBody;
class afSoftBody;
class afJoint;
class afWorld;
struct afRigidBodySurfaceProperties;
struct afSoftBodyConfigProperties;
struct afRenderOptions;

typedef afMultiBody* afMultiBodyPtr;
typedef afRigidBody* afRigidBodyPtr;
typedef afSoftBody* afSoftBodyPtr;
typedef afJoint* afJointPtr;
typedef afWorld* afWorldPtr;
typedef afRigidBodySurfaceProperties* afRigidBodySurfacePropertiesPtr;
typedef afSoftBodyConfigProperties* afSoftBodyConfigPropertiesPtr;
typedef std::map<std::string, afRigidBodyPtr> afRigidBodyMap;
typedef std::map<std::string, afSoftBodyPtr> afSoftBodyMap;
typedef std::map<std::string, afJointPtr> afJointMap;
typedef std::vector<afRigidBodyPtr> afRigidBodyVec;
typedef std::vector<afSoftBodyPtr> afSoftBodyVec;
typedef std::vector<afJointPtr> afJointVec;
//------------------------------------------------------------------------------
class afLight;
class afCamera;
typedef afLight* afLightPtr;
typedef afCamera* afCameraPtr;
typedef std::map<std::string, afLightPtr> afLightMap;
typedef std::map<std::string, afCameraPtr> afCameraMap;
typedef std::vector<afLightPtr> afLightVec;
typedef std::vector<afCameraPtr> afCameraVec;
//------------------------------------------------------------------------------
class afSensor;
class afResistanceSensor;
typedef afSensor* afSensorPtr;
typedef std::map<std::string, afSensorPtr> afSensorMap;
typedef std::vector<afSensorPtr> afSensorVec;
//------------------------------------------------------------------------------
class afActuator;
class afConstraintActuator;
typedef afActuator* afActuatorPtr;
typedef std::map<std::string, afActuatorPtr> afActuatorMap;
typedef std::vector<afActuatorPtr> afActuatorVec;
//------------------------------------------------------------------------------
class afMultiBody;
typedef afMultiBody* afMultiBodyPtr;
typedef std::map<std::string, afMultiBodyPtr> afMultiBodyMap;
typedef std::vector<afMultiBodyPtr> afMultiBodyVec;
//------------------------------------------------------------------------------
class afVehicle;
typedef afVehicle* afVehiclePtr;
typedef std::map<std::string, afVehiclePtr> afVehicleMap;
typedef std::vector<afVehiclePtr> afVehicleVec;
//------------------------------------------------------------------------------
class afPointCloudsHandler;
typedef cMultiPoint* cMultiPointPtr;


///
/// \brief toBTvec
/// \param cVec
/// \return
///
btVector3 toBTvec(const cVector3d &cVec);

///
/// \brief toCvec
/// \param bVec
/// \return
///
cVector3d toCvec(const btVector3 &bVec);

template <typename T>
///
/// \brief toXYZ
/// \param node
/// \return
///
T toXYZ(YAML::Node* node);


template <typename T>
///
/// \brief toRPY
/// \param node
/// \param v
/// \return
///
T toRPY(YAML::Node* node);


///
/// \brief The afUtils class
///
class afUtils{
public:

    afUtils(){}
    template<typename T1, typename T2>
    static T1 getRotBetweenVectors(const T2 &v1, const T2 &v2);

    template<typename T1, typename T2>
    static T1 convertDataType(const T2 &r);

    template <typename T>
    static std::string getNonCollidingIdx(std::string a_body_name, const T* tMap);

    static std::string removeAdjacentBackSlashes(std::string a_name);
    static std::string mergeNamespace(std::string a_namespace1, std::string a_namespace2);

    static void debugPrint(int line, std::string filename){
        std::cerr << "Line: "<< line << ", File: " << filename << std::endl;
    }
};

static std::string AF_DEPTH_COMPUTE_VTX =
        " attribute vec3 aPosition;                                  \n"
        " attribute vec3 aNormal;                                    \n"
        " attribute vec3 aTexCoord;                                  \n"
        " attribute vec4 aColor;                                     \n"
        " attribute vec3 aTangent;                                   \n"
        " attribute vec3 aBitangent;                                 \n"
        "                                                            \n"
        " varying vec4 vPosition;                                    \n"
        " varying vec3 vNormal;                                      \n"
        " varying vec3 vTexCoord;                                    \n"
        "                                                            \n"
        " void main(void)                                            \n"
        " {                                                          \n"
        "    vTexCoord = aTexCoord;                                  \n"
        "    gl_Position = vec4(aPosition.x, aPosition.y, 0.0, 1.0); \n"
        " }                                                          \n";

static std::string AF_DEPTH_COMPUTE_FRAG =
        " uniform sampler2D diffuseMap;                                                       \n"
        " varying vec3 vTexCoord;                                                             \n"
        " uniform vec3 maxWorldDimensions;                                                    \n"
        " uniform float nearPlane;                                                            \n"
        " uniform float farPlane;                                                             \n"
        "                                                                                     \n"
        " uniform mat4 invProjection;                                                         \n"
        "                                                                                     \n"
        " void main(void)                                                                     \n"
        " {                                                                                   \n"
        "     vec4 texColor = texture2D(diffuseMap, vTexCoord.xy);                            \n"
        "     float x = vTexCoord.x * 2.0 - 1.0;                                              \n"
        "     float y = vTexCoord.y * 2.0 - 1.0;                                              \n"
        "     uint b0 = texColor.x * 255.0;                                                   \n"
        "     uint b1 = texColor.y * 255.0;                                                   \n"
        "     uint b2 = texColor.z * 255.0;                                                   \n"
        "     uint b3 = texColor.w * 255.0;                                                   \n"
        "                                                                                     \n"
        "     uint depth = uint(b3 << 24 | b2 << 16 | b1 << 8 | b0 );                         \n"
        "     depth = uint(b3 << 24 | b2 << 16 | b1 << 8 | b0 );                              \n"
        "     float d = float(depth) / float(pow(2.0, 4*8));                                  \n"
        "                                                                                     \n"
        "     float z = d * 2.0 - 1.0;                                                        \n"
        "     vec4 P = vec4(x, y, z, 1.0);                                                    \n"
        "                                                                                     \n"
        "     P = invProjection * P;                                                          \n"
        "     P /= P.w;                                                                       \n"
        "                                                                                     \n"
        "     float deltaZ = farPlane - nearPlane;                                            \n"
        "     float normalized_z = (P.z - nearPlane)/deltaZ;                                  \n"
        "                                                                                     \n"
        "     // Assuming the frustrum is centered vertically and horizontally                \n"
        "     float normalized_x = (P.x + maxWorldDimensions.x / 2.0)/maxWorldDimensions.x;   \n"
        "     float normalized_y = (P.y + maxWorldDimensions.y / 2.0)/maxWorldDimensions.y;   \n"
        "                                                                                     \n"
        "     gl_FragColor = vec4(normalized_x, normalized_y, normalized_z, 1.0);             \n"
        " }                                                                                   \n";


///
/// \brief The afConfigHandler class
///
class afConfigHandler{

public:

    afConfigHandler();
    virtual ~afConfigHandler(){}
    std::string getConfigFile(std::string a_config_name);
    // The the multibody config file name at specifc index
    std::string getMultiBodyConfig(int i=0);
    // Get the filename of the color config file
    std::string getColorConfig();
    // Get the world config filename
    std::string getWorldConfig();
    // Get the config file for input devices
    std::string getInputDevicesConfig();
    // Get color's rgba values from the name of the color. Color names are defined
    // in the color config file
    std::vector<double> getColorRGBA(std::string a_color_name);
    // Load the base config file
    bool loadBaseConfig(std::string file);
    // Get the nuber of multibody config files defined in launch config file
    inline int getNumMBConfigs(){return s_multiBodyConfigFileNames.size();}

    std::string getBasePath(){return s_basePath.c_str();}

private:

    static boost::filesystem::path s_basePath;
    static std::string s_colorConfigFileName;
    static std::vector<std::string> s_multiBodyConfigFileNames;
    static std::string s_worldConfigFileName;
    static std::string s_inputDevicesConfigFileName;
    YAML::Node configNode;

protected:

    static YAML::Node s_colorsNode;

};


enum afCommType{
    ACTUATOR,
    CAMERA,
    LIGHT,
    OBJECT,
    RIGID_BODY,
    SOFT_BODY,
    SENSOR,
    VEHICLE,
    WORLD
};


class afComm{
public:
    afComm(){}

    virtual void afCreateCommInstance(afCommType type, std::string a_name, std::string a_namespace, int a_min_freq=50, int a_max_freq=2000, double time_out=0.5);

    //! This method is to retrieve all the commands for appropriate af comm instances.
    virtual void afExecuteCommand(double dt=0.001);

    //! This method applies updates Wall and Sim Time for AF State Message.
    virtual void afUpdateTimes(const double a_wall_time, const double a_sim_time);

    //! AF CHAI Env
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    std::shared_ptr<ambf_comm::Actuator> m_afActuatorCommPtr;
    std::shared_ptr<ambf_comm::Camera> m_afCameraCommPtr;
    std::shared_ptr<ambf_comm::Light> m_afLightCommPtr;
    std::shared_ptr<ambf_comm::Object> m_afObjectCommPtr;
    std::shared_ptr<ambf_comm::RigidBody> m_afRigidBodyCommPtr;
    std::shared_ptr<ambf_comm::Sensor> m_afSensorCommPtr;
    std::shared_ptr<ambf_comm::Vehicle> m_afVehicleCommPtr;
    std::shared_ptr<ambf_comm::World> m_afWorldCommPtr;
#endif

    // Flag to check if the any params have been set on the server for this comm instance
    bool m_paramsSet=false;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;

    // Get the type of communication instance
    afCommType getCommType(){return m_commType;}

private:
    afCommType m_commType;
};


///
/// \brief The afBodySurfaceProperties struct
///
struct afRigidBodySurfaceProperties{
public:
    afRigidBodySurfaceProperties(){
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
/// \brief The afSoftBodySurfaceProperties struct
///
struct afSoftBodyConfigProperties: public btSoftBody::Config{

};

///
/// \brief The Geometrytype enum
///
enum GeometryType{
    invalid= 0, mesh = 1, shape = 2, compound_shape = 3
};


enum class afControlType{
  position=0,
  force=1,
  velocity=2
};


///
/// \brief The afCartesianController struct
///
struct afCartesianController{
public:
    afCartesianController();

public:
    // Get Controller Gains
    inline double getP_lin(){return P_lin;}
    inline double getD_lin(){return D_lin;}
    inline double getP_ang(){return P_ang;}
    inline double getD_ang(){return D_ang;}

    inline void enable(bool a_enable){m_enabled = a_enable;}
    inline bool isEnabled(){return m_enabled;}

    void setLinearGains(double a_P, double a_I, double a_D);
    void setAngularGains(double a_P, double a_I, double a_D);

    // Set Controller Gains
    inline void setP_lin(double a_P) {P_lin = a_P;}
    inline void setD_lin(double a_D) {D_lin = a_D;}
    inline void setP_ang(double a_P) {P_ang = a_P;}
    inline void setD_ang(double a_D) {D_ang = a_D;}

    template <typename T1, typename T2>
    // This function computes the output torque from Rotation Data
    // The last argument ts is the time_scale and is computed at dt_fixed / dt
    T1 computeOutput(const T2 &process_val, const T2 &set_point, const double &dt, const double &ts=1);

    // Yet to be implemented
    void boundImpulse(double effort_cmd);
    // Yet to be implemented
    void boundEffort(double effort_cmd);

    // The default output type is velocity
    afControlType m_positionOutputType = afControlType::velocity;

    // The default output type is velocity
    afControlType m_orientationOutputType = afControlType::velocity;

private:
    // PID Controller Gains for Linear and Angular Controller
    double P_lin, I_lin, D_lin;
    double P_ang, I_ang, D_ang;

    // Vector storing the current position error
    btVector3 m_dPos;
    cVector3d m_dPos_cvec;
    // Matrix storing the current rotation error
    // between commanded and current rotation
    btMatrix3x3 m_dRot;
    cMatrix3d m_dRot_cvec;

    // Flag to enable disable this controller
    bool m_enabled;
};


///
/// \brief The afResistiveSurface struct
///
struct afResistiveSurface{
    bool enable = false;
    double faceResolution = 1; // Resolution along the face
    double edgeResolution = 1; // Resolution along the edges
    double range = 0.1; // Protrusion outward from the face normal
    double depth = 0.0; // Depth backward from the face normal
    double contactArea = 0.001; // Area of contact for "Stick" Friction
    double staticContactFriction = 1; // Stick friction coefficient
    double staticContactDamping = 1; // Stick friction Damping coefficient
    double dynamicFriction = 0.1; // Slip Friction coefficient
    double contactNormalStiffness = 0.0; // Stiffness along contact normal
    double contactNormalDamping = 0.0; // Damping along contact normal
    double useVariableCoeff = 0.1; // Normalize Coefficients based on depthFraction
    int sourceMesh = 0; // collision mesh

    bool generateResistiveSensors(afWorldPtr a_afWorld, afRigidBodyPtr a_afRigidBodyPtr, cBulletMultiMesh* a_multiMesh);
};

///
/// \brief The afChildJointPair struct
///
struct afChildJointPair{
    afChildJointPair(afRigidBodyPtr a_body, afJointPtr a_joint, bool a_directConnection = false){
        m_childBody = a_body;
        m_childJoint = a_joint;
        m_directConnection = false;
    }
    afRigidBodyPtr m_childBody;
    afJointPtr m_childJoint;
    // Flag for checking if the body is connected directly or not to the parent body
    bool m_directConnection = false;
};



class afBaseObject: public cBulletMultiMesh, public afComm{

public:
    afBaseObject(afWorldPtr a_afWorld);
    virtual ~afBaseObject();

    // Get the namespace of this body
    inline std::string getNamespace(){return m_namespace; }

    // Method called by afComm to apply positon, force or joint commands on the afRigidBody
    // In case the body is kinematic, only position cmds will be applied
    virtual void afExecuteCommand(double dt){}

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics(){}

    inline void setInitialPosition(cVector3d a_pos){m_initialPos = a_pos;}

    inline void setInitialRotation(cMatrix3d a_rot){m_initialRot = a_rot;}

    // Get Initial Position of this body
    inline cVector3d getInitialPosition(){return m_initialPos;}

    // Get Initial Rotation of this body
    inline cMatrix3d getInitialRotation(){return m_initialRot;}

    // This method toggles the viewing of frames of this rigid body.
    inline void toggleFrameVisibility(){m_showFrame = !m_showFrame;}

    // Get Min/Max publishing frequency for afObjectState for this body
    inline int getMinPublishFrequency(){return m_min_publish_frequency;}

    inline int getMaxPublishFrequency(){return m_max_publish_frequency;}

    // Resolve Parenting. Usuaully a mehtod to be called at a later if the object
    // to be parented to hasn't been loaded yet.
    virtual bool resolveParenting(std::string a_parent_name = ""){}

    bool isPassive(){return m_passive;}

    void setPassive(bool a_passive){m_passive = a_passive;}

    // Ptr to afWorld
    afWorldPtr m_afWorld;

    // Parent body name defined in the ADF
    std::string m_parentName;

    // Min publishing frequency
    int m_min_publish_frequency=50;

    // Max publishing frequency
    int m_max_publish_frequency=1000;

    // Enable Shader Program Associate with this object
    virtual void enableShaderProgram(){}

    // Flag for the Shader Program
    bool m_shaderProgramDefined = false;

    boost::filesystem::path m_vsFilePath;
    boost::filesystem::path m_fsFilePath;

protected:

    // The namespace for this body, this namespace affect afComm and the stored name of the body
    // in the internal body tree map.
    std::string m_namespace = "";

    // Initial location of Rigid Body
    cVector3d m_initialPos;

    // Initial rotation of Ridig Body
    cMatrix3d m_initialRot;

    // If passive, this instance will not be reported
    // for communication purposess.
    bool m_passive = false;
};


///
/// \brief The afBody class
///
class afRigidBody: public afBaseObject{

    friend class afMultiBody;
    friend class afJoint;
    friend class afWorld;

public:

    afRigidBody(afWorldPtr a_afWorld);
    virtual ~afRigidBody();

    // Method called by afComm to apply positon, force or joint commands on the afRigidBody
    // In case the body is kinematic, only position cmds will be applied
    virtual void afExecuteCommand(double dt);

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();

    // Load rigid body named by node_name from the a config file that may contain many bodies
    virtual bool loadRigidBody(std::string rb_config_file, std::string node_name, afMultiBodyPtr mB);

    // Load rigid body named by from the rb_node specification
    virtual bool loadRigidBody(YAML::Node* rb_node, std::string node_name, afMultiBodyPtr mB);

    // Add a child to the afRidigBody tree, this method will internally populate the dense body tree
    virtual void addChildJointPair(afRigidBodyPtr childBody, afJointPtr jnt);

    // Get the namespace of this body
    inline std::string getNamespace(){return m_namespace; }

    // Apply force that is specified in the world frame at a point specified in world frame
    // This force is first converted into body frame and then is used to compute
    // the resulting torque in the body frame. This torque in the body frame is
    // then converted to the world frame and is applied to the body in the world frame
    // along with the original force in the world frame
    void applyForceAtPointOnBody(const cVector3d & a_forceInWorld, const cVector3d & a_pointInWorld);

    // Vector of child joint pair. Includes joints of all the
    // connected children all the way down to the last child. Also a vector of all the
    // children (children's children ... and so on also count as children)
    std::vector<afChildJointPair> m_CJ_PairsAll;

    // This vector contains the list of only the bodies connected via active joints. A joint can
    // be set as passive in the ADF file.
    std::vector<afChildJointPair> m_CJ_PairsActive;

    // A vector of all the parent bodies (not just the immediate parents but all the way up to the root parent)
    std::vector<afRigidBodyPtr> m_parentBodies;

    // Set the angle of all the child joints
    virtual void setAngle(double &angle);

    // Set the angles based on the num elements in the argument vector
    virtual void setAngle(std::vector<double> &angle);

    // Set the config properties, this include, damping, friction restitution
    static void setConfigProperties(const afRigidBodyPtr a_body, const afRigidBodySurfacePropertiesPtr a_surfaceProps);

    // Compute the COM of the body and the tranform from mesh origin to the COM
    btVector3 computeInertialOffset(cMesh* mesh);

    // Cleanup this rigid body
    void remove();

    // function to check if this rigid body is part of the collision group
    // at a_idx
    bool checkCollisionGroupIdx(int a_idx);
    // function to check if this rigid body is part of the collision group
    // at a_idxs
    bool isCommonCollisionGroupIdx(std::vector<int> a_idx);

    // Check if the btRigidbody is child of this afBody
    bool isChild(btRigidBody* a_body);

    // Check if the btRigidBody is the direct child of this afBody
    bool isDirectChild(btRigidBody* a_body);

    // Add sensor to this body
    void addAFSensor(afSensorPtr a_sensor){m_afSensors.push_back(a_sensor);}

    // Add sensor to this body
    void addAFActuator(afActuatorPtr a_actuator){m_afActuators.push_back(a_actuator);}

    // Enable shader program if defined
    virtual void enableShaderProgram();

    // Get the sensors for this body
    inline std::vector<afSensorPtr> getAFSensors(){return m_afSensors;}

    // If the Position Controller is active, disable Position Controller from Haptic Device
    afControlType m_activeControllerType = afControlType::force;

    // Instance of Cartesian Controller
    afCartesianController m_controller;

    // Estimated Force acting on body
    btVector3 m_estimatedForce;

    // Estimated Torque acting on body
    btVector3 m_estimatedTorque;

protected:

    // Scale of mesh
    double m_scale;

    // Name of visual and collision mesh
    std::string m_mesh_name, m_collision_mesh_name;

    // cMultiMesh representation of collision mesh
    cMultiMesh m_lowResMesh;

    // Iterator of connected rigid bodies
    std::vector<afRigidBodyPtr>::const_iterator m_bodyIt;

    // Check if the linear gains have been defined
    bool m_lin_gains_defined = false;

    // Check if the linear gains have been defined
    bool m_ang_gains_defined = false;

    // Toggle publishing of joint positions
    bool m_publish_joint_positions = false;

    // Toggle publishing of children names
    bool m_publish_children_names = false;

    // Toggle publishing of joint names
    bool m_publish_joint_names = true;

    // Sensors for this Rigid Body
    afSensorVec m_afSensors;

    // Actuators for this Rigid Body
    afActuatorVec m_afActuators;

    // Function of compute body's controllers based on lumped masses
    void computeControllerGains();

    // Internal method called for population densely connected body tree
    void addParentBody(afRigidBodyPtr a_body);

    // Go higher in hierarchy to populate the body tree
    void updateUpwardHeirarchyForAddition(afRigidBodyPtr a_childbody, afJointPtr a_jnt);

    // Go lower in hierarchy to populate the body tree
    void updateDownwardHeirarchyForAddition(afRigidBodyPtr a_parentbody);

    // Go higher in the hierarch to inform of this body's removal
    void updateUpwardHeirarchyForRemoval();

    // Go lower in the hierarch to inform of this body's removal
    void updateDownwardHeirarchyForRemoval();

    // Update the children for this body in the afObject State Message
    virtual void afObjectStateSetChildrenNames();

    // Update the joints for this body in the afObject State Message
    virtual void afObjectStateSetJointNames();

    // Update the joint positions of children in afObject State Message
    virtual void afObjectSetJointPositions();

    // Update the joint velocitess of children in afObject State Message
    virtual void afObjectSetJointVelocities();

    // Update the joint efforts of children in afObject State Message
    virtual void afObjectSetJointEfforts();

    // Surface properties for damping, friction and restitution
    static afRigidBodySurfaceProperties m_surfaceProps;

    // Collision groups for this rigid body
    std::vector<int> m_collisionGroupsIdx;

    // pool of threads for solving the body's sensors in paralled
    std::vector<std::thread*> m_sensorThreads;

    // Block size. i.e. number of sensors per thread
    int m_sensorThreadBlockSize = 10;

    // This method uses the eq:
    // startIdx = threadIdx * m_sensorThreadBlockSize
    // endIdx = startIdx + m_sensorThreadBlockSize - 1
    // to compute the two indexes. The runs a loop to solve the indexes
    // in between so that we can progress in parallel
    bool updateBodySensors(int threadIdx);

    // boolean flags for each thread to progress
    std::vector<bool> m_threadUpdateFlags;

    // Global flag for all sensor threads
    bool m_keepSensorThreadsAlive = true;

private:
    // Ptr to afWorld
    afWorldPtr m_afWorld;

    // Positions of all child joints
    std::vector<float> m_joint_positions;

    // Velocities of all child joints
    std::vector<float> m_joint_velocities;

    // Efforts of all child joints
    std::vector<float> m_joint_efforts;

    // Pointer to Multi body instance that constains this body
    afMultiBodyPtr m_mBPtr;

    // Last Position Error
    btVector3 m_dpos;

    // Type of geometry this body has (MESHES OR PRIMITIVES)
    GeometryType m_visualGeometryType, m_collisionGeometryType;
};

///
/// \brief The afSoftBody class
///
class afSoftBody: public afSoftMultiMesh{

    friend class afMultiBody;

public:

    afSoftBody(afWorldPtr a_afWorld);

    // Execute the commands incomming of afObjectCmd handle
    virtual void afObjectCommandExecute(double dt){}

    // Load the softbody from filename
    virtual bool loadSoftBody(std::string sb_config_file, std::string node_name, afMultiBodyPtr mB);

    // Load the softbody from YAML Node data
    virtual bool loadSoftBody(YAML::Node* sb_node, std::string node_name, afMultiBodyPtr mB);

    // Add child a softbody
    virtual void addChildBody(afSoftBodyPtr childBody, afJointPtr jnt){}

    // Get the namespace of this body
    inline std::string getNamespace(){return m_namespace; }

    std::vector<afJointPtr> m_joints;

    std::vector<afSoftBodyPtr> m_childrenBodies;

    std::vector<afSoftBodyPtr> m_parentBodies;

    // Set angle of connected joint
    void setAngle(double &angle, double dt);

    // Set angles of connected joints
    void setAngle(std::vector<double> &angle, double dt);

    // Set softbody config properties
    static void setConfigProperties(const afSoftBodyPtr a_body, const afSoftBodyConfigPropertiesPtr a_configProps);

public:
    std::string m_namespace;

protected:

    double m_scale;

    std::string m_mesh_name;

    cMultiMesh m_lowResMesh;

    cVector3d pos;

    cMatrix3d rot;

    std::vector<afSoftBodyPtr>::const_iterator m_bodyIt;

    double K_lin, D_lin;

    double K_ang, D_ang;

    bool _lin_gains_computed = false;

    bool _ang_gains_computed = false;

    void computeGains();

protected:

    // Add a parent body
    void addParentBody(afSoftBodyPtr a_body);

    // Populate the parent tree
    void populateParentsTree(afSoftBodyPtr a_body, afJointPtr a_jnt);

    static afSoftBodyConfigProperties m_configProps;

protected:

    afWorldPtr m_afWorld;
};


///
/// \brief The afJointController class
///
class afJointController{
public:
    // Set some default values of PID
    // TODO: Maybe set PID's to 0 so the
    // user has to explicitly set them
    double P = 1000;
    double I = 0;
    double D = 50;
    double e[4] = {0, 0, 0, 0};
    double ie[4] = {0, 0, 0, 0};
    double de[4] = {0, 0, 0, 0};
    double t[4]= {0, 0, 0, 0};
    double Ie_sum = 0.0;
    size_t queue_length = 4;
    double output;
    double max_impulse;
    double max_effort;

    // Store the last effort command to compute and bound max impulse
    double m_last_cmd = 0;

    double computeOutput(double process_val, double set_point, double current_time);

    void boundImpulse(double& effort_cmd);

    void boundEffort(double& effort_cmd);

    // The default output type is velocity
    afControlType m_outputType = afControlType::velocity;
};

///
/// \brief The JointType enum
///
enum JointType{
    revolute = 0,
    prismatic = 1,
    linear_spring = 2,
    torsion_spring = 3,
    p2p = 4,
    fixed = 5
};

///
/// \brief The afJoint class
///
class afJoint{
    friend class afRigidBody;
    friend class afGripperLink;
    friend class afMultiBody;
    friend class afWorld;

public:

    afJoint(afWorldPtr a_afWorld);

    virtual ~afJoint();

    // Load joint from config filename
    virtual bool loadJoint(std::string jnt_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    // Load joint from YAML Node data
    virtual bool loadJoint(YAML::Node* jnt_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    // Apply damping to this joint
    void applyDamping(const double &dt=0.001);

    // Set open loop effort for this joint.
    void commandEffort(double &effort_cmd, bool skip_motor_check=false);

    // Set velocity for this joint
    void commandVelocity(double &velocity_cmd);

    // Set position target for this joint that is handeled by it's joint controller
    void commandPosition(double &position_cmd);

    double getShortestAngle(double current, double target);

    // Get the internal bullet constraint
    inline btTypedConstraint* getConstraint(){return m_btConstraint;}

    // Get lower joint limit
    inline double getLowerLimit(){return m_lowerLimit;}

    // Get upper joint limit
    inline double getUpperLimit(){return m_upperLimit;}

    // Get the position of this joint
    double getPosition();

    // Get the velocity of this joint
    double getVelocity();

    // Get the effort of this joint
    double getEffort();

    // Type of Joint to know what different operations to perform at the ambf level
    JointType m_jointType;

    // Method to remove the afJoint
    void remove();

    std::string getName(){return m_name;}

    bool isPassive(){return m_passive;}

    bool isFeedBackEnabled(){return m_feedbackEnabled;}

    // Hard coded for now
    cVector3d getBoundaryMax(){return cVector3d(0.5, 0.5, 0.5);}

    // Do nothing for Joint
    void setFrameSize(double size){}

protected:

    std::string m_name;
    std::string m_parentName, m_childName;
    std::string m_jointName;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    double m_jointDamping;
    double m_maxEffort;
    bool m_enableActuator;
    double m_lowerLimit, m_upperLimit;
    double m_jointOffset;

    // Store parent and child afRigidBody to prevent lookups.
    afRigidBodyPtr m_afParentBody;
    afRigidBodyPtr m_afChildBody;

    void printVec(std::string name, btVector3* v);
    afWorldPtr m_afWorld;

    // Is this a passive joint or not (REDUNDANT JOINT). If passive, this joint will not be reported
    // for communication purposess.
    bool m_passive = false;

    // Wrench Feedback information from the joint
    bool m_feedbackEnabled = false;

    // Bullet Joint Feedback Ptr
    btJointFeedback  *m_feedback = nullptr;

protected:

    btTypedConstraint *m_btConstraint = nullptr;

    // The estimated Effort for this joint if its a single DOF joint.
    double m_estimatedEffort = 0.0;

private:
    // Add these two pointers for faster access to constraint internals
    // rather than having to cast the m_btConstraint ptr in high speed
    // control loops
    btHingeConstraint* m_hinge;
    btSliderConstraint* m_slider;
    btGeneric6DofSpringConstraint* m_spring;
    btPoint2PointConstraint* m_p2p;
    afMultiBodyPtr m_mB;
    afJointController m_controller;

    // Vector of joint positions containing the last n joint values.
    int m_jpSize = 2;
    std::vector<double> m_posArray;
    std::vector<double> m_dtArray;

};


enum afActuatorType{
    constraint = 0
};


///
/// \brief The afActuator class
///
class afActuator: public afBaseObject{
public:
    afActuator(afWorldPtr a_afWorld);

    // Load actuator from filename
    virtual bool loadActuator(std::string actuator_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "")=0;

    // Load actuator form YAML node data
    virtual bool loadActuator(YAML::Node* actuator_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "")=0;

    virtual void actuate(){}

    virtual void deactuate(){}

    // Parent Body for this sensor
    afRigidBodyPtr m_parentBody;

    bool m_showActuator;

    virtual void afExecuteCommand(double dt){}

    virtual void updatePositionFromDynamics(){}

protected:
    bool m_actuate = false;

};



///
/// \brief The afConstraintActuator class. First type of actuator class. Ultimately we can add things like drills, cutters, magnets
/// etc all as actuators.
///
class afConstraintActuator: public afActuator{
public:
    afConstraintActuator(afWorldPtr a_afWorld);

    // Load actuator from filename
    virtual bool loadActuator(std::string actuator_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping = "");

    // Load actuator form YAML node data
    virtual bool loadActuator(YAML::Node* actuator_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping = "");

    // The actuate methods will all result in the same thing. I.e. constraint a desired body or soft-body (face) to the parent body,
    // on which this actuator is mounted. The methods will use the current position of bodies or soft-bodyies to compute the parent
    // and child offsets for forming the constraint
    virtual void actuate(std::string a_rigid_body_name);

    virtual void actuate(afRigidBodyPtr a_rigidBody);

    virtual void actuate(std::string a_softbody_name, int a_face_index);

    virtual void actuate(afSoftBodyPtr a_softBody, int a_face_index);

    // In these actuate methods, explicit body offsets are provided.

    virtual void actuate(std::string a_rigid_body_name, cVector3d a_bodyOffset);

    virtual void actuate(afRigidBodyPtr a_rigidBody, cVector3d a_bodyOffset);

    virtual void actuate(std::string a_softbody_name, int a_face_index, cVector3d a_bodyOffset);

    virtual void actuate(afSoftBodyPtr a_softBody, int a_face_index, cVector3d a_bodyOffset);

    // Remove the constraint
    virtual void deactuate();

    virtual void afExecuteCommand(double dt);

    virtual void updatePositionFromDynamics();

protected:

    double m_maxImpulse = 3.0;
    double m_tau = 0.001;
    btPoint2PointConstraint* m_constraint = 0;
    // Transform of actuator w.r.t. parent body
    cTransform m_T_aINp;


private:
    afRigidBodyPtr m_childBody = 0;
    afSensorPtr m_childSotBody = 0;
    int m_softBodyFaceIdx = -1;
    // Child offset w.r.t to actuator
    cVector3d m_P_cINp;

    bool m_active = false;
};

//-----------------------------------------------------------------------------
enum afSensorType{
    proximity=0, range=1, resistance=2
};

///
/// \brief The afSensor class
///
class afSensor: public afBaseObject{
    friend class afRigidBody;
public:
    afSensor(afWorldPtr a_afWorld);

    // Load sensor from filename
    virtual bool loadSensor(std::string sensor_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "")=0;

    // Load sensor form YAML node data
    virtual bool loadSensor(YAML::Node* sensor_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "")=0;


    // Toggle the debug display of the sensor
    inline void toggleSensorVisibility() {m_showSensor = !m_showSensor; }

    // Get the body this sensor is a child of
    inline afRigidBodyPtr getParentBody(){return m_parentBody;}

    // Parent Body for this sensor
    afRigidBodyPtr m_parentBody;

    // The type this sensor?
    afSensorType m_sensorType;

    // Toggle visibility of this sensor
    bool m_showSensor = true;

    virtual void afExecuteCommand(double dt);

    // Upate the sensor, usually called at each dynamic tick update of the physics engine
    virtual void updatePositionFromDynamics();

    // Go through all the parents to get the absolute world transform
    cTransform getWorldTransform(){

    }
};

// Declare enum to find out later what type of body we sensed
enum class afBodyType{
    RIGID_BODY=0, SOFT_BODY=1};

struct afRayTracerResult{

    // Direction rel to parent that this sensor is looking at
    cVector3d m_direction;

    // Range of this sensor, i.e. how far can it sense
    double m_range;

    // Based on the location, direciton and range, calculate
    // start and end points for the ray tracing in Local Frame
    cVector3d m_rayFromLocal;
    cVector3d m_rayToLocal;

    // The rigid body that this proximity sensor is sensing
    btRigidBody* m_sensedBTRigidBody;

    // This is the AF Rigid body sensed by this sensor
    afRigidBodyPtr m_sensedAFRigidBody = 0;

    // The soft body that this proximity sensor is sensing
    btSoftBody* m_sensedBTSoftBody;

    // This is the AF Soft body sensed by this sensor
    afSoftBodyPtr m_sensedAFSoftBody = 0;

    // The internal index of the face belonging to the sensed soft body
    int m_sensedSoftBodyFaceIdx = -1;

    // The internal index of the node belonging to the sensed soft body
    int m_sensedSoftBodyNodeIdx = -1;

    // The node ptr to the sensed soft body's face
    btSoftBody::Face* m_sensedSoftBodyFace;

    // The node ptr to the sensed soft body's node
    btSoftBody::Node* m_sensedSoftBodyNode;

    // Boolean for sensor sensing something
    bool m_triggered;

    // Location of sensed point in World Frame. This is along of the sensor direction
    cVector3d m_sensedLocationWorld;

    // Visual markers to show the hit point and the sensor start and end points
    cMesh *m_hitSphereMesh, *m_fromSphereMesh, *m_toSphereMesh;

    // Visual Mesh for Normal at Contact Point
    cMesh *m_hitNormalMesh;

    // Internal constraint for rigid body gripping
    btPoint2PointConstraint* _p2p;

    // Depth fraction is the penetration normalized depth of the sensor
    double m_depthFraction = 0;

    // Normal at Contact Point
    cVector3d m_contactNormal;

    // Type of sensed body, could be a rigid body or a soft body
    afBodyType m_sensedBodyType;
};


class afRayTracerSensor: public afSensor{

    friend class afProximitySensor;
    friend class afResistanceSensor;

public:
    // Constructor
    afRayTracerSensor(afWorldPtr a_afWorld);

    // Load the sensor from ambf format
    virtual bool loadSensor(std::string sensor_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    // Load the sensor from ambf format
    virtual bool loadSensor(YAML::Node* sensor_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    // Update sensor is called on each update of positions of RBs and SBs
    virtual void updatePositionFromDynamics();

    // Check if the sensor sensed something. Depending on what type of sensor this is
    inline bool isTriggered(int idx){return m_sensedResults[idx].m_triggered;}

    // Get the type of sensed body
    inline afBodyType getSensedBodyType(int idx){return m_sensedResults[idx].m_sensedBodyType;}

    // Return the sensed BT RigidBody's Ptr
    inline btRigidBody* getSensedBTRigidBody(int idx){return m_sensedResults[idx].m_sensedBTRigidBody;}

    // Get the sensed AF Rigid Body's Ptr
    inline afRigidBodyPtr getSensedAFRigidBody(int idx){return m_sensedResults[idx].m_sensedAFRigidBody;}

    // Get the sensed BT SoftBody's Ptr
    inline btSoftBody* getSensedBTSoftBody(int idx){return m_sensedResults[idx].m_sensedBTSoftBody;}

    // Get the sensed AF Soft Body's Ptr
    inline afSoftBodyPtr getSensedAFSoftBody(int idx){return m_sensedResults[idx].m_sensedAFSoftBody;}

    // Get the sensed SoftBody's Face
    inline btSoftBody::Face* getSensedSoftBodyFace(int idx){return m_sensedResults[idx].m_sensedSoftBodyFace;}

    // Get the sensed SofyBody's Closest node the sensed point Node if any
    inline btSoftBody::Node* getSensedSoftBodyNode(int idx){return m_sensedResults[idx].m_sensedSoftBodyNode;}

    // Get the sensed SofyBody's Face's index if any
    inline int getSensedSoftBodyFaceIdx(int idx){return m_sensedResults[idx].m_sensedSoftBodyFaceIdx;}

    // Get the sensed SofyBody's Node's Idx
    inline int getSensedSoftBodyNodeIdx(int idx){return m_sensedResults[idx].m_sensedSoftBodyNodeIdx;}

    // Get the sensed point in world frame
    inline cVector3d getSensedPoint(int idx){return m_sensedResults[idx].m_sensedLocationWorld;}

    inline void setRayFromInLocal(const cVector3d& a_rayFrom, int idx){m_sensedResults[idx].m_rayFromLocal = a_rayFrom;}

    inline void setRayToInLocal(const cVector3d& a_rayTo, int idx){m_sensedResults[idx].m_rayToLocal = a_rayTo;}

    inline void setDirection(const cVector3d& a_direction, int idx){m_sensedResults[idx].m_direction = a_direction;}

    inline void setRange(const double& a_range, int idx){m_sensedResults[idx].m_range = a_range;}

    inline void setSensorVisibilityRadius(const double& a_sensorVisibilityRadius){m_visibilitySphereRadius = a_sensorVisibilityRadius;}

    inline double getCount(){return m_count;}

    void enableVisualization();

    virtual void afExecuteCommand(double dt);


    double m_range;

protected:
    // The number of ray tracing elements belonging to this sensor
    int m_count;

    std::vector<afRayTracerResult> m_sensedResults;

    // Size of spheres for the sensor visualization
    double m_visibilitySphereRadius;
};


///
/// \brief The afProximitySensor class
///
class afProximitySensor: public afRayTracerSensor{
public:
    // Constructor
    afProximitySensor(afWorldPtr a_afWorld);
};


struct afResistanceContacts{
    // Contact point in Body A frame
    cVector3d m_bodyAContactPointLocal;

    // Contact point in body B frame
    cVector3d m_bodyBContactPointLocal;

    // Error tangential to the contact (or sensor) normal
    cVector3d m_tangentialError;

    // Pre value of tangential contact error
    cVector3d m_tangentialErrorLast;

    // Variable to store if the slick contacts are still valid (with in the contact area tolerance)
    bool m_contactPointsValid;

    // First time the sensor made contact with another object. Computed everytime a new contact happens
    bool m_firstTrigger = true;
};

//-----------------------------------------------------------------------------

class afResistanceSensor: public afRayTracerSensor{
public:
    afResistanceSensor(afWorldPtr a_afWorld);
    virtual bool loadSensor(YAML::Node *sensor_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx="");
    virtual void updatePositionFromDynamics();

public:
    inline void setStaticContactFriction(const double& a_staticFriction){m_staticContactFriction = a_staticFriction;}

    inline void setStaticContactDamping(const double& a_staticDamping){m_staticContactDamping = a_staticDamping;}

    inline void setDynamicFriction(const double& a_dynamicFriction){m_dynamicFriction = a_dynamicFriction;}

    inline void setContactNormalStiffness(const double& a_contactStiffness){m_contactNormalStiffness = a_contactStiffness;}

    inline void setContactNormalDamping(const double& a_contactDamping){m_contactNormalDamping = a_contactDamping;}

    inline void useVariableCoeff(const bool & a_enable){m_useVariableCoeff = a_enable;}

    inline void setContactArea(const double& a_contactArea){m_contactArea = a_contactArea;}

private:
    cVector3d m_lastContactPosInWorld;
    cVector3d m_curContactPosInWorld;

    // Contact Stiffness. The force along the direction of the sensor to
    // emulate contact stiffness. Default to 0
    double m_contactNormalStiffness;

    // Contact Damping. Soften the stiffness of the contact by damping. Default to 0
    double m_contactNormalDamping;

    // The static "stick" friction
    double m_staticContactFriction;

    // The damping along the contact point
    double m_staticContactDamping;

    // Friction due to slide, or kinetic friction
    double m_dynamicFriction;

    // Tolerance to slide of the contact points between two bodies
    // tangential to the direction of the sensor direction
    double m_contactArea;

    // Use Variable Coeffecients based on the Depth Fraction
    bool m_useVariableCoeff = false;

    std::vector<afResistanceContacts> m_resistanceContacts;
};


///
/// \brief The afDepthPointCloud class
///
class afDepthPointCloud{
    friend class afCamera;
public:
    int setup(int a_width, int a_height, int a_numFields);
    ~afDepthPointCloud();

    inline int getWidth(){return m_width;}
    inline int getHeight(){return m_height;}
    inline int getNumFields(){return m_numFields;}

protected:
    float *m_data = nullptr;
    int m_width=0;
    int m_height=0;
    int m_numFields=0;
};


///
/// \brief The afCamera class
///
class afCamera: public afBaseObject{
public:

    afCamera(afWorld* a_afWorld);
    ~afCamera();

    virtual void render(afRenderOptions &options);

    // Define the virtual method for camera
    virtual void afExecuteCommand(double dt);

    // Define the virtual method for camera
    virtual void updatePositionFromDynamics();

    // Initialize
    bool init();

    void updateLabels(afRenderOptions &options);

    // Create the default camera. Implemented in case not additional cameras
    // are define in the AMBF config file
    bool createDefaultCamera();

    cCamera* getInternalCamera(){return m_camera;}

    // Load camera from YAML Node data
    bool loadCamera(YAML::Node* camera_node, std::string camera_name, afWorldPtr a_world);

    // Since we changed the order of ADF loading such that cameras are loaded before
    // bodies etc. we wouldn't be able to find a body defined as a parent in the
    // camera data-block in the ADF file. Thus after loading the bodies, this method
    // should be called to find the parent.
    virtual bool resolveParenting(std::string a_parent_name = "");

    // Method similar to cCamera but providing a layer of abstraction
    // So that we can set camera transform internally and set the
    // transform of the afRigidBody surrounding the camera the same
    bool setView(const cVector3d& a_localPosition,
                     const cVector3d& a_localLookAt,
                     const cVector3d& a_localUp);

    // The following 5 methods override the cCamera internals as we don't
    // want the cameras base class "cGenericObject" to be representing the
    // kinematics. Instead we want the afRigidBody to do so.
    // This method returns the camera "look at" position vector for this camera.
    inline cVector3d getLookVector()  const { return (-m_localRot.getCol0()); }

    // This method returns the "up" vector for this camera.
    inline cVector3d getUpVector()    const { return (m_localRot.getCol2()); }

    // This method returns the "right direction" vector for this camera.
    inline cVector3d getRightVector() const { return (m_localRot.getCol1()); }

    // This method returns the field view angle in Radians.
    inline double getFieldViewAngle() const { return m_camera->getFieldViewAngleRad(); }

    // Get interval between the scene update and publishing of an image
    inline uint getImagePublishInterval(){return m_imagePublishInterval;}

    // Get interval between the scene update and publishing of the depth
    inline uint getDepthPublishInterval(){return m_depthPublishInterval;}

    // Set interval between the scene update and publishing of an image
    void setImagePublishInterval(uint a_interval);

    // Set interval between the scene update and publishing of the depth
    void setDepthPublishInterval(uint a_interval);

    // This method enables or disables output image mirroring vertically.
    inline void setMirrorVertical(bool a_enabled){m_camera->setMirrorVertical(a_enabled);}

    // This method renders the the camera view in OpenGL
    inline void renderView(const int a_windowWidth, const int a_windowHeight){
        m_camera->renderView(a_windowWidth, a_windowHeight);
    }

    void renderFrameBuffer();

    void computeDepthOnGPU();

    // Publish Image as a ROS Topic
    void publishImage();

    // Publish Depth as a ROS Topic
    void computeDepthOnCPU();

    // Publish Depth as Point Cloud
    void publishDepthPointCloud();

    // Front plane scene graph which can be used to attach widgets.
    inline cWorld* getFrontLayer(){
        return m_camera->m_frontLayer;
    }

    // Front plane scene graph which can be used to attach widgets.
    inline cWorld* getBackLayer(){
        return m_camera->m_backLayer;
    }

    // Is this camera orthographic or not
    inline bool isOrthographic(){return m_orthographic;}

    // Override the get Global Position method for camera
    cVector3d getGlobalPos();

    // Get the Target or the lookAt point
    cVector3d getTargetPos();

    // Set the Camera Target or LookAt position
    void setTargetPos(cVector3d a_pos);

    // Show a visual marker representing the position of CameraTaregetPosition
    void showTargetPos(bool a_show);

    cMesh* m_targetVisualMarker;

public:
    bool m_cam_pressed;
    GLFWwindow* m_window;

    static GLFWwindow* s_mainWindow;
    static GLFWmonitor** s_monitors;
    GLFWmonitor* m_monitor;
    static int s_numMonitors;

    cStereoMode m_stereMode;

    // Labels
    cLabel* m_graphicsDynamicsFreqLabel;
    cLabel* m_wallSimTimeLabel;
    cLabel* m_devicesModesLabel;
    cLabel* m_deviceButtonLabel;
    cLabel* m_controllingDeviceLabel;
    std::vector<cLabel*> m_devHapticFreqLabels;

    // Position of mouse's x,y and scrolls cur and last coordinates for contextual window
    double mouse_x[2], mouse_y[2], mouse_scroll[2];
    bool mouse_l_clicked = false, mouse_r_clicked= false, mouse_scroll_clicked = false;
    bool mouse_r_btn_rising_edge = false, mouse_l_btn_rising_edge = false;


    cMatrix3d camRot, camRotPre;

    // Window parameters
    int m_width, m_height;
    int m_win_x, m_win_y;

    std::vector<std::string> m_controllingDevNames;

    // Frame Buffer to write to OpenCV Transport stream
    cFrameBuffer* m_frameBuffer = nullptr;

    // Image to Convert the FrameBuffer into an image
    cImagePtr m_bufferColorImage;

    // Image to Convert the FrameBuffer into an depth image
    cImagePtr m_bufferDepthImage;

    /// IMPLEMENTATION FOR DEPTH IMAGE TO POINTCLOUD ///
    // A separate buffer to render and convert depth image to Cam XYZ
    cFrameBuffer* m_depthBuffer = nullptr;

    // A separate world attached to the depht Buffer
    cWorld* m_dephtWorld = nullptr;

    // A separate quad added as the only child to the depth world
    cMesh* m_depthMesh = nullptr;

    cImagePtr m_depthBufferColorImage;

    bool m_useGPUForDepthComputation = true;

protected:
    cVector3d m_pos, m_posClutched;
    cMatrix3d m_rot, m_rotClutched;

    // This is the position that the camera is supposed to be looking at
    // This is also the point along which the orbital/arcball rotation
    // of the camera takes place.
    cVector3d m_targetPos;

    static int s_numWindows;
    static int s_cameraIdx;
    static int s_windowIdx;

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
protected:

    // Open CV Image Matrix
    cv::Mat m_imageMatrix;

    // Image Transport CV Bridge Node
    static image_transport::ImageTransport *s_imageTransport;

    // Image Transport Publisher
    image_transport::Publisher m_imagePublisher;

    // Image Transport ROS Node
    static ros::NodeHandle* s_rosNode;

    // Flag to check if to check if ROS Node and CV ROS Node is initialized
    static bool s_imageTransportInitialized;
#endif

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_comm::ProjectionType m_projectionType;
    ambf_comm::ViewMode m_viewMode;
#endif

    // Depth to Point Cloud Impl
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    sensor_msgs::PointCloud2::Ptr m_depthPointCloudMsg;
    sensor_msgs::PointCloud2Modifier* m_depthPointCloudModifier = nullptr;
    ros::Publisher m_depthPointCloudPub;
#endif

private:
    afWorldPtr m_afWorld;

    // Hold the cCamera private and shield it's kinematics represented
    // by cGenericObject from the world since we want afRidigBody to
    // represent the kinematics instead
    cCamera* m_camera;

    // Flag to enable disable publishing of color image as a ROS topic
    bool m_publishImage = false;

    // Flag to enable disable publishing of depth image as a ROS topic
    bool m_publishDepth = false;

    cVector3d m_camPos;
    cVector3d m_camLookAt;
    cVector3d m_camUp;

    // Is this camera orthographic or not.
    bool m_orthographic = false;

    afDepthPointCloud m_depthPC;

    // The interval used to publish the image. A val of 1 means that publish every scene update
    // and a value of 10 means, publish every 10th scene udpate
    uint m_imagePublishInterval = 1;

    // The interval used to publish the depth. A val of 1 means that publish every scene update
    // and a value of 10 means, publish every 10th scene udpate
    uint m_depthPublishInterval = 10;

    // Incremented every scene update (render method call)
    uint m_sceneUpdateCounter = 0;
};

//-----------------------------------------------------------------------------

///
/// \brief The ShadowQuality enum
///
enum ShadowQuality{
    no_shadow=0,
    very_low=1,
    low=2,
    medium=3,
    high=4,
    very_high=5
};


///
/// \brief The afLight struct
///
class afLight: public afBaseObject{
public:
    afLight(afWorld* a_afWorld);

    // Load light from YAML Node data
    bool loadLight(YAML::Node* light_node, std::string light_name, afWorldPtr a_world);

    // Default light incase no lights are defined in the AMBF Config file
    bool createDefaultLight();

    virtual bool resolveParenting(std::string a_parent_name = "");

    virtual void afExecuteCommand(double dt);

    virtual void updatePositionFromDynamics();

    // Set direction of this light
    void setDir(const cVector3d& a_direction);

protected:
    cSpotLight* m_spotLight;

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_comm::LightType m_lightType;
#endif

private:
    afWorldPtr m_afWorld;
};


///
/// \brief The afMultiPointUnit struct
///
struct afMultiPointUnit{

    // The parent name for this struct
    std::string m_parentName;

    cMultiPointPtr m_mpPtr;

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_comm::PointCloudHandlerPtr m_pchPtr;
#endif
};


///
/// \brief The afPointCloud class
///
class afPointCloudsHandler: public afBaseObject{

public:
    afPointCloudsHandler(afWorldPtr a_afWorld);

    virtual void afExecuteCommand(double dt){}

    virtual void updatePositionFromDynamics();

    std::map<std::string, afMultiPointUnit> m_pcMap;

};


struct afRenderOptions{
    bool m_mirroredDisplay = false;
    bool m_updateLabels = true;
    bool m_windowClosed = false;
    std::string m_IIDModeStr = "";
    std::string m_IIDBtnActionStr = "";
};


//-----------------------------------------------------------------------------

///
/// \brief The afWorld class
///
class afWorld: public cBulletWorld, public afConfigHandler, public afComm{

    friend class afMultiBody;

public:

    afWorld(std::string a_global_namespace);

    virtual ~afWorld();

    virtual bool loadWorld(std::string a_world_config = "", bool showGUI=true);

    virtual void render(afRenderOptions &options);

    // Template method to add various types of objects
    template<typename T, typename TMap>
    bool addObject(T a_obj, std::string a_name, TMap* a_map);

     // Template method to get a specific type of object
    template <typename T, typename TMap>
    T getObject(std::string a_name, TMap* a_map, bool suppress_warning);

     // Template method to get all objects of specific type
    template <typename Tvec, typename TMap>
    Tvec getObjects(TMap* tMap);

    bool createDefaultWorld();

    double getEnclosureLength();

    double getEnclosureWidth();

    double getEnclosureHeight();

    void getEnclosureExtents(double &length, double &width, double &height);

    inline void pausePhysics(bool pause){m_pausePhx = pause;}

    bool isPhysicsPaused(){return m_pausePhx;}

    // Used when the Physics is paused
    inline void stepPhysicsManually(int a_steps){m_manualStepPhx += a_steps;}

    int getManualSteps(){return m_manualStepPhx;}

    void resetCameras();

    void resetDynamicBodies(bool reset_time=false);

    int getMaxIterations(){return m_maxIterations;}

    double computeStepSize(bool adjust_intetration_steps = false);

    void estimateBodyWrenches();

    //! This method updates the simulation over a time interval.
    virtual void updateDynamics(double a_interval, double a_wallClock=0, double a_loopFreq = 0, int a_numDevices = 0);

    //! This method updates the position and orientation from Bullet models to CHAI3D models.
    virtual void updatePositionFromDynamics(void);

    bool addAFLight(afLightPtr a_rb, std::string a_name);

    bool addAFCamera(afCameraPtr a_rb, std::string a_name);

    bool addAFRigidBody(afRigidBodyPtr a_rb, std::string a_name);

    bool addAFSoftBody(afSoftBodyPtr a_sb, std::string a_name);

    bool addAFJoint(afJointPtr a_jnt, std::string a_name);

    bool addAFActuator(afActuatorPtr a_actuator, std::string a_name);

    bool addAFSensor(afSensorPtr a_sensor, std::string a_name);

    bool addAFMultiBody(afMultiBodyPtr a_multiBody, std::string a_name);

    bool addAFVehicle(afVehiclePtr a_vehicle, std::string a_name);

    // This method build the collision graph based on the collision group numbers
    // defined in the bodies
    void buildCollisionGroups();


    afLightPtr getAFLight(std::string a_name, bool suppress_warning=false);

    afCameraPtr getAFCamera(std::string a_name, bool suppress_warning=false);

    afRigidBodyPtr getAFRigidBody(std::string a_name, bool suppress_warning=false);

    afRigidBodyPtr getAFRigidBody(btRigidBody* a_body, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBody(std::string a_name, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBody(btSoftBody* a_body, bool suppress_warning=false);

    afJointPtr getAFJoint(std::string a_name);

    afActuatorPtr getAFActuator(std::string a_name);

    afSensorPtr getAFSensor(std::string a_name);

    afMultiBodyPtr getAFMultiBody(std::string a_name, bool suppress_warning=false);

    afVehiclePtr getAFVehicle(std::string a_name, bool suppress_warning=false);


    inline afLightMap* getAFLightMap(){return &m_afLightMap;}

    inline afCameraMap* getAFCameraMap(){return &m_afCameraMap;}

    inline afRigidBodyMap* getAFRigidBodyMap(){return &m_afRigidBodyMap;}

    inline afSoftBodyMap* getAFSoftBodyMap(){return &m_afSoftBodyMap;}

    inline afJointMap* getAFJointMap(){return &m_afJointMap;}

    inline afActuatorMap* getAFActuatorMap(){return &m_afActuatorMap;}

    inline afSensorMap* getAFSensorMap(){return &m_afSensorMap;}

    inline afMultiBodyMap* getAFMultiBodyMap(){return &m_afMultiBodyMap;}

    inline afVehicleMap* getAFVehicleMap(){return &m_afVehicleMap;}


    afLightVec  getAFLighs();

    afCameraVec getAFCameras();

    afRigidBodyVec getAFRigidBodies();

    afSoftBodyVec getAFSoftBodies();

    afJointVec getAFJoints();

    afActuatorVec getAFActuators();

    afSensorVec getAFSensors();

    afMultiBodyVec getAFMultiBodies();

    afVehicleVec getAFVehicles();


    std::string resolveGlobalNamespace(std::string a_name);

    std::string getNamespace(){return m_namespace;}

    std::string setWorldNamespace(std::string a_namespace){m_namespace = a_namespace;}

    std::string getGlobalNamespace(){return m_global_namespace;}

    void setGlobalNamespace(std::string a_namespace);

    virtual void afExecuteCommand(double dt);

    // The collision groups are sorted by integer indices. A group is an array of
    // rigid bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // the a body can be a part of multiple groups
    std::map<int, std::vector<afRigidBodyPtr> > m_collisionGroups;

    // Get the root parent of a body, if null is provided, returns the parent body
    // with most children
    afRigidBodyPtr getRootAFRigidBody(afRigidBodyPtr a_bodyPtr = NULL);

    // Load and ADF constraint rigid bodies, joints, sensors, soft-bodies
    bool loadADF(std::string a_adf_filepath, bool enable_comm);

    bool loadADF(int i, bool enable_comm);

    void loadAllADFs(bool enable_com);

    bool pickBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    bool movePickedBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    void removePickingConstraint();

    virtual void enableShaderProgram();

    void loadSkyBox();

    GLFWwindow* m_mainWindow;

    //data for picking objects
    class btRigidBody* m_pickedBulletRigidBody=0;

    afRigidBodyPtr m_pickedAFRigidBody=0;

    cMaterialPtr m_pickedAFRigidBodyColor; // Original color of picked body for reseting later

    cMaterial m_pickColor; // The color to be applied to the picked body

    class btSoftBody* m_pickedSoftBody=0; // Picked SoftBody

    class btSoftBody::Node* m_pickedNode=0; // Picked SoftBody Node

    int m_pickedNodeIdx = -1; // Picked SoftBody Node

    double m_pickedNodeMass = 0;

    cVector3d m_pickedNodeGoal;

    class btTypedConstraint* m_pickedConstraint = nullptr;

    int m_savedState;

    cVector3d m_oldPickingPos;

    cVector3d m_hitPos;

    double m_oldPickingDist;

    cVector3d m_pickedOffset;

    cMesh* m_pickSphere;

    cPrecisionClock g_wallClock;

    bool m_shaderProgramDefined = false;

    // Vertex Shader Filepath
    boost::filesystem::path m_vsFilePath;

    // Fragment Shader Filepath
    boost::filesystem::path m_fsFilePath;
    //    cMesh* m_pickDragVector;

    // Is skybox defined?
    bool m_skyBoxDefined = false;

    // Skybox Mesh
    cMesh* m_skyBoxMesh = 0;

    // L, R, T, B, F and B images for the skybox
    boost::filesystem::path m_skyBoxLeft;

    boost::filesystem::path m_skyBoxRight;

    boost::filesystem::path m_skyBoxTop;

    boost::filesystem::path m_skyBoxBottom;

    boost::filesystem::path m_skyBoxFront;

    boost::filesystem::path m_skyBoxBack;

    bool m_skyBox_shaderProgramDefined = false;

    boost::filesystem::path m_skyBox_vsFilePath;

    boost::filesystem::path m_skyBox_fsFilePath;

    boost::filesystem::path m_world_config_path;

    // a frequency counter to measure the simulation graphic rate
    cFrequencyCounter m_freqCounterGraphics;

    // a frequency counter to measure the simulation haptic rate
    cFrequencyCounter m_freqCounterHaptics;


protected:

    afLightMap m_afLightMap;

    afCameraMap m_afCameraMap;

    afRigidBodyMap m_afRigidBodyMap;

    afSoftBodyMap m_afSoftBodyMap;

    afJointMap m_afJointMap;

    afActuatorMap m_afActuatorMap;

    afSensorMap m_afSensorMap;

    afMultiBodyMap m_afMultiBodyMap;

    afVehicleMap m_afVehicleMap;

    std::string m_namespace;

    // If this string is set, it will force itself to preeced all nampespaces
    // regardless of whether any namespace starts with a '/' or not.
    std::string m_global_namespace;

private:

    static double m_encl_length;

    static double m_encl_width;

    static double m_encl_height;

    static int m_maxIterations;

    cPositionalLight* m_light;
    // Global flag to pause simulation
    bool m_pausePhx = false;

    // Step the simulation by this many steps
    // Used when the Physics is paused
    int m_manualStepPhx = 0;

    afPointCloudsHandler* m_pointCloudHandlerPtr;
};


///
/// \brief The afPickingConstraintData struct
///
struct afPickingConstraintData{

};


///
/// \brief The afMultiBody class
///
class afMultiBody{

    friend class afRigidBody;
    friend class afSoftBody;
    friend class afJoint;

public:

    afMultiBody();

    afMultiBody(afWorldPtr a_afWorld);

    virtual ~afMultiBody();

    virtual bool loadMultiBody(std::string a_multibody_config, bool enable_comm);

    inline std::string getHighResMeshesPath(){return m_multibody_high_res_meshes_path;}

    inline std::string getLowResMeshesPath(){return m_multibody_low_res_meshes_path;}

    inline std::string getMultiBodyPath(){return m_multibody_path;}

    inline std::string getNamespace(){return m_namespace;}

    // We can have multiple bodies connected to a single body.
    // There isn't a direct way in bullet to disable collision
    // between all these bodies connected in a tree
    void removeOverlappingCollisionChecking();

    //Remove collision checking for this entire multi-body, mostly for
    // debugging purposes
    void ignoreCollisionChecking();

    // Get Rigid Body or Soft Body belonging to this Specific Multibody
    afRigidBodyPtr getAFRigidBodyLocal(std::string a_name, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBodyLocal(std::string a_name);

    // Get the root parent of a body, if null is provided, returns the parent body
    // with most children. This method is similar to the corresponding afWorld
    // method however it searches in the local multibody space than the world space
    afRigidBodyPtr getRootAFRigidBodyLocal(afRigidBodyPtr a_bodyPtr = NULL);


    // Hard coded for now
    cVector3d getBoundaryMax(){return cVector3d(0.5, 0.5, 0.5);}

    // Do nothing for MB
    void setFrameSize(double size){}

    // Global Constraint ERP and CFM
    double m_jointERP = 0.1;
    double m_jointCFM = 0.1;

protected:

    afWorldPtr m_afWorld;

    std::string m_multibody_high_res_meshes_path, m_multibody_low_res_meshes_path;
    std::string m_namespace="";
    std::string m_multibody_path;

protected:

    cMaterial mat;
    template <typename T>
    std::string getNonCollidingIdx(std::string a_body_name, const T* tMap);
    void remapName(std::string &name, std::string remap_idx_str);

private:
    // The world has a list of all the bodies and joints belonging to all multibodies
    // The multibody has list of bodies and joints defined for this specific multibody
    afRigidBodyMap m_afRigidBodyMapLocal;
    afSoftBodyMap m_afSoftBodyMapLocal;
    afVehicleMap m_afVehicleMapLocal;
    afJointMap m_afJointMapLocal;
};


struct afWheel{
    enum class WheelBodyType{
        MESH=0,
        RIGID_BODY=1,
        INVALID=2
    };

    cMultiMesh* m_mesh;
    afRigidBodyPtr m_wheelBody = 0;
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

    WheelBodyType m_wheelBodyType;
};

class afVehicle: public afBaseObject{
public:
    afVehicle(afWorldPtr a_afWorld);

    ~afVehicle();

    // Load the vehicle from ambf format
    virtual bool loadVehicle(std::string vehicle_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    // Load the vehicle from ambf format
    virtual bool loadVehicle(YAML::Node* vehicle_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");

    virtual void updatePositionFromDynamics();

    virtual void afExecuteCommand(double dt);

protected:
    btDefaultVehicleRaycaster* m_vehicleRayCaster = nullptr;
    btRaycastVehicle* m_vehicle = nullptr;
    btRaycastVehicle::btVehicleTuning m_tuning;
    afRigidBodyPtr m_chassis;
    int m_numWheels = 0;
    std::vector<afWheel> m_wheels;
};


}
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
