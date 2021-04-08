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
#include "afUtils.h"
#include "afAttributes.h"
#include "chai3d.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
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
#include "ambf_server/Actuator.h"
#include "ambf_server/Camera.h"
#include "ambf_server/Light.h"
#include "ambf_server/Object.h"
#include "ambf_server/RigidBody.h"
#include "ambf_server/Sensor.h"
#include "ambf_server/Vehicle.h"
#include "ambf_server/World.h"
#endif

// Support for Depth Image to PointCloud2
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#endif


#include <time.h>
#include <random>
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class afBaseObject;
class afModel;
class afInertialObject;
class afRigidBody;
class afSoftBody;
class afGhostObject;
class afJoint;
class afWorld;
struct afRenderOptions;
class afCartesianController;
class afJointController;
class afConstraintActuator;
class afRayTracerSensor;
class afResistanceSensor;


typedef afBaseObject* afBaseObjectPtr;
typedef afModel* afModelPtr;
typedef afInertialObject* afInertialObjectPtr;
typedef afRigidBody* afRigidBodyPtr;
typedef afSoftBody* afSoftBodyPtr;
typedef afGhostObject* afGhostObjectPtr;
typedef afJoint* afJointPtr;
typedef afWorld* afWorldPtr;
typedef afConstraintActuator* afConstraintActuatorPtr;
typedef afRayTracerSensor* afRayTracerSensorPtr;
typedef afResistanceSensor* afResistanceSensorPtr;

typedef map<string, afRigidBodyPtr> afRigidBodyMap;
typedef map<string, afSoftBodyPtr> afSoftBodyMap;
typedef map<string, afGhostObjectPtr> afGhostObjectMap;
typedef map<string, afJointPtr> afJointMap;
typedef vector<afRigidBodyPtr> afRigidBodyVec;
typedef vector<afSoftBodyPtr> afSoftBodyVec;
typedef vector<afGhostObjectPtr> afGhostObjectVec;
typedef vector<afJointPtr> afJointVec;
//------------------------------------------------------------------------------
class afLight;
class afCamera;
typedef afLight* afLightPtr;
typedef afCamera* afCameraPtr;
typedef map<string, afLightPtr> afLightMap;
typedef map<string, afCameraPtr> afCameraMap;
typedef vector<afLightPtr> afLightVec;
typedef vector<afCameraPtr> afCameraVec;
//------------------------------------------------------------------------------
class afSensor;
class afResistanceSensor;
typedef afSensor* afSensorPtr;
typedef map<string, afSensorPtr> afSensorMap;
typedef vector<afSensorPtr> afSensorVec;
//------------------------------------------------------------------------------
class afActuator;
class afConstraintActuator;
typedef afActuator* afActuatorPtr;
typedef map<string, afActuatorPtr> afActuatorMap;
typedef vector<afActuatorPtr> afActuatorVec;
//------------------------------------------------------------------------------
typedef afModel* afModelPtr;
typedef map<string, afModelPtr> afModelMap;
typedef vector<afModelPtr> afModelVec;
//------------------------------------------------------------------------------
class afVehicle;
typedef afVehicle* afVehiclePtr;
typedef map<string, afVehiclePtr> afVehicleMap;
typedef vector<afVehiclePtr> afVehicleVec;
//------------------------------------------------------------------------------
class afPointCloud;
typedef afPointCloud* afPointCloudPtr;
typedef cMultiPoint* cMultiPointPtr;


typedef unsigned long ulong;


class afShapeUtils{
public:
    static cMesh* createVisualShape(const afPrimitiveShapeAttributes* a_primitiveShape);

    static btCollisionShape* createCollisionShape(const afPrimitiveShapeAttributes* a_primitiveShape, double a_margin);

    static btCollisionShape* createCollisionShape(const cMesh* a_collisionMesh, double a_margin, afCollisionMeshShapeType a_meshType);

    static btCompoundShape* createCollisionShape(const cMultiMesh* a_collisionMesh, double a_margin, afTransform m_inertialOffset, afCollisionMeshShapeType a_meshType);

    static std::vector<afRayAttributes> createRayAttribs(cMultiMesh* a_contourMesh, double a_range);
};


class afMaterialUtils{
public:
    static cMaterial createMaterialFromColor(afColorAttributes* a_color);
};


class afComm{
public:
    afComm(){}
    virtual ~afComm(){}

    virtual void afCreateCommInstance(afObjectType type, string a_name, string a_namespace, int a_min_freq=50, int a_max_freq=2000, double time_out=0.5);

    // This method is to retrieve all the commands for appropriate af comm instances.
    virtual void fetchCommands(double dt=0.001);

    //! This method applies updates Wall and Sim Time for AF State Message.
    virtual void afUpdateTimes(const double a_wall_time, const double a_sim_time);

    // Check if object is active or passive for communication
    inline bool isPassive(){return m_passive;}

    // Set as passive so it doesn't communication outside
    inline void setPassive(bool a_passive){m_passive = a_passive;}

    // Get Name of this object
    inline string getName(){return m_name;}

    // Get Namespace for this object
    inline string getNamespace(){return m_namespace;}

    inline string getQualifiedName(){return m_namespace + m_name;}

    // Get Min publishing frequency for this object
    inline int getMinPublishFrequency(){return m_minPubFreq;}

    // Get Max publishing frequency for this object
    inline int getMaxPublishFrequency(){return m_maxPubFreq;}

    // Set Min publishing frequency for this object
    inline void setMinPublishFrequency(int freq){m_minPubFreq = freq;}

    // Set Max publishing frequency for this object
    inline void setMaxPublishFrequency(int freq){m_maxPubFreq = freq;}

    // Get the type of communication instance
    afObjectType getCommType(){return m_commType;}

    // Set Name of object
    inline void setName(string a_name){m_name = a_name;}

    // Set namespace for this object
    inline void setNamespace(string a_namespace){m_namespace = a_namespace; }

public:

    // Flag to check if the any params have been set on the server for this comm instance
    bool m_paramsSet=false;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;

    string m_name;

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

protected:
    // The namespace for this body, this namespace affect afComm and the stored name of the body
    // in the internal body tree map.
    string m_namespace = "";


private:
    afObjectType m_commType;


    // Min publishing frequency
    uint m_minPubFreq=50;

    // Max publishing frequency
    uint m_maxPubFreq=1000;

    // If passive, this instance will not be reported for communication purposess.
    bool m_passive = false;
};


///
/// \brief The afSoftBodySurfaceProperties struct
///
struct afSoftBodyConfigProperties: public btSoftBody::Config{

};


///
/// \brief The afCartesianController struct
///
class afCartesianController: public afCartesianControllerAttributes{
public:
    afCartesianController();


    bool createFromAttribs(afCartesianControllerAttributes *a_attribs);

    // Get Controller Gains
    inline double getP_lin(){return P_lin;}
    inline double getD_lin(){return D_lin;}
    inline double getP_ang(){return P_ang;}
    inline double getD_ang(){return D_ang;}

    inline void setEnabled(bool a_enable){m_enabled = a_enable;}
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

    void setOutputType(afControlType type);

private:

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
/// \brief The afJointController class
///
class afJointController: public afJointControllerAttributes{
public:

    afJointController();

    bool createFromAttribs(afJointControllerAttributes *a_attribs);

    // Set some default values of PID
    // TODO: Maybe set PID's to 0 so the
    // user has to explicitly set them

    double e[4] = {0, 0, 0, 0};
    double ie[4] = {0, 0, 0, 0};
    double de[4] = {0, 0, 0, 0};
    double t[4]= {0, 0, 0, 0};
    double Ie_sum = 0.0;
    uint queue_length = 4;
    double output;
    double m_maxImpulse;
    double max_effort;

    // The min dt allowed.
    double m_min_dt = 0.00001;

    // Store the last effort command to compute and bound max impulse
    double m_last_cmd = 0;

    double computeOutput(double process_val, double set_point, double current_time);

    void boundImpulse(double& effort_cmd);

    void boundEffort(double& effort_cmd);

    // The default output type is velocity
};


///
/// \brief The afRayTracerUnit struct
///
struct afRayTracerResult{

public:
    void enableVisualization(afRayTracerSensor* sensorPtr, const afRayAttributes* attribs, double sphereRadius=0.1);

    // The rigid body that this proximity sensor is sensing
    btRigidBody* m_sensedBTRigidBody = nullptr;

    // This is the AF Rigid body sensed by this sensor
    afRigidBodyPtr m_sensedAFRigidBody = nullptr;

    // The soft body that this proximity sensor is sensing
    btSoftBody* m_sensedBTSoftBody = nullptr;

    // This is the AF Soft body sensed by this sensor
    afSoftBodyPtr m_sensedAFSoftBody = nullptr;

    // The internal index of the face belonging to the sensed soft body
    int m_sensedSoftBodyFaceIdx = -1;

    // The internal index of the node belonging to the sensed soft body
    int m_sensedSoftBodyNodeIdx = -1;

    // The node ptr to the sensed soft body's face
    btSoftBody::Face* m_sensedSoftBodyFace = nullptr;

    // The node ptr to the sensed soft body's node
    btSoftBody::Node* m_sensedSoftBodyNode = nullptr;

    // Boolean for sensor sensing something
    bool m_triggered;

    // Location of sensed point in World Frame. This is along of the sensor direction
    cVector3d m_sensedLocationWorld;

    // Visual markers to show the hit point and the sensor start and end points
    cMesh *m_hitSphereMesh = nullptr;

    cMesh *m_fromSphereMesh = nullptr;

    cMesh *m_toSphereMesh = nullptr;

    // Visual Mesh for Normal at Contact Point
    cMesh *m_hitNormalMesh;

    // Internal constraint for rigid body gripping
    btPoint2PointConstraint *pointConstraint = nullptr;

    // Depth fraction is the normalized penetration depth of the sensor
    double m_depthFraction = 0;

    // Normal at Contact Point
    cVector3d m_contactNormal;

    // Type of sensed body, could be a rigid body or a soft body
    afBodyType m_sensedBodyType;
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

    bool generateResistiveSensors(afWorldPtr a_afWorld, afRigidBodyPtr a_afRigidBodyPtr, cMultiMesh* a_multiMesh);
};

///
/// \brief The afChildJointPair struct
///
struct afChildJointPair{
    afChildJointPair(afRigidBodyPtr a_body, afJointPtr a_joint, bool a_directConnection = false){
        m_childBody = a_body;
        m_childJoint = a_joint;
        m_directConnection = a_directConnection;
    }
    afRigidBodyPtr m_childBody;
    afJointPtr m_childJoint;
    // Flag for checking if the body is connected directly or not to the parent body
    bool m_directConnection = false;
};


class afSceneObject{
    friend class afBaseObject;
public:
    afSceneObject(){
        m_localOffset.identity();
    }
    afSceneObject(cTransform& a_trans){
        m_localOffset = a_trans;
    }

    inline cTransform getOffsetTransform(){return m_localOffset;}

    void setLocalOffset(const cTransform &a_trans){m_localOffset = a_trans;}

    inline cGenericObject* getChaiObject(){return m_chaiObject;}

    void setChaiObject(cGenericObject* a_object){m_chaiObject = a_object;}

protected:
    cGenericObject* m_chaiObject;
    cTransform m_localOffset;
};


class afBaseObject: public afComm{

public:
    afBaseObject(afWorldPtr a_afWorld, afModelPtr a_afModelPtr = nullptr);
    virtual ~afBaseObject();

    virtual bool createFromAttribs(afBaseObjectAttributes* a_attribs);

    inline string getQualifiedIdentifier(){return m_namespace + m_identifier;}

    void setIdentifier(string a_name){m_identifier = a_name;}

    // Method called by afComm to apply positon, force or joint commands on the afRigidBody
    // In case the body is kinematic, only position cmds will be applied
    virtual void fetchCommands(double){}

    // The update method called at every simulation iteration.
    virtual void update(){}

    cVector3d getLocalPos();

    cMatrix3d getLocalRot();

    cTransform getLocalTransform();

    cTransform getGlobalTransform();

    // Get Initial Pose of this body
    inline cTransform getInitialTransform(){return m_initialTransform;}

    afBaseObjectPtr getParentObject();

    inline cMultiMesh* getVisualMesh(){return m_visualMesh;}

    void setLocalPos(const cVector3d &pos);

    void setLocalPos(const afVector3d &pos);

    void setLocalPos(double px, double py, double pz);

    void setLocalRot(const cMatrix3d &mat);

    void setLocalRot(const afMatrix3d &mat);

    void setLocalRot(const cQuaternion &quat);

    void setLocalRot(double qx, double qy, double qz, double qw);

    void setLocalTransform(const cTransform &trans);

    void setLocalTransform(const afTransform &trans);

    bool setParentObject(afBaseObjectPtr a_afObject);

    void clearParentObject();

    bool addChildObject(afBaseObjectPtr a_afObject);

    inline void setInitialTransform(cTransform a_trans){m_initialTransform = a_trans;}

    void setScale(double a_scale);

    // This method toggles the viewing of frames of this rigid body.
    void toggleFrameVisibility();

    bool isSceneObjectAlreadyAdded(afSceneObject* a_object);

    bool addChildSceneObject(cGenericObject* a_object, const cTransform& a_trans);

    bool addChildSceneObject(afSceneObject* a_object);

    void scaleSceneObjects(double a_scale);

    bool removeChildSceneObject(cGenericObject* a_object, bool removeFromGraph);

    bool removeChildSceneObject(afSceneObject* a_object, bool removeFromGraph);

    void removeAllChildSceneObjects(bool removeFromGraphs=true);

    virtual void updateSceneObjects();

    virtual void updateGlobalPose();

    void showVisualFrame();

    inline bool isShaderPgmDefined(){
        return m_shaderAttribs.m_shaderDefined;
    }

    // Enable Shader Program Associate with this object
    virtual void enableShaderProgram(){}

    // Resolve Parenting. Usuaully a mehtod to be called at a later if the object
    // to be parented to hasn't been loaded yet.
    virtual bool resolveParenting(string a_parentName="");

    // Ptr to afWorld
    afWorldPtr m_afWorld;

    afModelPtr m_modelPtr;

    // Parent body name defined in the ADF
    string m_parentName;

    // Identifier name, which could be different from the name
    string m_identifier;

    // Filepath to the visual mesh
    afPath m_visualMeshFilePath;

    cMultiMesh* m_visualMesh;

    std::vector<afSceneObject*> m_childrenSceneObjects;

    vector<afBaseObjectPtr> m_afChildrenObjects;

protected:
    // Initial location of Rigid Body
    cTransform m_initialTransform;

    // Scale of mesh
    double m_scale;

    // Flag for the Shader Program
    afShaderAttributes m_shaderAttribs;

    cTransform m_localTransform;

    cTransform m_globalTransform;

    afBaseObjectPtr m_parentObject;
};


///
/// \brief The afInertialObject class
///
class afInertialObject: public afBaseObject{
public:
    afInertialObject(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
    ~afInertialObject();

    // Apply force that is specified in the world frame at a point specified in world frame
    // This force is first converted into body frame and then is used to compute
    // the resulting torque in the body frame. This torque in the body frame is
    // then converted to the world frame and is applied to the body in the world frame
    // along with the original force in the world frame
    void applyForceAtPointOnBody(const cVector3d & a_forceInWorld, const cVector3d & a_pointInWorld);

    void applyForce(const cVector3d &a_force, const cVector3d& a_offset = cVector3d(0, 0, 0));

    void applyTorque(const cVector3d &a_torque);

    virtual void createInertialObject(){}

    // Compute the COM of the body and the tranform from mesh origin to the COM
    btVector3 computeInertialOffset(cMesh* mesh);

    // Compute the COM of the body and the tranform from mesh origin to the COM
    btVector3 computeInertialOffset(cMultiMesh* mesh);

    // function to check if this rigid body is part of the collision group
    // at a_idx
    bool checkCollisionGroupIdx(uint a_idx);
    // function to check if this rigid body is part of the collision group
    // at a_idxs
    bool isCommonCollisionGroupIdx(vector<uint> a_idx);

    void estimateInertia();

    inline double getMass(){return m_mass;}

    inline btVector3 getInertia(){return m_inertia;}

    inline btTransform getInertialOffsetTransform(){return m_T_iINb;}

    inline btTransform getInverseInertialOffsetTransform(){return m_T_bINi;}

    // Get Center of Mass Transform
    btTransform getCOMTransform();

    afSurfaceAttributes getSurfaceProperties();

    inline void setMass(double a_mass){m_mass = a_mass;}

    void setInertia(afVector3d& a_inertia);

    void setInertialOffsetTransform(btTransform & a_trans);

    void setSurfaceProperties(const afSurfaceAttributes& props);

    btRigidBody* m_bulletRigidBody;

    btSoftBody* m_bulletSoftBody;

    btCollisionShape *m_bulletCollisionShape;

    // Filepath to the collision mesh
    afPath m_collisionMeshFilePath;

    // cMultiMesh representation of collision mesh
    cMultiMesh* m_collisionMesh;

    // Collision groups for this inertial object
    vector<uint> m_collisionGroups;

protected:
    //! Inertial Offset Transform defined in the body frame
    btTransform m_T_iINb;

    //! Body Frame in the Inertial Offset Transform. Inverse of the above transform
    btTransform m_T_bINi;

    btDefaultMotionState* m_bulletMotionState;

    // Mass
    double m_mass;

    // Inertia
    btVector3 m_inertia;
};


///
/// \brief The afBody class
///
class afRigidBody: public afInertialObject{

    friend class afModel;
    friend class afJoint;
    friend class afWorld;

public:

    afRigidBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
    virtual ~afRigidBody();

    virtual void setLocalTransform(cTransform &trans);

    // Method called by afComm to apply positon, force or joint commands on the afRigidBody
    // In case the body is kinematic, only position cmds will be applied
    virtual void fetchCommands(double dt);

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void update();

    virtual bool createFromAttribs(afRigidBodyAttributes* a_attribs);

    virtual void createInertialObject();

    // Add a child to the afRidigBody tree, this method will internally populate the body graph
    virtual void addChildBodyJointPair(afRigidBodyPtr childBody, afJointPtr jnt);

    // Vector of child joint pair. Includes joints of all the
    // connected children all the way down to the last child. Also a vector of all the
    // children (children's children ... and so on also count as children)
    vector<afChildJointPair> m_CJ_PairsAll;

    // This vector contains the list of only the bodies connected via active joints. A joint can
    // be set as passive in the ADF file.
    vector<afChildJointPair> m_CJ_PairsActive;

    // A vector of all the parent bodies (not just the immediate parents but all the way up to the root parent)
    vector<afRigidBodyPtr> m_parentBodies;

    // Set the angle of all the child joints
    virtual void setAngle(double &angle);

    // Set the angles based on the num elements in the argument vector
    virtual void setAngle(vector<double> &angle);

    // Cleanup this rigid body
    void remove();

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
    inline vector<afSensorPtr> getAFSensors(){return m_afSensors;}

    // If the Position Controller is active, disable Position Controller from Haptic Device
    afControlType m_activeControllerType;

    // Instance of Cartesian Controller
    afCartesianController m_controller;

    // Estimated Force acting on body
    btVector3 m_estimatedForce;

    // Estimated Torque acting on body
    btVector3 m_estimatedTorque;

protected:

    // Name of visual and collision mesh
    string m_mesh_name, m_collision_mesh_name;

    // Iterator of connected rigid bodies
    vector<afRigidBodyPtr>::const_iterator m_bodyIt;

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

    // Method to estimate controller gains based on lumped masses
    void estimateCartesianControllerGains(afCartesianController& controller, bool computeLinear = true, bool computeAngular = true);

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

    // pool of threads for solving the body's sensors in paralled
    vector<std::thread*> m_sensorThreads;

    // Block size. i.e. number of sensors per thread
    uint m_sensorThreadBlockSize = 10;

    // This method uses the eq:
    // startIdx = threadIdx * m_sensorThreadBlockSize
    // endIdx = startIdx + m_sensorThreadBlockSize - 1
    // to compute the two indexes. The runs a loop to solve the indexes
    // in between so that we can progress in parallel
    bool updateBodySensors(uint threadIdx);

    // boolean flags for each thread to progress
    vector<bool> m_threadUpdateFlags;

    // Global flag for all sensor threads
    bool m_keepSensorThreadsAlive = true;

private:

    // Positions of all child joints
    vector<float> m_joint_positions;

    // Velocities of all child joints
    vector<float> m_joint_velocities;

    // Efforts of all child joints
    vector<float> m_joint_efforts;

    // Pointer to Multi body instance that constains this body
    afModelPtr m_mBPtr;

    // Last Position Error
    btVector3 m_dpos;

    // Type of geometry this body has (MESHES OR PRIMITIVES)
    afGeometryType m_visualGeometryType;
    afGeometryType m_collisionGeometryType;
};

///
/// \brief The afSoftBody class
///
class afSoftBody: public afInertialObject{

    friend class afModel;

    struct VertexTree{
        std::vector<int> triangleIdx;
        std::vector<int> vertexIdx;
    };

public:

    afSoftBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    // Execute the commands incomming of afObjectCmd handle
    virtual void afObjectCommandExecute(double){}

    virtual bool createFromAttribs(afSoftBodyAttributes* a_attribs);

    // Add child a softbody
    virtual void addChildBody(afSoftBodyPtr, afJointPtr){}

    virtual void createInertialObject();

    virtual void setLocalTransform(cTransform &trans);

    virtual void updateSceneObjects();

    bool cleanupMesh(cMultiMesh* multiMesh, std::vector<VertexTree>& a_vertexTree, std::vector<unsigned int>& a_triangles);

    bool generateFromMesh(cMultiMesh* mesh, const double margin);

    void updateMins(cVector3d &vMin, cVector3d &v);

    void updateMaxs(cVector3d &vMax, cVector3d &v);

    void clearArrays(bool * vtxChkBlock, int * vtxIdxBlock, int blockSize);

    // Function to detect, index and store repeat vertices
    void computeUniqueVerticesandTriangles(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<uint>* outputTriangles, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);

    // Function to detect, index and store repeat vertices
    void computeUniqueVerticesandTrianglesSequential(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<uint>* outputTriangles, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);

    // Helper Function to Create Links from Lines
    bool createLinksFromLines(btSoftBody* a_sb, std::vector< std::vector<int>>* a_lines, cMesh* a_mesh);

    // Copied from btSoftBodyHelpers with few modifications
    btSoftBody* createFromMesh(btSoftBodyWorldInfo& worldInfo, const btScalar* vertices, int nNodes, const unsigned int* triangles, int ntriangles, bool randomizeConstraints=true);

    //! This method toggles the drawing of skeletal model.
    void toggleSkeletalModelVisibility();

private:
    // Ptr to scalar vertex arrays of the sofy body
    std::vector<btScalar> m_verticesPtr;

    // Ptr to Triangles arrays referring to vertices by indices
    std::vector<unsigned int> m_trianglesPtr;

    // Vertex Tree containing vtx idx's that are repeated for a given vtx
    std::vector<VertexTree> m_vertexTree;

    // Boolean flag to indicate if we have been successful in reducing the mesh.
    // A reduced mesh should speed up rendering.
    bool m_meshReductionSuccessful;
};


class afGhostObject: public afInertialObject{

    friend class afModel;
    friend class afJoint;
    friend class afWorld;

public:

    afGhostObject(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
    virtual ~afGhostObject();

    // Method called by afComm to apply positon, force or joint commands on the afRigidBody
    // In case the body is kinematic, only position cmds will be applied
    virtual void fetchCommands(double dt){}

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void update();

    virtual bool createFromAttribs(afGhostObjectAttributes* a_attribs);

    virtual void setLocalTransform(cTransform &trans);

    // Cleanup this ghost body
    void remove();

    btPairCachingGhostObject* m_bulletGhostObject;

protected:

    std::vector<btRigidBody*> m_sensedBodies;

    static btGhostPairCallback* m_bulletGhostPairCallback;
};


///
/// \brief The afJoint class
///
class afJoint: public afBaseObject{
    friend class afRigidBody;
    friend class afGripperLink;
    friend class afModel;
    friend class afWorld;

public:

    afJoint(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual ~afJoint();

    virtual bool createFromAttribs(afJointAttributes* a_attribs);

    virtual void fetchCommands(double);

    virtual void update();

    btVector3 getDefaultJointAxisInParent(afJointType a_type);

    void cacheState(const double &dt);

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
    afJointType m_jointType;

    // Method to remove the afJoint
    void remove();

    bool isFeedBackEnabled(){return m_enableFeedback;}

protected:

    string m_childName;
    string m_jointName;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    double m_damping;
    double m_maxEffort;
    bool m_enableActuator;
    bool m_enableLimits;
    double m_lowerLimit, m_upperLimit;

    // Rotational offset of joint along the free joint axis
    double m_jointOffset;

    // Rotational offset of child along the free joint axis
    double m_childOffset;

    // Store parent and child afRigidBody to prevent lookups.
    afRigidBodyPtr m_afParentBody;
    afRigidBodyPtr m_afChildBody;

    // Wrench Feedback information from the joint
    bool m_enableFeedback = false;

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
    btHingeConstraint* m_hinge = nullptr;
    btSliderConstraint* m_slider = nullptr;
    btGeneric6DofSpring2Constraint* m_spring = nullptr;
    btPoint2PointConstraint* m_p2p = nullptr;
    btConeTwistConstraint* m_coneTwist = nullptr;
    btGeneric6DofConstraint* m_sixDof = nullptr;
    btGeneric6DofSpring2Constraint* m_sixDofSpring = nullptr;

    afJointController m_controller;

    // Vector of joint positions containing the last n joint values.
    uint m_jpSize = 2;
    vector<double> m_posArray;
    vector<double> m_dtArray;

};


///
/// \brief The afActuator class
///
class afActuator: public afBaseObject{
public:
    afActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afActuatorAttributes *a_attribs){return false;}

    virtual void actuate(){}

    virtual void deactuate(){}

    // Parent Body for this sensor
    afRigidBodyPtr m_parentBody;

    bool m_showActuator;

    virtual void fetchCommands(double){}

    virtual void update(){}

protected:
    bool m_actuate = false;
};



///
/// \brief The afConstraintActuator class. First type of actuator class. Ultimately we can add things like drills, cutters, magnets
/// etc all as actuators.
///
class afConstraintActuator: public afActuator{
public:
    afConstraintActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
    ~afConstraintActuator();

    virtual bool createFromAttribs(afConstraintActuatorAttributes* a_attribs);

    // The actuate methods will all result in the same thing. I.e. constraint a desired body or soft-body (face) to the parent body,
    // on which this actuator is mounted. The methods will use the current position of bodies or soft-bodyies to compute the parent
    // and child offsets for forming the constraint
    virtual void actuate(string a_rigid_body_name);

    virtual void actuate(afRigidBodyPtr a_rigidBody);

    virtual void actuate(string a_softbody_name, int a_face_index);

    virtual void actuate(afSoftBodyPtr a_softBody, int a_face_index);

    // In these actuate methods, explicit body offsets are provided.

    virtual void actuate(string a_rigid_body_name, btTransform a_bodyOffset);

    virtual void actuate(afRigidBodyPtr a_rigidBody, btTransform a_bodyOffset);

    virtual void actuate(string a_softbody_name, int a_face_index, btTransform a_bodyOffset);

    virtual void actuate(afSoftBodyPtr a_softBody, int a_face_index, btTransform a_bodyOffset);

    // Remove the constraint
    virtual void deactuate();

    virtual void fetchCommands(double dt);

    virtual void update();

protected:

    double m_maxImpulse = 3.0;
    double m_tau = 0.001;
    btTypedConstraint* m_constraint = nullptr;
    // Transform of actuator w.r.t. parent body
    cTransform m_T_aINp;

    cMesh* m_actuatorVisual = nullptr;


private:
    afRigidBodyPtr m_childBody = nullptr;
    afSensorPtr m_childSotBody = nullptr;
    int m_softBodyFaceIdx = -1;
    // Child offset w.r.t to actuator
    cVector3d m_P_cINp;

    bool m_active = false;
};

///
/// \brief The afSensor class
///
class afSensor: public afBaseObject{
    friend class afRigidBody;
public:
    afSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afSensorAttributes* a_attribs){return false;}

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

    virtual void fetchCommands(double dt);

    // Upate the sensor, usually called at each dynamic tick update of the physics engine
    virtual void update();

};





class afRayTracerSensor: public afSensor{

    friend class afProximitySensor;
    friend class afResistanceSensor;

public:
    // Constructor
    afRayTracerSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afRayTracerSensorAttributes* a_attribs);

    // Update sensor is called on each update of positions of RBs and SBs
    virtual void update();

    // Check if the sensor sensed something. Depending on what type of sensor this is
    inline bool isTriggered(uint idx){return m_rayTracerResults[idx].m_triggered;}

    // Get the type of sensed body
    inline afBodyType getSensedBodyType(uint idx){return m_rayTracerResults[idx].m_sensedBodyType;}

    // Return the sensed BT RigidBody's Ptr
    inline btRigidBody* getSensedBTRigidBody(uint idx){return m_rayTracerResults[idx].m_sensedBTRigidBody;}

    // Get the sensed AF Rigid Body's Ptr
    inline afRigidBodyPtr getSensedAFRigidBody(uint idx){return m_rayTracerResults[idx].m_sensedAFRigidBody;}

    // Get the sensed BT SoftBody's Ptr
    inline btSoftBody* getSensedBTSoftBody(uint idx){return m_rayTracerResults[idx].m_sensedBTSoftBody;}

    // Get the sensed AF Soft Body's Ptr
    inline afSoftBodyPtr getSensedAFSoftBody(uint idx){return m_rayTracerResults[idx].m_sensedAFSoftBody;}

    // Get the sensed SoftBody's Face
    inline btSoftBody::Face* getSensedSoftBodyFace(uint idx){return m_rayTracerResults[idx].m_sensedSoftBodyFace;}

    // Get the sensed SofyBody's Closest node the sensed point Node if any
    inline btSoftBody::Node* getSensedSoftBodyNode(uint idx){return m_rayTracerResults[idx].m_sensedSoftBodyNode;}

    // Get the sensed SofyBody's Face's index if any
    inline int getSensedSoftBodyFaceIdx(uint idx){return m_rayTracerResults[idx].m_sensedSoftBodyFaceIdx;}

    // Get the sensed SofyBody's Node's Idx
    inline int getSensedSoftBodyNodeIdx(uint idx){return m_rayTracerResults[idx].m_sensedSoftBodyNodeIdx;}

    // Get the sensed point in world frame
    inline cVector3d getSensedPoint(uint idx){return m_rayTracerResults[idx].m_sensedLocationWorld;}

    inline void setRayFromInLocal(const cVector3d& a_rayFrom, uint idx);

    inline void setRayToInLocal(const cVector3d& a_rayTo, uint idx);

    inline void setDirection(const cVector3d& a_direction, uint idx);

    inline void setRange(const double& a_range, uint idx){m_raysAttribs[idx].m_range = a_range;}

    inline void setSensorVisibilityRadius(const double& a_sensorVisibilityRadius){m_visibilitySphereRadius = a_sensorVisibilityRadius;}

    inline double getCount(){return m_count;}

    void enableVisualization();

    virtual void fetchCommands(double dt);


    double m_range;

protected:
    // The number of ray tracing elements belonging to this sensor
    uint m_count;

    vector<afRayAttributes> m_raysAttribs;
    vector<afRayTracerResult> m_rayTracerResults;

    // Size of spheres for the sensor visualization
    double m_visibilitySphereRadius;
};


///
/// \brief The afProximitySensor class
///
class afProximitySensor: public afRayTracerSensor{
public:
    // Constructor
    afProximitySensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
};


struct afRayContactResult{
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
    afResistanceSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afResistanceSensorAttributes* a_attribs);

    virtual void update();

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

    vector<afRayContactResult> m_rayContactResults;
};


///
/// \brief The afDepthPointCloud class
///
class afDepthPointCloud{
    friend class afCamera;
public:
    bool setup(uint a_width, uint a_height, uint a_numFields);
    ~afDepthPointCloud();

    inline uint getWidth(){return m_width;}
    inline uint getHeight(){return m_height;}
    inline uint getNumFields(){return m_numFields;}

protected:
    float *m_data = nullptr;
    uint m_width=0;
    uint m_height=0;
    uint m_numFields=0;
};


struct afNoiseModel{
    // Attribs for depth noise model

    afNoiseModel(){}
    ~afNoiseModel(){
        if (m_randomDistribution){
            delete m_randomDistribution;
        }
    }

    void initialize(double mean, double std_dev, double bias, bool enable=true){
        m_attribs.m_enable = enable;
        m_attribs.m_mean = mean;
        m_attribs.m_std_dev = std_dev;
        m_attribs.m_bias = bias;

        // Init Generator and Distribution

        m_randomNumberGenerator = default_random_engine(time(0));
        m_randomDistribution = new normal_distribution<double>(m_attribs.m_mean, m_attribs.m_std_dev);
    }

    double generate(){
        if (m_randomDistribution){
            return (*m_randomDistribution)(m_randomNumberGenerator) + m_attribs.m_bias;
        }
        else{
            cerr << "ERROR! NOISE DISTRIBUTION NOT INITIALIZED" << endl;
            return 0.0;
        }
    }

    bool isEnabled(){
        return m_attribs.m_enable;
    }

protected:
    default_random_engine m_randomNumberGenerator;
    normal_distribution<double>* m_randomDistribution = nullptr;
    afNoiseModelAttribs m_attribs;
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
    virtual void fetchCommands(double dt);

    // Define the virtual method for camera
    virtual void update();

    // Initialize
    bool init();

    void updateLabels(afRenderOptions &options);

    // Create the default camera. Implemented in case not additional cameras
    // are define in the AMBF config file
    bool createDefaultCamera();

    cCamera* getInternalCamera(){return m_camera;}

    virtual bool createFromAttribs(afCameraAttributes* a_attribs);

    // Since we changed the order of ADF loading such that cameras are loaded before
    // bodies etc. we wouldn't be able to find a body defined as a parent in the
    // camera data-block in the ADF file. Thus after loading the bodies, this method
    // should be called to find the parent.

    // Method similar to cCamera but providing a layer of abstraction
    // So that we can set camera transform internally and set the
    // transform of the afRigidBody surrounding the camera the same
    bool setView(const cVector3d& a_localPosition,
                     const cVector3d& a_localLookAt,
                     const cVector3d& a_localUp);

    // This method returns the camera "look at" position vector for this camera.
    inline cVector3d getLookVector()  const { return -m_localTransform.getLocalRot().getCol0(); }

    // This method returns the "right direction" vector for this camera.
    inline cVector3d getRightVector() const { return m_localTransform.getLocalRot().getCol1(); }

    // This method returns the "up" vector for this camera.
    inline cVector3d getUpVector()    const { return m_localTransform.getLocalRot().getCol2(); }

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
    cVector3d getTargetPosLocal();

    // Get the Target or the lookAt point
    cVector3d getTargetPosGlobal();

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
    vector<cLabel*> m_devHapticFreqLabels;

    // Position of mouse's x,y and scrolls cur and last coordinates for contextual window
    double mouse_x[2], mouse_y[2], mouse_scroll[2];
    bool mouse_l_clicked = false, mouse_r_clicked= false, mouse_scroll_clicked = false;
    bool mouse_r_btn_rising_edge = false, mouse_l_btn_rising_edge = false;


    cMatrix3d camRot, camRotPre;

    // Window parameters
    int m_width, m_height;
    int m_win_x, m_win_y;

    vector<string> m_controllingDevNames;

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

    // A separate depth camera to render the depth buffer
    cCamera* m_depthCamera = nullptr;

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

    // Hold the cCamera private and shield it's kinematics represented
    // by cGenericObject from the world since we want afRidigBody to
    // represent the kinematics instead
    cCamera* m_camera;

    // Flag to enable disable publishing of color image as a ROS topic
    bool m_publishImage = false;

    // Flag to enable disable publishing of depth image as a ROS topic
    bool m_publishDepth = false;

    // Depth Noise Model
    afNoiseModel m_depthNoise;

    cVector3d m_camPos;
    cVector3d m_camLookAt;
    cVector3d m_camUp;

    // Is this camera orthographic or not.
    bool m_orthographic = false;

    afDepthPointCloud m_depthPC;

    // The interval used to publish the image. A val of 1 means that publish every scene update
    // and a value of 10 means, publish every 10th scene udpate
    uint m_imagePublishInterval = 1;

    // Resolution of image
    afImageResolutionAttribs m_publishImageResolution;

    // The interval used to publish the depth. A val of 1 means that publish every scene update
    // and a value of 10 means, publish every 10th scene udpate
    uint m_depthPublishInterval = 10;

//    // Resolution of image
//    afImageResolutionAttribs m_publishDepthResolution;

    // Incremented every scene update (render method call)
    uint m_sceneUpdateCounter = 0;
};


///
/// \brief The afLight struct
///
class afLight: public afBaseObject{
public:
    afLight(afWorld* a_afWorld);

    virtual bool createFromAttribs(afLightAttributes* a_attribs);

    // Default light incase no lights are defined in the AMBF Config file
    bool createDefaultLight();

    virtual void fetchCommands(double dt);

    virtual void update();

    // Set direction of this light
    void setDir(const cVector3d& a_direction);

protected:
    cSpotLight* m_spotLight;

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_comm::LightType m_lightType;
#endif
};


///
/// \brief The afPointCloud class
///
class afPointCloud: public afBaseObject{
public:
    afPointCloud(afWorldPtr a_afWorld);

    cMultiPointPtr m_mpPtr;

    virtual void fetchCommands(double){}

    virtual void update();

    std::string m_topicName;

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_comm::PointCloudHandlerPtr m_pcCommPtr;
#endif
};


struct afRenderOptions{
    bool m_mirroredDisplay = false;
    bool m_updateLabels = true;
    bool m_windowClosed = false;
    string m_IIDModeStr = "";
    string m_IIDBtnActionStr = "";
};


//-----------------------------------------------------------------------------

///
/// \brief The afWorld class
///
class afWorld: public afComm{

    friend class afModel;

public:

    // Template method to add various types of objects
    template<typename T, typename TMap>
    bool addAFObject(T a_obj, string a_name, TMap* a_map);

    bool checkIfExists(afBaseObject* a_obj);

     // Template method to get a specific type of object
    template <typename T, typename TMap>
    T getAFObject(string a_name, TMap* a_map, bool suppress_warning);

     // Template method to get all objects of specific type
    template <typename Tvec, typename TMap>
    Tvec getAFObjects(TMap* tMap);

    afWorld(string a_global_namespace);

    virtual ~afWorld();

    virtual bool createFromAttribs(afWorldAttributes* a_attribs);

    virtual void render(afRenderOptions &options);

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

    void setGravity(afVector3d &vec);

    // This method assigns integration settings of simulator.
    void setIntegrationSettings(const double a_integrationTimeStep = 0.001, const int a_integrationMaxIterations = 1) { setIntegrationTimeStep(a_integrationTimeStep); setIntegrationMaxIterations(a_integrationMaxIterations); }

    // This method sets the internal integration time step of the simulation.
    void setIntegrationTimeStep(const double a_integrationTimeStep = 0.001) { m_integrationTimeStep = cMax(a_integrationTimeStep, 0.000001); }

    // The method returns the integration time step of the simulation.
    double getIntegrationTimeStep() { return (m_integrationTimeStep); }

    // This method sets the maximum number of iteration per integration time step.
    void setIntegrationMaxIterations(const int a_integrationMaxIterations = 1) { m_integrationMaxIterations = cMax(a_integrationMaxIterations, 1); }

    //! This method returns the maximum number of iteration per integration time step.
    int getIntegrationMaxIterations() { return (m_integrationMaxIterations);}

    int getMaxIterations(){return m_maxIterations;}

    // This method returns the current simulation time
    double getWallTime(){return m_wallClock;}

    // This method returns the current simulation time
    double getSimulationTime(){return m_simulationTime;}

    // This method gets the time difference between current time and last simulation time
    double getSimulationDeltaTime();

    double computeStepSize(bool adjust_intetration_steps = false);

    void estimateBodyWrenches();

    //! This method updates the simulation over a time interval.
    virtual void updateDynamics(double a_interval, double a_wallClock=0, double a_loopFreq = 0, int a_numDevices = 0);

    //! This method updates the position and orientation from Bullet models to CHAI3D models.
    virtual void updateSceneObjects();

    void addSceneObjectToWorld(cGenericObject* a_cObject);

    void removeSceneObjectFromWorld(cGenericObject* a_cObject);

    string addAFLight(afLightPtr a_rb);

    string addAFCamera(afCameraPtr a_rb);

    string addAFRigidBody(afRigidBodyPtr a_rb);

    string addAFSoftBody(afSoftBodyPtr a_sb);

    string addAFGhostObject(afGhostObjectPtr a_go);

    string addAFJoint(afJointPtr a_jnt);

    string addAFActuator(afActuatorPtr a_actuator);

    string addAFSensor(afSensorPtr a_sensor);

    string addAFModel(afModelPtr a_model);

    string addAFVehicle(afVehiclePtr a_vehicle);

    // This method build the collision graph based on the collision group numbers
    // defined in the bodies
    void buildCollisionGroups();


    afLightPtr getAFLight(string a_name, bool suppress_warning=false);

    afCameraPtr getAFCamera(string a_name, bool suppress_warning=false);

    afRigidBodyPtr getAFRigidBody(string a_name, bool suppress_warning=false);

    afRigidBodyPtr getAFRigidBody(btRigidBody* a_body, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBody(string a_name, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBody(btSoftBody* a_body, bool suppress_warning=false);

    afGhostObjectPtr getAFGhostObject(string a_name, bool suppress_warning=false);

    afGhostObjectPtr getAFGhostObject(btGhostObject* a_body, bool suppress_warning=false);

    afJointPtr getAFJoint(string a_name);

    afActuatorPtr getAFActuator(string a_name);

    afSensorPtr getAFSensor(string a_name);

    afModelPtr getAFModel(string a_name, bool suppress_warning=false);

    afVehiclePtr getAFVehicle(string a_name, bool suppress_warning=false);


    inline afLightMap* getAFLightMap(){return &m_afLightMap;}

    inline afCameraMap* getAFCameraMap(){return &m_afCameraMap;}

    inline afRigidBodyMap* getAFRigidBodyMap(){return &m_afRigidBodyMap;}

    inline afSoftBodyMap* getAFSoftBodyMap(){return &m_afSoftBodyMap;}

    inline afGhostObjectMap* getAFGhostObjectMap(){return &m_afGhostObjectMap;}

    inline afJointMap* getAFJointMap(){return &m_afJointMap;}

    inline afActuatorMap* getAFActuatorMap(){return &m_afActuatorMap;}

    inline afSensorMap* getAFSensorMap(){return &m_afSensorMap;}

    inline afModelMap* getAFModelMap(){return &m_afModelMap;}

    inline afVehicleMap* getAFVehicleMap(){return &m_afVehicleMap;}


    afLightVec  getAFLighs();

    afCameraVec getAFCameras();

    afRigidBodyVec getAFRigidBodies();

    afSoftBodyVec getAFSoftBodies();

    afGhostObjectVec getAFGhostObjects();

    afJointVec getAFJoints();

    afActuatorVec getAFActuators();

    afSensorVec getAFSensors();

    afModelVec getAFMultiBodies();

    afVehicleVec getAFVehicles();


    string resolveGlobalNamespace(string a_name);

    string getGlobalNamespace(){return m_global_namespace;}

    void setGlobalNamespace(string a_namespace);

    virtual void fetchCommands(double dt);

    // Get the root parent of a body, if null is provided, returns the parent body
    // with most children
    afRigidBodyPtr getRootAFRigidBody(afRigidBodyPtr a_bodyPtr = nullptr);

    bool pickBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    bool movePickedBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    void removePickingConstraint();

    virtual void enableShaderProgram();

    void loadSkyBox();

    // The collision groups are sorted by integer indices. A group is an array of
    // rigid bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // a body can be a part of multiple groups
    map<uint, vector<afInertialObjectPtr> > m_collisionGroups;

public:

    vector<afBaseObjectPtr> m_childrenAFObjects;

    GLFWwindow* m_mainWindow;

    //data for picking objects
    class btRigidBody* m_pickedBulletRigidBody = nullptr;

    afRigidBodyPtr m_pickedAFRigidBody = nullptr;

    cMaterialPtr m_pickedAFRigidBodyColor; // Original color of picked body for reseting later

    cMaterial m_pickColor; // The color to be applied to the picked body

    class btSoftBody* m_pickedSoftBody = nullptr; // Picked SoftBody

    struct btSoftBody::Node* m_pickedNode = nullptr; // Picked SoftBody Node

    int m_pickedNodeIdx = -1; // Picked SoftBody Node

    double m_pickedNodeMass = 0;

    cVector3d m_pickedNodeGoal;

    class btTypedConstraint* m_pickedConstraint = nullptr;

    int m_savedState;

    cVector3d m_oldPickingPos;

    cVector3d m_hitPos;

    double m_oldPickingDist;

    cVector3d m_pickedOffset;

    cMesh* m_pickSphere = nullptr;

    cPrecisionClock g_wallClock;

    bool m_shaderProgramDefined = false;

    // Vertex Shader Filepath
    afPath m_vsFilePath;

    // Fragment Shader Filepath
    afPath m_fsFilePath;
    //    cMesh* m_pickDragVector;

    afShaderAttributes m_globalBodyShaderAttribs;

    // Skybox Mesh
    cMesh* m_skyBoxMesh = nullptr;

    afSkyBoxAttributes m_skyBoxAttribs;

    afPath m_world_config_path;

    // a frequency counter to measure the simulation graphic rate
    cFrequencyCounter m_freqCounterGraphics;

    // a frequency counter to measure the simulation haptic rate
    cFrequencyCounter m_freqCounterHaptics;

    map<string, afPointCloudPtr> m_pcMap;

    cWorld* m_chaiWorld = nullptr;

    // Bullet dynamics world.
    btDiscreteDynamicsWorld* m_bulletWorld = nullptr;

    // Bullet broad phase collision detection algorithm.
    btBroadphaseInterface* m_bulletBroadphase = nullptr;

    // Bullet collision configuration.
    btCollisionConfiguration* m_bulletCollisionConfiguration = nullptr;

    // Bullet collision dispatcher.
    btCollisionDispatcher* m_bulletCollisionDispatcher = nullptr;

    // Bullet physics solver.
    btConstraintSolver* m_bulletSolver = nullptr;

    // Bullet Softbody World Info
    btSoftBodyWorldInfo* m_bulletSoftBodyWorldInfo = nullptr;

    // Bullet Soft Body Solver
    btSoftBodySolver* m_bulletSoftBodySolver = nullptr;

    // The desired freq of physics loop
    uint m_physicsFrequency = 1000;

    // The desired freq of haptics loop
    uint m_hapticsFrequency = 1000;

    uint m_updateCounterLimit = 2000;

protected:

    afLightMap m_afLightMap;

    afCameraMap m_afCameraMap;

    afRigidBodyMap m_afRigidBodyMap;

    afSoftBodyMap m_afSoftBodyMap;

    afGhostObjectMap m_afGhostObjectMap;

    afJointMap m_afJointMap;

    afActuatorMap m_afActuatorMap;

    afSensorMap m_afSensorMap;

    afModelMap m_afModelMap;

    afVehicleMap m_afVehicleMap;

    // If this string is set, it will force itself to preeced all nampespaces
    // regardless of whether any namespace starts with a '/' or not.
    string m_global_namespace;

    // Current time of simulation.
    double m_simulationTime;

    // Integration time step.
    double m_integrationTimeStep;

    // Wall Clock in Secs
    double m_wallClock;

    // Last Simulation Time
    double m_lastSimulationTime;

    // Maximum number of iterations.
    int m_integrationMaxIterations;

private:

    static double m_enclosureL;

    static double m_enclosureW;

    static double m_enclosureH;

    static int m_maxIterations;

    // Global flag to pause simulation
    bool m_pausePhx = false;

    // Step the simulation by this many steps
    // Used when the Physics is paused
    int m_manualStepPhx = 0;
};


///
/// \brief The afPickingConstraintData struct
///
struct afPickingConstraintData{

};


///
/// \brief The afModel class
///
class afModel: public afBaseObject{

    friend class afRigidBody;
    friend class afSoftBody;
    friend class afGhostObject;
    friend class afJoint;

public:

    afModel(afWorldPtr a_afWorld);

    virtual ~afModel();

    virtual bool createFromAttribs(afModelAttributes* a_attribs);

    virtual void fetchCommands(double dt){}

    afRigidBodyMap* getRigidBodyMap(){return &m_afRigidBodyMapLocal;}
    afSoftBodyMap* getSoftBodyMap(){return &m_afSoftBodyMapLocal;}
    afGhostObjectMap* getGhostObjectMap(){return &m_afGhostObjectMapLocal;}
    afVehicleMap* getVehicleMap(){return &m_afVehicleMapLocal;}
    afJointMap* getJointMap(){return &m_afJointMapLocal;}
    afActuatorMap* getActuatorMap(){return &m_afActuatorMapLocal;}
    afSensorMap* getSensorMap(){return &m_afSensorMapLocal;}

    // We can have multiple bodies connected to a single body.
    // There isn't a direct way in bullet to disable collision
    // between all these bodies connected in a tree
    void removeOverlappingCollisionChecking();

    //Remove collision checking for this entire multi-body, mostly for
    // debugging purposes
    void ignoreCollisionChecking();

    // Get Rigid Body or Soft Body belonging to this Specific Model
    afRigidBodyPtr getAFRigidBodyLocal(string a_name, bool suppress_warning=false);

    afSoftBodyPtr getAFSoftBodyLocal(string a_name);

    // Get the root parent of a body, if null is provided, returns the parent body
    // with most children. This method is similar to the corresponding afWorld
    // method however it searches in the local model space than the world space
    afRigidBodyPtr getRootAFRigidBodyLocal(afRigidBodyPtr a_bodyPtr = nullptr);

    // Global Constraint ERP and CFM
    double m_jointERP = 0.1;
    double m_jointCFM = 0.1;

protected:

    cMaterial mat;
    template <typename T>
    string getNonCollidingIdx(string a_body_name, const T* tMap);
    void remapName(string &name, string remap_idx_str);

private:
    // The world has a list of all the bodies and joints belonging to all multibodies
    // The model has list of bodies and joints defined for this specific model
    afRigidBodyMap m_afRigidBodyMapLocal;
    afSoftBodyMap m_afSoftBodyMapLocal;
    afGhostObjectMap m_afGhostObjectMapLocal;
    afVehicleMap m_afVehicleMapLocal;
    afJointMap m_afJointMapLocal;
    afActuatorMap m_afActuatorMapLocal;
    afSensorMap m_afSensorMapLocal;
};


///
/// \brief The afWheel struct
///
struct afWheel{

    cMultiMesh* m_mesh;
    afRigidBodyPtr m_wheelBody = nullptr;

    afWheelRepresentationType m_wheelRepresentationType;
};


class afVehicle: public afInertialObject{
public:
    afVehicle(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    ~afVehicle();

    virtual bool createFromAttribs(afVehicleAttributes* a_attribs);

    virtual void update();

    virtual void fetchCommands(double dt);

protected:
    btDefaultVehicleRaycaster* m_vehicleRayCaster = nullptr;
    btRaycastVehicle* m_vehicle = nullptr;
    btRaycastVehicle::btVehicleTuning m_tuning;
    afRigidBodyPtr m_chassis;
    uint m_numWheels = 0;
    vector<afWheel> m_wheels;
    vector<afWheelAttributes> m_wheelAttribs;
};


}
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
