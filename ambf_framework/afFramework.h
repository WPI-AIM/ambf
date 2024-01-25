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
#ifndef AF_LIBRARY_H
#define AF_LIBRARY_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

#include "afUtils.h"
#include "afAttributes.h"
#include "afPluginInterface.h"
#include "afPluginManager.h"

//------------------------------------------------------------------------------

#include "chai3d.h"

//------------------------------------------------------------------------------

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

//------------------------------------------------------------------------------

#include <thread>

//-----------------------------------------------------------------------------

#include <GLFW/glfw3.h>
//-----------------------------------------------------------------------------

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
class afCamera;
typedef afCamera* afCameraPtr;
typedef map<string, afCameraPtr> afCameraMap;
typedef vector<afCameraPtr> afCameraVec;
//------------------------------------------------------------------------------
class afLight;
typedef afLight* afLightPtr;
typedef map<string, afLightPtr> afLightMap;
typedef vector<afLightPtr> afLightVec;
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
class afVolume;
typedef afVolume* afVolumePtr;
typedef map<string, afVolumePtr> afVolumeMap;
typedef vector<afVolumePtr> afVolumeVec;
//------------------------------------------------------------------------------
class afPointCloud;
typedef afPointCloud* afPointCloudPtr;
typedef cMultiPoint* cMultiPointPtr;
//------------------------------------------------------------------------------
typedef map<string, afBaseObject*> afBaseObjectMap;
typedef vector<afBaseObjectPtr> afBaseObjectVec;
typedef map<afType, map<string, afBaseObject*> > afChildrenMap;

typedef unsigned long ulong;


class afShapeUtils{
public:
    static cMesh* createVisualShape(const afPrimitiveShapeAttributes* a_primitiveShape);

    static btCollisionShape* createCollisionShape(const afPrimitiveShapeAttributes* a_primitiveShape, double a_margin);

    static btCollisionShape* createCollisionShape(cMesh* a_collisionMesh, double a_margin, afCollisionMeshShapeType a_meshType);

    static btCompoundShape* createCollisionShape(cMultiMesh* a_collisionMesh, double a_margin, afTransform m_inertialOffset, afCollisionMeshShapeType a_meshType);

    static std::vector<afRayAttributes> createRayAttribs(cMultiMesh* a_contourMesh, double a_range);
};


class afMaterialUtils{
public:
    static cMaterial createFromAttribs(afColorAttributes* a_color);
};

class afVisualUtils{
public:
    static bool createFromAttribs(afVisualAttributes* attribs, cMultiMesh* mesh, string obj_name);
};

class afShaderUtils{
public:
    static cShaderProgramPtr createFromAttribs(afShaderAttributes* attribs, string objName, string type);
};


class afIdentification{
public:

    afIdentification(afType a_type);

    // Get the type of communication instance
    afType getType(){return m_type;}

    string getTypeAsStr();

    // Get Name of this object
    string getName();

    // Get Namespace for this object
    string getNamespace();

    string getQualifiedName();

    // Set Name of object
    void setName(string a_name);

    // Set namespace for this object
    void setNamespace(string a_namespace);

    string getQualifiedIdentifier();

    void setIdentifier(string a_name){m_identifier = a_name;}

    void setGlobalRemapIdx(string idx){m_globalRemapIdx = idx;}

    string getGlobalRemapIdx(){return m_globalRemapIdx;}

protected:
    // The namespace for this body, this namespace affect afComm and the stored name of the body
    // in the internal body tree map.
    string m_namespace;

    string m_name;

    // Identifier name, which could be different from the name
    string m_identifier;

    // Type of object
    const afType m_type;

    string m_globalRemapIdx;
};


class afComm{
public:
    afComm(){}
    virtual ~afComm(){}

    // Check if object is active or passive for communication
    inline bool isPassive(){return m_passive;}

    // Set as passive so it doesn't communication outside
    inline void setPassive(bool a_passive){m_passive = a_passive;}

    // Get Max publishing frequency for this object
    int getMaxPublishFrequency();

    // Get Min publishing frequency for this object
    int getMinPublishFrequency();

    // Set Max publishing frequency for this object
    void setMaxPublishFrequency(int freq);

    // Set Min publishing frequency for this object
    void setMinPublishFrequency(int freq);

    inline double getCurrentTimeStamp(){return m_timeStamp;}

    inline void setTimeStamp(double a_sec){m_timeStamp = a_sec;}

    // Override the Max Freq
    static void overrideMaxPublishingFrequency(int freq);

    // Override the Min Freq
    static void overrideMinPublishingFrequency(int freq);

    static string getGlobalNamespacePrefix(){return s_global_namespace_prefix;}

    static void setGlobalNamespacePrefix(string a_namespace_prefix);

private:
    // Min publishing frequency
    uint m_minPubFreq=50;

    // Max publishing frequency
    uint m_maxPubFreq=1000;

    // If passive, this instance will not be reported for communication purposess.
    bool m_passive = false;

    static bool s_globalOverride;
    static int s_maxFreq;
    static int s_minFreq;

    static string s_global_namespace_prefix;

    double m_timeStamp = 0.0;
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

    // This is the Rigid body sensed by this sensor
    afRigidBodyPtr m_sensedRigidBody = nullptr;

    // The soft body that this proximity sensor is sensing
    btSoftBody* m_sensedBTSoftBody = nullptr;

    // This is the Soft body sensed by this sensor
    afSoftBodyPtr m_sensedSoftBody = nullptr;

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


class afBaseObject: public afIdentification, public afComm{

public:
    afBaseObject(afType a_type, afWorldPtr a_afWorld, afModelPtr a_afModelPtr = nullptr);
    virtual ~afBaseObject();

    virtual bool createFromAttribs(afBaseObjectAttributes* a_attribs);

    virtual bool loadPlugins(afBaseObjectPtr objPtr, afBaseObjectAttribsPtr attribs, vector<afPluginAttributes>* pluginAttribs);

    // The update method called at every simulation iteration.
    virtual void update(double dt);

    virtual void reset();

    virtual afBaseObjectAttribsPtr getAttributes(){
        return m_attributes;
    }

    cVector3d getLocalPos();

    cMatrix3d getLocalRot();

    cTransform getLocalTransform();

    cTransform getGlobalTransform();

    double getWallTime();

    double getSimulationTime();

    virtual bool getVisibleFlag(){return m_visible;}

    // Get Initial Pose of this body
    inline cTransform getInitialTransform(){return m_initialTransform;}

    inline afBaseObjectPtr getParentObject(){return m_parentObject;}

    void setLocalPos(const cVector3d &pos);

    void setLocalPos(const afVector3d &pos);

    void setLocalPos(double px, double py, double pz);

    void setLocalRot(const cMatrix3d &mat);

    void setLocalRot(const afMatrix3d &mat);

    void setLocalRot(const cQuaternion &quat);

    void setLocalRot(double qx, double qy, double qz, double qw);

    virtual void setLocalTransform(const cTransform &trans);

    bool setParentObject(afBaseObjectPtr a_afObject);

    virtual void setVisibleFlag(bool val){m_visible = val;}

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

    bool loadCommunicationPlugin(afBaseObjectPtr a_objPtr, afBaseObjectAttribsPtr a_attribs);

    virtual void updateSceneObjects();

    void pluginsGraphicsUpdate();

    void pluginsPhysicsUpdate(double dt);

    void pluginsReset();

    virtual void updateGlobalPose(bool a_forceUpdate, cTransform a_parentTransform = cTransform());

    void calculateFrameSize();

    // Resolve Parenting. Usuaully a mehtod to be called at a later if the object
    // to be parented to hasn't been loaded yet.
    virtual bool resolveParent(string a_parentName="", bool suppress_warning=false);

    // Ptr to afWorld
    afWorldPtr m_afWorld;

    afModelPtr m_modelPtr;

    // Parent body name defined in the ADF
    string m_parentName;

    std::vector<afSceneObject*> m_childrenSceneObjects;

    vector<afBaseObjectPtr> m_afChildrenObjects;

protected:

    // Initial location of Rigid Body
    cTransform m_initialTransform;

    // Scale of mesh
    double m_scale;

    cTransform m_localTransform;

    cTransform m_globalTransform;

    afBaseObjectPtr m_parentObject;

    // Plugin Manager
    afBaseObjectPluginManager m_pluginManager;

    vector<afBaseObjectPtr> m_childrenObjects;

    virtual void storeAttributes(const afBaseObjectAttribsPtr a_attribs){
        m_attributes = a_attribs;
    }

private:
    // Whether or not this object is visible
    bool m_visible = false;

    afBaseObjectAttribsPtr m_attributes;
};


///
/// \brief The afMeshObject class
///
class afMeshObject{
public:
    afMeshObject(afWorldPtr a_afWorld, afModelPtr a_afModel);

    cMultiMesh *getVisualObject();

    bool isShaderProgramDefined();

    bool isShaderProgramDefined(cMesh* a_mesh);

    // Enable Shader Program Associate with this object
    virtual void loadShaderProgram();

    virtual bool isNormalMapDefined();

    virtual bool isNormalMapDefined(cMesh* a_mesh);

    virtual bool isNormalTextureDefined();

    virtual bool isNormalTextureDefined(cMesh* a_mesh);

    virtual void enableShaderNormalMapping(bool enable);

    virtual void enableShaderNormalMapping(bool enable, cMesh* a_mesh);

    virtual cShaderProgramPtr getShaderProgram();

    virtual void setShaderProgram(cShaderProgramPtr a_program);

    virtual void backupShaderProgram();

    virtual void restoreShaderProgram();

    // Filepath to the visual mesh
    afPath m_visualMeshFilePath;

    cMultiMesh* m_visualMesh;

protected:

    // Flag for the Shader Program
    afShaderAttributes m_shaderAttribs;

    // Shader Program Backup
    cShaderProgramPtr m_shaderProgramBackup = nullptr;

private:
    afModelPtr m_model;

    afWorldPtr m_world;
};


///
/// \brief The afObjectManager class
///
class afObjectManager{
public:

    afObjectManager();

    string addLight(afLightPtr a_rb);

    string addCamera(afCameraPtr a_rb);

    string addRigidBody(afRigidBodyPtr a_rb);

    string addSoftBody(afSoftBodyPtr a_sb);

    string addGhostObject(afGhostObjectPtr a_go);

    string addJoint(afJointPtr a_jnt);

    string addActuator(afActuatorPtr a_actuator);

    string addSensor(afSensorPtr a_sensor);

    string addVehicle(afVehiclePtr a_vehicle);

    string addVolume(afVolumePtr a_volume);

    string addBaseObject(afBaseObjectPtr a_obj);

    bool addBaseObject(afBaseObjectPtr a_obj, string a_name);

    void resolveObjectsMissingParents(afBaseObjectPtr a_newObject);

    afLightPtr getLight(string a_name, bool suppress_warning=false);

    afCameraPtr getCamera(string a_name, bool suppress_warning=false);

    afRigidBodyPtr getRigidBody(string a_name, bool suppress_warning=false);

    afRigidBodyPtr getRigidBody(btRigidBody* a_body, bool suppress_warning=false);

    afSoftBodyPtr getSoftBody(string a_name, bool suppress_warning=false);

    afSoftBodyPtr getSoftBody(btSoftBody* a_body, bool suppress_warning=false);

    afGhostObjectPtr getGhostObject(string a_name, bool suppress_warning=false);

    afGhostObjectPtr getGhostObject(btGhostObject* a_body, bool suppress_warning=false);

    // Get an object by this name. The object could be of any type
    afBaseObjectPtr getBaseObject(string a_name, bool suppress_warning);

    // Mark that this object needs parenting
    void addObjectMissingParent(afBaseObjectPtr a_obj);

    // Get the root parent of a body, if null is provided, returns the parent body
    // with most children. This method is similar to the corresponding afWorld
    // method however it searches in the local model space than the world space
    afRigidBodyPtr getRootRigidBody(afRigidBodyPtr a_bodyPtr = nullptr);

    afJointPtr getJoint(string a_name, bool suppress_warning=false);

    afActuatorPtr getActuator(string a_name, bool suppress_warning=false);

    afSensorPtr getSensor(string a_name, bool suppress_warning=false);

    afVehiclePtr getVehicle(string a_name, bool suppress_warning=false);

    afVolumePtr getVolume(string a_name, bool suppress_warning=false);

    afBaseObjectPtr getBaseObject(string a_name, afBaseObjectMap* a_map, bool suppress_warning);

     // Template method to get all objects of specific type
    template <class T>
    vector<T*> getBaseObjects(afBaseObjectMap* objMap);


    afLightVec getLights();

    afCameraVec getCameras();

    afRigidBodyVec getRigidBodies();

    afSoftBodyVec getSoftBodies();

    afGhostObjectVec getGhostObjects();

    afJointVec getJoints();

    afActuatorVec getActuators();

    afSensorVec getSensors();

    afVehicleVec getVehicles();

    afVolumeVec getVolumes();


    inline afBaseObjectMap* getLightMap(){return &m_childrenObjectsMap[afType::LIGHT];}

    inline afBaseObjectMap* getCameraMap(){return &m_childrenObjectsMap[afType::CAMERA];}

    inline afBaseObjectMap* getRigidBodyMap(){return &m_childrenObjectsMap[afType::RIGID_BODY];}

    inline afBaseObjectMap* getSoftBodyMap(){return &m_childrenObjectsMap[afType::SOFT_BODY];}

    inline afBaseObjectMap* getGhostObjectMap(){return &m_childrenObjectsMap[afType::GHOST_OBJECT];}

    inline afBaseObjectMap* getJointMap(){return &m_childrenObjectsMap[afType::JOINT];}

    inline afBaseObjectMap* getActuatorMap(){return &m_childrenObjectsMap[afType::ACTUATOR];}

    inline afBaseObjectMap* getSensorMap(){return &m_childrenObjectsMap[afType::SENSOR];}

    inline afBaseObjectMap* getVehicleMap(){return &m_childrenObjectsMap[afType::VEHICLE];}

    inline afBaseObjectMap* getVolumeMap(){return &m_childrenObjectsMap[afType::VOLUME];}

    inline afChildrenMap* getChildrenMap(){return &m_childrenObjectsMap;}

    // Return true if object exists in the vec
    bool checkIfExists(afBaseObject* a_obj, vector<afBaseObject*> *a_objectsVec);

protected:
    afChildrenMap m_childrenObjectsMap;

    vector<afBaseObjectPtr> m_afObjectsMissingParents;
};



///
/// \brief The afModelManager class
///
class afModelManager: public afObjectManager{
public:
    afModelManager(afWorldPtr a_afWorld);

    string addModel(afModelPtr a_model);

    afModelPtr getModel(string a_name, bool suppress_warning=false);

    afModelVec getModels();

    inline afModelMap* getModelMap(){return &m_modelsMap;}

protected:

    void addModelsChildrenToWorld(afModelPtr a_model);

    void addChildsSceneObjectsToWorld(afBaseObjectPtr a_object);

    afModelMap m_modelsMap;
    afWorldPtr m_afWorld;
};


///
/// \brief The afInertialObject class
///
class afInertialObject: public afBaseObject, public afMeshObject{
public:
    afInertialObject(afType a_type, afWorldPtr a_afWorld, afModelPtr a_modelPtr);
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

    void setGravity(const cVector3d& a_gravity);

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

    // Gravity
    cVector3d m_gravity;
    bool m_overrideGravity;
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

    virtual void setLocalTransform(const cTransform &trans);

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void update(double dt);

    virtual void reset();

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
    void addSensor(afSensorPtr a_sensor){m_afSensors.push_back(a_sensor);}

    // Add sensor to this body
    void addActuator(afActuatorPtr a_actuator){m_afActuators.push_back(a_actuator);}

    // Get the sensors for this body
    inline vector<afSensorPtr> getSensors(){return m_afSensors;}

    // If the Position Controller is active, disable Position Controller from Haptic Device
    afControlType m_activeControllerType;

    // Instance of Cartesian Controller
    afCartesianController m_controller;

    // Estimated Force acting on body
    btVector3 m_estimatedForce;

    // Estimated Torque acting on body
    btVector3 m_estimatedTorque;

    // Toggle publishing of joint positions
    bool m_publish_joint_positions = false;

    // Toggle publishing of children names
    bool m_publish_children_names = false;

    // Toggle publishing of joint names
    bool m_publish_joint_names = true;

protected:

    // Name of visual and collision mesh
    string m_mesh_name, m_collision_mesh_name;

    // Iterator of connected rigid bodies
    vector<afRigidBodyPtr>::const_iterator m_bodyIt;

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
    afGeometryType m_collisionGeometryType;
};


struct afVertexTree{
    std::vector<int> triangleIdx;
    std::vector<int> vertexIdx;
};

///
/// \brief The afMeshCleanup class
///
class afMeshCleanup{
public:

    static void updateMins(cVector3d &vMin, cVector3d &v);

    static void updateMaxs(cVector3d &vMax, cVector3d &v);

    static void clearArrays(bool * vtxChkBlock, int * vtxIdxBlock, int blockSize);

    // Method to detect, index and store repeat vertices
    static void computeUniqueVerticesandTriangles(const cMesh* mesh, std::vector<double>* outputVertices, std::vector<uint>* outputTriangles, std::vector<afVertexTree>* a_vertexTrees, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);

    // Method to detect, index and store repeat vertices
    static void computeUniqueVerticesandTrianglesSequential(const cMesh* mesh, std::vector<double>* outputVertices, std::vector<uint>* outputTriangles, std::vector<afVertexTree>* a_vertexTrees, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);
};

///
/// \brief The afSoftBody class
///
class afSoftBody: public afInertialObject{

    friend class afModel;



public:

    afSoftBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    // Execute the commands incomming of afObjectCmd handle
    virtual void afObjectCommandExecute(double){}

    virtual bool createFromAttribs(afSoftBodyAttributes* a_attribs);

    // Add child a softbody
    virtual void addChildBody(afSoftBodyPtr, afJointPtr){}

    virtual void createInertialObject();

    virtual void setLocalTransform(const cTransform &trans);

    virtual void updateSceneObjects();

    bool cleanupMesh(cMultiMesh* multiMesh, std::vector<afVertexTree>& a_afVertexTree, std::vector<unsigned int>& a_triangles);

    bool generateFromMesh(cMultiMesh* mesh, const double margin);

    // Helper Function to Create Links from Lines
    bool createLinksFromLines(btSoftBody* a_sb, std::vector< std::vector<int>>* a_lines, cMesh* a_mesh);

    // Copied from btSoftBodyHelpers with few modifications
    btSoftBody* createFromMesh(btSoftBodyWorldInfo* worldInfo, cMesh* a_mesh, bool randomizeConstraints=true);

    //! This method toggles the drawing of skeletal model.
    void toggleSkeletalModelVisibility();
};


class afGhostObject: public afInertialObject{

    friend class afModel;
    friend class afJoint;
    friend class afWorld;

public:

    afGhostObject(afWorldPtr a_afWorld, afModelPtr a_modelPtr);
    virtual ~afGhostObject();

    // This method updates the AMBF position representation from the Bullet dynamics engine.
    virtual void update(double dt);

    virtual bool createFromAttribs(afGhostObjectAttributes* a_attribs);

    virtual void createInertialObject();

    virtual void setLocalTransform(const cTransform &trans);

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

    virtual void update(double dt);

    afRigidBodyPtr findConnectingBody(string body_name);

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

    virtual void enableVisualization(){}

    // Parent Body for this sensor
    afRigidBodyPtr m_parentBody;

    virtual void update(double dt){}

    afActuatorType m_actuatorType;

protected:

    double m_visibleSize = 0.002;

    bool m_actuate = false;

    bool m_visualizationEnabled = false;
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

    virtual void actuate(string a_softbody_name, btSoftBody::Face* face);

    virtual void actuate(afSoftBodyPtr a_softBody, btSoftBody::Face* face);

    virtual void actuate(afSoftBodyPtr a_softBody, vector<btSoftBody::Node*> nodes);

    // In these actuate methods, explicit body offsets are provided.

    virtual void actuate(string a_rigid_body_name, btTransform a_bodyOffset);

    virtual void actuate(afRigidBodyPtr a_rigidBody, btTransform a_bodyOffset);

    virtual void actuate(string a_softbody_name, btSoftBody::Face* face, btTransform a_bodyOffset);

    virtual void actuate(afSoftBodyPtr a_softBody, btSoftBody::Face* face, btTransform a_bodyOffset);

    virtual void actuate(afSoftBodyPtr a_softBody, vector<btSoftBody::Node*> nodes, btTransform a_bodyOffset);

    virtual void actuate(afSensorPtr a_sensorPtr);

    virtual void enableVisualization();

    void visualize(bool show);

    // Remove the constraint
    virtual void deactuate();

    virtual void update(double dt);

    inline bool isActuated(){return m_active;}

protected:

    double m_maxImpulse = 3.0;
    double m_tau = 0.001;
    btTypedConstraint* m_constraint = nullptr;
    // Transform of actuator w.r.t. parent body
    cTransform m_T_aINp;

    cMesh* m_actuatorVisual = nullptr;


private:
    afRigidBodyPtr m_childRigidBody = nullptr;
    afSoftBodyPtr m_childSoftBody = nullptr;
    int m_softBodyFaceIdx = -1;
    vector<btSoftBody::Node*> m_softBodyNodes;
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

    // Get the body this sensor is a child of
    inline afRigidBodyPtr getParentBody(){return m_parentBody;}

    // Parent Body for this sensor
    afRigidBodyPtr m_parentBody;

    // The type this sensor?
    afSensorType m_sensorType;

    bool m_visualizationEnabled = false;

    // Upate the sensor, usually called at each dynamic tick update of the physics engine
    virtual void update(double dt);

};


///
/// \brief The afRayTracerSensor class
///
class afRayTracerSensor: public afSensor{

    friend class afProximitySensor;
    friend class afResistanceSensor;

public:
    // Constructor
    afRayTracerSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afRayTracerSensorAttributes* a_attribs);

    // Update sensor is called on each update of positions of RBs and SBs
    virtual void update(double dt);

    // Check if the sensor sensed something. Depending on what type of sensor this is
    inline bool isTriggered(uint idx){return m_rayTracerResults[idx].m_triggered;}

    inline double getDepthFraction(uint idx){return m_rayTracerResults[idx].m_depthFraction;}

    // Get the type of sensed body
    inline afBodyType getSensedBodyType(uint idx){return m_rayTracerResults[idx].m_sensedBodyType;}

    // Return the sensed BT RigidBody's Ptr
    inline btRigidBody* getSensedBTRigidBody(uint idx){return m_rayTracerResults[idx].m_sensedBTRigidBody;}

    // Get the sensed Rigid Body's Ptr
    inline afRigidBodyPtr getSensedRigidBody(uint idx){return m_rayTracerResults[idx].m_sensedRigidBody;}

    // Get the sensed BT SoftBody's Ptr
    inline btSoftBody* getSensedBTSoftBody(uint idx){return m_rayTracerResults[idx].m_sensedBTSoftBody;}

    // Get the sensed Soft Body's Ptr
    inline afSoftBodyPtr getSensedSoftBody(uint idx){return m_rayTracerResults[idx].m_sensedSoftBody;}

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

    void visualize(bool show);


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


///
/// \brief This is an implementation of Sleep function that tries to adjust sleep between each cycle to maintain
/// the desired loop frequency. This class has been inspired from ROS Rate Sleep written by Eitan Marder-Eppstein
///
class afRate{
public:
    afRate(int a_freq){
        m_cycle_time = 1.0 / double(a_freq);
        m_rateClock.start();
        m_next_expected_time = m_rateClock.getCurrentTimeSeconds() + m_cycle_time;
    }
    bool sleep(){
        double cur_time = m_rateClock.getCurrentTimeSeconds();
        if (cur_time >= m_next_expected_time){
            m_next_expected_time = cur_time + m_cycle_time;
            return true;
        }
        while(m_rateClock.getCurrentTimeSeconds() <= m_next_expected_time){

        }
        m_next_expected_time = m_rateClock.getCurrentTimeSeconds() + m_cycle_time;
        return true;
    }
private:
    double m_next_expected_time;
    double m_cycle_time;
    cPrecisionClock m_rateClock;
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

    virtual void update(double dt);

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
    float* getData(){return m_data;}

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

    void createFromAttribs(afNoiseModelAttribs* a_attribs);

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


struct afCameraWindowCallBacks{
    afCameraWindowCallBacks(){
        windowSizeCallback = nullptr;
        errorCallback = nullptr;
        keyCallback = nullptr;
        mouseBtnsCallback = nullptr;
        mousePosCallback = nullptr;
        mouseScrollCallback = nullptr;
        dragDropCallback = nullptr;
    }
    // callback when the window display is resized
    void (*windowSizeCallback)(GLFWwindow*, int, int);

    // callback when an error GLFW occurs
    void (*errorCallback)(int, const char*);

    // callback when a key is pressed
    void (*keyCallback)(GLFWwindow*, int, int, int, int);

    //callback for mouse buttons
    void (*mouseBtnsCallback)(GLFWwindow*, int, int, int);

    //callback for mouse positions
    void (*mousePosCallback)(GLFWwindow*, double, double);

    //callback for mouse positions
    void (*mouseScrollCallback)(GLFWwindow*, double, double);

    // Drag and drop callback
    void (*dragDropCallback)(GLFWwindow*, int, const char**);
};


///
/// \brief The afCamera class
///
class afCamera: public afBaseObject{
public:

    afCamera(afWorld* a_afWorld, afModelPtr a_modelPtr);
    ~afCamera();

    virtual void render(afRenderOptions &options);

    virtual void updateGlobalPose(bool a_forceUpdate, cTransform a_parentTransform = cTransform());

    void renderSkyBox();

    void renderFrameBuffer();

    void enableImagePublishing(afImageResolutionAttribs* imageAttribs);

    void enableDepthPublishing(afImageResolutionAttribs* imageAttribs, afNoiseModelAttribs* noiseAtt, afShaderAttributes* depthComputeShaderAttribs);

    void makeWindowFullScreen(bool a_fullscreen);

    void destroyWindow();

    // Define the virtual method for camera
    virtual void update(double dt);

    // Initialize
    bool init();

    void updateLabels(afRenderOptions &options);

    cCamera* getInternalCamera();

    virtual bool createFromAttribs(afCameraAttributes* a_attribs);

    bool createWindow();

    bool assignWindowCallbacks(afCameraWindowCallBacks* a_callbacks);

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
    double getFieldViewAngle() const;

    // Get interval between the scene update and publishing of an image
    inline uint getImagePublishInterval(){return m_imagePublishInterval;}

    // Get interval between the scene update and publishing of the depth
    inline uint getDepthPublishInterval(){return m_depthPublishInterval;}

    // Set interval between the scene update and publishing of an image
    void setImagePublishInterval(uint a_interval);

    // Set interval between the scene update and publishing of the depth
    void setDepthPublishInterval(uint a_interval);

    // This method enables or disables output image mirroring vertically.
    void setWindowMirrorVertical(bool a_enabled);

    void computeDepthOnGPU();

    // Publish Depth as a ROS Topic
    void computeDepthOnCPU();

    // Front plane scene graph which can be used to attach widgets.
    cWorld* getFrontLayer();

    // Front plane scene graph which can be used to attach widgets.
    cWorld *getBackLayer();

    // Is this camera orthographic or not
    inline bool isOrthographic(){return m_orthographic;}

    inline void setOrthographic(bool val){m_orthographic = val;}

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

    cMesh* m_targetVisualMarker = nullptr;

    double getRenderTimeStamp();

    bool overrideRendering(){return m_overrideRenderingFlag;}

    void setOverrideRendering(bool val){m_overrideRenderingFlag = val;}

public:
    bool m_cam_pressed;

    afMouseControlScales m_mouseControlScales;

    GLFWwindow* m_window;

    static GLFWwindow* s_mainWindow;
    GLFWmonitor** m_monitors;
    GLFWmonitor* m_monitor;
    int m_numMonitors;

    cStereoMode m_stereoMode;

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

    std::map<afRigidBodyPtr, cShaderProgramPtr> m_shaderProgramBackup;

    afNoiseModel* getDepthNoiseModel(){return &m_depthNoise;}

    afDepthPointCloud* getDepthPointCloud(){return &m_depthPC;}

protected:
    void createFrameBuffers(afImageResolutionAttribs* imageAttribs);

    void createPreProcessingShaders(afShaderAttributes* preprocessingShaderAttribs);

    void activatePreProcessingShaders();

    void deactivatePreProcessingShaders();

    void preProcessingShadersUpdate();

protected:

    unsigned int m_monitorNumber = 0;

    bool m_frameBuffersCreated = false;

    cVector3d m_pos, m_posClutched;

    cMatrix3d m_rot, m_rotClutched;

    // This is the position that the camera is supposed to be looking at
    // This is also the point along which the orbital/arcball rotation
    // of the camera takes place.
    cVector3d m_targetPos;

    static int s_numWindows;
    static int s_cameraIdx;
    static int s_windowIdx;
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

    afShaderAttributes m_preprocessingShaderAttribs;
    cShaderProgramPtr m_preprocessingShaderProgram;

    double m_renderTimeStamp=0.0;

    // Flag to skip rendering from this camera in the world::render method
    // The current application is for a plugin that wants to take control
    // of rendering from the camera.
    bool m_overrideRenderingFlag;
};


///
/// \brief The afLight struct
///
class afLight: public afBaseObject{
public:
    afLight(afWorld* a_afWorld, afModelPtr a_modelPtr);

    virtual bool createFromAttribs(afLightAttributes* a_attribs);

    virtual void update(double dt);

    inline double getCutOffAngle(){return cDegToRad(m_spotLight->getCutOffAngleDeg());}

    // Set direction of this light
    void setDir(const cVector3d& a_direction);

    void setCutOffAngle(double rad);

    cGenericLight* getInternalLight();

protected:
    cSpotLight* m_spotLight;

};


///
/// \brief The afPointCloud class
///
class afPointCloud: public afBaseObject{
public:
    afPointCloud(afWorldPtr a_afWorld);

    ~afPointCloud();

    cMultiPointPtr m_mpPtr;

    int m_mpSize = 0;

    virtual void update(double dt);

    std::string m_topicName;
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
class afWorld: public afIdentification, public afComm, public afModelManager{

    friend class afModel;

public:
    afWorld();

    virtual ~afWorld();

    virtual bool createFromAttribs(afWorldAttributes* a_attribs);

    virtual bool loadPlugins(afWorldPtr worldPtr, afWorldAttribsPtr attribs, vector<afPluginAttributes>* pluginAttribs);

    virtual void render(afRenderOptions &options);

    cWorld* getChaiWorld();

    afCameraPtr getAssociatedCamera(GLFWwindow* window);

    void makeCameraWindowsFullScreen(bool a_fullscreen);

    void makeCameraWindowsMirrorVertical(bool a_mirrorVertical);

    void destroyCameraWindows();

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

    bool loadCommunicationPlugin(afWorldPtr, afWorldAttribsPtr);

    void resetCameras();

    void resetDynamicBodies();

    void reset();

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
    double getWallTime(){return m_wallClock.getCurrentTimeSeconds();}

    // This method returns the current simulation time
    double getSimulationTime(){return m_simulationTime;}

    double getSystemTime(){return chrono::duration<double>(chrono::system_clock::now().time_since_epoch()).count();}

    // This method gets the time difference between current time and last simulation time
    double getSimulationDeltaTime();

    double computeStepSize(bool adjust_intetration_steps = false);

    void estimateBodyWrenches();

    //! This method updates the simulation over a time interval.
    virtual void updateDynamics(double a_interval, int a_numDevices = 0);

    //! This method updates the position and orientation from Bullet models to CHAI3D models.
    virtual void updateSceneObjects();

    void pluginsGraphicsUpdate();

    void pluginsPhysicsUpdate(double dt);

    void pluginsReset();

    void addSceneObjectToWorld(cGenericObject* a_cObject);

    void removeSceneObjectFromWorld(cGenericObject* a_cObject);

    // This method build the collision graph based on the collision group numbers
    // defined in the bodies
    void buildCollisionGroups();

    bool pickBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    bool movePickedBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);

    void removePickingConstraint();

    virtual void loadShaderProgram();

    void loadSkyBox();

    void runHeadless(bool value);

    bool isHeadless();

    int getPhysicsFrequency(){return m_freqCounterPhysics.getFrequency();}

    int getGraphicsFrequency(){return m_freqCounterGraphics.getFrequency();}

    int getNumDevices(){return m_numDevices;}

    void setResetFlag(){m_resetFlag = true;}

    void clearResetFlag(){m_resetFlag = false;}

    void setResetBodiesFlag(){m_resetBodiesFlag = true;}

    void clearResetBodiesFlag(){m_resetBodiesFlag = false;}

public:

    GLFWwindow* m_mainWindow;

    // The collision groups are sorted by integer indices. A group is an array of
    // rigid bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // a body can be a part of multiple groups
    map<uint, vector<afInertialObjectPtr> > m_collisionGroups;

    afCameraWindowCallBacks m_cameraWindowCallbacks;

    //data for picking objects
    class btRigidBody* m_pickedBulletRigidBody = nullptr;

    afRigidBodyPtr m_pickedRigidBody = nullptr;

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

    cMultiPoint* m_pickMultiPoint = nullptr;

    cPrecisionClock m_wallClock;

    // Vertex Shader Filepath
    afPath m_vsFilePath;

    // Fragment Shader Filepath
    afPath m_fsFilePath;
    //    cMesh* m_pickDragVector;

    afShaderAttributes m_shaderAttribs;

    cShaderProgramPtr m_shaderProgram;

    // Skybox Mesh
    cMesh* m_skyBoxMesh = nullptr;

    afSkyBoxAttributes m_skyBoxAttribs;

    afPath m_world_config_path;

    // a frequency counter to measure the simulation graphic rate
    cFrequencyCounter m_freqCounterGraphics;

    // a frequency counter to measure the simulation haptic rate
    cFrequencyCounter m_freqCounterPhysics;

    map<string, afPointCloudPtr> m_pcMap;

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

    // Current time of simulation.
    double m_simulationTime;

    // Integration time step.
    double m_integrationTimeStep;

    // Last Simulation Timebn
    double m_lastSimulationTime;

    // Maximum number of iterations.
    int m_integrationMaxIterations;

    afWorldPluginManager m_pluginManager;

    int m_numDevices = 0;

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

    cWorld* m_chaiWorld = nullptr;

    bool m_headless = false;

    bool m_resetFlag = false;

    bool m_resetBodiesFlag = false;
};


///
/// \brief The afPickingConstraintData struct
///
struct afPickingConstraintData{

};


///
/// \brief The afModel class
///
class afModel: public afIdentification, public afObjectManager{

    friend class afRigidBody;
    friend class afSoftBody;
    friend class afGhostObject;
    friend class afJoint;

public:

    afModel(afWorldPtr a_afWorld);

    virtual ~afModel();

    virtual bool createFromAttribs(afModelAttributes* a_attribs);

    virtual bool loadPlugins(afModelPtr modePtr, afModelAttribsPtr attribs, vector<afPluginAttributes>* pluginAttribs);

    virtual void update(double dt);

    virtual void reset();

    virtual void updateGlobalPose();

    virtual void updateSceneObjects();

    virtual void loadShaderProgram();

    afWorldPtr getWorldPtr();

    void pluginsGraphicsUpdate();

    void pluginsPhysicsUpdate(double dt);

    void pluginsReset();

    // We can have multiple bodies connected to a single body.
    // There isn't a direct way in bullet to disable collision
    // between all these bodies connected in a tree
    void removeOverlappingCollisionChecking();

    //Remove collision checking for this entire multi-body, mostly for
    // debugging purposes
    void ignoreCollisionChecking();

    // Global Constraint ERP and CFM
    double m_jointERP = 0.1;
    double m_jointCFM = 0.1;

    afShaderAttributes m_shaderAttribs;

    cShaderProgramPtr m_shaderProgram;

    // Plugin Manager
    afModelPluginManager m_pluginManager;

protected:

    cMaterial mat;
    template <typename T>
    string getNonCollidingIdx(string a_body_name, const T* tMap);
    void remapName(string &name, string remap_idx_str);

protected:
    afWorldPtr m_afWorld;
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

    virtual void update(double dt);

    inline int getWheelCount(){return m_numWheels;}

    btRaycastVehicle* getInternalVehicle(){return m_vehicle;}

    afWheelAttributes& getWheelAttribs(int i){return m_wheelAttribs[i];}

    void engageBrake();

    void releaseBrake();

    void setWheelBrake(int i, double p);

    void setWheelPower(int i, double p);

    void setWheelSteering(int i, double s);

    void setChassisForce(btVector3 force);

    void setChassisTorque(btVector3 torque);

protected:
    btDefaultVehicleRaycaster* m_vehicleRayCaster = nullptr;
    btRaycastVehicle* m_vehicle = nullptr;
    btRaycastVehicle::btVehicleTuning m_tuning;
    afRigidBodyPtr m_chassis;
    uint m_numWheels = 0;
    vector<afWheel> m_wheels;
    vector<afWheelAttributes> m_wheelAttribs;
};


class afVolume: public afBaseObject{
public:
    afVolume(afWorldPtr a_afWorld, afModelPtr a_modelPtr);

    ~afVolume();

    virtual bool createFromAttribs(afVolumeAttributes* a_attribs);

    virtual void update(double dt);

    virtual void updateSceneObjects();

    virtual void reset();

    void resetTextures();

    virtual cShaderProgramPtr getShaderProgram();

    virtual void setShaderProgram(cShaderProgramPtr a_program);

    virtual void backupShaderProgram();

    virtual void restoreShaderProgram();

    cVoxelObject* getInternalVolume();

    cVector3d getDimensions();

    cVector3d getVoxelCount();

    cVector3d getResolution();

    // Get the idx of a voxel given a position in local volume coordinates
    bool localPosToVoxelIndex(cVector3d& pos, cVector3d& idx);

    // Get the position in location volume coordinates give a voxel index
    bool voxelIndexToLocalPos(cVector3d& idx, cVector3d& pos);

    // Pos in voxel's local space. NOT IN WORLD SPACE
    bool getVoxelValue(cVector3d& pos, cColorb& color);

    // Set a voxel RGBA value by specifying position in voxel's local space
    bool setVoxelValue(cVector3d& pos, cColorb& val);

    void backupTexture();

    void restoreTexture();

    static cTexture3dPtr copy3DTexture(cTexture1dPtr tex3D);

    void setResetFlag(){m_resetFlag = true;}

    void clearResetFlag(){m_resetFlag = false;}

protected:
    cVoxelObject* m_voxelObject;
    cMultiImagePtr m_multiImage;

private:
    int m_previousRenderingMode=0;
    bool m_prevLinearInterpolationFlag=false;

    cTexture3dPtr m_backupTexture;
    cTexture3dPtr m_originalTextureCopy;

    // The initial values of min and max pos coordiantes of the volume AABB.
    // Storing this as the corners can be changed at runtime to render only
    // a sub block of the volume.
    cVector3d m_minCornerInitial;
    cVector3d m_maxCornerInitial;

    // Should not reset the volume from physics thread, only from graphics thread. This flag is for that purpose.
    bool m_resetFlag;
};


}
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
