//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (www.aimlab.wpi.edu)

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

    \author:    <http://www.aimlab.wpi.edu>
    \author:    <amunawar@wpi.edu>
    \author:    Adnan Munawar
    \courtesy:  Dejaime Antônio de Oliveira Neto at https://www.gamedev.net/profile/187867-dejaime/ for initial direction
    \motivation:https://www.gamedev.net/articles/programming/engines-and-middleware/yaml-basics-and-parsing-with-yaml-cpp-r3508/
    \version:   $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletMultiBody_H
#define CBulletMultiBody_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "afSoftMultiMesh.h"
#include "CBullet.h"
#include "chai3d.h"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/path.hpp>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
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
//------------------------------------------------------------------------------
class afLight;
class afCamera;
typedef afLight* afLightPtr;
typedef afCamera* afCameraPtr;
typedef std::map<std::string, afCameraPtr> afCameraMap;
typedef std::vector<afCameraPtr> afCameraVec;
//------------------------------------------------------------------------------


///
/// \brief cVec2btVec
/// \param cVec
/// \return
///
btVector3 cVec2btVec(const cVector3d &cVec);

///
/// \brief btVec2cVec
/// \param bVec
/// \return
///
cVector3d btVec2cVec(const btVector3 &bVec);

///
/// \brief The afConfigHandler class
///
class afConfigHandler{

public:

    afConfigHandler();
    virtual ~afConfigHandler(){}
    std::string getConfigFile(std::string a_config_name);
    std::string getMultiBodyConfig(int i=0);
    std::string getColorConfig();
    std::string getWorldConfig();
    std::vector<double> getColorRGBA(std::string a_color_name);
    std::string getGripperConfig(std::string a_gripper_name);
    bool loadBaseConfig(std::string file);
    inline int numMultiBodyConfig(){return s_multiBody_configs.size();}

private:

    static boost::filesystem::path s_boostBaseDir;
    static std::string s_color_config;
    static std::vector<std::string> s_multiBody_configs;
    static std::string s_world_config;
    YAML::Node configNode;
    static std::map<std::string, std::string> s_gripperConfigFiles;

protected:

    static YAML::Node s_colorsNode;

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
        m_rolling_friction = 0.1;
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
    invalid= 0, mesh = 1, shape = 2
};

///
/// \brief The afBody class
///
class afRigidBody: public cBulletMultiMesh{

    friend class afMultiBody;
    friend class afJoint;

public:

    afRigidBody(afWorldPtr a_afWorld);
    virtual ~afRigidBody();
    virtual void afObjectCommandExecute(double dt);
    virtual bool loadRidigBody(std::string rb_config_file, std::string node_name, afMultiBodyPtr mB);
    virtual bool loadRidigBody(YAML::Node* rb_node, std::string node_name, afMultiBodyPtr mB);
    virtual void addChildBody(afRigidBodyPtr childBody, afJointPtr jnt);
    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();

    std::vector<afJointPtr> m_joints;
    std::vector<afRigidBodyPtr> m_childrenBodies;
    std::vector<afRigidBodyPtr> m_parentBodies;

    virtual void setAngle(double &angle);
    virtual void setAngle(std::vector<double> &angle);
    static void setConfigProperties(const afRigidBodyPtr a_body, const afRigidBodySurfacePropertiesPtr a_surfaceProps);
    std::string m_body_namespace;
    btVector3 computeInertialOffset(cMesh* mesh);

    //! This method toggles the viewing of frames of this rigid bodyl.
    inline void toggleFrameVisibility(){m_showFrame = !m_showFrame;}

    //! Get Min/Max publishing frequency for afObjectState for this body
    inline int getMinPublishFrequency(){return _min_publish_frequency;}
    inline int getMaxPublishFrequency(){return _max_publish_frequency;}

public:
    // function to check if this rigid body is part of the collision group
    // at a_idx
    bool checkCollisionGroupIdx(int a_idx);
    // function to check if this rigid body is part of the collision group
    // at a_idxs
    bool isCommonCollisionGroupIdx(std::vector<int> a_idx);

public:
    //! If the Position Controller is active, disable Position Controller from Haptic Device
    bool m_af_enable_position_controller;

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name, m_collision_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d m_initialPos;
    cMatrix3d m_initialRot;
    std::vector<afRigidBodyPtr>::const_iterator m_bodyIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    bool _publish_joint_positions = false;
    bool _publish_children_names = false;
    bool _publish_joint_names = true;
    int _min_publish_frequency;
    int _max_publish_frequency;
    void computeControllerGains();

protected:

    void addParentBody(afRigidBodyPtr a_body);
    void upwardTreePopulation(afRigidBodyPtr a_childbody, afJointPtr a_jnt);
    void downwardTreePopulation(afRigidBodyPtr a_parentbody);
    // Update the children for this body in the afObject State Message
    virtual void afObjectStateSetChildrenNames();
    // Update the joints for this body in the afObject State Message
    virtual void afObjectStateSetJointNames();
    // Update the joint positions of children in afObject State Message
    virtual void afObjectSetJointPositions();
    static afRigidBodySurfaceProperties m_surfaceProps;
    static cMaterial m_mat;

protected:
    std::vector<int> m_collisionGroupsIdx;

private:
    afWorldPtr m_afWorld;
    std::vector<float> m_joint_positions;
    afMultiBodyPtr m_mBPtr;
    // Counter for the times we have written to ambf_comm API
    // This is only of internal use as it could be reset
    unsigned short m_write_count = 0;
    double m_P=10;
    double m_I=0;
    double m_D=1;
    GeometryType m_visualGeometryType, m_collisionGeometryType;
};

///
/// \brief The afSoftBody class
///
class afSoftBody: public afSoftMultiMesh{

    friend class afMultiBody;

public:

    afSoftBody(afWorldPtr a_afWorld);
    virtual void afObjectCommandExecute(double dt){}
    virtual bool loadSoftBody(std::string sb_config_file, std::string node_name, afMultiBodyPtr mB);
    virtual bool loadSoftBody(YAML::Node* sb_node, std::string node_name, afMultiBodyPtr mB);
    virtual void addChildBody(afSoftBodyPtr childBody, afJointPtr jnt){}

    std::vector<afJointPtr> m_joints;
    std::vector<afSoftBodyPtr> m_childrenBodies;
    std::vector<afSoftBodyPtr> m_parentBodies;

    void setAngle(double &angle, double dt);
    void setAngle(std::vector<double> &angle, double dt);
    static void setConfigProperties(const afSoftBodyPtr a_body, const afSoftBodyConfigPropertiesPtr a_configProps);

protected:

    double m_scale;
    double m_total_mass;
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

    void addParentBody(afSoftBodyPtr a_body);
    void populateParentsTree(afSoftBodyPtr a_body, afJointPtr a_jnt);
    static afSoftBodyConfigProperties m_configProps;
    static cMaterial m_mat;

protected:
    afWorldPtr m_afWorld;
};


///
/// \brief The PID struct
///
class afController{
public:
    // Set some default values of PID
    // TODO: Maybe set PID's to 0 so the
    // user has to explicitly set them
    double P = 1000;
    double I = 0;
    double D = 50;
    double e[4] = {0, 0, 0, 0};
    double de[4] = {0, 0, 0, 0};
    double dde[4] = {0, 0, 0, 0};
    double t[4]= {0, 0, 0, 0};
    size_t n = 4;
    double output;
    double max_impulse;
    double max_effort;

    // Store the last effort command to compute and bound max impulse
    double m_last_cmd = 0;

    double computeOutput(double process_val, double set_point, double current_time);
    void boundImpulse(double& effort_cmd);
    void boundEffort(double& effort_cmd);
};

enum JointType{
    revolute = 0,
    prismatic = 1,
    fixed = 2
};

///
/// \brief The afJoint class
///
class afJoint{
    friend class afRigidBody;
    friend class afGripperLink;
    friend class afMultiBody;

public:

    afJoint(afWorldPtr a_afWorld);
    virtual ~afJoint();
    virtual bool loadJoint(std::string jnt_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");
    virtual bool loadJoint(YAML::Node* jnt_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");
    void commandEffort(double &effort_cmd);
    void commandPosition(double &position_cmd);
    inline btTypedConstraint* getConstraint(){return m_btConstraint;}
    double getPosition();
    JointType m_jointType;

protected:

    std::string m_name;
    std::string m_parent_name, m_child_name;
    std::string m_joint_name;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    double m_joint_damping;
    double m_max_effort;
    bool m_enable_actuator;
    double m_lower_limit, m_higher_limit;
    double m_joint_offset;
    btRigidBody *m_rbodyA, *m_rbodyB;
    void printVec(std::string name, btVector3* v);
    btQuaternion getRotationBetweenVectors(btVector3 &v1, btVector3 &v2);

    afWorldPtr m_afWorld;

protected:

    btTypedConstraint *m_btConstraint;

private:
    // Add these two pointers for faster access to constraint internals
    // rather than having to cast the m_btConstraint ptr in high speed
    // control loops
    btHingeConstraint* m_hinge;
    btSliderConstraint* m_slider;
    afMultiBodyPtr m_mB;
    afController m_controller;
};

//-----------------------------------------------------------------------------

///
/// \brief The afCamera class
///
class afCamera: public cCamera{
public:

    afCamera(afWorld* a_afWorld);
    bool createDefaultCamera();
    bool loadCamera(YAML::Node* camera_node, std::string camera_name);

    cVector3d measuredPos();
    cMatrix3d measuredRot();
    inline cVector3d getTargetPos(){return m_targetPos;}
    inline void setTargetPos(cVector3d a_pos){m_targetPos = a_pos;}

    bool init();

public:
    bool m_cam_pressed;
    GLFWwindow* m_window;

    static GLFWwindow* s_mainWindow;
    static GLFWmonitor** s_monitors;
    GLFWmonitor* m_monitor;
    static int s_numMonitors;

public:
    // Labels
    cLabel* m_graphicsDynamicsFreqLabel;
    cLabel* m_wallSimTimeLabel;
    cLabel* m_devicesModesLabel;
    cLabel* m_deviceButtonLabel;
    cLabel* m_controllingDeviceLabel;
    std::vector<cLabel*> m_devHapticFreqLabels;

public:
    // Position of mouse's x,y and scrolls cur and last coordinates for contextual window
    double mouse_x[2], mouse_y[2], mouse_scroll[2];
    bool mouse_l_clicked = false, mouse_r_clicked= false, mouse_scroll_clicked = false;
    bool mouse_r_btn_rising_edge = false, mouse_l_btn_rising_edge = false;


    cMatrix3d camRot, camRotPre;

    // Window parameters
    int m_width, m_height;
    int m_win_x, m_win_y;

public:
    std::vector<std::string> m_controllingDevNames;

protected:
    std::mutex m_mutex;
    cVector3d m_pos, m_posClutched;
    cMatrix3d m_rot, m_rotClutched;
    // This is the position that the camera is supposed to be looking at
    // This is also the point along which the orbital/arcball rotation
    // of the camera takes place.
    cVector3d m_targetPos;

protected:
    static int s_numWindows;
    static int s_cameraIdx;
    static int s_windowIdx;

private:
    afWorldPtr m_afWorld;
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
class afLight{
public:
    afLight(afWorld* a_afWorld);
    bool loadLight(YAML::Node* light_node, std::string light_name);
    bool createDefaultLight();

    std::string m_name;

protected:
    cSpotLight* m_spotLight;
private:
    afWorldPtr m_afWorld;
};


//-----------------------------------------------------------------------------

///
/// \brief The afWorld class
///
class afWorld: public afConfigHandler{

    friend class afMultiBody;

public:
    afWorld(cBulletWorld *bulletWorld);
    virtual ~afWorld(){}
    virtual bool loadWorld(std::string a_world_config = "");
    bool createDefaultWorld();
    double getEnclosureLength();
    double getEnclosureWidth();
    double getEnclosureHeight();
    void getEnclosureExtents(double &length, double &width, double &height);
    afCameraVec getCameras(){return m_cameras;}
    static cBulletWorld *s_bulletWorld;
    std::vector<afLightPtr> m_lights;
    std::vector<afCameraPtr> m_cameras;

    GLFWwindow* m_mainWindow;

    afCameraMap m_cameraMap;


public:

    bool addRigidBody(std::string a_name, afRigidBodyPtr a_rb);
    bool addSoftBody(std::string a_name, afSoftBodyPtr a_sb);
    bool addJoint(std::string a_name, afJointPtr a_jnt);

    afRigidBodyPtr getRidigBody(std::string a_name, bool suppress_warning=false);
    afRigidBodyPtr getRootRigidBody(afRigidBodyPtr a_bodyPtr = NULL);
    afSoftBodyPtr getSoftBody(std::string a_name);
    inline afSoftBodyMap* getSoftBodyMap(){return &m_afSoftBodyMap;}
    inline afRigidBodyMap* getRigidBodyMap(){return &m_afRigidBodyMap;}
    inline afJointMap* getJointMap(){return &m_afJointMap;}

protected:

    afRigidBodyMap m_afRigidBodyMap;
    afSoftBodyMap m_afSoftBodyMap;
    afJointMap m_afJointMap;

protected:

    afWorld(){}

private:

    static double m_encl_length;
    static double m_encl_width;
    static double m_encl_height;

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
    virtual bool loadMultiBody(int i);
    virtual bool loadMultiBody(std::string a_multibody_config);
    void loadAllMultiBodies();

    inline std::string getHighResMeshesPath(){return m_multibody_high_res_meshes_path;}
    inline std::string getLowResMeshesPath(){return m_multibody_low_res_meshes_path;}
    inline std::string getMultiBodyPath(){return m_multibody_path;}
    inline std::string getNameSpace(){return m_multibody_namespace;}

    // We can have multiple bodies connected to a single body.
    // There isn't a direct way in bullet to disable collision
    // between all these bodies connected in a tree
    void removeOverlappingCollisionChecking();

    // This method build the collision graph based on the collision group numbers
    // defined in the bodies
    void buildCollisionGroups();

    //Remove collision checking for this entire multi-body, mostly for
    // debugging purposes
    void ignoreCollisionChecking();

    bool pickBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);
    bool movePickedBody(const cVector3d& rayFromWorld, const cVector3d& rayToWorld);
    void removePickingConstraint();

    cPrecisionClock m_wallClock;

protected:

    afWorldPtr m_afWorld;

    std::string m_multibody_high_res_meshes_path, m_multibody_low_res_meshes_path;
    std::string m_multibody_namespace;
    std::string m_multibody_path;

protected:

    cMaterial mat;
    template <typename T>
    std::string remapBodyName(std::string a_body_name, const T* tMap);
    std::string remapJointName(std::string a_joint_name);
    void remapName(std::string &name, std::string remap_idx_str);

protected:
    // The collision groups are sorted by integer indices. A group is an array of
    // rigid bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // the a body can be a part of multiple groups
    std::map<int, std::vector<afRigidBodyPtr> > m_collisionGroups;

private:
    //data for picking objects
    class btRigidBody* m_pickedBody=0;
    class btTypedConstraint* m_pickedConstraint=0;
    int m_savedState;
    cVector3d m_oldPickingPos;
    cVector3d m_hitPos;
    double m_oldPickingDist;
    cMesh* m_pickSphere;

//    cMesh* m_pickDragVector;
};

}
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
