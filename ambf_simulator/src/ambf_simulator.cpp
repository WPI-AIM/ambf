//===========================================================================
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
    \courtesy:  Starting point CHAI3D-BULLET examples by Francois Conti from <www.chai3d.org>
    \version:   $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// GENERAL SETTINGS
//---------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// BULLET MODULE VARIABLES
//---------------------------------------------------------------------------

// bullet world
cBulletWorld* g_bulletWorld;

afMultiBody *g_afMultiBody;
afWorld *g_afWorld;

double g_dt_fixed = 0;
bool g_force_enable = true;
// Default switch index for clutches

std::string g_btn_action_str = "";
bool g_cam_btn_pressed = false;
bool g_clutch_btn_pressed = false;
cPrecisionClock g_clockWorld;

// Info for mouse events in case a body is picked
bool g_pickBody = false;
cVector3d g_pickFrom, g_pickTo;

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool g_simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool g_simulationFinished = true;

// Flag to check if any window is closed by the user
bool g_window_closed = false;

// Flag to toggle between inverted/non_inverted mouse pitch with mouse
bool g_mouse_inverted_y = false;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter g_freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter g_freqCounterHaptics;

// haptic thread
std::vector<cThread*> g_hapticsThreads;
// bullet simulation thread
cThread* g_bulletSimThread;

// swap interval for the display context (vertical synchronization)
int g_swapInterval = 0;

bool g_mousePickingEnabled = false;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
class PhysicalInputDevice;
class SimulatedInputDevice;
struct PhySimDevicePair;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

//callback for mouse buttons
void mouseBtnsCallback(GLFWwindow* a_window, int a_button, int a_action, int a_modes);

//callback for mouse positions
void mousePosCallback(GLFWwindow* a_window, double x_pos, double y_pos);

//callback for mouse positions
void mouseScrollCallback(GLFWwindow* a_window, double x_pos, double y_pos);

// Copied from CommonRidiBodyBase.h of Bullet Physics by Erwin Coumans with
// Ray Tracing for Camera Pick and Place
cVector3d getRayTo(int x, int y, afCameraPtr a_camera);

// this function contains the main haptics simulation loop
void updateHapticDevice(void*);

//this function contains the main Bullet Simulation loop
void updatePhysics(void);

// this function closes the application
void close(void);

const int MAX_DEVICES = 10;

// Vector of WindowCamera Handles Struct
std::vector<afCamera*> g_cameras;

// Global iterator for WindowsCamera Handle
std::vector<afCamera*>::iterator g_cameraIt;

// this function renders the scene
void updateGraphics();

// Function to update labels
void updateLabels();

// Bullet pretick callback
void preTickCallBack(btDynamicsWorld* world, btScalar timeStep);

///
/// \brief The SoftBodyGrippingConstraint struct
///
struct SoftBodyGrippingConstraint{
    btRigidBody* m_rBody; // Ptr to the RigidBody that is going to be constrained to the softbody
    btSoftBody* m_sBody; // Ptr to SoftBody that the constraint is part of
    std::vector<btSoftBody::Node*> m_nodePtrs; // Node Pointers that this constraint is holding on to
};

///
/// \brief This class encapsulates each haptic device in isolation and provides methods to get/set device
/// state/commands, button's state and grippers state if present
///
class PhysicalInputDevice{
public:
    PhysicalInputDevice(){}
    ~PhysicalInputDevice();
    virtual bool loadPhysicalDevice(std::string pd_config_file, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice);
    virtual bool loadPhysicalDevice(YAML::Node* pd_node, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice);
    cVector3d measuredPos();
    cMatrix3d measuredRot();
    cVector3d measuredPosPreclutch();
    cMatrix3d measuredRotPreclutch();
    void setPosPreclutch(cVector3d a_pos);
    void setRotPreclutch(cMatrix3d a_rot);
    cVector3d measuredPosCamPreclutch();
    cMatrix3d measuredRotCamPreclutch();
    void setPosCamPreclutch(cVector3d a_pos);
    void setRotCamPreclutch(cMatrix3d a_rot);
    cVector3d measuredVelLin();
    cVector3d mearuredVelAng();
    double measuredGripperAngle();
    void applyWrench(cVector3d a_force, cVector3d a_torque);
    bool isButtonPressed(int button_index);
    bool isButtonPressRisingEdge(int button_index);
    bool isButtonPressFallingEdge(int button_index);
    void enableForceFeedback(bool enable){m_dev_force_enabled = enable;}

public:
    cShapeSphere* createCursor(cBulletWorld* a_world);
    cBulletSphere* createAfCursor(cBulletWorld* a_world, std::string a_name);
    cGenericHapticDevicePtr m_hDevice;
    cHapticDeviceInfo m_hInfo;
    cVector3d m_pos, m_posClutched, m_posPreClutch;
    cMatrix3d m_rot, m_rotClutched, m_rotPreClutch;
    cVector3d m_posCamPreClutch;
    cMatrix3d m_rotCamPreClutch;
    cVector3d m_vel, m_avel;
    double m_workspace_scale_factor;
    cShapeSphere* m_cursor = NULL;
    cBulletSphere* m_af_cursor = NULL;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

private:
    std::mutex m_mutex;
    void updateCursorPose();
    bool m_dev_force_enabled = true;
};

///
/// \brief PhysicalDevice::create_cursor
/// \param a_world
/// \return
///
cShapeSphere* PhysicalInputDevice::createCursor(cBulletWorld* a_world){
    m_cursor = new cShapeSphere(0.05);
    m_cursor->setShowEnabled(true);
    m_cursor->setShowFrame(true);
    m_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_cursor->setMaterial(mat);
    a_world->addChild(m_cursor);
    return m_cursor;
}


///
/// \brief PhysicalDevice::~PhysicalDevice
///
PhysicalInputDevice::~PhysicalInputDevice(){
}

///
/// \brief PhysicalDevice::loadPhysicalDevice
/// \param pd_node
/// \param node_name
/// \param mB
/// \param name_remapping_idx
/// \return
///
bool PhysicalInputDevice::loadPhysicalDevice(string pd_config_file, string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice){
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(pd_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO PHYSICAL DEVICE CONFIG: " << pd_config_file << std::endl;
        return 0;
    }
    if (baseNode.IsNull()) return false;

    YAML::Node basePDNode = baseNode[node_name];
    return loadPhysicalDevice(&basePDNode, node_name, hDevHandler, simDevice);
}

///
/// \brief PhysicalDevice::loadPhysicalDevice
/// \param pd_node
/// \param node_name
/// \param mB
/// \param name_remapping_idx
/// \return
///
bool PhysicalInputDevice::loadPhysicalDevice(YAML::Node *pd_node, string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice){
    YAML::Node physicaDeviceNode = *pd_node;
    if (physicaDeviceNode.IsNull()){
        std::cerr << "ERROR: PHYSICAL DEVICE'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    YAML::Node pDHardwareName = physicaDeviceNode["hardware name"];
    YAML::Node pDHapticGain = physicaDeviceNode["haptic gain"];
    YAML::Node pDWorkspaceScaling = physicaDeviceNode["workspace scaling"];
    YAML::Node pDSimulatedGripper = physicaDeviceNode["simulated multibody"];
    YAML::Node pDRootLink = physicaDeviceNode["root link"];
    YAML::Node pDLocation = physicaDeviceNode["location"];

    std::string _hardwareName = "";
    double _KLgain = 0;
    double _KAgain = 0;
    double _workspaceScale = 10;

    // For the simulated gripper, the user can specify a MultiBody config to load.
    // We shall load this file as a proxy for Physical Input device in the simulation.
    // We shall get the root link of this multibody (baselink) and set Cartesian Position
    // control on this body.

    // Further, the user can sepcify a root link for the MultiBody config file. If this
    // is defined we shall infact use the specific link which can be different from
    // the bodies base link.

    // A second use case arises, in which the user doesnt want to provide a config file
    // but want to bind the physical input device to an existing multibody in the simulation.
    // In this case, the user should specify the root link only and we shall try to find a
    // body in simulation by that name. Following on, the use shall then be able to control
    // that link in Position control and control all the joints lower in heirarchy.

    bool _simulatedMBDefined = false;
    bool _rootLinkDefined = false;
    std::string _simulatedMBFileName = "";
    std::string _rootLinkName = "";
    cVector3d _locationPos, _locationRot;

    if (pDHardwareName.IsDefined()){
        _hardwareName = pDHardwareName.as<std::string>();
    }
    else{
        std::cerr << "ERROR: PHYSICAL DEVICE : \"" << node_name << "\" HARDWARE NAME NOT DEFINED, IGNORING \n";
        return 0;
    }

    if (pDHapticGain.IsDefined()){
        _KLgain = pDHapticGain["linear"].as<double>();
        _KAgain = pDHapticGain["angular"].as<double>();
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" HAPTIC GAINES NOT DEFINED \n";
    }

    if (pDWorkspaceScaling.IsDefined()){
        _workspaceScale = pDWorkspaceScaling.as<double>();
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" WORKSPACE SCALE NOT DEFINED \n";
    }

    if (pDSimulatedGripper.IsDefined()){
        _simulatedMBFileName = pDSimulatedGripper.as<std::string>();
        _simulatedMBDefined = true;
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" SIMULATED GRIPPER FILENAME NOT DEFINED \n";
    }

    if (pDRootLink.IsDefined()){
        _rootLinkName = pDRootLink.as<std::string>();
        _rootLinkDefined = true;
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" ROOT LINK NAME NOT DEFINED \n";
    }

    if (pDLocation.IsDefined()){
        assignXYZ(&pDLocation, &_locationPos);
        assignRPY(&pDLocation, &_locationRot);
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" LOCATION NOT DEFINED \n";
    }

    if (!_rootLinkDefined && !_simulatedMBDefined){
        std::cerr << "ERROR: PHYSICAL DEVICE : \"" << node_name << "\" REQUIRES EITHER A \"simulated multibody\""
                                                                   "or a \"root link\" TO DISPLAY A PROXY IN SIMULATION \n";
        return 0;
    }

    int nDevs = hDevHandler->getNumDevices();

    for (int dIdx = 0 ; dIdx < nDevs ; dIdx++){
        hDevHandler->getDeviceSpecifications(m_hInfo, dIdx);

        if (m_hInfo.m_modelName.compare(_hardwareName) == 0){
            // This is our device. Let's load it up
            hDevHandler->getDevice(m_hDevice, dIdx);
            break;
        }
    }

    if (_simulatedMBDefined){
        if (!simDevice->loadSimulatedGripper(_simulatedMBFileName, m_hInfo.m_modelName)){
            return 0;
        }
    }

    if (_rootLinkDefined){
        if (simDevice->getAFRigidBody(_rootLinkName, false)){
            simDevice->m_rootLink = simDevice->getAFRigidBody(_rootLinkName);
        }
        else{
            simDevice->m_rootLink = simDevice->getAFRootRigidBody();
        }
    }



}

///
/// \brief PhysicalDevice::create_af_cursor
/// \param a_world
/// \param a_name
/// \return
///
cBulletSphere* PhysicalInputDevice::createAfCursor(cBulletWorld *a_world, string a_name){
    m_af_cursor = new cBulletSphere(a_world, 0.05, a_name);
    m_af_cursor->setShowEnabled(true);
    m_af_cursor->setShowFrame(true);
    m_af_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_af_cursor->setMaterial(mat);
    a_world->addChild(m_af_cursor);
    return m_af_cursor;
}

///
/// \brief PhysicalDevice::measured_pos
/// \return
///
cVector3d PhysicalInputDevice::measuredPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getPosition(m_pos);
    updateCursorPose();
    return m_pos;
}

///
/// \brief PhysicalDevice::measured_pos_last
/// \return
///
cVector3d PhysicalInputDevice::measuredPosPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posPreClutch;
}

///
/// \brief PhysicalDevice::set_pos_preclutch
/// \param a_pos
///
void PhysicalInputDevice::setPosPreclutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posPreClutch = a_pos;
}

///
/// \brief PhysicalDevice::measured_rot
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getRotation(m_rot);
    return m_rot;
}

///
/// \brief PhysicalDevice::measured_rot_last
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRotPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotPreClutch;
}

///
/// \brief PhysicalDevice::set_rot_preclutch
/// \param a_rot
///
void PhysicalInputDevice::setRotPreclutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotPreClutch = a_rot;
}


///
/// \brief PhysicalDevice::measuredPosCamPreclutch
/// \return
///
cVector3d PhysicalInputDevice::measuredPosCamPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posCamPreClutch;
}

///
/// \brief PhysicalDevice::setPosCamPreclutch
/// \param a_pos
///
void PhysicalInputDevice::setPosCamPreclutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posCamPreClutch = a_pos;
}

///
/// \brief PhysicalDevice::measuredRotCamPreclutch
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRotCamPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotCamPreClutch;
}

///
/// \brief PhysicalDevice::setRotCamPreclutch
/// \param a_rot
///
void PhysicalInputDevice::setRotCamPreclutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotCamPreClutch = a_rot;
}

///
/// \brief PhysicalDevice::update_cursor_pose
///
void PhysicalInputDevice::updateCursorPose(){
    if(m_cursor){
        m_cursor->setLocalPos(m_pos * m_workspace_scale_factor);
        m_cursor->setLocalRot(m_rot);
    }
    if(m_af_cursor){
        m_af_cursor->setLocalPos(m_pos * m_workspace_scale_factor);
        m_af_cursor->setLocalRot(m_rot);
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_af_cursor->m_afObjectPtr->set_userdata_desc("haptics frequency");
        m_af_cursor->m_afObjectPtr->set_userdata(m_freq_ctr.getFrequency());
#endif
    }
}

///
/// \brief PhysicalDevice::measured_lin_vel
/// \return
///
cVector3d PhysicalInputDevice::measuredVelLin(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getLinearVelocity(m_vel);
    return m_vel;
}

///
/// \brief PhysicalDevice::mearured_ang_vel
/// \return
///
cVector3d PhysicalInputDevice::mearuredVelAng(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getAngularVelocity(m_avel);
    return m_avel;
}

///
/// \brief PhysicalDevice::measured_gripper_angle
/// \return
///
double PhysicalInputDevice::measuredGripperAngle(){
    std::lock_guard<std::mutex> lock(m_mutex);
    double angle;
    m_hDevice->getGripperAngleRad(angle);
    return angle;
}

///
/// \brief PhysicalDevice::is_button_pressed
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressed(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    return status;
}

///
/// \brief PhysicalDevice::is_button_press_rising_edge
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressRisingEdge(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_rising[button_index] ^ status){
        if (!m_btn_prev_state_rising[button_index]){
            m_btn_prev_state_rising[button_index] = true;
            return true;
        }
        else{
            m_btn_prev_state_rising[button_index] = false;
        }
    }
    return false;
}

///
/// \brief PhysicalDevice::is_button_press_falling_edge
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressFallingEdge(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_falling[button_index] ^ status){
        if (m_btn_prev_state_falling[button_index]){
            m_btn_prev_state_falling[button_index] = false;
            return true;
        }
        else{
            m_btn_prev_state_falling[button_index] = true;
        }
    }
    return false;
}

///
/// \brief PhysicalDevice::apply_wrench
/// \param force
/// \param torque
///
void PhysicalInputDevice::applyWrench(cVector3d force, cVector3d torque){
    std::lock_guard<std::mutex> lock(m_mutex);
    force = force * m_dev_force_enabled;
    torque = torque * m_dev_force_enabled;
    m_hDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
}

///
/// \brief This class encapsulates Simulation Parameters that deal with the interaction between a single haptic device
/// and the related Gripper simulated in Bullet. These Parameters include mapping the device buttons to
/// action/mode buttons, capturing button triggers in addition to presses, mapping the workspace scale factors
/// for a device and so on.
///
class SimulatedDeviceParams{
public:
    SimulatedDeviceParams();
    void setParams(cHapticDeviceInfo &a_hInfo, PhysicalInputDevice* a_dev);
    inline double getWorkspaceScaleFactor(){return m_workspaceScaleFactor;}

public:
    cVector3d m_posRef, m_posRefOrigin;
    cMatrix3d m_rotRef, m_rotRefOrigin;
    double m_workspaceScaleFactor;
    double K_lh;                    //Linear Haptic Stiffness Gain
    double K_ah;                    //Angular Haptic Stiffness Gain
    // Gain Ramps are used to softly get towards the setpoint when the simulation starts
    double K_lh_ramp;               //Linear Haptic Stiffness Gain Ramp
    double K_ah_ramp;               //Angular Haptic Stiffness Gain Ramp
    double P_lc_ramp;               //Linear Haptic Propotional Gain Ramp
    double P_ac_ramp;               //Angular Haptic Propotional Gain Ramp

    int act_1_btn;
    int act_2_btn;
    int mode_next_btn;
    int mode_prev_btn;
    int m_gripper_pinch_btn = -1;
    bool btn_cam_rising_edge;
    bool btn_clutch_rising_edge;
    bool m_loop_exec_flag;
};


///
/// \brief SimulationParams::SimulationParams
///
SimulatedDeviceParams::SimulatedDeviceParams(){
    m_workspaceScaleFactor = 30.0;
    K_lh = 0.02;
    K_ah = 0.03;
    K_lh_ramp = 0.0;
    K_ah_ramp = 0.0;
    P_lc_ramp = 0.0;
    P_ac_ramp = 0.0;
    act_1_btn   = 0;
    act_2_btn   = 1;
    mode_next_btn = 2;
    mode_prev_btn= 3;

    m_posRef.set(0,0,0);
    m_posRefOrigin.set(0, 0, 0);
    m_rotRef.identity();
    m_rotRefOrigin.identity();

    btn_cam_rising_edge = false;
    btn_clutch_rising_edge = false;
    m_loop_exec_flag = false;
}

///
/// \brief SimulationParams::set_sim_params
/// \param a_hInfo
/// \param a_dev
///
void SimulatedDeviceParams::setParams(cHapticDeviceInfo &a_hInfo, PhysicalInputDevice* a_dev){
    double maxStiffness	= a_hInfo.m_maxLinearStiffness / m_workspaceScaleFactor;

    // clamp the force output gain to the max device stiffness
    K_lh = cMin(K_lh, maxStiffness);
    if (strcmp(a_hInfo.m_modelName.c_str(), "MTM-R") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTMR") == 0 ||
            strcmp(a_hInfo.m_modelName.c_str(), "MTM-L") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTML") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        m_workspaceScaleFactor = 10.0;
        K_lh = K_lh/3;
        act_1_btn     =  1;
        act_2_btn     =  2;
        mode_next_btn =  3;
        mode_prev_btn =  4;
        K_lh = 0.04;
        K_ah = 0.0;
        m_gripper_pinch_btn = 0;
        a_dev->enableForceFeedback(false);
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "Falcon") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        act_1_btn     = 0;
        act_2_btn     = 2;
        mode_next_btn = 3;
        mode_prev_btn = 1;
        K_lh = 0.05;
        K_ah = 0.0;
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "PHANTOM Omni") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        K_lh = 0.01;
        K_ah = 0.0;
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "Razer Hydra") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        m_workspaceScaleFactor = 10.0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief The SimulatedGripper class
///
class SimulatedInputDevice: public SimulatedDeviceParams, public afGripper{
public:
    SimulatedInputDevice(afWorldPtr a_afWorld);
    ~SimulatedInputDevice(){}
    bool loadSimulatedGripper(std::string a_gripper_name, std::string a_device_name);
    cVector3d measuredPos();
    cMatrix3d measuredRot();
    void updateMeasuredPose();
    inline void applyForce(cVector3d force){if (!m_rootLink->m_af_enable_position_controller) m_rootLink->addExternalForce(force);}
    inline void applyTorque(cVector3d torque){if (!m_rootLink->m_af_enable_position_controller) m_rootLink->addExternalTorque(torque);}
    bool isWrenchSet();
    void clearWrench();
    void offsetGripperAngle(double offset);
    void setGripperAngle(double angle, double dt=0.001);

public:
    cVector3d m_pos;
    cMatrix3d m_rot;
    double m_gripper_angle;
    // Gripping constraints, the number is the same as the total
    // number of sensors attached to the gripper. Not all sensors
    // are proximity sensors and necessary for gripping.
    std::vector<btPoint2PointConstraint*> m_rigidGrippingConstraints;

    // We need different constraint for softBody picking
    // Softbody achors seems a good fit for now. This map stores
    // a pair
    std::vector<SoftBodyGrippingConstraint*> m_softGrippingConstraints;

private:
    std::mutex m_mutex;
};

///
/// \brief SimulatedGripper::SimulatedGripper
/// \param a_chaiWorld
///
SimulatedInputDevice::SimulatedInputDevice(afWorldPtr a_afWorld): afGripper (a_afWorld){
    m_gripper_angle = 0.5;
}

///
/// \brief SimulatedGripper::loadFromAMBF
/// \param a_gripper_name
/// \param a_device_name
/// \return
///
bool SimulatedInputDevice::loadSimulatedGripper(std::string a_gripper_name, std::string a_device_name){
    std::string config = m_afWorld->getGripperConfig(a_device_name);
    bool res = loadMultiBody(config, a_gripper_name, a_device_name);
    m_rootLink = getAFRootRigidBody();
    m_rigidGrippingConstraints.resize(m_rootLink->getSensors().size());
    m_softGrippingConstraints.resize(m_rootLink->getSensors().size());
    // Initialize all the constraint to null ptr
    for (int sIdx = 0 ; sIdx < m_rigidGrippingConstraints.size() ; sIdx++){
        m_rigidGrippingConstraints[sIdx] = 0;
        m_softGrippingConstraints[sIdx] = 0;
    }
    return res;
}

///
/// \brief SimulatedGripper::measured_pos
/// \return
///
cVector3d SimulatedInputDevice::measuredPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalPos();
}

///
/// \brief SimulatedGripper::measured_rot
/// \return
///
cMatrix3d SimulatedInputDevice::measuredRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalRot();
}

///
/// \brief SimulatedGripper::update_measured_pose
///
void SimulatedInputDevice::updateMeasuredPose(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pos  = m_rootLink->getLocalPos();
    m_rot = m_rootLink->getLocalRot();
}

///
/// \brief SimulatedGripper::setGripperAngle
/// \param angle
/// \param dt
///
void SimulatedInputDevice::setGripperAngle(double angle, double dt){
    m_rootLink->setAngle(angle, dt);
}

///
/// \brief SimulatedGripper::offset_gripper_angle
/// \param offset
///
void SimulatedInputDevice::offsetGripperAngle(double offset){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gripper_angle += offset;
    m_gripper_angle = cClamp(m_gripper_angle, 0.0, 1.0);
}

///
/// \brief SimulatedGripper::is_wrench_set
/// \return
///
bool SimulatedInputDevice::isWrenchSet(){
    btVector3 f = m_rootLink->m_bulletRigidBody->getTotalForce();
    btVector3 n = m_rootLink->m_bulletRigidBody->getTotalTorque();
    if (f.isZero()) return false;
    else return true;
}

///
/// \brief SimulatedGripper::clear_wrench
///
void SimulatedInputDevice::clearWrench(){
    m_rootLink->m_bulletRigidBody->clearForces();
}

///
/// \brief These are the currently availble modes for each device
///
enum MODES{ CAM_CLUTCH_CONTROL,
            GRIPPER_JAW_CONTROL,
            CHANGE_CONT_LIN_GAIN,
            CHANGE_CONT_ANG_GAIN,
            CHANGE_CONT_LIN_DAMP,
            CHANGE_CONT_ANG_DAMP,
            CHANGE_DEV_LIN_GAIN,
            CHANGE_DEV_ANG_GAIN
          };

///
/// \brief The DeviceGripperPair struct
///
struct PhySimDevicePair{
    PhysicalInputDevice* m_physicalDevice = NULL;
    SimulatedInputDevice* m_simulatedDevice = NULL;
    // The cameras that this particular device Gripper Pair control
    std::vector<afCameraPtr> m_cameras;

    // Label handed by the camera for updating info
    cLabel* m_devFreqLabel = NULL;

    std::string m_name;
};

///
/// \brief This is a higher level class that queries the number of haptics devices available on the sytem
/// and on the Network for dVRK devices and creates a single Bullet Gripper and a Device Handle for
/// each device.
///
class InputDevices{
public:
    InputDevices(afWorldPtr a_afWorld);
    ~InputDevices();

    virtual bool loadInputDevices(std::string a_inputdevice_config, int a_max_load_devs = MAX_DEVICES);

    SimulatedInputDevice* createSimulatedGripper(uint dev_num, PhysicalInputDevice* a_physicalDevice);
    void closeDevices();

    // Increment gains (haptic mean physical device and controller means simulated gripper)
    double increment_K_lh(double a_offset); // Stifness linear haptic
    double increment_K_ah(double a_offset); // Stifness angular haptic
    double increment_P_lc(double a_offset); // Stifness linear controller
    double increment_P_ac(double a_offset); // Stifness angular controller
    double increment_D_lc(double a_offset); // Damping linear controller
    double increment_D_ac(double a_offset); // Damping angular controller

    void nextMode();
    void prevMode();

    std::vector<PhySimDevicePair*> getDeviceGripperPairs(std::vector<std::string> a_device_names);
    std::vector<PhySimDevicePair*> getAllDeviceGripperPairs();

public:
    std::shared_ptr<cHapticDeviceHandler> m_deviceHandler;
    std::vector<PhySimDevicePair> m_psDevicePairs;

    uint m_numDevices;

    // bool to enable the rotation of simulated gripper be in camera frame. i.e. Orienting the camera
    // re-orients the simulate gripper.
    bool m_use_cam_frame_rot;
    MODES m_simModes;
    std::string m_mode_str;
    std::vector<MODES> m_modes_enum_vec {MODES::CAM_CLUTCH_CONTROL,
                MODES::GRIPPER_JAW_CONTROL,
                MODES::CHANGE_CONT_LIN_GAIN,
                MODES::CHANGE_CONT_ANG_GAIN,
                MODES::CHANGE_CONT_LIN_DAMP,
                MODES::CHANGE_CONT_ANG_DAMP,
                MODES::CHANGE_DEV_LIN_GAIN,
                MODES::CHANGE_DEV_ANG_GAIN};

    std::vector<std::string> m_modes_enum_str {"CAM_CLUTCH_CONTROL  ",
                                               "GRIPPER_JAW_CONTROL ",
                                               "CHANGE_CONT_LIN_GAIN",
                                               "CHANGE_CONT_ANG_GAIN",
                                               "CHANGE_CONT_LIN_DAMP",
                                               "CHANGE_CONT_ANG_DAMP",
                                               "CHANGE_DEV_LIN_GAIN ",
                                               "CHANGE_DEV_ANG_GAIN "};
    int m_mode_idx;

private:
    afWorldPtr m_afWorld;
};

///
/// \brief InputDevices::InputDevices
/// \param a_bullet_world
/// \param a_max_load_devs
///
InputDevices::InputDevices(afWorldPtr a_afWorld){
    m_deviceHandler = NULL;
    m_afWorld = a_afWorld;

    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

///
/// \brief InputDevices::~InputDevices
///
InputDevices::~InputDevices(){
}


bool InputDevices::loadInputDevices(std::string a_input_devices_config, int a_max_load_devs){
    if (a_input_devices_config.empty()){
        a_input_devices_config = m_afWorld->getInputDevicesConfig();
    }
    YAML::Node inputDevicesNode;
    try{
        inputDevicesNode = YAML::LoadFile(a_input_devices_config);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << a_input_devices_config << std::endl;
        return 0;
    }

    YAML::Node inputDevices = inputDevicesNode["input devices"];

    if (!inputDevices.IsDefined()){
        return 0;
    }

    if (a_max_load_devs > 0){
        m_deviceHandler.reset(new cHapticDeviceHandler());
        a_max_load_devs = a_max_load_devs < inputDevices.size() ? a_max_load_devs : inputDevices.size();
//        std::cerr << "Num of devices " << m_num_devices << std::endl;
        for (uint devIdx = 0; devIdx < a_max_load_devs; devIdx++){

            PhysicalInputDevice* pD = new PhysicalInputDevice();
            SimulatedInputDevice* sD = new SimulatedInputDevice(m_afWorld);

            std::string devKey = inputDevices[devIdx].as<std::string>();
            YAML::Node devNode = inputDevicesNode[devKey];

            if (pD->loadPhysicalDevice(&devNode, devKey, m_deviceHandler.get(), sD)){
                    PhySimDevicePair dgPair;
                    dgPair.m_physicalDevice = pD;
                    dgPair.m_simulatedDevice = sD;
                    dgPair.m_name = pD->m_hInfo.m_modelName;
                    m_psDevicePairs.push_back(dgPair);
                }
            }
        m_numDevices = m_psDevicePairs.size();
        }
    else{
        m_numDevices = 0;
    }
    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

///
/// \brief InputDevices::get_device_gripper_pairs
/// \return
///
std::vector<PhySimDevicePair*> InputDevices::getDeviceGripperPairs(std::vector<std::string> a_device_names){
    std::vector<PhySimDevicePair*> req_dg_Pairs;
    std::vector<PhySimDevicePair>::iterator dgIt;
    for(int req_name_Idx = 0 ; req_name_Idx < a_device_names.size() ; req_name_Idx++){
        std::string req_dev_name = a_device_names[req_name_Idx];
        bool _found_req_device = false;
        for(dgIt = m_psDevicePairs.begin(); dgIt != m_psDevicePairs.end() ; ++dgIt){
            if( dgIt->m_name.compare(req_dev_name) == 0 ){
                req_dg_Pairs.push_back(&(*dgIt));
                _found_req_device = true;
//                cerr << "INFO, DEVICE GRIPPER PAIR FOUND DEVICE \"" << req_dev_name << "\"" << endl;
            }
        }
        if (!_found_req_device){
            cerr << "INFO, DEVICE GRIPPER PAIR: \"" << req_dev_name << "\" NOT FOUND" << endl;
        }
    }

    return req_dg_Pairs;

}

///
/// \brief InputDevices::get_all_device_gripper_pairs
/// \return
///
std::vector<PhySimDevicePair*> InputDevices::getAllDeviceGripperPairs(){
     std::vector<PhySimDevicePair*> req_dg_Pairs;
      std::vector<PhySimDevicePair>::iterator dgIt;
     for(dgIt = m_psDevicePairs.begin(); dgIt != m_psDevicePairs.end() ; ++dgIt){
             req_dg_Pairs.push_back(&(*dgIt));
     }
    return req_dg_Pairs;
}


///
/// \brief InputDevices::next_mode
///
void InputDevices::nextMode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief InputDevices::prev_mode
///
void InputDevices::prevMode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief InputDevices::create_simulated_gripper
/// \param dev_num
/// \param a_physicalDevice
/// \return
///
SimulatedInputDevice* InputDevices::createSimulatedGripper(uint dev_num, PhysicalInputDevice* a_physicalDevice){
    std::ostringstream dev_str;
    dev_str << (dev_num + 1);
    std::string gripper_name = "Gripper" + dev_str.str();
    SimulatedInputDevice* simulatedGripper = new SimulatedInputDevice(g_afWorld);
    if(simulatedGripper->loadSimulatedGripper(gripper_name, a_physicalDevice->m_hInfo.m_modelName)){
        simulatedGripper->setParams(a_physicalDevice->m_hInfo, a_physicalDevice);
        a_physicalDevice->m_workspace_scale_factor = simulatedGripper->getWorkspaceScaleFactor();
        cVector3d localGripperPos = simulatedGripper->m_rootLink->getLocalPos();
        double l,w,h;
        g_afWorld->getEnclosureExtents(l,w,h);
        if (localGripperPos.length() == 0.0){
            double x = (int(dev_num / 2.0) * 0.8);
            double y = (dev_num % 2) ? +0.4 : -0.4;
            x /= simulatedGripper->m_workspaceScaleFactor;
            y /= simulatedGripper->m_workspaceScaleFactor;
            simulatedGripper->m_posRefOrigin.set(x, y, 0);
        }
        return simulatedGripper;
    }
    else{
        delete simulatedGripper;
        return NULL;
    }
}

///
/// \brief InputDevices::close_devices
///
void InputDevices::closeDevices(){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        m_psDevicePairs[devIdx].m_physicalDevice->m_hDevice->close();
    }
}


///
/// \brief InputDevices::increment_K_lh
/// \param a_offset
/// \return
///
double InputDevices::increment_K_lh(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_psDevicePairs[devIdx].m_simulatedDevice->K_lh + a_offset <= 0)
        {
            m_psDevicePairs[devIdx].m_simulatedDevice->K_lh = 0.0;
        }
        else{
            m_psDevicePairs[devIdx].m_simulatedDevice->K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_psDevicePairs[m_numDevices-1].m_simulatedDevice->K_lh;
        g_btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief InputDevices::increment_K_ah
/// \param a_offset
/// \return
///
double InputDevices::increment_K_ah(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_psDevicePairs[devIdx].m_simulatedDevice->K_ah + a_offset <=0){
            m_psDevicePairs[devIdx].m_simulatedDevice->K_ah = 0.0;
        }
        else{
            m_psDevicePairs[devIdx].m_simulatedDevice->K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_psDevicePairs[m_numDevices-1].m_simulatedDevice->K_ah;
        g_btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief InputDevices::increment_P_lc
/// \param a_offset
/// \return
///
double InputDevices::increment_P_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afGripperLinkPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getP_lin();
        if (_gain + a_offset <=0){
            sG->m_controller.setP_lin(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setP_lin( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "P_lc = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_P_ac
/// \param a_offset
/// \return
///
double InputDevices::increment_P_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afGripperLinkPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getP_ang();
        if (_gain + a_offset <=0){
            sG->m_controller.setP_ang(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setP_ang( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "P_ac = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_D_lc
/// \param a_offset
/// \return
///
double InputDevices::increment_D_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afGripperLinkPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getD_lin();
        if (_gain + a_offset <=0.01){
            // Keep a small value of Angular gain to avoid controller singularity
            sG->m_controller.setD_lin(0.01);
            _temp = 0.01;
        }
        else{
            sG->m_controller.setD_lin( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "D_lc = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_D_ac
/// \param a_offset
/// \return
///
double InputDevices::increment_D_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afGripperLinkPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getD_ang();
        if (_gain + a_offset <=0){
            sG->m_controller.setD_ang(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setD_ang( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "D_ac = " + cStr(_temp, 4);
    return _temp;
}

///
/// \brief This is an implementation of Sleep function that tries to adjust sleep between each cycle to maintain
/// the desired loop frequency. This class has been inspired from ROS Rate Sleep written by Eitan Marder-Eppstein
///
class RateSleep{
public:
    RateSleep(int a_freq){
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


std::shared_ptr<InputDevices> g_coordApp;
double g_margin = 0.02;
bool g_showPatch = false;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief main: This Application allows multi-manual tasks using several haptics devices.
/// Each device can perturb or control the dynamic bodies in the simulation
/// environment. The objects in the simulation are exposed via Asynchoronous
/// Framework (AMBF) to allow query and control via external applications.
/// \param argc
/// \param argv
/// \return
///
int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------
    namespace p_opt = boost::program_options;

    p_opt::options_description cmd_opts("InputDevices Application Usage");
    cmd_opts.add_options()
            ("help,h", "Show help")
            ("ndevs,n", p_opt::value<int>(), "Number of Haptic Devices to Load")
            ("timestep,t", p_opt::value<double>(), "Value in secs for fixed Simulation time step(dt)")
            ("enableforces,f", p_opt::value<bool>(), "Enable Force Feedback on Devices")
            ("margin,m", p_opt::value<double>(), "Cloth Collision Margin")
            ("show_patch,s", p_opt::value<bool>(), "Show Cloth Patch");
    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).run(), var_map);
    p_opt::notify(var_map);

    int num_devices_to_load = MAX_DEVICES;
    if(var_map.count("help")){ std::cout<< cmd_opts << std::endl; return 0;}
    if(var_map.count("ndevs")){ num_devices_to_load = var_map["ndevs"].as<int>();}
    if (var_map.count("timestep")){ g_dt_fixed = var_map["timestep"].as<double>();}
    if (var_map.count("enableforces")){ g_force_enable = var_map["enableforces"].as<bool>();}
    if (var_map.count("margin")){ g_margin = var_map["margin"].as<double>();}
    if (var_map.count("show_patch")){ g_showPatch = var_map["show_patch"].as<bool>();}

    cout << endl;
    cout << "____________________________________________________________" << endl << endl;
    cout << "ASYNCHRONOUS MULTI-BODY FRAMEWORK SIMULATOR (AMBF Simulator)" << endl;
    cout << endl << endl;
    cout << "\t\t(www.aimlab.wpi.edu)" << endl;
    cout << "\t\t  (Copyright 2019)" << endl;
    cout << "____________________________________________________________" << endl << endl;
    cout << "STARTUP COMMAND LINE OPTIONS: " << endl << endl;
    cout << "-t <float> simulation dt in seconds, (default: Dynamic RT)" << endl;
    cout << "-f <bool> enable haptic feedback, (default: True)" << endl;
    cout << "-n <int> max devices to load, (default: All Available)" << endl;
    cout << "------------------------------------------------------------" << endl << endl << endl;
    cout << endl;

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a dynamic world.
    g_bulletWorld = new cBulletWorld("World");

    // set the background color of the environment
    g_bulletWorld->m_backgroundColor.setWhite();

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////
    // set some gravity
    g_bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));


    //////////////////////////////////////////////////////////////////////////
    // AF MULTIBODY HANDLER
    //////////////////////////////////////////////////////////////////////////
    g_afWorld = new afWorld(g_bulletWorld);
    if (g_afWorld->loadBaseConfig("../../ambf_models/descriptions/launch.yaml")){
        // The world loads the lights and cameras + windows
        g_afMultiBody = new afMultiBody(g_afWorld);
        g_afMultiBody->loadAllMultiBodies();
        g_afWorld->loadWorld();
        g_cameras = g_afWorld->getAFCameras();

        g_bulletWorld->m_bulletWorld->setInternalTickCallback(preTickCallBack, 0, true);
    }

    //-----------------------------------------------------------------------------------------------------------
    // START: INTIALIZE SEPERATE WINDOWS FOR EACH WINDOW-CAMRERA PAIR
    //-----------------------------------------------------------------------------------------------------------
    for(g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        GLFWwindow* windowPtr = (*g_cameraIt)->m_window;
        GLFWmonitor* monitorPtr = (*g_cameraIt)->m_monitor;
        if (!windowPtr)
        {
            cout << "failed to create window" << endl;
            cSleepMs(1000);
            glfwTerminate();
            return 1;
        }

        // get width and height of window
        glfwGetWindowSize(windowPtr, &(*g_cameraIt)->m_width, &(*g_cameraIt)->m_height);

        // set position of window
        glfwSetWindowPos(windowPtr, (*g_cameraIt)->m_win_x, (*g_cameraIt)->m_win_y);

        // set key callback
        glfwSetKeyCallback(windowPtr, keyCallback);

        // set mouse buttons callback
        glfwSetMouseButtonCallback(windowPtr, mouseBtnsCallback);

        //set mouse buttons callback
        glfwSetCursorPosCallback(windowPtr, mousePosCallback);

        //set mouse scroll callback
        glfwSetScrollCallback(windowPtr, mouseScrollCallback);

        // set resize callback
        glfwSetWindowSizeCallback(windowPtr, windowSizeCallback);

        // set the current context
        glfwMakeContextCurrent(windowPtr);

        glfwSwapInterval(g_swapInterval);

        // initialize GLEW library
#ifdef GLEW_VERSION
        if (glewInit() != GLEW_OK)
        {
            cout << "failed to initialize GLEW library" << endl;
            glfwTerminate();
            return 1;
        }
#endif
    }

    //-----------------------------------------------------------------------------------------------------------
    // END: INTIALIZE SEPERATE WINDOWS FOR EACH WINDOW-CAMRERA PAIR
    //-----------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------
    // START: INITIALIZE THREADS FOR ALL REQUIRED HAPTIC DEVICES AND PHYSICS THREAD
    //-----------------------------------------------------------------------------------------------------------
    g_coordApp = std::make_shared<InputDevices>(g_bulletWorld, num_devices_to_load);

    //-----------------------------------------------------------------------------------------------------------
    // START: SEARCH FOR CONTROLLING DEVICES FOR CAMERAS IN AMBF AND ADD THEM TO RELEVANT WINDOW-CAMERA PAIR
    //-----------------------------------------------------------------------------------------------------------
    for (g_cameraIt = g_cameras.begin() ;  g_cameraIt !=  g_cameras.end() ; ++ g_cameraIt){
        std::vector<std::string> _controllingDevices = (*g_cameraIt)->m_controllingDevNames;
        (*g_cameraIt)->m_controllingDevNames.clear();
        unsigned long int n_controlling_devs = _controllingDevices.size();

        // If no controlling devices are defined for the camera context, add all
        // the current haptics devices specified for the simulation to each Window-Camera pair
        if(n_controlling_devs == 0){
            std::vector<PhySimDevicePair*> dgPairs = g_coordApp->getAllDeviceGripperPairs();
            for (int dgPairIdx = 0 ; dgPairIdx < dgPairs.size() ; dgPairIdx++){
                dgPairs[dgPairIdx]->m_cameras.push_back(*g_cameraIt);

                // Create labels for the contextual controlling devices for each Window-Camera Pair
                cFontPtr font = NEW_CFONTCALIBRI20();
                cLabel* devFreqLabel = new cLabel(font);
                devFreqLabel->m_fontColor.setBlack();
                devFreqLabel->setFontScale(0.8);
                dgPairs[dgPairIdx]->m_devFreqLabel = devFreqLabel;
                (*g_cameraIt)->m_devHapticFreqLabels.push_back(devFreqLabel);
                (*g_cameraIt)->getFrontLayer()->addChild(devFreqLabel);

                (*g_cameraIt)->m_controllingDevNames.push_back(
                            dgPairs[dgPairIdx]->m_physicalDevice->m_hInfo.m_modelName);
            }
        }
        else{
            // Pass the names of the controlling devices to only get the controlling devices
            // defined for the window camera pair in the context
            std::vector<PhySimDevicePair*> dgPairs  = g_coordApp->getDeviceGripperPairs(_controllingDevices);
            for (int dgPairIdx = 0 ; dgPairIdx < dgPairs.size() ; dgPairIdx++){
                dgPairs[dgPairIdx]->m_cameras.push_back(*g_cameraIt);
                // Create labels for the contextual controlling devices for each Window-Camera Pair
                cFontPtr font = NEW_CFONTCALIBRI20();
                cLabel* devFreqLabel = new cLabel(font);
                devFreqLabel->m_fontColor.setBlack();
                devFreqLabel->setFontScale(0.8);
                dgPairs[dgPairIdx]->m_devFreqLabel = devFreqLabel;
                (*g_cameraIt)->m_devHapticFreqLabels.push_back(devFreqLabel);
                (*g_cameraIt)->getFrontLayer()->addChild(devFreqLabel);


                (*g_cameraIt)->m_controllingDevNames.push_back(
                            dgPairs[dgPairIdx]->m_physicalDevice->m_hInfo.m_modelName);
            }
        }
    }


    if (g_showPatch){
        const btScalar s = 0.6;
        const int r = 5;
        btVector3 p1(-s, -s, 0);
        btVector3 p2( s, -s, 0);
        btVector3 p3(-s,  s, 0);
        btVector3 p4( s,  s, 0);
        btSoftBody* btPatch = btSoftBodyHelpers::CreatePatch(*g_bulletWorld->m_bulletSoftBodyWorldInfo,
                                                             p1,
                                                             p2,
                                                             p3,
                                                             p4, r, r, 1+4, true);
        btPatch->getCollisionShape()->setMargin(g_margin);
        btSoftBody::Material* pm = btPatch->appendMaterial();
        pm->m_kLST = 0.001;
        //    btPatch->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
        btPatch->generateBendingConstraints(2, pm);

        cGELSkeletonNode::s_default_radius = g_margin;

        afSoftMultiMesh* cloth = new afSoftMultiMesh(g_bulletWorld);
        cloth->setSoftBody(btPatch);
        cloth->createGELSkeleton();
        cloth->setMass(1);
        cloth->m_gelMesh.connectVerticesToSkeleton(false);
        cloth->buildDynamicModel();
        //    g_afWorld->addSoftBody(cloth, "cloth");
        g_bulletWorld->addChild(cloth);
    }

    //-----------------------------------------------------------------------------------------------------------
    // END: SEARCH FOR CONTROLLING DEVICES FOR CAMERAS IN AMBF AND ADD THEM TO RELEVANT WINDOW-CAMERA PAIR
    //-----------------------------------------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    int dev_num[10] = {0,1,2,3,4,5,6,7,8,9};
    for (int gIdx = 0 ; gIdx < g_coordApp->m_numDevices; gIdx++){
        g_hapticsThreads.push_back(new cThread());
        g_hapticsThreads[gIdx]->start(updateHapticDevice, CTHREAD_PRIORITY_HAPTICS, &dev_num[gIdx]);
    }

    //create a thread which starts the Bullet Simulation loop
    g_bulletSimThread = new cThread();
    g_bulletSimThread->start(updatePhysics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);
    //-----------------------------------------------------------------------------------------------------------
    // END: INITIALIZE THREADS FOR ALL REQUIRED HAPTIC DEVICES AND PHYSICS THREAD
    //-----------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    for (g_cameraIt = g_cameras.begin(); g_cameraIt != g_cameras.end() ; ++ g_cameraIt){
        windowSizeCallback((*g_cameraIt)->m_window, (*g_cameraIt)->m_width, (*g_cameraIt)->m_height);
    }

    // main graphic loop
    while (!g_window_closed)
    {
        // Call the update graphics method
        updateGraphics();

        // process events
        glfwPollEvents();

        // signal frequency counter
        g_freqCounterGraphics.signal(1);
    }

    // close window
    for (g_cameraIt = g_cameras.begin(); g_cameraIt !=  g_cameras.end() ; ++ g_cameraIt){
        glfwDestroyWindow((*g_cameraIt)->m_window);
    }


    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

///
/// \brief windowSizeCallback
/// \param a_window
/// \param a_width
/// \param a_height
///
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    std::vector<afCameraPtr>::iterator cameraIt;

    for(cameraIt = g_cameras.begin(); cameraIt != g_cameras.end() ; ++cameraIt){
        if( (*cameraIt)->m_window == a_window){
            // update window size
            (*cameraIt)->m_width = a_width;
            (*cameraIt)->m_height = a_height;
        }
    }

}

///
/// \brief errorCallback
/// \param a_error
/// \param a_description
///
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

bool g_updateLabels = true;
bool g_enableGrippingAssist = true;

///
/// \brief keyCallback
/// \param a_window
/// \param a_key
/// \param a_scancode
/// \param a_action
/// \param a_mods
///
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        std::vector<afCameraPtr>::iterator cameraIt;
        for (cameraIt = g_cameras.begin() ; cameraIt != g_cameras.end() ; ++cameraIt){

            // get handle to monitor
            GLFWmonitor* monitor = (*cameraIt)->m_monitor;

            // get handle to window
            GLFWwindow* window = (*cameraIt)->m_window;

            // get information about monitor
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);

            // set fullscreen or window mode
            if (fullscreen)
            {
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
                glfwSwapInterval(g_swapInterval);
            }
            else
            {
                int w = 0.8 * mode->height;
                int h = 0.5 * mode->height;
                int x = 0.5 * (mode->width - w);
                int y = 0.5 * (mode->height - h);
                glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
                glfwSwapInterval(g_swapInterval);
            }
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M){
         mirroredDisplay = !mirroredDisplay;
        std::vector<afCameraPtr>::iterator cameraIt;
        for (cameraIt = g_cameras.begin() ; cameraIt != g_cameras.end() ; ++cameraIt){
            (*cameraIt)->setMirrorVertical(mirroredDisplay);
        }
    }

    // option - help menu
    else if (a_key == GLFW_KEY_H)
    {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;
        cout << "[3] - decrease linear haptic gain" << endl;
        cout << "[4] - increase linear haptic gain" << endl;
        cout << "[5] - decrease angular haptic gain" << endl;
        cout << "[6] - increase angular haptic gain" << endl << endl;
        cout << "[7] - decrease linear stiffness" << endl;
        cout << "[8] - increase linear stiffness" << endl;
        cout << "[9] - decrease angular stiffness" << endl;
        cout << "[0] - increase angular stiffness" << endl;
        cout << "[PgUp] - increase linear damping" << endl;
        cout << "[PgDown] - decrease linear damping" << endl;
        cout << "[Home] - increate angular damping" << endl;
        cout << "[End] - decrease angular damping" << endl << endl;
        cout << "[v] - toggle frame/skeleton visualization" << endl;
        cout << "[s] - toggle sensors visibility" << endl;
        cout << "[w] - use world frame for orientation clutch" << endl;
        cout << "[c] - use9999 camera frame for orientation clutch" << endl;
        cout << "[n] - next device mode" << endl << endl;
        cout << "[i] - toogle inverted y for camera control via mouse" << endl << endl;
        cout << "[t] - toogle gripper picking constraints" << endl << endl;
        cout << "[p] - toogle mouse picking constraints" << endl << endl;
        cout << "[u] - toogle update of labels" << endl << endl;
        cout << "[q] - Exit application\n" << endl;
        cout << endl << endl;
    }

    // option - enable gravity
    else if (a_key == GLFW_KEY_1)
    {
        // enable gravity
        g_bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));
        printf("gravity ON:\n");
    }

    // option - disable gravity
    else if (a_key == GLFW_KEY_2)
    {
        // disable gravity
        g_bulletWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("gravity OFF:\n");
    }

    // option - decrease linear haptic gain
    else if (a_key == GLFW_KEY_3)
    {
        printf("linear haptic gain:  %f\n", g_coordApp->increment_K_lh(-0.05));
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        printf("linear haptic gain:  %f\n", g_coordApp->increment_K_lh(0.05));
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        printf("angular haptic gain:  %f\n", g_coordApp->increment_K_ah(-0.05));
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        printf("angular haptic gain:  %f\n", g_coordApp->increment_K_ah(0.05));
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        printf("linear stiffness:  %f\n", g_coordApp->increment_P_lc(-50));
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        printf("linear stiffness:  %f\n", g_coordApp->increment_P_lc(50));
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_P_ac(-1));
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_P_ac(1));
    }

    // option - decrease linear damping
    else if (a_key == GLFW_KEY_PAGE_DOWN)
    {
        printf("linear damping:  %f\n", g_coordApp->increment_D_lc(-0.1));
    }

    // option - increase linear damping
    else if (a_key == GLFW_KEY_PAGE_UP)
    {
        printf("linear damping:  %f\n", g_coordApp->increment_D_lc(0.1));
    }

    // option - decrease angular damping
    else if (a_key == GLFW_KEY_END)
    {
        printf("angular damping:  %f\n", g_coordApp->increment_D_ac(-0.1));
    }

    // option - increase angular damping
    else if (a_key == GLFW_KEY_HOME)
    {
        printf("angular damping:  %f\n", g_coordApp->increment_D_ac(0.1));
    }

    // option - grippers orientation w.r.t contextual camera
    else if (a_key == GLFW_KEY_C){
        g_coordApp->m_use_cam_frame_rot = true;
        printf("Gripper Rotation w.r.t Camera Frame:\n");
    }

    // option - grippers orientation w.r.t world
    else if (a_key == GLFW_KEY_W){
        g_coordApp->m_use_cam_frame_rot = false;
        printf("Gripper Rotation w.r.t World Frame:\n");
    }

    // option - Change to next device mode
    else if (a_key == GLFW_KEY_N){
        g_coordApp->nextMode();
        printf("Changing to next device mode:\n");
    }

    // option - Toogle visibility of body frames and softbody skeleton
    else if (a_key == GLFW_KEY_V){
        auto rbMap = g_afWorld->getAFRigidBodyMap();
        afRigidBodyMap::const_iterator rbIt;
        for (rbIt = rbMap->begin() ; rbIt != rbMap->end(); ++rbIt){
            rbIt->second->toggleFrameVisibility();
        }

        auto sbMap = g_afWorld->getAFSoftBodyMap();
        afSoftBodyMap::const_iterator sbIt;
        for (sbIt = sbMap->begin() ; sbIt != sbMap->end(); ++sbIt){
            sbIt->second->toggleSkeletalModelVisibility();
        }
    }

    // option - Toogle mouse picking
    else if (a_key == GLFW_KEY_P){
        g_mousePickingEnabled = !g_mousePickingEnabled;
    }

    // option - Toggle Inverted Y axis of mouse for camera control
    else if (a_key == GLFW_KEY_I){
        g_mouse_inverted_y = !g_mouse_inverted_y;
    }

    // option - Toggle visibility of label updates
    else if (a_key == GLFW_KEY_U){
        g_updateLabels = !g_updateLabels;
    }

    // option - Toggle Ray Test for Gripper Picking
    else if (a_key == GLFW_KEY_T){
        g_enableGrippingAssist = !g_enableGrippingAssist;
    }

    // option - Toggle Visibility of Sensors
    else if (a_key == GLFW_KEY_S){
        auto sMap = g_afWorld->getAFSensorMap();
        afSensorMap::const_iterator sIt;
        for (sIt = sMap->begin() ; sIt != sMap->end(); ++sIt){
            sIt->second->toggleSensorVisibility();
        }
    }

}

///
/// \brief mouseBtnCallback
/// \param window
/// \param a_button1
/// \param a_button2
/// \param a_button3
/// \param a_button4
///
void mouseBtnsCallback(GLFWwindow* a_window, int a_button, int a_action, int a_modes){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            if (a_button == GLFW_MOUSE_BUTTON_1){
                (*g_cameraIt)->mouse_l_clicked = a_action;
//                (*g_cameraIt)->showTargetPos(true);
                if (a_action){
                    if (g_mousePickingEnabled){
                        g_pickBody = true;
                        cVector3d rayFrom = (*g_cameraIt)->getLocalPos();
                        double x_pos, y_pos;
                        glfwGetCursorPos(a_window, &x_pos, &y_pos);
                        cVector3d rayTo = getRayTo(x_pos, y_pos, *g_cameraIt);
                        g_pickFrom = rayFrom;
                        g_pickTo = rayTo;
                    }
                }
                else{
                    g_pickBody = false;
                }
            }
            if (a_button == GLFW_MOUSE_BUTTON_2){
                (*g_cameraIt)->mouse_r_clicked = a_action;
            }
            if (a_button == GLFW_MOUSE_BUTTON_3){
                (*g_cameraIt)->mouse_scroll_clicked = a_action;
            }
        }
    }
}

///
void mousePosCallback(GLFWwindow* a_window, double a_xpos, double a_ypos){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            afCamera* devCam = (*g_cameraIt);
            (*g_cameraIt)->mouse_x[1] = (*g_cameraIt)->mouse_x[0];
            (*g_cameraIt)->mouse_x[0] = a_xpos;
            (*g_cameraIt)->mouse_y[1] = (*g_cameraIt)->mouse_y[0];
            (*g_cameraIt)->mouse_y[0] = a_ypos;

            if( devCam->mouse_l_clicked ){
                if(g_mousePickingEnabled){
                    cVector3d rayFrom = (*g_cameraIt)->getLocalPos();
                    cVector3d rayTo = getRayTo(a_xpos, a_ypos, (*g_cameraIt));
                    g_pickFrom = rayFrom;
                    g_pickTo = rayTo;
                }
                else{
                    double scale = 0.01;
                    double x_vel = scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]);
                    double y_vel = scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]);
                    if (g_mouse_inverted_y){
                        y_vel = -y_vel;
                    }
                    cVector3d camVel(0, -x_vel, y_vel);
                    cVector3d dPos = devCam->getLocalPos() + devCam->getLocalRot() * camVel;
                    devCam->setLocalPos(dPos);
                }
            }

            if( devCam->mouse_r_clicked ){
                cMatrix3d camRot;
                double scale = 0.3;
                double z_vel = scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]);
                double y_vel = scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]);
                if (g_mouse_inverted_y){
                    y_vel = -y_vel;
                }

                cVector3d camStrafe = cCross(devCam->getLookVector(), devCam->getUpVector());
                cVector3d nz(0, 0, 1);
                cVector3d ny(0, 1, 0);

                cMatrix3d camViewWithoutPitch(cCross(camStrafe, nz), camStrafe ,nz);
                cMatrix3d camViewPitchOnly;
                double pitchAngle = -cAngle(nz, devCam->getUpVector());
                camViewPitchOnly.setAxisAngleRotationRad(ny, pitchAngle);
                camRot.setIntrinsicEulerRotationDeg(0, y_vel, z_vel, cEulerOrder::C_EULER_ORDER_XYZ);
                (*g_cameraIt)->camRot = camRot;

                devCam->setLocalRot( camViewWithoutPitch * (*g_cameraIt)->camRot * camViewPitchOnly );
            }
            else{
                devCam->camRotPre = (*g_cameraIt)->camRot;
            }

            if( devCam->mouse_scroll_clicked){
//                devCam->showTargetPos(true);
                double scale = 0.03;
                double x_vel = scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]);
                double y_vel = scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]);
                if (g_mouse_inverted_y){
                    y_vel = -y_vel;
                }
                cVector3d dVel(0, -x_vel, y_vel);
                cVector3d newPos = devCam->getLocalPos() + devCam->getLocalRot() * dVel;
                devCam->setView(newPos, devCam->getTargetPos(), cVector3d(0,0,1));
            }
//            else{
//                devCam->showTargetPos(false);
//            }

        }
    }
}

void mouseScrollCallback(GLFWwindow *a_window, double a_xpos, double a_ypos){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            afCameraPtr cameraPtr = (*g_cameraIt);
            (*g_cameraIt)->mouse_scroll[1] = (*g_cameraIt)->mouse_scroll[0];
            (*g_cameraIt)->mouse_scroll[0] = -a_ypos;

            double scale = 0.1;
            cVector3d camVelAlongLook(scale * (*g_cameraIt)->mouse_scroll[0], 0, 0);
            cVector3d _targetPos = cameraPtr->getTargetPos();
            cVector3d _newPos = cameraPtr->getLocalPos() + cameraPtr->getLocalRot() * camVelAlongLook;
            cVector3d dPos = _newPos - _targetPos;
            if(dPos.length() < 0.5){
                _targetPos = _targetPos + cameraPtr->getLocalRot() * camVelAlongLook;
            }
            cameraPtr->setLocalPos( cameraPtr->getLocalPos() + cameraPtr->getLocalRot() * camVelAlongLook );
            cameraPtr->setTargetPos(_targetPos);
        }
    }
}

// The following functions have been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief getRayTo
/// \param x
/// \param y
/// \param a_cameraPtr
/// \return
///
cVector3d getRayTo(int x, int y, afCameraPtr a_cameraPtr)
{
    float top = 1.f;
    float bottom = -1.0f;
    float nearPlane = 1.f;
    float tanFov = (top - bottom) * 0.5f / nearPlane;
    float fov = btScalar(1.0) * btAtan(tanFov);

    btVector3 camPos, camTarget;

    camPos = cVec2btVec(a_cameraPtr->getLocalPos() );
    camTarget = cVec2btVec(a_cameraPtr->getTargetPos() );

    btVector3 rayFrom = camPos;
    btVector3 rayForward = (camTarget - camPos);
    rayForward.normalize();
    float farPlane = 10000.f;
    rayForward *= farPlane;

    btVector3 cameraUp = btVector3(0, 0, 0);
    cameraUp[2] = 1;

    btVector3 vertical = cameraUp;

    btVector3 hor;
    hor = rayForward.cross(vertical);
    hor.safeNormalize();
    vertical = hor.cross(rayForward);
    vertical.safeNormalize();

    float tanfov = tanf(0.5f * fov);

    hor *= 2.f * farPlane * tanfov;
    vertical *= 2.f * farPlane * tanfov;

    btScalar aspect;
    float width = float(a_cameraPtr->m_width);
    float height = float(a_cameraPtr->m_height);

    aspect = width / height;

    hor *= aspect;

    btVector3 rayToCenter = rayFrom + rayForward;
    btVector3 dHor = hor * 1.f / width;
    btVector3 dVert = vertical * 1.f / height;

    btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
    rayTo += btScalar(x) * dHor;
    rayTo -= btScalar(y) * dVert;
    cVector3d cRay = btVec2cVec(rayTo);
    return cRay;
}


///
/// \brief preTickCallBack: This function is there to account for all the
/// desired features of AMBF that we need from Bullet but are either not
/// implemented or difficult to wrap around from AMBF
/// \param world
/// \param timeStep
///
void preTickCallBack(btDynamicsWorld *world, btScalar timeStep){
    // Check if a softbody has been picked
    if (g_afMultiBody->m_pickedSoftBody){
        cVector3d delta = g_afMultiBody->m_pickedNodeGoal - btVec2cVec(g_afMultiBody->m_pickedNode->m_x);
        static const double maxdrag = 10;
        if (delta.length() > (maxdrag * maxdrag))
        {
            delta.normalize();
            delta = delta * maxdrag;
        }
        g_afMultiBody->m_pickedNode->m_v += cVec2btVec(delta) / timeStep;
    }
}

///
/// \brief close
///
void close(void)
{
    // stop the simulation
    g_simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!g_simulationFinished) { cSleepMs(100); }
    g_bulletSimThread->stop();
    for(int i = 0 ; i < g_coordApp->m_numDevices ; i ++){
        g_hapticsThreads[i]->stop();
    }

    // delete resources
    g_coordApp->closeDevices();
    for(int i = 0 ; i < g_coordApp->m_numDevices ; i ++){
        delete g_hapticsThreads[i];
    }
    delete g_bulletWorld;
    delete g_afWorld;
    delete g_afMultiBody;
}


///
/// \brief updateGraphics
///
void updateGraphics()
{
    // Update shadow maps once
    g_bulletWorld->updateShadowMaps(false, mirroredDisplay);

    for (g_cameraIt = g_cameras.begin(); g_cameraIt != g_cameras.end(); ++ g_cameraIt){
        afCameraPtr cameraPtr = (*g_cameraIt);
        // set current display context
        glfwMakeContextCurrent(cameraPtr->m_window);

        // get width and height of window
        glfwGetWindowSize(cameraPtr->m_window, &cameraPtr->m_width, &cameraPtr->m_height);

        // Update the Labels in a separate sub-routine
        if (g_updateLabels)
            updateLabels();

        // render world
        cameraPtr->renderView(cameraPtr->m_width, cameraPtr->m_height);

        // swap buffers
        glfwSwapBuffers(cameraPtr->m_window);

        // Only set the _window_closed if the condition is met
        // otherwise a non-closed window will set the variable back
        // to false
        if (glfwWindowShouldClose(cameraPtr->m_window)){
            g_window_closed = true;
        }

//        // wait until all GL commands are completed
//        glFinish();

//        // check for any OpenGL errors
//        GLenum err = glGetError();
//        if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
    }

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

}

///
/// \brief updateLabels
///
void updateLabels(){
    // Not all labels change at every frame buffer.
    // We should prioritize the update of freqeunt labels
    afCameraVec::iterator cameraIt;
    for (cameraIt = g_cameras.begin(); cameraIt != g_cameras.end(); ++ cameraIt){
        afCameraPtr cameraPtr = *cameraIt;
//        int n_devsAttached = cameraPtr->m_deviceGripperPairs.size();
        int width = cameraPtr->m_width;
        int height = cameraPtr->m_height;

        cLabel* dynFreqLabel = cameraPtr->m_graphicsDynamicsFreqLabel;
        cLabel* timesLabel = cameraPtr->m_wallSimTimeLabel;
        cLabel* modesLabel = cameraPtr->m_devicesModesLabel;
        cLabel* btnLabel = cameraPtr->m_deviceButtonLabel;
        cLabel* contextDevicesLabel = cameraPtr->m_controllingDeviceLabel;
        std::vector<cLabel*> devFreqLabels = cameraPtr->m_devHapticFreqLabels;

        // update haptic and graphic rate data
        std::string wallTimeStr = "Wall Time:" + cStr(g_clockWorld.getCurrentTimeSeconds(),2) + " s";
        std::string simTimeStr = "Sim Time" + cStr(g_bulletWorld->getSimulationTime(),2) + " s";

        std::string graphicsFreqStr = "Gfx (" + cStr(g_freqCounterGraphics.getFrequency(), 0) + " Hz)";
        std::string hapticFreqStr = "Phx (" + cStr(g_freqCounterHaptics.getFrequency(), 0) + " Hz)";

        std::string timeLabelStr = wallTimeStr + " / " + simTimeStr;
        std::string dynHapticFreqLabelStr = graphicsFreqStr + " / " + hapticFreqStr;
        std::string modeLabelStr = "MODE: " + g_coordApp->m_mode_str;
        std::string btnLabelStr = " : " + g_btn_action_str;

        timesLabel->setText(timeLabelStr);
        dynFreqLabel->setText(dynHapticFreqLabelStr);
        modesLabel->setText(modeLabelStr);
        btnLabel->setText(btnLabelStr);

        std::string controlling_dev_names;
        for (int devIdx = 0 ; devIdx < devFreqLabels.size() ; devIdx++){
            devFreqLabels[devIdx]->setLocalPos(10, (int)( height - ( devIdx + 1 ) * 20 ) );
            controlling_dev_names += cameraPtr->m_controllingDevNames[devIdx] + " <> ";
        }

        contextDevicesLabel->setText("Controlling Devices: [ " + controlling_dev_names + " ]");

        // update position of label
        timesLabel->setLocalPos((int)(0.5 * (width - timesLabel->getWidth() ) ), 30);
        dynFreqLabel->setLocalPos((int)(0.5 * (width - dynFreqLabel->getWidth() ) ), 10);
        modesLabel->setLocalPos((int)(0.5 * (width - modesLabel->getWidth())), 50);
        btnLabel->setLocalPos((int)(0.5 * (width - modesLabel->getWidth()) + modesLabel->getWidth()), 50);
        contextDevicesLabel->setLocalPos((int)(0.5 * (width - contextDevicesLabel->getWidth())), (int)(height - 20));

    }

}

///
/// \brief Function to fix time dilation
/// \param adjust_int_steps
/// \return
///
double compute_dt(bool adjust_int_steps = false){
    double dt = g_clockWorld.getCurrentTimeSeconds() - g_bulletWorld->getSimulationTime();
    int min_steps = 2;
    int max_steps = 10;
    if (adjust_int_steps){
        if (dt >= g_bulletWorld->getIntegrationTimeStep() * min_steps){
            int int_steps_max =  dt / g_bulletWorld->getIntegrationTimeStep();
            if (int_steps_max > max_steps){
                int_steps_max = max_steps;
            }
            g_bulletWorld->setIntegrationMaxIterations(int_steps_max + min_steps);        }
    }
    return dt;
}

///
/// \brief updateBulletSim
///
void updatePhysics(){
    g_simulationRunning = true;
    g_simulationFinished = false;

    // start haptic device
    g_clockWorld.start(true);

    double sleepHz;
    if (g_dt_fixed > 0.0)
        sleepHz = (1.0/g_dt_fixed);
    else
        sleepHz= 1000;

    RateSleep rateSleep(sleepHz);
    bool _bodyPicked = false;

    cVector3d torque, torque_prev;
    torque.set(0, 0, 0);
    torque_prev.set(0, 0, 0);
    while(g_simulationRunning)
    {
        g_freqCounterHaptics.signal(1);

        // Take care of any picked body by mouse
        if (g_pickBody){
            if (_bodyPicked == false){
                _bodyPicked = true;
                g_afMultiBody->pickBody(g_pickFrom, g_pickTo);
            }
            else{
                g_afMultiBody->movePickedBody(g_pickFrom, g_pickTo);
            }
        }
        else{
            _bodyPicked = false;
            g_afMultiBody->removePickingConstraint();
        }

        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt(true);
        for (unsigned int devIdx = 0 ; devIdx < g_coordApp->m_numDevices ; devIdx++){
            // update position of simulate gripper
            SimulatedInputDevice * simGripper = g_coordApp->m_psDevicePairs[devIdx].m_simulatedDevice;
            afGripperLinkPtr rootLink = simGripper->m_rootLink;
            simGripper->updateMeasuredPose();

            if (g_enableGrippingAssist){
                for (int sIdx = 0 ; sIdx < rootLink->getSensors().size() ; sIdx++){
                    afSensorPtr sensorPtr = rootLink->getSensors()[sIdx];
                    if (sensorPtr->m_sensorType == afSensorType::proximity){
                        afProximitySensor* proximitySensorPtr = (afProximitySensor*) sensorPtr;
                        if (proximitySensorPtr->isTriggered() && simGripper->m_gripper_angle < 0.5){
                            if (proximitySensorPtr->m_sensedBodyType == afProximitySensor::RIGID_BODY){
                                if (!simGripper->m_rigidGrippingConstraints[sIdx]){
                                    btRigidBody* bodyAPtr = proximitySensorPtr->getParentBody()->m_bulletRigidBody;
                                    btRigidBody* bodyBPtr = proximitySensorPtr->getSensedRigidBody();
                                    if (!rootLink->isChild(bodyBPtr)){
                                        cVector3d hitPointInWorld = proximitySensorPtr->getSensedPoint();
                                        btVector3 pvtA = bodyAPtr->getCenterOfMassTransform().inverse() * cVec2btVec(hitPointInWorld);
                                        btVector3 pvtB = bodyBPtr->getCenterOfMassTransform().inverse() * cVec2btVec(hitPointInWorld);
                                        simGripper->m_rigidGrippingConstraints[sIdx] = new btPoint2PointConstraint(*bodyAPtr, *bodyBPtr, pvtA, pvtB);
                                        simGripper->m_rigidGrippingConstraints[sIdx]->m_setting.m_impulseClamp = 3.0;
                                        simGripper->m_rigidGrippingConstraints[sIdx]->m_setting.m_tau = 0.001f;
                                        g_bulletWorld->m_bulletWorld->addConstraint(simGripper->m_rigidGrippingConstraints[sIdx]);
                                    }
                                }
                            }

                            if (proximitySensorPtr->m_sensedBodyType == afProximitySensor::SOFT_BODY){
                                if (!simGripper->m_softGrippingConstraints[sIdx]){
                                    // Here we implemented the softBody grad logic. We want to move the
                                    // soft body as we move the simulated end effector

                                    // Get the parent body that owns this sensor
                                    btRigidBody* _rBody = proximitySensorPtr->getParentBody()->m_bulletRigidBody;
                                    // Get the sensed softbody
                                    btSoftBody* _sBody = proximitySensorPtr->getSensedSoftBody();

                                    simGripper->m_softGrippingConstraints[sIdx] = new SoftBodyGrippingConstraint();
                                    simGripper->m_softGrippingConstraints[sIdx]->m_sBody = _sBody;
                                    simGripper->m_softGrippingConstraints[sIdx]->m_rBody = _rBody;

                                    // If we get a sensedSoftBody, we should check if it has a detected face. If a face
                                    // is found, we can anchor all the connecting nodes.
                                    if (proximitySensorPtr->getSensedSoftBodyFace()){
                                        btSoftBody::Face* _sensedFace = proximitySensorPtr->getSensedSoftBodyFace();
                                        for (int nIdx = 0; nIdx < 3 ; nIdx++){
                                            btSoftBody::Node* _node = _sensedFace->m_n[nIdx];
                                            btVector3 _localPivot = _rBody->getCenterOfMassTransform().inverse() * _node->m_x;

                                            btSoftBody::Anchor _anchor;
                                            _node->m_battach = 1;
                                            _anchor.m_body = _rBody;
                                            _anchor.m_node = _node;
                                            _anchor.m_influence = 1;
                                            _anchor.m_local = _localPivot;
                                            _sBody->m_anchors.push_back(_anchor);
                                            simGripper->m_softGrippingConstraints[sIdx]->m_nodePtrs.push_back(_node);
                                        }
                                    }
                                    // Otherwise we shall directly anchor to nodes. This case
                                    // arises for ropes, suturing thread etc
                                    else{
                                        btSoftBody::Node* _node = proximitySensorPtr->getSensedSoftBodyNode();
                                        btVector3 _localPivot = _rBody->getCenterOfMassTransform().inverse() * _node->m_x;
                                        btSoftBody::Anchor _anchor;
                                        _node->m_battach = 1;
                                        _anchor.m_body = _rBody;
                                        _anchor.m_node = _node;
                                        _anchor.m_influence = 1;
                                        _anchor.m_local = _localPivot;
                                        _sBody->m_anchors.push_back(_anchor);
                                        simGripper->m_softGrippingConstraints[sIdx]->m_nodePtrs.push_back(_node);

                                    }
                                }
                            }
                        }
                        else{
                            if(simGripper->m_rigidGrippingConstraints[sIdx]){
                                g_bulletWorld->m_bulletWorld->removeConstraint(simGripper->m_rigidGrippingConstraints[sIdx]);
                                simGripper->m_rigidGrippingConstraints[sIdx] = 0;
                            }
                            if(simGripper->m_softGrippingConstraints[sIdx]){
                                for (int nIdx = 0 ; nIdx < simGripper->m_softGrippingConstraints[sIdx]->m_nodePtrs.size()  ; nIdx++){
                                    btSoftBody::Node* _nodePtr = simGripper->m_softGrippingConstraints[sIdx]->m_nodePtrs[nIdx];
                                    btSoftBody* _sBody = simGripper->m_softGrippingConstraints[sIdx]->m_sBody;
                                    btRigidBody* _rBody = simGripper->m_softGrippingConstraints[sIdx]->m_rBody;
                                    for (int aIdx = 0 ; aIdx < _sBody->m_anchors.size() ; aIdx++){
                                        if (_sBody->m_anchors[aIdx].m_body == _rBody){
                                            btSoftBody::Anchor* _anchor = &_sBody->m_anchors[aIdx];
                                            if (_anchor->m_node == _nodePtr){
                                                _sBody->m_anchors.removeAtIndex(aIdx);
                                                break;
                                            }
                                        }
                                    }
                                }
                                simGripper->m_softGrippingConstraints[sIdx] = 0;
                            }
                        }
                    }
                }
            }

            cVector3d force, torque;

            force = rootLink->m_controller.computeOutput_cvec(simGripper->m_pos, simGripper->m_posRef, dt);
            force = simGripper->P_lc_ramp * force;

            torque = rootLink->m_controller.computeOutput_cvec(simGripper->m_rot, simGripper->m_rotRef, dt);
            simGripper->applyForce(force);
            simGripper->applyTorque(torque);
            simGripper->setGripperAngle(simGripper->m_gripper_angle, dt);

            if (simGripper->P_lc_ramp < 1.0)
            {
                simGripper->P_lc_ramp = simGripper->P_lc_ramp + 0.5 * dt;
            }
            else
            {
                simGripper->P_lc_ramp = 1.0;
            }

            if (simGripper->P_ac_ramp < 1.0)
            {
                simGripper->P_ac_ramp = simGripper->P_ac_ramp + 0.5 * dt;
            }
            else
            {
                simGripper->P_ac_ramp = 1.0;
            }
        }
        g_bulletWorld->updateDynamics(dt, g_clockWorld.getCurrentTimeSeconds(), g_freqCounterHaptics.getFrequency(), g_coordApp->m_numDevices);
        rateSleep.sleep();
    }
    g_simulationFinished = true;
}

///
/// \brief updateHaptics
/// \param a_arg
///
void updateHapticDevice(void* a_arg){
    int devIdx = *(int*) a_arg;
    // simulation in now running
    g_simulationRunning = true;
    g_simulationFinished = false;

    // update position and orientation of simulated gripper
    PhysicalInputDevice *pDev = g_coordApp->m_psDevicePairs[devIdx].m_physicalDevice;
    SimulatedInputDevice* simGripper = g_coordApp->m_psDevicePairs[devIdx].m_simulatedDevice;
    std::vector<afCameraPtr> devCams = g_coordApp->m_psDevicePairs[devIdx].m_cameras;
    cLabel* devFreqLabel = g_coordApp->m_psDevicePairs[devIdx].m_devFreqLabel;
    if (g_coordApp->m_psDevicePairs[devIdx].m_cameras.size() == 0){
        cerr << "WARNING: DEVICE HAPTIC LOOP \"" << pDev->m_hInfo.m_modelName << "\" NO WINDOW-CAMERA PAIR SPECIFIED, USING DEFAULT" << endl;
        devCams = g_cameras;
    }

    pDev->m_posClutched.set(0.0,0.0,0.0);
    pDev->measuredRot();
    pDev->m_rotClutched.identity();
    simGripper->m_rotRefOrigin = pDev->m_rot;

    cVector3d dpos, ddpos, dposLast;
    cMatrix3d drot, ddrot, drotLast;
    dpos.set(0,0,0); ddpos.set(0,0,0); dposLast.set(0,0,0);
    drot.identity(); ddrot.identity(); drotLast.identity();

    double P_lc_offset = 10;
    double P_ac_offset = 1;
    double D_lc_offset = 1;
    double D_ac_offset = 1;
    double K_lh_offset = 5;
    double K_ah_offset = 1;

    double wait_time = 1.0;
    if (std::strcmp(pDev->m_hInfo.m_modelName.c_str(), "Razer Hydra") == 0 ){
        wait_time = 5.0;
    }

    // main haptic simulation loop
    while(g_simulationRunning)
    {
        pDev->m_freq_ctr.signal(1);
        if (devFreqLabel != NULL){
            devFreqLabel->setText(pDev->m_hInfo.m_modelName + ": " + cStr(pDev->m_freq_ctr.getFrequency(), 0) + " Hz");
        }
        // Adjust time dilation by computing dt from clockWorld time and the simulationTime
        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt();

        pDev->m_pos = pDev->measuredPos();
        pDev->m_rot = pDev->measuredRot();

        if(simGripper->m_gripper_pinch_btn >= 0){
            if(pDev->isButtonPressed(simGripper->m_gripper_pinch_btn)){
                pDev->enableForceFeedback(true);
            }
        }
        if (pDev->m_hInfo.m_sensedGripper){
            simGripper->m_gripper_angle = pDev->measuredGripperAngle();
        }
        else{
            simGripper->m_gripper_angle = 0.5;
        }

        if(pDev->isButtonPressRisingEdge(simGripper->mode_next_btn)) g_coordApp->nextMode();
        if(pDev->isButtonPressRisingEdge(simGripper->mode_prev_btn)) g_coordApp->prevMode();

        bool btn_1_rising_edge = pDev->isButtonPressRisingEdge(simGripper->act_1_btn);
        bool btn_1_falling_edge = pDev->isButtonPressFallingEdge(simGripper->act_1_btn);
        bool btn_2_rising_edge = pDev->isButtonPressRisingEdge(simGripper->act_2_btn);
        bool btn_2_falling_edge = pDev->isButtonPressFallingEdge(simGripper->act_2_btn);

        double gripper_offset = 0;
        switch (g_coordApp->m_simModes){
        case MODES::CAM_CLUTCH_CONTROL:
            g_clutch_btn_pressed  = pDev->isButtonPressed(simGripper->act_1_btn);
            g_cam_btn_pressed     = pDev->isButtonPressed(simGripper->act_2_btn);
            if(g_clutch_btn_pressed) g_btn_action_str = "Clutch Pressed";
            if(g_cam_btn_pressed)   {g_btn_action_str = "Cam Pressed";}
            if(btn_1_falling_edge || btn_2_falling_edge) g_btn_action_str = "";
            break;
        case MODES::GRIPPER_JAW_CONTROL:
            if (btn_1_rising_edge) gripper_offset = 0.1;
            if (btn_2_rising_edge) gripper_offset = -0.1;
            simGripper->offsetGripperAngle(gripper_offset);
            break;
        case MODES::CHANGE_CONT_LIN_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_P_lc(P_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_P_lc(-P_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_P_ac(P_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_P_ac(-P_ac_offset);
            break;
        case MODES::CHANGE_CONT_LIN_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_D_lc(D_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_D_lc(-D_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_D_ac(D_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_D_ac(-D_ac_offset);
            break;
        case MODES::CHANGE_DEV_LIN_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_lh(K_lh_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_lh(-K_lh_offset);
            break;
        case MODES::CHANGE_DEV_ANG_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_ah(K_ah_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_ah(-K_ah_offset);
            break;
        }

        pDev->m_hDevice->getUserSwitch(simGripper->act_2_btn, devCams[0]->m_cam_pressed);
        if(devCams[0]->m_cam_pressed && g_coordApp->m_simModes == MODES::CAM_CLUTCH_CONTROL){
            double scale = 0.1;
            for (int dcIdx = 0 ; dcIdx < devCams.size() ; dcIdx++){
                devCams[dcIdx]->setLocalPos(devCams[dcIdx]->measuredPos() + cMul(scale, devCams[dcIdx]->measuredRot() * pDev->measuredVelLin() ) );
                devCams[dcIdx]->setLocalRot(pDev->measuredRotCamPreclutch() * cTranspose(pDev->measuredRotPreclutch()) * pDev->measuredRot());
            }

        }
        if (!devCams[0]->m_cam_pressed){
                pDev->setRotCamPreclutch( devCams[0]->measuredRot() );
                pDev->setRotPreclutch( pDev->measuredRot() );
        }


        if (g_clockWorld.getCurrentTimeSeconds() < wait_time){
            pDev->m_posClutched = pDev->m_pos;
        }

        if(g_cam_btn_pressed){
            if(simGripper->btn_cam_rising_edge){
                simGripper->btn_cam_rising_edge = false;
                simGripper->m_posRefOrigin = simGripper->m_posRef / simGripper->m_workspaceScaleFactor;
                simGripper->m_rotRefOrigin = simGripper->m_rotRef;
            }
            pDev->m_posClutched = pDev->m_pos;
            pDev->m_rotClutched = pDev->m_rot;
        }
        else{
            simGripper->btn_cam_rising_edge = true;
        }
        if(g_clutch_btn_pressed){
            if(simGripper->btn_clutch_rising_edge){
                simGripper->btn_clutch_rising_edge = false;
                simGripper->m_posRefOrigin = simGripper->m_posRef / simGripper->m_workspaceScaleFactor;
                simGripper->m_rotRefOrigin = simGripper->m_rotRef;
            }
            pDev->m_posClutched = pDev->m_pos;
            pDev->m_rotClutched = pDev->m_rot;
        }
        else{
            simGripper->btn_clutch_rising_edge = true;
        }

        simGripper->m_posRef = simGripper->m_posRefOrigin +
                (devCams[0]->getLocalRot() * (pDev->m_pos - pDev->m_posClutched));
        if (!g_coordApp->m_use_cam_frame_rot){
            simGripper->m_rotRef = simGripper->m_rotRefOrigin * devCams[0]->getLocalRot() *
                    cTranspose(pDev->m_rotClutched) * pDev->m_rot *
                    cTranspose(devCams[0]->getLocalRot());
        }
        else{
            simGripper->m_rotRef = pDev->m_rot;
        }
        simGripper->m_posRef.mul(simGripper->m_workspaceScaleFactor);

        // update position of simulated gripper
        simGripper->updateMeasuredPose();


        double P_lin = simGripper->m_rootLink->m_controller.getP_lin();
        double D_lin = simGripper->m_rootLink->m_controller.getD_lin();
        double P_ang = simGripper->m_rootLink->m_controller.getP_ang();
        double D_ang = simGripper->m_rootLink->m_controller.getD_ang();

        dposLast = dpos;
        dpos = simGripper->m_posRef - simGripper->m_pos;
        ddpos = (dpos - dposLast) / dt;

        drotLast = drot;
        drot = cTranspose(simGripper->m_rot) * simGripper->m_rotRef;
        ddrot = (cTranspose(drot) * drotLast);

        double angle, dangle;
        cVector3d axis, daxis;
        drot.toAxisAngle(axis, angle);
        ddrot.toAxisAngle(daxis, dangle);

        cVector3d force, torque;

        force  = - g_force_enable * simGripper->K_lh_ramp * (P_lin * dpos + D_lin * ddpos);
        torque = - g_force_enable * simGripper->K_ah_ramp * (P_ang * angle * axis);

        pDev->applyWrench(force, torque);

        if (simGripper->K_lh_ramp < simGripper->K_lh)
        {
            simGripper->K_lh_ramp = simGripper->K_lh_ramp + 0.1 * dt * simGripper->K_lh;
        }
        else
        {
            simGripper->K_lh_ramp = simGripper->K_lh;
        }

        if (simGripper->K_ah_ramp < simGripper->K_ah)
        {
            simGripper->K_ah_ramp = simGripper->K_ah_ramp + 0.1 * dt * simGripper->K_ah;
        }
        else
        {
            simGripper->K_ah_ramp = simGripper->K_ah;
        }

    }
    // exit haptics thread
}
