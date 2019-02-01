//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

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

    * Neither the name of CHAI3D nor the names of its contributors may
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

    \author    <http://www.aimlab.wpi.edu>
    \author    Adnan Munawar
    \version   3.2.1 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
//---------------------------------------------------------------------------
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

// bullet static walls and ground
cBulletStaticPlane* g_bulletGround;

cBulletStaticPlane* g_bulletBoxWallX[2];
cBulletStaticPlane* g_bulletBoxWallY[2];
cBulletStaticPlane* g_bulletBoxWallZ[1];

afMultiBody *g_afMultiBody;
cBulletSoftMultiMesh* g_softBody;
afWorld *g_afWorld;

cVector3d g_camPos(0,0,0);
cVector3d g_dev_vel;
cMatrix3d g_cam_rot_last[10], g_dev_rot_last[10], g_dev_rot_cur[10];
bool _cam_pressed[10];
double g_dt_fixed = 0;
bool g_force_enable = true;
// Default switch index for clutches


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------


// a camera to render the world in the window display
cCamera* g_camera;

// a light source to illuminate the objects in the world
cSpotLight *g_light;

// a label to display the rates [Hz] at which the simulation is running
cLabel* g_labelRates;
cLabel* g_labelDevRates[10];
cLabel* g_labelTimes;
cLabel* g_labelModes;
cLabel* g_labelBtnAction;
std::string g_btn_action_str = "";
bool g_cam_btn_pressed = false;
bool g_clutch_btn_pressed = false;
cPrecisionClock g_clockWorld;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool g_simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool g_simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter g_freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter g_freqCounterHaptics;

// haptic thread
cThread* g_hapticsThreads[10];
// bullet simulation thread
cThread* g_bulletSimThread;

// a handle to window display context
GLFWwindow* g_window = NULL;

// current width of window
int g_width = 0;

// current height of window
int g_height = 0;

// swap interval for the display context (vertical synchronization)
int g_swapInterval = 1;

// root resource path
string resourceRoot;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void*);

//this function contains the main Bullet Simulation loop
void updateBulletSim(void);

// this function closes the application
void close(void);

const int MAX_DEVICES = 10;


///
/// \brief This class encapsulates each haptic device in isolation and provides methods to get/set device
/// state/commands, button's state and grippers state if present
///
class Device{
public:
    Device(){}
    virtual ~Device();
    virtual cVector3d measured_pos();
    virtual cMatrix3d measured_rot();
    virtual cVector3d measured_lin_vel();
    virtual cVector3d mearured_ang_vel();
    virtual double measured_gripper_angle();
    virtual void apply_wrench(cVector3d a_force, cVector3d a_torque);
    virtual bool is_button_pressed(int button_index);
    virtual bool is_button_press_rising_edge(int button_index);
    virtual bool is_button_press_falling_edge(int button_index);
    void enable_force_feedback(bool enable){m_dev_force_enabled = enable;}
    cShapeSphere* create_cursor(cBulletWorld* a_world);
    cBulletSphere* create_af_cursor(cBulletWorld* a_world, std::string a_name);
    cGenericHapticDevicePtr m_hDevice;
    cHapticDeviceInfo m_hInfo;
    cVector3d m_pos, m_posClutched, m_vel, m_avel;
    cMatrix3d m_rot, m_rotClutched;
    double m_workspace_scale_factor;
    cShapeSphere* m_cursor = NULL;
    cBulletSphere* m_af_cursor = NULL;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

private:
    std::mutex m_mutex;
    void update_cursor_pose();
    bool m_dev_force_enabled = true;
};

///
/// \brief Device::create_cursor
/// \param a_world
/// \return
///
cShapeSphere* Device::create_cursor(cBulletWorld* a_world){
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

Device::~Device(){
}

///
/// \brief Device::create_af_cursor
/// \param a_world
/// \param a_name
/// \return
///
cBulletSphere* Device::create_af_cursor(cBulletWorld *a_world, string a_name){
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
/// \brief Device::measured_pos
/// \return
///
cVector3d Device::measured_pos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getPosition(m_pos);
    update_cursor_pose();
    return m_pos;
}

cMatrix3d Device::measured_rot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getRotation(m_rot);
    return m_rot;
}

///
/// \brief Device::update_cursor_pose
///
void Device::update_cursor_pose(){
    if(m_cursor){
        m_cursor->setLocalPos(m_pos * m_workspace_scale_factor);
        m_cursor->setLocalRot(m_rot);
    }
    if(m_af_cursor){
        m_af_cursor->setLocalPos(m_pos * m_workspace_scale_factor);
        m_af_cursor->setLocalRot(m_rot);
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_af_cursor->m_afObjectPtr->set_userdata_desc("haptics frequency");
        m_af_cursor->m_afObjectPtr->set_userdata(m_freq_ctr.getFrequency());
#endif
    }
}

///
/// \brief Device::measured_lin_vel
/// \return
///
cVector3d Device::measured_lin_vel(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getLinearVelocity(m_vel);
    return m_vel;
}

///
/// \brief Device::mearured_ang_vel
/// \return
///
cVector3d Device::mearured_ang_vel(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getAngularVelocity(m_avel);
    return m_avel;
}

///
/// \brief Device::measured_gripper_angle
/// \return
///
double Device::measured_gripper_angle(){
    std::lock_guard<std::mutex> lock(m_mutex);
    double angle;
    m_hDevice->getGripperAngleRad(angle);
    return angle;
}

///
/// \brief Device::is_button_pressed
/// \param button_index
/// \return
///
bool Device::is_button_pressed(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    return status;
}

///
/// \brief Device::is_button_press_rising_edge
/// \param button_index
/// \return
///
bool Device::is_button_press_rising_edge(int button_index){
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
/// \brief Device::is_button_press_falling_edge
/// \param button_index
/// \return
///
bool Device::is_button_press_falling_edge(int button_index){
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
/// \brief Device::apply_wrench
/// \param force
/// \param torque
///
void Device::apply_wrench(cVector3d force, cVector3d torque){
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
class Sim{
public:
    Sim(){
        m_workspaceScaleFactor = 30.0;
        K_lh = 0.02;
        K_ah = 0.03;
        K_lc = 200;
        K_ac = 30;
        B_lc = 5.0;
        B_ac = 3.0;
        K_lh_ramp = 0.0;
        K_ah_ramp = 0.0;
        K_lc_ramp = 0.0;
        K_ac_ramp = 0.0;
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
    void set_sim_params(cHapticDeviceInfo &a_hInfo, Device* a_dev);
    inline void set_loop_exec_flag(){m_loop_exec_flag=true;}
    inline void clear_loop_exec_flag(){m_loop_exec_flag = false;}
    inline bool is_loop_exec(){return m_loop_exec_flag;}
    inline double get_workspace_scale_factor(){return m_workspaceScaleFactor;}
    cVector3d m_posRef, m_posRefOrigin;
    cMatrix3d m_rotRef, m_rotRefOrigin;
    double m_workspaceScaleFactor;
    double K_lh;                    //Linear Haptic Stiffness Gain
    double K_ah;                    //Angular Haptic Stiffness Gain
    double K_lh_ramp;               //Linear Haptic Stiffness Gain Ramped
    double K_ah_ramp;               //Angular Haptic Stiffness Gain Ramped
    double K_lc_ramp;               //Linear Haptic Stiffness Gain Ramped
    double K_ac_ramp;               //Angular Haptic Stiffness Gain Ramped
    double K_lc;                    //Linear Controller Stiffness Gain
    double K_ac;                    //Angular Controller Stiffness Gain
    double B_lc;                    //Linear Controller Damping Gain
    double B_ac;                    //Angular Controller Damping Gain
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
/// \brief Sim::set_sim_params
/// \param a_hInfo
/// \param a_dev
///
void Sim::set_sim_params(cHapticDeviceInfo &a_hInfo, Device* a_dev){
    double maxStiffness	= a_hInfo.m_maxLinearStiffness / m_workspaceScaleFactor;

    // clamp the force output gain to the max device stiffness
    K_lh = cMin(K_lh, maxStiffness / K_lc);
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
        a_dev->enable_force_feedback(false);
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
/// \brief This class encapsulates a single Gripper, simulated in Bullet and provides methods to get/set state/commands
///        for interface with the haptics device
///
class ToolGripper: public Sim, public afGripper{
public:
    ToolGripper(cBulletWorld *a_chaiWorld);
    ~ToolGripper(){}
    bool loadFromAFMB(std::string a_gripper_name, std::string a_device_name);
    virtual cVector3d measured_pos();
    virtual cMatrix3d measured_rot();
    virtual void update_measured_pose();
    virtual inline void apply_force(cVector3d force){if (!m_rootLink->m_af_enable_position_controller) m_rootLink->addExternalForce(force);}
    virtual inline void apply_torque(cVector3d torque){if (!m_rootLink->m_af_enable_position_controller) m_rootLink->addExternalTorque(torque);}
    bool is_wrench_set();
    void clear_wrench();
    void offset_gripper_angle(double offset);
    void setGripperAngle(double angle, double dt=0.001);
    cVector3d m_pos;
    cMatrix3d m_rot;
    double m_gripper_angle;

    std::mutex m_mutex;
};

///
/// \brief ToolGripper::ToolGripper
/// \param a_chaiWorld
/// \param a_gripper_name
/// \param a_device_name
///
ToolGripper::ToolGripper(cBulletWorld *a_chaiWorld): afGripper (a_chaiWorld){
    m_gripper_angle = 0.5;
}

///
/// \brief ToolGripper::loadFromAFMB
/// \param a_gripper_name
/// \param a_device_name
/// \return
///
bool ToolGripper::loadFromAFMB(std::string a_gripper_name, std::string a_device_name){
    std::string config = getGripperConfig(a_device_name);
    bool res = loadMultiBody(config, a_gripper_name, a_device_name);
    m_rootLink = getRootRigidBody();
    return res;
}

///
/// \brief ToolGripper::measured_pos
/// \return
///
cVector3d ToolGripper::measured_pos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalPos();
}

///
/// \brief ToolGripper::measured_rot
/// \return
///
cMatrix3d ToolGripper::measured_rot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalRot();
}

///
/// \brief ToolGripper::update_measured_pose
///
void ToolGripper::update_measured_pose(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pos  = m_rootLink->getLocalPos();
    m_rot = m_rootLink->getLocalRot();
}

///
/// \brief ToolGripper::set_gripper_angle
/// \param angle
/// \param dt
///
void ToolGripper::setGripperAngle(double angle, double dt){
    m_rootLink->setAngle(angle, dt);
}

///
/// \brief ToolGripper::offset_gripper_angle
/// \param offset
///
void ToolGripper::offset_gripper_angle(double offset){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gripper_angle += offset;
    m_gripper_angle = cClamp(m_gripper_angle, 0.0, 1.0);
}

///
/// \brief ToolGripper::is_wrench_set
/// \return
///
bool ToolGripper::is_wrench_set(){
    btVector3 f = m_rootLink->m_bulletRigidBody->getTotalForce();
    btVector3 n = m_rootLink->m_bulletRigidBody->getTotalTorque();
    if (f.isZero()) return false;
    else return true;
}

///
/// \brief ToolGripper::clear_wrench
///
void ToolGripper::clear_wrench(){
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
/// \brief This is a higher level class that queries the number of haptics devices available on the sytem
/// and on the Network for dVRK devices and creates a single Bullet Gripper and a Device Handle for
/// each device.
///
class Coordination{
public:
    Coordination(cBulletWorld* a_bullet_world, int a_max_load_devs = MAX_DEVICES);
    ~Coordination();
    bool retrieve_device_handle(uint dev_num);
    bool create_bullet_gripper(uint dev_num);
    void close_devices();

    double increment_K_lh(double a_offset);
    double increment_K_ah(double a_offset);
    double increment_K_lc(double a_offset);
    double increment_K_ac(double a_offset);
    double increment_B_lc(double a_offset);
    double increment_B_ac(double a_offset);
    bool are_all_haptics_loop_exec();
    int num_of_haptics_loop_execd();
    void clear_all_haptics_loop_exec_flags();

    void next_mode();
    void prev_mode();

    std::shared_ptr<cHapticDeviceHandler> m_deviceHandler;
    ToolGripper *m_bulletGrippers[MAX_DEVICES];
    Device m_hapticDevices[MAX_DEVICES];
    uint m_num_devices;
    uint m_num_grippers;
    cBulletWorld* m_bulletWorld;

    // bool to enable the rotation of tool be in camera frame. i.e. Orienting the camera
    // re-orients the tool.
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
};

///
/// \brief Coordination::Coordination
/// \param a_bullet_world
/// \param a_max_load_devs
///
Coordination::Coordination(cBulletWorld* a_bullet_world, int a_max_load_devs){
    m_bulletWorld = NULL;
    m_deviceHandler = NULL;
    m_bulletWorld = a_bullet_world;
    if (a_max_load_devs > 0){
        m_deviceHandler.reset(new cHapticDeviceHandler());
        int numDevs = m_deviceHandler->getNumDevices();
        m_num_devices = a_max_load_devs < numDevs ? a_max_load_devs : numDevs;
        m_num_grippers = m_num_devices;
        std::cerr << "Num of devices " << m_num_devices << std::endl;
        for (uint i = 0; i < m_num_grippers; i++){
            m_deviceHandler->getDeviceSpecifications(m_hapticDevices[i].m_hInfo, i);
            retrieve_device_handle(i);
            if (!create_bullet_gripper(i)){
                delete m_bulletGrippers[i];
                m_num_grippers--;
            }
        }
    }
    else{
        m_num_devices = 0;
        m_num_grippers = 0;
    }
    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

Coordination::~Coordination(){
}

///
/// \brief Coordination::next_mode
///
void Coordination::next_mode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief Coordination::prev_mode
///
void Coordination::prev_mode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief Coordination::retrieve_device_handle
/// \param dev_num
/// \return
///
bool Coordination::retrieve_device_handle(uint dev_num){
    bool result = m_deviceHandler->getDevice(m_hapticDevices[dev_num].m_hDevice, dev_num);
    if (result){
        m_hapticDevices[dev_num].m_hDevice->open();
        std::string name = "Device" + std::to_string(dev_num+1);
        //        m_hapticDevices[i].create_cursor(m_bulletWorld);
        m_hapticDevices[dev_num].create_af_cursor(m_bulletWorld, name);
    }
    return result;
}

///
/// \brief Coordination::create_bullet_gripper
/// \param dev_num
///
bool Coordination::create_bullet_gripper(uint dev_num){
    std::ostringstream dev_str;
    dev_str << (dev_num + 1);
    std::string gripper_name = "Gripper" + dev_str.str();
    m_bulletGrippers[dev_num] = new ToolGripper(m_bulletWorld);
    bool res = m_bulletGrippers[dev_num]->loadFromAFMB(gripper_name, m_hapticDevices[dev_num].m_hInfo.m_modelName);
    if (res){
        m_bulletGrippers[dev_num]->set_sim_params(m_hapticDevices[dev_num].m_hInfo, & m_hapticDevices[dev_num]);
        m_hapticDevices[dev_num].m_workspace_scale_factor = m_bulletGrippers[dev_num]->get_workspace_scale_factor();
        cVector3d localGripperPos = m_bulletGrippers[dev_num]->m_rootLink->getLocalPos();
        double l,w,h;
        m_bulletGrippers[dev_num]->getEnclosureExtents(l,w,h);
        if (localGripperPos.length() == 0.0){
            double x = (int(dev_num / 2.0) * 0.8);
            double y = (dev_num % 2) ? +0.4 : -0.4;
            x /= m_bulletGrippers[dev_num]->m_workspaceScaleFactor;
            y /= m_bulletGrippers[dev_num]->m_workspaceScaleFactor;
            m_bulletGrippers[dev_num]->m_posRefOrigin.set(x, y, 0);
        }
    }
    return res;
}

///
/// \brief Coordination::close_devices
///
void Coordination::close_devices(){
    for (int i = 0 ; i < m_num_grippers ; i++){
        m_hapticDevices[i].m_hDevice->close();
    }
}

///
/// \brief Coordination::num_of_haptics_loop_execd
/// \return
///
int Coordination::num_of_haptics_loop_execd(){
    int num_devs_loop_execd = 0;
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->is_loop_exec()) num_devs_loop_execd++;
    }
    return num_devs_loop_execd;
}

///
/// \brief Coordination::are_all_haptics_loop_exec
/// \return
///
bool Coordination::are_all_haptics_loop_exec(){
    bool flag = true;
    for (int i = 0 ; i < m_num_grippers ; i++){
        flag &= m_bulletGrippers[i]->is_loop_exec();
    }
    return flag;
}

///
/// \brief Coordination::clear_all_haptics_loop_exec_flags
///
void Coordination::clear_all_haptics_loop_exec_flags(){
    for (int i = 0 ; i < m_num_grippers ; i++){
        m_bulletGrippers[i]->clear_loop_exec_flag();
    }
}

///
/// \brief Coordination::increment_K_lh
/// \param a_offset
/// \return
///
double Coordination::increment_K_lh(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->K_lh + a_offset <= 0)
        {
            m_bulletGrippers[i]->K_lh = 0.0;
        }
        else{
            m_bulletGrippers[i]->K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->K_lh;
        g_btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief Coordination::increment_K_ah
/// \param a_offset
/// \return
///
double Coordination::increment_K_ah(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->K_ah + a_offset <=0){
            m_bulletGrippers[i]->K_ah = 0.0;
        }
        else{
            m_bulletGrippers[i]->K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->K_ah;
        g_btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief Coordination::increment_K_lc
/// \param a_offset
/// \return
///
double Coordination::increment_K_lc(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->K_lc + a_offset <=0){
            m_bulletGrippers[i]->K_lc = 0.0;
        }
        else{
            m_bulletGrippers[i]->K_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->K_lc;
        g_btn_action_str = "K_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief Coordination::increment_K_ac
/// \param a_offset
/// \return
///
double Coordination::increment_K_ac(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->K_ac + a_offset <=0){
            m_bulletGrippers[i]->K_ac = 0.0;
        }
        else{
            m_bulletGrippers[i]->K_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->K_ac;
        g_btn_action_str = "K_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief Coordination::increment_B_lc
/// \param a_offset
/// \return
///
double Coordination::increment_B_lc(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->B_lc + a_offset <=0){
            m_bulletGrippers[i]->B_lc = 0.0;
        }
        else{
            m_bulletGrippers[i]->B_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->B_lc;
        g_btn_action_str = "B_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief Coordination::increment_B_ac
/// \param a_offset
/// \return
///
double Coordination::increment_B_ac(double a_offset){
    for (int i = 0 ; i < m_num_grippers ; i++){
        if (m_bulletGrippers[i]->B_ac + a_offset <=0){
            m_bulletGrippers[i]->B_ac = 0.0;
        }
        else{
            m_bulletGrippers[i]->B_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_grippers > 0){
        a_offset = m_bulletGrippers[m_num_grippers-1]->B_ac;
        g_btn_action_str = "B_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
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


std::shared_ptr<Coordination> g_coordApp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===========================================================================
/*
    Application:    08-bullet-coordination.cpp

    This Application allows multi-manual tasks using several haptics devices.
    Each device can perturb or control the dynamic bodies in the simulation
    environment. The objects in the simulation are exposed via Asynchoronous
    Framework (AF) to allow query and control via external applications.
 */
//===========================================================================
int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------
    namespace p_opt = boost::program_options;

    p_opt::options_description cmd_opts("Coordination Application Usage");
    cmd_opts.add_options()
            ("help,h", "Show help")
            ("ndevs,n", p_opt::value<int>(), "Number of Haptic Devices to Load")
            ("timestep,t", p_opt::value<double>(), "Value in secs for fixed Simulation time step(dt)")
            ("enableforces,f", p_opt::value<bool>(), "Enable Force Feedback on Devices");
    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).run(), var_map);
    p_opt::notify(var_map);

    int num_devices_to_load = MAX_DEVICES;
    if(var_map.count("help")){ std::cout<< cmd_opts << std::endl; return 0;}
    if(var_map.count("ndevs")){ num_devices_to_load = var_map["ndevs"].as<int>();}
    if (var_map.count("timestep")){ g_dt_fixed = var_map["timestep"].as<double>();}
    if (var_map.count("enableforces")){ g_force_enable = var_map["enableforces"].as<bool>();}

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Application: 08-bullet-coordination" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << endl << endl;

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

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

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

    // create display context
    g_window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!g_window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(g_window, &g_width, &g_height);

    // set position of window
    glfwSetWindowPos(g_window, x, y);

    // set key callback
    glfwSetKeyCallback(g_window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(g_window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(g_window);

    // sets the swap interval for the current display context
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


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a dynamic world.
    g_bulletWorld = new cBulletWorld("World");

    // set the background color of the environment
    g_bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    g_camera = new cCamera(g_bulletWorld);
    g_bulletWorld->addChild(g_camera);

    // position and orient the camera
    g_camera->set(cVector3d(4.0, 0.0, 2.0),    // camera position (eye)
                  cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                  cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    g_camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    g_camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    g_camera->setStereoEyeSeparation(0.02);
    g_camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    g_camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    g_light = new cSpotLight(g_bulletWorld);

    // attach light to camera
    g_bulletWorld->addChild(g_light);

    // enable light source
    g_light->setEnabled(true);

    // position the light source
    g_light->setLocalPos( 0, -0.5, 2.5);

    // define the direction of the light beam
    g_light->setDir(0, 0, -1.0);

    // set uniform concentration level of light
    g_light->setSpotExponent(0.3);

    // enable this light source to generate shadows
    g_light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    g_light->m_shadowMap->setQualityHigh();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    g_light->setCutOffAngleDeg(45);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    g_labelRates = new cLabel(font);
    g_labelTimes = new cLabel(font);
    g_labelModes = new cLabel(font);
    g_labelBtnAction = new cLabel(font);
    g_labelRates->m_fontColor.setBlack();
    g_labelTimes->m_fontColor.setBlack();
    g_labelModes->m_fontColor.setBlack();
    g_labelBtnAction->m_fontColor.setBlack();
    g_camera->m_frontLayer->addChild(g_labelRates);
    g_camera->m_frontLayer->addChild(g_labelTimes);
    g_camera->m_frontLayer->addChild(g_labelModes);
    g_camera->m_frontLayer->addChild(g_labelBtnAction);

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////
    // set some gravity
    g_bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));


    //////////////////////////////////////////////////////////////////////////
    // AF MULTIBODY HANDLER
    //////////////////////////////////////////////////////////////////////////
    g_afWorld = new afWorld(g_bulletWorld);
    if (g_afWorld->loadBaseConfig("../resources/config/coordination.yaml")){
        g_afWorld->loadWorld();

        g_afMultiBody = new afMultiBody(g_bulletWorld);
    //    g_afMultiBody->loadMultiBody();
        g_afMultiBody->loadAllMultiBodies();
    }

    // end puzzle meshes
    //////////////////////////////////////////////////////////////////////////
    // INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////
    // we create 5 static walls to contain the dynamic objects within a limited workspace
    double box_l, box_w, box_h;
    box_l = g_afWorld->getEnclosureLength();
    box_w = g_afWorld->getEnclosureWidth();
    box_h = g_afWorld->getEnclosureHeight();
    g_bulletBoxWallZ[0] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 0.0, -1.0), -0.5 * box_h);
    g_bulletBoxWallY[0] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, -1.0, 0.0), -0.5 * box_w);
    g_bulletBoxWallY[1] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 1.0, 0.0), -0.5 * box_w);
    g_bulletBoxWallX[0] = new cBulletStaticPlane(g_bulletWorld, cVector3d(-1.0, 0.0, 0.0), -0.5 * box_l);
    g_bulletBoxWallX[1] = new cBulletStaticPlane(g_bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.5 * box_l);

    cVector3d worldZ(0.0, 0.0, 1.0);
    cMaterial matPlane;
    matPlane.setWhiteIvory();
    matPlane.setShininess(0.3);
    cVector3d planeNorm;
    cMatrix3d planeRot;

    for (int i = 0 ; i < 2 ; i++){
        planeNorm = cCross(g_bulletBoxWallX[i]->getPlaneNormal(), worldZ);
        planeRot.setAxisAngleRotationDeg(planeNorm, 90);
        g_bulletWorld->addChild(g_bulletBoxWallX[i]);
        cCreatePlane(g_bulletBoxWallX[i], box_h, box_w,
                     g_bulletBoxWallX[i]->getPlaneConstant() * g_bulletBoxWallX[i]->getPlaneNormal(),
                     planeRot);
        g_bulletBoxWallX[i]->setMaterial(matPlane);
        if (i == 0) g_bulletBoxWallX[i]->setTransparencyLevel(0.3, true, true);
        else g_bulletBoxWallX[i]->setTransparencyLevel(0.5, true, true);
    }

    for (int i = 0 ; i < 2 ; i++){
        planeNorm = cCross(g_bulletBoxWallY[i]->getPlaneNormal(), worldZ);
        planeRot.setAxisAngleRotationDeg(planeNorm, 90);
        g_bulletWorld->addChild(g_bulletBoxWallY[i]);
        cCreatePlane(g_bulletBoxWallY[i], box_l, box_h,
                     g_bulletBoxWallY[i]->getPlaneConstant() * g_bulletBoxWallY[i]->getPlaneNormal(),
                     planeRot);
        g_bulletBoxWallY[i]->setMaterial(matPlane);
        g_bulletBoxWallY[i]->setTransparencyLevel(0.5, true, true);
    }

    g_camera->setLocalPos(1.5 + box_l/2, 0, 0.2 + box_h/2);


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create ground plane
    g_bulletGround = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 0.0, 1.0), -0.5 * box_h);

    // add plane to world as we will want to make it visibe
    g_bulletWorld->addChild(g_bulletGround);

    // create a mesh plane where the static plane is located
    cCreatePlane(g_bulletGround, box_l + 0.4, box_w + 0.8, g_bulletGround->getPlaneConstant() * g_bulletGround->getPlaneNormal());

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setGreenChartreuse();
    matGround.m_emission.setGrayLevel(0.3);
    g_bulletGround->setMaterial(matGround);
    g_bulletGround->m_bulletRigidBody->setFriction(1.0);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    g_coordApp = std::make_shared<Coordination>(g_bulletWorld, num_devices_to_load);

    // create a thread which starts the main haptics rendering loop
    int dev_num[10] = {0,1,2,3,4,5,6,7,8,9};
    for (int i = 0 ; i < g_coordApp->m_num_grippers ; i++){
        g_hapticsThreads[i] = new cThread();
        g_hapticsThreads[i]->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS, &dev_num[i]);
        g_labelDevRates[i] = new cLabel(font);
        g_labelDevRates[i]->m_fontColor.setBlack();
        g_labelDevRates[i]->setFontScale(0.8);
        g_camera->m_frontLayer->addChild(g_labelDevRates[i]);
    }

    //create a thread which starts the Bullet Simulation loop
    g_bulletSimThread = new cThread();
    g_bulletSimThread->start(updateBulletSim, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(g_window, g_width, g_height);

    // main graphic loop
    while (!glfwWindowShouldClose(g_window))
    {
        // get width and height of window
        glfwGetWindowSize(g_window, &g_width, &g_height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(g_window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        g_freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(g_window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    g_width = a_width;
    g_height = a_height;
}

//---------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

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

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(g_window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(g_swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(g_window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(g_swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        g_camera->setMirrorVertical(mirroredDisplay);
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
        cout << "[0] - increase angular stiffness" << endl << endl;
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
        printf("linear stiffness:  %f\n", g_coordApp->increment_K_lc(-50));
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        printf("linear stiffness:  %f\n", g_coordApp->increment_K_lc(50));
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_K_ac(-1));
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_K_ac(1));
    }
    else if (a_key == GLFW_KEY_C){
        g_coordApp->m_use_cam_frame_rot = true;
        printf("Gripper Rotation w.r.t Camera Frame:\n");
    }
    else if (a_key == GLFW_KEY_W){
        g_coordApp->m_use_cam_frame_rot = false;
        printf("Gripper Rotation w.r.t World Frame:\n");
    }
    else if (a_key == GLFW_KEY_N){
        g_coordApp->next_mode();
        printf("Changing to next device mode:\n");
    }
    else if (a_key == GLFW_KEY_S){
        auto sbMap = g_afMultiBody->getSoftBodyMap();
        afSoftBodyMap::const_iterator sbIt;
        for (sbIt = sbMap->begin() ; sbIt != sbMap->end(); ++sbIt){
            sbIt->second->toggleSkeletalModelVisibility();
        }
    }
    else if (a_key == GLFW_KEY_V){
        auto rbMap = g_afMultiBody->getRigidBodyMap();
        afRigidBodyMap::const_iterator rbIt;
        for (rbIt = rbMap->begin() ; rbIt != rbMap->end(); ++rbIt){
            rbIt->second->toggleFrameVisibility();
        }
    }
    //    // option - open gripper
    //    else if (a_key == GLFW_KEY_S)
    //    {
    //        grip_angle -= 0.01;
    //        printf("gripper angle:  %f\n", grip_angle);
    //    }
    //    // option - open close gripper
    //    else if (a_key == GLFW_KEY_D)
    //    {
    //        grip_angle += 0.01;
    //        printf("gripper angle:  %f\n", grip_angle);
    //    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    g_simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!g_simulationFinished) { cSleepMs(100); }
    g_bulletSimThread->stop();
    for(int i = 0 ; i < g_coordApp->m_num_grippers ; i ++){
        g_hapticsThreads[i]->stop();
    }

    // delete resources
    g_coordApp->close_devices();
    for(int i = 0 ; i < g_coordApp->m_num_grippers ; i ++){
        delete g_hapticsThreads[i];
    }
    delete g_bulletWorld;
    delete g_afWorld;
    delete g_afMultiBody;
}

//---------------------------------------------------------------------------
cPrecisionClock tClock;
int cnt = 0;
int first_time = true;
const int tElemCnt = 400;
cVector3d n[tElemCnt];
cVector3d p[tElemCnt];
int nElem = 0;
afRigidBody * tBody;
cMesh * tMesh;

void updateMesh(){
    if (first_time == true){
        tClock.start(true);
        tClock.reset();
        tBody = g_afMultiBody->getRidigBody("body2");
        tMesh = (*(tBody->m_meshes))[0];
        nElem = tElemCnt < tMesh->getNumVertices() ? tElemCnt : tMesh->getNumVertices();
        for(int i = 0 ; i < nElem ; i++){
            p[i] = tMesh->m_vertices->getLocalPos(i);
            n[i] = tMesh->m_vertices->getNormal(i);
        }
        first_time = false;
    }
    cnt ++;
    cVector3d dp, dn;
    double t = tClock.getCurrentTimeSeconds();
    double tc = 4.0;
    double scale = 3.0;
    for(int i = 0 ; i < nElem ; i++){
        dn = p[i] * (sin(tc*t)/scale);
        dp = p[i] + dn;
        tMesh->m_vertices->setLocalPos(i, dp);
//        if ((i == 10) && (cnt % 10 == 0)){
//            printf("p %f, %f, %f \n", p[i].x(), p[i].y(), p[i].z());
//            printf("dp %f, %f, %f \n", dp.x(), dp.y(), dp.z());
//            printf("n %f, %f, %f \n", n[i].x(), n[i].y(), n[i].z());
//            printf("sin(t) %f\n", sin(tc*t)/scale);
//            cVector3d tp;
//            tp = tMesh->m_vertices->getLocalPos(i);
//            printf("tp %f, %f, %f \n", tp.x(), tp.y(), tp.z());
//        }
        tMesh->markForUpdate(true);
    }
    tMesh->computeAllNormals();
    if (cnt >= 2000){
        cnt = 0;
    }
}

void renderSb(){
    cColorf col;
    col.setPurpleAmethyst();
    btSoftBody *softBody = g_softBody->getSoftBody();
    col.render();
//    glBegin(GL_POINTS);
//    for (int i = 0 ; i < softBody->m_nodes.size() ; i++){
//        cVector3d v(softBody->m_nodes[i].m_x.x(), softBody->m_nodes[i].m_x.y(), softBody->m_nodes[i].m_x.z());
//        glVertex3dv( (const double *)&v);
////        if (i %300 == 0)
////            printf("%d node pos = %f, %f, %f \n", i, v.x(), v.y(), v.z());
//    }
//    glEnd();
    glBegin(GL_LINES);
    for (int i = 0 ; i < softBody->m_links.size() ; i++){
        cVector3d v1(softBody->m_links[i].m_n[0]->m_x.x(), softBody->m_links[i].m_n[0]->m_x.y(), softBody->m_links[i].m_n[0]->m_x.z());
        cVector3d v2(softBody->m_links[i].m_n[1]->m_x.x(), softBody->m_links[i].m_n[1]->m_x.y(), softBody->m_links[i].m_n[1]->m_x.z());
        glColor4fv( (const float *)&col);

            glVertex3dv( (const double *)&v1);
            glVertex3dv( (const double *)&v2);
//        if (i %300 == 0)
//            printf("%d node pos = %f, %f, %f \n", i, v.x(), v.y(), v.z());
    }
//    glEnd();
//    glEnable(GL_CULL_FACE);
//    glCullFace(GL_BACK);
//    glBegin(GL_TRIANGLES);
//    for (int i = 0 ; i < softBody->m_faces.size() ; i++){
//        cVector3d v1(softBody->m_faces[i].m_n[0]->m_x.x(), softBody->m_faces[i].m_n[0]->m_x.y(), softBody->m_faces[i].m_n[0]->m_x.z());
//        cVector3d v2(softBody->m_faces[i].m_n[1]->m_x.x(), softBody->m_faces[i].m_n[1]->m_x.y(), softBody->m_faces[i].m_n[1]->m_x.z());
//        cVector3d v3(softBody->m_faces[i].m_n[2]->m_x.x(), softBody->m_faces[i].m_n[2]->m_x.y(), softBody->m_faces[i].m_n[2]->m_x.z());
//        glColor4fv( (const float *)&col);
//            glVertex3dv( (const double *)&v1);
//            glVertex3dv( (const double *)&v2);
//            glVertex3dv( (const double *)&v3);
////        if (i %300 == 0)
////            printf("%d node pos = %f, %f, %f \n", i, v.x(), v.y(), v.z());
//    }
    glEnd();

}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    g_labelTimes->setText("Wall Time: " + cStr(g_clockWorld.getCurrentTimeSeconds(),2) + " s" +
                          + " / "+" Simulation Time: " + cStr(g_bulletWorld->getSimulationTime(),2) + " s");
    g_labelRates->setText(cStr(g_freqCounterGraphics.getFrequency(), 0) + " Hz / " + cStr(g_freqCounterHaptics.getFrequency(), 0) + " Hz");
    g_labelModes->setText("MODE: " + g_coordApp->m_mode_str);
    g_labelBtnAction->setText(" : " + g_btn_action_str);

    for (int i = 0 ; i < g_coordApp->m_num_grippers ; i++){
        g_labelDevRates[i]->setText(g_coordApp->m_hapticDevices[i].m_hInfo.m_modelName + ": " + cStr(g_coordApp->m_hapticDevices[i].m_freq_ctr.getFrequency(), 0) + " Hz");
        g_labelDevRates[i]->setLocalPos(10, (int)(g_height - (i+1)*20));
    }

//    updateMesh();

    // update position of label
    g_labelTimes->setLocalPos((int)(0.5 * (g_width - g_labelTimes->getWidth())), 30);
    g_labelRates->setLocalPos((int)(0.5 * (g_width - g_labelRates->getWidth())), 10);
    g_labelModes->setLocalPos((int)(0.5 * (g_width - g_labelModes->getWidth())), 50);
    g_labelBtnAction->setLocalPos((int)(0.5 * (g_width - g_labelModes->getWidth()) + g_labelModes->getWidth()), 50);

    for (size_t gripper_num = 0; gripper_num < g_coordApp->m_num_grippers ; gripper_num ++){
        g_coordApp->m_hapticDevices[gripper_num].m_hDevice->getUserSwitch(g_coordApp->m_bulletGrippers[gripper_num]->act_2_btn, _cam_pressed[gripper_num]);
        if(_cam_pressed[gripper_num] && g_coordApp->m_simModes == MODES::CAM_CLUTCH_CONTROL){
            double scale = 0.3;
            g_dev_vel = g_coordApp->m_hapticDevices[gripper_num].measured_lin_vel();
            g_coordApp->m_hapticDevices[gripper_num].m_hDevice->getRotation(g_dev_rot_cur[gripper_num]);
            g_camera->setLocalPos(g_camera->getLocalPos() + cMul(scale, cMul(g_camera->getGlobalRot(), g_dev_vel)));
            g_camera->setLocalRot(cMul(g_cam_rot_last[gripper_num], cMul(cTranspose(g_dev_rot_last[gripper_num]), g_dev_rot_cur[gripper_num])));
        }
        if(!_cam_pressed[gripper_num]){
            g_cam_rot_last[gripper_num] = g_camera->getGlobalRot();
            g_coordApp->m_hapticDevices[gripper_num].m_hDevice->getRotation(g_dev_rot_last[gripper_num]);
        }
    }

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    g_bulletWorld->updateShadowMaps(false, mirroredDisplay);

    // render world
    g_camera->renderView(g_width, g_height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
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

//---------------------------------------------------------------------------
void updateBulletSim(){
    g_simulationRunning = true;
    g_simulationFinished = false;

    // start haptic device
    g_clockWorld.start(true);
    // main Bullet simulation loop
    unsigned int n = g_coordApp->m_num_grippers;
    std::vector<cVector3d> dpos, ddpos, dposPre;
    std::vector<cMatrix3d> drot, ddrot, drotPre;

    dpos.resize(n); ddpos.resize(n); dposPre.resize(n);
    drot.resize(n); ddrot.resize(n); drotPre.resize(n);

    for(unsigned int i = 0 ; i < n; i ++){
        dpos[i].set(0,0,0); ddpos[i].set(0,0,0); dposPre[i].set(0,0,0);
        drot[i].identity(); ddrot[i].identity(); drotPre[i].identity();
    }
    double sleepHz;
    if (g_dt_fixed > 0.0)
        sleepHz = (1.0/g_dt_fixed);
    else
        sleepHz= 1000;

    RateSleep rateSleep(sleepHz);
    while(g_simulationRunning)
    {
        g_freqCounterHaptics.signal(1);
        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt(true);
        for (unsigned int i = 0 ; i < g_coordApp->m_num_grippers ; i++){
            // update position of tool
            ToolGripper * bGripper = g_coordApp->m_bulletGrippers[i];
            bGripper->update_measured_pose();

            dposPre[i] = dpos[i];
            dpos[i] = bGripper->m_posRef - bGripper->m_pos;
            ddpos[i] = (dpos[i] - dposPre[i]) / dt;

            drotPre[i] = drot[i];
            drot[i] = cTranspose(bGripper->m_rot) * bGripper->m_rotRef;
            ddrot[i] = (cTranspose(drot[i]) * drotPre[i]);

            double angle, dangle;
            cVector3d axis, daxis;
            drot[i].toAxisAngle(axis, angle);
            ddrot[i].toAxisAngle(daxis, dangle);

            cVector3d force, torque;

            force = bGripper->K_lc_ramp * (bGripper->K_lc * dpos[i] + (bGripper->B_lc) * ddpos[i]);
            torque = bGripper->K_ac_ramp * ((bGripper->K_ac * angle) * axis);
            bGripper->m_rot.mul(torque);

            bGripper->apply_force(force);
            bGripper->apply_torque(torque);
            bGripper->setGripperAngle(bGripper->m_gripper_angle, dt);

            if (bGripper->K_lc_ramp < 1.0)
            {
                bGripper->K_lc_ramp = bGripper->K_lc_ramp + 0.5 * dt;
            }
            else
            {
                bGripper->K_lc_ramp = 1.0;
            }

            if (bGripper->K_ac_ramp < 1.0)
            {
                bGripper->K_ac_ramp = bGripper->K_ac_ramp + 0.5 * dt;
            }
            else
            {
                bGripper->K_ac_ramp = 1.0;
            }
        }
        g_bulletWorld->updateDynamics(dt, g_clockWorld.getCurrentTimeSeconds(), g_freqCounterHaptics.getFrequency(), g_coordApp->m_num_grippers);
        g_coordApp->clear_all_haptics_loop_exec_flags();
        rateSleep.sleep();
    }
    g_simulationFinished = true;
}


void updateHaptics(void* a_arg){
    int i = *(int*) a_arg;
    // simulation in now running
    g_simulationRunning = true;
    g_simulationFinished = false;

    // update position and orientation of tool
    Device *hDev = & g_coordApp->m_hapticDevices[i];
    ToolGripper* bGripper = g_coordApp->m_bulletGrippers[i];
    hDev->m_posClutched.set(0.0,0.0,0.0);
    hDev->measured_rot();
    hDev->m_rotClutched.identity();
    bGripper->m_rotRefOrigin = hDev->m_rot;

    cVector3d dpos, ddpos, dposLast;
    cMatrix3d drot, ddrot, drotLast;
    dpos.set(0,0,0); ddpos.set(0,0,0); dposLast.set(0,0,0);
    drot.identity(); ddrot.identity(); drotLast.identity();

    double K_lc_offset = 10;
    double K_ac_offset = 1;
    double B_lc_offset = 1;
    double B_ac_offset = 1;
    double K_lh_offset = 5;
    double K_ah_offset = 1;

    double wait_time = 1.0;
    if (std::strcmp(hDev->m_hInfo.m_modelName.c_str(), "Razer Hydra") == 0 ){
        wait_time = 5.0;
    }

    // main haptic simulation loop
    while(g_simulationRunning)
    {
        hDev->m_freq_ctr.signal(1);
        // Adjust time dilation by computing dt from clockWorld time and the simulationTime
        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt();

        hDev->m_pos = hDev->measured_pos();
        hDev->m_rot = hDev->measured_rot();

        if(bGripper->m_gripper_pinch_btn >= 0){
            if(hDev->is_button_pressed(bGripper->m_gripper_pinch_btn)){
                hDev->enable_force_feedback(true);
            }
        }
        if (hDev->m_hInfo.m_sensedGripper){
            bGripper->m_gripper_angle = hDev->measured_gripper_angle();
        }
        else{
            bGripper->m_gripper_angle = 0.5;
        }

        if(hDev->is_button_press_rising_edge(bGripper->mode_next_btn)) g_coordApp->next_mode();
        if(hDev->is_button_press_rising_edge(bGripper->mode_prev_btn)) g_coordApp->prev_mode();

        bool btn_1_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_1_btn);
        bool btn_1_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_1_btn);
        bool btn_2_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_2_btn);
        bool btn_2_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_2_btn);

        double gripper_offset = 0;
        switch (g_coordApp->m_simModes){
        case MODES::CAM_CLUTCH_CONTROL:
            g_clutch_btn_pressed  = hDev->is_button_pressed(bGripper->act_1_btn);
            g_cam_btn_pressed     = hDev->is_button_pressed(bGripper->act_2_btn);
            if(g_clutch_btn_pressed) g_btn_action_str = "Clutch Pressed";
            if(g_cam_btn_pressed)   {g_btn_action_str = "Cam Pressed";}
            if(btn_1_falling_edge || btn_2_falling_edge) g_btn_action_str = "";
            break;
        case MODES::GRIPPER_JAW_CONTROL:
            if (btn_1_rising_edge) gripper_offset = 0.1;
            if (btn_2_rising_edge) gripper_offset = -0.1;
            bGripper->offset_gripper_angle(gripper_offset);
            break;
        case MODES::CHANGE_CONT_LIN_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_lc(K_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_lc(-K_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_ac(K_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_ac(-K_ac_offset);
            break;
        case MODES::CHANGE_CONT_LIN_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_B_lc(B_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_B_lc(-B_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_B_ac(B_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_B_ac(-B_ac_offset);
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


        if (g_clockWorld.getCurrentTimeSeconds() < wait_time){
            hDev->m_posClutched = hDev->m_pos;
        }

        if(g_cam_btn_pressed){
            if(bGripper->btn_cam_rising_edge){
                bGripper->btn_cam_rising_edge = false;
                bGripper->m_posRefOrigin = bGripper->m_posRef / bGripper->m_workspaceScaleFactor;
                bGripper->m_rotRefOrigin = bGripper->m_rotRef;
            }
            hDev->m_posClutched = hDev->m_pos;
            hDev->m_rotClutched = hDev->m_rot;
        }
        else{
            bGripper->btn_cam_rising_edge = true;
        }
        if(g_clutch_btn_pressed){
            if(bGripper->btn_clutch_rising_edge){
                bGripper->btn_clutch_rising_edge = false;
                bGripper->m_posRefOrigin = bGripper->m_posRef / bGripper->m_workspaceScaleFactor;
                bGripper->m_rotRefOrigin = bGripper->m_rotRef;
            }
            hDev->m_posClutched = hDev->m_pos;
            hDev->m_rotClutched = hDev->m_rot;
        }
        else{
            bGripper->btn_clutch_rising_edge = true;
        }

        bGripper->m_posRef = bGripper->m_posRefOrigin +
                (g_camera->getLocalRot() * (hDev->m_pos - hDev->m_posClutched));
        if (!g_coordApp->m_use_cam_frame_rot){
            bGripper->m_rotRef = bGripper->m_rotRefOrigin * g_camera->getLocalRot() *
                    cTranspose(hDev->m_rotClutched) * hDev->m_rot *
                    cTranspose(g_camera->getLocalRot());
        }
        else{
            bGripper->m_rotRef = hDev->m_rot;
        }
        bGripper->m_posRef.mul(bGripper->m_workspaceScaleFactor);

        // update position of tool
        bGripper->update_measured_pose();

        dposLast = dpos;
        dpos = bGripper->m_posRef - bGripper->m_pos;
        ddpos = (dpos - dposLast) / dt;

        drotLast = drot;
        drot = cTranspose(bGripper->m_rot) * bGripper->m_rotRef;
        ddrot = (cTranspose(drot) * drotLast);

        double angle, dangle;
        cVector3d axis, daxis;
        drot.toAxisAngle(axis, angle);
        ddrot.toAxisAngle(daxis, dangle);

        cVector3d force, torque;

        force  = - g_force_enable * bGripper->K_lh_ramp * (bGripper->K_lc * dpos + (bGripper->B_lc) * ddpos);
        torque = - g_force_enable * bGripper->K_ah_ramp * ((bGripper->K_ac * angle) * axis);

        hDev->apply_wrench(force, torque);

        if (bGripper->K_lh_ramp < bGripper->K_lh)
        {
            bGripper->K_lh_ramp = bGripper->K_lh_ramp + 0.1 * dt * bGripper->K_lh;
        }
        else
        {
            bGripper->K_lh_ramp = bGripper->K_lh;
        }

        if (bGripper->K_ah_ramp < bGripper->K_ah)
        {
            bGripper->K_ah_ramp = bGripper->K_ah_ramp + 0.1 * dt * bGripper->K_ah;
        }
        else
        {
            bGripper->K_ah_ramp = bGripper->K_ah;
        }

        bGripper->set_loop_exec_flag();
    }
    // exit haptics thread
}
