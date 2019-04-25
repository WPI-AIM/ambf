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
    \version:   $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_INPUT_DEVICES_H
#define AF_INPUT_DEVICES_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "afMultiBody.h"
#include "afGripper.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;

// GLOBALS
const int MAX_DEVICES = 10;

//---------------------------------------------------------------------------
// FORWARD DECLARATIONS
//---------------------------------------------------------------------------
class PhysicalInputDevice;
class SimulatedInputDevice;
class InputDevices;
struct PhySimDevicePair;

///
/// \brief The SoftBodyGrippingConstraint struct
///
struct SoftBodyGrippingConstraint{
    btRigidBody* m_rBody; // Ptr to the RigidBody that is going to be constrained to the softbody
    btSoftBody* m_sBody; // Ptr to SoftBody that the constraint is part of
    std::vector<btSoftBody::Node*> m_nodePtrs; // Node Pointers that this constraint is holding on to
};

///
/// \brief This class encapsulates Simulation Parameters that deal with the interaction between a single haptic device
/// and the related Gripper simulated in Bullet. These Parameters include mapping the device buttons to
/// action/mode buttons, capturing button triggers in addition to presses, mapping the workspace scale factors
/// for a device and so on.
///
struct AsynchronousDataStructure{
public:
    AsynchronousDataStructure();

public:
    cVector3d m_posRef;
    cVector3d m_posRefOrigin;
    cMatrix3d m_rotRef;
    cMatrix3d m_rotRefOrigin;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief The SimulatedGripper class
///
class SimulatedInputDevice: public AsynchronousDataStructure, public afGripper{
public:
    SimulatedInputDevice(afWorldPtr a_afWorld);
    ~SimulatedInputDevice(){}
    bool loadSimulatedGripper(std::string a_config_filename, std::string a_gripper_name ,std::string a_device_name);
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
    double P_lc_ramp;        //Linear Haptic Propotional Gain Ramp
    double P_ac_ramp;        //Angular Haptic Propotional Gain Ramp
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
/// \brief This class encapsulates each haptic device in isolation and provides methods to get/set device
/// state/commands, button's state and grippers state if present
///
class PhysicalInputDevice{
public:
    PhysicalInputDevice(){}
    ~PhysicalInputDevice();
    virtual bool loadPhysicalDevice(std::string pd_config_file, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice, InputDevices* a_iD);
    virtual bool loadPhysicalDevice(YAML::Node* pd_node, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice, InputDevices* a_iD);
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
    double m_workspaceScale;
    cShapeSphere* m_cursor = NULL;
    cBulletSphere* m_af_cursor = NULL;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

public:
    double K_lh;                    //Linear Haptic Stiffness Gain
    double K_ah;                    //Angular Haptic Stiffness Gain

    double K_lh_ramp = 0;           //Linear Haptic Stiffness Gain Ramp
    double K_ah_ramp = 0;           //Angular Haptic Stiffness Gain Ramp

    int act_1_btn;
    int act_2_btn;
    int mode_next_btn;
    int mode_prev_btn;
    int m_gripper_pinch_btn = -1;
    bool btn_cam_rising_edge;
    bool btn_clutch_rising_edge;
private:
    std::mutex m_mutex;
    void updateCursorPose();
    bool m_dev_force_enabled = true;

private:
    InputDevices* m_iDPtr;
};

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

    // Get an instance of AFWorld from Input Deivces class
    const afWorldPtr getAFWorld(){return m_afWorld;}

    virtual bool loadInputDevices(std::string a_inputdevice_config, int a_max_load_devs = MAX_DEVICES);

    boost::filesystem::path getBasePath(){return m_basePath;}

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

    // Add the index of a claimed device
    void addClaimedDeviceIndex(int a_idx);

    // Check if a specific index is already claimed or not
    bool checkClaimedDeviceIdx(int a_idx);

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


    std::string g_btn_action_str = "";
    bool g_cam_btn_pressed = false;
    bool g_clutch_btn_pressed = false;


private:
    // Base of the config file location of this Input Device Handler
    boost::filesystem::path m_basePath;
    afWorldPtr m_afWorld;
    // Integer index to keep track of device indexes that have already been
    // claimed so that we dont mistakenly claim and already claimed device
    std::vector<int> m_devicesClaimed;
};

}

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
