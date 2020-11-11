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
#ifndef AF_INPUT_DEVICES_H
#define AF_INPUT_DEVICES_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "afFramework.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;

// GLOBALS
const int MAX_DEVICES = 10;

//---------------------------------------------------------------------------
// FORWARD DECLARATIONS
//---------------------------------------------------------------------------
class afPhysicalDevice;
class afSimulatedDevice;
class afCollateralControlManager;
struct afCollateralControlUnit;

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
struct afSharedDataStructure{
public:
    afSharedDataStructure();


public:
    void setPosRef(cVector3d a_pos);
    void setRotRef(cMatrix3d a_rot);
    void setPosRefOrigin(cVector3d a_pos);
    void setRotRefOrigin(cMatrix3d a_rot);

    cVector3d getPosRef();
    cMatrix3d getRotRef();
    cVector3d getPosRefOrigin();
    cMatrix3d getRotRefOrigin();

private:
    cVector3d m_posRef;
    cMatrix3d m_rotRef;

    cVector3d m_posRefOrigin;
    cMatrix3d m_rotRefOrigin;

protected:
    std::mutex m_mutex;
};

///
/// \brief The afSimulatedDevice class
///
class afSimulatedDevice: public afSharedDataStructure, public afMultiBody{
public:
    afSimulatedDevice(afWorldPtr a_afWorld);

    ~afSimulatedDevice(){}

    cVector3d getPos();

    cMatrix3d getRot();

    void updatePose();

    inline void applyForce(cVector3d force){
        if (m_rootLink->m_activeControllerType == afControlType::force){
            m_rootLink->addExternalForce(force);
        }
    }

    inline void applyTorque(cVector3d torque){
        if (m_rootLink->m_activeControllerType == afControlType::force){
            m_rootLink->addExternalTorque(torque);
        }
    }

    bool isWrenchSet();

    void clearWrench();

    void offsetGripperAngle(double offset);

    void setGripperAngle(double angle);

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

    // Root link for this simulated device hhhhhhh
    afRigidBodyPtr m_rootLink = nullptr;

    //private:
    //    std::mutex m_mutex;
};


///
/// \brief The afButtons struct
///
struct afButtons{
    int A1; // Action 1 Button
    int A2; // Action 2 Button
    int G1; // Gripper 1 Button
    int NEXT_MODE; // Next Mode Button
    int PREV_MODE; // Prev Mode Button
};

///
/// \brief The afPhysicalDevice class: This class encapsulates each haptic device in isolation and provides methods to get/set device
/// state/commands, button's state and grippers state if present
///
class afPhysicalDevice{
public:
    afPhysicalDevice(afWorldPtr a_afWorld){m_afWorld = a_afWorld;}
    ~afPhysicalDevice();
    virtual bool loadPhysicalDevice(std::string pd_config_file, std::string node_name, cHapticDeviceHandler* hDevHandler, afSimulatedDevice* simDevice, afCollateralControlManager* a_iD);

    virtual bool loadPhysicalDevice(YAML::Node* pd_node, std::string node_name, cHapticDeviceHandler* hDevHandler, afSimulatedDevice* simDevice, afCollateralControlManager* a_iD);

    void createAfCursor(afWorldPtr a_afWorld, std::string a_name, std::string name_space, int minPF, int maxPF);

    inline void enableJointControl(bool a_enable){m_jointControlEnable = a_enable;}

    inline bool isJointControlEnabled(){return m_jointControlEnable;}

    cVector3d getPos();

    cMatrix3d getRot();

    cVector3d getPosClutched();

    cMatrix3d getRotClutched();

    void setPosClutched(cVector3d a_pos);

    void setRotClutched(cMatrix3d a_rot);

    cVector3d getPosPreClutch();

    cMatrix3d getRotPreClutch();

    void setPosPreClutch(cVector3d a_pos);

    void setRotPreClutch(cMatrix3d a_rot);

    cVector3d getPosCamPreClutch();

    cMatrix3d getRotCamPreClutch();

    void setPosCamPreClutch(cVector3d a_pos);

    void setRotCamPreClutch(cMatrix3d a_rot);

    cMatrix3d getSimRotInitial();

    cMatrix3d getSimRotOffset();

    cMatrix3d getSimRotOffsetInverse();

    cVector3d getLinVel();

    cVector3d getAngVel();

    double getGripperAngle();

    void applyWrench(cVector3d a_force, cVector3d a_torque);

    bool isButtonPressed(int button_index);

    bool isButtonPressRisingEdge(int button_index);

    bool isButtonPressFallingEdge(int button_index);

    void enableForceFeedback(bool enable){m_dev_force_enabled = enable;}

public:
    cGenericHapticDevicePtr m_hDevice;
    cHapticDeviceInfo m_hInfo;
    cVector3d m_pos, m_posClutched, m_posPreClutch;
    cMatrix3d m_rot, m_rotClutched, m_rotPreClutch;
    cVector3d m_posCamPreClutch;
    cMatrix3d m_rotCamPreClutch;
    cVector3d m_vel, m_avel;
    double m_workspaceScale;
    afRigidBody* m_afCursor = NULL;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

public:
    double K_lh;                    //Linear Haptic Stiffness Gain
    double K_ah;                    //Angular Haptic Stiffness Gain

    double K_lh_ramp = 0;           //Linear Haptic Stiffness Gain Ramp
    double K_ah_ramp = 0;           //Angular Haptic Stiffness Gain Ramp

    afButtons m_buttons;
    double m_deadBand = 0.001;
    double m_maxForce = 1;
    double m_maxJerk = 1;
    int m_gripper_pinch_btn = -1;
    bool btn_cam_rising_edge;
    bool btn_clutch_rising_edge;
private:
    std::mutex m_mutex;
    void updateCursorPose();
    bool m_dev_force_enabled = false;

public:
    // Initial offset between the simulated end effector and the
    // physical device
    cMatrix3d m_simRotInitial;

    // A transform between simulated and pyhsical devices' frame
    // to store any intended offset
    cMatrix3d m_simRotOffset;

    // Inverse of the simRotOffset
    cMatrix3d m_simRotOffsetInverse;

    // Flag to enable disable showing of reference marker
    bool m_showMarker;

    // Marker size to display
    double m_markerSize;

    // Visual Marker to show the target position of the device
    cMesh* m_refSphere = new cMesh();

    // Declare a controller for the physical device as well
    afCartesianController m_controller;

    // Flag to enable and disable the joint control of simulated end effector
    // for this physical device
    bool m_jointControlEnable;

    // The names of camera that this device can control. The first camera in the
    // list the parent of this device for hand-eye coordination.
    std::vector<std::string> m_pairedCameraNames;

private:
    afWorldPtr m_afWorld = nullptr; // Ref to world ptr
};

///
/// \brief The MODES enum
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
/// \brief The InputControlUnit struct
/// Basically a struct of an IID, its SDE and all the
/// controllable cameras by that SDE-IID
///
struct afCollateralControlUnit{
    afPhysicalDevice* m_physicalDevicePtr = nullptr;
    afSimulatedDevice* m_simulatedDevicePtr = nullptr;
    // The cameras that this particular device Gripper Pair control
    std::vector<afCameraPtr> m_cameras;

    // Label handed by the camera for updating info
    cLabel* m_devFreqLabel = NULL;

    std::string m_name;
};

///
/// \brief This is a higher level class that queries the number of haptics devices available on the sytem
/// and on the Network for dVRK devices and creates a Simulated and Physical Device Handle
///
class afCollateralControlManager{
public:
    afCollateralControlManager(afWorldPtr a_afWorld);
    ~afCollateralControlManager();

    // Get an instance of AFWorld from Input Deivces class
    const afWorldPtr getAFWorld(){return m_afWorld;}

    virtual bool loadInputDevices(std::string a_input_devices_config, int a_max_load_devs = MAX_DEVICES);

    virtual bool loadInputDevices(std::string a_input_devices_config, std::vector<int> a_device_indices);

    bool pairCamerasToCCU(afCollateralControlUnit& a_ccu);

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

    std::vector<afCollateralControlUnit*> getCollateralControlUnits(std::vector<std::string> a_device_names);

    std::vector<afCollateralControlUnit*> getAllCollateralControlUnits();

    // Add the index of a claimed device
    void addClaimedDeviceIndex(int a_idx);

    // Check if a specific index is already claimed or not
    bool checkClaimedDeviceIdx(int a_idx);

public:
    std::shared_ptr<cHapticDeviceHandler> m_deviceHandler;
    std::vector<afCollateralControlUnit> m_collateralControlUnits;

    uint m_numDevices = 0;

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


    std::string m_btn_action_str = "";
    bool m_cam_btn_pressed = false;
    bool m_clutch_btn_pressed = false;

    // Number of input devices loaded. To be used by the devices while launching
    // thier afCommunication
    static int s_inputDeviceCount;

private:
    // Base of the config file location of this Input Device Handler
    boost::filesystem::path m_basePath;
    afWorldPtr m_afWorld = nullptr;
    // Integer index to keep track of device indexes that have already been
    // claimed so that we dont mistakenly claim and already claimed device
    std::vector<int> m_devicesClaimed;
};

}

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
