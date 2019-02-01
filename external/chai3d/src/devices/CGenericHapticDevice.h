//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
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

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2141 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericHapticDeviceH
#define CGenericHapticDeviceH
//------------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "math/CMaths.h"
#include "system/CGlobals.h"
#include "timers/CPrecisionClock.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericHapticDevice.h

    \brief
    Implements a base class for haptic devices.
*/
//==============================================================================


//------------------------------------------------------------------------------
// GENERAL CONSTANTS
//------------------------------------------------------------------------------
//! Size of buffer.
const int C_DEVICE_HISTORY_SIZE = 200; // [number of samples]

//! Smallest time interval between two position/status reads from a haptic device.
const double C_DEVICE_MIN_ACQUISITION_TIME = 0.0001;   // [s]

//! Maximum number of joint of a haptic device.
const int C_MAX_DOF = 16;

//------------------------------------------------------------------------------


//==============================================================================
/*!
    Defines the list of devices currently supported by CHAI3D. 
*/
//==============================================================================
enum cHapticDeviceModel
{
    C_HAPTIC_DEVICE_VIRTUAL,
    C_HAPTIC_DEVICE_DELTA_3,
    C_HAPTIC_DEVICE_DELTA_6,
    C_HAPTIC_DEVICE_OMEGA_3,
    C_HAPTIC_DEVICE_OMEGA_6,
    C_HAPTIC_DEVICE_OMEGA_7,
    C_HAPTIC_DEVICE_SIGMA_6P,
    C_HAPTIC_DEVICE_SIGMA_7,
    C_HAPTIC_DEVICE_FALCON,
    C_HAPTIC_DEVICE_XTH_1,
    C_HAPTIC_DEVICE_XTH_2,
    C_HAPTIC_DEVICE_MPR,
    C_HAPTIC_DEVICE_KSD_1,
    C_HAPTIC_DEVICE_VIC_1,
    C_HAPTIC_DEVICE_PHANTOM_TOUCH,
    C_HAPTIC_DEVICE_PHANTOM_OMNI,
    C_HAPTIC_DEVICE_PHANTOM_DESKTOP,
    C_HAPTIC_DEVICE_PHANTOM_15_6DOF,
    C_HAPTIC_DEVICE_PHANTOM_30_6DOF,
    C_HAPTIC_DEVICE_PHANTOM_OTHER,
    C_TRACKER_DEVICE_SIXENSE,
    C_TRACKER_DEVICE_RAZER,
    C_TRACKER_DEVICE_LEAP,
    C_HAPTIC_DEVICE_CUSTOM,
    C_HAPTIC_DVRK_MTM
};


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \struct     cTimestampValue
    \ingroup    devices

    \brief
    This structure stores a double value and a time stamp.
*/
//==============================================================================
struct cTimestampValue
{
    //! Time in seconds when value data was acquired.
    double m_time;

    //! Value data.
    double m_value;
};


//==============================================================================
/*!
    \struct     cTimestampPos
    \ingroup    devices
    
    \brief
    This structure stores a position value and a time stamp.
*/
//==============================================================================
struct cTimestampPos
{
    //! Time in seconds when position data was acquired.
    double m_time;

    //! Position data.
    cVector3d m_pos;
};


//==============================================================================
/*!
    \struct     cTimestampRot
    \ingroup    devices

    \brief
    This structure stores a rotation matrix and a time stamp.
*/
//==============================================================================
struct cTimestampRot
{
    //! Time in seconds when rotation matrix data data was acquired.
    double m_time;

    //! Rotation matrix data.
    cMatrix3d m_rot;
};


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    \struct     cHapticDeviceInfo
    \ingroup    devices

    \brief
    This structure stores all technical specifications of a haptic device.

    \details
    This structure stores all technical specifications of a haptic device.
*/
//==============================================================================
struct cHapticDeviceInfo
{
    //! Haptic device model.
    cHapticDeviceModel m_model;

    //! Name of the haptic device model.
    std::string m_modelName;

    //! Name of the haptic device manufacturer.
    std::string m_manufacturerName;

    //! Maximum continuous force in [N] that can be generated by the haptic device in translation.
    double m_maxLinearForce;

    //! Maximum continuous torque in [N*m] that can be generated by the haptic device in orientation.
    double m_maxAngularTorque;

    //! Maximum continuous force in [N] that can be produced by the haptic gripper.
    double m_maxGripperForce;

    //! Maximum closed loop linear stiffness [N/m] for a simulation running at 1 KHz.
    double m_maxLinearStiffness;

    //! Maximum closed loop angular stiffness [N*m/rad] for a simulation running at 1 KHz.
    double m_maxAngularStiffness;

    //! Maximum closed loop gripper stiffness [N/m] for a simulation running at 1 KHz.
    double m_maxGripperLinearStiffness;

    //! Maximum recommended linear damping factor Kv when using the getVelocity() method from the device class.
    double m_maxLinearDamping;

    //! Maximum recommended angular damping factor Kv when using the getAngularVelocity() method from the device class.
    double m_maxAngularDamping;

    //! Maximum recommended angular damping factor Kv when using the getGripperAngularVelocity() method from the device class.
    double m_maxGripperAngularDamping;

    //! Radius which describes the largest sphere (3D devices) or circle (2D Devices) which can be enclosed inside the physical workspace of the device.
    double m_workspaceRadius;

    //! Maximum open angle of the gripper [rad].
    double m_gripperMaxAngleRad;

    //! If __true__ then device supports position sensing (x,y,z axis), __false__ otherwise.
    bool m_sensedPosition;

    //! If __true__ then device supports rotation sensing. (i.e stylus, pen), __false__ otherwise.
    bool m_sensedRotation;

    //! If __true__ then device supports a sensed gripper interface, __false__ otherwise.
    bool m_sensedGripper;

    //! If __true__ then device provides actuation capabilities for translation degrees of freedom (x,y,z axis), __false__ otherwise.
    bool m_actuatedPosition;

    //! If __true__ then device provides actuation capabilities for orientation degrees of freedom (i.e stylus, wrist, pen), __false__ otherwise.
    bool m_actuatedRotation;

    //! If __true__ then device provides an actuated gripper, __false__ otherwise.
    bool m_actuatedGripper;

    //! If __true__ then the device can be used for left hands, __false__ otherwise.
    bool m_leftHand;

    //! If __true__ then the device can be used for right hands, __false__ otherwise.
    bool m_rightHand;
};

//------------------------------------------------------------------------------
class cGenericHapticDevice;
typedef std::shared_ptr<cGenericHapticDevice> cGenericHapticDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cGenericHapticDevice
    \ingroup    devices

    \brief
    This class implements a base class for haptic devices.

    \details
    This class implements a base class from which all haptic devices are derived.
*/
//==============================================================================
class cGenericHapticDevice : public cGenericDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericHapticDevice.
    cGenericHapticDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cGenericHapticDevice.
    virtual ~cGenericHapticDevice() {};

    //! Shared cGenericHapticDevice allocator.
    static cGenericHapticDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cGenericHapticDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL COMMANDS:
    //--------------------------------------------------------------------------
 
public:

    //! This method opens a connection to the haptic device.
    virtual bool open() { return (C_ERROR); }

    //! This method closes the connection to the haptic device.
    virtual bool close() { return (C_ERROR); }

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false) { return (m_deviceReady); }

    //! This method returns the position of the haptic device.
    virtual bool getPosition(cVector3d& a_position) { a_position.zero(); return (m_deviceReady); }

    //! This method returns the linear velocity of the haptic device.
    virtual bool getLinearVelocity(cVector3d& a_linearVelocity) { a_linearVelocity = m_linearVelocity; return (m_deviceReady); }

    //! This method returns the orientation frame of the haptic device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation) { a_rotation.identity(); return (m_deviceReady); }

    //! This method returns the joint angles of the haptic device.
    virtual bool getJointAnglesRad(double a_jointAnglesRad[C_MAX_DOF]) { return (C_ERROR); }

    //! This method returns the angular velocity of haptic device.
    virtual bool getAngularVelocity(cVector3d& a_angularVelocity) { a_angularVelocity = m_angularVelocity; return (m_deviceReady); }

    //! This method returns the position and orientation of the haptic device through a transformation matrix.
    virtual bool getTransform(cTransform& a_transform);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the gripper angle in degrees [deg].
    inline bool getGripperAngleDeg(double& a_angle) { double angle; bool result = getGripperAngleRad(angle); a_angle = cRadToDeg(angle); return (result); }

    //! This method returns the angular velocity of the gripper. Units are in radians per second [rad/s].
    virtual bool getGripperAngularVelocity(double& a_gripperAngularVelocity) { a_gripperAngularVelocity = m_gripperAngularVelocity; return (m_deviceReady); }

    //! This method returns the sensed force [N] from the haptic device.
    virtual bool getForce(cVector3d& a_force) { a_force = m_prevForce; return (m_deviceReady); }

    //! This method returns the sensed torque [N*m] from the haptic device.
    virtual bool getTorque(cVector3d& a_torque) { a_torque = m_prevTorque; return (m_deviceReady); }

    //! This method returns the sensed torque [N*m] from the force gripper.
    virtual bool getGripperForce(double& a_gripperForce) { a_gripperForce = m_prevGripperForce; return (m_deviceReady); }

    //! This method returns the status of a selected user switch [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitch(int a_switchIndex, bool& a_status);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches) { a_userSwitches = 0; return (m_deviceReady); }

    //! This method returns the technical specifications of this haptic device.
    cHapticDeviceInfo getSpecifications() { return (m_specifications); }

    //! This method enables or disables the virtual gripper switch.
    virtual void setEnableGripperUserSwitch(const bool a_status) { m_gripperUserSwitchEnabled = a_status; }

    //! This method returns the status of the virtual gripper user switch. If __true__, then gripper is used to emulate a user switch. Return __false__ otherwise. 
    virtual bool getEnableGripperUserSwitch() const { return (m_gripperUserSwitchEnabled); }

    //! This method sends a force [N] command to the haptic device.
    bool setForce(const cVector3d& a_force) {  return (setForceAndTorqueAndGripperForce(a_force, cVector3d(0.0,0.0,0.0), 0.0)); }

    //! This method sends a force [N] and torque [N*m] command to the haptic device.
    bool  setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque) { return (setForceAndTorqueAndGripperForce(a_force, a_torque, 0.0)); }

    //! This method sends a force [N], torque [N*m], and gripper force [N] command to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce) { cSleepMs(1); return (m_deviceReady); }


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! This method returns the number of haptic devices available for this class of devices.
    static unsigned int getNumDevices() { return (0); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GENERAL:
    //--------------------------------------------------------------------------

public:

    //! Technical specifications of haptic device.
    cHapticDeviceInfo m_specifications;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - CURRENT VALUES:
    //--------------------------------------------------------------------------

protected:

    //! Last force sent to haptic device.
    cVector3d m_prevForce;

    //! Last torque sent to haptic device.
    cVector3d m_prevTorque;

    //! Last gripper force sent to haptic device.
    double m_prevGripperForce;

    //! Last estimated linear velocity.
    cVector3d m_linearVelocity;

    //! Last estimated angular velocity.
    cVector3d m_angularVelocity;

    //! Last estimated gripper angular velocity.
    double m_gripperAngularVelocity;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - VELOCITY ESTIMATION:
    //--------------------------------------------------------------------------

protected:

    //! History position data of the device.
    cTimestampPos m_historyPos[C_DEVICE_HISTORY_SIZE];

    //! History orientation data of the device.
    cTimestampRot m_historyRot[C_DEVICE_HISTORY_SIZE];

    //! History position of device gripper.
    cTimestampValue m_historyGripper[C_DEVICE_HISTORY_SIZE];

    //! Current index position in history data table.
    int m_indexHistoryPos;

    //! Current index position in history data table.
    int m_indexHistoryRot;

    //! Current index position in history data table.
    int m_indexHistoryGripper;

    //! Last index position used to compute velocity.
    int m_indexHistoryPosWin;

    //! Last index position used to compute velocity.
    int m_indexHistoryRotWin;

    //! Last index position used to compute velocity.
    int m_indexHistoryGripperWin;

    //! Window time interval for measuring linear velocity.
    double m_linearVelocityWindowSize;

    //! Window time interval for measuring angular velocity.
    double m_angularVelocityWindowSize;

    //! Window time interval for measuring gripper velocity.
    double m_gripperVelocityWindowSize;

    //! General clock used to compute velocity signals.
    cPrecisionClock m_clockGeneral;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - GRIPPER USER SWITCH:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then virtual gripper user switch is enabled.
    bool m_gripperUserSwitchEnabled;

    //! Position of the gripper when the user encounters the virtual switch.
    double m_gripperUserSwitchAngleStart;

    //! Position of the gripper when the virtual switch is enabled and the "click" occurs.
    double m_gripperUserSwitchAngleClick;

    //! Maximum force level at the force gripper when the "click" occurs.
    double m_gripperUserSwitchForceClick;

    //! Force level when the gripper is completely closed after the "click" event has occurred.
    double m_gripperUserSwitchForceEngaged;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - SIMULATED GRIPPER FOR DEVICES WITH USER SWITCH:
    //--------------------------------------------------------------------------

protected:

    //! Virtual gripper current angle in radians [rad].
    double m_virtualGripperAngle;

    //! Virtual gripper minimum angle in radians [rad].
    double m_virtualGripperAngleMin;

    //! Virtual gripper maximum angle in radians [rad].
    double m_virtualGripperAngleMax;

    //! Virtual speed value used for simulating the opening and closing of the virtual gripper [rad/s].
    double m_virtualGripperAngularVelocity;

    //! Clock for computing the position/velocity of the virtual gripper.
    cPrecisionClock m_virtualGripperClock;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - VELOCITY ESTIMATION:
    //--------------------------------------------------------------------------

protected:

    //! Estimate linear velocity of handle by passing the latest position.
    void estimateLinearVelocity(cVector3d& a_newPosition);

    //! Estimate angular velocity of handle by passing the latest orientation frame.
    void estimateAngularVelocity(cMatrix3d& a_newRotation);

    //! Estimate velocity of gripper by passing the latest gripper position.
    void estimateGripperVelocity(double a_newGripperPosition);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - GRIPPER USER SWITCH:
    //--------------------------------------------------------------------------

protected:

    //! This method computes the virtual gripper force.
    double computeGripperUserSwitchForce(const double& a_gripperAngle,
                                         const double& a_gripperAngularVelocity);

    //! This method returns the status of gripper user switch. Return __true__ if virtual user switch is engaged, __false_ otherwise.
    bool getGripperUserSwitch();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! This method opens libraries for this class of devices.
    static bool openLibraries() { return (C_SUCCESS); }

    //! This method closes libraries for this class of devices.
    static bool closeLibraries() { return (C_SUCCESS); }
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
