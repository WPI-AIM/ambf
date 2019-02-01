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
    \author    Force Dimension - www.forcedimension.com
    \version   3.2.0 $Rev: 2098 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CDeltaDevicesH
#define CDeltaDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CDeltaDevices.h

    \brief
    Implements support for all Force Dimension and Novint haptic devices.
*/
//==============================================================================

//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

/* devices */
#define DHD_DEVICE_NONE              0
#define DHD_DEVICE_3DOF             31
#define DHD_DEVICE_6DOF             61
#define DHD_DEVICE_6DOF_500         62
#define DHD_DEVICE_DELTA3           63
#define DHD_DEVICE_DELTA6           64
#define DHD_DEVICE_OMEGA            32
#define DHD_DEVICE_OMEGA3           33
#define DHD_DEVICE_OMEGA33          34
#define DHD_DEVICE_OMEGA33_LEFT     36
#define DHD_DEVICE_OMEGA331         35
#define DHD_DEVICE_OMEGA331_LEFT    37
#define DHD_DEVICE_FALCON           60
#define DHD_DEVICE_CONTROLLER       81
#define DHD_DEVICE_CONTROLLER_HR    82
#define DHD_DEVICE_CUSTOM           91
#define DHD_DEVICE_SIGMA331        104
#define DHD_DEVICE_SIGMA331_LEFT   105
#define DHD_DEVICE_SIGMA33P        106
#define DHD_DEVICE_SIGMA33P_LEFT   107

/* status */
#define DHD_ON                     1
#define DHD_OFF                    0

/* device count */
#define DHD_MAX_DEVICE             4

/* TimeGuard return value */
#define DHD_TIMEGUARD              1

/* status count */
#define DHD_MAX_STATUS            16

/* status codes */
#define DHD_STATUS_POWER           0
#define DHD_STATUS_CONNECTED       1
#define DHD_STATUS_STARTED         2
#define DHD_STATUS_RESET           3
#define DHD_STATUS_IDLE            4
#define DHD_STATUS_FORCE           5
#define DHD_STATUS_BRAKE           6
#define DHD_STATUS_TORQUE          7
#define DHD_STATUS_WRIST_DETECTED  8
#define DHD_STATUS_ERROR           9
#define DHD_STATUS_GRAVITY        10
#define DHD_STATUS_TIMEGUARD      11
#define DHD_STATUS_ROTATOR_RESET  12
#define DHD_STATUS_REDUNDANCY     13
#define DHD_STATUS_FORCEOFFCAUSE  14
#define DHD_STATUS_LOCKS          15

/* velocity estimator computation mode */
#define DHD_VELOCITY_WINDOWING     0
#define DHD_VELOCITY_AVERAGING     1

/* encoder count */
#define DHD_MAX_DOF                8

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------
#endif  // WIN32
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cDeltaDevice;
typedef std::shared_ptr<cDeltaDevice> cDeltaDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cDeltaDevice
    \ingroup    devices

    \brief
    This class implements an interface to all Force Dimension and Novint haptic
    devices.

    \details
    This class implements an interface to all Force Dimension and Novint haptic
    devices that include the omega.x, delta.x, sigma.x and Falcon.
*/
//==============================================================================
class cDeltaDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cDeltaDevice.
    cDeltaDevice(unsigned int a_deviceNumber = 0, bool a_external = false);

    //! Destructor of cDeltaDevice.
    virtual ~cDeltaDevice();

    //! Shared cDeltaDevice allocator.
    static cDeltaDevicePtr create(unsigned int a_deviceNumber = 0, bool a_external = false) { return (std::make_shared<cDeltaDevice>(a_deviceNumber, a_external)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method opens a connection to the haptic device.
    virtual bool open();

    //! This method closes the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the haptic device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the linear velocity of the haptic device.
    virtual bool getLinearVelocity(cVector3d& a_linearVelocity);

    //! This method returns the orientation frame of the haptic device.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the joint angles of the haptic device.
    virtual bool getJointAnglesRad(double a_jointAnglesRad[C_MAX_DOF]);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches);

    //! This method sends a force, torque, and gripper force to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);


    //--------------------------------------------------------------------------
    // PUBLIC  STATIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of haptic devices available for this class of devices.
    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! This method opens libraries for this class of devices.
    static bool openLibraries();

    //! This method closes libraries for this class of devices.
    static bool closeLibraries();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DEVICE LIBRARIES:
    //--------------------------------------------------------------------------

protected:

    //! Allocation table for devices of this class. __true__ means that the device has been allocated, __false__ means free.
    static bool s_allocationTable[C_MAX_DEVICES];

    //! Number of instances for this class of devices currently using the libraries.
    static unsigned int s_libraryCounter;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS RESTRICTED TO FORCE DIMENSION DEVICES ONLY:
    //--------------------------------------------------------------------------

public:

    //! This method returns the type of this haptic device.
    int getDeviceType() { return (m_deviceType); }

    //! This method enables or disables forces.
    bool enableForces(bool a_value);

    //! This method orients the device around the y-axis.
    bool setDeviceAngleDeg(double a_angleDeg);

    //! This method assign a vibration to the devices that support this feature.
    bool setVibration(double a_freq, double a_amplitude, int a_type = 0);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

protected:

    //! Device ID number among the Force Dimension devices connected to the computer.
    int m_deviceID;

    //! Device type among the Force Dimension devices.
    int m_deviceType;

    //! Data structure for simulating a low-pass filter on user switches.
    int m_userSwitchCount[8];

    //! Last state of user switch.
    int m_userSwitchStatus[8];

    //! Time guard for user switch.
    cPrecisionClock m_userSwitchClock[8];

    //! Have forces been enable yet since the connection to the device was opened?
    bool m_statusEnableForcesFirstTime;

    //! Was dhdOpen() called outside CHAI3D?
    bool m_external;

    //! Translational workspace offset
    cVector3d m_posWorkspaceOffset;

    //--------------------------------------------------------------------------
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    //--------------------------------------------------------------------------
    //! Status of DHD API calls.
    static bool s_dhdGetDeviceCount;
    static bool s_dhdGetDeviceID;
    static bool s_dhdGetSystemType;
    static bool s_dhdOpenID;
    static bool s_dhdClose;
    static bool s_dhdReset;
    static bool s_dhdGetButton;
    static bool s_dhdGetButtonMask;
    static bool s_dhdGetPosition;
    static bool s_dhdGetLinearVelocity;
    static bool s_dhdGetOrientationRad;
    static bool s_dhdSetTorque;
    static bool s_dhdGetOrientationFrame;
    static bool s_dhdSetForce;
    static bool s_dhdSetForceAndTorque;
    static bool s_dhdSetForceAndGripperForce;
    static bool s_dhdSetForceAndTorqueAndGripperForce;
    static bool s_dhdGetGripperThumbPos;
    static bool s_dhdGetGripperFingerPos;
    static bool s_dhdGetGripperAngleRad;
    static bool s_dhdEnableExpertMode;
    static bool s_dhdDisableExpertMode;
    static bool s_dhdEnableForce;
    static bool s_dhdIsLeftHanded;
    static bool s_dhdSetBaseAngleZDeg;
    static bool s_dhdSetVelocityThreshold;
    static bool s_dhdSetWatchdog;
    static bool s_drdOpenID;
    static bool s_drdClose;
    static bool s_drdIsInitialized;
    static bool s_drdAutoInit;
    static bool s_drdStop;
    static bool s_flagUsingSDK32;
    static bool s_dhdSetDeviceAngleDeg;
    static bool s_dhdGetJointAngles;
    static bool s_dhdSetVibration;

    //--------------------------------------------------------------------------
    #endif  // DOXYGEN_SHOULD_SKIP_THIS
    //--------------------------------------------------------------------------
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif // C_ENABLE_DELTA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
