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
    \author    Federico Barbagli
    \author    Francois Conti
    \version   3.2.0 $Rev: 1875 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPhantomDevicesH
#define CPhantomDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPhantomDevices.h

    \brief
    Implements support for the 3D-System Phantom haptic device.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cPhantomDevice;
typedef std::shared_ptr<cPhantomDevice> cPhantomDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cPhantomDevice
    \ingroup    devices

    \brief
    This class implements an interface to all 3D-System Phantom haptic 
    devices.

    \details
    This class implements an interface to all 3D-System Phantom haptic 
    devices.
*/
//==============================================================================
class cPhantomDevice : public cGenericHapticDevice
{
  public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cPhantomDevice.
    cPhantomDevice(unsigned int a_deviceNumber);

    //! Destructor of cPhantomDevice.
    virtual ~cPhantomDevice();

    //! Shared cPhantomDevice allocator.
    static cPhantomDevicePtr create(unsigned int a_deviceNumber) { return (std::make_shared<cPhantomDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //! This method open a connection to the haptic device.
    virtual bool open();

    //! This method close the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns orientation frame of the haptic device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 

    //! This method sends a force, torque, and gripper force to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
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
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

protected:

    //! Device ID number among the Phantom devices connected to the computer.
    int m_deviceID;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif // C_ENABLE_PHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
