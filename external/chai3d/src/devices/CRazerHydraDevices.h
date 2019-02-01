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
    \author    Adnan Munawar
    \version   3.2.1 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CRazerHydraDevicesH
#define CRazerHydraDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "razer_hydra/hydra.h"
#include <mutex>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       cRazerHydraDevices.h

    \brief
    Implements support for the Sixense Razor Hydra magnetic tracker interface.
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

#define SIXENSE_BUTTON_BUMPER   (0x01<<7)
#define SIXENSE_BUTTON_JOYSTICK (0x01<<8)
#define SIXENSE_BUTTON_1        (0x01<<5)
#define SIXENSE_BUTTON_2        (0x01<<6)
#define SIXENSE_BUTTON_3        (0x01<<3)
#define SIXENSE_BUTTON_4        (0x01<<4)
#define SIXENSE_BUTTON_START    (0x01<<0)
#define SIXENSE_SUCCESS         0
#define SIXENSE_FAILURE        -1
#define SIXENSE_MAX_CONTROLLERS 4


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
class cRazerHydraDevice;
typedef std::shared_ptr<cRazerHydraDevice> cRazerHydraDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cRazerHydraDevice
    \ingroup    devices

    \brief
    This class implements an interface to the Razer Hydra magnetic tracker
    interface.

    \details
    This class implements an interface to the Razer Hydra magnetic tracker
    interface.
*/
//==============================================================================
class cRazerHydraDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cRazerHydraDevice.
    cRazerHydraDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cRazerHydraDevice.
    virtual ~cRazerHydraDevice();

    //! Shared cPhantomDevice allocator.
    static cRazerHydraDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cRazerHydraDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

 public:

    //! This method opens a connection to the device.
    virtual bool open();

    //! This method closes the connection to the device.
    virtual bool close();

    //! This method calibrates the device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 


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

    //! This method opens the libraries for this class of devices.
    static bool openLibraries();

    //! This method closes the libraries for this class of devices.
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
    // PROTECTED MEMBERS - INTERNAL:
    //--------------------------------------------------------------------------
    
protected:

    //! Time guard for data acquisition.
    static cPrecisionClock m_timeguard;

    //! Data acquired from the controller.
//    static sixenseAllControllerData m_data;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL:
    //--------------------------------------------------------------------------

protected:

    //! Update data from controllers.
    bool updateData();
private:
    static std::shared_ptr<razer_hydra::RazerHydra> s_hydra_dev;
    static std::mutex m_mutex;
};

//------------------------------------------------------------------------------
} // namespace chai3ds
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //_DISABLE_SIXENSE_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
