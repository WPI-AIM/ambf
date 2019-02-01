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
    \version   3.2.0 $Rev: 2009 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "devices/CHapticDeviceHandler.h"
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#include <process.h>
#endif

#if defined(C_ENABLE_VIRTUAL_DEVICE_SUPPORT)
#include "devices/CVirtualDevice.h"
#endif

#if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)
#include "devices/CDeltaDevices.h"
#endif

#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
#include "devices/CPhantomDevices.h"
#endif

#if defined(C_ENABLE_LEAP_DEVICE_SUPPORT)
#include "devices/CLeapDevices.h"
#endif

#if defined(C_ENABLE_SIXENSE_DEVICE_SUPPORT)
#include "devices/CSixenseDevices.h"
#endif

#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
#include "devices/CMyCustomDevice.h"
#endif

#if defined(C_ENABLE_AMBF_DVRK_DEVICE_SUPPORT)
#include "devices/CdVRKDevices.h"
#endif

#if defined(C_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT)
#include "devices/CRazerHydraDevices.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cHapticDeviceHandler.
*/
//==============================================================================
cHapticDeviceHandler::cHapticDeviceHandler()
{
    // clear number of devices
    m_numDevices = 0;

    // create a null haptic device. a pointer to this device is returned
    // if no device is found. this insures that applications which forget
    // to address the case when no device is connected start sending commands
    // to a NULL pointer...
    m_nullHapticDevice = cGenericHapticDevice::create();

    // clear device table
    unsigned int i;
    for (i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        m_devices[i] = cGenericHapticDevicePtr();
    }

    // search for available haptic devices
    update();
}


//==============================================================================
/*!
    Destructor of cHapticDeviceHandler.
*/
//==============================================================================
cHapticDeviceHandler::~cHapticDeviceHandler()
{
    // clear current list of devices
    for (unsigned int i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        m_devices[i] = cGenericHapticDevicePtr();
    }
}


//==============================================================================
/*!
    This method searches and lists all haptic devices connected to the computer.
*/
//==============================================================================
void cHapticDeviceHandler::update()
{
    // temp variables
    int count;
    cGenericHapticDevicePtr device;

    // clear current list of devices
    m_numDevices = 0;
    for (unsigned int i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        m_devices[i] = nullptr;
    }

    //--------------------------------------------------------------------------
    // search for Force Dimension devices
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cDeltaDevice::getNumDevices();
 
    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cDeltaDevice::create(i);
        if (device->open())
        {
            m_devices[m_numDevices] = device;
            m_numDevices++;
            device->close();
        }
    }

    #endif


    //--------------------------------------------------------------------------
    // search for Leap devices
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_LEAP_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cLeapDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cLeapDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif


    //--------------------------------------------------------------------------
    // search for Sixense devices
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_SIXENSE_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cSixenseDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cSixenseDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif


    //--------------------------------------------------------------------------
    // search for 3D-Systems devices
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cPhantomDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cPhantomDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif

    //--------------------------------------------------------------------------
    // search for MyCustom device
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cMyCustomDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cMyCustomDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif

    //--------------------------------------------------------------------------
    // search for dVRKMTM device
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_AMBF_DVRK_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cDvrkDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cDvrkDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif

    //--------------------------------------------------------------------------
    // search for Razer Hydra device
    //--------------------------------------------------------------------------
    #if defined(C_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT)

    // check for how many devices are available for this class of devices
    count = cRazerHydraDevice::getNumDevices();

    // open all remaining devices
    for (int i=0; i<count; i++)
    {
        device = cRazerHydraDevice::create(i);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif
}


//==============================================================================
/*!
    This method returns the specifications of the i'th device.

    \param  a_deviceSpecifications  Returned result
    \param  a_index                 Index number of the device.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cHapticDeviceHandler::getDeviceSpecifications(cHapticDeviceInfo& a_deviceSpecifications, 
    unsigned int a_index)
{
    if (a_index < m_numDevices)
    {
        a_deviceSpecifications = m_devices[a_index]->getSpecifications();
        return (C_SUCCESS);
    }
    else
    {
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method returns a handle to the i'th device if available.

    \param  a_hapticDevice  Handle to device
    \param  a_index         Index number of the device.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cHapticDeviceHandler::getDevice(cGenericHapticDevicePtr& a_hapticDevice,
    unsigned int a_index)
{
    if (a_index < m_numDevices)
    {
        a_hapticDevice = m_devices[a_index];
        return (C_SUCCESS);
    }
    else
    {
        a_hapticDevice = m_nullHapticDevice;
        return (C_ERROR);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
