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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CHapticDeviceHandlerH
#define CHapticDeviceHandlerH
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CHapticDeviceHandler.h

    \brief
    Implements a universal haptic device handler.
*/
//==============================================================================

//------------------------------------------------------------------------------
//! Maximum number of devices that can be connected at the same time.
const unsigned int C_MAX_HAPTIC_DEVICES = 16;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cHapticDeviceHandler
    \ingroup    devices  

    \brief
    This class implements a universal haptic device handler.

    \details
    This class implements a manager which lists the different devices
    available on the computer and provides handles to them.
*/
//==============================================================================
class cHapticDeviceHandler
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cHapticDeviceHandler.
    cHapticDeviceHandler();

    //! Destructor of cHapticDeviceHandler.
    virtual ~cHapticDeviceHandler();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of devices currently connected to the computer.
    unsigned int getNumDevices() { return (m_numDevices); }

    //! This method updates information about the devices that are currently connected to the computer.
    void update();

    //! This method returns the specifications of the i-th device.
    bool getDeviceSpecifications(cHapticDeviceInfo& a_deviceSpecifications, 
        unsigned int a_index = 0);

    //! This method returns a handle to the i-th device, if available.
    bool getDevice(cGenericHapticDevicePtr&,
        unsigned int a_index = 0);


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //--------------------------------------------------------------------------

private:

    //! Number of devices.
    unsigned int m_numDevices;

    //! Array of available haptic devices.
    cGenericHapticDevicePtr m_devices[C_MAX_HAPTIC_DEVICES];

    //! A default device with no functionalities.
    cGenericHapticDevicePtr m_nullHapticDevice;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
