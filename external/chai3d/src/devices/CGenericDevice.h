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
#ifndef CGenericDeviceH
#define CGenericDeviceH
//------------------------------------------------------------------------------
#include "math/CConstants.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericDevice.h

    \brief
    Implements a device base class.
*/
//==============================================================================

//------------------------------------------------------------------------------
const unsigned int C_MAX_DEVICES = 16;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cGenericDevice
    \ingroup    devices

    \brief
    This class implements an abstract class for hardware devices.

    \details
    This class implements an general interface to communicate with hardware 
    devices. The device can be opened or closed by calling methods open() and 
    close() respectively. A static method getNumDevices() is used to query the 
    number of available devices for its class of devices.
*/
//==============================================================================
class cGenericDevice
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericDevice.
    cGenericDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cGenericDevice.
    virtual ~cGenericDevice() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method opens a connection to this device.
    virtual bool open() { return (C_ERROR); }

    //! This method closes the connection to this device.
    virtual bool close() { return (C_ERROR); }

    //! This method returns __true__ if the device is available for communication, __false__ otherwise.
    bool isDeviceAvailable() { return (m_deviceAvailable); }

    //! This method returns __true__ if the connection to the device has been established by calling method open(), __false__ otherwise.
    bool isDeviceReady() { return (m_deviceReady); }


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of haptic devices available for this class of devices.
    static unsigned int getNumDevices() { return (0); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Flag that indicates if the device is available to the computer.
    bool m_deviceAvailable;

    //! Flag that indicates if connection to device was opened successfully by calling method open().
    bool m_deviceReady;

    //! Device number ID for this category of devices. Value must be equal or bigger than __0__. A value of __-1__ means that the ID has not yet been defined.
    int m_deviceNumber;


    //--------------------------------------------------------------------------
    // PROTECTED STATIC METHODS - DEVICE LIBRARY INITIALIZATION:
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
