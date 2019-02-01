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
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 1877 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_LEAP_DEVICE_SUPPORT)
#include "devices/CLeapDevices.h"
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Number of instances for this class of devices currently using the libraries.
unsigned int cLeapDevice::s_libraryCounter = 0;

#if defined(WIN32) | defined(WIN64)
HINSTANCE LeapDLL = NULL;

extern "C"
{
    int  (__cdecl *tdLeapGetNumDevices)      (void);
    int  (__cdecl *tdLeapOpen)               (void);
    int  (__cdecl *tdLeapClose)              (void);
    bool (__cdecl *tdLeapUpdate)             ();
    bool (__cdecl *tdLeapGetPosition)        (cVector3d a_position[2]);
    bool (__cdecl *tdLeapGetRotation)        (cMatrix3d a_rotation[2]);
    bool (__cdecl *tdLeapGetGripperAngleRad) (double a_angle[2]);
    bool (__cdecl *tdLeapGetUserSwitches)    (unsigned int a_userSwitches[2]);
    bool (__cdecl *tdLeapGetFrame)           (void* &a_frame);
}

#else

    void * LeapSO = NULL;

extern "C"
{
    int  (*tdLeapGetNumDevices)      (void);
    int  (*tdLeapOpen)               (void);
    int  (*tdLeapClose)              (void);
    bool (*tdLeapUpdate)             ();
    bool (*tdLeapGetPosition)        (cVector3d a_position[2]);
    bool (*tdLeapGetRotation)        (cMatrix3d a_rotation[2]);
    bool (*tdLeapGetGripperAngleRad) (double a_angle[2]);
    bool (*tdLeapGetUserSwitches)    (unsigned int a_userSwitches[2]);
    bool (*tdLeapGetFrame)           (void* &a_frame);
}

#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method open the libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::openLibraries()
{ 
    // increment number of instances using the libraries for this class of devices
    s_libraryCounter++;

    // if libraries are already initialized, then we are done
    if (s_libraryCounter > 1) return (C_SUCCESS);


    ////////////////////////////////////////////////////////////////////////////
    // initialize libraries
    ////////////////////////////////////////////////////////////////////////////

#if defined(WIN32) | defined(WIN64)

    // load library
#if defined (WIN64)
    if (LeapDLL == NULL) LeapDLL = LoadLibrary("tdLeap64.dll");
#else
    if (LeapDLL == NULL) LeapDLL = LoadLibrary("tdLeap32.dll");
#endif

    // check if DLL loaded correctly
    if (LeapDLL == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    tdLeapGetNumDevices      =                            (int (__cdecl*)(void))GetProcAddress(LeapDLL, "tdLeapGetNumDevices");
    tdLeapOpen               =                            (int (__cdecl*)(void))GetProcAddress(LeapDLL, "tdLeapOpen");
    tdLeapClose              =                            (int (__cdecl*)(void))GetProcAddress(LeapDLL, "tdLeapClose");
    tdLeapUpdate             =                           (bool (__cdecl*)(void))GetProcAddress(LeapDLL, "tdLeapUpdate");
    tdLeapGetPosition        =        (bool (__cdecl*)(cVector3d a_position[2]))GetProcAddress(LeapDLL, "tdLeapGetPosition");
    tdLeapGetRotation        =        (bool (__cdecl*)(cMatrix3d a_rotation[2]))GetProcAddress(LeapDLL, "tdLeapGetRotation");
    tdLeapGetGripperAngleRad =              (bool (__cdecl*)(double a_angle[2]))GetProcAddress(LeapDLL, "tdLeapGetGripperAngleRad");
    tdLeapGetUserSwitches    = (bool (__cdecl*)(unsigned int a_userSwitches[2]))GetProcAddress(LeapDLL, "tdLeapGetUserSwitches");
    tdLeapGetFrame           =                 (bool (__cdecl*)(void* &a_frame))GetProcAddress(LeapDLL, "tdLeapGetFrame");

#endif

#if defined(LINUX) | defined(MACOSX)

    // load shared library
#if defined(LINUX)
    LeapSO = dlopen ("libtdLeap.so", RTLD_NOW);
#else
    LeapSO = dlopen ("tdLeap.dylib", RTLD_NOW);
#endif

    // check that it loaded correctly
    if (LeapSO == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    *(void**)(&tdLeapGetNumDevices     ) = dlsym (LeapSO, "tdLeapGetNumDevices");
    *(void**)(&tdLeapOpen              ) = dlsym (LeapSO, "tdLeapOpen");
    *(void**)(&tdLeapClose             ) = dlsym (LeapSO, "tdLeapClose");
    *(void**)(&tdLeapUpdate            ) = dlsym (LeapSO, "tdLeapUpdate");
    *(void**)(&tdLeapGetPosition       ) = dlsym (LeapSO, "tdLeapGetPosition");
    *(void**)(&tdLeapGetRotation       ) = dlsym (LeapSO, "tdLeapGetRotation");
    *(void**)(&tdLeapGetGripperAngleRad) = dlsym (LeapSO, "tdLeapGetGripperAngleRad");
    *(void**)(&tdLeapGetUserSwitches   ) = dlsym (LeapSO, "tdLeapGetUserSwitches");
    *(void**)(&tdLeapGetFrame          ) = dlsym (LeapSO, "tdLeapGetFrame");

#endif

    // open driver
    tdLeapOpen();

    // report success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method close the libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::closeLibraries()
{
    // sanity check
    if (s_libraryCounter < 1)
    {
        return (C_ERROR);
    }

    // decrement library counter; exit if other objects are still using libraries
    s_libraryCounter--;
    if (s_libraryCounter > 0)
    {
        return (C_SUCCESS);
    }

    // close driver
    tdLeapClose();

    // free libraries
    #if defined(WIN32) | defined(WIN64)
    if ((s_libraryCounter == 0) && (LeapDLL != NULL))
    {
        FreeLibrary(LeapDLL);
        LeapDLL = 0;
    }
    #endif

    #if defined(LINUX) | defined(MACOSX)
    if ((s_libraryCounter == 0) && (LeapSO != NULL))
    {
        dlclose (LeapSO);
        LeapSO = 0;
    }
    #endif

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the number of devices available for this class
    of device.

    \return  Number of available haptic devices.
*/
//==============================================================================
unsigned int cLeapDevice::getNumDevices()
{
    // open libraries
    openLibraries();

    // sanity check
    if (s_libraryCounter < 1) return (C_ERROR);

    // get device count
    unsigned int result = tdLeapGetNumDevices();

    // close libraries
    closeLibraries();

    // return number of devices
    return (2*result);
}


//==============================================================================
/*!
    Constructor of cLeapDevice.
*/
//==============================================================================
cLeapDevice::cLeapDevice(unsigned int a_deviceNumber)
{
    // initialize specifications
    m_specifications.m_model                         = C_TRACKER_DEVICE_LEAP;
    m_specifications.m_manufacturerName              = "Leap Motion";
    m_specifications.m_modelName                     = "Leap Motion";
    m_specifications.m_maxLinearForce                = 0.0;     // [N]
    m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
    m_specifications.m_maxGripperForce               = 0.0;     // [N]
    m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
    m_specifications.m_maxAngularStiffness           = 1.0;     // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N*m/Rad]
    m_specifications.m_maxLinearDamping              = 0.0;     // [N/(m/s)]
    m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping      = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.5;     // [m]
    m_specifications.m_gripperMaxAngleRad            = cDegToRad(30.0);
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = true;
    m_specifications.m_actuatedPosition              = false;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    // device is not yet available or ready
    m_deviceAvailable   = false;
    m_deviceReady       = false;

    // sanity check
    if (a_deviceNumber > 2)
    {
        return;
    }

    // assign device number
    m_deviceNumber = a_deviceNumber;

    // test libraries
    if (openLibraries() == false)
    {
        return;
    }
    closeLibraries();

    // device is available
    m_deviceAvailable = true;

    // init  virtual gripper user switch
    m_gripperUserSwitchEnabled = false;
}


//==============================================================================
/*!
    Destructor of cLeapDevice.
*/
//==============================================================================
cLeapDevice::~cLeapDevice()
{
    // close device
    if (m_deviceReady)
    {
        close();
    }
}


//==============================================================================
/*!
    This method opens a connection to the device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable)
    {
        return (C_ERROR);
    }

    // if system is already opened then return
    if (m_deviceReady)
    {
        return (C_SUCCESS);
    }

    // open libraries
    if (openLibraries() == false)
    {
        return (C_ERROR);
    }

    // flag the device as ready for use
    m_deviceReady = true;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This methods closes the connection to the device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // update status
    m_deviceReady = false;

    // close libraries
    closeLibraries();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method calibrates the device.

    \param  a_forceCalibration  Enforce calibration.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::calibrate(bool a_forceCalibration)
{
    if (m_deviceReady)
    {
        return (C_SUCCESS);
    }
    else
    {
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method returns the position of the  device. Units are meters [m].

    \param  a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::getPosition(cVector3d& a_position)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    if (updateData())
    {
        cVector3d position[2];
        if (tdLeapGetPosition(position))
        {
            a_position = position[m_deviceNumber];

            // return success
            return (C_SUCCESS);
        }
    }

    return (C_ERROR);
}


//==============================================================================
/*!
    This method returns the orientation frame of the device end-effector.

    \param  a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    if (updateData())
    {
        cMatrix3d rotation[2];
        if (tdLeapGetRotation(rotation))
        {
            a_rotation = rotation[m_deviceNumber];

            // estimate angular velocity
            estimateAngularVelocity(a_rotation);

            // return success
            return (C_SUCCESS);
        }
    }

    return (C_ERROR);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian [rad].

    \param  a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::getGripperAngleRad(double& a_angle)
{
    // default value
    a_angle = 0.0;

    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    if (updateData())
    {
        double angle[2];
        if (tdLeapGetGripperAngleRad(angle))
        {
            a_angle = angle[m_deviceNumber];

            // estimate angular velocity
            estimateGripperVelocity(a_angle);

            // return success
            return (C_SUCCESS);
        }
    }

    return (C_ERROR);
}


//==============================================================================
/*!
    This method returns the status of all user switches
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    if (updateData())
    {
        unsigned int userSwitches[2];
        if (tdLeapGetUserSwitches(userSwitches))
        {
            a_userSwitches = userSwitches[m_deviceNumber];

            // return success
            return (C_SUCCESS);
        }
    }

    return (C_ERROR);
}


//==============================================================================
/*!
    Update data from controllers.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cLeapDevice::updateData()
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // update data
    if (tdLeapUpdate())
    {

        // return success
        return (C_SUCCESS);
    }
    else
    {
        // report error
        return (C_ERROR);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //C_ENABLE_LEAP_DEVICE_SUPPORT
//------------------------------------------------------------------------------
