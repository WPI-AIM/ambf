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
    \version   3.2.0 $Rev: 1922 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CSixenseDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_SIXENSE_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Number of instances for this class of devices currently using the libraries.
unsigned int cSixenseDevice::s_libraryCounter = 0;

// Allocation table for devices of this class.
bool cSixenseDevice::s_allocationTable[C_MAX_DEVICES] = {false, false, false, false,
    false, false, false, false,
    false, false, false, false,
    false, false, false, false}; 

//! Time guard for data acquisition.
cPrecisionClock cSixenseDevice::m_timeguard;

//! Data acquired from the controller.
sixenseAllControllerData cSixenseDevice::m_data;


#if defined(WIN32) | defined(WIN64)
HINSTANCE SixenseDLL = NULL;

extern "C"
{
    int (__cdecl *sixenseInit)                          (void);
    int (__cdecl *sixenseExit)                          (void);
    int (__cdecl *sixenseGetMaxBases)                   (void);
    int (__cdecl *sixenseSetActiveBase)                 (int i);
    int (__cdecl *sixenseIsBaseConnected)               (int i);
    int (__cdecl *sixenseGetMaxControllers)             (void);
    int (__cdecl *sixenseIsControllerEnabled)           (int which);
    int (__cdecl *sixenseGetNumActiveControllers)       (void);
    int (__cdecl *sixenseGetHistorySize)                (void);
    int (__cdecl *sixenseGetData)                       (int which, int index_back, sixenseControllerData *);
    int (__cdecl *sixenseGetAllData)                    (int index_back, sixenseAllControllerData *);
    int (__cdecl *sixenseGetNewestData)                 (int which, sixenseControllerData*);
    int (__cdecl *sixenseGetAllNewestData)              (sixenseAllControllerData *);
    int (__cdecl *sixenseSetHemisphereTrackingMode)     (int which_controller, int state);
    int (__cdecl *sixenseGetHemisphereTrackingMode)     (int which_controller, int *state);
    int (__cdecl *sixenseAutoEnableHemisphereTracking)  (int which_controller);
    int (__cdecl *sixenseSetHighPriorityBindingEnabled) (int on_or_off);
    int (__cdecl *sixenseGetHighPriorityBindingEnabled) (int *on_or_off);
    int (__cdecl *sixenseTriggerVibration)              (int controller_id, int duration_100ms, int pattern_id);
    int (__cdecl *sixenseSetFilterEnabled)              (int on_or_off);
    int (__cdecl *sixenseGetFilterEnabled)              (int *on_or_off);
    int (__cdecl *sixenseSetFilterParams)               (float near_range, float near_val, float far_range, float far_val);
    int (__cdecl *sixenseGetFilterParams)               (float *near_range, float *near_val, float *far_range, float *far_val);
    int (__cdecl *sixenseSetBaseColor)                  (unsigned char red, unsigned char green, unsigned char blue);
    int (__cdecl *sixenseGetBaseColor)                  (unsigned char *red, unsigned char *green, unsigned char *blue);
}

#else

    void * SixenseSO = NULL;

extern "C"
{
    int (*sixenseInit                         ) (void);
    int (*sixenseExit                         ) (void);
    int (*sixenseGetMaxBases                  ) ();
    int (*sixenseSetActiveBase                ) (int i);
    int (*sixenseIsBaseConnected              ) (int i);
    int (*sixenseGetMaxControllers            ) (void);
    int (*sixenseIsControllerEnabled          ) (int which);
    int (*sixenseGetNumActiveControllers      ) ();
    int (*sixenseGetHistorySize               ) ();
    int (*sixenseGetData                      ) (int which, int index_back, sixenseControllerData *);
    int (*sixenseGetAllData                   ) (int index_back, sixenseAllControllerData *);
    int (*sixenseGetNewestData                ) (int which, sixenseControllerData *);
    int (*sixenseGetAllNewestData             ) (sixenseAllControllerData *);
    int (*sixenseSetHemisphereTrackingMode    ) (int which_controller, int state);
    int (*sixenseGetHemisphereTrackingMode    ) (int which_controller, int *state);
    int (*sixenseAutoEnableHemisphereTracking ) (int which_controller);
    int (*sixenseSetHighPriorityBindingEnabled) (int on_or_off);
    int (*sixenseGetHighPriorityBindingEnabled) (int *on_or_off);
    int (*sixenseTriggerVibration             ) (int controller_id, int duration_100ms, int pattern_id);
    int (*sixenseSetFilterEnabled             ) (int on_or_off);
    int (*sixenseGetFilterEnabled             ) (int *on_or_off);
    int (*sixenseSetFilterParams              ) (float near_range, float near_val, float far_range, float far_val);
    int (*sixenseGetFilterParams              ) (float *near_range, float *near_val, float *far_range, float *far_val);
    int (*sixenseSetBaseColor                 ) (unsigned char red, unsigned char green, unsigned char blue);
    int (*sixenseGetBaseColor                 ) (unsigned char *red, unsigned char *green, unsigned char *blue);
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
bool cSixenseDevice::openLibraries() 
{ 
    // increment number of instances using the libraries for this class of devices
    s_libraryCounter++;

    // if libraries are already initialized, then we are done
    if (s_libraryCounter > 1) return (C_SUCCESS);


    ////////////////////////////////////////////////////////////////////////////
    // initialize libraries
    ////////////////////////////////////////////////////////////////////////////

#if defined(WIN32) | defined(WIN64)
#if defined(WIN64)
    // load 64-bit library
    if (SixenseDLL==NULL) SixenseDLL = LoadLibrary("sixense_x64.dll");
#else
    // load 32-bit library
    if (SixenseDLL==NULL) SixenseDLL = LoadLibrary("sixense.dll");
#endif
#endif

#if defined(WIN32) | defined(WIN64)
    // check if DLL loaded correctly
    if (SixenseDLL == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    sixenseInit = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseInit");
    sixenseExit = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseExit");
    sixenseGetMaxBases = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseGetMaxBases");
    sixenseSetActiveBase = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseSetActiveBase");
    sixenseIsBaseConnected = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseIsBaseConnected");
    sixenseGetMaxControllers = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseGetMaxControllers");
    sixenseIsControllerEnabled = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseIsControllerEnabled");
    sixenseGetNumActiveControllers = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseGetNumActiveControllers");
    sixenseGetHistorySize = (int (__cdecl*)(void))GetProcAddress(SixenseDLL, "sixenseGetHistorySize");
    sixenseGetData = (int (__cdecl*)(int, int, sixenseControllerData*))GetProcAddress(SixenseDLL, "sixenseGetData");
    sixenseGetAllData = (int (__cdecl*)(int, sixenseAllControllerData*))GetProcAddress(SixenseDLL, "sixenseGetAllData");
    sixenseGetNewestData = (int (__cdecl*)(int, sixenseControllerData*))GetProcAddress(SixenseDLL, "sixenseGetNewestData");
    sixenseGetAllNewestData = (int (__cdecl*)(sixenseAllControllerData*))GetProcAddress(SixenseDLL, "sixenseGetAllNewestData");
    sixenseSetHemisphereTrackingMode = (int (__cdecl*)(int, int))GetProcAddress(SixenseDLL, "sixenseSetHemisphereTrackingMode");
    sixenseGetHemisphereTrackingMode = (int (__cdecl*)(int, int*))GetProcAddress(SixenseDLL, "sixenseGetHemisphereTrackingMode");
    sixenseAutoEnableHemisphereTracking = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseAutoEnableHemisphereTracking");
    sixenseSetHighPriorityBindingEnabled = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseSetHighPriorityBindingEnabled");
    sixenseGetHighPriorityBindingEnabled = (int (__cdecl*)(int*))GetProcAddress(SixenseDLL, "sixenseGetHighPriorityBindingEnabled");
    sixenseTriggerVibration = (int (__cdecl*)(int, int, int))GetProcAddress(SixenseDLL, "sixenseTriggerVibration");
    sixenseSetFilterEnabled = (int (__cdecl*)(int))GetProcAddress(SixenseDLL, "sixenseSetFilterEnabled");
    sixenseGetFilterEnabled = (int (__cdecl*)(int*))GetProcAddress(SixenseDLL, "sixenseGetFilterEnabled");
    sixenseSetFilterParams = (int (__cdecl*)(float, float, float, float))GetProcAddress(SixenseDLL, "sixenseSetFilterParams");
    sixenseGetFilterParams = (int (__cdecl*)(float*, float*, float*, float*))GetProcAddress(SixenseDLL, "sixenseGetFilterParams");
    sixenseSetBaseColor = (int (__cdecl*)(unsigned char, unsigned char, unsigned char))GetProcAddress(SixenseDLL, "sixenseSetBaseColor");
    sixenseGetBaseColor = (int (__cdecl*)(unsigned char*, unsigned char*, unsigned char*))GetProcAddress(SixenseDLL, "sixenseGetBaseColor");
#endif

#if defined(LINUX)

    // load shared library
    #ifdef __LP64__
    SixenseSO = dlopen ("libsixense_x64.so", RTLD_NOW);
    #else
    SixenseSO = dlopen ("libsixense.so", RTLD_NOW);
    #endif

#endif

#if defined(MACOSX)

    // load shared library
    #ifdef __LP64__
    SixenseSO = dlopen ("libsixense_x64.dylib", RTLD_NOW);
    #else
    SixenseSO = dlopen ("libsixense.dylib", RTLD_NOW);
    #endif


#endif

#if defined(LINUX) | defined(MACOSX)

    // check that it loaded correctly
    if (SixenseSO == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }


    // load different callbacks
    *(void**)(&sixenseInit                         ) = dlsym (SixenseSO, "sixenseInit");
    *(void**)(&sixenseExit                         ) = dlsym (SixenseSO, "sixenseExit");
    *(void**)(&sixenseGetMaxBases                  ) = dlsym (SixenseSO, "sixenseGetMaxBases");
    *(void**)(&sixenseSetActiveBase                ) = dlsym (SixenseSO, "sixenseSetActiveBase");
    *(void**)(&sixenseIsBaseConnected              ) = dlsym (SixenseSO, "sixenseIsBaseConnected");
    *(void**)(&sixenseGetMaxControllers            ) = dlsym (SixenseSO, "sixenseGetMaxControllers");
    *(void**)(&sixenseIsControllerEnabled          ) = dlsym (SixenseSO, "sixenseIsControllerEnabled");
    *(void**)(&sixenseGetNumActiveControllers      ) = dlsym (SixenseSO, "sixenseGetNumActiveControllers");
    *(void**)(&sixenseGetHistorySize               ) = dlsym (SixenseSO, "sixenseGetHistorySize");
    *(void**)(&sixenseGetData                      ) = dlsym (SixenseSO, "sixenseGetData");
    *(void**)(&sixenseGetAllData                   ) = dlsym (SixenseSO, "sixenseGetAllData");
    *(void**)(&sixenseGetNewestData                ) = dlsym (SixenseSO, "sixenseGetNewestData");
    *(void**)(&sixenseGetAllNewestData             ) = dlsym (SixenseSO, "sixenseGetAllNewestData");
    *(void**)(&sixenseSetHemisphereTrackingMode    ) = dlsym (SixenseSO, "sixenseSetHemisphereTrackingMode");
    *(void**)(&sixenseGetHemisphereTrackingMode    ) = dlsym (SixenseSO, "sixenseGetHemisphereTrackingMode");
    *(void**)(&sixenseAutoEnableHemisphereTracking ) = dlsym (SixenseSO, "sixenseAutoEnableHemisphereTracking");
    *(void**)(&sixenseSetHighPriorityBindingEnabled) = dlsym (SixenseSO, "sixenseSetHighPriorityBindingEnabled");
    *(void**)(&sixenseGetHighPriorityBindingEnabled) = dlsym (SixenseSO, "sixenseGetHighPriorityBindingEnabled");
    *(void**)(&sixenseTriggerVibration             ) = dlsym (SixenseSO, "sixenseTriggerVibration");
    *(void**)(&sixenseSetFilterEnabled             ) = dlsym (SixenseSO, "sixenseSetFilterEnabled");
    *(void**)(&sixenseGetFilterEnabled             ) = dlsym (SixenseSO, "sixenseGetFilterEnabled");
    *(void**)(&sixenseSetFilterParams              ) = dlsym (SixenseSO, "sixenseSetFilterParams");
    *(void**)(&sixenseGetFilterParams              ) = dlsym (SixenseSO, "sixenseGetFilterParams");
    *(void**)(&sixenseSetBaseColor                 ) = dlsym (SixenseSO, "sixenseSetBaseColor");
    *(void**)(&sixenseGetBaseColor                 ) = dlsym (SixenseSO, "sixenseGetBaseColor");

#endif

    // initialize libraries
    if (sixenseInit() == SIXENSE_SUCCESS)
    {
        return (C_SUCCESS);
    }
    else
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    } 
}


//==============================================================================
/*!
    This method close the libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::closeLibraries() 
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

    // free libraries
    #if defined(WIN32) | defined(WIN64)
    if ((s_libraryCounter == 0) && (SixenseDLL != NULL))
    {
        sixenseExit();
        FreeLibrary(SixenseDLL);
        SixenseDLL = 0;
    }
    #endif

    #if defined(LINUX) | defined(MACOSX)
    if ((s_libraryCounter == 0) && (SixenseSO != NULL))
    {
        sixenseExit ();
        dlclose (SixenseSO);
        SixenseSO = 0;
    }
    #endif

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the number of haptic devices available for this class 
    of device.

    \return  Number of available haptic devices.
*/
//==============================================================================
unsigned int cSixenseDevice::getNumDevices()
{
    // open libraries
    openLibraries();

    // sanity check
    if (s_libraryCounter < 1) return (C_ERROR);

    unsigned int result = 0;
    bool found = false;
    int counter = 0;
    while ((counter < 20) && (!found))
    {
        int value = sixenseIsBaseConnected(0);
        if (value > 0)
        {
            result = 2;
            found = true;
        }
        counter++;
        cSleepMs(10);
    }

    // close libraries
    closeLibraries();

    // return number of devices
    return (result);
}


//==============================================================================
/*!
    Constructor of cSixenseDevice.
*/
//==============================================================================
cSixenseDevice::cSixenseDevice(unsigned int a_deviceNumber)
{
    // initialize time guard for data acquisition
    m_timeguard.setTimeoutPeriodSeconds(0.001);
    m_timeguard.start(true);

    // initialize specifications
    m_specifications.m_model                         = C_TRACKER_DEVICE_SIXENSE;
    m_specifications.m_manufacturerName              = "Sixense";
    m_specifications.m_modelName                     = "Razor Hydra";
    m_specifications.m_maxLinearForce                = 0.0;   // [N]
    m_specifications.m_maxAngularTorque              = 0.0;   // [N*m]
    m_specifications.m_maxGripperForce               = 0.0;   // [N]
    m_specifications.m_maxLinearStiffness            = 1000.0;   // [N/m]
    m_specifications.m_maxAngularStiffness           = 1.0;   // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 0.0;   // [N*m/Rad]
    m_specifications.m_maxLinearDamping              = 0.0;   // [N/(m/s)]
    m_specifications.m_maxAngularDamping             = 0.0;   // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping      = 0.0;   // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.5;   // [m]
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
    if (a_deviceNumber > (C_MAX_DEVICES - 1))
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
    Destructor of cSixenseDevice.
*/
//==============================================================================
cSixenseDevice::~cSixenseDevice()
{
    // close device
    if (m_deviceReady)
    {
        close();
    }

    // release device
    if (m_deviceAvailable)
    {
        s_allocationTable[m_deviceNumber] = false;
    }
}


//==============================================================================
/*!
    This method opens a connection to the device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::open()
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
bool cSixenseDevice::close()
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
bool cSixenseDevice::calibrate(bool a_forceCalibration)
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
bool cSixenseDevice::getPosition(cVector3d& a_position)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    updateData();

    // return data
    double x = 0.001 * (double)(m_data.controllers[m_deviceNumber].pos[2]) - 0.5;
    double y = 0.001 * (double)(m_data.controllers[m_deviceNumber].pos[0]);
    double z = 0.001 * (double)(m_data.controllers[m_deviceNumber].pos[1]);
    a_position.set(x, y, z);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the orientation frame of the device end-effector.

    \param  a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // acquire data from controller
    updateData();

    cMatrix3d frame;
    frame.identity();

    frame(0,0) = m_data.controllers[m_deviceNumber].rot_mat[2][2];
    frame(1,0) = m_data.controllers[m_deviceNumber].rot_mat[2][0];
    frame(2,0) = m_data.controllers[m_deviceNumber].rot_mat[2][1];

    frame(0,1) = m_data.controllers[m_deviceNumber].rot_mat[0][2];
    frame(1,1) = m_data.controllers[m_deviceNumber].rot_mat[0][0];
    frame(2,1) = m_data.controllers[m_deviceNumber].rot_mat[0][1];

    frame(0,2) = m_data.controllers[m_deviceNumber].rot_mat[1][2];
    frame(1,2) = m_data.controllers[m_deviceNumber].rot_mat[1][0];
    frame(2,2) = m_data.controllers[m_deviceNumber].rot_mat[1][1];

    // return result
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian [rad].

    \param  a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::getGripperAngleRad(double& a_angle)
{
    // default value
    a_angle = 0.0;

    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // read gripper angle
    a_angle = 0.5 * (1.0 - m_data.controllers[m_deviceNumber].trigger);

    // estimate velocity
    estimateGripperVelocity(a_angle);

    // return result
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the status of all user switches
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // read all user switches
    unsigned int mask = 0;

    if (m_data.controllers[m_deviceNumber].buttons & 1) 
    {
        mask = mask | 1;
    }
    if (m_data.controllers[m_deviceNumber].buttons & 32) 
    {
        mask = mask | 2;
    }
    if (m_data.controllers[m_deviceNumber].buttons & 64) 
    {
        mask = mask | 4;
    }
    if (m_data.controllers[m_deviceNumber].buttons & 8) 
    {
        mask = mask | 8;
    }
    if (m_data.controllers[m_deviceNumber].buttons & 16) 
    {
        mask = mask | 16;
    }
    if (m_data.controllers[m_deviceNumber].buttons & 128) 
    {
        mask = mask | 32;
    }

    // return result
    a_userSwitches = mask;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Update data from controllers.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cSixenseDevice::updateData()
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // update data
    if (m_timeguard.timeoutOccurred())
    {
        sixenseGetAllNewestData(&m_data);
        m_timeguard.start(true);
    }

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //C_ENABLE_SIXENSE_DEVICE_SUPPORT
//------------------------------------------------------------------------------
