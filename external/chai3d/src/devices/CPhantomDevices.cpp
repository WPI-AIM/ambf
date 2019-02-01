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
    \version   3.2.0 $Rev: 1938 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CPhantomDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Number of instances for this class of devices currently using the libraries.
unsigned int cPhantomDevice::s_libraryCounter = 0;

// Allocation table for devices of this class.
bool cPhantomDevice::s_allocationTable[C_MAX_DEVICES] = {false, false, false, false,
    false, false, false, false,
    false, false, false, false,
    false, false, false, false}; 

//------------------------------------------------------------------------------
// WIN32
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
HINSTANCE hdPhantomDLL = NULL;
HINSTANCE hdPhantomDriverDLL = NULL;

extern "C"
{

int (__stdcall *hdPhantomGetNumDevices)  ();

int (__stdcall *hdPhantomOpen)           (const int a_deviceID);

int (__stdcall *hdPhantomClose)          (const int a_deviceID);

int (__stdcall *hdPhantomGetPosition)    (const int a_deviceID,
                                          double *a_posX,
                                          double *a_posY,
                                          double *a_posZ);

int (__stdcall *hdPhantomGetLinearVelocity)(const int a_deviceID,
                                            double *a_velX,
                                            double *a_velY,
                                            double *a_velZ);

int (__stdcall *hdPhantomGetRotation)    (const int a_deviceID,
                                          double *a_rot00,
                                          double *a_rot01,
                                          double *a_rot02,
                                          double *a_rot10,
                                          double *a_rot11,
                                          double *a_rot12,
                                          double *a_rot20,
                                          double *a_rot21,
                                          double *a_rot22);

int (__stdcall *hdPhantomGetButtons)     (const int a_deviceID);

int (__stdcall *hdPhantomSetForce)       (const int a_deviceID,
                                          const double *a_forceX,
                                          const double *a_forceY,
                                          const double *a_forceZ);

int (__stdcall *hdPhantomSetTorque)      (int a_deviceID,
                                          const double *a_torqueX,
                                          const double *a_torqueY,
                                          const double *a_torqueZ);

int (__stdcall *hdPhantomSetForceAndTorque) (int a_deviceID,
                                             const double *a_forceX,
                                             const double *a_forceY,
                                             const double *a_forceZ,
                                             const double *a_torqueX,
                                             const double *a_torqueY,
                                             const double *a_torqueZ);

int (__stdcall *hdPhantomGetWorkspaceRadius)(const int a_deviceID,
                                             double *a_workspaceRadius);

int (__stdcall *hdPhantomGetType)(int a_deviceID,
                                  char* a_typeName);

// initialize servo controller
void (__stdcall *hdPhantomStartServo)();

// stop servo controller
void (__stdcall *hdPhantomStopServo)();

}

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// LINUX, MACOSX
//------------------------------------------------------------------------------
#if defined(LINUX) | defined(MACOSX)

void * HDSO = NULL;
void * PhantomSO = NULL;

// functions that communicate data with the Phantom devices
int  (*hdPhantomGetNumDevices      ) (void);
int  (*hdPhantomOpen               ) (const int);
int  (*hdPhantomClose              ) (const int);
int  (*hdPhantomGetPosition        ) (const int, double*, double*, double*);
int  (*hdPhantomGetLinearVelocity  ) (const int, double*, double*, double*);
int  (*hdPhantomGetRotation        ) (const int, double*, double*, double*, double*, double*, double*, double*, double*, double*);
int  (*hdPhantomGetButtons         ) (const int);
int  (*hdPhantomSetForce           ) (const int, const double*, const double*, const double*);
int  (*hdPhantomSetTorque          ) (const int, const double*, const double*, const double*);
int  (*hdPhantomSetForceAndTorque  ) (const int, const double*, const double*, const double*, const double*, const double*, const double*);
int  (*hdPhantomGetWorkspaceRadius ) (const int, double*);
int  (*hdPhantomGetType            ) (const int, char*);
void (*hdPhantomStartServo         ) (void);
void (*hdPhantomStopServo          ) (void);

//------------------------------------------------------------------------------
#endif  // LINUX, MACOSX
//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method opens the libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::openLibraries() 
{ 
    // increment number of instances using the libraries for this class of devices
    s_libraryCounter++;

    // if libraries are already initialized, then we are done
    if (s_libraryCounter > 1) return (C_SUCCESS); 


    ////////////////////////////////////////////////////////////////////////////
    // initialize libraries
    ////////////////////////////////////////////////////////////////////////////

    // check if Phantom drivers installed
#if defined(WIN32) | defined(WIN64)
    if (hdPhantomDriverDLL == NULL)
    {
        hdPhantomDriverDLL = LoadLibrary("HD.dll");
    }

    if (hdPhantomDriverDLL == NULL) 
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }
#endif

    // load phantom interface library
#if defined(WIN32) | defined(WIN64)
    #if defined (WIN64)
        hdPhantomDLL = LoadLibrary("hdPhantom64.dll");
    #else
        hdPhantomDLL = LoadLibrary("hdPhantom32.dll");
    #endif
#endif


#if defined(WIN32) | defined(WIN64)

    // check if DLL loaded correctly
    if (hdPhantomDLL == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    hdPhantomGetNumDevices = (int (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomGetNumDevices");
    hdPhantomOpen = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomOpen");
    hdPhantomClose = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomClose");
    hdPhantomGetPosition = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetPosition");
    hdPhantomGetLinearVelocity = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetLinearVelocity");
    hdPhantomGetRotation = (int (__stdcall*)(const int, double*, double*, double*, double*, double*, double*, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetRotation");
    hdPhantomGetButtons = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomGetButtons");
    hdPhantomSetForce = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForce");
    hdPhantomSetTorque = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetTorque");
    hdPhantomSetForceAndTorque = (int (__stdcall*)(const int, const double*, const double*, const double*, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForceAndTorque");
    hdPhantomGetWorkspaceRadius = (int (__stdcall*)(const int, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetWorkspaceRadius");
    hdPhantomGetType = (int (__stdcall*)(const int, char*))GetProcAddress(hdPhantomDLL, "hdPhantomGetType");
    hdPhantomStartServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStartServo");
    hdPhantomStopServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStopServo");
    
    // check if all functions were loaded
    if ((!hdPhantomGetNumDevices) ||
        (!hdPhantomOpen) ||
        (!hdPhantomClose) ||
        (!hdPhantomGetPosition) ||
        (!hdPhantomGetLinearVelocity) ||
        (!hdPhantomGetRotation) ||
        (!hdPhantomGetButtons) ||
        (!hdPhantomSetForce) ||
        (!hdPhantomSetTorque) ||
        (!hdPhantomSetForceAndTorque) ||
        (!hdPhantomGetWorkspaceRadius) ||
        (!hdPhantomGetType) ||
        (!hdPhantomStartServo) ||
        (!hdPhantomStopServo))
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }
#endif


#if defined(LINUX) | defined(MACOSX)

    // load shared library
#ifdef LINUX
    HDSO = dlopen ("libHD.so", RTLD_LAZY|RTLD_GLOBAL);
#endif
#ifdef MACOSX
    HDSO = dlopen ("libHD.dylib", RTLD_LAZY|RTLD_GLOBAL);
#endif

    // check that it loaded correctly
    if (HDSO == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load shared library
#ifdef LINUX
    PhantomSO = dlopen ("libhdPhantom.so", RTLD_LAZY|RTLD_GLOBAL);
#endif
#ifdef MACOSX
    PhantomSO = dlopen ("libhdPhantom.dylib", RTLD_LAZY|RTLD_GLOBAL);
#endif

    // check that it loaded correctly
    if (PhantomSO == NULL)
    {
        printf ("*** CPhantomDevices: %s\n", dlerror());
        dlclose (HDSO);
        HDSO = NULL;
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    *(void**)(&hdPhantomGetNumDevices     ) = dlsym (PhantomSO, "hdPhantomGetNumDevices");
    *(void**)(&hdPhantomOpen              ) = dlsym (PhantomSO, "hdPhantomOpen");
    *(void**)(&hdPhantomClose             ) = dlsym (PhantomSO, "hdPhantomClose");
    *(void**)(&hdPhantomGetPosition       ) = dlsym (PhantomSO, "hdPhantomGetPosition");
    *(void**)(&hdPhantomGetLinearVelocity ) = dlsym (PhantomSO, "hdPhantomGetLinearVelocity");
    *(void**)(&hdPhantomGetRotation       ) = dlsym (PhantomSO, "hdPhantomGetRotation");
    *(void**)(&hdPhantomGetButtons        ) = dlsym (PhantomSO, "hdPhantomGetButtons");
    *(void**)(&hdPhantomSetForce          ) = dlsym (PhantomSO, "hdPhantomSetForce");
    *(void**)(&hdPhantomSetTorque         ) = dlsym (PhantomSO, "hdPhantomSetTorque");
    *(void**)(&hdPhantomSetForceAndTorque ) = dlsym (PhantomSO, "hdPhantomSetForceAndTorque");
    *(void**)(&hdPhantomGetWorkspaceRadius) = dlsym (PhantomSO, "hdPhantomGetWorkspaceRadius");
    *(void**)(&hdPhantomGetType           ) = dlsym (PhantomSO, "hdPhantomGetType");
    *(void**)(&hdPhantomStartServo        ) = dlsym (PhantomSO, "hdPhantomStartServo");
    *(void**)(&hdPhantomStopServo         ) = dlsym (PhantomSO, "hdPhantomStopServo");

    // check if all functions were loaded
    if ((!hdPhantomStopServo) ||
            (!hdPhantomOpen) ||
            (!hdPhantomClose) ||
            (!hdPhantomGetPosition) ||
            (!hdPhantomGetLinearVelocity) ||
            (!hdPhantomGetRotation) ||
            (!hdPhantomGetButtons) ||
            (!hdPhantomSetForce) ||
            (!hdPhantomSetTorque) ||
            (!hdPhantomSetForceAndTorque) ||
            (!hdPhantomGetWorkspaceRadius) ||
            (!hdPhantomGetType) ||
            (!hdPhantomStartServo) ||
            (!hdPhantomStopServo))
    {
        printf ("*** CPhantomDevices: %s\n", dlerror());
        s_libraryCounter = 0;
        return (C_ERROR);
    }

#endif

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method closes the libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::closeLibraries() 
{ 
    // sanity check
    if (s_libraryCounter < 1) return (C_ERROR);


    ///////////////////////////////////////////////////////////////////////////////
    // The following section has been commited due to a bug in the Phantom driver
    // where the driver fails it is loaded and closed a second time by the 
    // application.
    ///////////////////////////////////////////////////////////////////////////////

    /*
    // decrement library counter; exit if other objects are still using libraries
    s_libraryCounter--;
    if (s_libraryCounter > 0) return (C_SUCCESS);

    // free libraries
    #if defined(WIN32) | defined(WIN64)
    if ((s_libraryCounter == 0) && (hdPhantomDLL != NULL))
    {
        FreeLibrary(hdPhantomDLL);
        //FreeLibrary(hdPhantomDriverDLL);
        hdPhantomDLL = NULL;
        //hdPhantomDriverDLL = NULL;
    }
    #endif

    // free libraries
    #if defined(LINUX) | defined(MACOSX)
    if ((s_libraryCounter == 0) && (PhantomSO != NULL))
    {
        dlclose (PhantomSO);
        dlclose (HDSO);
        PhantomSO = NULL;
        HDSO = NULL;
    }
    #endif
    */

    // exit
    return (C_SUCCESS); 
}


//==============================================================================
/*!
    This method returns the number of haptic devices available for this 
    class of device.

    \return Number of available haptic devices.
*/
//==============================================================================
unsigned int cPhantomDevice::getNumDevices()
{
    // open libraries
    if (openLibraries() == false)
    {
        //printf ("*** hdPhantom: open libraries failed\n");
        return (0);
    }

    // get device number
    int result = hdPhantomGetNumDevices();

    // close libraries
    closeLibraries();

    // return result
    return (result);
}


//==============================================================================
/*!
    Constructor of cPhantomDevice.

    \param  a_deviceNumber  Index number to the i'th Phantom device
*/
//==============================================================================
cPhantomDevice::cPhantomDevice(unsigned int a_deviceNumber)
{
    // open libraries
    if (openLibraries() == false)
    {
        return;
    }

    // default specification setup
    m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_OTHER;
    m_specifications.m_manufacturerName              = "Sensable Technologies";
    m_specifications.m_modelName                     = "PHANTOM";
    m_specifications.m_maxLinearForce                = 6.0;     // [N]
    m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
    m_specifications.m_maxGripperForce               = 0.0;     // [N]
    m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
    m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 1000.0;  // [N/m]
    m_specifications.m_maxLinearDamping              = 8.0;     // [N/(m/s)]
    m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.10;    // [m];
    m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    // device is not yet available or ready
    m_deviceAvailable = false;
    m_deviceReady = false;

    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;
    m_deviceNumber = a_deviceNumber;

    // get the number of Phantom devices connected to this computer
    int numDevices = hdPhantomGetNumDevices();

    // check if such device is available
    if ((a_deviceNumber + 1) > (unsigned int)numDevices)
    {
        // no, such ID does not lead to an existing device
        m_deviceAvailable = false;
    }
    else
    {
        // yes, this ID leads to an existing device
        m_deviceAvailable = true;
    }

    // read information related to the device
    hdPhantomGetWorkspaceRadius(m_deviceID, &m_specifications.m_workspaceRadius);

    // read the device model
    char name[255];
    hdPhantomGetType(m_deviceID, &name[0]);
    m_specifications.m_modelName = name;


    /////////////////////////////////////////////////////////////////////
    // Define specifications given the device model
    /////////////////////////////////////////////////////////////////////

    if (m_specifications.m_modelName == "Touch")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_TOUCH;
        m_specifications.m_maxLinearForce                = 3.3;     // [N]
        m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
        m_specifications.m_maxGripperForce               = 0.0;     // [N]
        m_specifications.m_maxLinearStiffness            = 400.0;   // [N/m]
        m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
        m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
        m_specifications.m_maxLinearDamping              = 4.0;     // [N/(m/s)]
        m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_maxGripperAngularDamping      = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_workspaceRadius               = 0.075;    // [m];
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = false;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }

    else if (m_specifications.m_modelName == "PHANTOM Omni")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_OMNI;
        m_specifications.m_maxLinearForce                = 3.3;     // [N]
        m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
        m_specifications.m_maxGripperForce               = 0.0;     // [N]
        m_specifications.m_maxLinearStiffness            = 700.0;   // [N/m]
        m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
        m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
        m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
        m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_maxGripperAngularDamping      = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_workspaceRadius               = 0.10;    // [m];
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = false;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }

    else if (m_specifications.m_modelName == "PHANTOM Desktop")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_DESKTOP;
        m_specifications.m_maxLinearForce                = 7.9;     // [N]
        m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
        m_specifications.m_maxGripperForce               = 0.0;     // [N]
        m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
        m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
        m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
        m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
        m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_maxGripperAngularDamping      = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_workspaceRadius               = 0.10;    // [m];
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = false;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }

    else if (m_specifications.m_modelName == "PHANTOM Premium 1.5 6DOF")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_15_6DOF;
        m_specifications.m_maxLinearForce                = 8.5;     // [N]
        m_specifications.m_maxAngularTorque              = 0.188;   // [N*m]
        m_specifications.m_maxGripperForce               = 0.0;     // [N]
        m_specifications.m_maxLinearStiffness            = 800.0;   // [N/m]
        m_specifications.m_maxAngularStiffness           = 0.3;     // [N*m/Rad]
        m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
        m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
        m_specifications.m_maxAngularDamping             = 0.004;   // [N*m/(Rad/s)]
        m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_workspaceRadius               = 0.12;    // [m];
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = true;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }

    else if (m_specifications.m_modelName == "PHANTOM Premium 3.0 6DOF")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_30_6DOF;
        m_specifications.m_maxLinearForce                = 22.0;     // [N]
        m_specifications.m_maxAngularTorque              = 0.188;   // [N*m]
        m_specifications.m_maxGripperForce               = 0.0;     // [N]
        m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
        m_specifications.m_maxAngularStiffness           = 0.3;     // [N*m/Rad]
        m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
        m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
        m_specifications.m_maxAngularDamping             = 0.004;   // [N*m/(Rad/s)]
        m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
        m_specifications.m_workspaceRadius               = 0.25;    // [m];
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = true;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }
}


//==============================================================================
/*!
    Destructor of cPhantomDevice.
*/
//==============================================================================
cPhantomDevice::~cPhantomDevice()
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

    // close libraries
    closeLibraries();
}


//==============================================================================
/*!
    This method opens a connection to the Phantom device.

    \return __true__if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_SUCCESS);

    // try to open the device
    hdPhantomOpen(m_deviceID);

    // update device status
    m_deviceReady = true;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Close connection to phantom device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    // yes, the device is open so let's close it
    int result = hdPhantomClose(m_deviceID);

     // update status
    m_deviceReady = false;

    // turn off servo loop
    hdPhantomStopServo();

    // exit
    return (result != 0);
}


//==============================================================================
/*!
    Calibrate the phantom device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::calibrate(bool a_forceCalibration)
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
    This method returns the position of the device. Units are meters [m].

    \param  a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // get position
    double x,y,z;
    int error = hdPhantomGetPosition(m_deviceID, &x, &y, &z);
    a_position.set(x, y, z);

    // offset adjustment depending of device type
    if (m_specifications.m_model == C_HAPTIC_DEVICE_PHANTOM_15_6DOF)
    {
        a_position.add(0.0, 0.0, -0.10);
    }

    // estimate velocity
    estimateLinearVelocity(a_position);

    // return result
    return (error != 0);
}


//==============================================================================
/*!
    This method returns the orientation frame of the device end-effector.

    \param  a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);

    // get rotation
    double rot[3][3];
    hdPhantomGetRotation(m_deviceID,
                         &rot[0][0],
                         &rot[0][1],
                         &rot[0][2],
                         &rot[1][0],
                         &rot[1][1],
                         &rot[1][2],
                         &rot[2][0],
                         &rot[2][1],
                         &rot[2][2]);

    a_rotation.set(rot[0][0], rot[0][1], rot[0][2],
                   rot[1][0], rot[1][1], rot[1][2],
                   rot[2][0], rot[2][1], rot[2][2]);

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
	This method sends a force [N] and a torque [N*m] and gripper torque [N*m]
	to the haptic device.

	\param  a_force         Force command.
	\param  a_torque        Torque command.
	\param  a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce)
{
    // check if drivers are installed
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // send force and torque command
    int error = hdPhantomSetForceAndTorque(m_deviceID, &a_force(0), &a_force(1), &a_force(2), &a_torque(0), &a_torque(1), &a_torque(2));
    if (error == -1)
    { 
        return (C_ERROR);
    }

    // store new commanded values
    m_prevForce  = a_force;
    m_prevTorque = a_torque;

    // return success
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
bool cPhantomDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if drivers are installed
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // retrieve user switches
    a_userSwitches = (unsigned int)hdPhantomGetButtons(m_deviceID);

    // return success
    return (C_SUCCESS);
}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_PHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------

