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
    \version   3.2.0 $Rev: 2144 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CDeltaDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Internal device definitions.
#define DHD_DEVICE_DLR331       102
#define DHD_DEVICE_DLR331_LEFT  103
#define DHD_CASE_101            101
#define DHD_CASE_102            102
#define DHD_CASE_103            103
#define DHD_CASE_110            110
#define DHD_CASE_111            111
#define DHD_CASE_112            112
#define DHD_CASE_113            113
#define DHD_CASE_114            114
#define DHD_CASE_200            200
#define DHD_CASE_201            201
#define DHD_CASE_204            204
#define DHD_CASE_205            205

// Number of instances for this class of devices currently using the libraries.
unsigned int cDeltaDevice::s_libraryCounter = 0;

// Allocation table for devices of this class.
bool cDeltaDevice::s_allocationTable[C_MAX_DEVICES] = {false, false, false, false,
    false, false, false, false,
    false, false, false, false,
    false, false, false, false};

// DHD function availability
bool cDeltaDevice::s_dhdOpenID                               = true;
bool cDeltaDevice::s_dhdClose                                = true;
bool cDeltaDevice::s_dhdGetDeviceCount                       = true;
bool cDeltaDevice::s_dhdGetDeviceID                          = true;
bool cDeltaDevice::s_dhdGetSystemType                        = true;
bool cDeltaDevice::s_dhdReset                                = true;
bool cDeltaDevice::s_dhdGetButton                            = true;
bool cDeltaDevice::s_dhdGetButtonMask                        = true;
bool cDeltaDevice::s_dhdGetPosition                          = true;
bool cDeltaDevice::s_dhdGetLinearVelocity                    = true;
bool cDeltaDevice::s_dhdGetOrientationRad                    = true;
bool cDeltaDevice::s_dhdGetOrientationFrame                  = true;
bool cDeltaDevice::s_dhdSetForce                             = true;
bool cDeltaDevice::s_dhdSetTorque                            = true;
bool cDeltaDevice::s_dhdSetForceAndTorque                    = true;
bool cDeltaDevice::s_dhdSetForceAndGripperForce              = true;
bool cDeltaDevice::s_dhdSetForceAndTorqueAndGripperForce     = true;
bool cDeltaDevice::s_dhdGetGripperThumbPos                   = true;
bool cDeltaDevice::s_dhdGetGripperFingerPos                  = true;
bool cDeltaDevice::s_dhdGetGripperAngleRad                   = true;
bool cDeltaDevice::s_dhdEnableExpertMode                     = true;
bool cDeltaDevice::s_dhdDisableExpertMode                    = true;
bool cDeltaDevice::s_dhdEnableForce                          = true;
bool cDeltaDevice::s_dhdIsLeftHanded                         = true;
bool cDeltaDevice::s_dhdSetBaseAngleZDeg                     = true;
bool cDeltaDevice::s_dhdSetVelocityThreshold                 = true;
bool cDeltaDevice::s_dhdSetWatchdog                          = true;
bool cDeltaDevice::s_dhdSetDeviceAngleDeg                    = true;
bool cDeltaDevice::s_dhdGetJointAngles                       = true;
bool cDeltaDevice::s_dhdSetVibration                         = true;

// DRD function availability
bool cDeltaDevice::s_drdOpenID                               = true;
bool cDeltaDevice::s_drdClose                                = true;
bool cDeltaDevice::s_drdIsInitialized                        = true;
bool cDeltaDevice::s_drdAutoInit                             = true;
bool cDeltaDevice::s_drdStop                                 = true;

// Extras
bool cDeltaDevice::s_flagUsingSDK32                          = false;

#if defined(WIN32) | defined(WIN64)
HINSTANCE fdDLL = NULL;

int  (__stdcall *dhdGetDeviceCount)                   (void);
int  (__stdcall *dhdGetDeviceID)                      (void);
int  (__stdcall *dhdOpenID)                           (char ID);
int  (__stdcall *dhdGetSystemType)                    (char ID);
int  (__stdcall *dhdClose)                            (char ID);
int  (__stdcall *dhdReset)                            (char ID);
int  (__stdcall *dhdGetButton)                        (int index, char ID);
unsigned int (__stdcall *dhdGetButtonMask)            (char ID);
int  (__stdcall *dhdGetPosition)                      (double *px, double *py, double *pz, char ID);
int  (__stdcall *dhdGetLinearVelocity)                (double *vx, double *vy, double *vz, char ID);
int  (__stdcall *dhdSetForce)                         (double  fx, double  fy, double  fz, char ID);
int  (__stdcall *dhdGetOrientationRad)                (double *oa, double *ob, double *og, char ID);
int  (__stdcall *dhdSetTorque)                        (double  ta, double  tb, double  tg, char ID);
int  (__stdcall *dhdGetOrientationFrame)              (double matrix[3][3], char ID);
int  (__stdcall *dhdSetForceAndGripperForce)          (double fx, double fy, double fz, double f, char ID);
int  (__stdcall *dhdSetForceAndTorque)                (double fx, double fy, double fz, double  ta, double  tb, double  tg, char ID);
int  (__stdcall *dhdSetForceAndTorqueAndGripperForce) (double fx, double fy, double fz, double  ta, double  tb, double  tg, double f, char ID);
int  (__stdcall *dhdGetGripperThumbPos)               (double *tx, double *ty, double *tz,  char ID);
int  (__stdcall *dhdGetGripperFingerPos)              (double *fx, double *fy, double *fz,  char ID);
int  (__stdcall *dhdGetGripperAngleRad)               (double *a, char ID);
int  (__stdcall *dhdEnableExpertMode)                 (void);
int  (__stdcall *dhdDisableExpertMode)                (void);
int  (__stdcall *dhdEnableForce)                      (unsigned char val, char ID);
bool (__stdcall *dhdIsLeftHanded)                     (char ID);
int  (__stdcall *dhdSetBaseAngleZDeg)                 (double angle, char ID);
int  (__stdcall *dhdSetVelocityThreshold)             (unsigned char val, char ID);
int  (__stdcall *dhdSetWatchdog)                      (unsigned char val, char ID);
int  (__stdcall *drdOpenID)                           (char ID);
int  (__stdcall *drdClose)                            (char ID);
bool (__stdcall *drdIsInitialized)                    (char ID);
int  (__stdcall *drdAutoInit)                         (char ID);
int  (__stdcall *drdStop)                             (bool frc, char ID);
int  (__stdcall *dhdSetDeviceAngleDeg)                (double angle, char ID);
int  (__stdcall *dhdGetJointAngles)                   (double j[DHD_MAX_DOF], char ID);
int  (__stdcall *dhdSetVibration)                     (double freq, double amplitude, int type, char ID);

#else
#include "drdc.h"
#endif

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method opens libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::openLibraries()
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
    // load 64-bit fd library (favoring DRD over DHD)
    if (fdDLL==NULL) fdDLL = LoadLibrary("drd64.dll");
    if (fdDLL==NULL) fdDLL = LoadLibrary("dhd64.dll");
    #else
    // load 32-bit fd library (favoring DRD over DHD)
    if (fdDLL==NULL) fdDLL = LoadLibrary("drd.dll");
    if (fdDLL==NULL) fdDLL = LoadLibrary("dhd.dll");
    #endif
#endif

#if defined(WIN32) | defined(WIN64)

    // check if DLL loaded correctly
    if (fdDLL == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    dhdGetDeviceCount = (int (__stdcall*)(void))GetProcAddress(fdDLL, "dhdGetDeviceCount");
    if (dhdGetDeviceCount == NULL) { s_dhdGetDeviceCount = false; }

    dhdGetDeviceID = (int (__stdcall*)(void))GetProcAddress(fdDLL, "dhdGetDeviceID");
    if (dhdGetDeviceID == NULL) { s_dhdGetDeviceID = false; }

    dhdGetSystemType = (int (__stdcall*)(char))GetProcAddress(fdDLL, "dhdGetSystemType");
    if (dhdGetSystemType == NULL) { s_dhdGetSystemType = false; }

    dhdOpenID = (int (__stdcall*)(char))GetProcAddress(fdDLL, "dhdOpenID");
    if (dhdOpenID == NULL) { s_dhdOpenID = false; }

    dhdClose = (int (__stdcall*)(char))GetProcAddress(fdDLL, "dhdClose");
    if (dhdClose == NULL) { s_dhdClose = false; }

    dhdReset = (int (__stdcall*)(char))GetProcAddress(fdDLL, "dhdReset");
    if (dhdReset == NULL) { s_dhdReset = false; }

    dhdGetButton = (int (__stdcall*)(int, char))GetProcAddress(fdDLL, "dhdGetButton");
    if (dhdGetButton == NULL) { s_dhdGetButton = false; }

    dhdGetButtonMask = (unsigned int (__stdcall*)(char))GetProcAddress(fdDLL, "dhdGetButtonMask");
    if (dhdGetButtonMask == NULL) { s_dhdGetButtonMask = false; }

    dhdGetPosition = (int (__stdcall*)(double*, double*, double*, char))GetProcAddress(fdDLL, "dhdGetPosition");
    if (dhdGetPosition == NULL) { s_dhdGetPosition = false; }

    dhdSetForce = (int (__stdcall*)(double, double, double, char))GetProcAddress(fdDLL, "dhdSetForce");
    if (dhdSetForce == NULL) { s_dhdSetForce = false; }

    dhdGetOrientationRad = (int( __stdcall*)(double*, double*, double*, char))GetProcAddress(fdDLL, "dhdGetOrientationRad");
    if (dhdGetOrientationRad == NULL) { s_dhdGetOrientationRad = false; }

    dhdSetTorque = (int (__stdcall*)(double, double, double, char))GetProcAddress(fdDLL, "dhdSetTorque");
    if (dhdSetTorque == NULL) { s_dhdSetTorque = false; }

    dhdGetOrientationFrame = (int (__stdcall*)(double[3][3], char ID))GetProcAddress(fdDLL, "dhdGetOrientationFrame"); 
    if (dhdGetOrientationFrame == NULL)
    {
        dhdGetOrientationFrame = (int (__stdcall*)(double[3][3], char ID))GetProcAddress(fdDLL, "dhdGetRotatorMatrix");
        s_flagUsingSDK32 = true;
        if (dhdGetOrientationFrame == NULL)
        {
            s_dhdGetOrientationFrame = false;
        }
    }
    if (dhdGetOrientationFrame == NULL) { s_dhdGetOrientationFrame = false; }

    dhdGetLinearVelocity = (int (__stdcall*)(double *vx, double *vy, double *vz, char ID))GetProcAddress(fdDLL, "dhdGetLinearVelocity");
    if (dhdGetLinearVelocity == NULL) { s_dhdGetLinearVelocity = false; }

    dhdSetForceAndTorque = (int (__stdcall*)(double fx, double fy, double fz, double tx, double ty, double tz, char ID))GetProcAddress(fdDLL, "dhdSetForceAndTorque");
    if (dhdSetForceAndTorque == NULL) { s_dhdSetForceAndTorque = false; }

    dhdSetForceAndGripperForce = (int (__stdcall*)(double fx, double fy, double fz, double f, char ID))GetProcAddress(fdDLL, "dhdSetForceAndGripperForce");
    if (dhdSetForceAndGripperForce == NULL) { s_dhdSetForceAndGripperForce = false; }

    dhdSetForceAndTorqueAndGripperForce = (int (__stdcall*)(double fx, double fy, double fz, double tx, double ty, double tz, double f, char ID))GetProcAddress(fdDLL, "dhdSetForceAndTorqueAndGripperForce");
    if (dhdSetForceAndGripperForce == NULL) { s_dhdSetForceAndTorqueAndGripperForce = false; }

    dhdGetGripperThumbPos = (int (__stdcall*)(double *tx, double *ty, double *tz,  char ID))GetProcAddress(fdDLL, "dhdGetGripperThumbPos");
    if (dhdGetGripperThumbPos == NULL) { s_dhdGetGripperThumbPos = false; }

    dhdGetGripperFingerPos = (int (__stdcall*)(double *fx, double *fy, double *fz,  char ID))GetProcAddress(fdDLL, "dhdGetGripperFingerPos");
    if (dhdGetGripperFingerPos == NULL) { s_dhdGetGripperFingerPos = false; }

    dhdGetGripperAngleRad = (int (__stdcall*)(double *a, char ID))GetProcAddress(fdDLL, "dhdGetGripperAngleRad");
    if (dhdGetGripperAngleRad == NULL) { s_dhdGetGripperAngleRad = false; }

    dhdEnableExpertMode = (int (__stdcall*)(void))GetProcAddress(fdDLL, "dhdEnableExpertMode");
    if (dhdEnableExpertMode == NULL) { s_dhdEnableExpertMode = false; }

    dhdDisableExpertMode = (int (__stdcall*)(void))GetProcAddress(fdDLL, "dhdDisableExpertMode");
    if (dhdDisableExpertMode == NULL) { s_dhdDisableExpertMode = false; }

    dhdEnableForce = (int (__stdcall*)(unsigned char val, char ID))GetProcAddress(fdDLL, "dhdEnableForce");
    if (dhdEnableForce == NULL) { s_dhdEnableForce = false; }

    dhdIsLeftHanded = (bool(__stdcall*)(char ID))GetProcAddress(fdDLL, "dhdIsLeftHanded");
    if (dhdIsLeftHanded == NULL) { s_dhdIsLeftHanded = false; }

    dhdSetBaseAngleZDeg = (int (__stdcall*)(double angle, char ID))GetProcAddress(fdDLL, "dhdSetBaseAngleZDeg");
    if (dhdSetBaseAngleZDeg == NULL) { s_dhdSetBaseAngleZDeg = false; }

    dhdSetVelocityThreshold = (int (__stdcall*)(unsigned char val, char ID))GetProcAddress(fdDLL, "dhdSetVelocityThreshold");
    if (dhdSetVelocityThreshold == NULL) { s_dhdSetVelocityThreshold = false; }

    dhdSetWatchdog = (int (__stdcall*)(unsigned char val, char ID))GetProcAddress(fdDLL, "dhdSetWatchdog");
    if (dhdSetWatchdog == NULL) { s_dhdSetWatchdog = false; }

    drdOpenID = (int (__stdcall*)(char ID))GetProcAddress(fdDLL, "drdOpenID");
    if (drdOpenID == NULL) { s_drdOpenID = false; }

    drdClose = (int (__stdcall*)(char ID))GetProcAddress(fdDLL, "drdClose");
    if (drdClose == NULL) { s_drdClose = false; }

    drdIsInitialized = (bool (__stdcall*)(char ID))GetProcAddress(fdDLL, "drdIsInitialized");
    if (drdIsInitialized == NULL) { s_drdIsInitialized = false; }

    drdAutoInit = (int (__stdcall*)(char ID))GetProcAddress(fdDLL, "drdAutoInit");
    if (drdAutoInit == NULL) { s_drdAutoInit = false; }

    drdStop = (int (__stdcall*)(bool frc, char ID))GetProcAddress(fdDLL, "drdStop");
    if (drdStop == NULL) { s_drdStop = false; }

    dhdSetDeviceAngleDeg = (int(__stdcall*)(double angle, char ID))GetProcAddress(fdDLL, "dhdSetDeviceAngleDeg");
    if (dhdSetDeviceAngleDeg == NULL) { s_dhdSetDeviceAngleDeg = false; }

    dhdGetJointAngles = (int(__stdcall*)(double j[DHD_MAX_DOF], char ID))GetProcAddress(fdDLL, "dhdGetJointAngles");
    if (dhdGetJointAngles == NULL) { s_dhdGetJointAngles = false; }

    dhdSetVibration = (int(__stdcall*)(double freq, double amplitude, int type, char ID))GetProcAddress(fdDLL, "dhdSetVibration");
    if (dhdSetVibration == NULL) { s_dhdSetVibration = false; }

#endif

    // display a message if some function calls are missing
    if (   (!s_dhdGetDeviceCount)
        || (!s_dhdGetDeviceID)
        || (!s_dhdGetSystemType)
        || (!s_dhdOpenID)
        || (!s_dhdClose)
        || (!s_dhdGetButton)
        || (!s_dhdGetButtonMask)
        || (!s_dhdReset)
        || (!s_dhdGetPosition)
        || (!s_dhdGetLinearVelocity)
        || (!s_dhdSetForce)
        || (!s_dhdSetForceAndTorque)
        || (!s_dhdGetOrientationRad)
        || (!s_dhdGetOrientationFrame)
        || (!s_dhdSetForceAndGripperForce)
        || (!s_dhdSetForceAndTorqueAndGripperForce)
        || (!s_dhdGetGripperThumbPos)
        || (!s_dhdGetGripperFingerPos)
        || (!s_dhdGetGripperAngleRad)
        || (!s_dhdEnableExpertMode)
        || (!s_dhdDisableExpertMode)
        || (!s_dhdEnableForce)
        || (!s_dhdIsLeftHanded)
        || (!s_dhdSetBaseAngleZDeg)
        || (!s_dhdSetVelocityThreshold)
        || (!s_dhdSetWatchdog)
        || (!s_drdOpenID)
        || (!s_drdClose)
        || (!s_drdIsInitialized)
        || (!s_drdAutoInit)
        || (!s_drdStop))
    {
        printf("NOTICE: For optimal performances, we suggest that you update the drivers \n        of your Force Dimension haptic device (www.forcedimension.com).\n");
    }

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method closes libraries for this class of devices.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::closeLibraries()
{
    // check if older version of SDK being used (3.2)
    if (s_flagUsingSDK32) return (C_SUCCESS);

    // sanity check
    if (s_libraryCounter < 1) return (C_ERROR);

    // decrement library counter; exit if other objects are still using libraries
    s_libraryCounter--;
    if (s_libraryCounter > 0) return (C_SUCCESS);

    // free libraries
    #if defined(WIN32) | defined(WIN64)
    if ((s_libraryCounter == 0) && (fdDLL != NULL))
    {
        FreeLibrary(fdDLL);
        fdDLL = NULL;
    }
    #endif

    // exit
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the number of haptic devices available for this class
    of device.

    \return  Number of available haptic devices.
*/
//==============================================================================
unsigned int cDeltaDevice::getNumDevices()
{
    // open libraries
    if (openLibraries() == false)
    {
        return (0);
    }

    // check if DHD-API call is available
    if (!s_dhdGetDeviceCount)
    {
        closeLibraries();
        return (0);
    }

    // get device number
    int result = dhdGetDeviceCount();

    // close libraries
    closeLibraries();

    // return result
    return (result);
}


//==============================================================================
/*!
    Constructor of cDeltaDevice.

    \param  a_deviceNumber  Device index number. [0, C_DEVICE_MAX]
    \param  a_external      __true__ if dhdOpen() called outside CHAI3D,
                            __false__ otherwise.
*/
//==============================================================================
cDeltaDevice::cDeltaDevice(unsigned int a_deviceNumber, bool a_external): cGenericHapticDevice(a_deviceNumber)
{
    // name of the device manufacturer
    m_specifications.m_manufacturerName = "Force Dimension";

    // device is not yet available or ready
    m_deviceAvailable   = false;
    m_deviceReady       = false;
    m_deviceType        = -1;

    // open libraries
    if (openLibraries() == false)
    {
        return;
    }

    // sanity check
    if (a_deviceNumber > (C_MAX_DEVICES - 1))
    {
        return;
    }

    // set device number
    m_deviceNumber = a_deviceNumber;

    // get actual number of Force Dimension devices connected to this computer
    int numDevices = 0;
    if (s_dhdGetDeviceCount)
    {
        numDevices = dhdGetDeviceCount();
    }

    // check if such device is available or if it has already been allocated
    if  ( ((a_deviceNumber + 1) > (unsigned int)numDevices) ||
          (s_allocationTable[a_deviceNumber] == true))
    {
        // no, such ID does not lead to an existing device
        m_deviceAvailable = false;
    }
    else
    {
        // remember if the device was opened outside CHAI3D
        // (this means we should not close/open it again)
        m_external = a_external;

        // tentatively enable device
        m_deviceAvailable = true;

        // open the device to read all the technical specifications about it
        if (open())
        {
            // update allocation table by marking device as allocated
            s_allocationTable[m_deviceNumber] = true;

            // every thing is ok, we can now close the device
            close();
        }
        else
        {
            // no, this device number does not lead to a valid or available device
            m_deviceAvailable = false;
        }
    }

    // init code to handle user switches
    for (int i=0; i<8; i++)
    {
        m_userSwitchCount[i] = 0;
        m_userSwitchStatus[i] = 0;
        m_userSwitchClock[i].reset();
        m_userSwitchClock[i].start();
    }

    //init virtual gripper user switch
    m_gripperUserSwitchEnabled          = false;
    m_gripperUserSwitchAngleStart       = 10;
    m_gripperUserSwitchAngleClick       = 7;
    m_gripperUserSwitchForceClick       = 3;
    m_gripperUserSwitchForceEngaged     = 2;
}


//==============================================================================
/*!
    Destructor of cDeltaDevice.
*/
//==============================================================================
cDeltaDevice::~cDeltaDevice()
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
    This method opens a connection to the haptic device.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_SUCCESS);

    // check if DHD-API call is available
    if (!s_dhdOpenID) return (C_ERROR);

    // try to open the device
    int result;
    if (s_drdOpenID) result = drdOpenID(m_deviceNumber);
    else             result = dhdOpenID(m_deviceNumber);

    // update device status
    if (result < 0)
    {
        m_deviceReady = false;
        m_deviceID    = 0;
        return (C_ERROR);
    }
    else
    {
        m_deviceReady = true;
        m_deviceID    = result;
    }

    // init force status
    m_statusEnableForcesFirstTime = true;

    // read the device type
    m_deviceType = DHD_DEVICE_OMEGA;
    if (s_dhdGetSystemType)
    {
        m_deviceType = dhdGetSystemType(m_deviceID);
    }

    // left/right hand
    bool leftHandOnly = false;
    if (s_dhdIsLeftHanded)
    {
        leftHandOnly = dhdIsLeftHanded(m_deviceID);
    }

    // default information
    m_specifications.m_model                         = C_HAPTIC_DEVICE_OMEGA_3;
    m_specifications.m_manufacturerName              = "Force Dimension";
    m_specifications.m_modelName                     = "default";
    m_specifications.m_maxLinearForce                =   12.0;   // [N]
    m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
    m_specifications.m_maxGripperForce               =    0.0;   // [N]
    m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
    m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
    m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
    m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               =    0.075; // [m]
    m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;
    m_posWorkspaceOffset.set(0.0, 0.0, 0.0);


    // setup information regarding the device
    switch (m_deviceType)
    {
        //------------------------------------------------------------------
        // delta.x devices
        //------------------------------------------------------------------
        case (DHD_DEVICE_3DOF):
        case (DHD_DEVICE_DELTA3):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_DELTA_3;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "delta.3";
            m_specifications.m_maxLinearForce                =   20.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.15;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = false;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        case (DHD_DEVICE_6DOF):
        case (DHD_DEVICE_6DOF_500):
        case (DHD_DEVICE_DELTA6):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_DELTA_6;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "delta.6";
            m_specifications.m_maxLinearForce                =   20.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.15;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // omega.x devices
        //------------------------------------------------------------------
        case (DHD_DEVICE_OMEGA):
        case (DHD_DEVICE_OMEGA3):
        case (DHD_CASE_101):
        case (DHD_CASE_110):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_OMEGA_3;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "omega.3";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.075; // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = false;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        case (DHD_DEVICE_OMEGA33):
        case (DHD_DEVICE_OMEGA33_LEFT):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_OMEGA_6;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "omega.6";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.075; // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = leftHandOnly;
            m_specifications.m_rightHand                     = !leftHandOnly;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        case (DHD_DEVICE_OMEGA331):
        case (DHD_DEVICE_OMEGA331_LEFT):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_OMEGA_7;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "omega.7";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    8.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.5;   // [N/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.075; // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(30.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = true;
            m_specifications.m_leftHand                      = leftHandOnly;
            m_specifications.m_rightHand                     = !leftHandOnly;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // sigma.6+ devices
        //------------------------------------------------------------------

        case(DHD_DEVICE_SIGMA33P):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_SIGMA_6P;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "sigma.6+";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxLinearStiffness            = 4000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =  0.090;   // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = false;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
            m_posWorkspaceOffset.set(-0.01, 0.02, 0.00);
        }
        break;

        case(DHD_DEVICE_SIGMA33P_LEFT):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_SIGMA_6P;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "sigma.6+";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxLinearStiffness            = 4000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =  0.090;   // [m]
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = false;
            m_posWorkspaceOffset.set(-0.01,-0.02, 0.00);
        }
        break;

        //------------------------------------------------------------------
        // sigma.7 devices
        //------------------------------------------------------------------
        case(DHD_DEVICE_DLR331):
        case(DHD_DEVICE_SIGMA331):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_SIGMA_7;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "sigma.7";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxGripperForce               =    8.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 4000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =  0.500;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =  0.075;   // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(30.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = true;
            m_specifications.m_leftHand                      = false;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(-0.01, 0.02, 0.00);
        }
        break;

        case(DHD_DEVICE_DLR331_LEFT):
        case(DHD_DEVICE_SIGMA331_LEFT):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_SIGMA_7;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "sigma.7";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxGripperForce               =    8.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 4000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =  0.500;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =  0.075;   // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(30.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = true;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = false;
            m_posWorkspaceOffset.set(-0.01,-0.02, 0.00);
        }
        break;

        //------------------------------------------------------------------
        // falcon device
        //------------------------------------------------------------------
        case(DHD_DEVICE_FALCON):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_FALCON;
            m_specifications.m_manufacturerName              = "Novint Technologies";
            m_specifications.m_modelName                     = "Falcon";
            m_specifications.m_maxLinearForce                =    8.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 3000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   20.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.04;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = false;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // special device - MPR
        //------------------------------------------------------------------
        case (DHD_CASE_111):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_MPR;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "mpr";
            m_specifications.m_maxLinearForce                =   20.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.15;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // special device - XTH-1
        //------------------------------------------------------------------
        case (DHD_CASE_200):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_XTH_1;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "xth-1";
            m_specifications.m_maxLinearForce                =   20.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.15;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = false;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        case (DHD_CASE_201):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_XTH_1;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "xth-1";
            m_specifications.m_maxLinearForce                =   20.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 5000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               =    0.15;  // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = false;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // special device - XTH-2
        //------------------------------------------------------------------
        case(DHD_CASE_112):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_XTH_2;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "xth-2";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 8000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =  0.500;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               = 0.1125;   // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(25.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = false;
            m_specifications.m_rightHand                     = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        case(DHD_CASE_113):
        {
            m_specifications.m_model                         = C_HAPTIC_DEVICE_XTH_2;
            m_specifications.m_manufacturerName              = "Force Dimension";
            m_specifications.m_modelName                     = "xth-2";
            m_specifications.m_maxLinearForce                =   12.0;   // [N]
            m_specifications.m_maxAngularTorque              =    0.2;   // [N*m]
            m_specifications.m_maxGripperForce               =    0.0;   // [N]
            m_specifications.m_maxLinearStiffness            = 8000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness           =    1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness     = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping              =   30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping             =  0.070;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping      =  0.500;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius               = 0.1125;   // [m]
            m_specifications.m_gripperMaxAngleRad            = cDegToRad(25.0);
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = false;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // special device - KSD-1
        //------------------------------------------------------------------

        case(DHD_CASE_114) :
        case(DHD_CASE_204) :
        {
            m_specifications.m_model                        = C_HAPTIC_DEVICE_KSD_1;
            m_specifications.m_manufacturerName             = "Force Dimension";
            m_specifications.m_modelName                    = "ksd-1";
            m_specifications.m_maxLinearForce               = 12.0;   // [N]
            m_specifications.m_maxAngularTorque             = 0.2;   // [N*m]
            m_specifications.m_maxGripperForce              = 0.0;   // [N]
            m_specifications.m_maxLinearStiffness           = 8000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness          = 1.0;   // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness    = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping             = 30.0;   // [N/(m/s)]
            m_specifications.m_maxAngularDamping            = 0.070;   // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping     = 0.500;   // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius              = 0.1125;   // [m]
            m_specifications.m_gripperMaxAngleRad           = cDegToRad(30.0);
            m_specifications.m_sensedPosition               = true;
            m_specifications.m_sensedRotation               = true;
            m_specifications.m_sensedGripper                = true;
            m_specifications.m_actuatedPosition             = true;
            m_specifications.m_actuatedRotation             = true;
            m_specifications.m_actuatedGripper              = false;
            m_specifications.m_leftHand                     = true;
            m_specifications.m_rightHand                    = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;

        //------------------------------------------------------------------
        // special device - VIC 1:
        //------------------------------------------------------------------

        case(DHD_CASE_205) :
        {
            m_specifications.m_model                        = C_HAPTIC_DEVICE_VIC_1;
            m_specifications.m_manufacturerName             = "Force Dimension";
            m_specifications.m_modelName                    = "vic-1";
            m_specifications.m_maxLinearForce               = 12.0;     // [N]
            m_specifications.m_maxAngularTorque             = 0.2;      // [N*m]
            m_specifications.m_maxGripperForce              = 0.0;      // [N]
            m_specifications.m_maxLinearStiffness           = 8000.0;   // [N/m]
            m_specifications.m_maxAngularStiffness          = 1.0;      // [N*m/Rad]
            m_specifications.m_maxGripperLinearStiffness    = 2000.0;   // [N*m/Rad]
            m_specifications.m_maxLinearDamping             = 30.0;     // [N/(m/s)]
            m_specifications.m_maxAngularDamping            = 0.070;    // [N*m/(Rad/s)]
            m_specifications.m_maxGripperAngularDamping     = 0.500;    // [N*m/(Rad/s)]
            m_specifications.m_workspaceRadius              = 0.1125;   // [m]
            m_specifications.m_gripperMaxAngleRad           = cDegToRad(0.0);
            m_specifications.m_sensedPosition               = true;
            m_specifications.m_sensedRotation               = true;
            m_specifications.m_sensedGripper                = false;
            m_specifications.m_actuatedPosition             = true;
            m_specifications.m_actuatedRotation             = true;
            m_specifications.m_actuatedGripper              = false;
            m_specifications.m_leftHand                     = false;
            m_specifications.m_rightHand                    = true;
            m_posWorkspaceOffset.set(0.0, 0.0, 0.0);
        }
        break;
    }


    if (s_dhdEnableExpertMode)
    {
        dhdEnableExpertMode();

        if (s_dhdSetVelocityThreshold)
        {
            if (!s_flagUsingSDK32)
            {
                dhdSetVelocityThreshold(0, m_deviceID);
            }
        }

        if (s_dhdSetWatchdog)
        {
            dhdSetWatchdog (255, m_deviceID);
        }
    }

    // success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method closes the connection to the haptic device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    // check if DHD-API call is available
    if (!s_dhdClose) return (C_ERROR);

    // yes, the device is open so let's close it
    // as long as it was not opened outside of CHAI3D
    int result;
    if (!m_external)
    {
        if (s_drdClose) result = drdClose(m_deviceID);
        else            result = dhdClose(m_deviceID);
    }
    else
    {
        result = 0;
    }

    // update status
    m_deviceReady = false;

    // reset force status
    m_statusEnableForcesFirstTime = true;

    // return result
    if (result < 0)
    {
        return (C_ERROR);
    }
    else
    {
        return (C_SUCCESS);
    }
}


//==============================================================================
/*!
    This methods calibrates the haptic device. \n
    If argument \p a_forceCalibration is equal to __true__, then calibration
    is enforced even if the device has already been calibrated. If
    __false__, then the device is only calibrated if needed.

    \param  a_forceCalibration  Enforce calibration.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::calibrate(bool a_forceCalibration)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // check if DRD-API call is available
    if (!s_drdAutoInit || !s_drdStop || !s_drdIsInitialized)
    {
        // check if DHD-API call is available
        if (!s_dhdReset) return (C_ERROR);

        // reset device (asking user to calibrate manually)
        if (a_forceCalibration)
        {
            if (dhdReset(m_deviceID) < 0) return (C_ERROR);
        }

        return (C_SUCCESS);
    }

    // auto-calibrate is necessary
    if (a_forceCalibration || (drdIsInitialized(m_deviceID) == false))
    {
        if (0 > drdAutoInit(m_deviceID))   return (C_ERROR);
        if (0 > drdStop(true, m_deviceID)) return (C_ERROR);
    }

    // enable force if safe
    if (drdIsInitialized(m_deviceID))
    {
      return enableForces(true);
    }

    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method reads the position of the haptic device.
    Units are meters [m].

    \param  a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::getPosition(cVector3d& a_position)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // check if DHD-API call is available
    if (!s_dhdGetPosition) return (C_ERROR);

    // get position
    double x,y,z;
    int error = dhdGetPosition(&x, &y, &z, m_deviceID);
    if (error >= 0)
    {
        a_position.set(x + m_posWorkspaceOffset(0),
                       y + m_posWorkspaceOffset(1),
                       z + m_posWorkspaceOffset(2));

#if !defined(MACOSX) & !defined(LINUX)
        estimateLinearVelocity(a_position);
#endif
        return (C_SUCCESS);
    }
    else
    {
        a_position.set(0.0, 0.0, 0.0);
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method returns the  linear velocity of the haptic device.
    Units are in [m/s].

    \param  a_linearVelocity  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    int error = 0;

#if defined(MACOSX) | defined(LINUX)
    double vx,vy,vz;
    error = dhdGetLinearVelocity(&vx, &vy, &vz, m_deviceID);
    if (error >= 0)
    {
        m_linearVelocity.set(vx, vy, vz);
    }
#endif

    // retrieve value
    a_linearVelocity = m_linearVelocity;

    // return result
    if (error >= 0)
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
    This method returns the orientation frame of the haptic device end-effector.

    \param  a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    int error = 0;
    cMatrix3d frame;
    frame.identity();

    switch (m_deviceType)
    {
        // first generation
        case (DHD_DEVICE_3DOF):
        case (DHD_DEVICE_6DOF):
        case (DHD_DEVICE_6DOF_500):
        case (DHD_DEVICE_DELTA6):
        {
            // read angles
            cVector3d angles;
            angles.set(0,0,0);

            // check if DHD-API call is available
            if (s_dhdGetOrientationRad)
            {
                error = dhdGetOrientationRad(&angles(0) , &angles(1) , &angles(2) , m_deviceID);
            }

            // compute rotation matrix
            if (error != -1)
            {
                angles.mul(1.5);
                frame.rotateAboutGlobalAxisRad(cVector3d(1,0,0), angles(0) );
                frame.rotateAboutGlobalAxisRad(cVector3d(0,1,0), angles(1) );
                frame.rotateAboutGlobalAxisRad(cVector3d(0,0,1), angles(2) );
            }
        }
        break;

        // second generation
        default:
        {
            // read rotation matrix
            double rot[3][3];
            rot[0][0] = 1.0; rot[0][1] = 0.0; rot[0][2] = 0.0;
            rot[1][0] = 0.0; rot[1][1] = 1.0; rot[1][2] = 0.0;
            rot[2][0] = 0.0; rot[2][1] = 0.0; rot[2][2] = 1.0;

            if (s_dhdGetOrientationFrame)
            {
                error = dhdGetOrientationFrame(rot, m_deviceID);
            }

            if (error != -1)
            {
                frame(0,0) = rot[0][0];
                frame(0,1) = rot[0][1];
                frame(0,2) = rot[0][2];
                frame(1,0) = rot[1][0];
                frame(1,1) = rot[1][1];
                frame(1,2) = rot[1][2];
                frame(2,0) = rot[2][0];
                frame(2,1) = rot[2][1];
                frame(2,2) = rot[2][2];
            }
        }
    }

    // retrieve rotation frame
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // return result
    if (error >= 0)
    {
        return (C_SUCCESS);
    }
    else
    {
        a_rotation.identity();
        return (C_ERROR);
    }
}



//==============================================================================
/*!
    This method returns the joint angles of the haptic device.

    \param  a_jointAnglesRad  Array of returned joints angles.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::getJointAnglesRad(double a_jointAnglesRad[C_MAX_DOF])
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // return joint angles
    if (s_dhdGetJointAngles)
    {
        dhdEnableExpertMode();
        int error = dhdGetJointAngles(a_jointAnglesRad, m_deviceID);
        dhdDisableExpertMode();
        if (error < 0)
        {
            return (C_ERROR);
        }
        else
        {
            return (C_SUCCESS);
        }
    }
    else
    {
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method returns the gripper angle in radian [rad].

    \param  a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::getGripperAngleRad(double& a_angle)
{
    // default value
    a_angle = 0.0;

    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // read gripper angle
    int error = 0;
    if (m_specifications.m_sensedGripper)
    {
        if (s_dhdGetGripperAngleRad)
        {
            double angle = 0.0;
            error = dhdGetGripperAngleRad(&angle, m_deviceID);

            if (m_specifications.m_model == C_HAPTIC_DEVICE_XTH_2)
            {
                a_angle = angle;
            }
            else
            {
                if (m_specifications.m_rightHand)
                {
                    a_angle = cClamp0(angle);
                }
                else if (m_specifications.m_leftHand)
                {
                    a_angle = cClamp0(-angle);
                }
            }
        }
        else
        {
            a_angle = 0.0;
        }

        estimateGripperVelocity(a_angle);
    }
    else
    {
        cGenericHapticDevice::getGripperAngleRad(a_angle);
    }

    // return result
    if (error >= 0)
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
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m]
    to the haptic device.

    \param  a_force         Force command.
    \param  a_torque        Torque command.
    \param  a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                   const cVector3d& a_torque,
                                                   double a_gripperForce)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // check if forces need to be enable (happens only once)
    if (m_statusEnableForcesFirstTime) { enableForces(true); }

    // computer gripper user switch gripper force
    double gripperAngle;
    double gripperUserSwitchForce = 0.0;
    if (getGripperAngleDeg(gripperAngle) == 1)
    {
        gripperUserSwitchForce = computeGripperUserSwitchForce(gripperAngle, 0.0);
    }

    // adjust gripper command for left or right hand device
    double gripperForce = a_gripperForce + gripperUserSwitchForce;
    if (m_specifications.m_leftHand)
    {
        gripperForce =-gripperForce;
    }

    if (m_specifications.m_actuatedRotation)
    {
        if (s_dhdSetForceAndTorqueAndGripperForce)
        {
            int error = dhdSetForceAndTorqueAndGripperForce(a_force(0),
                                                            a_force(1),
                                                            a_force(2),
                                                            a_torque(0),
                                                            a_torque(1),
                                                            a_torque(2),
                                                            gripperForce,
                                                            m_deviceID);
            if (error < 0) { return (C_ERROR); }

            // store new commanded values
            m_prevForce  = a_force;
            m_prevTorque  = a_torque;
            m_prevGripperForce = a_gripperForce;
        }
        else
        {
            return (C_ERROR);
        }
    }
    else
    {
        if (s_flagUsingSDK32 && !m_specifications.m_actuatedGripper && s_dhdSetForce)
        {
            int error = dhdSetForce(a_force(0),
                a_force(1),
                a_force(2),
                m_deviceID);
            if (error < 0) { return (C_ERROR); }

            // store new commanded values
            m_prevForce = a_force;
        }
        else if (s_dhdSetForceAndGripperForce)
        {
            int error = dhdSetForceAndGripperForce(a_force(0),
                                                   a_force(1),
                                                   a_force(2),
                                                   gripperForce,
                                                   m_deviceID);
            if (error < 0) { return (C_ERROR); }

            // store new commanded values
            m_prevForce = a_force;
            m_prevGripperForce = a_gripperForce;
        }
        else
        {
            return (C_ERROR);
        }
    }


    // success
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
bool cDeltaDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // retrieve button data
    unsigned int result = 0;
    if (s_dhdGetButtonMask)
    {
        result = dhdGetButtonMask(m_deviceID) & 0x001F;
    }
    else if (s_dhdGetButton)
    {
        if (dhdGetButton(0, m_deviceID) == 1)
        {
            result = true;
        }
        else
        {
            result = false;
        }
    }

    bool gripperSwitch = getGripperUserSwitch();
    if (gripperSwitch)
    {
        result = result | 1;
    }

    // special case
    if (dhdGetSystemType (m_deviceID) == 110)
    {
        if (result & 2)
        {
            result = result | 1;
        }
    }

    // return result
    a_userSwitches = result;

    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method enables or disables the motors of the haptic device.
    This function overrides the force button located at the base of the
    device or on the controller panel.

    \param   a_value  Actuation status.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::enableForces(bool a_value)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // check if DHD-API call is available
    if (!s_dhdEnableExpertMode) return (C_ERROR);
    if (!s_dhdEnableForce) return (C_ERROR);
    if (!s_dhdDisableExpertMode) return (C_ERROR);

    int error;
    if (a_value)
    {
        // enable forces
        dhdEnableExpertMode();
        error = dhdEnableForce(DHD_ON, m_deviceID);
        dhdDisableExpertMode();
        m_statusEnableForcesFirstTime = false;
    }
    else
    {
        // disable forces
        dhdEnableExpertMode();
        error = dhdEnableForce(DHD_OFF, m_deviceID);
        dhdDisableExpertMode();
    }

    // return result
    if (error >= 0)
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
    This mehtod sets the device base plate angle around the Y axis.
    Please refer to your device user manual for more information on your device
    coordinate system.

    An angle value of 0 corresponds to the device "upright" position, with its
    base plate perpendicular to axis X. An angle value of 90 corresponds to
    the device base plate resting horizontally.

    \param   a_angleDeg  Angle in degrees.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::setDeviceAngleDeg(double a_angleDeg)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // sanity check
    if (!s_dhdSetDeviceAngleDeg) return (C_ERROR);

    // assign device angle around y-axis
    int error = dhdSetDeviceAngleDeg(a_angleDeg, m_deviceID);

    // return result
    if (error >= 0)
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
    This method assign a vibration to the devices that support this feature.

    \param   a_freq  Frequency in Hz.
    \param   a_amplitude  Vibration amplitude.
    \param   a_type  Vibration signal type.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDeltaDevice::setVibration(double a_freq, double a_amplitude, int a_type)
{
    // check if the system is available
    if (!m_deviceReady) return (C_ERROR);

    // sanity check
    if (!s_dhdSetVibration) return (C_ERROR);

    // assign vibration
    int error = dhdSetVibration(a_freq, a_amplitude, a_type, m_deviceID);

    // return result
    if (error >= 0)
    {
        return (C_SUCCESS);
    }
    else
    {
        return (C_ERROR);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //C_ENABLE_DELTA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
