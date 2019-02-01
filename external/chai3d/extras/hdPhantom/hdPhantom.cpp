//===========================================================================
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

    \author     Federico Barbagli
    \author     Francois Conti
    \author     Sebastien Grange
*/
//===========================================================================

//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#include <windows.h>
#include "assert.h"
#include <conio.h>
#endif
//---------------------------------------------------------------------------
#ifdef LINUX
#include <pthread.h>
struct  Event                                    { pthread_mutex_t m; pthread_cond_t  c; };
typedef Event* HANDLE;
void    SetEvent(HANDLE e)                       { pthread_cond_signal (&(e->c)); }
HANDLE  CreateEvent(void *, bool r, bool, char*) { HANDLE e = new Event; pthread_mutex_init(&(e->m), NULL); pthread_cond_init(&(e->c), NULL); return e; }
int     WaitForSingleObject(HANDLE e, unsigned)  { pthread_mutex_lock(&(e->m)); pthread_cond_wait(&(e->c), &(e->m)); pthread_mutex_unlock(&(e->m)); return 0; }
void    CloseHandle(HANDLE e)                    { pthread_mutex_destroy(&(e->m)); pthread_cond_destroy(&(e->c)); delete e; }
#endif
//---------------------------------------------------------------------------
#include "hdPhantom.h"
#include "HD/hd.h"
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// maximum supported devices
#if defined(WIN32) | defined(WIN64)
#define PHANTOM_NUM_DEVICES_MAX 4
#endif
#ifdef LINUX
#define PHANTOM_NUM_DEVICES_MAX 1
#endif

// structure used to store data related to each device entity
struct CPhantomDeviceStatus
{
    HHD handle;
    double position[3];
    double linearVelocity[3];
    double rotation[9];
    double force[3];
    double torque[3];
    int    button;
    bool   enabled;
    bool   initialized;
    double workspaceRadius;
    HANDLE sync;
};

// has DLL been initialed
bool initPhantomDLL = false;

// attached process counter
static int processCount = 0;

// table containing information for each device.
CPhantomDeviceStatus phantomDevices[PHANTOM_NUM_DEVICES_MAX];

// predefined value that expresses the absence of a Phantom.
int numPhantomDevices = 0;

// has servo controller been started yet
bool servoStarted = false;

// main servo controller callback
HDCallbackCode servoCallbackHandle;
HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData);


//=============================================================================
// library entry point
// =============================================================================

#ifdef LINUX

// these will only get executed once when the library is loaded,
// but should do the job just fine regardless
void __attribute__ ((constructor)) _hdLoad(void);
void __attribute__ ((destructor))  _hdUnload(void);

#endif


void
_hdLoad(void)
{
    processCount++;
    if (!initPhantomDLL)
    {
        initPhantomDLL = true;

        for (int i=0; i<PHANTOM_NUM_DEVICES_MAX; i++)
        {
            // init button data
            phantomDevices[i].button = 0;

            // init position data
            phantomDevices[i].position[0] = 0.0;
            phantomDevices[i].position[1] = 0.0;
            phantomDevices[i].position[2] = 0.0;

            // init rotation data
            phantomDevices[i].rotation[0] = 1.0;
            phantomDevices[i].rotation[1] = 0.0;
            phantomDevices[i].rotation[2] = 0.0;
            phantomDevices[i].rotation[3] = 0.0;
            phantomDevices[i].rotation[4] = 1.0;
            phantomDevices[i].rotation[5] = 0.0;
            phantomDevices[i].rotation[6] = 0.0;
            phantomDevices[i].rotation[7] = 0.0;
            phantomDevices[i].rotation[8] = 1.0;

            // init force data
            phantomDevices[i].force[0] = 0.0;
            phantomDevices[i].force[1] = 0.0;
            phantomDevices[i].force[2] = 0.0;

            // init torque data
            phantomDevices[i].torque[0] = 0.0;
            phantomDevices[i].torque[1] = 0.0;
            phantomDevices[i].torque[2] = 0.0;

            // init enable/disable data
            phantomDevices[i].enabled = false;

            // init phantom api initialized
            phantomDevices[i].initialized = false;

            // device synchronization
            phantomDevices[i].sync = CreateEvent(NULL, false, false, NULL);
        }

        //------------------------------------------------------------------
        // INITIALIZE DEVICE 0
        //------------------------------------------------------------------
        HDErrorInfo error;
        numPhantomDevices = 0;

        // search for a first device
        HHD hHD0 = hdInitDevice(HD_DEFAULT_DEVICE);

        // check if device is available
        if (!HD_DEVICE_ERROR(error = hdGetError()) && hHD0 != HD_INVALID_HANDLE)
        {
            // enable forces
            hdMakeCurrentDevice(hHD0);
            hdEnable(HD_FORCE_OUTPUT);

            // add device to list
            phantomDevices[numPhantomDevices].handle = hHD0;
            phantomDevices[numPhantomDevices].enabled = true;
            numPhantomDevices++;
        }


        // exit if only single device are supported
        if (PHANTOM_NUM_DEVICES_MAX == 1)
        {
            return;
        }


        //------------------------------------------------------------------
        // INITIALIZE DEVICE 1
        //------------------------------------------------------------------

        // search for a possible second device
        HHD hHD1 = hdInitDevice("Phantom2");

        // check if device is available
		if (!HD_DEVICE_ERROR(error = hdGetError()) && hHD1 != HD_INVALID_HANDLE)
        {
            // enable forces
            hdMakeCurrentDevice(hHD1);
            hdEnable(HD_FORCE_OUTPUT);

            // add device to list
            phantomDevices[numPhantomDevices].handle = hHD1;
            phantomDevices[numPhantomDevices].enabled = true;
            numPhantomDevices++;
        }

        //------------------------------------------------------------------
        // INITIALIZE DEVICE 2
        //------------------------------------------------------------------

        // search for a possible second device
        HHD hHD2 = hdInitDevice("Phantom3");

        // check if device is available
		if (!HD_DEVICE_ERROR(error = hdGetError()) && hHD2 != HD_INVALID_HANDLE)
        {
            // enable forces
            hdMakeCurrentDevice(hHD2);
            hdEnable(HD_FORCE_OUTPUT);

            // add device to list
            phantomDevices[numPhantomDevices].handle = hHD2;
            phantomDevices[numPhantomDevices].enabled = true;
            numPhantomDevices++;
        }

        //------------------------------------------------------------------
        // INITIALIZE DEVICE 3
        //------------------------------------------------------------------

        // search for a possible second device
        HHD hHD3 = hdInitDevice("Phantom4");

        // check if device is available
		if (!HD_DEVICE_ERROR(error = hdGetError()) && hHD3 != HD_INVALID_HANDLE)
        {
            // enable forces
            hdMakeCurrentDevice(hHD3);
            hdEnable(HD_FORCE_OUTPUT);

            // add device to list
            phantomDevices[numPhantomDevices].handle = hHD3;
            phantomDevices[numPhantomDevices].enabled = true;
            numPhantomDevices++;
        }
    }
}

void _hdUnload(void)
{
    processCount--;
    if ((processCount == 0) && (initPhantomDLL))
    {
        for (int i=0; i<numPhantomDevices; i++)
        {
            // device synchronization
            CloseHandle(phantomDevices[i].sync);
        }

        //------------------------------------------------------------------
        // CLOSE DEVICES
        //------------------------------------------------------------------

        // \todo

        initPhantomDLL = false;
    }
}

#if defined(WIN32) | defined(WIN64)

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved)
{
    switch (ul_reason_for_call) 
    {
    case DLL_PROCESS_ATTACH:
        _hdLoad();
        return (true);
    case DLL_PROCESS_DETACH:
        _hdUnload();
        return(true);
    }
}

#endif


//==========================================================================
// FUNCTIONS ACCESSIBLE FROM OUTSIDE
//==========================================================================

//==========================================================================
/*!
    Retrieves the number of devices of type phantom.

    \fn     int __FNCALL hdPhantomGetNumDevices()

    \return Returns the number of devices found      
*/
//==========================================================================
int __FNCALL hdPhantomGetNumDevices()
{
    return (numPhantomDevices);
}


//==========================================================================
/*!
    Open a connexion to the device selected by a_deviceID.

    \fn     int __FNCALL hdPhantomOpen(const int a_deviceID)

    \param  a_deviceID device identification number. 
    
     \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL hdPhantomOpen(const int a_deviceID)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // init device
    if (!phantomDevices[a_deviceID].initialized)
    {
        phantomDevices[a_deviceID].initialized = true;
    }

    // enable device
    phantomDevices[a_deviceID].enabled = true;

    // return result
    return (phantomDevices[a_deviceID].handle);
}


//==========================================================================
/*!
    Closes a connexion to the device selected by a_deviceID.

    \fn     int __FNCALL hdPhantomClose(const int a_deviceID)

    \param  a_deviceID device identification number. 
    
    \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL hdPhantomClose(const int a_deviceID)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // disable device
    phantomDevices[a_deviceID].enabled = false;

    // exit
    return (0);
}


//==========================================================================
/*!
    Returns the position of the device end-effector.

    \fn       int __FNCALL hdPhantomGetPosition(const int a_deviceID,
                                                         double *a_posX,
                                                         double *a_posY,
                                                         double *a_posZ)

    \param  a_deviceID device identification number.
    \param  a_posX Component X of position vector.
    \param  a_posY Component Y of position vector.
    \param  a_posZ Component Z of position vector.

    \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL hdPhantomGetPosition(const int a_deviceID,
                                  double *a_posX,
                                  double *a_posY,
                                  double *a_posZ)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // get position
    *a_posX = phantomDevices[a_deviceID].position[2];
    *a_posY = phantomDevices[a_deviceID].position[0];
    *a_posZ = phantomDevices[a_deviceID].position[1];

    // success
    return (0);
}


//==========================================================================
/*!
    Returns the linear velocity of the device end-effector.

    \fn       int __FNCALL hdPhantomGetLinearVelocity(const int a_deviceID,
                                                      double *a_velX,
                                                      double *a_velY,
                                                      double *a_velZ)

    \param  a_deviceID device identification number.
    \param  a_velX Component X of velocity vector.
    \param  a_velY Component Y of velocity vector.
    \param  a_velZ Component Z of velocity vector.

    \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL hdPhantomGetLinearVelocity(const int a_deviceID,
                                        double *a_velX,
                                        double *a_velY,
                                        double *a_velZ)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // get linear velocity
    *a_velX = phantomDevices[a_deviceID].linearVelocity[2];
    *a_velY = phantomDevices[a_deviceID].linearVelocity[0];
    *a_velZ = phantomDevices[a_deviceID].linearVelocity[1];

    // success
    return (0);
}


//==========================================================================
/*!
    Returns the orientation matrix (frame) of the device end-effector.

    \fn     int __FNCALL hdPhantomGetRotation(const int a_deviceID,
                                              double *a_rot00,
                                              double *a_rot01,
                                              double *a_rot02,
                                              double *a_rot10,
                                              double *a_rot11,
                                              double *a_rot12,
                                              double *a_rot20,
                                              double *a_rot21,
                                              double *a_rot22)

    \param  a_deviceID device identification number.
    \param  double a_rot00 Component [0,0] of rotation matrix.
    \param  double a_rot01 Component [0,1] of rotation matrix.
    \param  double a_rot02 Component [0,2] of rotation matrix.
    \param  double a_rot10 Component [1,0] of rotation matrix.
    \param  double a_rot11 Component [1,1] of rotation matrix.
    \param  double a_rot12 Component [1,2] of rotation matrix.
    \param  double a_rot20 Component [2,0] of rotation matrix.
    \param  double a_rot21 Component [2,1] of rotation matrix.
    \param  double a_rot22 Component [2,2] of rotation matrix.

    \return Return 0 if success, otherwise -1
*/
//==========================================================================
int __FNCALL hdPhantomGetRotation(const int a_deviceID,
                                  double *a_rot00,
                                  double *a_rot01,
                                  double *a_rot02,
                                  double *a_rot10,
                                  double *a_rot11,
                                  double *a_rot12,
                                  double *a_rot20,
                                  double *a_rot21,
                                  double *a_rot22)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // return rotation matrix
    /*
    *a_rot00 = phantomDevices[a_deviceID].rotation[8];	
    *a_rot01 = phantomDevices[a_deviceID].rotation[2];	
    *a_rot02 = phantomDevices[a_deviceID].rotation[5];	
    *a_rot10 = phantomDevices[a_deviceID].rotation[6];	
    *a_rot11 = phantomDevices[a_deviceID].rotation[0];	
    *a_rot12 = phantomDevices[a_deviceID].rotation[3];	
    *a_rot20 = phantomDevices[a_deviceID].rotation[7];	
    *a_rot21 = phantomDevices[a_deviceID].rotation[1];	
    *a_rot22 = phantomDevices[a_deviceID].rotation[4];	
    */

    // read value from matrix and correct matrix to be orthogonal
    // unfortunately there seems be some small errors coming
    // from the OpenHaptics library, so we condition the matrix.
    cVector3d v0, v1, v2;
    v0.set( phantomDevices[a_deviceID].rotation[8],
            phantomDevices[a_deviceID].rotation[6],
            phantomDevices[a_deviceID].rotation[7]);
    v1.set( phantomDevices[a_deviceID].rotation[2],
            phantomDevices[a_deviceID].rotation[0],
            phantomDevices[a_deviceID].rotation[1]);
    v0.normalize();
    v1.normalize();
    v0.crossr(v1, v2);
    v2.crossr(v0, v1);

    *a_rot00 = v0.x();	
    *a_rot01 = v1.x(); 
    *a_rot02 = v2.x(); 
    *a_rot10 = v0.y(); 
    *a_rot11 = v1.y();
    *a_rot12 = v2.y();
    *a_rot20 = v0.z(); 
    *a_rot21 = v1.z(); 
    *a_rot22 = v2.z();

    // success
    return (0);
}


//==========================================================================
/*!
    Read the values of each end-effector user button.

    \fn     int __FNCALL hdPhantomGetButtons(const int a_deviceID)

    \param  a_deviceID device identification number.
    
    \return Return a integer for which each bit corresponds to a button status.
*/
//==========================================================================
int __FNCALL hdPhantomGetButtons(const int a_deviceID)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // return value
    return (phantomDevices[a_deviceID].button);
}


//==========================================================================
/*!
    Send a force to the device

    \fn     int __FNCALL hdPhantomSetForce(const int a_deviceID,
                                           double *a_forceX,
                                           double *a_forceY,
                                           double *a_forceZ)

    \param  a_deviceID device identification number.
    \param  a_forceX Component X of force vector.
    \param  a_forceY Component Y of force vector.
    \param  a_forceZ Component Z of force vector.
    \return Return 0 if success, otherwise -1
*/
//==========================================================================
int __FNCALL hdPhantomSetForce(const int a_deviceID,
                               double *a_forceX,
                               double *a_forceY,
                               double *a_forceZ)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // set force
    phantomDevices[a_deviceID].force[2] = *a_forceX;
    phantomDevices[a_deviceID].force[0] = *a_forceY;
    phantomDevices[a_deviceID].force[1] = *a_forceZ;

    // wait for synchronization event
    WaitForSingleObject(phantomDevices[a_deviceID].sync, 1000);

    // success
    return (0);
}


//==========================================================================
/*!
    Send a torque to the device

    \fn     int __FNCALL hdPhantomSetTorque(const int a_deviceID,
                                            double *a_torqueX,
                                            double *a_torqueY,
                                            double *a_torqueZ)

    \param  a_deviceID device identification number.
    \param  a_torqueX Component X of torque vector.
    \param  a_torqueY Component Y of torque vector.
    \param  a_torqueZ Component Z of torque vector.

    \return Return 0 if success, otherwise -1
*/
//==========================================================================
int __FNCALL hdPhantomSetTorque(const int a_deviceID,
                                double *a_torqueX,
                                double *a_torqueY,
                                double *a_torqueZ)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // set torque
    phantomDevices[a_deviceID].torque[2] = *a_torqueX;
    phantomDevices[a_deviceID].torque[0] = *a_torqueY;
    phantomDevices[a_deviceID].torque[1] = *a_torqueZ;

    // wait for synchronization event
    WaitForSingleObject(phantomDevices[a_deviceID].sync, 1000);

    // success
    return (0);
}


//==========================================================================
/*!
    Send a force and a torque to the device

    \fn     int hdPhantomSetForceAndTorque(const int a_deviceID,
                double *a_forceX,
                double *a_forceY,
                double *a_forceZ,
                double *a_torqueX,
                double *a_torqueY,
                double *a_torqueZ)

    \param  a_deviceID device identification number.
    \param  double a_forceX Component X of force vector.
    \parma  double a_forceY Component Y of force vector.
    \param  double a_forceZ Component Z of force vector.
    \param  double a_torqueX Component X of torque vector.
    \parma  double a_torqueY Component Y of torque vector.
    \param  double a_torqueZ Component Z of torque vector.

    \return Return 0 if success, otherwise -1
*/
//==========================================================================
int __FNCALL hdPhantomSetForceAndTorque(const int a_deviceID,
                                        double *a_forceX,
                                        double *a_forceY,
                                        double *a_forceZ,
                                        double *a_torqueX,
                                        double *a_torqueY,
                                        double *a_torqueZ)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // set force
    phantomDevices[a_deviceID].force[2] = *a_forceX;
    phantomDevices[a_deviceID].force[0] = *a_forceY;
    phantomDevices[a_deviceID].force[1] = *a_forceZ;

    // set torque
    phantomDevices[a_deviceID].torque[2] = *a_torqueX;
    phantomDevices[a_deviceID].torque[0] = *a_torqueY;
    phantomDevices[a_deviceID].torque[1] = *a_torqueZ;

    // wait for synchronization event
    WaitForSingleObject(phantomDevices[a_deviceID].sync, 1000);

    // success
    return (0);
}


//==========================================================================
/*!
    Send a force to the device

    \fn     int hdPhantomGetWorkspaceRadius(int a_deviceID, double *a_workspaceRadius)

    \param  a_deviceID device identification number.
    \param  a_workspaceRadius  Radius of the workspace

    \return Return 0 if success, otherwise -1
*/
//==========================================================================
int __FNCALL hdPhantomGetWorkspaceRadius(const int a_deviceID,
                                         double *a_workspaceRadius)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // retrieve handle
    HHD hHD = phantomDevices[a_deviceID].handle;

    // activate ith device
    hdMakeCurrentDevice(hHD);

    // read workspace of device
    double size[6];
    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, size);
    double sizeX = size[3] - size[0];
    double sizeY = size[4] - size[1];
    double sizeZ = size[5] - size[2];
    double radius = 0.5 * sqrt(sizeX * sizeX +
                               sizeY * sizeY +
                               sizeZ * sizeZ);

    // convert value to [m]
    phantomDevices[a_deviceID].workspaceRadius = 0.001 * radius;

    // return estimated workspace radius
    *a_workspaceRadius = phantomDevices[a_deviceID].workspaceRadius;

    // success
    return (0);
}


//==========================================================================
/*!
    Read the name type of the device

    \fn     int hdPhantomGetType(int a_deviceID, 
                                 const char* a_typeName)

    \param  a_deviceID device identification number.
    \param  a_typeName  String containing the the device type.

    \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL hdPhantomGetType(const int a_deviceID,
                              char* a_typeName)
{
    // check id
    if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

    // check if servo started
    if (!servoStarted) { hdPhantomStartServo(); }

    // check if enabled
    if (!phantomDevices[a_deviceID].enabled) { return (-1); }

    // retrieve handle
    HHD hHD = phantomDevices[a_deviceID].handle;

    // activate ith device
    hdMakeCurrentDevice(hHD);

    // read device model
    const char* typeName = hdGetString(HD_DEVICE_MODEL_TYPE);
    strcpy(a_typeName, typeName);

    // exit
    return (0);
}


//==========================================================================
/*
    Start servo controller

    \fn     void hdPhantomStartServo(void)
*/
//==========================================================================
void __FNCALL hdPhantomStartServo(void)
{
    if (!servoStarted)
    {
        // servo controller has been started
        servoStarted = true;

        // create callback
        HDCallbackCode servoCallbackHandle = hdScheduleAsynchronous(
        servoPhantomDevices, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

        // start scheduler
        hdStartScheduler();
    }
}


//==========================================================================
/*
    Stop  servo controller

    \fn     void __FNCALL hdPhantomStopServo(void)
*/
//==========================================================================
void __FNCALL hdPhantomStopServo(void)
{
    if (servoStarted)
    {
        // check if any device is enabled
        bool deviceEnabled = false;
        for (int i=0; i<PHANTOM_NUM_DEVICES_MAX; i++)
        {
           deviceEnabled = deviceEnabled || phantomDevices[i].enabled;
        }

        // if a device is still enabled, do not stop servo loop
        if (deviceEnabled) { return; };

        // stop servo controller
        servoStarted = false;
        hdStopScheduler();
    }
}


//==========================================================================
// FUNCTIONS INTERNAL TO THE DLL
//==========================================================================

//==========================================================================
/*
    Servo controller callback

    \fn     HDLServoOpExitCode servophantomDevices(void* pUserData)

    \param  pUserData pointer to user data information (not used here)
*/
//==========================================================================
HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData)
{
    for (int i=0; i<numPhantomDevices; i++)
    {
        // for each activated phantom device
        if (phantomDevices[i].enabled)
        {
            // retrieve handle
            HHD hHD = phantomDevices[i].handle;

            // activate ith device
            hdMakeCurrentDevice(hHD);

            // start sending commands
            hdBeginFrame(hHD);
            
            // retrieve the position and orientation of the end-effector.
            double frame[16];
            hdGetDoublev(HD_CURRENT_TRANSFORM, frame);

            // convert position from [mm] to [m] 
            frame[12] = frame[12] * 0.001;
            frame[13] = frame[13] * 0.001;
            frame[14] = frame[14] * 0.001;

            phantomDevices[i].position[0] = frame[12];
            phantomDevices[i].position[1] = frame[13];
            phantomDevices[i].position[2] = frame[14];

            phantomDevices[i].rotation[0] = frame[0];
            phantomDevices[i].rotation[1] = frame[1];
            phantomDevices[i].rotation[2] = frame[2];
            phantomDevices[i].rotation[3] = frame[4];
            phantomDevices[i].rotation[4] = frame[5];
            phantomDevices[i].rotation[5] = frame[6];
            phantomDevices[i].rotation[6] = frame[8];
            phantomDevices[i].rotation[7] = frame[9];
            phantomDevices[i].rotation[8] = frame[10];

            // read linear velocity
            double vel[3];
            hdGetDoublev(HD_CURRENT_VELOCITY, vel);

            // convert position from [mm] to [m] 
            vel[0] = vel[0] * 0.001;
            vel[1] = vel[1] * 0.001;
            vel[2] = vel[2] * 0.001;
            
            phantomDevices[i].linearVelocity[0] = vel[0];
            phantomDevices[i].linearVelocity[1] = vel[1];
            phantomDevices[i].linearVelocity[2] = vel[2];

            // read user buttons
            int buttons;
            hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
            phantomDevices[i].button = buttons;

            // issue synchronization event
            SetEvent (phantomDevices[i].sync);

            // send force to end-effector
            double force[3];
            force[0] = phantomDevices[i].force[0];
            force[1] = phantomDevices[i].force[1];
            force[2] = phantomDevices[i].force[2];
            hdSetDoublev(HD_CURRENT_FORCE, force);

            // send torque to end-effector
            double torque[3];
            torque[0] = phantomDevices[i].torque[0] * 1000.0;
            torque[1] = phantomDevices[i].torque[1] * 1000.0;
            torque[2] = phantomDevices[i].torque[2] * 1000.0;
            hdSetDoublev(HD_CURRENT_TORQUE, torque);

            // flush commands
            hdEndFrame(hHD);
        }
    }

    return (HD_CALLBACK_CONTINUE);
}
