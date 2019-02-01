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
    \version   3.2.1 $Rev: 1922 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CRazerHydraDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Number of instances for this class of devices currently using the libraries.
unsigned int cRazerHydraDevice::s_libraryCounter = 0;
std::shared_ptr<razer_hydra::RazerHydra> cRazerHydraDevice::s_hydra_dev;
std::mutex cRazerHydraDevice::m_mutex;

// Allocation table for devices of this class.
bool cRazerHydraDevice::s_allocationTable[C_MAX_DEVICES] = {false, false, false, false,
    false, false, false, false,
    false, false, false, false,
    false, false, false, false}; 

//! Time guard for data acquisition.
cPrecisionClock cRazerHydraDevice::m_timeguard;


#if defined(LINUX)

    void * RazerHydraSO = NULL;

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
bool cRazerHydraDevice::openLibraries()
{ 
    // increment number of instances using the libraries for this class of devices
    s_libraryCounter++;

    // if libraries are already initialized, then we are done
    if (s_libraryCounter > 1) return (C_SUCCESS);


    ////////////////////////////////////////////////////////////////////////////
    // initialize libraries
    ////////////////////////////////////////////////////////////////////////////


#if defined(LINUX) | (MACOSX)

    // load shared library
//    #ifdef __LP64__
//    RazerHydraSO = dlopen ("libsixense_x64.so", RTLD_NOW);
//    #else
//    RazerHydraSO = dlopen ("libsixense.so", RTLD_NOW);
//    #endif

#endif

#if defined(MACOSX)

    // load shared library
    #ifdef __LP64__
    RazerHydraSO = dlopen ("libsixense_x64.dylib", RTLD_NOW);
    #else
    RazerHydraSO = dlopen ("libsixense.dylib", RTLD_NOW);
    #endif


#endif

#if defined(LINUX) | defined(MACOSX)

//    // check that it loaded correctly
//    if (RazerHydraSO == NULL)
//    {
//        s_libraryCounter = 0;
//        return (C_ERROR);
//    }


#endif

    // initialize libraries
    std::string dev_name = "/dev/hydra";
    s_hydra_dev.reset(new razer_hydra::RazerHydra());
    if (s_hydra_dev->init(dev_name.c_str()) == true)
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
bool cRazerHydraDevice::closeLibraries()
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
    #if defined(LINUX) | defined(MACOSX)
    if ((s_libraryCounter == 0) && (RazerHydraSO != NULL))
    {
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
unsigned int cRazerHydraDevice::getNumDevices()
{
    // open libraries
    openLibraries();

    // sanity check
    unsigned int result = 0;
    if (s_libraryCounter < 1) return (C_ERROR);
    else{result = 2;}

    // close libraries
    closeLibraries();

    // return number of devices
    return (result);
}


//==============================================================================
/*!
    Constructor of cRazerHydraDevice.
*/
//==============================================================================
cRazerHydraDevice::cRazerHydraDevice(unsigned int a_deviceNumber)
{
    // initialize time guard for data acquisition
    m_timeguard.setTimeoutPeriodSeconds(0.003);
    m_timeguard.start(true);

    // initialize specifications
    m_specifications.m_model                         = C_TRACKER_DEVICE_RAZER;
    m_specifications.m_manufacturerName              = "Razer";
    m_specifications.m_modelName                     = "Razer Hydra";
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
    Destructor of cRazerHydraDevice.
*/
//==============================================================================
cRazerHydraDevice::~cRazerHydraDevice()
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
bool cRazerHydraDevice::open()
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

    s_hydra_dev.reset(new razer_hydra::RazerHydra());
    std::string dev_name = "/dev/hydra";

    // flag the device as ready for use
    m_deviceReady = s_hydra_dev->init(dev_name.c_str());
    // Clear the buttons to prevent garbage values
    s_hydra_dev->raw_buttons[0] = 0;
    s_hydra_dev->raw_buttons[1] = 0;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This methods closes the connection to the device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cRazerHydraDevice::close()
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
bool cRazerHydraDevice::calibrate(bool a_forceCalibration)
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
bool cRazerHydraDevice::getPosition(cVector3d& a_position)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // Lock the instance
    std::lock_guard<std::mutex> lock(m_mutex);
    // acquire data from controller
    updateData();

    // return data
    tf::Vector3 grab(0.047,0.035,0.0);
    tf::Transform tran(s_hydra_dev->quat[m_deviceNumber], s_hydra_dev->pos[m_deviceNumber]);
    tf::Vector3 pos = tran * grab;
    double x =  -pos.getX() - 0.5;
    double y =  -pos.getY();
    double z =   pos.getZ();
    a_position.set(x, y, z);

    // estimate angular velocity
    estimateLinearVelocity(a_position);

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
bool cRazerHydraDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // Lock the instance
    std::lock_guard<std::mutex> lock(m_mutex);
    // acquire data from controller
    updateData();

    cMatrix3d frame;
    a_rotation.setAxisAngleRotationRad(-s_hydra_dev->quat[m_deviceNumber].getAxis().getX(),
                                       -s_hydra_dev->quat[m_deviceNumber].getAxis().getY(),
                                        s_hydra_dev->quat[m_deviceNumber].getAxis().getZ(),
                                        s_hydra_dev->quat[m_deviceNumber].getAngle());

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
bool cRazerHydraDevice::getGripperAngleRad(double& a_angle)
{
    // default value
    a_angle = 0.0;

    // Lock the instance
    std::lock_guard<std::mutex> lock(m_mutex);
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // read gripper angle
    a_angle =  1.0 - s_hydra_dev->analog[(3*m_deviceNumber)+2];

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
bool cRazerHydraDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // Lock the instance
    std::lock_guard<std::mutex> lock(m_mutex);
    a_userSwitches = s_hydra_dev->raw_buttons[m_deviceNumber];

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Update data from controllers.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cRazerHydraDevice::updateData()
{
    // check if the system is available
    if (!m_deviceReady)
    {
        return (C_ERROR);
    }

    // update data
    if (m_timeguard.timeoutOccurred())
    {
        s_hydra_dev->poll(5, 3);
        m_timeguard.start(true);
    }

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //C_ENABLE_RAZER_HYDRA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
