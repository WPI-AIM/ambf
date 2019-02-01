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
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericHapticDevice. \n
    Initializes basic parameters of generic haptic device class.

    \param  a_deviceNumber  Device number ID for this class of devices.
*/
//==============================================================================
cGenericHapticDevice::cGenericHapticDevice(unsigned int a_deviceNumber) : cGenericDevice(a_deviceNumber)
{
    // set default values
    m_specifications.m_manufacturerName              = "CHAI3D";
    m_specifications.m_modelName                     = "no device";
    m_specifications.m_maxLinearForce                = 0.1; // [N]
    m_specifications.m_maxAngularTorque              = 0.1; // [N*m]
    m_specifications.m_maxGripperForce               = 0.1; // [N]
    m_specifications.m_maxLinearStiffness            = 0.1; // [N/m]
    m_specifications.m_maxAngularStiffness           = 0.1; // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 0.1; // [N*m]
    m_specifications.m_maxLinearDamping              = 0.0; // [N/(m/s)]
    m_specifications.m_maxAngularDamping			 = 0.0; // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping		 = 0.0; // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.1; // [m]
    m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
    m_specifications.m_sensedPosition                = false;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = false;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = false;
    m_specifications.m_rightHand                     = false;

    // initialize variables about the state of the haptic device
    m_prevForce.zero();
    m_prevTorque.zero();
    m_prevGripperForce = 0.0;

    m_angularVelocity.zero();
    m_linearVelocity.zero();
    m_gripperAngularVelocity = 0.0;

    // start the general clock of the device
    m_clockGeneral.reset();
    m_clockGeneral.start();

    // reset history tables for velocity estimation
    double time = m_clockGeneral.getCurrentTimeSeconds();

    m_indexHistoryPos       = 0;
    m_indexHistoryPosWin    = C_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<C_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyPos[i].m_pos.zero();
        m_historyPos[i].m_time = time;
    }

    m_indexHistoryRot       = 0;
    m_indexHistoryRotWin    = C_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<C_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyRot[i].m_rot.identity();
        m_historyRot[i].m_time = time;
    }

    m_indexHistoryGripper    = 0;
    m_indexHistoryGripperWin = C_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<C_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyGripper[i].m_value = 0.0;
        m_historyGripper[i].m_time = time;
    }

    // set window time interval for measuring linear velocity
    m_linearVelocityWindowSize  = 0.015; // [s]

    // set window time interval for measuring angular velocity
    m_angularVelocityWindowSize  = 0.030; // [s]

    // set window time interval for measuring gripper linear velocity
    m_gripperVelocityWindowSize  = 0.015; // [s]

    // set gripper settings to emulate user switch
    m_gripperUserSwitchEnabled			= false;
    m_gripperUserSwitchAngleStart		= cDegToRad(10);
    m_gripperUserSwitchAngleClick		= cDegToRad(5);
    m_gripperUserSwitchForceClick		= 3;
    m_gripperUserSwitchForceEngaged		= 2;

    // set settings to emulate gripper with user switch
    m_virtualGripperAngleMin			= cDegToRad(5.0);
    m_virtualGripperAngleMax			= cDegToRad(25.0);
    m_virtualGripperAngularVelocity		= cDegToRad(80.0);
    m_virtualGripperAngle				= m_virtualGripperAngleMin;
    m_virtualGripperClock.reset();
}


//==============================================================================
/*!
    This method returns the status of the user switch [1 = __ON__ / 0 = __OFF__].

    \param  a_switchIndex  Index number of the switch.
    \param  a_status       Result value from reading the selected input switch.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericHapticDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // get status of all user switches
    unsigned int userSwitches;
    bool result = getUserSwitches(userSwitches);

    // check for errors
    if (result == false)
    {
        a_status = false;
        return (C_ERROR);
    }

    // check particular bit
    a_status = cCheckBit(userSwitches, a_switchIndex);

    // success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the position and orientation of the device through a 
    transformation matrix.

    \param  a_transform  Returned transformation matrix.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericHapticDevice::getTransform(cTransform& a_transform)
{
    cVector3d pos(0,0,0);
    cMatrix3d rot;
    rot.identity();
    
    bool result0 = getPosition(pos);
    bool result1 = getRotation(rot);
    a_transform.set(pos, rot);

    bool result = result0 | result1;
    return (result);
}


//==============================================================================
/*!
    This method estimates the linear velocity by passing the latest position.

    \param  a_newPosition  New position of the haptic device.
*/
//==============================================================================
void cGenericHapticDevice::estimateLinearVelocity(cVector3d& a_newPosition)
{
    // get current time
    double time = m_clockGeneral.getCurrentTimeSeconds();

    // check the time interval between the current and previous sample
    if ((time - m_historyPos[m_indexHistoryPos].m_time) < C_DEVICE_MIN_ACQUISITION_TIME)
    {
        return;
    }

    // store new value
    m_indexHistoryPos = (m_indexHistoryPos + 1) % C_DEVICE_HISTORY_SIZE;
    m_historyPos[m_indexHistoryPos].m_time = time;
    m_historyPos[m_indexHistoryPos].m_pos  = a_newPosition;

    // search table to find a sample that occurred before current time
    // minus time window interval
    int i=0;
    bool completed = false;
    while ((i < C_DEVICE_HISTORY_SIZE) && (!completed))
    {
        double interval = time - m_historyPos[m_indexHistoryPosWin].m_time;
        if ((interval < m_linearVelocityWindowSize) || (i == (C_DEVICE_HISTORY_SIZE-1)))
        {
            // compute result
            cVector3d result;
            m_historyPos[m_indexHistoryPos].m_pos.subr(m_historyPos[m_indexHistoryPosWin].m_pos, result);
            if (interval > 0)
            {
                result.divr(interval, m_linearVelocity);
                completed = true;
            }
            else
            {
                completed = true;
            }
        }
        else
        {
            m_indexHistoryPosWin = (m_indexHistoryPosWin + 1) % C_DEVICE_HISTORY_SIZE;
        }
    }
}


//==============================================================================
/*!
    This method estimates the angular velocity by passing the latest orientation frame.

    \param  a_newRotation  New rotation frame of the haptic device.
*/
//==============================================================================
void cGenericHapticDevice::estimateAngularVelocity(cMatrix3d& a_newRotation)
{
    // get current time
    double time = m_clockGeneral.getCurrentTimeSeconds();

    // check the time interval between the current and previous sample
    if ((time - m_historyRot[m_indexHistoryRot].m_time) < C_DEVICE_MIN_ACQUISITION_TIME)
    {
        return;
    }

    // store new value
    m_indexHistoryRot = (m_indexHistoryRot + 1) % C_DEVICE_HISTORY_SIZE;
    m_historyRot[m_indexHistoryRot].m_time  = time;
    m_historyRot[m_indexHistoryRot].m_rot = a_newRotation;

    // search table to find a sample that occurred before current time
    // minus time window interval
    int i=0;
    bool completed = false;
    while ((i < C_DEVICE_HISTORY_SIZE) && (!completed))
    {
        double interval = time - m_historyRot[m_indexHistoryRotWin].m_time;
        if ((interval < m_angularVelocityWindowSize) || (i == (C_DEVICE_HISTORY_SIZE-1)))
        {
            // compute result
            if (interval > 0)
            {
                cMatrix3d mat = cMul(cTranspose(m_historyRot[m_indexHistoryRotWin].m_rot), m_historyRot[m_indexHistoryRot].m_rot); 
                cVector3d axis;
                double angle = 0;
                mat.toAxisAngle(axis, angle);
                angle = angle / interval; 
                m_angularVelocity = cMul(a_newRotation, cMul(angle, axis));
                completed = true;
            }
            else
            {
                completed = true;
            }
        }
        else
        {
            m_indexHistoryRotWin = (m_indexHistoryRotWin + 1) % C_DEVICE_HISTORY_SIZE;
        }
    }
}


//==============================================================================
/*!
    This method estimates the velocity of the gripper by passing the latest 
    gripper position.

    \param  a_newGripperAngle  New angular position of the gripper.
*/
//==============================================================================
void cGenericHapticDevice::estimateGripperVelocity(double a_newGripperAngle)
{
    // get current time
    double time = m_clockGeneral.getCurrentTimeSeconds();

    // check the time interval between the current and previous sample
    if ((time - m_historyGripper[m_indexHistoryGripper].m_time) < C_DEVICE_MIN_ACQUISITION_TIME)
    {
        return;
    }

    // store new value
    m_indexHistoryGripper = (m_indexHistoryGripper + 1) % C_DEVICE_HISTORY_SIZE;
    m_historyGripper[m_indexHistoryGripper].m_time  = time;
    m_historyGripper[m_indexHistoryGripper].m_value = a_newGripperAngle;

    // search table to find a sample that occurred before current time
    // minus time window interval
    int i=0;
    bool completed = false;
    while ((i < C_DEVICE_HISTORY_SIZE) && (!completed))
    {
        double interval = time - m_historyGripper[m_indexHistoryGripperWin].m_time;
        if ((interval < m_gripperVelocityWindowSize) || (i == (C_DEVICE_HISTORY_SIZE-1)))
        {
            // compute result
            if (interval > 0)
            {
                m_gripperAngularVelocity = (m_historyGripper[m_indexHistoryGripper].m_value - 
                                            m_historyGripper[m_indexHistoryGripperWin].m_value) / interval;
                completed = true;
            }
            else
            {
                completed = true;
            }
        }
        else
        {
            m_indexHistoryGripperWin = (m_indexHistoryGripperWin + 1) % C_DEVICE_HISTORY_SIZE;
        }
    }
}


//==========================================================================
/*!
    This method compute a haptic force that simulates a user switch (button)
    given the angular position and velocity of the gripper.

    \param    a_gripperAngle            Current position angle of gripper.
    \param    a_gripperAngularVelocity  Current angular velocity of gripper.

    \return   Computed force to be applied to the gripper.
*/
//==============================================================================
double cGenericHapticDevice::computeGripperUserSwitchForce(const double& a_gripperAngle, 
                                                           const double& a_gripperAngularVelocity)
{
    if (m_gripperUserSwitchEnabled)
    {
        // compute damping term
        double damping = 0.0;
        double gripperAngularVelocity = 0.0;
        getGripperAngularVelocity(gripperAngularVelocity);
        damping = -0.1 * m_specifications.m_maxGripperAngularDamping * gripperAngularVelocity;

        // PHASE 0: outside of switch, zero force
        if (a_gripperAngle > m_gripperUserSwitchAngleStart)
        { 
            double force = 0.0;
            return (force);
        }

        // PHASE 1: switch is being engaged. (Force is rising until "click")
        else if ((a_gripperAngle <= m_gripperUserSwitchAngleStart) &&
                 (a_gripperAngle > m_gripperUserSwitchAngleClick))
        {
            double force = (m_gripperUserSwitchAngleStart - a_gripperAngle) * ((m_gripperUserSwitchForceClick) / (m_gripperUserSwitchAngleStart - m_gripperUserSwitchAngleClick));
            return (force + damping);
        }

        // PHASE 2: switch has been engaged. (Force is constant)
        else if (a_gripperAngle <= m_gripperUserSwitchAngleClick)
        {
            double force = m_gripperUserSwitchForceEngaged;
            return (force + damping);
        }
    }
    
    return (0.0);
}


//==========================================================================
/*!
    This method returns __true__ if the gripper is closed, or __false__ if
    it is open. This mode requires that the gripper has been configured 
    to simulate a user switch by calling method setEnableGripperUserSwitch(),
    furthermore the gripper needs to provide force-feedback capabilities.

    \return __true__ if the gripper user switch is closed, __false__ otherwise.
*/
//==============================================================================
bool cGenericHapticDevice::getGripperUserSwitch()
{
    if (m_gripperUserSwitchEnabled && m_specifications.m_sensedGripper && m_specifications.m_actuatedGripper)
    {
        double gripperAngle;
        if (getGripperAngleDeg(gripperAngle))
        {
            if (gripperAngle < m_gripperUserSwitchAngleClick)
            {
                return (true);
            }
            else
            {
                return (false);
            }
        }
        else
        {
            return (false);
        }
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param  a_angle  Return value.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericHapticDevice::getGripperAngleRad(double& a_angle)
{
    // get user switch
    bool userSwitch = false;
    getUserSwitch(0, userSwitch);

    // read clock time
    double timeElapsed = m_virtualGripperClock.stop();
    m_virtualGripperClock.start(true);

    // update position
    double nextAngle;
    if (userSwitch)
    {
        // simulating the closing of the virtual gripper
        nextAngle = m_virtualGripperAngle - m_virtualGripperAngularVelocity * timeElapsed;
        m_gripperAngularVelocity = -m_virtualGripperAngularVelocity;
    }
    else
    {
        // simulating the opening of the virtual gripper
        nextAngle = m_virtualGripperAngle + m_virtualGripperAngularVelocity * timeElapsed;
        m_gripperAngularVelocity = m_virtualGripperAngularVelocity;
    }
    
    // clamp and update new virtual gripper position
    m_virtualGripperAngle = cClamp(nextAngle, m_virtualGripperAngleMin, m_virtualGripperAngleMax);

    // return value
    a_angle = m_virtualGripperAngle;

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
