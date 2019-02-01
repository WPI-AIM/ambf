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
    \version   3.2.0 $Rev: 2174 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "tools/CGenericTool.h"
#include "world/CMesh.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericTool.

    \param  a_parentWorld  Parent world in which the tool belongs.
*/
//==============================================================================
cGenericTool::cGenericTool(cWorld* a_parentWorld)
{
    // set parent world
    m_parentWorld = a_parentWorld;

    // no device is currently connected to this tool
    m_hapticDevice = cGenericHapticDevicePtr();

    // tool is not yet enabled
    m_enabled = false;

    // initialize variable that control the force rise mode and the waiting for small forces mode.
    m_forceOn               = true;
    m_forceEngaged          = false;
    m_useWaitForSmallForce  = false;
    m_smallForceCounter     = 0;
    m_smallForceThresh      = 0.2;
    m_useForceRise          = false;
    m_forceRiseTime         = 2.0;
    m_forceRiseClock.reset();
    m_flagForceRiseFirstTime = true;
    m_flagForceRiseActivated = true;
    m_freqRead.setTimePeriod(0.1);
    m_freqRead.reset();
    m_freqWrite.setTimePeriod(0.1);
    m_freqWrite.reset();

    // initialize all members
    m_userSwitches          = 0;
    m_deviceGlobalForce     = cVector3d(0,0,0);
    m_deviceLocalTorque     = cVector3d(0,0,0);
    m_deviceGlobalTorque    = cVector3d(0,0,0);
    m_deviceLocalTorque     = cVector3d(0,0,0);
    m_gripperForce          = 0.0;
    m_workspaceRadius       = 1.0;
    m_workspaceScaleFactor  = 1.0;
    m_deviceLocalPos        = cVector3d(0,0,0);
    m_deviceGlobalPos       = cVector3d(0,0,0);
    m_deviceLocalRot        = cIdentity3d();
    m_deviceGlobalRot       = cIdentity3d();
    m_deviceLocalLinVel     = cVector3d(0,0,0);
    m_deviceGlobalLinVel    = cVector3d(0,0,0);
    m_deviceLocalAngVel     = cVector3d(0,0,0);
    m_deviceGlobalAngVel    = cVector3d(0,0,0);
    m_gripperAngle          = 0.0;
    m_gripperAngVel         = 0.0;

    // clear haptic points
    m_hapticPoints.clear();

    // create a mesh for tool display purposes
    m_image = new cMesh();
}


//==============================================================================
/*!
    Destructor of cGenericTool.
*/
//==============================================================================
cGenericTool::~cGenericTool()
{
    // delete the display mesh
    delete m_image;
}


//==============================================================================
/*!
    This method sets the radius size of all haptic points. The radius 
    affects the physical radius of the proxy and spheres used to render the 
    goal and proxy positions.

    \param  a_radius  New radius for graphic display and haptic computation.
*/
//==============================================================================
void cGenericTool::setRadius(double a_radius)
{
    for (unsigned int i=0; i<m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->setRadius(a_radius);
    }
}


//==============================================================================
/*!
   This method assign new values to the radius of the displayed spheres 
   (goal and proxy) and the radius of the contact sphere (proxy) that is used
   to compute the contact forces. 
   
   For creating more realistic conditions, settings both parameters to the 
   same value is recommended. However, setting \p a_radiusContact to zero will
   generally accelerate the force rendering algorithm as the contacts will 
   simply be computed between the center of the sphere and the polygons.

    \param  a_radiusDisplay  New radius for display of spheres (proxy and goal).
    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//==============================================================================
void cGenericTool::setRadius(double a_radiusDisplay, double a_radiusContact)
{
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->setRadius(a_radiusDisplay, a_radiusContact);
    }
}


//==============================================================================
/*!
    This method sets the radius of the physical proxy. The change affects all
    haptic points that compose the tool.

    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//==============================================================================
void cGenericTool::setRadiusContact(double a_radiusContact)
{
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->setRadiusContact(a_radiusContact);
    }
}


//==============================================================================
/*!
    This method set the radius of the sphere used to display the proxy and 
    goal position.

    \param  a_radiusDisplay  New radius of display spheres (proxy and goal).
*/
//==============================================================================
void cGenericTool::setRadiusDisplay(double a_radiusDisplay)
{
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->setRadiusDisplay(a_radiusDisplay);
    }
}


//==============================================================================
/*!
    This method sets the display options of the goal and proxy spheres.
    If both spheres are enabled, a small line is drawn between both spheres.

    \param  a_showProxy  If __true__, then the proxy sphere is displayed.
    \param  a_showGoal   If __true__, then the goal sphere is displayed.
    \param  a_colorLine  Color of line connecting proxy to goal spheres.
*/
//==============================================================================
void cGenericTool::setShowContactPoints(bool a_showProxy, 
                                        bool a_showGoal, 
                                        cColorf a_colorLine)
{
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->setShow(a_showProxy, a_showGoal, a_colorLine);
    }
}


//==============================================================================
/*!
    This method enables or disables the dynamic proxy algorithm to support 
    dynamic objects. \n\n

    This option must be enabled if you have cMesh objects which move inside
    the world and collide with your tool. However, if your world contains
    only static cMesh objects or shapes that rely on haptic effects, then this
    option can be disabled. Not enabling the dynamic proxy model for moving 
    objects will create a "pop through" situation where the tool traverses 
    the mesh without detecting any collision.

    \param  a_enabled  If __true__ then support for mesh objects in motion is 
                       enabled.
*/
//==============================================================================
void cGenericTool::enableDynamicObjects(bool a_enabled)
{
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->m_algorithmFingerProxy->m_useDynamicProxy = a_enabled;
    }
}


//==============================================================================
/*!
    This method overrides the value of a specified user switch of the device.

    \param  a_switchIndex  Switch index number.
    \param  a_value        Switch value.
*/
//==============================================================================
void cGenericTool::setUserSwitch(const unsigned int a_switchIndex, const bool a_value)
{
    m_userSwitches = cSetBit(m_userSwitches, a_switchIndex, a_value);
}


//==============================================================================
/*!
    This method overrides the value of all user switches of the device.

    \param  a_userSwitches  User switch value.
*/
//==============================================================================
void cGenericTool::setUserSwitches(const unsigned int a_userSwitches)
{
    m_userSwitches = a_userSwitches;
}


//==============================================================================
/*!
    This method overrides the gripper angle in __radians__.

    \param  a_gripperAngleRad  Gripper angle in __radians__.
*/
//==============================================================================
void cGenericTool::setGripperAngleRad(const double& a_gripperAngleRad)
{
    m_gripperAngle = a_gripperAngleRad;
}


//==============================================================================
/*!
    This method overrides the gripper angle in __degrees__.

    \param  a_gripperAngleDeg  Gripper angle in __degrees__.
*/
//==============================================================================
void cGenericTool::setGripperAngleDeg(const double& a_gripperAngleDeg)
{
    m_gripperAngle = cDegToRad(a_gripperAngleDeg);
}


//==============================================================================
/*!
    This method overrides the angular velocity of the gripper in radians per 
    second.

    \param  a_gripperAngVel  Gripper angular velocity in radians per second.
*/
//==============================================================================
void cGenericTool::setGripperAngVel(const double& a_gripperAngVel)
{
    m_gripperAngVel = a_gripperAngVel;
}


//==============================================================================
/*!
    This method overrides the position of the haptic device in local coordinates.

    \param  a_localPos  Position of device.
*/
//==============================================================================
void cGenericTool::setDeviceLocalPos(const cVector3d& a_localPos)
{
    m_deviceLocalPos = a_localPos;
    m_deviceGlobalPos = m_globalPos + m_globalRot * m_deviceLocalPos;

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the position of the haptic device in global coordinates.

    \param  a_globalPos  Position of device.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalPos(const cVector3d& a_globalPos)
{
    m_deviceGlobalPos = a_globalPos;
    m_deviceLocalPos = cTranspose(m_globalRot) * (m_deviceGlobalPos - m_globalPos);

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the orientation of the haptic device in local 
    coordinates.

    \param  a_localRot  Orientation of device.
*/
//==============================================================================
void cGenericTool::setDeviceLocalRot(const cMatrix3d& a_localRot)
{
    m_deviceLocalRot = a_localRot;
    m_deviceGlobalRot = m_globalRot * m_deviceLocalRot;

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the orientation of the haptic device in global
    coordinates.

    \param  a_globalRot  Orientation of device.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalRot(const cMatrix3d& a_globalRot)
{
    m_deviceGlobalRot = a_globalRot;
    m_deviceLocalRot = cTranspose(m_globalRot) * m_deviceGlobalRot;

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the linear velocity of the device in local coordinates.

    \param  a_localLinVel  Linear velocity of device.
*/
//==============================================================================
void cGenericTool::setDeviceLocalLinVel(const cVector3d& a_localLinVel)
{
    m_deviceLocalLinVel = a_localLinVel;
    m_deviceGlobalLinVel = m_globalRot * m_deviceLocalLinVel;
}


//==============================================================================
/*!
    This method overrides the linear velocity of the device in global coordinates.

    \param  a_globalLinVel  Linear velocity of device.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalLinVel(const cVector3d& a_globalLinVel)
{
    m_deviceGlobalLinVel = a_globalLinVel;
    m_deviceLocalLinVel = cTranspose(m_globalRot) * m_deviceGlobalLinVel;
}


//==============================================================================
/*!
    This method overrides the angular velocity of the device in local coordinates.

    \param  a_localAngVel  Angular velocity of device.
*/
//==============================================================================
void cGenericTool::setDeviceLocalAngVel(const cVector3d& a_localAngVel)
{
    m_deviceLocalAngVel = a_localAngVel;
    m_deviceGlobalAngVel = m_globalRot * m_deviceLocalAngVel;
}


//==============================================================================
/*!
    This method overrides the angular velocity of the device in global coordinates.

    \param  a_globalAngVel  Angular velocity of device.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalAngVel(const cVector3d& a_globalAngVel)
{
    m_deviceGlobalAngVel = a_globalAngVel;
    m_deviceLocalAngVel = cTranspose(m_globalRot) * m_deviceGlobalAngVel;
}


//==============================================================================
/*!
    This method overrides the position and orientation of device in local
    coordinates.

    \param  a_localTransform  Position and orientation of device.
*/
//==============================================================================
void cGenericTool::setDeviceLocalTransform(const cTransform& a_localTransform)
{
    m_deviceLocalPos = a_localTransform.getLocalPos();
    m_deviceLocalRot = a_localTransform.getLocalRot();

    m_deviceGlobalPos = m_globalPos + m_globalRot * m_deviceLocalPos;
    m_deviceGlobalRot = m_globalRot * m_deviceLocalRot;

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the position and orientation of device in global
    coordinates.

    \param  a_globalTransform  Position and orientation of device.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalTransform(const cTransform& a_globalTransform)
{
    m_deviceGlobalPos = a_globalTransform.getLocalPos();
    m_deviceGlobalRot = a_globalTransform.getLocalRot();

    m_deviceLocalPos = cTranspose(m_globalRot) * (m_deviceGlobalPos - m_globalPos);
    m_deviceLocalRot = cTranspose(m_globalRot) * m_deviceGlobalRot;

    updateToolImagePosition();
}


//==============================================================================
/*!
    This method overrides the gripper force.

    \param  a_gripperForce  Gripper force in Newtons.
*/
//==============================================================================
void cGenericTool::setGripperForce(const double& a_gripperForce)
{
    m_gripperForce = a_gripperForce;
}


//==============================================================================
/*!
    This method adds a force to the current gripper force.

    \param  a_gripperForce  Gripper force in Newtons.
*/
//==============================================================================
void cGenericTool::addGripperForce(const double& a_gripperForce)
{
    m_gripperForce += a_gripperForce;
}


//==============================================================================
/*!
    This method overrides the force in local tool or device coordinates.

    \param  a_localForce  Force in local coordinates.
*/
//==============================================================================
void cGenericTool::setDeviceLocalForce(const cVector3d& a_localForce)
{
    m_deviceLocalForce = a_localForce;
    m_deviceGlobalForce = m_globalRot * m_deviceLocalForce;
}


//==============================================================================
/*!
    This method adds a force to the current force. The force is expressed in 
    local device or tool coordinates.

    \param  a_localForce  Force in local coordinates.
*/
//==============================================================================
void cGenericTool::addDeviceLocalForce(const cVector3d& a_localForce)
{
    m_deviceLocalForce.add(a_localForce);
    m_deviceGlobalForce = m_globalRot * m_deviceLocalForce;
}


//==============================================================================
/*!
    This method overrides a force in world coordinates.

    \param  a_globalForce  Force expressed in world coordinates.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalForce(const cVector3d& a_globalForce)
{
    m_deviceGlobalForce = a_globalForce;
    m_deviceLocalForce = cTranspose(m_globalRot) * m_deviceGlobalForce;
}


//==============================================================================
/*!
    This method adds a new force to the current force. The force is expressed
    in world coordinates.

    \param  a_globalForce  Force expressed in world coordinates.
*/
//==============================================================================
void cGenericTool::addDeviceGlobalForce(const cVector3d& a_globalForce)
{
    m_deviceGlobalForce.add(a_globalForce);
    m_deviceLocalForce = cTranspose(m_globalRot) * m_deviceGlobalForce;
}


//==============================================================================
/*!
    This method overrides a torque in local tool or device coordinates.

    \param  a_localTorque  Torque in local coordinates.
*/
//==============================================================================
void cGenericTool::setDeviceLocalTorque(const cVector3d& a_localTorque)
{
    m_deviceLocalTorque = a_localTorque;
    m_deviceGlobalTorque = m_globalRot * m_deviceLocalTorque;
}


//==============================================================================
/*!
    This method adds a torque to current torque. The torque is expressed in 
    local device or tool coordinates.

    \param  a_localTorque  Torque in local coordinates.
*/
//==============================================================================
void cGenericTool::addDeviceLocalTorque(const cVector3d& a_localTorque)
{
    m_deviceLocalTorque.add(a_localTorque);
    m_deviceGlobalTorque = m_globalRot * m_deviceLocalTorque;
}


//==============================================================================
/*!
    This method overrides a torque in world coordinates.

    \param  a_globalTorque  Torque expressed in world coordinates.
*/
//==============================================================================
void cGenericTool::setDeviceGlobalTorque(const cVector3d& a_globalTorque)
{
    m_deviceGlobalTorque = a_globalTorque;
    m_deviceLocalTorque = cTranspose(m_globalRot) * m_deviceGlobalTorque;
}


//==============================================================================
/*!
    This method adds a torque to the current torque. The torque is expressed 
    in world coordinates.

    \param  a_globalTorque  Torque expressed in world coordinates.
*/
//==============================================================================
void cGenericTool::addDeviceGlobalTorque(const cVector3d& a_globalTorque)
{
    m_deviceGlobalTorque.add(a_globalTorque);
    m_deviceLocalTorque = cTranspose(m_globalRot) * m_deviceGlobalTorque;
}


//==============================================================================
/*!
    This method starts the haptic tool. A connection is opened to the haptic 
    device and the position of the tool initialized by reading the current 
    position of the haptic device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::start()
{
    // check if device is available
    if (m_hapticDevice == nullptr)
    {
        return (C_ERROR);
    }

    // open connection to device
    if (m_hapticDevice->open())
    {
        m_enabled = true;
    }
    else
    {
        m_enabled = false;
        return (C_ERROR);
    }

    // reset startup variables
    m_smallForceCounter     = 0;
    m_forceRiseClock.reset();
    m_flagForceRiseFirstTime = true;
    m_flagForceRiseActivated = true;
    m_freqRead.reset();
    m_freqWrite.reset();

    // update position
    updateFromDevice();

    // initialize tool by resetting the force models
    initialize();

    // return result
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method stops the haptic tool. The connection to the haptic device is 
    closed

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::stop()
{
    // check if tool is currently enabled
    if (!m_enabled) { return (false); }

    // stop the device
    return (m_hapticDevice->close());
}


//==============================================================================
/*!
    This method resets all force models according to the current position of the 
    haptic device.
*/
//==============================================================================
void cGenericTool::initialize()
{
    // update position from haptic device
    updateFromDevice();

    // initialize all haptic points
    for (unsigned int i=0; i< m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->initialize(m_deviceGlobalPos);
    }
}


//==============================================================================
/*!
    This method updates the position and orientation of the tool by reading
    values from the haptic device.
*/
//==============================================================================
void cGenericTool::updateFromDevice()
{
    // check if device is available
    if ((m_hapticDevice == nullptr) || (!m_enabled)) 
    {
        cSleepMs(1);
        return; 
    }


    //////////////////////////////////////////////////////////////////////
    // retrieve data from haptic device
    //////////////////////////////////////////////////////////////////////

    // temp variables
    cVector3d devicePos, deviceLinVel, deviceAngVel;
    cMatrix3d deviceRot;
    double gripperAngle;
    double gripperAngVel;
    unsigned int userSwitches;

    // init temp variable
    devicePos.zero();
    deviceLinVel.zero();
    deviceAngVel.zero();
    deviceRot.identity();
    gripperAngle = 0.0;
    gripperAngVel = 0.0;
    userSwitches = 0;

    // update position, orientation, linear and angular velocities from device
    m_hapticDevice->getPosition(devicePos);
    m_hapticDevice->getRotation(deviceRot);
    m_hapticDevice->getGripperAngleRad(gripperAngle);
    m_hapticDevice->getGripperAngularVelocity(gripperAngVel);
    m_hapticDevice->getLinearVelocity(deviceLinVel);
    m_hapticDevice->getAngularVelocity(deviceAngVel);
    m_hapticDevice->getUserSwitches(userSwitches);


    //////////////////////////////////////////////////////////////////////
    // update information inside tool
    //////////////////////////////////////////////////////////////////////

    // compute local position - adjust for tool workspace scale factor
    m_deviceLocalPos = m_workspaceScaleFactor * devicePos;

    // compute global position in world coordinates
    m_deviceGlobalPos = m_globalPos + m_globalRot * m_deviceLocalPos;

    // compute local rotation
    m_deviceLocalRot = deviceRot; 

    // compute global rotation
    m_deviceGlobalRot = m_globalRot * m_deviceLocalRot;

    // compute local linear velocity - adjust for tool workspace scale factor
    m_deviceLocalLinVel = m_workspaceScaleFactor * deviceLinVel;

    // compute global linear velocity
    m_deviceGlobalLinVel = m_globalRot * m_deviceLocalLinVel;

    // compute local rotational velocity
    m_deviceLocalAngVel = deviceAngVel;

    // compute global rotational velocity
    m_deviceGlobalAngVel = m_globalRot * m_deviceLocalAngVel;

    // store gripper angle
    m_gripperAngle = gripperAngle;

    // store gripper angular velocity
    m_gripperAngVel = gripperAngVel;

    // store user switch status
    m_userSwitches = userSwitches;

    // update the position and orientation of the tool image
    updateToolImagePosition();

    // update frequency counter
    m_freqRead.signal(1);
}


//==============================================================================
/*!
    This method computes the global position of the tool in the world.

    \param  a_frameOnly  If __false__, then global the position of any vertices
                         is computed.
*/
//==============================================================================
void cGenericTool::updateGlobalPositions(const bool a_frameOnly)
{
    if (m_image != NULL)
    {
        m_image->computeGlobalPositions(a_frameOnly, 
                                        m_globalPos, 
                                        m_globalRot);
    }
}


//==============================================================================
/*!
    This method updates the position and orientation of the tool image
    according to the latest data queried from the haptic device.
*/
//==============================================================================
void cGenericTool::updateToolImagePosition()
{
    // set the position and orientation of the tool image to be equal to the 
    // one of the haptic device. Under more complex tools, these values could 
    // typically be function of the combined positions of various contact 
    // points (or proxy positions).
    m_image->setLocalPos(m_deviceLocalPos);
    m_image->setLocalRot(m_deviceLocalRot);
}


//==============================================================================
/*!
    This method computes the interaction forces between the tool and the
    objects inside the virtual world.
*/
//==============================================================================
void cGenericTool::computeInteractionForces()
{
    // for each haptic point compute the interaction force
    // and combine their overall contribution to compute the output force
    // and torque to be sent to the haptic device

    // initialize variables
    cVector3d force, torque;
    force.zero();
    torque.zero();

    int numContactPoint = (int)(m_hapticPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next haptic point
        cHapticPoint* nextContactPoint = m_hapticPoints[i];

        // compute force at haptic point as well as new proxy position
        cVector3d t_force = nextContactPoint->computeInteractionForces(m_deviceGlobalPos, 
                                                                       m_deviceGlobalRot, 
                                                                       m_deviceGlobalLinVel, 
                                                                       m_deviceGlobalAngVel);

        cVector3d t_pos = nextContactPoint->getGlobalPosProxy();

        // combine force contributions together
        force.add(t_force);
        torque.add(cCross(t_pos, t_force));
    }

    // update global forces
    setDeviceGlobalForce(force);
    setDeviceGlobalTorque(torque);
    setGripperForce(0.0);
}


//==============================================================================
/*!
    This method applies the latest computed force to the haptic device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::applyToDevice()
{
    // check if device is available
    if ((m_hapticDevice == nullptr) || (!m_enabled)) { return (C_ERROR); }

    // retrieve force values to be applied to device
    cVector3d deviceLocalForce = m_deviceLocalForce;
    cVector3d deviceLocalTorque = m_deviceLocalTorque;
    double gripperForce = m_gripperForce;


    ////////////////////////////////////////////////////////////////////////////
    // STARTUP MODE: WAIT FOR SMALL FORCE
    ////////////////////////////////////////////////////////////////////////////

    // if forces are already fully engaged, skip this part
    if (!m_forceEngaged)
    {
        // check if waiting for small forces mode is engaged. engage forces otherwise.
        if (m_useWaitForSmallForce)
        {
            // check if desired force is smaller that threshold.
            if (m_deviceLocalForce.length() < m_smallForceThresh)
            {
                // engage forces only if small forces have been maintained for three cycles.
                if (m_smallForceCounter > 3)
                {
                    m_forceEngaged = true;
                    m_smallForceCounter = 0;
                }
                else
                {
                    m_smallForceCounter++;
                }
            }
            else
            {
                m_smallForceCounter = 0;
            }
        }
        else
        {
            m_forceEngaged = true;
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    // STARTUP MODE: FORCE RISE
    ////////////////////////////////////////////////////////////////////////////

    // check if forces are supposed to rise.
    if (m_forceEngaged && m_flagForceRiseActivated)
    {
        if (m_flagForceRiseFirstTime)
        {
            if (m_useForceRise)
            {
                m_forceRiseClock.reset();
                m_forceRiseClock.start();
                m_flagForceRiseFirstTime = false;
            }
            else
            {
                m_flagForceRiseActivated = false;
            }
        }

        // scale forces
        double scale = 1.0;
        double time = cClamp(m_forceRiseClock.getCurrentTimeSeconds(), 0.0, m_forceRiseTime);
        if (m_forceRiseTime > C_SMALL)
        {
            scale = time / m_forceRiseTime;
            deviceLocalForce.mul(scale);
            deviceLocalTorque.mul(scale);
            gripperForce *= scale;
        }

        // apply 30% maximum damping when forces rise, if and only if, update rate is running above 400 Hz.
        const double MIN_FREQUENCY = 400.0;
        if ((m_freqRead.getFrequency() > MIN_FREQUENCY) && (m_freqWrite.getFrequency() > MIN_FREQUENCY))
        {
            double deviceLinDamping = (1.0 - scale) * 0.2 * m_hapticDevice->getSpecifications().m_maxLinearDamping;
            double deviceAngDamping = (1.0 - scale) * 0.2 * m_hapticDevice->getSpecifications().m_maxAngularDamping;
            double gripperAngDamping = (1.0 - scale) * 0.2 * m_hapticDevice->getSpecifications().m_maxGripperAngularDamping;

            deviceLocalForce = deviceLocalForce - deviceLinDamping * m_deviceLocalLinVel;
            deviceLocalTorque = deviceLocalTorque - deviceAngDamping * m_deviceLocalAngVel;
            gripperForce = gripperForce - gripperAngDamping * m_gripperAngVel;
        }

        // disable force rizing when completed
        if (m_forceRiseClock.getCurrentTimeSeconds() >= m_forceRiseTime)
        {
            m_flagForceRiseActivated = false;
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    // APPLY FORCES
    ////////////////////////////////////////////////////////////////////////////

    // send force commands to haptic device
    if ((m_forceOn) && (m_forceEngaged))
    {
        m_hapticDevice->setForceAndTorqueAndGripperForce(deviceLocalForce, 
                                                         deviceLocalTorque, 
                                                         gripperForce);
    }
    else
    {
        cVector3d nullv3d (0.0, 0.0, 0.0);
        m_hapticDevice->setForceAndTorqueAndGripperForce(nullv3d,
                                                         nullv3d,
                                                         0.0);
    }

    // update frequency counter
    m_freqWrite.signal(1);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method enables forces to be displayed on the haptic device.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::setForcesON()
{
    // check if device is available
    if ((m_hapticDevice == nullptr) || (!m_enabled)) { return (C_ERROR); }

    // flag forces as ON
    if (!m_forceOn)
    {
        m_forceEngaged = false;
        m_forceOn = true;
        m_smallForceCounter = 0;
        m_forceRiseClock.reset();
        m_flagForceRiseFirstTime = true;
        m_flagForceRiseActivated = true;
        m_freqRead.reset();
        m_freqWrite.reset();
    }

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method disables forces to be displayed on the haptic device.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::setForcesOFF()
{
    // check if device is available
    if ((m_hapticDevice == nullptr) || (!m_enabled)) { return (C_ERROR); }

    // flag force as OFF
    m_forceOn = false;

    // set all force variables to zero
    m_deviceLocalForce.zero();
    m_deviceGlobalForce.zero();
    m_deviceLocalTorque.zero();
    m_deviceGlobalTorque.zero();
    m_gripperForce = 0.0;

    // apply updated forces to haptic device
    applyToDevice();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method checks if the tool is currently interacting with a given object
    passed as argument.

    \return __true__ if the tool is interacting with the object, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::isInContact(cGenericObject* a_object)
{
    for (unsigned int i=0; i<m_hapticPoints.size(); i++)
    {
        if (m_hapticPoints[i]->isInContact(a_object))
        {
            return (true);
        }
    }

    return (false);
}


//==============================================================================
/*!
    This method defines the size of the virtual workspace covered by the haptic 
    device.

    \param  a_workspaceRadius  Radius of the workspace.
*/
//==============================================================================
bool cGenericTool::setWorkspaceRadius(const double& a_workspaceRadius)
{
    // check if device is available
    if (m_hapticDevice == nullptr) { return (false); }

    // update new workspace size
    m_workspaceRadius = a_workspaceRadius;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    if (m_hapticDevice != nullptr)
    {
        m_workspaceScaleFactor = m_workspaceRadius / m_hapticDevice->getSpecifications().m_workspaceRadius;
    }
    else
    {
        m_workspaceScaleFactor = 1.0;
    }

    // since the workspace has changed, the position of the tool will change
    // too therefore it is necessary to re-initialize the contact models
    // in order to avoid a possible force spike coming from the proxy models.
    initialize();

    // return success
    return (true);
}


//==============================================================================
/*!
    This method defines a scale factor between the physical workspace of the 
    haptic device and the workspace covered by the virtual tool.

    \param  a_workspaceScaleFactor  Workspace scale factor.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cGenericTool::setWorkspaceScaleFactor(const double& a_workspaceScaleFactor)
{
    // check if device is available
    if (m_hapticDevice == nullptr) { return (C_ERROR); }

    // make sure that input value is bigger than zero
    double value = fabs(a_workspaceScaleFactor);
    if (value == 0) { return (C_ERROR); }

    // update scale factor
    m_workspaceScaleFactor = value;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    m_workspaceRadius =  m_workspaceScaleFactor * m_hapticDevice->getSpecifications().m_workspaceRadius;

    // since the workspace has changed, the position of the tool will change
    // too therefore it is necessary to re-initialize the contact models
    // in order to avoid a possible force spike coming from the proxy models.
    initialize();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method assigns a shader program to this tool and all the haptic
    points associated with the tool. \n

    If \p a_affectChildren is set to __true__ then all children are assigned
    with the shader program.

    \param  a_shaderProgram   Shader program to be assigned to object.
    \param  a_affectChildren  If __true__ then children are updated too.
*/
//==============================================================================
void cGenericTool::setShaderProgram(cShaderProgramPtr a_shaderProgram,
    const bool a_affectChildren)
{
    m_shaderProgram = a_shaderProgram;

    for (unsigned int i = 0; i<m_hapticPoints.size(); i++)
    {
        m_hapticPoints[i]->m_sphereGoal->setShaderProgram(a_shaderProgram, true);
        m_hapticPoints[i]->m_sphereProxy->setShaderProgram(a_shaderProgram, true);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShaderProgram(a_shaderProgram, true);
        }
    }
}


//==============================================================================
/*!
    This method creates an audio source for this tool.

    \param  a_audioDevice  Audio device.
*/
//==============================================================================
bool cGenericTool::createAudioSource(cAudioDevice* a_audioDevice)
{
    for (unsigned int i=0; i<m_hapticPoints.size(); i++)
    {
        if (m_hapticPoints[i]->createAudioSource(a_audioDevice) == false)
        {
            return (C_ERROR);
        }
    }

    return (C_SUCCESS);
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
