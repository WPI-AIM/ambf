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
    \version   3.2.0 $Rev: 2188 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericToolH
#define CGenericToolH
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "devices/CGenericHapticDevice.h"
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "timers/CFrequencyCounter.h"
#include "tools/CHapticPoint.h"
#include "world/CGenericObject.h"
#include "world/CWorld.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericTool.h

    \brief
    Implements a base class for modeling haptic tools.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericTool
    \ingroup    tools

    \brief
    This class implements a base class for modeling haptic tools

    \details
    cGenericTool implements a base class for modeling virtual haptic tools 
    inside a virtual environment (cWorld) that are connected to haptic 
    devices.
*/
//==============================================================================
class cGenericTool : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericTool.
    cGenericTool(cWorld* a_parentWorld);

    //! Destructor of cGenericTool.
    virtual ~cGenericTool();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC DEVICE
    //--------------------------------------------------------------------------

public:

    //! This method define a haptic device that will control this tool.
    void setHapticDevice(cGenericHapticDevicePtr a_hapticDevice) { if (a_hapticDevice != nullptr) { m_hapticDevice = a_hapticDevice; } }

    //! This method returns a pointer to the haptic device that is controlling this tool.
    cGenericHapticDevicePtr getHapticDevice() { return (m_hapticDevice); }

    //! This method starts communication with the haptic device and initializes the tool.
    virtual bool start();

    //! This method stops communication with the haptic device.
    virtual bool stop();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FORCE CONTROL
    //--------------------------------------------------------------------------

public:

    //! This method resets the contact force models and sets the tool at the position of the haptic device.
    virtual void initialize();

    //! This method updates the position, orientation, velocity and other degree of freedoms of the tool by reading sensor data from the haptic device.
    virtual void updateFromDevice();

    //! This method computes all interaction forces between the tool's haptic points and the virtual environment.
    virtual void computeInteractionForces();

    //! This method sends the latest computed interaction force, torque, and gripper force to the haptic device.
    virtual bool applyToDevice();

    //! This method enables forces to the haptic device.
    virtual bool setForcesON();

    //! This method disables forces to the haptic device.
    virtual bool setForcesOFF();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - STARTUP MODES
    //--------------------------------------------------------------------------

public:

    //! This method enables the tool to wait for the simulation to compute a small force before sending it to the haptic device. This feature typically avoids having the device "jump" at the initialization of an application.
    inline void setWaitForSmallForce(const bool a_value) { m_useWaitForSmallForce = a_value; }

    //! This method returns the status about waiting for small small forces to appear before sending them to the haptic device.
    inline bool getWaitForSmallForce() { return (m_useWaitForSmallForce); }

    //! This method sets the force threshold required to engage forces when \ref setWaitForSmallForce() is enabled.
    void setSmallForceThresh(const double a_smallForceThresh) { m_smallForceThresh = fabs(a_smallForceThresh); }

    //! This method returns the force threshold required to engage forces when \ref setWaitForSmallForce() is enabled.
    double getSmallForceThresh() { return (m_smallForceThresh); }

    //! This method enables or disables the tool to gradually rise forces at startup.
    void setUseForceRise(const bool a_value) { m_useForceRise = a_value; if (a_value) m_forceEngaged = true; }

    //! This method returns the status about force rising mode.
    bool getUseForceRise() { return (m_useForceRise); }

    //! This method sets the time required to rise the forces from 0% to 100%.
    void setRiseTime(const double a_riseTime) { m_forceRiseTime = fabs(a_riseTime); }

    //! This method returns the force rise time.
    double getRiseTime() { return (m_forceRiseTime); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - USER SWITCH DATA
    //--------------------------------------------------------------------------

public:

    //! This method returns the status of a user switch from the device.
    virtual bool getUserSwitch(const unsigned int a_switchIndex) const { return (cCheckBit(m_userSwitches, a_switchIndex)); }

    //! This method overrides the status of a selected user switch.
    virtual void setUserSwitch(const unsigned int a_switchIndex, const bool a_value);

    //! This method returns the status of all user switches on the device.
    virtual unsigned int getUserSwitches() const { return (m_userSwitches); }

    //! This method overrides the status of all user switches on the device.
    virtual void setUserSwitches(const unsigned int a_userSwitches);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRIPPER ANGLE POSITION AND ANGULAR VELOCITY
    //--------------------------------------------------------------------------

public:

    //! This method returns the angle in __radians__ of the gripper.
    virtual double getGripperAngleRad() const { return (m_gripperAngle); }

    //! This method overrides the angle in __radians__ of the gripper.
    virtual void setGripperAngleRad(const double& a_gripperAngleRad);

    //! This method returns the angle in __degrees__ of the gripper.
    virtual double getGripperAngleDeg() const { return (cRadToDeg(m_gripperAngle)); }

    //! This method overrides the angle in __degrees__ of the gripper.
    virtual void setGripperAngleDeg(const double& a_gripperAngleDeg);

    //! This method returns the angular velocity in __radians per second__ of the gripper.
    virtual double getGripperAngVel() const { return (m_gripperAngVel); }

    //! This method overrides the angular velocity of the gripper in __radians per second__.
    virtual void setGripperAngVel(const double& a_gripperAngVel);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DEVICE POSITION AND VELOCITY
    //--------------------------------------------------------------------------

public:

    //! This method returns the position of the haptic device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalPos() const { return (m_deviceLocalPos); }

    //! This method overrides the position of the haptic device expressed in device local coordinates.
    virtual void setDeviceLocalPos(const cVector3d& a_localPos);

    //! This method overrides the position of the haptic device expressed in device local coordinates.
    void setDeviceLocalPos(const double& a_x, const double& a_y, const double& a_z) { setDeviceLocalPos(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the position of the haptic device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalPos() const { return (m_deviceGlobalPos); }

    //! This method overrides the position of the haptic device expressed in world global coordinates.
    virtual void setDeviceGlobalPos(const cVector3d& a_globalPos);

    //! This method overrides the position of the haptic device expressed in world global coordinates.
    void setDeviceGlobalPos(const double& a_x, const double& a_y, const double& a_z) { setDeviceGlobalPos(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the linear velocity of the device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalLinVel() const { return (m_deviceLocalLinVel); }

    //! This method overrides the linear velocity of the device expressed in device local coordinates.
    virtual void setDeviceLocalLinVel(const cVector3d& a_localLinVel);

    //! This method overrides the linear velocity of the device expressed in device local coordinates.
    void setDeviceLocalLinVel(const double& a_x, const double& a_y, const double& a_z) { setDeviceLocalLinVel(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the linear velocity of the device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalLinVel() const { return (m_deviceGlobalLinVel); }

    //! This method overrides the linear velocity of the device expressed in world global coordinates.
    virtual void setDeviceGlobalLinVel(const cVector3d& a_globalLinVel);

    //! This method overrides the linear velocity of the device expressed in world global coordinates.
    void setDeviceGlobalLinVel(const double& a_x, const double& a_y, const double& a_z) { setDeviceGlobalLinVel(cVector3d(a_x, a_y, a_z)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DEVICE ROTATION AND ANGULAR VELOCITY
    //--------------------------------------------------------------------------

public:

    //! This method returns the orientation of the haptic device expressed in device local coordinates.
    virtual cMatrix3d getDeviceLocalRot() const { return (m_deviceLocalRot); }

    //! This method overrides the orientation of the haptic device expressed in device local coordinates.
    virtual void setDeviceLocalRot(const cMatrix3d& a_localRot);

    //! This method overrides the orientation of the haptic device expressed in world global coordinates.
    virtual void setDeviceGlobalRot(const cMatrix3d& a_globalRot);

    //! This method returns the orientation of the haptic device expressed in world global coordinates.
    virtual cMatrix3d getDeviceGlobalRot() const { return (m_deviceGlobalRot); }

    //! This method returns the angular velocity of the device expressed in device local coordinates.
    virtual cVector3d getDeviceLocalAngVel() const { return (m_deviceLocalAngVel); }

    //! This method overrides the angular velocity of the device expressed in device local coordinates.
    virtual void setDeviceLocalAngVel(const cVector3d& a_localAngVel);

    //! This method overrides the angular velocity of the device expressed in device local coordinates.
    void setDeviceLocalAngVel(const double& a_x, const double& a_y, const double& a_z) { setDeviceLocalAngVel(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the angular velocity of the device expressed in world global coordinates.
    virtual cVector3d getDeviceGlobalAngVel() const { return (m_deviceGlobalAngVel); }

    //! This method overrides the angular velocity of the device expressed in world global coordinates.
    virtual void setDeviceGlobalAngVel(const cVector3d& a_globalAngVel);

    //! This method overrides the angular velocity of the device expressed in world global coordinates.
    void setDeviceGlobalAngVel(const double& a_x, const double& a_y, const double& a_z) { setDeviceGlobalAngVel(cVector3d(a_x, a_y, a_z)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DEVICE POSITION AND ROTATION TRANSFORM
    //--------------------------------------------------------------------------

public:

    //! This method returns a transformation matrix which expresses the position and orientation of the haptic device in the local tool frame.
    virtual cTransform getDeviceLocalTransform() const { return (cTransform(m_deviceLocalPos, m_deviceLocalRot)); }

    //! This method overrides the transformation matrix which expresses the position and orientation of the haptic device in the local tool frame.
    virtual void setDeviceLocalTransform(const cTransform& a_localTransform);

    //! This method returns a transformation matrix which expresses the position and orientation of the haptic device in world coordinates.
    virtual cTransform getDeviceGlobalTransform() const { return (cTransform(m_deviceGlobalPos, m_deviceGlobalRot)); }

    //! This method overrides the transformation matrix which expresses the position and orientation of the haptic device in world coordinates.
    virtual void setDeviceGlobalTransform(const cTransform& a_globalTransform);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRIPPER FORCE
    //--------------------------------------------------------------------------

public:

    //! This method returns the currently computed gripper force.
    virtual double getGripperForce() const { return (m_gripperForce); }

    //! This method overrides the current gripper force.
    virtual void setGripperForce(const double& a_gripperForce);

    //! This method adds a force to the currently computed gripper force.
    virtual void addGripperForce(const double& a_gripperForce);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DEVICE FORCE
    //--------------------------------------------------------------------------

public:

    //! This method returns the currently computed force expressed in device or tool local coordinates.
    virtual cVector3d getDeviceLocalForce() const { return (m_deviceLocalForce); }

    //! This method overrides the current force expressed in device or tool local coordinates.
    virtual void setDeviceLocalForce(const cVector3d& a_localForce);

    //! This method overrides the current force expressed in device or tool local coordinates.
    void setDeviceLocalForce(const double& a_x, const double& a_y, const double& a_z) { setDeviceLocalForce(cVector3d(a_x, a_y, a_z)); }

    //! This method adds a force to the currently computed force expressed in device or tool local coordinates.
    virtual void addDeviceLocalForce(const cVector3d& a_localForce);

    //! This method adds a force to the currently computed force expressed in device or tool local coordinates.
    void addDeviceLocalForce(const double& a_x, const double& a_y, const double& a_z) { addDeviceLocalForce(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the currently computed force expressed in world coordinates.
    virtual cVector3d getDeviceGlobalForce() const { return (m_deviceGlobalForce); }

    //! This method overrides the current force expressed in world coordinates.
    virtual void setDeviceGlobalForce(const cVector3d& a_globalForce);

    //! This method overrides the current force expressed in world coordinates.
    void setDeviceGlobalForce(const double& a_x, const double& a_y, const double& a_z) { setDeviceGlobalForce(cVector3d(a_x, a_y, a_z)); }

    //! This method adds a force to the currently computed force expressed in world coordinates.
    virtual void addDeviceGlobalForce(const cVector3d& a_globalForce);

    //! This method adds a force to the currently computed force expressed in world coordinates.
    void addDeviceGlobalForce(const double& a_x, const double& a_y, const double& a_z) { addDeviceGlobalForce(cVector3d(a_x, a_y, a_z)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DEVICE TORQUE
    //--------------------------------------------------------------------------

public:

    //! This method returns the currently computed torque expressed in device or tool local coordinates.
    virtual cVector3d getDeviceLocalTorque() const { return (m_deviceLocalTorque); }

    //! This method overrides the current torque expressed in device local coordinates.
    virtual void setDeviceLocalTorque(const cVector3d& a_localTorque);

    //! This method overrides the current torque expressed in device local coordinates.
    void setDeviceLocalTorque(const double& a_x, const double& a_y, const double& a_z) { setDeviceLocalTorque(cVector3d(a_x, a_y, a_z)); }

    //! This method adds a torque to the currently computed torque expressed in device local coordinates.
    virtual void addDeviceLocalTorque(const cVector3d& a_localTorque);

    //! This method adds a torque to the currently computed torque expressed in device local coordinates.
    void addDeviceLocalTorque(const double& a_x, const double& a_y, const double& a_z) { addDeviceLocalTorque(cVector3d(a_x, a_y, a_z)); }

    //! This method returns the currently computed torque expressed in world coordinates.
    virtual cVector3d getDeviceGlobalTorque() const { return (m_deviceGlobalTorque); }

    //! This method overrides the current torque expressed in world coordinates.
    virtual void setDeviceGlobalTorque(const cVector3d& a_globalTorque);

    //! This method overrides the current torque expressed in world coordinates.
    void setDeviceGlobalTorque(const double& a_x, const double& a_y, const double& a_z) { setDeviceGlobalTorque(cVector3d(a_x, a_y, a_z)); }

    //! This method adds a torque to the currently computed torque expressed in world coordinates.
    virtual void addDeviceGlobalTorque(const cVector3d& a_globalTorque);

    //! This method adds a torque to the curently computed torque expressed in world coordinates.
    void addDeviceGlobalTorque(const double& a_x, const double& a_y, const double& a_z) { addDeviceGlobalTorque(cVector3d(a_x, a_y, a_z)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WORKSPACE SETTINGS
    //--------------------------------------------------------------------------

public:

    //! This method set the translational virtual workspace dimensions in which tool will be working.
    bool setWorkspaceRadius(const double& a_workspaceRadius);

    //! This method returns the radius of the workspace of the tool.
    double getWorkspaceRadius() { return(m_workspaceRadius); }

    //! This method sets the scale factor between the workspaces of the haptic device and the virtual tool.
    bool setWorkspaceScaleFactor(const double& a_workspaceScaleFactor);

    //! This method returns the current workspace scale factor between the haptic device and the tool.
    double getWorkspaceScaleFactor() { return (m_workspaceScaleFactor); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC POINTS
    //--------------------------------------------------------------------------

public:

    //! This method checks if the tool is touching a particular object.
    virtual bool isInContact(cGenericObject* a_object);

    //! This method returns the number of haptic points that describe this tool.
    int getNumHapticPoints() { return (int)(m_hapticPoints.size()); }

    //! This method returns a pointer to a haptic point by passing its index number.
    cHapticPoint* getHapticPoint(int a_index) { return (m_hapticPoints[a_index]); }

    //! This method sets the radius size of all haptic points.
    virtual void setRadius(double a_radius);

    //! This method sets the radius size of all haptic points.
    virtual void setRadius(double a_radiusDisplay, double a_radiusContact);

    //! This method sets the radius size of the of the physical proxy used in all haptic points.
    virtual void setRadiusContact(double a_radiusContact);

    //! This method sets the radius size of the spheres used to display the proxies and goals positions.
    virtual void setRadiusDisplay(double a_radiusDisplay); 

    //! This method sets the display properties of all haptic points.
    virtual void setShowContactPoints(bool a_showProxy = true, 
                                      bool a_showGoal = false, 
                                      cColorf a_colorLine = cColorf(0.5, 0.5, 0.5));


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WORLD
    //--------------------------------------------------------------------------

public:

    //! This method returns the parent world of this tool.
    cWorld* getParentWorld() { return (m_parentWorld); }

    //! This method enables or disables the dynamic proxy algorithm to support environments where mesh objects move.
    virtual void enableDynamicObjects(bool a_enabled);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - AUDIO
    //--------------------------------------------------------------------------

public:

    //! This method creates an audio source for each haptic point of this tool.
    bool createAudioSource(cAudioDevice* a_audioDevice);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC REPRESENTATION
    //--------------------------------------------------------------------------

public:

    //! This method renders the tool graphically using OpenGL.
    virtual void render(cRenderOptions& a_options) {};

    //! This method assigns a shader program to this object, optionally propagating the operation to its children..
    virtual void setShaderProgram(cShaderProgramPtr a_shaderProgram, const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS 
    //--------------------------------------------------------------------------

public:

    //! Generic image model that can be used to display any a representation of the tool.
    cGenericObject* m_image;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC POINTS
    //--------------------------------------------------------------------------

protected:

    //! Haptic points that describe the tool.
    std::vector <cHapticPoint*> m_hapticPoints;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC DEVICE
    //--------------------------------------------------------------------------

protected:

    //! Handle to the haptic device.
    cGenericHapticDevicePtr m_hapticDevice;

    //! Status of the user switches of the device attached to the tool.
    unsigned int m_userSwitches;

    //! If __true__ then the tool has been started and is enabled. __false__ otherwise.
    bool m_enabled;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - FORCE DATA
    //--------------------------------------------------------------------------

protected:

    //! Currently computed interaction force [N] in device or tool local coordinates.
    cVector3d m_deviceLocalForce;

    //! Currently computed interaction force [N] in world global coordinates.
    cVector3d m_deviceGlobalForce;

    //! Currently computed interaction torque [N*m] in device or tool local coordinates.
    cVector3d m_deviceLocalTorque;

    //! Currently computed interaction torque [N*m] in world global coordinates.
    cVector3d m_deviceGlobalTorque;

    //! Currently computed interaction gripper force [N].
    double m_gripperForce;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - TOOL POSE AND WORKSPACE SETTINGS
    //--------------------------------------------------------------------------

protected:

    //! Radius of the workspace which can be accessed by the tool.
    double m_workspaceRadius;

    //! Scale factor applied between the workspaces of the haptic device and the tool.
    double m_workspaceScaleFactor;

    //! Position of the haptic device in device local coordinates.
    cVector3d m_deviceLocalPos;

    //! Position of the haptic device in world global coordinates.
    cVector3d m_deviceGlobalPos;

    //! Orientation of the haptic device in device local coordinates.
    cMatrix3d m_deviceLocalRot;

    //! Orientation of the haptic device in world global coordinates.
    cMatrix3d m_deviceGlobalRot;

    //! Linear velocity of the haptic device in device local coordinates.
    cVector3d m_deviceLocalLinVel;

    //! Linear velocity of the haptic device in world global coordinates.
    cVector3d m_deviceGlobalLinVel;

    //! Angular velocity of the haptic device in device local coordinates.
    cVector3d m_deviceLocalAngVel;

    //! Angular velocity of the haptic device in world global coordinates.
    cVector3d m_deviceGlobalAngVel;

    //! Gripper angle in radians.
    double m_gripperAngle;

    //! Gripper angular velocity in radians per second.
    double m_gripperAngVel;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - INTERNAL
    //--------------------------------------------------------------------------

protected:

    //! Parent world of tool.
    cWorld* m_parentWorld;

    //! This flag indicates whether the forces are enabled or disabled on the hapitc device.
    bool m_forceOn;

    //! This flag is used to avoid initial bumps in force (has the user sent a _small_ force yet?).
    bool m_forceEngaged;

    //! The flag indicates if the device should wait for a small commanded force before engaging forces.
    bool m_useWaitForSmallForce;

    //! Counter which waits for up to several cycles of "small forces" before enabling force.
    int m_smallForceCounter;

    //! Force threshold that determines when forces are engaged if \ref m_useWaitForSmallForce is enabled.
    double m_smallForceThresh;

    //! The flag indicates if the force rising mode is enabled or disabled.
    bool m_useForceRise;

    //! Force rising time in seconds.
    double m_forceRiseTime;

    //! Clock to measure force rise.
    cPrecisionClock m_forceRiseClock;

    //! Flag that indicates when force ramping is called for the first time.
    bool m_flagForceRiseFirstTime;

    //! Flag that indicates if forces are currently rising.
    bool m_flagForceRiseActivated;

    //! Frequency counter to measure how often data is read from haptic device.
    cFrequencyCounter m_freqRead;

    //! Frequency counter to measure how often data is writing to haptic device.
    cFrequencyCounter m_freqWrite;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method updates the position and orientation of the tool image.
    virtual void updateToolImagePosition();

    //! This method updates the global position of this tool in the world.
    virtual void updateGlobalPositions(const bool a_frameOnly);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
