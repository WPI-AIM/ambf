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
#ifndef CAlgorithmFingerProxyH
#define CAlgorithmFingerProxyH
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "forces/CGenericForceAlgorithm.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
//------------------------------------------------------------------------------
#include <map>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cWorld;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CAlgorithmFingerProxy.h

    \brief
    Implements a finger-proxy force rendering algorithm.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cAlgorithmFingerProxy
    \ingroup    forces

    \brief
    This class implements a finger-proxy force rendering algorithm.

    \details
    This class implements a finger-proxy force rendering algorithm for polygonal 
    objects.
*/
//==============================================================================
class cAlgorithmFingerProxy : public cGenericForceAlgorithm
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cAlgorithmFingerProxy.
    cAlgorithmFingerProxy();

    //! Destructor of cAlgorithmFingerProxy.
    virtual ~cAlgorithmFingerProxy() {}


    //----------------------------------------------------------------------
    // METHODS - BASIC PROXY:
    //----------------------------------------------------------------------

    //! This method initialize the algorithm.
    void initialize(cWorld* a_world, const cVector3d& a_initialGlobalPosition);

    //! This method reset the algorithm and sets the __proxy__ position to the __device__ position.
    void reset();

    //! This method calculates the interaction forces.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel);


    //----------------------------------------------------------------------
    // METHODS - GETTER AND SETTER FUNCTIONS:
    //----------------------------------------------------------------------

    //! This method sets the radius of the __proxy__.
    void setProxyRadius(const double& a_radius) { m_radius = a_radius; m_collisionSettings.m_collisionRadius = m_radius;}

    //! This method returns the radius of the __proxy__.
    inline double getProxyRadius() const { return (m_radius); }

    //! This method returns the last computed position of the __proxy__ in world coordinates.
    inline cVector3d getProxyGlobalPosition() const { return (m_proxyGlobalPos); }

    //! This method sets the position of the __proxy__ in world coordinates.
    inline void setProxyGlobalPosition(const cVector3d& a_position)  { m_proxyGlobalPos = a_position; }

    //! This method returns the last specified position of the __device__ in world coordinates.
    inline cVector3d getDeviceGlobalPosition() const { return (m_deviceGlobalPos); }

    //! This method returns the last computed force in world coordinates.
    inline cVector3d getForce() { return (m_lastGlobalForce); }

    //! This method returns the most recently calculated __normal__ force.
    inline cVector3d getNormalForce() { return (m_normalForce); }

    //! This method returns the most recently calculated __tangential__ force.
    inline cVector3d getTangentialForce() { return (m_tangentialForce); }


    //----------------------------------------------------------------------
    // METHODS - COLLISION INFORMATION BETWEEN PROXY AND WORLD
    //----------------------------------------------------------------------

public:

    //! This method return the number of collision events (0, 1, 2 or 3):
    int getNumCollisionEvents() { return (m_numCollisionEvents); }


    //----------------------------------------------------------------------
    // MEMMBERS - COLLISION INFORMATION BETWEEN PROXY AND WORLD
    //----------------------------------------------------------------------

public:

    //! Table of collision events (0-3). Call \ref getNumCollisionEvents() to see how many are actually valid.
    cCollisionEvent* m_collisionEvents[3];


    //----------------------------------------------------------------------
    // MEMBERS - FORCE MODELS
    //----------------------------------------------------------------------

public:

    //! If __true__ the the dynamic proxy is enabled to handle moving objects.
    bool m_useDynamicProxy;

    /*!
        Dynamic friction hysteresis multiplier
        In CHAI's proxy, the angle computed from the coefficient is multiplied
        by this constant to avoid rapidly oscillating between slipping and sticking
        without having to turn the dynamic friction level way down.
    */
    double m_frictionDynHysteresisMultiplier;

    //! Maximum force shading angle (radians) threshold between normals of triangle.
    double m_forceShadingAngleThreshold;

    //! Collision settings.
    cCollisionSettings m_collisionSettings;


    //----------------------------------------------------------------------
    // METHODS - RESOLUTION / ERRORS
    //----------------------------------------------------------------------

public:

    //! This method sets the epsilon tolerance error base value.
    void setEpsilonBaseValue(double a_value);

    //! This method returns the current epsilon tolerance error base value.
    double getEpsilonBaseValue() { return (m_epsilonBaseValue); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - GRAPHICS:
    //--------------------------------------------------------------------------

protected:

    //! This method renders the force algorithms graphically in OpenGL. (For debug purposes)
    virtual void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL:
    //--------------------------------------------------------------------------

protected:

    //! This method tests whether the proxy has reached the goal point.
    virtual bool goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const;

    //! This method computes the next goal position of the proxy.
    virtual void computeNextBestProxyPosition(const cVector3d& a_goal);

    //! This method attempts to move the proxy, subject to friction constraints.
    virtual void testFrictionAndMoveProxy(const cVector3d& a_goal, const cVector3d& a_proxy, cVector3d& a_normal, cGenericObject* a_parent);

    //! This method computes the resulting force which will be sent to the haptic device.
    virtual void updateForce();


    //----------------------------------------------------------------------
    // MEMBERS - PROXY, DEVICE AND FORCE INFORMATION:
    //----------------------------------------------------------------------

protected:

    //! Global position of the proxy.
    cVector3d m_proxyGlobalPos;

    //! Global position of device.
    cVector3d m_deviceGlobalPos;

    //! Last computed force (in global coordinate frame).
    cVector3d m_lastGlobalForce;

    //! Next best position for the proxy (in global coordinate frame).
    cVector3d m_nextBestProxyGlobalPos;

    //! If __true__ then we are currently in a "slip friction".
    bool m_slipping;

    //! Normal force.
    cVector3d m_normalForce;

    //! Tangential force.
    cVector3d m_tangentialForce;

    //! Number of collision events between proxy and triangles (0, 1, 2 or 3).
    unsigned int m_numCollisionEvents;

    //! Radius of the proxy.
    double m_radius;


    //----------------------------------------------------------------------
    // PROTECTED MEMBERS - PROXY ALGORITHM
    //----------------------------------------------------------------------

protected:

    //! Collision detection recorder for searching first constraint.
    cCollisionRecorder m_collisionRecorderConstraint0;

    //! Collision detection recorder for searching second constraint.
    cCollisionRecorder m_collisionRecorderConstraint1;

    //! Collision detection recorder for searching third constraint.
    cCollisionRecorder m_collisionRecorderConstraint2;

    //! Local position of contact point first object.
    cVector3d m_contactPointLocalPos0;

    //! Local position of contact point first object.
    cVector3d m_contactPointLocalPos1;

    //! Local position of contact point first object.
    cVector3d m_contactPointLocalPos2;

    /*!
        To address numerical errors during geometric computation,
        several epsilon values are computed and used.
    */

    //! Epsilon value - used for handling numerical limits.
    double m_epsilonInitialValue;

    //! Epsilon value - used for handling numerical limits.
    double m_epsilon;

    //! Epsilon value - used for handling numerical limits.
    double m_epsilonCollisionDetection;

    //! Epsilon value - used for handling numerical limits.
    double m_epsilonBaseValue;

    //! Epsilon value - used for handling numerical limits.
    double m_epsilonMinimalValue;

    //! Value of state machine.
    unsigned int m_algoCounter;


    //----------------------------------------------------------------------
    // PROTECTED METHODS - PROXY ALGORITHM
    //----------------------------------------------------------------------

protected:

    //! This method ajust the position of __proxy__ by taking into account motion of objects in the world.
    void adjustDynamicProxy(const cVector3d& a_goal);

    //! This method updates the position of the __proxy__ - constraint 0.
    bool computeNextProxyPositionWithContraints0(const cVector3d& a_goalGlobalPos);

    //! This method updates the position of the __proxy__ - constraint 1.
    bool computeNextProxyPositionWithContraints1(const cVector3d& a_goalGlobalPos);

    //! This method updates the position of the __proxy__ - constraint 2.
    bool computeNextProxyPositionWithContraints2(const cVector3d& a_goalGlobalPos);

    //! This method computes the local surface normal from interpolated vertex normals 
    cVector3d computeShadedSurfaceNormal(cCollisionEvent* a_contactPoint);


    //----------------------------------------------------------------------
    // DEBUG PURPOSES
    //----------------------------------------------------------------------

protected:

    //! Surface normal.
    cVector3d surfaceNormal;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

