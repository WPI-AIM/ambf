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
    \version   3.2.0 $Rev: 2162 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "forces/CAlgorithmFingerProxy.h"
//------------------------------------------------------------------------------
#include "world/CWorld.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cAlgorithmFingerProxy.
*/
//==============================================================================
cAlgorithmFingerProxy::cAlgorithmFingerProxy()
{
    // no world defined yet
    m_world = NULL;

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

    // set epsilon base value
    setEpsilonBaseValue(0.0001);

    // force model settings
    m_frictionDynHysteresisMultiplier = 0.6;
    m_forceShadingAngleThreshold = 0.75;

    // initialize device and proxy positions
    m_deviceGlobalPos.zero();
    m_proxyGlobalPos.zero();
    m_lastGlobalForce.zero();

    // this will generally be over-written by the calling pointer
    m_radius = 0.01;

    // by default, we do not use dynamic proxy (which handles dynamic objects)
    m_useDynamicProxy = false;

    // initialize dynamic proxy members
    m_collisionRecorderConstraint0.m_nearestCollision.clear();
    m_collisionRecorderConstraint1.m_nearestCollision.clear();
    m_collisionRecorderConstraint2.m_nearestCollision.clear();

    // initilize local points
    m_contactPointLocalPos0.zero();
    m_contactPointLocalPos1.zero();
    m_contactPointLocalPos2.zero();

    // friction properties
    m_slipping = true;

    // setup collision detector settings
    m_collisionSettings.m_checkForNearestCollisionOnly  = true;
    m_collisionSettings.m_returnMinimalCollisionData    = false;
    m_collisionSettings.m_checkVisibleObjects           = false;
    m_collisionSettings.m_checkHapticObjects            = true;
    m_collisionSettings.m_ignoreShapes                  = true;
    m_collisionSettings.m_adjustObjectMotion            = m_useDynamicProxy;

    // setup pointers to collision recorders so that user can access
    // collision information about each haptic point.
    m_collisionEvents[0] = &(m_collisionRecorderConstraint0.m_nearestCollision);
    m_collisionEvents[1] = &(m_collisionRecorderConstraint1.m_nearestCollision);
    m_collisionEvents[2] = &(m_collisionRecorderConstraint2.m_nearestCollision);

    // initialize algorithm variables
    m_algoCounter = 0;

    // render settings (for debug purposes)
    m_showEnabled = true;
}


//==============================================================================
/*!
    This method Initializes the algorithm, including setting the pointer to the world
    in which the algorithm is to operate, and setting the initial position
    of the device.

    \param  a_world                  Pointer to world in which force algorithm is operating.
    \param  a_initialGlobalPosition  Initial position of the haptic device.
*/
//==============================================================================
void cAlgorithmFingerProxy::initialize(cWorld* a_world, const cVector3d& a_initialGlobalPosition)
{
    // reset some variables
    m_lastGlobalForce.zero();

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

    // the proxy can slip along surfaces
    m_slipping = true;

    // initialize counter
    m_algoCounter = 0;

    // initialize device and proxy positions
    m_deviceGlobalPos = a_initialGlobalPosition;
    m_proxyGlobalPos  = a_initialGlobalPosition;

    // set pointer to world in which force algorithm operates
    m_world = a_world;
}


//==============================================================================
/*!
    This method reset the algorithm and sets the __proxy__ position to the 
    __device__ position.
*/
//==============================================================================
void cAlgorithmFingerProxy::reset()
{
    // reset some variables
    m_lastGlobalForce.zero();

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

    // the proxy can slip along surfaces
    m_slipping = true;

    // initialize counter
    m_algoCounter = 0;

    // set proxy position to be equal to the device position
    m_proxyGlobalPos = m_deviceGlobalPos;
}


//==============================================================================
/*!
    This method sets the epsilon value which is used during geometry 
    computation of the proxy model.
*/
//==============================================================================
void cAlgorithmFingerProxy::setEpsilonBaseValue(double a_value)
{
    m_epsilonBaseValue = a_value;
    m_epsilonMinimalValue = 0.01 * m_epsilonBaseValue;
    m_epsilon = m_epsilonBaseValue;
    m_epsilonCollisionDetection = 1.0 * m_epsilon;
}


//==============================================================================
/*!
    This method computes the interaction forces that are associated with the
    position of the __device__ and the __proxy__.

    \param  a_toolPos  New position of tool
    \param  a_toolVel  New velocity of tool

    \return Haptic force.
*/
//==============================================================================
cVector3d cAlgorithmFingerProxy::computeForces(const cVector3d& a_toolPos,
                                              const cVector3d& a_toolVel)
{
    // update device position
    m_deviceGlobalPos = a_toolPos;

    // check if world has been defined; if so, compute forces
    if (m_world != NULL)
    {
        // compute next best position of proxy
        computeNextBestProxyPosition(m_deviceGlobalPos);

        // update proxy to next best position
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;

        // compute force vector applied to device
        updateForce();

        // return result
        return (m_lastGlobalForce);
    }

    // if no world has been defined in which algorithm operates, there is no force
    else
    {
        return (cVector3d(0.0, 0.0, 0.0));
    }
}


//==============================================================================
/*!
    Given the new position of the device and considering the current
    position of the proxy, this method attempts to move the proxy towards
    the device position (the goal).  If its path is blocked by an obstacle
    (e.g., a triangle in a mesh), the proxy is moved to this intersection
    point and a new goal is calculated as the closest point to the original
    goal in the half-plane above the intersection triangle.
    The process is repeated if necessary, bringing the proxy to its
    final location.

    \param  a_goal  The goal position of the __proxy__ subject to constraints.
*/
//==============================================================================
void cAlgorithmFingerProxy::computeNextBestProxyPosition(const cVector3d& a_goal)
{
    bool hit0, hit1, hit2;

    if (m_useDynamicProxy)
    {
        // adjust the proxy according moving objects that may have collided with the proxy
        adjustDynamicProxy(a_goal);

        // search for a first contact
        hit0 = computeNextProxyPositionWithContraints0(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit0) 
        { 
            return;
        }

        // search for a second contact
        hit1 = computeNextProxyPositionWithContraints1(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit1) 
        { 
            m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
            return; 
        }

        // search for a third contact
        hit2 = computeNextProxyPositionWithContraints2(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit2) 
        { 
            m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
            m_contactPointLocalPos1 = cTranspose(m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalPos());
            return; 
        }
        m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
        m_contactPointLocalPos1 = cTranspose(m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalPos());
        m_contactPointLocalPos2 = cTranspose(m_collisionRecorderConstraint2.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint2.m_nearestCollision.m_object->getGlobalPos());
    }
    else
    {
        // In order to keep the finger-proxy algorithm running fast in static mode, we only
        // compute collision with one constraint at the time. The next time
        // the algorithm is called, it searches for the second or
        // third obstacle.

        switch(m_algoCounter)
        {
            case 0:
                hit0 = computeNextProxyPositionWithContraints0(a_goal);
                if (hit0)
                {
                    m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
                }
                break;

            case 1:
                hit1 = computeNextProxyPositionWithContraints1(a_goal);
                if (hit1)
                {
                    m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
                    m_contactPointLocalPos1 = cTranspose(m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalPos());
                }
                break;

            case 2:
                hit2 = computeNextProxyPositionWithContraints2(a_goal);
                if (hit2)
                {
                    m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
                    m_contactPointLocalPos1 = cTranspose(m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint1.m_nearestCollision.m_object->getGlobalPos());
                    m_contactPointLocalPos2 = cTranspose(m_collisionRecorderConstraint2.m_nearestCollision.m_object->getGlobalRot()) * (m_nextBestProxyGlobalPos - m_collisionRecorderConstraint2.m_nearestCollision.m_object->getGlobalPos());
                }
                break;
        }
    }
}


//------------------------------------------------------------------------------

void cAlgorithmFingerProxy::adjustDynamicProxy(const cVector3d& a_goal)
{
        // set collision settings for dynamic proxy
        cCollisionSettings collisionSettings;
        collisionSettings.m_checkForNearestCollisionOnly  = false;
        collisionSettings.m_returnMinimalCollisionData    = false;
        collisionSettings.m_checkVisibleObjects           = false;
        collisionSettings.m_checkHapticObjects            = true;
        collisionSettings.m_ignoreShapes                  = true;
        collisionSettings.m_adjustObjectMotion            = true;
        collisionSettings.m_collisionRadius = m_radius;

        // setup recorder
        cCollisionRecorder collisionRecorder;
        collisionRecorder.clear();

        cVector3d nextProxyOffset(0.0, 0.0, 0.0);

        // check if any moving objects have hit the proxy
        bool hit = m_world->computeCollisionDetection(m_proxyGlobalPos,
                                                        m_proxyGlobalPos,
                                                        collisionRecorder,
                                                        collisionSettings);

        // one or more collisions have occured
        if (hit)
        {
            int numCollisions = (int)(collisionRecorder.m_collisions.size());

            for (int i=0; i<numCollisions; i++)
            {
                // retrieve new position of proxy
                cVector3d posLocal = collisionRecorder.m_collisions[i].m_adjustedSegmentAPoint;
                cGenericObject* obj = collisionRecorder.m_collisions[i].m_object;
                cVector3d posGlobal = cAdd(obj->getGlobalPos(), cMul( obj->getGlobalRot(), posLocal ));
                cVector3d offset = posGlobal - m_proxyGlobalPos;

                if (offset.length() > C_SMALL)
                {
                    cVector3d projection = cProject(nextProxyOffset, offset);
                    double diff = offset.length() - projection.length();
                    if (diff > 0.0)
                    {
                        nextProxyOffset = nextProxyOffset + diff * cNormalize(offset); 
                    }
                }
            }
        }

        // compute new proxy position to stay away from collision with objects moving
        m_proxyGlobalPos = m_proxyGlobalPos + nextProxyOffset;
        
        // if possible maintain proxy on surface of objects in contact
        if (m_numCollisionEvents == 1)
        {
            collisionRecorder.clear();
            collisionSettings.m_checkForNearestCollisionOnly = true;

            // computed new desired position of proxy on consytraint 0
            cVector3d globalPos = m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos();
            cMatrix3d globalRot = m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot();
            cVector3d proxyDesiredGlobalPos = globalPos + globalRot * m_contactPointLocalPos0;

            // check if any objects are in the way
            bool hit = m_world->computeCollisionDetection(m_proxyGlobalPos,
                                                          proxyDesiredGlobalPos,
                                                          collisionRecorder,
                                                          collisionSettings);

            // no collision has occured, therfore proxy can move with object
            if (!hit)
            {
                m_proxyGlobalPos = proxyDesiredGlobalPos;
            }
        }
}

//------------------------------------------------------------------------------

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints0(const cVector3d& a_goalGlobalPos)
{
    // we define the goal position of the proxy.
    cVector3d goalGlobalPos = a_goalGlobalPos;

    // to address numerical errors of the computer, we make sure to keep the proxy
    // slightly above any triangle and not directly on it. If we are using a radius of
    // zero, we need to define a default small value for epsilon
    m_epsilonInitialValue = fabs(0.0001 * m_radius);
    if (m_epsilonInitialValue < m_epsilonBaseValue)
    {
        m_epsilonInitialValue = m_epsilonBaseValue;
    }

    // the epsilon value is dynamic (can be reduced); we set it to its initial
    // value if the proxy is not touching any triangle.
    if (m_numCollisionEvents == 0)
    {
        m_epsilon = m_epsilonInitialValue;
        m_slipping = true;
    }

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position

    // compute the distance between the proxy and the goal positions
    double distanceProxyGoal = cDistance(m_proxyGlobalPos, goalGlobalPos);

    // a vector from the proxy to the goal
    cVector3d vProxyToGoal;
    cVector3d vProxyToGoalNormalized;

    if (distanceProxyGoal > m_epsilon)
    {
        goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);
        vProxyToGoal.normalizer(vProxyToGoalNormalized);
    }
    else
    {
        vProxyToGoal.zero();
        vProxyToGoalNormalized.zero();
    }

    // test whether the path from the proxy to the goal is obstructed;
    // for this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos =  goalGlobalPos + cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for a collision between the first segment (proxy-device)
    // and the environment.
    m_collisionRecorderConstraint0.clear();
    bool hit = m_world->computeCollisionDetection(m_proxyGlobalPos,
                                                  targetPos,
                                                  m_collisionRecorderConstraint0,
                                                  m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint0.m_nearestCollision.m_squareDistance);

        if (collisionDistance > (distanceProxyGoal + C_SMALL))
        {
            hit = false;
        }


        if (hit)
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // if no collision occurs, then we move the proxy to its goal, and we're done
    if (!hit)
    {
        m_numCollisionEvents = 0;
        m_algoCounter = 0;
        m_slipping = true;
        m_nextBestProxyGlobalPos = goalGlobalPos;
        return (false);
    }

    // a first collision has occurred
    m_algoCounter = 1;


    //--------------------------------------------------------------------------
    // FIRST COLLISION OCCURES:
    //--------------------------------------------------------------------------

    // we want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // so we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);

    // now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // if only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / fabs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // we compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // a vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_collisionEvents[0]->m_globalPos, vCollisionToProxy);

    // move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  we still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[0]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numCollisionEvents = 1;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//------------------------------------------------------------------------------

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints1(const cVector3d& a_goalGlobalPos)
{
    cVector3d goalGlobalPos;

    if (m_collisionRecorderConstraint0.m_nearestCollision.m_object->m_material->getUseHapticShading())
    {
        // computed adjusted surface normal for modeling haptic shading and texture rendering
        cVector3d normal = computeShadedSurfaceNormal(&m_collisionRecorderConstraint0.m_nearestCollision);

        // the proxy is now constrained on the shaded plane; we now calculate the nearest
        // point to the original goal (device position) on this shaded plane; this point
        // is computed by projecting the desired goal onto the shaded plane.
        cVector3d goalGlobalPosOnForceShadingPlane = cProjectPointOnPlane(a_goalGlobalPos,
                  m_proxyGlobalPos,
                  normal);

        // we now project the point from the shaded plane back onto the original triangle
        // (See publication: The Haptic Display of Complex Graphical Environments from
        // Ruspini and al. for more information about the shading algorithm)
        goalGlobalPos = cProjectPointOnPlane(goalGlobalPosOnForceShadingPlane,
                                             m_proxyGlobalPos,
                                             m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);
    }
    else
    {
        // the proxy is now constrained on a plane; we now calculate the nearest
        // point to the original goal (device position) on this plane; this point
        // is computed by projecting the desired goal onto the plane defined by the
        // intersected triangle.
        goalGlobalPos = cProjectPointOnPlane(a_goalGlobalPos,
                                             m_proxyGlobalPos,
                                             m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);
    }

    // a vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numCollisionEvents = 1;
        return (false);
    }

    // compute the normalized vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint1.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint1,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint1.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + C_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so.
    if (!hit)
    {
        testFrictionAndMoveProxy(goalGlobalPos,
                                 m_proxyGlobalPos,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_object);

        m_numCollisionEvents = 1;
        m_algoCounter = 0;

        return (false);
    }

    // a second collision has occurred
    m_algoCounter = 2;


    //--------------------------------------------------------------------------
    // SECOND COLLISION OCCURES:
    //--------------------------------------------------------------------------
    
    // we want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle; we want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.

    // so we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal);

    // now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // if only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / fabs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // we compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // a vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_collisionEvents[1]->m_globalPos, vCollisionToProxy);

    // move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[1]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numCollisionEvents = 2;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//------------------------------------------------------------------------------

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints2(const cVector3d& a_goalGlobalPos)
{
    // the proxy is now constrained by two triangles and can only move along
    // a virtual line; we now calculate the nearest point to the original
    // goal (device position) along this line by projecting the ideal
    // goal onto the line.
    //
    // The line is expressed by the cross product of both surface normals,
    // which have both been oriented to point away from the device
    cVector3d line;
    m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal.crossr(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal, line);

    // check result.
    if (line.equals(cVector3d(0,0,0)))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numCollisionEvents = 2;
        return (false);
    }

    line.normalize();

    // compute the projection of the device position (goal) onto the line; this
    // gives us the new goal position.
    cVector3d goalGlobalPos = cProjectPointOnLine(a_goalGlobalPos, m_proxyGlobalPos, line);

    // a vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numCollisionEvents = 2;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint2.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint2,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint2.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + C_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // if no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so
    if (!hit)
    {
        cVector3d normal = cMul(0.5,cAdd(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
                                         m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal));

        testFrictionAndMoveProxy(goalGlobalPos,
                                 m_proxyGlobalPos,
                                 normal,
                                 m_collisionRecorderConstraint1.m_nearestCollision.m_object);
        m_numCollisionEvents = 2;
        m_algoCounter = 0;

        return (false);
    }


    //--------------------------------------------------------------------------
    // THIRD COLLISION OCCURES:
    //--------------------------------------------------------------------------

    // we want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle; we want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.

    // so we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint2.m_nearestCollision.m_globalNormal);

    // now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // if only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / fabs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // we compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // a vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_collisionEvents[2]->m_globalPos, vCollisionToProxy);

    // move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[2]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;
    m_algoCounter = 0;
    m_numCollisionEvents = 3;

    // TODO: There actually should be a third friction test to see if we
    // can make it to our new goal position, but this is generally such a
    // small movement in one iteration that it's irrelevant...
    return (true);
}


//==============================================================================
/*!
    This method tests whether the proxy has reached the goal point, allowing
    for subclass-specific approximations.

    \param  a_goal  The location to which we'd like to move the proxy.
    \param  a_proxy The current position of the proxy.

    \return __true__ is the proxy has effectively reached the goal, __false__ otherwise.
*/
//==============================================================================
bool cAlgorithmFingerProxy::goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const
{
    return (a_proxy.distance(a_goal) < (m_epsilonBaseValue));
}


//==============================================================================
/*!
    This method attempts to move the proxy, subject to friction constraints. 
    This is called from \ref computeNextBestProxyPosition() when the proxy is 
    ready to move along a known surface.

    \param  a_goal    The location to which we'd like to move the proxy.
    \param  a_proxy   The current position of the proxy.
    \param  a_normal  The surface normal at the obstructing surface.
    \param  a_parent  The surface along which we're moving.
*/
//==============================================================================
void cAlgorithmFingerProxy::testFrictionAndMoveProxy(const cVector3d& a_goal, 
                                                    const cVector3d& a_proxy,
                                                    cVector3d& a_normal, 
                                                    cGenericObject* a_parent)
{
    // check if friction is enabled
    if (!a_parent->m_material->getUseHapticFriction())
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // compute penetration depth; how far is the device "behind" the
    // plane of the obstructing surface
    cVector3d projectedGoal = cProjectPointOnPlane(m_deviceGlobalPos, a_proxy, a_normal);
    double penetrationDepth = cSub(m_deviceGlobalPos,projectedGoal).length();

    // find the appropriate friction coefficient
    double mud = a_parent->m_material->getDynamicFriction();
    double mus = a_parent->m_material->getStaticFriction();

    // no friction; don't try to compute friction cones
    if ((mud == 0) && (mus == 0))
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // the corresponding friction cone radii
    double atmd = atan(mud);
    double atms = atan(mus);

    // compute a vector from the device to the proxy, for computing
    // the angle of the friction cone
    cVector3d vDeviceProxy = cSub(a_proxy, m_deviceGlobalPos);
    vDeviceProxy.normalize();

    // now compute the angle of the friction cone...
    double theta = acos(vDeviceProxy.dot(a_normal));

    // manage the "slip-friction" state machine

    // if the dynamic friction radius is for some reason larger than the
    // static friction radius, always slip
    if (mud > mus)
    {
        m_slipping = true;
    }

    // if we're slipping...
    else if (m_slipping)
    {
        if (theta < (atmd * m_frictionDynHysteresisMultiplier))
        {
            m_slipping = false;
        }
        else
        {
            m_slipping = true;
        }
    }

    // if we're not slipping...
    else
    {
        if (theta > atms)
        {
            m_slipping = true;
        }
        else
        {
            m_slipping = false;
        }
    }

    // the friction coefficient we're going to use...
    double mu;
    if (m_slipping)
    {
        mu = mud;
    }
    else
    {
        mu = mus;
    }

    // calculate the friction radius as the absolute value of the penetration
    // depth times the coefficient of friction
    double frictionRadius = fabs(penetrationDepth * mu);

    // calculate the distance between the proxy position and the current
    // goal position.
    double r = a_proxy.distance(a_goal);

    // if this distance is smaller than C_SMALL, we consider the proxy
    // to be at the same position as the goal, and we're done...
    if (r < C_SMALL)
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

    // if the proxy is outside the friction cone, update its position to
    // be on the perimeter of the friction cone...
    else if (r > frictionRadius)
    {
        m_nextBestProxyGlobalPos = cAdd(a_goal, cMul(frictionRadius/r, cSub(a_proxy, a_goal)));
    }

    // otherwise, if the proxy is inside the friction cone, the proxy
    // should not be moved (set next best position to current position)
    else
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

    // we're done; record the fact that we're still touching an object...
    return;
}


//==============================================================================
/*!
    This method uses the contact to determine the force to apply to the device.
    The function computes a force proportional to the distance between the
    positions of the proxy and the device and scaled by the average
    stiffness of each contact triangle.
*/
//==============================================================================
void cAlgorithmFingerProxy::updateForce()
{
    // initialize variables
    double stiffness;
    cVector3d averagedSurfaceNormal;

    //---------------------------------------------------------------------
    // CHECK FOR CONTACTS
    //---------------------------------------------------------------------

    // if proxy is not touching any object, then the force is simply set to zero.
    if (m_numCollisionEvents == 0)
    {
        m_tangentialForce.zero();
        m_normalForce.zero();
        m_lastGlobalForce.zero();
        return;
    }


    //---------------------------------------------------------------------
    // STIFFNESS AND SURFACE NORMAL ESTIMATION 
    //---------------------------------------------------------------------

    // initialize surface normal and stiffness values
    averagedSurfaceNormal.zero();
    stiffness = 0.0;

    // compute the average surface normal and stiffness
    for (unsigned int i=0; i<m_numCollisionEvents; i++)
    {
        // compute stiffness
        stiffness += ( m_collisionEvents[i]->m_object->m_material->getStiffness() );

        // compute surface normal
        averagedSurfaceNormal.add(m_collisionEvents[i]->m_globalNormal);
    }

    if (m_numCollisionEvents > 0)
    {
        double scale = 1.0/(double)m_numCollisionEvents;
        stiffness *= scale;
        averagedSurfaceNormal.mul(scale);
    }


    //---------------------------------------------------------------------
    // COMPUTE FORCE
    //---------------------------------------------------------------------

    // compute the force by modeling a spring between the proxy and the device
    cVector3d force;
    m_proxyGlobalPos.subr(m_deviceGlobalPos, force);
    force.mul(stiffness);

    if (force.lengthsq() == 0)
    {
        m_normalForce.zero();
        m_tangentialForce.zero();
        m_lastGlobalForce.zero();
        return;
    }


    //---------------------------------------------------------------------
    // APPLY TEXTURE RENDERING
    //---------------------------------------------------------------------

    // is haptic texture rendering enabled?
    bool useHapticTexture = m_collisionEvents[0]->m_object->m_material->getUseHapticTexture();
    if ((useHapticTexture) && (m_collisionEvents[0]->m_type == C_COL_TRIANGLE))
    {
        for (unsigned int i=0; i<1; i++)
        {
            if (m_collisionEvents[i] != NULL)
            {
                // get triangle physical normal
                cVector3d normal = m_collisionEvents[i]->m_globalNormal;

                // compute normal and tangential components of the force for current triangle
                cVector3d tangentialForce, normalForce;

                // compute tangential and normal forces for current force
                normalForce = cProject(force, normal);
                force.subr(normalForce, tangentialForce);

                // get vertices of contact triangles
                unsigned int index0 =  m_collisionEvents[i]->m_triangles->getVertexIndex0(m_collisionEvents[i]->m_index);
                unsigned int index1 =  m_collisionEvents[i]->m_triangles->getVertexIndex1(m_collisionEvents[i]->m_index);
                unsigned int index2 =  m_collisionEvents[i]->m_triangles->getVertexIndex2(m_collisionEvents[i]->m_index);

                cVector3d vertex0 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangles->m_vertices->getLocalPos(index0)));
                cVector3d vertex1 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangles->m_vertices->getLocalPos(index1)));
                cVector3d vertex2 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangles->m_vertices->getLocalPos(index2)));

                // retrieve pointer to normal map object
                cNormalMapPtr normalMap = m_collisionEvents[i]->m_object->m_normalMap;

                // sanity check
                if (normalMap != nullptr)
                {
                    // retrieve pointer to texture texture object
                    cTexture1dPtr texture = m_collisionEvents[i]->m_object->m_texture;

                    // retrieve the texture coordinate for each triangle vertex
             
                    cVector3d texCoord0 = m_collisionEvents[i]->m_triangles->m_vertices->getTexCoord(index0);
                    cVector3d texCoord1 = m_collisionEvents[i]->m_triangles->m_vertices->getTexCoord(index1);
                    cVector3d texCoord2 = m_collisionEvents[i]->m_triangles->m_vertices->getTexCoord(index2);

                    // compute texture variations
                    cVector3d vTexCoord01 = texCoord1 - texCoord0;
                    cVector3d vTexCoord02 = texCoord2 - texCoord0;

                    // compute the exact texture coordinate at the contact point
                    cVector3d texCoord = m_collisionEvents[i]->m_triangles->getTexCoordAtPosition(m_collisionEvents[i]->m_index, m_collisionEvents[i]->m_localPos);

                    // surface gradient
                    cVector3d gradientSurface(0,0,0);
                    cVector3d gradientTexture;

                    // pixel and colors
                    cColorb color00, color01, color10, color11;
                    int pixelX0, pixelX1;
                    int pixelY0, pixelY1;

                    // image size
                    int w = normalMap->m_image->getWidth();
                    int h = normalMap->m_image->getHeight();

                    // compute nearest pixels along the X axis.
                    double px = (w-1) * texCoord(0);
                    double py = (h-1) * texCoord(1);

                    pixelX0 = (int)(floor(px));
                    pixelY0 = (int)(floor(py));
                    pixelX1 = cClamp(pixelX0+1, 0, (w-1));
                    pixelY1 = cClamp(pixelY0+1, 0, (h-1));

                    // get normals from 
                    normalMap->m_image->getPixelColor(pixelX0, pixelY0, color00);
                    normalMap->m_image->getPixelColor(pixelX1, pixelY0, color10);
                    normalMap->m_image->getPixelColor(pixelX1, pixelY1, color11);
                    normalMap->m_image->getPixelColor(pixelX0, pixelY1, color01);

                    // compute relative position within 4 texels
                    double x = px - floor(px);
                    double y = py - floor(py);

                    const double SCALE = (1.0/255.0);
                    double fX00 = SCALE * (color00.getR() - 128);
                    double fY00 =-SCALE * (color00.getG() - 128);
                    double fZ00 = SCALE * (color00.getB() - 128);
                    double fX01 = SCALE * (color01.getR() - 128);
                    double fY01 =-SCALE * (color01.getG() - 128);
                    double fZ01 = SCALE * (color01.getB() - 128);
                    double fX10 = SCALE * (color10.getR() - 128);
                    double fY10 =-SCALE * (color10.getG() - 128);
                    double fZ10 = SCALE * (color10.getB() - 128);
                    double fX11 = SCALE * (color11.getR() - 128);
                    double fY11 =-SCALE * (color11.getG() - 128);
                    double fZ11 = SCALE * (color11.getB() - 128);

                    // bilinear interpolation
                    double fX = fX00 * (1-x)*(1-y) + fX10*x*(1-y) + fX01*(1-x)*y + fX11*x*y;
                    double fY = fY00 * (1-x)*(1-y) + fY10*x*(1-y) + fY01*(1-x)*y + fY11*x*y;
                    double fZ = fZ00 * (1-x)*(1-y) + fZ10*x*(1-y) + fZ01*(1-x)*y + fZ11*x*y;

                    // assign gradient (negate fy!)
                    gradientTexture.set(fX,-fY, fZ);

                    // boolean used to inform us if the projection succeeds
                    bool success = false;

                    // setup projection matrix
                    double length_vTexCoord01 = vTexCoord01.length();
                    double length_vTexCoord02 = vTexCoord02.length();

                    if ((length_vTexCoord01 > 0.0) && (length_vTexCoord02 > 0.0))
                    {
                        // normalize vector in texture coordinate space
                        vTexCoord01.div(length_vTexCoord01);
                        vTexCoord02.div(length_vTexCoord02);

                        double m00 = vTexCoord01(0);
                        double m01 = vTexCoord02(0);
                        double m10 = vTexCoord01(1);
                        double m11 = vTexCoord02(1);

                        // compute projected gradient
                        double det = m00 * m11 - m10 * m01;
                        if (det == 0.0)
                        {
                            success = false;
                        }
                        else
                        {
                            double dtx = gradientTexture(0);
                            double dty = gradientTexture(1);

                            double f = 1.0 / det;
                            double level =  m_collisionEvents[0]->m_object->m_material->getTextureLevel();
                            double c01 = level * f * ( m11 * dtx - m01 * dty);
                            double c02 = level * f * (-m10 * dtx + m00 * dty);

                            cVector3d vert01 = vertex1 - vertex0;
                            cVector3d vert02 = vertex2 - vertex0;
                            double v01_length = vert01.length();
                            double v02_length = vert02.length();

                            if ((v01_length > 0.0) && (v02_length > 0.0))
                            {
                                gradientSurface = (c01 / cSqr(v01_length)) * vert01 + (c02 / cSqr(v02_length)) * vert02 + gradientTexture(2) * normal;
                            }
                            else
                            {
                                gradientSurface.zero();
                            }

                            success = true;
                        }
                    }

                    // if the operation succeeds, we compute a new surface normal that will
                    // be used to reorient the previously computed reaction force
                    if (success)
                    {
                        cVector3d newNormal = gradientSurface * m_collisionEvents[i]->m_object->m_material->getTextureLevel();
                        double length = newNormal.length(); 
                        if (length > 0.0)
                        {
                            newNormal.div(length);
                            if (cAngle(newNormal, normal) > 1.57)
                            {
                                newNormal.negate();
                            }
                            normal = newNormal;
                        }
                    }

                    // we now compute the magnitude of the current surface normal and create a new force vector in the 
                    // direction of the gradient normal.
                    double forceMagnitude = normalForce.length();
                    force = cAdd(tangentialForce, cMul(forceMagnitude, normal));
                    surfaceNormal = normal;
                }
            }
        }
    }


    //---------------------------------------------------------------------
    // FINALIZATION
    //---------------------------------------------------------------------

    // compute tangential and normal forces for current triangle
    m_normalForce = cProject(force, averagedSurfaceNormal);
    force.subr(m_normalForce, m_tangentialForce);

    // return computed force
    m_lastGlobalForce = force;
}


//==============================================================================
/*!
    This method computes the surface normal at the proxy position.

    \param  a_contactPoint  Position of proxy.

    \return Surface normal.
*/
//==============================================================================
cVector3d cAlgorithmFingerProxy::computeShadedSurfaceNormal(cCollisionEvent* a_contactPoint)
{
    // get triangle physical normal
    cVector3d normal = a_contactPoint->m_globalNormal;

    // is force shading enabled for that object?
    bool useForceShading = a_contactPoint->m_object->m_material->getUseHapticShading();

    if ((useForceShading) && (a_contactPoint->m_type == C_COL_TRIANGLE))
    {
        if (a_contactPoint != NULL)
        {
            unsigned int index0 =  a_contactPoint->m_triangles->getVertexIndex0(a_contactPoint->m_index);
            unsigned int index1 =  a_contactPoint->m_triangles->getVertexIndex1(a_contactPoint->m_index);
            unsigned int index2 =  a_contactPoint->m_triangles->getVertexIndex2(a_contactPoint->m_index);

            // get vertices of contact triangles
            cVector3d vertex0 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getLocalPos(index0)));
            cVector3d vertex1 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getLocalPos(index1)));
            cVector3d vertex2 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getLocalPos(index2)));

            // get vertex normals of contact triangle
            cVector3d normal0 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getNormal(index0));
            cVector3d normal1 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getNormal(index1));
            cVector3d normal2 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangles->m_vertices->getNormal(index2));

            // project the current contact point on triangle
            double a0 = 0; 
            double a1 = 0;
            cProjectPointOnPlane(a_contactPoint->m_globalPos, vertex0, vertex1, vertex2, a0, a1);
            cVector3d vertex = cAdd(vertex0, cMul(a0, cSub(vertex1, vertex0)), cMul(a1, cSub(vertex2, vertex0)));

            // compute area of triangle
            double area  = cTriangleArea(vertex0, vertex1, vertex2);

            // compute areas of three sub-triangles formed by the three vertices of the triangle and the contact point.
            double area0 = cTriangleArea(vertex, vertex1, vertex2);
            double area1 = cTriangleArea(vertex, vertex0, vertex2);
            double area2 = cTriangleArea(vertex, vertex0, vertex1);
            
            // compute weights based on position of contact point
            double c0, c1, c2;
            if (area > 0.0)
            {
                c0 = area0/area;
                c1 = area1/area;
                c2 = area2/area;
            }
            else
            {
                c0 = c1 = c2 = (1.0/3.0);
            }


            //--------------------------------------------------------------------------------------------
            // FORCE SHADING
            //--------------------------------------------------------------------------------------------
            if (useForceShading)
            {
                // compute angles between normals
                double angle01 = cAngle(normal0, normal1);
                double angle02 = cAngle(normal0, normal2);
                double angle12 = cAngle(normal1, normal2);

                // if angles are withing a certain threshold, perform shading
                if ((angle01 < m_forceShadingAngleThreshold) || (angle02 < m_forceShadingAngleThreshold) || (angle12 < m_forceShadingAngleThreshold))
                {
                    // just like with Fong shading in computer graphics, we compute the surface normal at the actual contact point 
                    // by combining the normals defined at each vertex on the triangle
                    cVector3d normalShaded = normal;
                   
                    // compute new interpolated normal 
                    normalShaded = cAdd( cMul(c0, normal0), cMul(c1, normal1), cMul(c2, normal2));

                    // sanity check and normalization
                    double length = normalShaded.length();
                    if (length > 0.0)
                    {
                        normalShaded.div(length);
                    }
                    else
                    {
                        normalShaded = normal;
                    }

                    normal = normalShaded;
                }
            }
        }
    }


    //---------------------------------------------------------------------
    // return result
    //---------------------------------------------------------------------
    return (normal);
}

//==============================================================================
/*!
    This method render the force algorithm graphically using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cAlgorithmFingerProxy::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    if (!m_showEnabled) { return; }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // disable lighting
        glDisable(GL_LIGHTING);

        glLineWidth(1.0);

        cVector3d posA = m_proxyGlobalPos;
        cVector3d posB = m_proxyGlobalPos + 0.1 * surfaceNormal;

        // draw line
        glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3dv(&posA(0) );
            glVertex3dv(&posB(0) );
        glEnd();

        // restore lighting to default value
        glEnable(GL_LIGHTING);
    }

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
