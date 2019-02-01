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
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CPointArray.h"
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "world/CMultiPoint.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method checks if the given line point intersects a selected a point from
    this array.\n

    If a collision occurs, the collision point is reported to the collision
    recorder.\n

    \param  a_elementIndex   Point index number.
    \param  a_object         Pointer to the object on which collision detection is being performed.
    \param  a_segmentPointA  Point from where collision ray starts (in local frame).
    \param  a_segmentPointB  Direction vector of collision ray (in local frame).
    \param  a_recorder       Stores collision events.
    \param  a_settings       Collision detection settings.

    \return __true__ if a collision has occurred, otherwise __false__.
*/
//==============================================================================
bool cPointArray::computeCollision(const unsigned int a_elementIndex,
                                      cGenericObject* a_object,
                                      cVector3d& a_segmentPointA,
                                      cVector3d& a_segmentPointB,
                                      cCollisionRecorder& a_recorder,
                                      cCollisionSettings& a_settings) const
{
    // verify that point is active
    if (!m_allocated[a_elementIndex]) { return (false); }

    // temp variables
    bool hit = false;
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double collisionDistanceSq = C_LARGE;
    double collisionPointV01 = 0.0;

    // retrieve vertex positions
    cVector3d vertex0 = m_vertices->getLocalPos(getVertexIndex0(a_elementIndex));

    // If m_collisionRadius == 0, no collision is possible.
    if (a_settings.m_collisionRadius == 0.0)
    {
        return (false);
    }

    // If m_collisionRadius > 0, we search for a possible intersection between
    // the segment AB and the point and m_collisionRadius.
    else
    {
        cVector3d t_collisionPoint, t_collisionNormal;
        double t_collisionDistanceSq;

        // check for collision between sphere located at vertex 0.
        cVector3d t_p, t_n;
        if (cIntersectionSegmentSphere(a_segmentPointA,
                                       a_segmentPointB,
                                       vertex0,
                                       a_settings.m_collisionRadius,
                                       t_collisionPoint,
                                       t_collisionNormal,
                                       t_p,
                                       t_n) > 0)
        {
            hit = true;
            t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
            if (t_collisionDistanceSq <= collisionDistanceSq)
            {
                collisionPoint = t_collisionPoint;
                collisionNormal = t_collisionNormal;
                collisionDistanceSq = t_collisionDistanceSq;
                collisionPointV01 = 0.0;
            }
        }
    }

    // report collision
    if (hit)
    {
        // we verify if a new collision needs to be created or if we simply
        // need to update the nearest collision.
        if (a_settings.m_checkForNearestCollisionOnly)
        {
            // no new collision event is create. We just check if we need
            // to update the nearest collision
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                // report basic collision data
                a_recorder.m_nearestCollision.m_type = C_COL_POINT;
                a_recorder.m_nearestCollision.m_object = a_object;
                a_recorder.m_nearestCollision.m_points = ((cMultiPoint*)(a_object))->m_points;
                a_recorder.m_nearestCollision.m_index = a_elementIndex;
                a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;
                a_recorder.m_nearestCollision.m_posV01 = collisionPointV01;
                a_recorder.m_nearestCollision.m_posV02 = 0.0;

                // report advanced collision data
                if (!a_settings.m_returnMinimalCollisionData)
                {
                    a_recorder.m_nearestCollision.m_globalPos = cAdd(a_object->getGlobalPos(),
                            cMul(a_object->getGlobalRot(),
                                    a_recorder.m_nearestCollision.m_localPos));
                    a_recorder.m_nearestCollision.m_globalNormal = cMul(a_object->getGlobalRot(),
                            a_recorder.m_nearestCollision.m_localNormal);
                }
            }
        }
        else
        {
            cCollisionEvent newCollisionEvent;

            // report basic collision data
            newCollisionEvent.m_type = C_COL_POINT;
            newCollisionEvent.m_object = a_object;
            newCollisionEvent.m_points = ((cMultiPoint*)(a_object))->m_points;
            newCollisionEvent.m_index = a_elementIndex;
            newCollisionEvent.m_localPos = collisionPoint;
            newCollisionEvent.m_localNormal = collisionNormal;
            newCollisionEvent.m_squareDistance = collisionDistanceSq;
            newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;
            newCollisionEvent.m_posV01 = collisionPointV01;
            newCollisionEvent.m_posV02 = 0.0;

            // report advanced collision data
            if (!a_settings.m_returnMinimalCollisionData)
            {
                newCollisionEvent.m_globalPos = cAdd(a_object->getGlobalPos(),
                                                        cMul(a_object->getGlobalRot(),
                                                        newCollisionEvent.m_localPos));
                newCollisionEvent.m_globalNormal = cMul(a_object->getGlobalRot(),
                                                        newCollisionEvent.m_localNormal);
            }

            // add new collision even to collision list
            a_recorder.m_collisions.push_back(newCollisionEvent);

            // check if this new collision is a candidate for "nearest one"
            if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
            {
                a_recorder.m_nearestCollision = newCollisionEvent;
            }
        }
    }

    return (hit);
}


//==============================================================================
/*!
    This method creates a copy of all allocated points. Please note that this 
    method does not copy points that were previously deallocated.

    \return  New point array.
*/
//==============================================================================
cPointArrayPtr cPointArray::copy()
{
    // create new array of points
    cPointArrayPtr pointArray = cPointArray::create(m_vertices);

    // get number of allocated points
    unsigned int numPoints = getNumElements();

    // copy every allocated points
    for (unsigned int i=0; i<numPoints; i++)
    {
        if (getAllocated(i))
        {
            unsigned int index0 = getVertexIndex0(i);

            pointArray->newPoint(index0);
        }
    }

    // return new point array
    return (pointArray);
}


//==============================================================================
/*!
    This method compress the point array by removing all non allocated points.
    If many points are unused, this method can effectively reduce 
    the memory footprint allocated by the array.\n

    __IMPORTANT:__ \n
    After calling this method, it is important to immediately update any
    collision detector as the collision tree may try to access points
    that no longer exist.\n
*/
//==============================================================================
void cPointArray::compress()
{
    // get number of allocated points
    unsigned int numPoints = getNumElements();

    // remove non allocated points
    unsigned int j= (unsigned int)(-1);
    for (unsigned int i=0; i<numPoints; i++)
    {
        if (getAllocated(i))
        {
            j++;

            if (i!=j)
            {
                unsigned int index = getVertexIndex0(i);
                m_allocated[j] = true;
                setVertex(j, index);
            }
        }
    }

    // resize arrays
    unsigned size = j+1;
    m_allocated.resize(size);
    m_indices.resize(1*size);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
