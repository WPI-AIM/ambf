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
#include "graphics/CTriangleArray.h"
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "world/CMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method checks if the given line segment intersects a selected a 
    triangle from this array.\n

    If a collision occurs, the collision point is reported in the collision
    recorder.\n

    \param  a_elementIndex   Triangle index number.
    \param  a_object         Pointer to the object on which collision detection is being performed.
    \param  a_segmentPointA  Point from where collision ray starts (in local frame).
    \param  a_segmentPointB  Direction vector of collision ray (in local frame).
    \param  a_recorder       Stores collision events
    \param  a_settings       Collision detector settings.

    \return  __true__ if a collision occurred, otherwise __false__.
*/
//==============================================================================
bool cTriangleArray::computeCollision(const unsigned int a_elementIndex,
                                      cGenericObject* a_object,
                                      cVector3d& a_segmentPointA,
                                      cVector3d& a_segmentPointB,
                                      cCollisionRecorder& a_recorder,
                                      cCollisionSettings& a_settings) const
{
    // verify that triangle is active
    if (!m_allocated[a_elementIndex]) { return (false); }

    // temp variables
    bool hit = false;
    cVector3d collisionPoint;
    cVector3d collisionNormal;
    double collisionDistanceSq = C_LARGE;
    double collisionPointV01 = 0.0;
    double collisionPointV02 = 0.0;

    // retrieve information about which side of the triangles need to be checked
    cMaterialPtr material = a_object->m_material;
    bool checkFrontSide = material->getHapticTriangleFrontSide();
    bool checkBackSide = material->getHapticTriangleBackSide();

    // retrieve vertex positions
    cVector3d vertex0 = m_vertices->getLocalPos(getVertexIndex0(a_elementIndex));
    cVector3d vertex1 = m_vertices->getLocalPos(getVertexIndex1(a_elementIndex));
    cVector3d vertex2 = m_vertices->getLocalPos(getVertexIndex2(a_elementIndex));

    // If m_collisionRadius == 0, we search for a possible intersection between
    // the segment AB and the triangle defined by its three vertices V0, V1, V2.
    if (a_settings.m_collisionRadius == 0.0)
    {
        // check for collision between segment and triangle only
        if (cIntersectionSegmentTriangle(a_segmentPointA,
                                         a_segmentPointB,
                                         vertex0,
                                         vertex1,
                                         vertex2,
                                         checkFrontSide,
                                         checkBackSide,
                                         collisionPoint,
                                         collisionNormal,
                                         collisionPointV01,
                                         collisionPointV02))
        {
            hit = true;
            collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
        }
    }

    // If m_collisionRadius > 0, we search for a possible intersection between
    // the segment AB and the shell of the selected triangle which is described
    // by its three vertices and m_collisionRadius.
    else
    {
        cVector3d t_collisionPoint, t_collisionNormal;
        double t_collisionDistanceSq;
        cVector3d normal = cComputeSurfaceNormal(vertex0, vertex1, vertex2);
        cVector3d offset; normal.mulr(a_settings.m_collisionRadius, offset);
        cVector3d t_vertex0, t_vertex1, t_vertex2;
        double t_collisionPointV01, t_collisionPointV02, t_collisionPointV12;

        // check for collision between segment and triangle upper shell
        vertex0.addr(offset, t_vertex0);
        vertex1.addr(offset, t_vertex1);
        vertex2.addr(offset, t_vertex2);
        if (cIntersectionSegmentTriangle(a_segmentPointA,
                                         a_segmentPointB,
                                         t_vertex0,
                                         t_vertex1,
                                         t_vertex2,
                                         checkFrontSide,
                                         false,
                                         collisionPoint,
                                         collisionNormal,
                                         collisionPointV01,
                                         collisionPointV02))
        {
            hit = true;
            collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
        }

        // check for collision between segment and triangle lower shell
        vertex0.subr(offset, t_vertex0);
        vertex1.subr(offset, t_vertex1);
        vertex2.subr(offset, t_vertex2);
        if (cIntersectionSegmentTriangle(a_segmentPointA,
                                         a_segmentPointB,
                                         t_vertex0,
                                         t_vertex1,
                                         t_vertex2,
                                         false,
                                         checkBackSide,
                                         t_collisionPoint,
                                         t_collisionNormal,
                                         t_collisionPointV01,
                                         t_collisionPointV02))
        {
            hit = true;
            t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
            if (t_collisionDistanceSq <= collisionDistanceSq)
            {
                collisionPoint = t_collisionPoint;
                collisionNormal = t_collisionNormal;
                collisionDistanceSq = t_collisionDistanceSq;
                collisionPointV01 = t_collisionPointV01;
                collisionPointV02 = t_collisionPointV02;
            }
        }

        // check for collision between sphere located at vertex 0.
        // if the starting point (a_segmentPointA) is located inside
        // the sphere, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        cVector3d t_p, t_n;
        double t_c;
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
                collisionPointV02 = 0.0;
            }
        }

        // check for collision between sphere located at vertex 1.
        // if the starting point (a_segmentPointA) is located inside
        // the sphere, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        if (cIntersectionSegmentSphere(a_segmentPointA,
                                       a_segmentPointB,
                                       vertex1,
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
                collisionPointV01 = 1.0;
                collisionPointV02 = 0.0;
            }
        }

        // check for collision between sphere located at vertex 2.
        // if the starting point (a_segmentPointA) is located inside
        // the sphere, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        if (cIntersectionSegmentSphere(a_segmentPointA,
                                       a_segmentPointB,
                                       vertex2,
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
                collisionPointV02 = 1.0;
            }
        }

        // check for collision between segment and triangle edge01 shell.
        // if the starting point (a_segmentPointA) is located inside
        // the cylinder, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                                a_segmentPointB,
                                                vertex0,
                                                vertex1,
                                                a_settings.m_collisionRadius,
                                                t_collisionPoint,
                                                t_collisionNormal,
                                                t_collisionPointV01,
                                                t_p,
                                                t_n,
                                                t_c) > 0)
        {
            hit = true;
            t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
            if (t_collisionDistanceSq <= collisionDistanceSq)
            {
                collisionPoint = t_collisionPoint;
                collisionNormal = t_collisionNormal;
                collisionDistanceSq = t_collisionDistanceSq;
                collisionPointV01 = t_collisionPointV01;
                collisionPointV02 = 0.0;
            }
        }

        // check for collision between segment and triangle edge02 shell.
        // if the starting point (a_segmentPointA) is located inside
        // the cylinder, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                                a_segmentPointB,
                                                vertex0,
                                                vertex2,
                                                a_settings.m_collisionRadius,
                                                t_collisionPoint,
                                                t_collisionNormal,
                                                t_collisionPointV02,
                                                t_p,
                                                t_n,
                                                t_c) > 0)
        {
            hit = true;
            t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
            if (t_collisionDistanceSq <= collisionDistanceSq)
            {
                collisionPoint = t_collisionPoint;
                collisionNormal = t_collisionNormal;
                collisionDistanceSq = t_collisionDistanceSq;
                collisionPointV01 = 0.0;
                collisionPointV02 = t_collisionPointV02;
            }
        }

        // check for collision between segment and triangle edge12 shell.
        // if the starting point (a_segmentPointA) is located inside
        // the cylinder, we ignore the collision to avoid remaining
        // stuck inside the triangle.
        if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                                a_segmentPointB,
                                                vertex1,
                                                vertex2,
                                                a_settings.m_collisionRadius,
                                                t_collisionPoint,
                                                t_collisionNormal,
                                                t_collisionPointV12,
                                                t_p,
                                                t_n,
                                                t_c) > 0)
        {
            hit = true;
            t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
            if (t_collisionDistanceSq <= collisionDistanceSq)
            {
                collisionPoint = t_collisionPoint;
                collisionNormal = t_collisionNormal;
                collisionDistanceSq = t_collisionDistanceSq;
                collisionPointV01 = 1.0 - t_collisionPointV12;
                collisionPointV02 = t_collisionPointV12;
            }
        }
    }

    // report collision
    if (hit)
    {
        // before reporting the new collision, we need to check if
        // the collision settings require us to verify the side of the
        // triangle which has been hit.
        bool hit_confirmed = false;

        if (checkFrontSide && checkBackSide)
        {
            // settings specify that a collision can occur on both sides
            // of the triangle, so the new collision is reported.
            hit_confirmed = true;
        }
        else
        {
            // we need check on which side of the triangle the collision occurred
            // and see if it needs to be reported.
            cVector3d segmentAB;
            a_segmentPointB.subr(a_segmentPointA, segmentAB);

            cVector3d v01, v02, triangleNormal;
            vertex2.subr(vertex0, v02);
            vertex1.subr(vertex0, v01);
            v01.crossr(v02, triangleNormal);

            double value = cCosAngle(segmentAB, triangleNormal);
            if (value <= 0.0)
            {
                if (checkFrontSide)
                    hit_confirmed = true;
            }
            else
            {
                if (checkBackSide)
                    hit_confirmed = true;
            }
        }

        // here we finally report the new collision to the collision event handler.
        if (hit_confirmed)
        {
            // we verify if anew collision needs to be created or if we simply
            // need to update the nearest collision.
            if (a_settings.m_checkForNearestCollisionOnly)
            {
                // no new collision event is create. We just check if we need
                // to update the nearest collision
                if(collisionDistanceSq <= a_recorder.m_nearestCollision.m_squareDistance)
                {
                    // report basic collision data
                    a_recorder.m_nearestCollision.m_type = C_COL_TRIANGLE;
                    a_recorder.m_nearestCollision.m_object = a_object;
                    a_recorder.m_nearestCollision.m_triangles = ((cMesh*)(a_object))->m_triangles;
                    a_recorder.m_nearestCollision.m_index = a_elementIndex;
                    a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                    a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                    a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                    a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;
                    a_recorder.m_nearestCollision.m_posV01 = collisionPointV01;
                    a_recorder.m_nearestCollision.m_posV02 = collisionPointV02;

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
                newCollisionEvent.m_type = C_COL_TRIANGLE;
                newCollisionEvent.m_object = a_object;
                newCollisionEvent.m_triangles = ((cMesh*)(a_object))->m_triangles;
                newCollisionEvent.m_index = a_elementIndex;
                newCollisionEvent.m_localPos = collisionPoint;
                newCollisionEvent.m_localNormal = collisionNormal;
                newCollisionEvent.m_squareDistance = collisionDistanceSq;
                newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;
                newCollisionEvent.m_posV01 = collisionPointV01;
                newCollisionEvent.m_posV02 = collisionPointV02;

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

        // return result
        return (hit_confirmed);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method copies all allocated triangles. Please note that this method
    does copy triangles that were previously deallocated.

    \return  New triangle array.
*/
//==============================================================================
cTriangleArrayPtr cTriangleArray::copy()
{
    // create new array of triangles
    cTriangleArrayPtr triangleArray = cTriangleArray::create(m_vertices);

    // get number of allocated triangles
    unsigned int numTriangles = getNumElements();

    // copy every allocated triangle
    for (unsigned int i=0; i<numTriangles; i++)
    {
        if (getAllocated(i))
        {
            unsigned int index0 = getVertexIndex0(i);
            unsigned int index1 = getVertexIndex1(i);
            unsigned int index2 = getVertexIndex2(i);

            triangleArray->newTriangle(index0, index1, index2);
        }
    }

    // return new triangle array
    return (triangleArray);
}


//==============================================================================
/*!
    This method compressed the triangle array by removing all non allocated 
    triangles. If many triangles are unused, this method can effectively reduce 
    the memory footprint allocated by the array.\n

    __IMPORTANT:__ \n
    After calling this method, it is important to immediately update any
    collision detector as the collision tree may try to access triangles
    that no longer exist.\n
*/
//==============================================================================
void cTriangleArray::compress()
{
    // get number of allocated triangles
    unsigned int numTriangles = getNumElements();

    // remove non allocated triangles
    unsigned int j= (unsigned int)(-1);
    for (unsigned int i=0; i<numTriangles; i++)
    {
        if (getAllocated(i))
        {
            j++;

            if (i!=j)
            {
                unsigned int index0 = getVertexIndex0(i);
                unsigned int index1 = getVertexIndex1(i);
                unsigned int index2 = getVertexIndex2(i);
                m_allocated[j] = true;
                setVertices(j, index0, index1, index2);
            }
        }
    }

    // resize arrays
    unsigned size = j+1;
    m_allocated.resize(size);
    m_indices.resize(3*size);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
