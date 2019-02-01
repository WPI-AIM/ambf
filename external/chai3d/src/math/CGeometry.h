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
    \version   3.2.0 $Rev: 2171 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGeometryH
#define CGeometryH
//------------------------------------------------------------------------------
#include "math/CPolySolver.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGeometry.h
    \ingroup    math

    \brief
    Implements general geometry utility functions.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup math
*/
//------------------------------------------------------------------------------

//@{

//==============================================================================
/*!
    \brief
    This function computes the area of a triangle.

    \details
    This function computes the area of a triangle described by the position of 
    its three vertices \p a_vertex0, \p a_vertex1, and \p a_vertex2.

    \param  a_vertex0  Vertex 0 of triangle.
    \param  a_vertex1  Vertex 1 of triangle.
    \param  a_vertex2  Vertex 2 of triangle.

    \return The area of the triangle.
*/
//==============================================================================
inline double cTriangleArea(const cVector3d& a_vertex0,
                            const cVector3d& a_vertex1,
                            const cVector3d& a_vertex2)
{
    cVector3d u, v;
    a_vertex1.subr(a_vertex0, u);
    a_vertex2.subr(a_vertex0, v);
    return (0.5 * (cCross(u,v).length()));
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a plane.

    \details
    This function computes the projection of a point \p a_point onto a plane.
    The plane is expressed by a point \p a_planePoint and a surface normal 
    \p a_planeNormal.

    \param  a_point        Point to be projected on the plane.
    \param  a_planePoint   Point on the plane.
    \param  a_planeNormal  Surface normal of the plane.

    \return The projection of the point onto the plane.
*/
//==============================================================================
inline cVector3d cProjectPointOnPlane(const cVector3d& a_point,
                                      const cVector3d& a_planePoint,
                                      const cVector3d& a_planeNormal)
{
    cVector3d n = a_planeNormal;

    // compute a projection matrix
    cMatrix3d projectionMatrix;
    projectionMatrix.set(
       (n(1) * n(1)) + (n(2) * n(2)), -(n(0) * n(1) ),-(n(0) * n(2)),
      -(n(1) * n(0)), (n(0) * n(0)) + (n(2) * n(2) ),-(n(1) * n(2)),
      -(n(2) * n(0)),-(n(2) * n(1)), (n(0) * n(0) ) + (n(1) * n(1))
      );

    // project point on plane and return projected point.
    cVector3d point;
    a_point.subr(a_planePoint, point);
    projectionMatrix.mul(point);
    point.add(a_planePoint);

    // return result
    return (point);
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a plane.

    \details
    This function computes the projection of a point onto a plane. The plane is 
    expressed by a set of three points passed by argument.

    \param  a_point        Point to be projected onto the plane.
    \param  a_planePoint0  Point 0 of plane.
    \param  a_planePoint1  Point 1 of plane.
    \param  a_planePoint2  Point 2 of plane.

    \return The projection of the point onto the plane.
*/
//==============================================================================
inline cVector3d cProjectPointOnPlane(const cVector3d& a_point,
                                      const cVector3d& a_planePoint0,
                                      const cVector3d& a_planePoint1,
                                      const cVector3d& a_planePoint2)
{
    // create two vectors from the three input points lying in the projection plane.
    cVector3d v01, v02;
    a_planePoint1.subr(a_planePoint0, v01);
    a_planePoint2.subr(a_planePoint0, v02);

    // compute the normal vector of the plane
    cVector3d n;
    v01.crossr(v02, n);
    n.normalize();

    // return projected point
    return (cProjectPointOnPlane(a_point,a_planePoint0,n));
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a plane.

    \details
    This function computes the projection of a point \p a_point onto a plane 
    described by three points \p a_planePoint0, \p a_planePoint1, 
    and \p a_planePoint2.

    The function returns two parameters \p a_r01 and \p a_r01 which describe
    the position of the projected point onto the plane. \n

    \code
    projectedPoint = a_planePoint0 + a_r01 * V01 + a_r02 * V02
    \endcode

    where: \n
    \code
    V01 = a_planePoint1 - a_planePoint0
    V02 = a_planePoint2 - a_planePoint0
    \endcode

    \param  a_point        Point to be projected onto plane.
    \param  a_planePoint0  Point 0 of plane.
    \param  a_planePoint1  Point 1 of plane.
    \param  a_planePoint2  Point 2 of plane.
    \param  a_r01          Returned relative position factor v01.
    \param  a_r02          Returned relative position factor v02.
*/
//==============================================================================
inline void cProjectPointOnPlane(const cVector3d& a_point,
                                 const cVector3d& a_planePoint0,
                                 const cVector3d& a_planePoint1,
                                 const cVector3d& a_planePoint2,
                                 double& a_r01,
                                 double& a_r02)
{
    cVector3d v01 = cSub(a_planePoint1, a_planePoint0);
    cVector3d v02 = cSub(a_planePoint2, a_planePoint0);
    cVector3d point = cSub(a_point, a_planePoint0);

    // matrix
    double m00 = v01(0) * v01(0) + v01(1) * v01(1) + v01(2) * v01(2);
    double m01 = v01(0) * v02(0) + v01(1) * v02(1) + v01(2) * v02(2);
    double m10 = m01;
    double m11 = v02(0) * v02(0) + v02(1) * v02(1) + v02(2) * v02(2);
    double det = m00 * m11 - m10 * m01;

    // vector
    double vm0 = v01(0) * point(0) + v01(1) * point(1) + v01(2) * point(2);
    double vm1 = v02(0) * point(0) + v02(1) * point(1) + v02(2) * point(2);

    // inverse
    a_r01 = (1.0 / det) * ( m11 * vm0 - m01 * vm1);
    a_r02 = (1.0 / det) * (-m10 * vm0 + m00 * vm1);
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a line.

    \details
    This function computes the projection of a point \p a_point on a line. 
    The line is expressed by a point \p a_pointOnLine located on the line and 
    a direction vector \p a_directionOfLine.

    \param  a_point            Point to be projected onto the line.
    \param  a_pointOnLine      Point located on the line.
    \param  a_directionOfLine  Direction vector of the line.

    \return The projection of the point onto the line.
*/
//==============================================================================
inline cVector3d cProjectPointOnLine(const cVector3d& a_point,
                                     const cVector3d& a_pointOnLine,
                                     const cVector3d& a_directionOfLine)
{
    // temp variable
    cVector3d point, result;

    // compute norm of line direction
    double lengthDirSq = a_directionOfLine.lengthsq();

    if (lengthDirSq > 0.0)
    {
        a_point.subr(a_pointOnLine, point);

        a_directionOfLine.mulr( (point.dot(a_directionOfLine) / (lengthDirSq)),
                                result);

        result.add(a_pointOnLine);
        return (result);
    }
    else
    {
        return (a_pointOnLine);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a segment.

    \details
    This function computes the projection of a point \p a_point onto a segment.
    The segment is described by its two extremity points \p a_segmentPointA 
    and \p a_segmentPointB.

    \param  a_point          Point to be projected.
    \param  a_segmentPointA  Point A of segment.
    \param  a_segmentPointB  Point B of segment.

    \return The projection of the point onto the segment.
*/
//==============================================================================
inline cVector3d cProjectPointOnSegment(const cVector3d& a_point,
                                        const cVector3d& a_segmentPointA,
                                        const cVector3d& a_segmentPointB)
{
    // if both points are equal
    if (a_segmentPointA.equals(a_segmentPointB))
    {
        return (a_segmentPointA);
    }

    // compute line
    cVector3d segmentAB;
    a_segmentPointB.subr(a_segmentPointA, segmentAB);

    // project tool onto segment
    cVector3d projection = cProjectPointOnLine(a_point, a_segmentPointA, segmentAB);

    double distanceAB = segmentAB.lengthsq();
    double distanceAP = cDistanceSq(projection, a_segmentPointA);
    double distanceBP = cDistanceSq(projection, a_segmentPointB);

    if (distanceAP > distanceAB)
    {
        return(a_segmentPointB);
    }
    else
    if (distanceBP > distanceAB)
    {
        return(a_segmentPointA);
    }
    else
    {
        return(projection);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a disk.

    \details
    This function computes the projection of a point onto a disk lying in 
    the XY plane. \n

    The position of the disk along the z-axis is expressed by parameter 
    \p a_height. \n

    The radius of the disk is expressed by parameter \p a_radius.

    \param  a_point   Point to be projected.
    \param  a_height  Position of the disk along the z-axis.
    \param  a_radius  Radius of the disk.

    \return The projection of the point onto the disk.
*/
//==============================================================================
inline cVector3d cProjectPointOnDiskXY (const cVector3d& a_point,
                                        const double& a_height,
                                        const double& a_radius)
{
    cVector3d center(0.0, 0.0, a_height);
    cVector3d pointProj(a_point);

    pointProj(2)  = a_height;

    cVector3d ray = cSub (pointProj, center);

    if (ray.length() < a_radius) return (pointProj);
    else
    {
        ray.normalize();
        return center + a_radius * ray;
    }
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a point onto a triangle.

    \details
    This function computes the projection of a point onto a triangle
    expressed by its three vertices.

    \param  a_point    Point to be projected.
    \param  a_vertex0  Triangle vertex 0.
    \param  a_vertex1  Triangle vertex 1.
    \param  a_vertex2  Triangle vertex 2.

    \return The projection of the point onto the triangle.
*/
//==============================================================================
inline cVector3d cProjectPointOnTriangle (const cVector3d& a_point,
                                          const cVector3d& a_vertex0,
                                          const cVector3d& a_vertex1,
                                          const cVector3d& a_vertex2)
{
    cVector3d result;

    // project point on plane
    double r01, r02;
    cProjectPointOnPlane(a_point, a_vertex0, a_vertex1, a_vertex2, r01, r02);

    // check if point in located inside triangle
    if ((r01 >= 0.0) && (r02 >= 0.0) && (r01 <= 1.0) && (r02 <= 1.0) && ((r01 + r02) <= 1.0))
    {
        result = a_vertex0 + r01 * (a_vertex1 - a_vertex0) + r02 * (a_vertex2 - a_vertex0);
    }
    else
    {
        // project point on edge 01
        cVector3d p01 = cProjectPointOnSegment(a_point, a_vertex0, a_vertex1);

        // project point on edge 02
        cVector3d p02 = cProjectPointOnSegment(a_point, a_vertex0, a_vertex2);

        // project point on edge 12
        cVector3d p12 = cProjectPointOnSegment(a_point, a_vertex1, a_vertex2);

        // check for nearest point
        double d01 = cDistanceSq(a_point, p01);
        double d02 = cDistanceSq(a_point, p02);
        double d12 = cDistanceSq(a_point, p12);

        if (d01 < d02)
        {
            if (d01 < d12)
            {
                result = p01;
            }
            else
            {
                result = p12;
            }
        }
        else
        {
            if (d02 < d12)
            {
                result = p02;
            }
            else
            {
                result = p12;
            }
        }
    }

    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the projection of a vector \e V0 onto a vector \e V1.

    \details
    This function computes the projection of a vector \e V0 onto a vector \e V1,
    where \e V0 and \e V1 are passed by argument as \p a_vector0 and
    \p a_vector1.

    \param  a_vector0  Vector \e V0.
    \param  a_vector1  Vector \e V1.

    \return The projection of vector \e V0 onto vector \e V1.
*/
//==============================================================================
inline cVector3d cProject(const cVector3d& a_vector0, 
                          const cVector3d& a_vector1)
{
    cVector3d result;

    // compute projection
    double lengthSq = a_vector1.lengthsq();
    a_vector1.mulr( (a_vector0.dot(a_vector1) / (lengthSq)), result);

    // return result
    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the distance between a point and a line.

    \details
    This function computes the distance between a point and a line. The line is 
    described by two points \p a_linePoint1 and \p a_linePoint2 located along 
    the line.

    \param  a_point       Point to be tested.
    \param  a_linePoint1  Point 1 on line.
    \param  a_linePoint2  Point 2 on line.

    \return The distance between the point and the line.
*/
//==============================================================================
inline double cDistanceToLine(const cVector3d& a_point, 
                              const cVector3d& a_linePoint1,
                              const cVector3d& a_linePoint2)
{
    cVector3d point = cProjectPointOnLine(a_point, a_linePoint1, cSub(a_linePoint2, a_linePoint1));
    return (cDistance(a_point, point));
}


//==============================================================================
/*!
    \brief
    This function computes the surface normal of a plane.

    \details
    This function computes the surface normal of a plane defined by three points 
    passed by argument.

    \param  a_surfacePoint0  Point 0 on surface.
    \param  a_surfacePoint1  Point 1 on surface.
    \param  a_surfacePoint2  Point 2 on surface.

    \return The surface normal.
*/
//==============================================================================
inline cVector3d cComputeSurfaceNormal(const cVector3d& a_surfacePoint0,
                                       const cVector3d& a_surfacePoint1, 
                                       const cVector3d& a_surfacePoint2)
{
    cVector3d v01, v02, result;

    // compute surface normal
    a_surfacePoint1.subr(a_surfacePoint0, v01);
    a_surfacePoint2.subr(a_surfacePoint0, v02);
    v01.normalize();
    v02.normalize();
    v01.crossr(v02, result);
    result.normalize();

    // return result
    return (result);
}


//==============================================================================
/*!
    \brief
    This function checks if a point is contained in a box.

    \details
    This function check if a point is contained in a box. The box is described 
    by its minimum and maximum corners \p a_boxMin and \p a_boxMax. \n
    
    If the point is located inside  the box, then then function returns __true__, 
    otherwise it returns __false__.

    \param  a_point   Test point.
    \param  a_boxMin  Minimum coordinate of the box.
    \param  a_boxMax  Maximum coordinate of the box.

    \return __true__ if the point is contained in the box, __false__ otherwise.
*/
//==============================================================================
inline bool cBoxContains(const cVector3d& a_point,
                         const cVector3d& a_boxMin,
                         const cVector3d& a_boxMax)
{
    if ((a_point(0)  >= a_boxMin(0) ) && (a_point(0)  <= a_boxMax(0)) &&
        (a_point(1)  >= a_boxMin(1) ) && (a_point(1)  <= a_boxMax(1)) &&
        (a_point(2)  >= a_boxMin(2) ) && (a_point(2)  <= a_boxMax(2)))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a plane.

    \details
    This function computes the intersection between a segment and a plane. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n
    
    The plane is described by a point \p a_planePos and a surface normal
    \p a_planeNormal. \n
    
    The function returns the number of collision points that were 
    detected (0 or 1). \n
    
    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal.

    \param  a_segmentPointA    First point of segment AB.
    \param  a_segmentPointB    Second point of segment AB.
    \param  a_planePos         Position of a point on the plane.
    \param  a_planeNormal      Surface normal of the plane.
    \param  a_collisionPoint   Returned intersection point (if detected).
    \param  a_collisionNormal  Returned surface normal at intersection point (if detected).

    \return The number of intersection points found (0 or 1).
*/
//==============================================================================
inline int cIntersectionSegmentPlane(const cVector3d& a_segmentPointA,
                                     const cVector3d& a_segmentPointB,
                                     const cVector3d& a_planePos,
                                     const cVector3d& a_planeNormal,
                                     cVector3d& a_collisionPoint,
                                     cVector3d& a_collisionNormal)
{
    cVector3d d = a_segmentPointB - a_segmentPointA;
    cVector3d p = a_segmentPointA;
    double length = d.length();

    // sanity check
    if (cZero(length)) 
    { 
        return (0);
    }

    // normalize d
    d.mul(1.0 / length);

    // compute intersection between segment and disk plane
    double c[2];
    double s[1];

    c[0] = a_planeNormal(0)*p(0) - a_planeNormal(0)*a_planePos(0) +
           a_planeNormal(1)*p(1) - a_planeNormal(1)*a_planePos(1) +
           a_planeNormal(2)*p(2) - a_planeNormal(2)*a_planePos(2);
    c[1] = a_planeNormal(0)*d(0) + a_planeNormal(1)*d(1) + a_planeNormal(2)*d(2);

    int num = cSolveLinear(c, s);

    if (num == 0)
    {
        return (0);
    }
    else
    {
        if ((s[0] >= 0.0) && (s[0] <= length))
        {
            a_collisionPoint = p + s[0] * d;
            if (cDot(a_planeNormal, cSub(a_segmentPointA, a_collisionPoint)) >= 0.0)
            {
                a_collisionNormal = a_planeNormal;
            }
            else
            {
                a_collisionNormal =-a_planeNormal;
            }
            return (1);
        }
        else
        {
            return (0);
        }
    }
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a disk.

    \details
    This function computes the intersection between a segment and a disk. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n
    
    The disk is described by a point \p a_diskPos, a surface normal
    \p a_diskNormal, and a radius \p a_diskRadius. \n
    
    The function returns the number of collision points that were 
    detected (0 or 1). \n
    
    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal.

    \param  a_segmentPointA    First point of segment AB.
    \param  a_segmentPointB    Second point of segment AB.
    \param  a_diskPos          Position of the disk center.
    \param  a_diskNormal       Surface normal of the disk.
    \param  a_diskRadius       Radius of the disk.
    \param  a_collisionPoint   Returned intersection point (if detected).
    \param  a_collisionNormal  Returned surface normal at intersection point (if detected).

    \return The number of intersection points found (0 or 1).
*/
//==============================================================================
inline int cIntersectionSegmentDisk(const cVector3d& a_segmentPointA,
                                    const cVector3d& a_segmentPointB,
                                    const cVector3d& a_diskPos,
                                    const cVector3d& a_diskNormal,
                                    const double& a_diskRadius,
                                    cVector3d& a_collisionPoint,
                                    cVector3d& a_collisionNormal)
{
    // compute intersection between segment and disk plane
    if (cIntersectionSegmentPlane(a_segmentPointA,
                                  a_segmentPointB,
                                  a_diskPos,
                                  a_diskNormal,
                                  a_collisionPoint,
                                  a_collisionNormal) == 0)
    {
        return (0);
    }

    // check if intersection point within disk
    if (cDistance(a_collisionPoint, a_diskPos) <= a_diskRadius)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a sphere.

    \details
    This function computes the intersection between a segment and a sphere. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The sphere is described by its position \p a_spherePos and a radius \p a_sphereRadius. \n

    The function returns the number of collision points that were 
    detected (0, 1, or 2). \n

    If a collision occurs, the collision point(s) and collision normal(s) are 
    returned by arguments \p a_collisionPoint0, \p a_collisionNormal0, 
    \p a_collisionPoint1, and \p a_collisionNormal1. \n

    If two intersection points are detected, \e collisionPoint0 is the nearest point
    to \e segmentPointA of segment AB. \n

    If the segment begins inside the sphere, then all intersections are ignored.

    \param  a_segmentPointA     First point of segment AB.
    \param  a_segmentPointB     Second point of segment AB.
    \param  a_spherePos         Position of sphere center.
    \param  a_sphereRadius      Radius of sphere.
    \param  a_collisionPoint0   Returned intersection point 0 (if detected).
    \param  a_collisionNormal0  Returned surface normal 0 at intersection point (if detected).
    \param  a_collisionPoint1   Returned intersection point 1 (if detected).
    \param  a_collisionNormal1  Returned surface normal 1 at intersection point (if detected).

    \return The number of intersection points found (0, 1, or 2).
*/
//==============================================================================
inline int cIntersectionSegmentSphere(const cVector3d& a_segmentPointA,
                                      const cVector3d& a_segmentPointB,
                                      const cVector3d& a_spherePos,
                                      const double& a_sphereRadius,
                                      cVector3d& a_collisionPoint0,
                                      cVector3d& a_collisionNormal0,
                                      cVector3d& a_collisionPoint1,
                                      cVector3d& a_collisionNormal1)
{
    // temp variables
    cVector3d AB, CA;
    a_segmentPointB.subr(a_segmentPointA, AB);
    a_segmentPointA.subr(a_spherePos, CA);
    double radiusSq = a_sphereRadius * a_sphereRadius;

    double a = AB.lengthsq();
    double b = 2.0 * cDot(AB, CA);
    double c = CA.lengthsq() - radiusSq;

    // invalid segment
    if (a == 0)
    {
        return (0);
    }

    double d = b*b - 4*a*c;

    // segment ray is located outside of sphere
    if (d < 0)
    {
        return (0);
    }

    // segment ray intersects sphere
    d = sqrt(d);
    double e = 2.0 * a;

    // compute both solutions
    double u0 = (-b + d) / e;
    double u1 = (-b - d) / e;

    // check if the solutions are located along the segment AB
    bool valid_u0 = cContains(u0, 0.0, 1.0);
    bool valid_u1 = cContains(u1, 0.0, 1.0);

    // two intersection points are located along segment AB
    if (valid_u0 && valid_u1)
    {
        if (u0 > u1) { cSwap(u0, u1); }

        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // compute point 1
        AB.mulr(u1, a_collisionPoint1);
        a_collisionPoint1.add(a_segmentPointA);

        a_collisionPoint1.subr(a_spherePos, a_collisionNormal1);
        a_collisionNormal1.normalize();

        return (2);
    }

    // one intersection point is located along segment AB
    else if (valid_u0)
    {
        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // one intersection point is located along segment AB
    else if (valid_u1)
    {
        // compute point 0
        AB.mulr(u1, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // both points are located outside of the segment AB
    else
    {
        return (0);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a sphere.

    \details
    This function computes the intersection between a segment and a sphere. \n

    The segment is described by two points \p a_segmentPointA
    and \p a_segmentPointB. \n

    The sphere is described by its position \p a_spherePos and a radius \p a_sphereRadius. \n

    The function returns the number of collision points that were
    detected (0, 1, or 2). \n

    If a collision occurs, the collision point(s) and collision normal(s) are
    returned by arguments \p a_collisionPoint0, \p a_collisionNormal0,
    \p a_collisionPoint1, and \p a_collisionNormal1. \n

    If two intersection points are detected, \e collisionPoint0 is the nearest point
    to \e segmentPointA of segment AB. \n

    If the segment begins inside the sphere, then all intersections are ignored.

    \param  a_segmentPointA     First point of segment AB.
    \param  a_segmentPointB     Second point of segment AB.
    \param  a_pos               Position of ellipsoid center.
    \param  a_radiusX           Ellipsoid radius along __x__ axis.
    \param  a_radiusY           Ellipsoid radius along __y__ axis.
    \param  a_radiusZ           Ellipsoid radius along __z__ axis.
    \param  a_collisionPoint0   Returned intersection point 0 (if detected).
    \param  a_collisionNormal0  Returned surface normal 0 at intersection point (if detected).
    \param  a_collisionPoint1   Returned intersection point 1 (if detected).
    \param  a_collisionNormal1  Returned surface normal 1 at intersection point (if detected).

    \return The number of intersection points found (0, 1, or 2).
*/
//==============================================================================
inline int cIntersectionSegmentEllipsoid(const cVector3d& a_segmentPointA,
    const cVector3d& a_segmentPointB,
    const cVector3d& a_pos,
    const double& a_radiusX,
    const double& a_radiusY,
    const double& a_radiusZ,
    cVector3d& a_collisionPoint0,
    cVector3d& a_collisionNormal0,
    cVector3d& a_collisionPoint1,
    cVector3d& a_collisionNormal1)
{
    // scale to sphere
    double radius = cMin(a_radiusX, cMin(a_radiusY, a_radiusZ));
    double scaleX = a_radiusX / radius;
    double scaleY = a_radiusY / radius;
    double scaleZ = a_radiusZ / radius;

    cVector3d segmentPointA = a_segmentPointA;
    segmentPointA.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);
    cVector3d segmentPointB = a_segmentPointB;
    segmentPointB.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);

    // temp variables
    cVector3d AB, CA;
    segmentPointB.subr(segmentPointA, AB);
    segmentPointA.subr(a_pos, CA);
    double radiusSq = radius * radius;

    double a = AB.lengthsq();
    double b = 2.0 * cDot(AB, CA);
    double c = CA.lengthsq() - radiusSq;

    // invalid segment
    if (a == 0)
    {
        return (0);
    }

    double d = b*b - 4 * a*c;

    // segment ray is located outside of sphere
    if (d < 0)
    {
        return (0);
    }

    // segment ray intersects sphere
    d = sqrt(d);
    double e = 2.0 * a;

    // compute both solutions
    double u0 = (-b + d) / e;
    double u1 = (-b - d) / e;

    // check if the solutions are located along the segment AB
    bool valid_u0 = cContains(u0, 0.0, 1.0);
    bool valid_u1 = cContains(u1, 0.0, 1.0);

    // two intersection points are located along segment AB
    if (valid_u0 && valid_u1)
    {
        if (u0 > u1) { cSwap(u0, u1); }

        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(segmentPointA);
        a_collisionPoint0.subr(a_pos, a_collisionNormal0);
        a_collisionNormal0.normalize();
        a_collisionNormal0.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);
        a_collisionPoint0.mul(scaleX, scaleY, scaleZ);

        // compute point 1
        AB.mulr(u1, a_collisionPoint1);
        a_collisionPoint1.add(segmentPointA);
        a_collisionPoint1.subr(a_pos, a_collisionNormal1);
        a_collisionNormal1.normalize();
        a_collisionNormal1.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);
        a_collisionPoint1.mul(scaleX, scaleY, scaleZ);

        return (2);
    }

    // one intersection point is located along segment AB
    else if (valid_u0)
    {
        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(segmentPointA);
        a_collisionPoint0.subr(a_pos, a_collisionNormal0);
        a_collisionNormal0.normalize();
        a_collisionNormal0.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);
        a_collisionPoint0.mul(scaleX, scaleY, scaleZ);

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // one intersection point is located along segment AB
    else if (valid_u1)
    {
        // compute point 0
        AB.mulr(u1, a_collisionPoint0);
        a_collisionPoint0.add(segmentPointA);
        a_collisionPoint0.subr(a_pos, a_collisionNormal0);
        a_collisionNormal0.normalize();
        a_collisionNormal0.mul(1.0 / scaleX, 1.0 / scaleY, 1.0 / scaleZ);
        a_collisionPoint0.mul(scaleX, scaleY, scaleZ);

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // both points are located outside of the segment AB
    else
    {
        return (0);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a topless cylinder.

    \details
    This function computes the intersection between a segment and a topless cylinder.

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The cylinder is described by two extremity points \p a_cylinderPointA and \p a_cylinderPointB,
    and a radius \p a_cylinderRadius. \n

    The function returns the number of collision points that were 
    detected (0, 1, or 2). \n

    If a collision occurs, the collision point(s) and collision normal(s) are 
    returned by arguments \p a_collisionPoint0, \p \p a_collisionPoint1,
    \p a_collisionNormal0 and \p a_collisionNormal1. \n

    If two intersection points occur, \e collisionPoint0 is the nearest point
    to \e segmentPointA of segment AB. \n

    If the segment begins inside the cylinder, then all intersections are ignored.

    \param  a_segmentPointA            First point of segment AB.
    \param  a_segmentPointB            Second point of segment AB.
    \param  a_cylinderPointA           Extremity point A of cylinder.
    \param  a_cylinderPointB           Extremity point B of cylinder.
    \param  a_cylinderRadius           Radius of cylinder.
    \param  a_collisionPoint0          Returned intersection point 0 (if detected).
    \param  a_collisionNormal0         Returned surface normal 0 at intersection point (if detected).
    \param  a_collisionPointRelPosAB0  Returned relative position of collision point 0 projected on segment \p a_segmentPointA - \p a_segmentPointB
    \param  a_collisionPoint1          Returned intersection point 1 (if detected).
    \param  a_collisionNormal1         Returned surface normal 1 at intersection point (if detected).
    \param  a_collisionPointRelPosAB1  Returned relative position of collision point 1 projected on segment \p a_segmentPointA - \p a_segmentPointB

    \return The number of intersection points found (0, 1, or 2).
*/
//==============================================================================
inline int cIntersectionSegmentToplessCylinder(const cVector3d& a_segmentPointA,
                                               const cVector3d& a_segmentPointB,
                                               const cVector3d& a_cylinderPointA,
                                               const cVector3d& a_cylinderPointB,
                                               const double& a_cylinderRadius,
                                               cVector3d& a_collisionPoint0,
                                               cVector3d& a_collisionNormal0,
                                               double& a_collisionPointRelPosAB0,
                                               cVector3d& a_collisionPoint1,
                                               cVector3d& a_collisionNormal1,
                                               double& a_collisionPointRelPosAB1)
{
    // compute some initial values
    cVector3d cylinderAB = a_cylinderPointB - a_cylinderPointA;
    double lengthCylinderAB = cylinderAB.length();

    // sanity check
    if (lengthCylinderAB == 0.0) { return (0); }

    // compute more initial values
    cVector3d cylinderDir = cNormalize(cylinderAB);
    cVector3d RC = a_segmentPointA - a_cylinderPointA;
    cVector3d segmentAB = a_segmentPointB - a_segmentPointA;
    if (segmentAB.lengthsq() == 0) { return (0); }

    cVector3d segmentDir = cNormalize(segmentAB);
    cVector3d n = cCross(segmentDir, cylinderDir);

    // segment is parallel to cylinder
    double length = n.length();
    if (length == 0.0)
    {
        return (0);
    }

    n.normalize();
    double d = fabs (cDot (RC,n));

    if (d <= a_cylinderRadius)
    {
        cVector3d O = cCross(RC, cylinderDir);
        double t = -cDot(O,n) / length;
        O = cCross(n, cylinderDir);
        O.normalize();
        double s = fabs(sqrt(a_cylinderRadius*a_cylinderRadius - d*d) / cDot(segmentDir,O));
        double u0 = t - s;
        double u1 = t + s;

        // reorder solutions
        if (u0 > u1) { cSwap (u0,u1); }

        bool valid_u0 = true;
        bool valid_u1 = true;

        // check if solutions along segment
        double lengthAB = segmentAB.length();
        if (!cContains(u0, 0.0, lengthAB)) { valid_u0 = false; }
        if (!cContains(u1, 0.0, lengthAB)) { valid_u1 = false; }

        // check if solutions lay along cylinder
        cVector3d P0, P1;
        cVector3d cylinderDirNeg = cNegate(cylinderDir);
        
        if (valid_u0)
        {
            P0 = a_segmentPointA + u0 * segmentDir;
            cVector3d P0A = cSub(P0, a_cylinderPointA);
            cVector3d P0B = cSub(P0, a_cylinderPointB);

            double cosAngleA = cCosAngle(cylinderDir, P0A);
            double cosAngleB = cCosAngle(cylinderDirNeg, P0B);

            if ((cosAngleA <= 0.0) || (cosAngleB <= 0.0))
            {
                valid_u0 = false;
            }
            else
            {
                a_collisionPointRelPosAB0 = cosAngleA * (P0A.length() / lengthCylinderAB);
            }
        }

        if (valid_u1)
        {
            P1 = a_segmentPointA + u1 * segmentDir;
            cVector3d P1A = cSub(P1, a_cylinderPointA);
            cVector3d P1B = cSub(P1, a_cylinderPointB);

            double cosAngleA = cCosAngle(cylinderDir, P1A);
            double cosAngleB = cCosAngle(cylinderDirNeg, P1B);

            if ((cosAngleA <= 0.0) || (cosAngleB <= 0.0))
            {
                valid_u1 = false;
            }
            else
            {
                a_collisionPointRelPosAB1 = cosAngleA * (P1A.length() / lengthCylinderAB);
            }
        }

        if (valid_u0 && valid_u1)
        {
            a_collisionPoint0 = P0;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P0, a_cylinderPointA), cylinderDir)));
            a_collisionPoint1 = P1;
            a_collisionNormal1 = cNormalize(cCross(cylinderDir, cCross(cSub(P1, a_cylinderPointA), cylinderDir)));
            return (2);
        }
        else if (valid_u0)
        {
            a_collisionPoint0 = P0;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P0, a_cylinderPointA), cylinderDir)));

            // check dot product to see if the intial segment point is located inside the cylinder.
            double dotProduct = cDot(segmentAB, a_collisionNormal0);
            if (dotProduct < 0.0)
            {
                return (1);
            }
            else
            {
                return (0);
            }
        }
        else if (valid_u1)
        {
            a_collisionPoint0 = P1;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P1, a_cylinderPointA), cylinderDir)));

            // check dot product to see if the intial segment point is located inside the cylinder.
            double dotProduct = cDot(segmentAB, a_collisionNormal0);
            if (dotProduct < 0.0)
            {
                return (1);
            }
            else
            {
                return (0);
            }
        }
    }

    return (0);
}


//==============================================================================
/*!
    \brief
    This function computes the nearest intersection point between a segment and a cylinder.

    \details
    This function computes the nearest intersection point between a segment and a 
    cylinder. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The cylinder is aligned along the z-axis and is described by its 
    height \p a_height, base radius \p a_baseRadius, and a top radius \p a_topRadius. \n

    The function returns the number of collision points that were 
    detected (0 or 1). \n

    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal. \n

    If the segment begins inside the cylinder, then all intersections are ignored.

    \param  a_segmentPointA    First point of segment AB.
    \param  a_segmentPointB    Second point of segment AB.
    \param  a_baseRadius       Radius of cylinder a z = 0.0.
    \param  a_topRadius        Radius of cylinder a z = a_height.
    \param  a_height           Height of cylinder.
    \param  a_collisionPoint   Returned intersection point (if detected).
    \param  a_collisionNormal  Returned surface normal at intersection point (if detected).

    \return The number of intersection points found (0 or 1).
*/
//==============================================================================
inline int cIntersectionSegmentCylinder(const cVector3d& a_segmentPointA,
                                        const cVector3d& a_segmentPointB,
                                        const double& a_baseRadius,
                                        const double& a_topRadius,
                                        const double& a_height,
                                        cVector3d& a_collisionPoint,
                                        cVector3d& a_collisionNormal)
{
    // initialization
    cVector3d p = a_segmentPointA;
    cVector3d d = a_segmentPointB - a_segmentPointA;
    double length = d.length();

    // sanity check
    if (cZero(length))
    {
        return (0);
    }
    d.mul(1.0 / length);

    // compute collision with cylinder side
    double A = a_baseRadius;
    double B = (a_topRadius - a_baseRadius) / a_height;

    double c[3];
    double s[2];
    
    c[0] =-2.0*A*B*p(2) - cSqr(B*p(2)) - cSqr(A) + cSqr(p(0)) + cSqr(p(1));
    c[1] = 2.0*p(0)*d(0) + 2.0*p(1)*d(1) - 2.0*A*B*d(2) - 2.0*cSqr(B)*p(2)*d(2);
    c[2] = cSqr(d(0)) + cSqr(d(1)) - cSqr(B*d(2));

    int numCol = cSolveQuadric(c, s);

    // compute collision with top cap
    cVector3d collisionPointTop, collisionNormalTop;

    int numColTop = cIntersectionSegmentDisk(a_segmentPointA,
                                             a_segmentPointB,
                                             cVector3d(0.0, 0.0, a_height),
                                             cVector3d(0.0, 0.0, 1.0),
                                             a_topRadius,
                                             collisionPointTop,
                                             collisionNormalTop);

    // compute collision with base cap
    cVector3d collisionPointBase, collisionNormalBase;

    int numColBase = cIntersectionSegmentDisk(a_segmentPointA,
                                              a_segmentPointB,
                                              cVector3d(0.0, 0.0, 0.0),
                                              cVector3d(0.0, 0.0,-1.0),
                                              a_baseRadius,
                                              collisionPointBase,
                                              collisionNormalBase);

    // evaluate solutions
    bool hit = false;
    double distance = C_LARGE;

    // check top cap
    if (numColTop > 0)
    {
        hit = true;
        distance = cDistance(a_segmentPointA, collisionPointTop);
        a_collisionPoint = collisionPointTop;
        a_collisionNormal = cVector3d(0.0, 0.0, 1.0);
    }

    // check base cap
    if (numColBase > 0)
    {
        hit = true;
        double dist = cDistance(a_segmentPointA, collisionPointBase);
        if (dist < distance)
        {
            a_collisionPoint = collisionPointBase;
            a_collisionNormal = cVector3d(0.0, 0.0,-1.0);
            distance = dist;
        }
    }

    // check cylinder solution 1
    if (numCol > 0)
    {
        if ((s[0] >= 0.0) && (s[0] <= length) && (s[0] < distance))
        {
            cVector3d collisionPoint = p + d * s[0];
            if ((collisionPoint(2) > 0.0) && (collisionPoint(2) < a_height))
            {
                cVector3d normal(collisionPoint(0), collisionPoint(1), 0.0);
                normal.normalize();
                normal(2) = (a_baseRadius - a_topRadius) / a_height;
                a_collisionNormal = normal;
                a_collisionPoint = collisionPoint;
                distance = s[0];
                hit = true;
            }
        }
    }

    // check cylinder solution 2
    if (numCol > 1)
    {
        if ((s[1] >= 0.0) && (s[1] <= length) && (s[1] < distance))
        {
            cVector3d collisionPoint = p + d * s[1];
            if ((collisionPoint(2) > 0.0) && (collisionPoint(2) < a_height))
            {
                cVector3d normal(collisionPoint(0), collisionPoint(1), 0.0);
                normal.normalize();
                normal(2) = (a_baseRadius - a_topRadius) / a_height; 
                a_collisionNormal = normal;
                a_collisionPoint = collisionPoint;
                hit = true;
            }
        }
    }

    if (hit)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the nearest intersection point between a segment and a box.

    \details
    This function computes the nearest intersection point between a segment and 
    a box. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The box is aligned with all axes and is described by its minimum and maximum
    corners \p a_boxMin and \p a_boxMax.

    The function returns the number of collision points that were 
    detected (0 or 1). \n

    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal. \n

    \param  a_segmentPointA    First point of segment AB.
    \param  a_segmentPointB    Second point of segment AB.
    \param  a_boxMin           Minimum coordinate of box.
    \param  a_boxMax           Maximum coordinate of box.
    \param  a_collisionPoint   Returned intersection point (if detected).
    \param  a_collisionNormal  Returned surface normal at intersection point (if detected).

    \return The number of intersection points found (0 or 1).
*/
//==============================================================================
inline int cIntersectionSegmentBox(const cVector3d& a_segmentPointA,
                                   const cVector3d& a_segmentPointB,
                                   const cVector3d& a_boxMin,
                                   const cVector3d& a_boxMax,
                                   cVector3d& a_collisionPoint,
                                   cVector3d& a_collisionNormal)
{
    double tmin = 0.0;
    double tmax = C_LARGE;

    cVector3d d = a_segmentPointB - a_segmentPointA;
    double distance = d.length();
    if (distance == 0)
    {
        return(0);
    }
    d.normalize();

    // for all three slabs
    double normalDir = 1.0;
    int normalAxis = 0;

    for (int i = 0; i<3; i++)
    {
        if (fabs(d(i)) < C_SMALL)
        {
            // ray is parallel to slab. No hit if origin not within slab
            if (a_segmentPointA(i) < a_boxMin(i) || a_segmentPointA(i) > a_boxMax(i))
            {
                return (0);
            }
        }
        else
        {
            // compute intersection t value of ray with near and far plane of slab
            double ood = 1.0 / d(i);
            double t1 = (a_boxMin(i) - a_segmentPointA(i)) * ood;
            double t2 = (a_boxMax(i) - a_segmentPointA(i)) * ood;
            int n = 1;

            // make t1 be intersection with near plane, t2 with far plane
            if (t1 > t2)
            {
                cSwap(t1, t2);
                n = -1;
            }

            // compute the intersection of slab intersection intervals
            if (t1 > tmin)
            {
                tmin = t1;
                normalAxis = i;
                normalDir = -n;
            }

            if (t2 < tmax)
            {
                tmax = t2;
            }

            // exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax)
            {
                return (0);
            }
        }
    }

    if (tmin > distance)
    {
        return (0);
    }

    // compute collision point and surface normal
    a_collisionPoint = a_segmentPointA + cMul(tmin, d);
    a_collisionNormal.zero();
    a_collisionNormal(normalAxis) = normalDir;

    return (1);
}


//==============================================================================
/*!
    \brief
    This function computes the nearest intersection point between a segment and a torus.

    \details
    This function computes the nearest intersection point between a segment and 
    a torus centered around the z-axis. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The torus is described by its inner \p a_innerRadius and outer radius
    \p a_outerRadius. \n

    The function returns the number of collision points that were 
    detected (0 or 1). \n

    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal. \n

    \param  a_segmentPointA    First point of segment AB.
    \param  a_segmentPointB    Second point of segment AB.
    \param  a_innerRadius      Inner radius of torus.
    \param  a_outerRadius      Outer radius of torus.
    \param  a_collisionPoint   Returned intersection point (if detected).
    \param  a_collisionNormal  Returned surface normal at intersection point (if detected).

    \return The number of intersection points found (0 or 1).
*/
//==============================================================================
inline int cIntersectionSegmentTorus(const cVector3d& a_segmentPointA,
                                     const cVector3d& a_segmentPointB,
                                     const double& a_innerRadius, 
                                     const double& a_outerRadius,
                                     cVector3d& a_collisionPoint,
                                     cVector3d& a_collisionNormal)
{
    cVector3d collisionPoint, collisionNormal;

    // compute boundary box
    double width = a_outerRadius + a_innerRadius;
    cVector3d boxMin(-width,-width,-a_innerRadius);
    cVector3d boxMax( width, width, a_innerRadius);

    // check collision with boundary box
    if (cIntersectionSegmentBox(a_segmentPointA,
                                a_segmentPointB,
                                boxMin,
                                boxMax,
                                collisionPoint,
                                collisionNormal) == 0)
    {
        return (0);
    }

    cVector3d p = a_segmentPointA;
    cVector3d d = cNormalize(a_segmentPointB - a_segmentPointA);

    double dd = cDot(d, d);
    double pd = cDot(p, d);
    double pp = cDot(p, p);
    double r2 = cSqr(a_innerRadius);
    double R2 = cSqr(a_outerRadius);

    double c[5];
    double s[4];

    c[4] = cSqr(dd);
    c[3] = 4.0 * dd * pd;
    c[2] = 4.0 * cSqr(pd) + 2.0 * dd * (pp - r2 - R2) + 4.0 * R2 * cSqr(d(2));
    c[1] = 4.0 * pd * (pp - r2 - R2) + 8.0 * R2 * p(2) * d(2);
    c[0] = cSqr(pp - r2 - R2) + 4.0 * R2 * cSqr(p(2)) - 4.0 * R2 * r2;

    int num = cSolveQuartic(c, s);

    if (num == 0)
    {
        // no solutions found
        return (0);
    }

    // search for nearest solution (sollision point)
    double sol = C_LARGE;
    for (int i=0; i<num; i++)
    {
        if (s[i] < sol)
        {
            sol = s[i];
        }
    }

    // compute point
    a_collisionPoint = p + sol * d;

    // compute surface normal
    cVector3d pt = cMul(a_outerRadius, cNormalize(cVector3d(a_collisionPoint(0), a_collisionPoint(1), 0.0)));
    a_collisionNormal = cNormalize(a_collisionPoint - pt);

    // return one collision point detected
    return (1);
}


//==============================================================================
/*!
    \brief
    This function computes the intersection between a segment and a triangle.

    \details
    This function computes the intersection between a segment and a triangle 
    defined by three vertices \p a_triangleVertex0, \p a_triangleVertex1, and
    \p a_triangleVertex2. \n

    The segment is described by two points \p a_segmentPointA 
    and \p a_segmentPointB. \n

    The function returns the number of collision points that were 
    detected (0 or 1). \n

    If a collision occurs, the collision point and collision normal are 
    returned by arguments \p a_collisionPoint and \p a_collisionNormal. \n

    \param  a_segmentPointA             First point of segment AB.
    \param  a_segmentPointB             Second point of segment AB.
    \param  a_triangleVertex0           Vertex 0 of triangle.
    \param  a_triangleVertex1           Vertex 1 of triangle.
    \param  a_triangleVertex2           Vertex 2 of triangle.
    \param  a_reportFrontSideCollision  If __true__, then front side collisions are reported.
    \param  a_reportBackSideCollision   If __true__, then back side collisions are reported.
    \param  a_collisionPoint            Returned intersection point (if detected).
    \param  a_collisionNormal           Returned surface normal at intersection point (if detected).
    \param  a_collisionPosVertex01      Returned relative position of collision point projected on segment \p a_triangleVertex0 -\p a_triangleVertex1.
    \param  a_collisionPosVertex02      Returned relative position of collision point projected on segment \p a_triangleVertex0 -\p a_triangleVertex1

    \return __true__ if a collision has occurred, otherwise __false__.
*/
//==============================================================================
inline bool cIntersectionSegmentTriangle(const cVector3d& a_segmentPointA,
                                         const cVector3d& a_segmentPointB,
                                         const cVector3d& a_triangleVertex0,
                                         const cVector3d& a_triangleVertex1,
                                         const cVector3d& a_triangleVertex2,
                                         const bool a_reportFrontSideCollision,
                                         const bool a_reportBackSideCollision,
                                         cVector3d& a_collisionPoint,
                                         cVector3d& a_collisionNormal,
                                         double& a_collisionPosVertex01,
                                         double& a_collisionPosVertex02
                                         )
{
    // This value controls how close rays can be to parallel to the triangle
    // surface before we discard them
    const double C_INTERSECT_EPSILON = 10e-14f;

    // compute a ray and check its length
    cVector3d rayDir;
    a_segmentPointB.subr(a_segmentPointA, rayDir);
    double segmentLengthSquare = rayDir.lengthsq();
    if (segmentLengthSquare == 0.0) { return (false); }

    // Compute the triangle's normal
    cVector3d t_E0, t_E1, t_N;
    a_triangleVertex1.subr(a_triangleVertex0, t_E0);
    a_triangleVertex2.subr(a_triangleVertex0, t_E1);
    t_E0.crossr(t_E1, t_N);

    // If the ray is parallel to the triangle (perpendicular to the
    // normal), there's no collision
    double dotProduct = t_N.dot(rayDir);
    if (fabs(dotProduct)<10E-15f) return (false);

    if ((dotProduct < 0.0) && (a_reportFrontSideCollision == false)) return (false);
    if ((dotProduct > 0.0) && (a_reportBackSideCollision == false)) return (false);

    double t_T = cDot(t_N, cSub(a_triangleVertex0, a_segmentPointA)) / cDot(t_N, rayDir);

    if (t_T + C_INTERSECT_EPSILON < 0) return (false);

    cVector3d t_Q = cSub(cAdd(a_segmentPointA, cMul(t_T, rayDir)), a_triangleVertex0);
    double t_Q0 = cDot(t_E0,t_Q);
    double t_Q1 = cDot(t_E1,t_Q);
    double t_E00 = cDot(t_E0,t_E0);
    double t_E01 = cDot(t_E0,t_E1);
    double t_E11 = cDot(t_E1,t_E1);
    double t_D = (t_E00 * t_E11) - (t_E01 * t_E01);

    if ((t_D > -C_INTERSECT_EPSILON) && (t_D < C_INTERSECT_EPSILON)) return(false);

    double t_S0 = ((t_E11 * t_Q0) - (t_E01 * t_Q1)) / t_D;
    double t_S1 = ((t_E00 * t_Q1) - (t_E01 * t_Q0)) / t_D;

    // Collision has occurred. It is reported.
    if (
      (t_S0 >= 0.0 - C_INTERSECT_EPSILON) &&
      (t_S1 >= 0.0 - C_INTERSECT_EPSILON) &&
      ((t_S0 + t_S1) <= 1.0 + C_INTERSECT_EPSILON)
      )
    {
        cVector3d t_I = cAdd(a_triangleVertex0, cMul(t_S0,t_E0), cMul(t_S1, t_E1));

        // Square distance between ray origin and collision point.
        double distanceSquare = a_segmentPointA.distancesq(t_I);

        // check if collision occurred within segment. If yes, report collision
        if (distanceSquare <= segmentLengthSquare)
        {
            a_collisionPoint = cAdd(a_segmentPointA, cMul(t_T, rayDir));
            t_N.normalizer(a_collisionNormal);
            if (cCosAngle(a_collisionNormal, rayDir) > 0.0)
            {
                a_collisionNormal.negate();
            }
            
            a_collisionPosVertex01 = t_S0;
            a_collisionPosVertex02 = t_S1;

            return (true);
        }
    }

    // no collision occurred
    return (false);
}

//@}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // CGeometryH
//------------------------------------------------------------------------------


