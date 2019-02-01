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
#ifndef CBezierH
#define CBezierH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBezier.h
    \ingroup    math

    \brief
    Implements support for Bezier curves and surface patches.
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
    This function determines the position of a point along a Bezier curve.

    \details
    This function determines the position of a point along a Bezier
    curve at __t__ [0:1].

    \param  a_controlPoints  Control points. (4 in total)
    \param  a_t              Parameter __t__ between 0 and 1.

    \return The computed point.
*/
//==============================================================================
inline cVector3d cEvalBezierCurve(const cVector3d *a_controlPoints, double a_t)
{
    const cVector3d *P = a_controlPoints;
    const double t = a_t;

    double b0 = (1 - t) * (1 - t) * (1 - t);
    double b1 = 3 * t * (1 - t) * (1 - t);
    double b2 = 3 * t * t * (1 - t);
    double b3 = t * t * t;

    return (P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3);
}


//==============================================================================
/*!
    \brief
    This function determines the position of a point along a Bezier patch.

    \details
    This function determines the position of a point along a Bezier
    patch at __u__ [0:1] and __v__ [0:1].

    \param  a_controlPoints  Control points. (16 in total)
    \param  a_u              Parameter __u__ between 0 and 1.
    \param  a_v              Parameter __v__ between 0 and 1.

    \return The computed point.
*/
//==============================================================================
inline cVector3d cEvalBezierPatch(const cVector3d *a_controlPoints, double a_u, double a_v)
{
    cVector3d uCurve[4];
    for (int i = 0; i < 4; ++i)
        uCurve[i] = cEvalBezierCurve(a_controlPoints + 4 * i, a_u);

    return (cEvalBezierCurve(uCurve, a_v));
}


//==============================================================================
/*!
    \brief
    This function determines the derivative of a point along a Bezier curve.

    \details
    This function determines the derivative of a point along a Bezier
    curve at __t__ [0:1].

    \param  a_controlPoints  Control points. (4 in total)
    \param  a_t              Parameter __t__ between 0 and 1.

    \return The computed derivative.
*/
//==============================================================================
inline cVector3d cDerivBezier(const cVector3d *a_controlPoints, double a_t)
{
    const cVector3d *P = a_controlPoints;
    const double t = a_t;

    return ((-3 * (1 - t) * (1 - t) * P[0] +
        (3 * (1 - t) * (1 - t) - 6 * t * (1 - t)) * P[1] +
        (6 * t * (1 - t) - 3 * t * t) * P[2] +
        3 * t * t * P[3]));
}


//==============================================================================
/*!
    \brief
    This function determines the derivative of a point on a Bezier patch
    along the __u__ parametric direction.

    \details
    This function determines the derivative of a point on a Bezier patch
    along the __u__ parametric direction.

    \param  a_controlPoints  Control points. (16 in total)
    \param  a_u              Parameter __u__ between 0 and 1.
    \param  a_v              Parameter __v__ between 0 and 1.

    \return The computed derivative.
*/
//==============================================================================
inline cVector3d cDerivUBezier(const cVector3d *a_controlPoints, double a_u, double a_v)
{
    cVector3d P[4];
    cVector3d vCurve[4];
    for (int i = 0; i < 4; ++i) {
        P[0] = a_controlPoints[i];
        P[1] = a_controlPoints[4 + i];
        P[2] = a_controlPoints[8 + i];
        P[3] = a_controlPoints[12 + i];
        vCurve[i] = cEvalBezierCurve(P, a_v);
    }

    return (cDerivBezier(vCurve, a_u));
}


//==============================================================================
/*!
    \brief
    This function determines the derivative of a point on a Bezier patch
    along the __v__ parametric direction.

    \details
    This function determines the derivative of a point on a Bezier patch
    along the __v__ parametric direction.

    \param  a_controlPoints  Control points. (16 in total)
    \param  a_u              Parameter __u__ between 0 and 1.
    \param  a_v              Parameter __v__ between 0 and 1.

    \return The computed derivative.
*/
//==============================================================================
inline cVector3d cDerivVBezier(const cVector3d *a_controlPoints, double a_u, double a_v)
{
    cVector3d uCurve[4];
    for (int i = 0; i < 4; ++i) {
        uCurve[i] = cEvalBezierCurve(a_controlPoints + 4 * i, a_u);
    }

    return cDerivBezier(uCurve, a_v);
}


//==============================================================================
/*!
    \brief
    This function determines the surface normal at a point on a Bezier patch.

    \details
    This function determines the surface normal at a point on a Bezier patch
    at __u__ [0:1] and __v__ [0:1].

    \param  a_controlPoints  Control points. (16 in total)
    \param  a_u              Parameter __u__ between 0 and 1.
    \param  a_v              Parameter __v__ between 0 and 1.

    \return The computed surface normal.
*/
//==============================================================================
inline cVector3d cSurfaceNormalBezier(const cVector3d *a_controlPoints, double a_u, double a_v)
{
    cVector3d du = cDerivUBezier(a_controlPoints, a_u, a_v);
    cVector3d dv = cDerivVBezier(a_controlPoints, a_u, a_v);

    cVector3d normal = cCross(du, dv);
    normal.normalize();

    return (normal);
}


//@}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // CBezierH
//------------------------------------------------------------------------------