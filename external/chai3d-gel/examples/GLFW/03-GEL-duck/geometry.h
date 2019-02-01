//===========================================================================
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
//===========================================================================

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "chai3d.h"

inline double squaredDistance(const cVector3d &Y, const cTriangle &T)
{
    cVector3d D0 = T.getVertex1()->getLocalPos() - T.getVertex0()->getLocalPos();
    cVector3d D1 = T.getVertex2()->getLocalPos() - T.getVertex0()->getLocalPos();
    cVector3d Delta = Y - T.getVertex0()->getLocalPos();

    double a00 = D0.lengthsq(), a01 = cDot(D0, D1), a11 = D1.lengthsq();
    double b0 = cDot(D0, Delta), b1 = cDot(D1, Delta);

    double n0 = a11 * b0 - a01 * b1;
    double n1 = a00 * b1 - a01 * b0;
    double d = a00 * a11 - a01 * a01;   // theoretically positive
    double c = Delta.lengthsq();

    double eps = 1e-8;
    if (n0 + n1 <= d) {
        if (n0 >= eps) { // 0.0) {
            if (n1 >= eps) { // 0.0) {
                // region 0 - point is inside the triangle
                return 0.0;
            }
            else {
                // region 5
                if (b0 > eps) { // 0.0) {
                    if (b0 < a00)   return c - b0 * b0 / a00;
                    else            return a00 - 2.0 * b0 + c;
                }
                else                return c;
            }
        }
        else if (n1 >= eps) { // 0.0) {
            // region 3
            if (b1 > eps) { // 0.0) {
                if (b1 < a11)   return c - b1 * b1 / a11;
                else            return a11 - 2.0 * b0 + c;
            }
            else                return c;
        }
        else {
            // region 4
            if (b0 < a00) {
                if (b0 > eps)           return c - b0 * b0 / a00;
                else {
                    if (b1 < a11) {
                        if (b1 > eps)   return c - b1 * b1 / a11;
                        else            return c;
                    }
                    else                return a11 - 2.0 * b1 + c;
                }
            }
            else                        return a00 - 2.0 * b0 + c;
        }
    }
    else if (n0 < -eps) { // 0.0) {
        // region 2
        if (b1 > eps) { // 0.0) {
            if (b1 < a11)       return c - b1 * b1 / a11;
            else {
                double n = a11 - a01 + b0 - b1, d = a00 - 2.0 * a01 + a11;
                if (n > eps) { // 0.0)  {
                    if (n < d)  return (a11 - 2.0 * b1 + c) - n * n / d;
                    else        return a00 - 2.0 * b0 + c;
                }
                else            return a11 - 2.0 * b1 + c;
            }
        }
        else                    return c;
    }
    else if (n1 < -eps) { // 0.0) {
        // region 6
        if (b0 > eps) { // 0.0) {
            if (b0 < a00)       return c - b0 * b0 / a00;
            else {
                double n = a11 - a01 + b0 - b1, d = a00 - 2.0 * a01 + a11;
                if (n > eps) { // 0.0) {
                    if (n < d)  return (a11 - 2.0 * b1 + c) - n * n / d;
                    else        return a00 - 2.0 * b0 + c;
                }
                else            return a11 - 2.0 * b1 + c;
            }
        }
        else                    return c;
    }
    else {
        // region 1
        double n = a11 - a01 + b0 - b1, d = a00 - 2.0 * a01 + a11;
        if (n > eps) { // 0.0) {
            if (n < d)  return (a11 - 2.0 * b1 + c) - n * n / d;
            else        return a00 - 2.0 * b0 + c;
        }
        else            return a11 - 2.0 * b1 + c;
    }
}

double area(const cVector3d &v0, const cVector3d &v1, const cVector3d &v2)
{
    return 0.5 * cCross((v1 - v0), (v2 - v0)).length();
}

cVector3d barycentric(const cVector3d &p, const cTriangle &t)
{
    cVector3d p0 = t.getVertex0()->getLocalPos();
    cVector3d p1 = t.getVertex1()->getLocalPos();
    cVector3d p2 = t.getVertex2()->getLocalPos();

    double a = area(p0, p1, p2);

    cVector3d b;
    if (a != 0.0) {
        b.x(area(p, p1, p2) / a);
        b.y(area(p0, p, p2) / a);
        b.z(area(p0, p1, p) / a);
    }
    return b;
}

#endif
