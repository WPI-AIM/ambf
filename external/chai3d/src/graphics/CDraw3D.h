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
    \author    Dan Morris
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CDraw3DH
#define CDraw3DH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CDraw3D.h
    \ingroup    graphics

    \brief
    Implements OpenGL drawing macros.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup graphics
*/
//------------------------------------------------------------------------------

//@{

//------------------------------------------------------------------------------
// GENERAL PURPOSE FUNCTIONS
//------------------------------------------------------------------------------

//! This function aligns the current z-axis with a reference frame (similar to function gluLookAt).
void cLookAt(const cVector3d& a_eye, 
    const cVector3d& a_at, 
    const cVector3d& a_up);

//! This function draws an x-y-z frame.
void cDrawFrame(const double& a_scale = 1.0);

//! This function draws an x-y-z frame.
void cDrawFrame(const double& a_axisLengthScale, 
    const double& a_axisThicknessScale);

//! This function draws a box using lines.
void cDrawWireBox(const double& a_xMin, const double& a_xMax,
    const double& a_yMin, const double& a_yMax,
    const double& a_zMin, const double& a_zMax);

//! This function draws a sphere.
void cDrawSphere(const double& a_radius,
    const unsigned int a_numSlices=10, 
    const unsigned int a_numStacks=10);

//! This function draws an arrow on the z-axis using a cone and a cylinder.
void cDrawArrow(const cVector3d& a_arrowStart, 
    const cVector3d& a_arrowTip, 
    const double a_width = 0.05);

//! This function draws a torus.
void cDrawSolidTorus(const double& a_innerRadius,
    const double& a_outerRadius, 
    const int a_sides, 
    const int a_rings);

//@}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------


