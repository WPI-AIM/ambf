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
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPrimitivesH
#define CPrimitivesH
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "world/CMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPrimitives.h
    \ingroup    graphics

    \brief
    Implements functions to create basic mesh primitives.
*/
//==============================================================================

//------------------------------------------------------------------------------
// GENERAL PURPOSE FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/*!
    \addtogroup graphics
*/
//------------------------------------------------------------------------------

//@{

//! This function creates a plane.
void cCreatePlane(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a plane.
void cCreatePlane2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_colorTopLeft = cColorf(1.0, 1.0, 1.0, 1.0),
    const cColorf& a_colorTopRight = cColorf(1.0, 1.0, 1.0, 1.0),
    const cColorf& a_colorBottomLeft = cColorf(1.0, 1.0, 1.0, 1.0),
    const cColorf& a_colorBottomRight = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a height map. Similar to a map, but with more triangles.
void cCreateMap(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const unsigned int a_numSidesX = 10,
    const unsigned int a_numSidesY = 10,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a disk.
void cCreateDisk(cMesh* a_mesh, 
    const double& a_radiusX, 
    const double& a_radiusY, 
    const unsigned int a_numSlices = 36,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a panel with optional rounded corners.
void cCreatePanel(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_radiusCorners = 0,
    const int& a_numSegmentsPerCorner = 8,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a panel with optional rounded corners.
void cCreatePanel2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_cornerTopLeftRadius = 0,
    const double& a_cornerTopRightRadius = 0,
    const double& a_cornerBottomLeftRadius = 0,
    const double& a_cornerBottomRightRadius = 0,
    const int& a_numSegmentsPerCorner = 8,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_colorTopLeft = cColorf(0.5f, 0.5f, 0.5f, 1.0f),
    const cColorf& a_colorTopRight = cColorf(0.5f, 0.5f, 0.5f, 1.0f),
    const cColorf& a_colorBottomLeft = cColorf(0.3f, 0.3f, 0.3f, 1.0f),
    const cColorf& a_colorBottomRight = cColorf(0.3f, 0.3f, 0.3f, 1.0f));

//! This function creates a box.
void cCreateBox(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_lengthZ,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a tea pot.
void cCreateTeaPot(cMesh* a_mesh, 
    const double& a_size,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a tea pot.
void cCreateTeaPot(cMesh* a_mesh,
    const double& a_size,
    const int& a_quality = 4,
    const cVector3d& a_pos = cVector3d(0, 0, 0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a cylinder.
void cCreateCylinder(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_radius,
    const unsigned int a_numSides = 32,
    const unsigned int a_numHeightSegments = 1,
    const unsigned int a_numRings = 1,
    const bool a_includeTop = true,
    const bool a_includeBottom = true,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a pipe.
void cCreatePipe(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides = 32,
    const unsigned int a_numHeightSegments = 1,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a sphere.
void cCreateSphere(cMesh* a_mesh, 
    const double& a_radius,  
    const unsigned int a_numSlices = 32,
    const unsigned int a_numStacks = 32,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates an ellipsoid.
void cCreateEllipsoid(cMesh* a_mesh, 
    const double& a_radiusX,
    const double& a_radiusY,
    const double& a_radiusZ,
    const unsigned int a_numSlices = 32,
    const unsigned int a_numStacks = 32,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a ring.
void cCreateRing(cMesh* a_mesh, 
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides = 32,
    const unsigned int a_numRings = 32,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a ring section.
void cCreateRingSection(cMesh* a_mesh, 
    const double& a_innerRadius0,
    const double& a_innerRadius1,
    const double& a_outerRadius,
    const double& a_coverageAngleDEG = 360,
    const bool a_includeExtremityFaces = true,
    const unsigned int a_numSides = 32,
    const unsigned int a_numRings = 32,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a cone.
void cCreateCone(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_radiusBottom,
    const double& a_radiusTop = 0.0,
    const unsigned int a_numSides = 32,
    const unsigned int a_numHeightSegments = 1,
    const unsigned int a_numRings = 1,
    const bool a_includeBottom = true,
    const bool a_includeTop = true,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a pyramid.
void cCreateSquarePyramid(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_baseSize,
    const bool a_includeBottom = true,
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates an arrow.
void cCreateArrow(cMesh* a_mesh,
    const double& a_length = 0.4,
    const double& a_radiusShaft = 0.01,
    const double& a_lengthTip = 0.1,
    const double& a_radiusTip = 0.03,
    const bool a_includeTipsAtBothExtremities = false,
    const unsigned int a_numSides = 32,
    const cVector3d& a_direction = cVector3d(0,0,1),
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates an circular arrow.
void cCreateCircularArrow(cMesh* a_mesh, 
    const double& a_innerRadius0 = 0.05,
    const double& a_innerRadius1 = 0.05,
    const double& a_outerRadius = 0.3,
    const double& a_lengthTip = 0.2,
    const double& a_radiusTip = 0.1,
    const double& a_coverageAngleDEG = 270,
    const bool a_includeTipsAtBothExtremities = false,
    const unsigned int a_numSides = 32,
    const unsigned int a_numRings = 32,  
    const cVector3d& a_direction = cVector3d(0,0,1),
    const cVector3d& a_pos = cVector3d(0,0,0),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//! This function creates a Bezier patch
void cCreateBezierPatch(cMesh* a_mesh,
    const cVector3d *a_controlPoints,
    const int a_numDivisions = 8,
    const cVector3d& a_pos = cVector3d(0, 0, 0),
    const cMatrix3d& a_rot = cIdentity3d(),
    const cColorf& a_color = cColorf(1.0, 1.0, 1.0, 1.0));

//@}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------


