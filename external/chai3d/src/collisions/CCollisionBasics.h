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
    \author    Chris Sewell
    \author    Charity Lu
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionBasicsH
#define CCollisionBasicsH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CPointArray.h"
#include "graphics/CSegmentArray.h"
#include "graphics/CTriangleArray.h"
#include "materials/CMaterial.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionBasics.h
    
    \brief
    Implements basic data structures for storing collision events.
*/
//==============================================================================

enum cCollisionType
{
    C_COL_NOT_DEFINED,
    C_COL_SHAPE,
    C_COL_POINT,
    C_COL_SEGMENT,
    C_COL_TRIANGLE,
    C_COL_VOXEL
};


//==============================================================================
/*!
    \struct     cCollisionEvent
    \ingroup    collisions

    \brief
    This structure stores the data related to a collision event.
*/
//==============================================================================
struct cCollisionEvent
{
    //! Collision type (triangle, voxel, line, point)
    cCollisionType m_type;

    //! Pointer to the collided object.
    cGenericObject* m_object;

    //! Pointer to point array (if available).
    cPointArrayPtr m_points;

    //! Pointer to segment array (if available).
    cSegmentArrayPtr m_segments;

    //! Pointer to triangle array (if available).
    cTriangleArrayPtr m_triangles;

    //! Index to collided point, segment, or triangle. This pointer may be NULL for collisions with non triangle based objects.
    int m_index;

    //! Index X of voxel in texture image (if available).
    int m_voxelIndexX;

    //! Index Y of voxel in texture image (if available).
    int m_voxelIndexY;

    //! Index Z of voxel in texture image (if available).
    int m_voxelIndexZ;

    //! Position of the collision point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Position of the collision point in world coordinates (global coordinates).
    cVector3d m_globalPos;

    //! Surface normal at collision point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localNormal;

    //! Surface normal at collision point in world coordinates (global coordinates).
    cVector3d m_globalNormal;

    //! Square distance between ray origin and collision point.
    double m_squareDistance;

    //! Projection of collision point onto line going from Vertex0 to Vertex1 of triangle/segment/point (if available). Value is bounded to [0.0, 1.0].
    double m_posV01;

    //! Projection of collision point onto line going from Vertex0 to Vertex2 of triangle/segment/point (if available). Value is bounded to [0.0, 1.0].
    double m_posV02;

    /*!
    If the position of segment A is modified to take into account motion
    (see m_adjustObjectMotion in cCollisionSettings), the value is stored here.
    */
    cVector3d m_adjustedSegmentAPoint;

    //! Initialize all data
    void clear()
    {
        m_type              = C_COL_NOT_DEFINED;
        m_object            = NULL;
        m_points            = nullptr;
        m_segments          = nullptr;
        m_triangles         = nullptr;
        m_index             = -1;
        m_voxelIndexX       = -1;
        m_voxelIndexY       = -1;
        m_voxelIndexZ       = -1;
        m_localPos.zero();
        m_globalPos.zero();
        m_localNormal.zero();
        m_globalNormal.zero();
        m_squareDistance    = C_LARGE;
        m_squareDistance    = C_LARGE;
        m_posV01            = 0.0;
        m_posV02            = 0.0;
    }
};


//==============================================================================
/*!
    \class      cCollisionRecorder
    \ingroup    collisions

    \brief
    This class implements a collision detection recorder that stores all collision
    events that are reported by a collision detector.

    \details
    This class implements a collision detection recorder that stores all collision
    events that are reported by a collision detector.
*/
//==============================================================================
class cCollisionRecorder
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionRecorder
    cCollisionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder
    virtual ~cCollisionRecorder() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method clears all collision records.
    void clear()
    {
        m_nearestCollision.clear();
        m_collisions.clear();
    }


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Nearest collision event from the start point of the collision segment.
    cCollisionEvent m_nearestCollision;

    //! List of all detected collision events.
    std::vector<cCollisionEvent> m_collisions;
};


//==============================================================================
/*!
    \struct     cCollisionSettings
    \ingroup    collisions

    \brief
    This structure stores the collision settings that are passed to a collision 
    detector when querying for collisions.

    \details
    This structure stores the collision settings that are passed to a collision 
    detector when querying for collisions.
*/
//==============================================================================
struct cCollisionSettings
{
    //! Constructor of cCollisionSettings.
    cCollisionSettings()
    {
        m_checkForNearestCollisionOnly  = true;
        m_returnMinimalCollisionData    = false;
        m_checkVisibleObjects           = true;
        m_checkHapticObjects            = true;
        m_adjustObjectMotion            = false;
        m_ignoreShapes                  = false;
        m_collisionRadius               = 0.0;
    }

    //! If __true__, only return the nearest collision event.
    bool m_checkForNearestCollisionOnly;

    //! If __true__, return minimal amount of data about the collision.
    bool m_returnMinimalCollisionData;

    //! If __true__, then collision detector shall check for collisions on visible objects (m_showEnabled == true).
    bool m_checkVisibleObjects;

    //! If __true__, then collision detector shall check for collisions on haptic enabled objects (m_hapticEnabled == true).
    bool m_checkHapticObjects;

    //! If __true__, then adjust for object motion. (See dynamic proxy model).
    bool m_adjustObjectMotion;

    //! If __true__, collision with shape objects are ignored (e.g. cShapeSphere, cShapeCylinder, cShapeBox, etc...)
    bool m_ignoreShapes;

    //! Collision radius. This value typically corresponds to the radius of the virtual tool or cursor.
    double m_collisionRadius;
};

//------------------------------------------------------------------------------
}   // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
