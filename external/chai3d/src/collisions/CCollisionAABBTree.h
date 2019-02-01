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
#ifndef CCollisionAABBTreeH
#define CCollisionAABBTreeH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionAABBTree.h

    \brief
    Implements an axis-aligned bounding box collision tree (AABB)
*/
//==============================================================================

//------------------------------------------------------------------------------
//! Internal AABB Node Types.
typedef enum
{
    C_AABB_NODE_INTERNAL,
    C_AABB_NODE_LEAF,
    C_AABB_NOT_DEFINED
} cAABBNodeType;

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cCollisionAABBNode
    \ingroup    collisions

    \brief
    This structure implements a tree node inside an AABB collision tree.
*/
//==============================================================================
struct cCollisionAABBNode
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionAABBNode.
    cCollisionAABBNode();

    //! Destructor of cCollisionAABBNode.
    virtual ~cCollisionAABBNode() {}
    
    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a boundary box for a point.
    void fitBBox(double a_radius,
        cVector3d& a_vertex0);

    //! This method creates a boundary box for a segment.
    void fitBBox(double a_radius,
        cVector3d& a_vertex0,
        cVector3d& a_vertex1);

    //! This method creates a boundary box for a triangle.
    void fitBBox(double a_radius,
        cVector3d& a_vertex0,
        cVector3d& a_vertex1,
        cVector3d& a_vertex2);

    //! This method draws the edges of the boundary box for this node, if at the given depth.
    void render(int a_depth = -1);

    //! This method determines whether a line segment intersects any elements covered by this node.
    bool computeCollision(cGenericObject* a_owner,
        cVector3d& a_segmentPointA,
        cVector3d& a_segmentDirection,
        cCollisionAABBBox& a_lineBox,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Bounding box for this node.
    cCollisionAABBBox m_bbox;

    //! Depth of this node in the collision tree.
    int m_depth;

    //! Node type.
    cAABBNodeType m_nodeType;

    //! Left child node index.
    int m_leftSubTree;

    //! Right child node index.
    int m_rightSubTree;
};


//------------------------------------------------------------------------------
}   // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

