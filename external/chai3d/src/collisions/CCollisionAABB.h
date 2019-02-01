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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionAABBH
#define CCollisionAABBH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionAABBTree.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionAABB.h

    \brief
    Implements an axis-aligned bounding box collision tree (AABB)
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionAABB
    \ingroup    collisions

    \brief
    This class implements an axis-aligned bounding box collision detector.

    \details
    This class implements an axis-aligned bounding box collision detection
    tree to efficiently detect for any collision between a line segment and 
    a collection of elements (point, segment, triangle) that compose an object.
*/
//==============================================================================
class cCollisionAABB : public cGenericCollision
{
    enum cCollisionAABBState
    {
        C_AABB_STATE_TEST_CURRENT_NODE,
        C_AABB_STATE_TEST_LEFT_NODE,
        C_AABB_STATE_TEST_RIGHT_NODE,
        C_AABB_STATE_POP_STACK
    };

    struct cCollisionAABBStack
    {
        int m_index;
        cCollisionAABBState m_state;
    };

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionAABB.
    cCollisionAABB();

    //! Destructor of cCollisionAABB.
    virtual ~cCollisionAABB();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This methods updates the collision detector and should be called if the 3D model it represents is modified.
    virtual void update();

    //! This method computes all collisions between a segment passed as argument and the attributed 3D object.
    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings);

    //! This method renders a visual representation of the collision tree.
    virtual void render(cRenderOptions& a_options);

    //! This method initializes and builds the AABB collision tree.
    void initialize(const cGenericArrayPtr a_elements,
                    const double a_radius = 0.0);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    // This method is used to recursively build the collision tree.
    int buildTree(const int a_indexFirstNode, const int a_indexLastNode, const int a_depth);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Collision shell radius around elements.
    double m_radius;

    //! Number of elements inside tree.
    int m_numElements;

    //! Pointer to the list of elements in the object.
    cGenericArrayPtr m_elements;

    //! List of nodes.
    std::vector<cCollisionAABBNode> m_nodes;

    //! Index number of root node.
    int m_rootIndex;

    //! Maximum depth of tree.
    int m_maxDepth;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
