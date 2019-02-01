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
#include "collisions/CCollisionAABBTree.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


//==============================================================================
/*!
    Constructor of cCollisionAABBNode.
*/
//==============================================================================
cCollisionAABBNode::cCollisionAABBNode()
{
    m_bbox.setEmpty();
    m_depth        = -1;
    m_nodeType     = C_AABB_NOT_DEFINED;
    m_leftSubTree  = -1;
    m_rightSubTree = -1;
}


//==============================================================================
/*!
      This method creates a boundary box to enclose a point belonging to the 
      leaf node.

      \param  a_radius   Radius around the point.
      \param  a_vertex0  Vertex 0.
*/
//==============================================================================
void cCollisionAABBNode::fitBBox(double a_radius,
    cVector3d& a_vertex0)
{
    // empty box
    m_bbox.setEmpty();

    // enclose vertex
    m_bbox.enclose(a_vertex0);

    // retrieve boundary box min and max values
    cVector3d min = m_bbox.m_min;
    cVector3d max = m_bbox.m_max;

    // add radius envelope
    min.sub(a_radius, a_radius, a_radius);
    max.add(a_radius, a_radius, a_radius);

    // store new values
    m_bbox.setValue(min, max);
}


//==============================================================================
/*!
      This method creates a boundary box to enclose the two vertices of a segment
      belonging to the leaf node.

      \param  a_radius   Radius around the segment.
      \param  a_vertex0  Vertex 0.
      \param  a_vertex1  Vertex 1.
*/
//==============================================================================
void cCollisionAABBNode::fitBBox(double a_radius,
    cVector3d& a_vertex0,
    cVector3d& a_vertex1)
{
    // empty box
    m_bbox.setEmpty();

    // enclose vertices
    m_bbox.enclose(a_vertex0);
    m_bbox.enclose(a_vertex1);

    // retrieve boundary box min and max values
    cVector3d min = m_bbox.m_min;
    cVector3d max = m_bbox.m_max;

    // add radius envelope
    min.sub(a_radius, a_radius, a_radius);
    max.add(a_radius, a_radius, a_radius);

    // store new values
    m_bbox.setValue(min, max);
}


//==============================================================================
/*!
      This method creates a boundary box to enclose the three vertices of a
      triangle belonging to the leaf node.

      \param  a_radius   Radius around the element.
      \param  a_vertex0  Vertex 0.
      \param  a_vertex1  Vertex 1.
      \param  a_vertex2  Vertex 2.
*/
//==============================================================================
void cCollisionAABBNode::fitBBox(double a_radius,
    cVector3d& a_vertex0,
    cVector3d& a_vertex1,
    cVector3d& a_vertex2)
{
    // empty box
    m_bbox.setEmpty();

    // enclose all vertices
    m_bbox.enclose(a_vertex0);
    m_bbox.enclose(a_vertex1);
    m_bbox.enclose(a_vertex2);

    // retrieve boundary box min and max values
    cVector3d min = m_bbox.m_min;
    cVector3d max = m_bbox.m_max;

    // add radius envelope
    min.sub(a_radius, a_radius, a_radius);
    max.add(a_radius, a_radius, a_radius);

    // store new values
    m_bbox.setValue(min, max);
}


//==============================================================================
/*!
    This method draws the edges of the boundary box for an internal tree node 
    if it is at depth a_depth in the tree, and calls the draw function for its 
    children.

    \param  a_depth  If a_depth > 0, then only draw nodes at this level in the tree.
                     If a_depth < 0 render all nodes up to this level.
*/
//==============================================================================
void cCollisionAABBNode::render(int a_depth)
{
#ifdef C_USE_OPENGL
    if ( ((a_depth < 0) && (abs(a_depth) >= m_depth)) || (a_depth == m_depth))
    {
        m_bbox.render();
    }
#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
