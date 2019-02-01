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
#include "collisions/CCollisionAABB.h"
//------------------------------------------------------------------------------
#include <iostream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cCollisionAABB.
*/
//==============================================================================
cCollisionAABB::cCollisionAABB()
{
    // radius padding around elements
    m_radiusAroundElements = 0.0;

    // list of elements
    m_elements = nullptr;

    // number of elements
    m_numElements = 0;

    // clear nodes
    m_nodes.clear();

    // initialize variables
    m_rootIndex = -1;
    m_maxDepth = 0;
    m_radius = 0.0;
}


//==============================================================================
/*!
    Destructor of cCollisionAABB.
*/
//==============================================================================
cCollisionAABB::~cCollisionAABB()
{
    // clear all nodes
    m_nodes.clear();
}


//==============================================================================
/*!
    This method builds an axis-aligned bounding box collision-detection tree for
    a collection of elements passed as argument. \n\n

    Each leaf is associated with one element and with a boundary box of minimal
    dimensions such that it fully encloses the element and is aligned with
    the coordinate axes (no rotations).  Each internal node is associated
    with a boundary box of minimal dimensions such that it fully encloses
    the boundary boxes of its two children and is aligned with the axes.

    \param  a_elements  Pointer to element array.
    \param  a_radius    Bounding radius to add around each elements.
*/
//==============================================================================
void cCollisionAABB::initialize(const cGenericArrayPtr a_elements, const double a_radius)
{
    ////////////////////////////////////////////////////////////////////////////
    // INITIALIZATION
    ////////////////////////////////////////////////////////////////////////////

    // sanity check
    if (a_elements == nullptr)
    {
        m_rootIndex = -1;
        return;
    }
    m_elements = a_elements;

    // store radius
    m_radius = a_radius;

    // clear previous tree
    m_nodes.clear();

    // get number of elements
    m_numElements = m_elements->getNumElements();

    // init variables
    m_maxDepth = 0;

    // if zero elements, then exit
    if (m_numElements == 0)
    {
        m_rootIndex = -1;
        return;
    }


    ////////////////////////////////////////////////////////////////////////////
    // CREATE LEAF NODES
    ////////////////////////////////////////////////////////////////////////////

    // get number of vertices per element
    int numVerticesPerElement = m_elements->getNumVerticesPerElement();

    switch (numVerticesPerElement)
    {
    case 1:
        {
            // create leaf node for each element
            for (int i=0; i<m_numElements; ++i)
            {
                // get position of vertices
                cVector3d vertex0 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 0));

                // create leaf node
                cCollisionAABBNode leaf;

                leaf.fitBBox(m_radius, vertex0);
                leaf.m_leftSubTree = i;
                leaf.m_nodeType = C_AABB_NODE_LEAF;

                // add leaf to list
                m_nodes.push_back(leaf);
            }
            break;
        }

    case 2:
        {
            // create leaf node for each element
            for (int i=0; i<m_numElements; ++i)
            {
                // get position of vertices
                cVector3d vertex0 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 0));
                cVector3d vertex1 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 1));

                // create leaf node
                cCollisionAABBNode leaf;

                leaf.fitBBox(m_radius, vertex0, vertex1);
                leaf.m_leftSubTree = i;
                leaf.m_nodeType = C_AABB_NODE_LEAF;

                // add leaf to list
                m_nodes.push_back(leaf);
            }
            break;
        }

    case 3:
        {
            // create leaf node for each element
            for (int i=0; i<m_numElements; ++i)
            {
                // get position of vertices
                cVector3d vertex0 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 0));
                cVector3d vertex1 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 1));
                cVector3d vertex2 = m_elements->m_vertices->getLocalPos(m_elements->getVertexIndex(i, 2));

                // create leaf node
                cCollisionAABBNode leaf;

                leaf.fitBBox(m_radius, vertex0, vertex1, vertex2);
                leaf.m_leftSubTree = i;
                leaf.m_nodeType = C_AABB_NODE_LEAF;

                // add leaf to list
                m_nodes.push_back(leaf);
            }
            break;
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    // CREATE TREE
    ////////////////////////////////////////////////////////////////////////////
    int indexFirst = 0;
    int indexLast = m_numElements - 1;
    int depth = 0;

    if (m_numElements > 1)
    {
        m_rootIndex = buildTree(indexFirst, indexLast, depth);
    }
    else
    {
        m_rootIndex = 0;
    }
}


//==============================================================================
/*!
    This methods updates the collision detector and should be called if the 
    3D model it represents is modified.
*/
//==============================================================================
void cCollisionAABB::update()
{
    initialize(m_elements, m_radius);
}


//==============================================================================
/*!
    Given a __start__ and __end__ index value of leaf nodes, this method creates
    a collision tree.

    \param  a_indexFirstNode  Lower index value of leaf node.
    \param  a_indexLastNode   Upper index value of leaf node
    \param  a_depth           Current depth of the tree. Root starts at 0.
*/
//==============================================================================
int cCollisionAABB::buildTree(const int a_indexFirstNode, const int a_indexLastNode, const int a_depth)
{
    // create new node
    cCollisionAABBNode node;

    // set depth of this node.
    node.m_depth = a_depth;
    node.m_nodeType = C_AABB_NODE_INTERNAL;

    // create a box to enclose all the leafs below this internal node
    node.m_bbox.setEmpty();
    for (int i=a_indexFirstNode; i<=a_indexLastNode; i++)
    {
        node.m_bbox.enclose(m_nodes[i].m_bbox);
    }

    // move leafs with smaller coordinates (on the longest axis) towards the
    // beginning of the array and leaves with larger coordinates towards the
    // end of the array
    int axis = node.m_bbox.getLongestAxis();
    int i = a_indexFirstNode;
    int mid = a_indexLastNode;

    double center = node.m_bbox.getCenter().get(axis);
    while (i < mid)
    {
        if (m_nodes[i].m_bbox.getCenter().get(axis) < center)
        {
            i++;
        }
        else
        {
            // swap nodes. For efficiency, we swap the minimum amount of information necessary.
            int t_leftSubTree               = m_nodes[i].m_leftSubTree;
            cCollisionAABBBox t_bbox        = m_nodes[i].m_bbox;

            m_nodes[i].m_leftSubTree        = m_nodes[mid].m_leftSubTree;
            m_nodes[i].m_bbox               = m_nodes[mid].m_bbox;

            m_nodes[mid].m_leftSubTree      = t_leftSubTree;
            m_nodes[mid].m_bbox             = t_bbox;

            //cSwap(m_nodes[i], m_nodes[mid]);
            mid--;
        }
    }

    // increment depth for child nodes
    int depth = a_depth + 1;
    m_maxDepth = cMax(m_maxDepth, depth);

    // we expect mid, used as the right iterator in the "insertion sort" style
    // rearrangement above, to have moved roughly to the middle of the array;
    // however, if it never moved left or moved all the way left, set it to
    // the middle of the array so that neither the left nor right subtree will
    // be empty
    if ((mid == a_indexFirstNode) || (mid == a_indexLastNode))
    {
        mid = (a_indexLastNode + a_indexFirstNode) / 2;
    }

    // if there are only two nodes then assign both child nodes as leaves
    if ((a_indexLastNode - a_indexFirstNode) == 1)
    {
        // set left leaf
        node.m_leftSubTree = a_indexFirstNode;
        m_nodes[a_indexFirstNode].m_depth = depth;

        // set right leaf
        node.m_rightSubTree = a_indexLastNode;
        m_nodes[a_indexLastNode].m_depth = depth;
    }

    // there are more than 2 nodes
    else
    {
        // if the left subtree contains multiple elements, create new internal node
        if (mid > a_indexFirstNode)
        {
            node.m_leftSubTree = buildTree(a_indexFirstNode, mid, depth);
        }

        // if there is only one element in the right subtree, the right subtree
        // pointer should just point to the leaf node
        else
        {
            node.m_leftSubTree = a_indexFirstNode;
            m_nodes[a_indexFirstNode].m_depth = depth;
        }

        // if the right subtree contains multiple elements, create new internal node
        if ((mid+1) < a_indexLastNode)
        {
            node.m_rightSubTree = buildTree((mid+1), a_indexLastNode, depth);
        }

        // if there is only one element in the left subtree, the left subtree
        // pointer should just point to the leaf node
        else
        {
            node.m_rightSubTree = a_indexLastNode;
            m_nodes[a_indexLastNode].m_depth = depth;
        }
    }

    // insert node
    m_nodes.push_back(node);
    return (int)(m_nodes.size()-1);
}


//==============================================================================
/*!
    This method checks if the given line segment intersects any element of the 
    mesh. 

    If a collision occurs, the method returns __true__, and the collision events
    are reported through the collision recorder. Each collision event reports 
    pointers to the intersected element, the mesh of which this element is a part, 
    the point of intersection, and the distance from the origin of the segment to 
    the collision point.

    \param  a_object         Object for which collision detector is being used.
    \param  a_segmentPointA  Initial point of segment.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder       Recorder which stores all collision events.
    \param  a_settings       Contains collision settings information.

    \return  __true__ if a collision event has occurred, __false__otherwise.
*/
//==============================================================================
bool cCollisionAABB::computeCollision(cGenericObject* a_object,
                                      cVector3d& a_segmentPointA, 
                                      cVector3d& a_segmentPointB,
                                      cCollisionRecorder& a_recorder, 
                                      cCollisionSettings& a_settings)
{
    // sanity check
    if (m_rootIndex == -1) { return (false); }

    // init stack
    std::vector<cCollisionAABBStack> stack;
    stack.resize(m_maxDepth+1);

    int index = 0;
    stack[0].m_index = m_rootIndex;
    stack[0].m_state = C_AABB_STATE_TEST_CURRENT_NODE;

    // no collision occurred yet
    bool result = false;

    // create an axis-aligned boundary box for the line
    cCollisionAABBBox lineBox;
    lineBox.setEmpty();
    lineBox.enclose(a_segmentPointA);
    lineBox.enclose(a_segmentPointB);

    // collision search
    while (index > -1)
    {
        // get index of current node on stack 
        int nodeIndex = stack[index].m_index;

        // get type of current node
        cAABBNodeType nodeType = m_nodes[nodeIndex].m_nodeType;


        //----------------------------------------------------------------------
        // INTERNAL NODE:
        //----------------------------------------------------------------------
        if (nodeType == C_AABB_NODE_INTERNAL)
        {
            switch (stack[index].m_state)
            {
                ////////////////////////////////////////////////////////////////
                // TEST CURRENT NODE
                ////////////////////////////////////////////////////////////////
                case C_AABB_STATE_TEST_CURRENT_NODE:
                {
                    // check if line box intersects box of current node
                    if (m_nodes[nodeIndex].m_bbox.intersect(lineBox))
                    {
                        // check if segment intersects box of current node
                        if (m_nodes[nodeIndex].m_bbox.intersect(a_segmentPointA, a_segmentPointB))
                        {
                            stack[index].m_state = C_AABB_STATE_TEST_LEFT_NODE;
                        }
                        else
                        {
                            stack[index].m_state = C_AABB_STATE_TEST_CURRENT_NODE;
                            index--;
                        }
                    }
                    else
                    {
                        stack[index].m_state = C_AABB_STATE_TEST_CURRENT_NODE;
                        index--;
                    }
                }
                break;

                ////////////////////////////////////////////////////////////////
                // TEST LEFT NODE
                ////////////////////////////////////////////////////////////////
                case C_AABB_STATE_TEST_LEFT_NODE:
                {
                    stack[index].m_state = C_AABB_STATE_TEST_RIGHT_NODE;

                    // push left child node on stack
                    index++;
                    stack[index].m_index =  m_nodes[nodeIndex].m_leftSubTree;
                    stack[index].m_state = C_AABB_STATE_TEST_CURRENT_NODE;
                }
                break;

                ////////////////////////////////////////////////////////////////
                // TEST RIGHT NODE
                ////////////////////////////////////////////////////////////////
                case C_AABB_STATE_TEST_RIGHT_NODE:
                {
                    stack[index].m_state = C_AABB_STATE_POP_STACK;

                    // push right child node on stack
                    index++;
                    stack[index].m_index =  m_nodes[nodeIndex].m_rightSubTree;
                    stack[index].m_state = C_AABB_STATE_TEST_CURRENT_NODE;
                }
                break;

                ////////////////////////////////////////////////////////////////
                // POP STACK
                ////////////////////////////////////////////////////////////////
                case C_AABB_STATE_POP_STACK:
                {
                    // restore state of current node for next search and pop stack
                    stack[index].m_state = C_AABB_STATE_TEST_CURRENT_NODE;
                    index--;
                }
                break;
            }
        }


        //----------------------------------------------------------------------
        // LEAF NODE:
        //----------------------------------------------------------------------
        else if (nodeType == C_AABB_NODE_LEAF)
        {
            // get index of leaf element
            int elementIndex =  m_nodes[nodeIndex].m_leftSubTree;

            // call the element's collision detection method
            if (m_elements->m_allocated[elementIndex])
            {
                if (m_elements->computeCollision(elementIndex,
                    a_object,
                    a_segmentPointA, 
                    a_segmentPointB, 
                    a_recorder, 
                    a_settings))
                {
                    result = true;
                }
            }

            // pop stack
            index--;
        }
    }

    // return result
    return (result);
}


//==============================================================================
/*!
    This method graphically renders the boundary boxes of the collision tree 
    using OpenGL.
*/
//==============================================================================
void cCollisionAABB::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // set rendering settings
    glDisable(GL_LIGHTING);
    glLineWidth(1.0);
    glColor4fv(m_color.getData());

    // render tree by calling the root, which recursively calls the children
    vector<cCollisionAABBNode>::iterator i;
    for(i = m_nodes.begin(); i != m_nodes.end(); i++)
    {
        i->render(m_displayDepth);
    }

    // restore lighting settings
    glEnable(GL_LIGHTING);

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
