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

//---------------------------------------------------------------------------
#include "CGELMesh.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    This method initializes the deformable mesh.
*/
//===========================================================================
void cGELMesh::initialise()
{
    m_showSkeletonModel = false;
    m_showMassParticleModel = false;
    m_useSkeletonModel = false;
    m_useMassParticleModel = false;
}


//===========================================================================
/*!
    This method clears all force variables and sets them to zero.
*/
//===========================================================================
void cGELMesh::clearForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->clearForces();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->clearForces();
        }
    }
}


//===========================================================================
/*!
    This method clears all external force variables and sets them to zero.
*/
//===========================================================================
void cGELMesh::clearExternalForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->clearExternalForces();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->clearExternalForces();
        }
    }
}


//===========================================================================
/*!
    This method computes all internal forces by parsing every elastic link
    of the skeleton.
*/
//===========================================================================
void cGELMesh::computeForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonLink*>::iterator i;

        for(i = m_links.begin(); i != m_links.end(); ++i)
        {
            (*i)->computeForces();
        }
    }
    if (m_useMassParticleModel)
    {
        list<cGELLinearSpring*>::iterator i;

        for(i = m_linearSprings.begin(); i != m_linearSprings.end(); ++i)
        {
            (*i)->computeForces();
        }
    }
}


//===========================================================================
/*!
    This method updates the simulation over a given time interval passed
    as argument.

    \param  a_timeInterval  Time interval.
*/
//===========================================================================
void cGELMesh::computeNextPose(double a_timeInterval)
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->computeNextPose(a_timeInterval);
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->computeNextPose(a_timeInterval);
        }
    }
}


//===========================================================================
/*!
    This method updates the position of every skeleton node.
*/
//===========================================================================
void cGELMesh::applyNextPose()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->applyNextPose();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->applyNextPose();
        }
    }
}


//===========================================================================
/*!
    This method graphically renders the deformable mesh in OpenGL.

    \param  a_options  Rendering options.
*/
//===========================================================================
void cGELMesh::render(cRenderOptions& a_options)
{
    // render mesh
    cMultiMesh::render(a_options);

    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        if (m_showSkeletonModel)
        {
            list<cGELSkeletonNode*>::iterator i;
            for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
            {
                cGELSkeletonNode* nextItem = *i;
                nextItem->render();
            }

            list<cGELSkeletonLink*>::iterator j;
            for(j = m_links.begin(); j != m_links.end(); ++j)
            {
                cGELSkeletonLink* nextItem = *j;
                nextItem->render();
            }
        }

        if (m_showMassParticleModel)
        {
            glDisable(GL_LIGHTING);

            list<cGELLinearSpring*>::iterator j;
            for(j = m_linearSprings.begin(); j != m_linearSprings.end(); ++j)
            {
                (*j)->render();
            }

            vector<cGELVertex>::iterator i;
            for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
            {
                i->m_massParticle->render();
            }

            glEnable(GL_LIGHTING);
        }
    }
}


//===========================================================================
/*!
    This method builds the dynamic vertices of this deformable mesh.
*/
//===========================================================================
void cGELMesh::buildVertices()
{
    // for all meshes, activate user data
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->m_vertices->allocateData(0, true, true, true, true, true, true);
    }

    // clear all deformable vertices
    m_gelVertices.clear();

    // get number of vertices
    int numVertices = getNumVertices();

    // create a table of deformable vertices based on the vertices of image
    for (int i=0; i<numVertices; i++)
    {
        cMesh* mesh = NULL;
        unsigned int vertexIndex = 0;

        // create a new deformable vertex data
        if (getVertex(i, mesh, vertexIndex))
        {
            cGELVertex newDefVertex(mesh, vertexIndex);
            newDefVertex.m_link = NULL;
            newDefVertex.m_node = NULL;
            mesh->m_vertices->setUserData(vertexIndex, i);

            // add vertex to list
            m_gelVertices.push_back(newDefVertex);
        }
    }
}


//===========================================================================
/*!
    This method connects each dynamic vertex to the nearest node on the
    skeleton.

    \param  a_connectToNodesOnly  if __true__, then skin is only connected 
            to nodes only, if __false__ then skin mesh is connected to nodes
            and links.
*/
//===========================================================================
void cGELMesh::connectVerticesToSkeleton(bool a_connectToNodesOnly)
{
    // get number of vertices
    int numVertices = (int)(m_gelVertices.size());

    // for each deformable vertex we search for the nearest sphere or link
    for (int i=0; i<numVertices; i++)
    {
        // get current deformable vertex
        cGELVertex* curVertex = &m_gelVertices[i];

        // get current vertex position
        cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

        // initialize constant
        double min_distance = 99999999999999999.0;
        cGELSkeletonNode* nearest_node = NULL;
        cGELSkeletonLink* nearest_link = NULL;

        // search for the nearest node
        list<cGELSkeletonNode*>::iterator itr;
        for(itr = m_nodes.begin(); itr != m_nodes.end(); ++itr)
        {
            cGELSkeletonNode* nextNode = *itr;
            double distance = cDistance(pos, nextNode->m_pos);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_node = nextNode;
                nearest_link = NULL;
            }
        }

        // search for the nearest link if any
        if (!a_connectToNodesOnly)
        {
            list<cGELSkeletonLink*>::iterator j;
            for(j = m_links.begin(); j != m_links.end(); ++j)
            {
                cGELSkeletonLink* nextLink = *j;
                double angle0 = cAngle(nextLink->m_wLink01, cSub(pos, nextLink->m_node0->m_pos));
                double angle1 = cAngle(nextLink->m_wLink10, cSub(pos, nextLink->m_node1->m_pos));

                if ((angle0 < (C_PI / 2.0)) && (angle1 < (C_PI / 2.0)))
                {
                    cVector3d p = cProjectPointOnLine(pos,
                                                      nextLink->m_node0->m_pos,
                                                      nextLink->m_wLink01);

                    double distance = cDistance(pos, p);

                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_node = NULL;
                        nearest_link = nextLink;
                    }
                }
            }
        }

        // attach vertex to nearest node if it exists
        if (nearest_node != NULL)
        {
            curVertex->m_node = nearest_node;
            curVertex->m_link = NULL;
            cVector3d posRel = cSub(pos, nearest_node->m_pos);
            curVertex->m_massParticle->m_pos = cMul(cTranspose(nearest_node->m_rot), posRel);
        }

        // attach vertex to nearest link if it exists
        else if (nearest_link != NULL)
        {
            curVertex->m_node = NULL;
            curVertex->m_link = nearest_link;

            cMatrix3d rot;
            rot.setCol( nearest_link->m_A0,
                        nearest_link->m_B0,
                        nearest_link->m_wLink01);
            cVector3d posRel = cSub(pos, nearest_link->m_node0->m_pos);
            curVertex->m_massParticle->m_pos = cMul(cInverse(rot), posRel);
        }
    }
}


//===========================================================================
/*!
    This method updates the position of each vertex attached to the skeleton.
*/
//===========================================================================
void cGELMesh::updateVertexPosition()
{
    if (m_useSkeletonModel)
    {
        // get number of vertices
        int numVertices = (int)(m_gelVertices.size());

        // for each deformable vertex, update its position
        for (int i=0; i<numVertices; i++)
        {
            // get current deformable vertex
            cGELVertex* curVertex = &m_gelVertices[i];

            // vertex is attached to an node
            if (curVertex->m_node != NULL)
            {
                cVector3d newPos;
                curVertex->m_node->m_rot.mulr(curVertex->m_massParticle->m_pos, newPos);
                newPos.add(curVertex->m_node->m_pos);
                curVertex->m_mesh->m_vertices->setLocalPos(curVertex->m_vertexIndex, newPos);
            }

            // vertex is attached to a link
            else if (curVertex->m_link != NULL)
            {
                cVector3d newPos;
                curVertex->m_link->m_node0->m_pos.addr(curVertex->m_massParticle->m_pos.z() * curVertex->m_link->m_wLink01, newPos);
                newPos.add(curVertex->m_massParticle->m_pos.x() * curVertex->m_link->m_wA0);
                newPos.add(curVertex->m_massParticle->m_pos.y() * curVertex->m_link->m_wB0);


                //curVertex->node->m_rot.mulr(curVertex->pos, newPos);
                //newPos.add(curVertex->node->m_pos);
                curVertex->m_mesh->m_vertices->setLocalPos(curVertex->m_vertexIndex, newPos);
            }
        }
    }

    if (m_useMassParticleModel)
    {
        // get number of vertices
        int numVertices = (int)(m_gelVertices.size());

        // for each deformable vertex, update its position
        for (int i=0; i<numVertices; i++)
        {
            // get current deformable vertex
            cGELVertex* curVertex = &m_gelVertices[i];

            // vertex is attached to an node
            if (curVertex->m_massParticle != NULL)
            {
                curVertex->m_mesh->m_vertices->setLocalPos(curVertex->m_vertexIndex, curVertex->m_massParticle->m_pos);
            }
        }
    }
}
