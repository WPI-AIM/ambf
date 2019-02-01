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
#ifndef CGELMeshH
#define CGELMeshH
//---------------------------------------------------------------------------
#include "CGELSkeletonNode.h"
#include "CGELSkeletonLink.h"
#include "CGELLinearSpring.h"
#include "CGELVertex.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELMesh.h

    \brief
    Implementation of a deformable mesh.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELMesh
    \ingroup    GEL

    \brief
    This class inherits from class cMesh and integrate a skeleton model for 
    modelling deformations of the mesh.
*/
//===========================================================================
class cGELMesh : public chai3d::cMultiMesh
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELMesh.
    cGELMesh(){ initialise(); };

    //! Destructor of cGELMesh.
    virtual ~cGELMesh() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method builds all dynamic vertices that describe the deformable mesh.
    void buildVertices();

    //! This method connects each dynamic vertex to the skeleton.
    void connectVerticesToSkeleton(bool a_connectToNodesOnly);

    //! This method updates the position of all vertices connected to the skeleton.
    void updateVertexPosition();

    //! This method sets all computed forces to zero.
    void clearForces();

    //! This method sets all external forces to zero.
    void clearExternalForces();

    //! This method computes all internal forces.
    void computeForces();

    //! This method updates the simulation over a time interval.
    void computeNextPose(double iTimeInterval);

    //! This method updates the position of the mesh after computation.
    void applyNextPose();

    //! This method renders the deformable mesh graphically.
    virtual void render(chai3d::cRenderOptions& a_options);


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! List of nodes composing the skeleton.
    std::list<cGELSkeletonNode*> m_nodes;

    //! List of links connecting the different nodes.
    std::list<cGELSkeletonLink*> m_links;

    //! List of linear springs connecting vertices together.
    std::list<cGELLinearSpring*> m_linearSprings;

    //! List of deformable vertices.
    std::vector<cGELVertex> m_gelVertices;

    //! If __true__ then display skeleton.
    bool m_showSkeletonModel;

    //! If __true__ then display mass particle model.
    bool m_showMassParticleModel;

    //! If __true__ then use skeleton model.
    bool m_useSkeletonModel;

    //! If __true__ then use vertex mass particle model.
    bool m_useMassParticleModel;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

private:

    //! This method initializes the deformable mesh.
    void initialise();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
