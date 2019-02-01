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
#ifndef CGELVertexH
#define CGELVertexH
//---------------------------------------------------------------------------
#include "CGELMassParticle.h"
#include "CGELSkeletonLink.h"
#include "CGELSkeletonNode.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELVertex.h

    \brief
    Implementation of a vertex for deformable objects.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELVertex
    \ingroup    GEL

    \brief
    This class implements a vertex for deformable objects.

    \details
    cGELVertex implements a vertex for deformable objects. The class includes
    information that describes the position of the mesh vertex in respect to the
    deformable skeleton that models the physical properties of the object.
*/
//===========================================================================
class cGELVertex
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELVertex.
    cGELVertex(chai3d::cMesh* a_mesh, unsigned int a_vertexIndex);

    //! Destructor of cGELVertex.
    virtual ~cGELVertex();


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Vertex index.
    unsigned int m_vertexIndex;

    //! Mesh object
    chai3d::cMesh* m_mesh;

    //! Mass particle for current vertex.
    cGELMassParticle* m_massParticle;

    //! Skeleton link to which this vertex may be linked to.
    cGELSkeletonLink* m_link;

    //! Skeleton node to which this vertex may be linked to.
    cGELSkeletonNode* m_node;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
