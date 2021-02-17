//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
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
    \contributor <amunawar@wpi.edu>
    \contributor Adnan Munawar
    \version   3.2.1 $Rev: 2017 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletMultiMeshH
#define CBulletMultiMeshH
//------------------------------------------------------------------------------
#include "CBulletGenericObject.h"
#include "chai3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletMultiMesh.h

    \brief
    <b> Bullet Module </b> \n 
    Bullet MultiMesh Object.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBulletMultiMesh
    \ingroup    Bullet

    \brief
    This class implements a Bullet dynamic multi-mesh.

    \details
    cBulletMesh models a dynamic multi-mesh object.
*/
//==============================================================================
class cBulletMultiMesh : public cMultiMesh, public cBulletGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletMultiMesh.
    cBulletMultiMesh(cBulletWorld* a_world) : cBulletGenericObject(a_world), cMultiMesh() {
    }

    //! Destructor of cBulletMultiMesh.
    virtual ~cBulletMultiMesh() {};


    //--------------------------------------------------------------------------
    // IMPORT:
    //--------------------------------------------------------------------------

public:

    //! import base class overloaded virtual and non-virtual methods
    using cGenericObject::setLocalPos;
    using cGenericObject::setLocalRot;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //--------------------------------------------------------------------------

public:

    //! This method sets the local position of object.
    virtual void setLocalPos(const cVector3d& a_position);

    //! This method sets the orientation of this object.
    virtual void setLocalRot(const cMatrix3d& a_rotation);

    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CONTACT MODEL:
    //--------------------------------------------------------------------------

public:

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactConvexTriangles(const double a_margin = 0.01, cMultiMesh *lowResMesh = NULL);

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactTriangles(const double a_margin = 0.01, cMultiMesh *lowResMesh = NULL) ;

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactHull(const double a_margin = 0.01, cMultiMesh *lowResMesh = NULL);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
