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
    \version   3.2.0 $Rev: 2181 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletStaticPlaneH
#define CBulletStaticPlaneH
//------------------------------------------------------------------------------
#include "CBulletMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletStaticPlane.h

    \brief
    <b> Bullet Module </b> \n 
    Bullet Static Plane.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBulletStaticPlane
    \ingroup    Bullet

    \brief
    This class implements a Bullet static plane.

    \details
    cBulletStaticPlane models a static plane.
*/
//==============================================================================
class cBulletStaticPlane : public cBulletMesh
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletStaticPlane.
    cBulletStaticPlane(cBulletWorld* a_world,
        const cVector3d& a_planeNormal,
        const double& a_planeConstant);

    //! Destructor of cBulletStaticPlane.
    // \todo actually implement destructor
    virtual ~cBulletStaticPlane() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION MODEL:
    //--------------------------------------------------------------------------

public:

    //! This method returns the plane surface normal.
    cVector3d getPlaneNormal() const { return (m_planeNormal); }

    //! This method returns the plane constant.
    double getPlaneConstant() const { return (m_planeConstant); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Plane surface normal.
    cVector3d m_planeNormal;

    //! Plane constant.
    double m_planeConstant;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
