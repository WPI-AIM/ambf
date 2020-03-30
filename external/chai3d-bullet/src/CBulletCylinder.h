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
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletCylinderH
#define CBulletCylinderH
//------------------------------------------------------------------------------
#include "CBulletMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletCylinder.h

    \brief
    <b> Bullet Module </b> \n 
    Bullet Cylinder Object.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBulletCylinder
    \ingroup    Bullet

    \brief
    This class implements a Bullet dynamic cylinder.

    \details
    cBulletCylinder models a dynamic cylinder.
*/
//==============================================================================
class cBulletCylinder : public cBulletMesh
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletCylinder.
    cBulletCylinder(cBulletWorld* a_world,
        const double& a_height, 
        const double& a_radius);

    //! Destructor of cBulletBox.
    virtual ~cBulletCylinder() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION MODEL:
    //--------------------------------------------------------------------------

public:

    //! This method returns the height of the cylinder.
    inline double getHeight() const { return (m_height); }

    //! This method returns the radius of the cylinder.
    inline double getRadius() const { return (m_radius); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Height of cylinder.
    double m_height;

    //! Radius of cylinder.
    double m_radius;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
