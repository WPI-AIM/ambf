//===========================================================================
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
#include "CBulletBox.h"
//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cBulletBox.

    \param  a_world  Bullet world to which object belongs.
    \param  a_sizeX  Size of box along X axis.
    \param  a_sizeY  Size of box along Y axis.
    \param  a_sizeZ  Size of box along Z axis.
*/
//==============================================================================
cBulletBox::cBulletBox(cBulletWorld* a_world,
    const double& a_sizeX,
    const double& a_sizeY,
    const double& a_sizeZ) : cBulletMesh(a_world)
{ 
    // set dimensions
    m_sizeX = cAbs(a_sizeX);
    m_sizeY = cAbs(a_sizeY);
    m_sizeZ = cAbs(a_sizeZ);

    // create object
    cCreateBox(this, a_sizeX, a_sizeY, a_sizeZ);

    // create display list
    setUseDisplayList(true);

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    markForUpdate(false);

    // create collision model
    btVector3 boxHalfExtents(0.5 * m_sizeX, 0.5 * m_sizeY, 0.5 * m_sizeZ);
    m_bulletCollisionShape = new btBoxShape(boxHalfExtents);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
