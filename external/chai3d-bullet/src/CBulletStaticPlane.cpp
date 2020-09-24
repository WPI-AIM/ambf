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
    \contributor <amunawar@wpi.edu>
    \contributor Adnan Munawar
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletStaticPlane.h"
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
    Constructor of cBulletStaticPlane.

    \param  a_world          Bullet world to which object belongs.
    \param  a_planeNormal    Surface normal of plane.
    \param  a_planeConstant  Distance of plane from origin.
*/
//==============================================================================
cBulletStaticPlane::cBulletStaticPlane(cBulletWorld* a_world,
    const cVector3d& a_planeNormal,
    const double& a_planeConstant) : cBulletMesh(a_world)
{ 
    // set dimensions
    m_planeNormal = a_planeNormal;
    m_planeConstant = a_planeConstant;

    // sanity check
    if (m_planeNormal.length() == 0)
    { 
        m_planeNormal.set(0,0,1);
    }
    else
    {
        m_planeNormal.normalize();
    }

    // object is static
    m_static = true;

    // create collision model
    m_bulletCollisionShape = new btStaticPlaneShape(btVector3(m_planeNormal(0), m_planeNormal(1), m_planeNormal(2)), m_planeConstant);

    // create rigid body
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, m_bulletMotionState, m_bulletCollisionShape, btVector3(0, 0, 0));
    m_bulletRigidBody = new btRigidBody(rigidBodyCI);

    // add bullet rigid body to bullet world
    m_dynamicWorld->m_bulletWorld->addRigidBody(m_bulletRigidBody);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

