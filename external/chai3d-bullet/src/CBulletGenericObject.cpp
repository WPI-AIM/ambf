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
    \contributor Adnan Munawar
    \contributor <amunawar@wpi.edu>
    \version   3.2.0 $Rev: 2181 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletGenericObject.h"
//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This method initializes the Bullet dynamic model.

    \param  a_world  Bullet world to which this new object belongs to.
*/
//==============================================================================
void cBulletGenericObject::initialize(cBulletWorld* a_world)
{
    //! init variables
    m_dynamicWorld = NULL;
    m_bulletRigidBody = NULL;
    m_bulletSoftBody = NULL;
    m_bulletCollisionShape = NULL;
    m_bulletMotionState = NULL;
    m_inertia.zero();
    m_mass = 0;
    m_static = true;

    // store reference to bullet world
    m_dynamicWorld = a_world;

    // add body to world
    m_dynamicWorld->m_bodies.push_back(this);

    // create motion state
    m_bulletMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

    // set inertial offset transform to origin
    m_T_iINb.setIdentity();
}


//==============================================================================
/*!
    This method initializes the Bullet dynamic model.
*/
//==============================================================================
cBulletGenericObject::~cBulletGenericObject()
{
    if (m_bulletRigidBody)
    {
        delete m_bulletRigidBody;
    }
    if (m_bulletSoftBody)
    {
        delete m_bulletSoftBody;
    }
    if (m_bulletCollisionShape)
    {
        delete m_bulletCollisionShape;
    }
    if (m_bulletMotionState)
    {
        delete m_bulletMotionState;
    }
}


//==============================================================================
/*!
    This method assigns a mass value to this object.

    \param  a_mass  Mass value.
*/
//==============================================================================
void cBulletGenericObject::setMass(const double a_mass)
{
    m_mass = a_mass;
}


//==============================================================================
/*!
    This method assigns inertia properties to this object.

    \param  a_inertia  Inertia properties.
*/
//==============================================================================
void cBulletGenericObject::setInertia(const cVector3d& a_inertia)
{
    m_inertia = a_inertia;
}


//==============================================================================
/*!
    This method assigns Inertial Offset Transform.

    \param  a_trans  Inertial Offset Transform.
*/
//==============================================================================
void cBulletGenericObject::setInertialOffsetTransform(btTransform & a_trans)
{
    m_T_iINb = a_trans;
    // Compute the inverse transform to save computation
    m_T_bINi = m_T_iINb.inverse();
}


//==============================================================================
/*!
    This method gives the Inertial Offset Transform.

    \param  a_trans  Inertial Offset Transform.
*/
//==============================================================================
btTransform cBulletGenericObject::getInertialOffsetTransform()
{
    return m_T_iINb;
}


//==============================================================================
/*!
    This method gives the Inverse Inertial Offset Transform.

    \param  a_trans  Inverse Inertial Offset Transform.
*/
//==============================================================================
btTransform cBulletGenericObject::getInverseInertialOffsetTransform()
{
    return m_T_bINi;
}


//==============================================================================
/*!
    This method assigns mass and inertia properties to this object.

    \param  a_mass     Mass value.
    \param  a_inertia  Inertia properties.
*/
//==============================================================================
void cBulletGenericObject::setMassInertia(const double a_mass, const cVector3d& a_inertia)
{
    setMass(a_mass);
    setInertia(a_inertia);
}


//==============================================================================
/*!
    This method estimates the inertia properties for this object. Make sure
    to define its mass beforehand.
*/
//==============================================================================
void cBulletGenericObject::estimateInertia()
{
    if (m_bulletCollisionShape)
    {
        btVector3 inertia(0, 0, 0);
        btScalar mass = m_mass;

        // compute inertia
        m_bulletCollisionShape->calculateLocalInertia(mass, inertia);

        // return value
        m_inertia(0) = inertia[0];
        m_inertia(1) = inertia[1];
        m_inertia(2) = inertia[2];
    }
}


//==============================================================================
/*!
    This method enables of disables the object to be static in the world.

    \param  a_static  If __true__ then object is static, __false__ it is dynamic
    as long as mass and inertia properties have been defined.
*/
//==============================================================================
void cBulletGenericObject::setStatic(bool a_static)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->activate(!a_static);
    }
}


//==============================================================================
/*!
    This method builds a dynamic representation of the object in the Bullet
    world.
*/
//==============================================================================
void cBulletGenericObject::buildDynamicModel()
{
    // get mass and inertia properties
    btVector3 inertia(m_inertia(0), m_inertia(1), m_inertia(2));
    btScalar mass = m_mass;

    // create rigid body
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, m_bulletMotionState, m_bulletCollisionShape, inertia);
    m_bulletRigidBody = new btRigidBody(rigidBodyCI);

    // by default deactivate sleeping mode
    m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_bulletRigidBody->setSleepingThresholds(0, 0);

    // add bullet rigid body to bullet world
    m_dynamicWorld->m_bulletWorld->addRigidBody(m_bulletRigidBody);
}


//==============================================================================
/*!
    This method applies an external force to the object at a given position.
    The relative position and force are expressed in global coordinates.

    \param  a_force        Force vector.
    \param  a_relativePos  Relative position where force is applied.
*/
//==============================================================================
void cBulletGenericObject::addExternalForceAtPoint(const cVector3d& a_force,
                                                   const cVector3d& a_relativePos)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->applyForce(btVector3(a_force(0), a_force(1), a_force(2)),  btVector3(a_relativePos(0), a_relativePos(1), a_relativePos(2)));
    }
}


//==============================================================================
/*!
    This method applies an external force at the center of mass of the object.

    \param  a_force  Force vector.
*/
//==============================================================================
void cBulletGenericObject::addExternalForce(const cVector3d& a_force)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->applyCentralForce(btVector3(a_force(0), a_force(1), a_force(2)));
    }
}


//==============================================================================
/*!
    This method applies an external torque at the center of mass of the object.

    \param  a_torque  Torque vector.
*/
//==============================================================================
void cBulletGenericObject::addExternalTorque(const cVector3d& a_torque)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->applyTorque(btVector3(a_torque(0), a_torque(1), a_torque(2)));
    }
}


//==============================================================================
/*!
    This method assigns a linear and angular damping term to the object.

    \param  a_dampingLin  Linear damping.
    \param  a_dampingAng  Angular damping.
*/
//==============================================================================
void cBulletGenericObject::setDamping(const double& a_dampingLin, const double& a_dampingAng)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->setDamping(a_dampingLin, a_dampingAng);
    }
}


//==============================================================================
/*!
    This method returns the linear damping term for this object. Zero is
    returned if the object has no dynamic model.

    \return Linear damping.
*/
//==============================================================================
double cBulletGenericObject::getDampingLin()
{
    double result = 0.0;
    if (m_bulletRigidBody)
    {
        result = m_bulletRigidBody->getLinearDamping();
    }
    return (result);
}


//==============================================================================
/*!
    This method returns the angular damping term for this object. Zero is
    returned if the object has no dynamic model.

    \return Angular damping.
*/
//==============================================================================
double cBulletGenericObject::getDampingAng()
{
    double result = 0.0;
    if (m_bulletRigidBody)
    {
        result = m_bulletRigidBody->getAngularDamping();
    }
    return (result);
}


//==============================================================================
/*!
    This method assign a coefficient of friction to this object.

    \return Angular damping.
*/
//==============================================================================
void cBulletGenericObject::setSurfaceFriction(const double a_friction)
{
    if (m_bulletRigidBody)
    {
        m_bulletRigidBody->setFriction(a_friction);
    }
}

//==============================================================================
/*!
    This method returns the coefficient of friction for this object. Zero is
    returned if the object has no dynamic model.

    \return Friction coefficient.
*/
//==============================================================================
double cBulletGenericObject::getSurfaceFriction()
{
    double result = 0.0;
    if (m_bulletRigidBody)
    {
        result = m_bulletRigidBody->getFriction();
    }
    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
