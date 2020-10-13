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
    \version   3.2.0 $Rev: 2126 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletGenericObjectH
#define CBulletGenericObjectH
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletGenericObject.h

    \brief
    <b> Bullet Module </b> \n
    Bullet Generic Object.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cBulletWorld;
//------------------------------------------------------------------------------
//! Bullet geometry used for dynamic collision computation.
enum cBulletDynamicModelType
{
    BULLET_MODEL_BOX,
    BULLET_MODEL_SPHERE,
    BULLET_MODEL_CYLINDER,
    BULLET_MODEL_PLANE,
    BULLET_MODEL_TRIMESH
};
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cBulletGenericObjectH
    \ingroup    Bullet

    \brief
    This class implements a base class for modelling Bullet dynamic objects.

    \details
    cBulletGenericBody is a base class for modeling Bullet dynamic objects.
*/
//==============================================================================
class cBulletGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletGenericBody.
    cBulletGenericObject(cBulletWorld* a_world) {
        initialize(a_world); }

    //! Destructor of cBulletGenericBody.
    virtual ~cBulletGenericObject();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //--------------------------------------------------------------------------

public:

    //! This method updates the position and orientation from the Bullet model to CHAI3D model.
    virtual void updateBodyPosition(void) {}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DYNAMIC PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! This method assigns a mass value to this object.
    virtual void setMass(const double a_mass);

    //! This method returns the mass value for this object.
    double getMass() const { return (m_mass); }

    //! This method assigns an inertia properties to this object.
    virtual void setInertia(const cVector3d& a_inertia);

    //! This method returns the inertia properties of this object.
    cVector3d getInertia() { return (m_inertia); }

    //! This method assigns a mass and an inertia to this object.
    virtual void setMassInertia(const double a_mass, const cVector3d& a_inertia);

    //! This method estimates the inertia properties for this object.
    virtual void estimateInertia();

    //! This method enables of disables the object to be static in the world.
    virtual void setStatic(bool a_static);

    //! This method returns __true__ if the object is static, __false__ otherwise.
    virtual bool getStatic() const { return (m_static); }

    //! This method builds a dynamic representation of the object in the Bullet world.
    virtual void buildDynamicModel();

    //! This method updates the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics() {}

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FRICTION PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! This method assign a coefficient of friction to this object.
    void setSurfaceFriction(const double a_friction);

    //! This method returns the coefficient of friction for this object.
    double getSurfaceFriction();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - EXTERNAL FORCES:
    //--------------------------------------------------------------------------

public:

    //! This method applies an external force at the center of mass of this object.
    void addExternalForce(const cVector3d& a_force);

    //! This method applies an external torque at the center of mass of of this object.
    void addExternalTorque(const cVector3d& a_torque);

    //! This method applies an external force at a given position described in global coordinates to this object.
    void addExternalForceAtPoint(const cVector3d& a_force,
                                 const cVector3d& a_relativePos);

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - DAMPING:
    //--------------------------------------------------------------------------

public:

    //! This method assigns a linear and angular damping term to the object.
    void setDamping(const double& a_dampingLin, const double& a_dampingAng);

    //! This method returns the linear damping term for this object.
    double getDampingLin();

    //! This method returns the angular damping term for this object.
    double getDampingAng();

    //! Set the Inertial Offset Transfrom;
    void setInertialOffsetTransform(btTransform &a_trans);

    //! Get the Inertial Offset Transfrom;
    btTransform getInertialOffsetTransform();

    //! Get the inverse inertial offset Transform;
    btTransform getInverseInertialOffsetTransform();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Bullet world.
    cBulletWorld* m_dynamicWorld;

    //! Bullet rigid body.
    btRigidBody* m_bulletRigidBody;

    //! Bullet Soft Body
    btSoftBody* m_bulletSoftBody;

    //! Bullet collision shape
    btCollisionShape* m_bulletCollisionShape;

    //! Bullet motion state.
    btDefaultMotionState* m_bulletMotionState;

    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes the Bullet object.
    virtual void initialize(cBulletWorld* a_world);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then object is static and does not move, __false__ otherwise.
    bool m_static;

    //! Mass property.
    double m_mass;

    //! Inertia properties.
    cVector3d m_inertia;

    //! Command Position from Asynchronous Framework
    cVector3d m_dpos, m_dpos_prev, m_ddpos;
    cMatrix3d m_drot, m_drot_prev, m_ddrot;

private:

    //! Inertial Offset Transform defined in the body frame
    btTransform m_T_iINb;

    //! Body Frame in the Inertial Offset Transform. Inverse of the above transform
    btTransform m_T_bINi;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
