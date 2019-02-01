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
#ifndef CBulletVehicleH
#define CBulletVehicleH
//------------------------------------------------------------------------------
#include "CBulletGenericObject.h"
#include "chai3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletVehicle.h

    \brief
    <b> Bullet Module </b> \n 
    Bullet Vehicle Object.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBulletVehicle
    \ingroup    Bullet

    \brief
    This class implements a Bullet vehicle.

    \details
    cBulletMesh models a vehicle object.
*/
//==============================================================================
class cBulletVehicle : public cMultiMesh, public cBulletGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletVehicle.
    cBulletVehicle(cBulletWorld* a_world);

    //! Destructor of cBulletVehicle.
    virtual ~cBulletVehicle() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method builds a dynamic representation of this vehicle in the Bullet world.
    virtual void buildDynamicModel();

    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();

    //! This method sets the local position of object.
    //virtual void setLocalPos(const chai3d::cVector3d &a_position);

    //! This method sets the orientation of this object.
    //virtual void setLocalRot(const chai3d::cMatrix3d &a_rotation);

    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    //virtual void updatePositionFromDynamics();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - VEHICLE COMMANDS:
    //--------------------------------------------------------------------------

public:

    //! This method applies an engine force to a selected wheel.
    void applyEngineForce(const double a_engineForce, const int a_wheelIndex);

    //! This method applies braking to a selected wheel.
    void setBrake(const double a_brake, const int a_wheelIndex);

    //! This method applies steering command to a selected wheel.
    void setSteeringValue(const double a_steering, const int a_wheelIndex);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBER:
    //--------------------------------------------------------------------------

public:

    btRaycastVehicle::btVehicleTuning m_bulletTuning;
    btVehicleRaycaster* m_bulletVehicleRayCaster;
    btRaycastVehicle*   m_bulletVehicle;
    btCollisionShape*   m_bulletWheelShape;

    double m_suspensionStiffness;
    double m_suspensionCompression;
    double m_suspensionDamping;
    double m_suspensionRestLength;
    double m_maxSuspensionTravelCm;
    double m_frictionSlip;
    double m_wheelWidth;
    double m_wheelRadius;
    double m_rollInfluence;

    cVector3d m_wheelDirectionCS;
    cVector3d m_wheelAxleCS;
    cVector3d m_wheelConnectionPointCS;

    cMultiMesh* meshWheelFL;
    cMultiMesh* meshWheelFR;
    cMultiMesh* meshWheelRL;
    cMultiMesh* meshWheelRR;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
