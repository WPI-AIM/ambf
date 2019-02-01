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
    \version   3.2.0 $Rev: 2166 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletVehicle.h"
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
    Constructor of cBulletVehicle.

    \param  a_world   Bullet world to which object belongs.
*/
//==============================================================================
cBulletVehicle::cBulletVehicle(cBulletWorld* a_world) : cBulletGenericObject(a_world), cMultiMesh()
{ 
    // default values
    m_mass = 300.0;

    m_suspensionStiffness = 180.0;
    m_suspensionCompression = 8.5;
    m_suspensionDamping = 2.3;
    m_suspensionRestLength = 0.5;
    m_maxSuspensionTravelCm = 0.8;
    m_frictionSlip = 10.0;
    m_wheelWidth = 0.2;
    m_wheelRadius = 0.25;
    m_rollInfluence = 0.1;

    m_wheelDirectionCS.set(0.0, 0.0,-1.0);
    m_wheelAxleCS.set(1.0, 0.0, 0.0);
    m_wheelConnectionPointCS.set(1.0, 1.0, 0.5);

    // create 4 multi-mesh objects for each wheel
    meshWheelFL = new cMultiMesh();
    a_world->addChild(meshWheelFL);

    meshWheelFR = new cMultiMesh();
    a_world->addChild(meshWheelFR);

    meshWheelRL = new cMultiMesh();
    a_world->addChild(meshWheelRL);

    meshWheelRR = new cMultiMesh();
    a_world->addChild(meshWheelRR);
}


//==============================================================================
/*!
    This method applies an engine force to a selected wheel of the vehicle.

    \param  a_engineForce  Engine force to be applied to wheel.
    \param  a_wheelIndex   Selected wheel index.
*/
//==============================================================================
void cBulletVehicle::applyEngineForce(const double a_engineForce, const int a_wheelIndex)
{
    if (m_bulletVehicle)
    {
        m_bulletVehicle->applyEngineForce(a_engineForce, a_wheelIndex);
    }
}


//==============================================================================
/*!
    This method applies a braking command to a selected wheel.

    \param  a_brake       Brake command.
    \param  a_wheelIndex  Selected wheel index.
*/
//==============================================================================
void cBulletVehicle::setBrake(const double a_brake, const int a_wheelIndex)
{
    if (m_bulletVehicle)
    {
        m_bulletVehicle->setBrake(a_brake, a_wheelIndex);
    }
}


//==============================================================================
/*!
    This method applies a braking command to a selected wheel.

    \param  a_steering    Steering command.
    \param  a_wheelIndex  Selected wheel index.
*/
//==============================================================================
void cBulletVehicle::setSteeringValue(const double a_steering, const int a_wheelIndex)
{
    if (m_bulletVehicle)
    {
        m_bulletVehicle->setSteeringValue(a_steering, a_wheelIndex);
    }
}

//==============================================================================
/*!
    This method updates the position and orientation data from the Bullet 
    representation to the CHAI3D representation.
*/
//==============================================================================
void cBulletVehicle::updatePositionFromDynamics()
{
    if (m_bulletRigidBody)
    {
        // get transformation matrix of object
        btTransform trans;
        m_bulletRigidBody->getMotionState()->getWorldTransform(trans);

        btVector3 pos = trans.getOrigin();
        btQuaternion q = trans.getRotation();

       // set new position
        m_localPos.set(pos[0],pos[1],pos[2]);

        // set new orientation
        cQuaternion quaternion(q.getW(), q.getX(), q.getY(), q.getZ());
        quaternion.toRotMat(m_localRot);

        // orthogonalize frame
        m_localRot.orthogonalize();

        // update wheels
        for (int i = 0; i < m_bulletVehicle->getNumWheels(); i++)
        {
            btWheelInfo& wheel = m_bulletVehicle->getWheelInfo(i);
            btTransform trans = wheel.m_worldTransform;

            btVector3 pos = trans.getOrigin();
            btQuaternion q = trans.getRotation();

           // set new position
            cVector3d localPos(pos[0],pos[1],pos[2]);

            // set new orientation
            cMatrix3d localRot;
            cQuaternion quaternion(q.getW(), q.getX(), q.getY(), q.getZ());
            quaternion.toRotMat(localRot);

            if (i == 0)
            {
                meshWheelFL->setLocalPos(localPos);
                meshWheelFL->setLocalRot(localRot);
            }
            if (i == 1)
            {
                meshWheelFR->setLocalPos(localPos);
                meshWheelFR->setLocalRot(localRot);
            }
            if (i == 2)
            {
                meshWheelRL->setLocalPos(localPos);
                meshWheelRL->setLocalRot(localRot);
            }
            if (i == 3)
            {
                meshWheelRR->setLocalPos(localPos);
                meshWheelRR->setLocalRot(localRot);
            }
        }

    }
}


//==============================================================================
/*!
    This method builds a dynamic representation of the object in the Bullet 
    world.
*/
//==============================================================================
void cBulletVehicle::buildDynamicModel()
{
    ////////////////////////////////////////////////////////////////////////////
    // SETUP CHASSIS
    ////////////////////////////////////////////////////////////////////////////
    cMesh* mesh;

    // build chassis
    mesh = newMesh();
    cCreateBox(mesh, 1.6 * m_wheelConnectionPointCS(0), 2.0 * m_wheelConnectionPointCS(1), 0.2 * m_wheelRadius);

    // build wheels
    mesh = meshWheelFL->newMesh();
    cCreateCylinder(mesh, m_wheelWidth, m_wheelRadius, 32, 1, 1, true, true, cVector3d(-0.5 * m_wheelWidth, 0, 0), cMatrix3d(0, 90, 0, C_EULER_ORDER_XYZ, false, true));
    mesh = meshWheelFR->newMesh();
    cCreateCylinder(mesh, m_wheelWidth, m_wheelRadius, 32, 1, 1, true, true, cVector3d(-0.5 * m_wheelWidth, 0, 0), cMatrix3d(0, 90, 0, C_EULER_ORDER_XYZ, false, true));
    mesh = meshWheelRL->newMesh();
    cCreateCylinder(mesh, m_wheelWidth, m_wheelRadius, 32, 1, 1, true, true, cVector3d(-0.5 * m_wheelWidth, 0, 0), cMatrix3d(0, 90, 0, C_EULER_ORDER_XYZ, false, true));
    mesh = meshWheelRR->newMesh();
    cCreateCylinder(mesh, m_wheelWidth, m_wheelRadius, 32, 1, 1, true, true, cVector3d(-0.5 * m_wheelWidth, 0, 0), cMatrix3d(0, 90, 0, C_EULER_ORDER_XYZ, false, true));

    // update bounding box
    updateBoundaryBox();

    // create collision model
    btVector3 boxHalfExtents(0.5 * (m_boundaryBoxMax(0) - m_boundaryBoxMin(0)),
                             0.5 * (m_boundaryBoxMax(1) - m_boundaryBoxMin(1)),
                             0.5 * (m_boundaryBoxMax(2) - m_boundaryBoxMin(2)));
    m_bulletCollisionShape = new btBoxShape(boxHalfExtents);

    // get mass and inertia properties
    estimateInertia();
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

    m_dynamicWorld->setShowFrame(true, true);
    m_dynamicWorld->setFrameSize(0.5, true);

    ////////////////////////////////////////////////////////////////////////////
    // SETUP VEHICLE
    ////////////////////////////////////////////////////////////////////////////

    // vehicle tuning
    m_bulletTuning.m_suspensionStiffness =  m_suspensionStiffness;
    m_bulletTuning.m_suspensionCompression = m_suspensionCompression * 2 * btSqrt(m_suspensionStiffness);;
    m_bulletTuning.m_suspensionDamping = m_suspensionDamping;
    m_bulletTuning.m_maxSuspensionTravelCm = m_maxSuspensionTravelCm;
    m_bulletTuning.m_frictionSlip = m_frictionSlip;
    
    // create vehicle ray caster
    m_bulletVehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicWorld->getBulletWorld());

    // create vehicle model
    m_bulletVehicle = new btRaycastVehicle(m_bulletTuning, m_bulletRigidBody, m_bulletVehicleRayCaster);

    // set coordinate system
    m_bulletVehicle->setCoordinateSystem(0, 2, 1);

    // add vehicle to world
    m_dynamicWorld->m_bulletWorld->addVehicle(m_bulletVehicle);

    ////////////////////////////////////////////////////////////////////////////
    // SETUP WHEELS
    ////////////////////////////////////////////////////////////////////////////
    
    // https://github.com/bulletphysics/bullet3/pull/448/files
    
    // declare variables
    btVector3 connectionPointCS, wheelDirectionCS, wheelAxleCS;
    btScalar suspensionRestLength, wheelRadius;

    wheelDirectionCS = btVector3(m_wheelDirectionCS(0), m_wheelDirectionCS(1), m_wheelDirectionCS(2));
    wheelAxleCS = btVector3(m_wheelAxleCS(0), m_wheelAxleCS(1), m_wheelAxleCS(2));
    suspensionRestLength = m_suspensionRestLength;
    wheelRadius = m_wheelRadius;

    // Setup front-left wheel
    connectionPointCS = btVector3(m_wheelConnectionPointCS(0), m_wheelConnectionPointCS(1), m_wheelConnectionPointCS(2));
    m_bulletVehicle->addWheel(connectionPointCS, wheelDirectionCS, wheelAxleCS, suspensionRestLength, wheelRadius, m_bulletTuning, true);

    // Setup front-right wheel
    connectionPointCS = btVector3(-m_wheelConnectionPointCS(0), m_wheelConnectionPointCS(1), m_wheelConnectionPointCS(2));
    m_bulletVehicle->addWheel(connectionPointCS, wheelDirectionCS, wheelAxleCS, suspensionRestLength, wheelRadius, m_bulletTuning, true);

    // Setup rear-left wheel
    connectionPointCS = btVector3(m_wheelConnectionPointCS(0),-m_wheelConnectionPointCS(1), m_wheelConnectionPointCS(2));
    m_bulletVehicle->addWheel(connectionPointCS, wheelDirectionCS, wheelAxleCS, suspensionRestLength, wheelRadius, m_bulletTuning, false);

    // Setup rear-right wheel
    connectionPointCS = btVector3(-m_wheelConnectionPointCS(0),-m_wheelConnectionPointCS(1), m_wheelConnectionPointCS(2));
    m_bulletVehicle->addWheel(connectionPointCS, wheelDirectionCS, wheelAxleCS, suspensionRestLength, wheelRadius, m_bulletTuning, false);

    //Configures each wheel of our vehicle, setting its friction, damping compression, etc.
    //For more details on what each parameter does, refer to the docs
    for (int i = 0; i < m_bulletVehicle->getNumWheels(); i++)
    {
        btWheelInfo& wheel = m_bulletVehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = 10*m_suspensionStiffness;
        wheel.m_wheelsDampingCompression = btScalar(0.3) * 2 * btSqrt(m_suspensionStiffness);//btScalar(0.8);
        wheel.m_wheelsDampingRelaxation = btScalar(0.5) * 2 * btSqrt(m_suspensionStiffness);//1;
        //Larger friction slips will result in better handling
        wheel.m_frictionSlip = m_frictionSlip;
        wheel.m_rollInfluence = m_rollInfluence;
        wheel.m_maxSuspensionForce = 100000;
    }


    /*
    btVector3* halfExtents,

    //The direction of the raycast, the btRaycastVehicle uses raycasts instead of simiulating the wheels with rigid bodies
    btVector3 wheelDirectionCS0(0, -1, 0);

    //The axis which the wheel rotates arround
    btVector3 wheelAxleCS(-1, 0, 0);
    
    btScalar suspensionRestLength(0.7);
    
    btScalar wheelWidth(0.4);
    
    btScalar wheelRadius(0.5);
    
    //The height where the wheels are connected to the chassis
    btScalar connectionHeight(1.2); 

    //All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
    btVector3 wheelConnectionPoint(halfExtents->x() - wheelRadius, connectionHeight, halfExtents->z() - wheelWidth);
    
    //Adds the front wheels
    vehicle->addWheel(wheelConnectionPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);
    
    vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);
    
    //Adds the rear wheels
    vehicle->addWheel(wheelConnectionPoint* btVector3(1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);
    
    vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);
    
    //Configures each wheel of our vehicle, setting its friction, damping compression, etc.
    //For more details on what each parameter does, refer to the docs
    for (int i = 0; i < vehicle->getNumWheels(); i++)
    {
        btWheelInfo& wheel = vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = 50;
        wheel.m_wheelsDampingCompression = btScalar(0.3) * 2 * btSqrt(wheel.m_suspensionStiffness);//btScalar(0.8);
        wheel.m_wheelsDampingRelaxation = btScalar(0.5) * 2 * btSqrt(wheel.m_suspensionStiffness);//1;
        //Larger friction slips will result in better handling
        wheel.m_frictionSlip = btScalar(1.2);
        wheel.m_rollInfluence = 1;
    }



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
    */

    /*
    m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
    m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

    ///never deactivate the vehicle
    m_carChassis->setActivationState(DISABLE_DEACTIVATION);
    m_dynamicsWorld->addVehicle(m_vehicle);
    float connectionHeight = -18.2f;
    bool isFrontWheel=true;


    //choose coordinate system
    m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#ifdef FORCE_ZAXIS_UP
    btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
    btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(25.3),connectionHeight,10*CUBE_HALF_EXTENTS);
#endif


    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(25.3),connectionHeight,10*CUBE_HALF_EXTENTS);
#endif


    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(25.3),connectionHeight,-10*CUBE_HALF_EXTENTS);
#endif //FORCE_ZAXIS_UP
    isFrontWheel = false;
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
#ifdef FORCE_ZAXIS_UP
    connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
    connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(25.3),connectionHeight,-10*CUBE_HALF_EXTENTS);
#endif
    m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

    for (int i=0;i<m_vehicle->getNumWheels();i++)
    {
        btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = suspensionDamping;
        wheel.m_wheelsDampingCompression = suspensionCompression;
        wheel.m_frictionSlip = wheelFriction;
        wheel.m_rollInfluence = rollInfluence;
    }

    */
}

/*

btTransform tr;
tr.setIdentity();
btAlignedObjectArray<btCollisionShape*> m_collisionShapes;


#ifdef FORCE_ZAXIS_UP
//   indexRightAxis = 0; 
//   indexUpAxis = 2; 
//   indexForwardAxis = 1; 
btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
btCompoundShape* compound = new btCompoundShape();
btTransform localTrans;
localTrans.setIdentity();
//localTrans effectively shifts the center of mass with respect to the chassis
localTrans.setOrigin(btVector3(0,0,1));
#else
btCollisionShape* chassisShape = new btBoxShape(btVector3(25.0f, 15.0f, 15.0f));
m_collisionShapes.push_back(chassisShape);


btCompoundShape* compound = new btCompoundShape();
m_collisionShapes.push_back(compound);
btTransform localTrans;
localTrans.setIdentity();
//localTrans effectively shifts the center of mass with respect to the chassis
localTrans.setOrigin(btVector3(0,1,0));
#endif

compound->addChildShape(localTrans,chassisShape);
tr.setOrigin(btVector3(0,0.f,0));


m_carChassis = localCreateRigidBody(800.0f,tr,compound);//chassisShape); 800.0f
//m_carChassis->setDamping(0.2,0.2);


m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
*/



//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
