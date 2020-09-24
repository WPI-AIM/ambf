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
    \version   3.2.0 $Rev: 2126 $
*/
//==============================================================================
//------------------------------------------------------------------------------
#ifndef CBulletWorldH
#define CBulletWorldH
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "CBulletGenericObject.h"
//------------------------------------------------------------------------------
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBulletWorld.h

    \brief
    Implementation of a Bullet world.
*/
//==============================================================================


//==============================================================================
/*!
    \class      cBulletWorld
    \ingroup    Bullet

    \brief
    This class implements an Bullet dynamic world.

    \details
    cBulletWorld implements a virtual world to handle Bullet based objects
    (cBulletGenericBody).
*/
//==============================================================================
class cBulletWorld : public chai3d::cWorld
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletWorld.
    cBulletWorld();

    //! Destructor of cBulletWorld.
    virtual ~cBulletWorld();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! List of all Bullet dynamic bodies in world.
    std::list<cBulletGenericObject*> m_bodies;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns a point to the Bullet world object.
    btDiscreteDynamicsWorld* getBulletWorld() { return (m_bulletWorld); }

    //! This method assigns integration settings of simulator.
    void setIntegrationSettings(const double a_integrationTimeStep = 0.001, const int a_integrationMaxIterations = 1) { setIntegrationTimeStep(a_integrationTimeStep); setIntegrationMaxIterations(a_integrationMaxIterations); }

    //! This method sets the internal integration time step of the simulation.
    void setIntegrationTimeStep(const double a_integrationTimeStep = 0.001) { m_integrationTimeStep = chai3d::cMax(a_integrationTimeStep, 0.000001); }

    //! The method returns the integration time step of the simulation.
    double getIntegrationTimeStep() { return (m_integrationTimeStep); }

    //! This method sets the maximum number of iteration per integration time step.
    void setIntegrationMaxIterations(const int a_integrationMaxIterations = 1) { m_integrationMaxIterations = chai3d::cMax(a_integrationMaxIterations, 1); }

    //! This method returns the maximum number of iteration per integration time step.
    int getIntegrationMaxIterations() { return (m_integrationMaxIterations); }

    //! This method sets the gravity field vector.
    void setGravity(const double& a_x, const double& a_y, const double& a_z) { setGravity(cVector3d(a_x, a_y, a_z)); }

    //! This method sets the gravity field vector.
    void setGravity(const cVector3d& a_gravity);

    //! This method returns the gravity field vector.
    chai3d::cVector3d getGravity();

    //! This method updates the simulation over a time interval.
    virtual void updateDynamics(double a_interval, double a_wallClock=0, double a_loopFreq = 0, int a_numDevices = 0);

    //! This method updates the position and orientation from Bullet models to CHAI3D models.
    virtual void updatePositionFromDynamics(void);

    //! This method returns the current simulation time
    double getWallTime(void);

    //! This method returns the current simulation time
    double getSimulationTime(void);

    //! This method gets the time difference between current time and last simulation time
    double getSimulationDeltaTime();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Bullet dynamics world.
    btDiscreteDynamicsWorld* m_bulletWorld;

    //! Bullet broad phase collision detection algorithm.
    btBroadphaseInterface* m_bulletBroadphase;

    //! Bullet collision configuration.
    btCollisionConfiguration* m_bulletCollisionConfiguration;

    //! Bullet collision dispatcher.
    btCollisionDispatcher* m_bulletCollisionDispatcher;

    //! Bullet physics solver.
    btConstraintSolver* m_bulletSolver;

    //! Bullet Softbody World Info
    btSoftBodyWorldInfo* m_bulletSoftBodyWorldInfo;

    //! Bullet Soft Body Solver
    btSoftBodySolver* m_bulletSoftBodySolver;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Current time of simulation.
    double m_simulationTime;

    //! Integration time step.
    double m_integrationTimeStep;

    //! Wall Clock in Secs
    double m_wallClock;

    //! Last Simulation Time
    double m_lastSimulationTime;

    //! Maximum number of iterations.
    int m_integrationMaxIterations;

};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
