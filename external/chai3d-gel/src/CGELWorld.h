//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
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
    \version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGELWorldH
#define CGELWorldH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGELMesh.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELWorld.h

    \brief
    Implementation of a virtual world that manages deformable objects.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELWorld
    \ingroup    GEL

    \brief
    This class implements a virtual world that manages deformable objects.

    \details
    cGELWorld implements a virtual world that simulates deformable objects.
*/
//===========================================================================
class cGELWorld : public chai3d::cGenericObject
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELWorld.
    cGELWorld();

    //! Destructor of cGELWorld.
    virtual ~cGELWorld();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method integrates the dynamic simulation over a time period passed as argument.
    void updateDynamics(double a_timeInterval);

    //! This method clears all external forces applied on all deformable objects.
    void clearExternalForces();

    //! This method updates the mesh of all deformable objects.
    void updateSkins(bool a_updateNormals = true);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! List of deformable solids.
    std::list<cGELMesh*> m_gelMeshes;

    //! Current time of simulation.
    double m_simulationTime;

    //! Gravity constant.
    chai3d::cVector3d m_gravity;


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
  
private:

    //! This method renders graphically all deformable objects contained in the world.
    virtual void render(chai3d::cRenderOptions& a_options);
};


//===========================================================================
/*!
    \class      cGELWorldCollision
    \ingroup    GEL

    \brief
    This class implements a collision detector for class cGELWorld.

    \details
    cGELWorldCollision implements a collision detector that supports
    deformable objects.
*/
//===========================================================================
class cGELWorldCollision : public chai3d::cGenericCollision
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELWorldCollision.
    cGELWorldCollision(cGELWorld* a_gelWorld) {m_gelWorld = a_gelWorld;}

    //! Destructor of cGELWorldCollision.
    virtual ~cGELWorldCollision() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method initialized the collision detector.
    virtual void initialize() {};

    //! This method renders the collision detector graphically.
    virtual void render() {};

    //! This method computes all collisions between a segment passed as argument and the objects in this world.
    virtual bool computeCollision(chai3d::cVector3d& a_segmentPointA,
                                  chai3d::cVector3d& a_segmentPointB,
                                  chai3d::cCollisionRecorder& a_recorder,
                                  chai3d::cCollisionSettings& a_settings);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------

private:

    //! Pointer to the GEL virtual world.
    cGELWorld* m_gelWorld;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


