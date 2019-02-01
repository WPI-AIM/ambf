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
    \version   3.2.0 $Rev: 1925 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CDemo2.h"
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDemo2.
*/
//===========================================================================
cDemo2::cDemo2(const string a_resourceRoot,
               const int a_numDevices,
               shared_ptr<cGenericHapticDevice> a_hapticDevice0,
               shared_ptr<cGenericHapticDevice> a_hapticDevice1):cGenericDemo(a_resourceRoot, a_numDevices, a_hapticDevice0, a_hapticDevice1)
{
    cMaterial matBase;
    matBase.setGrayLevel(0.3);
    matBase.setStiffness(1500);

    m_ODEBase0 = new cBulletCylinder(m_bulletWorld, 0.075, 0.005);
    m_bulletWorld->addChild(m_ODEBase0);
    m_ODEBase0->createAABBCollisionDetector(m_toolRadius);
    m_ODEBase0->setMaterial(matBase);
    m_ODEBase0->buildDynamicModel();
    m_ODEBase0->setLocalPos(0.015, 0.0, 0.0375);

    m_ODEBase1 = new cBulletCylinder(m_bulletWorld, 0.075, 0.005);
    m_bulletWorld->addChild(m_ODEBase1);
    m_ODEBase1->createAABBCollisionDetector(m_toolRadius);
    m_ODEBase1->setMaterial(matBase);
    m_ODEBase1->buildDynamicModel();
    m_ODEBase1->setLocalPos(-0.015, -0.012, 0.0375);

    m_ODEBase2 = new cBulletCylinder(m_bulletWorld, 0.075, 0.005);
    m_bulletWorld->addChild(m_ODEBase2);
    m_ODEBase2->createAABBCollisionDetector(m_toolRadius);
    m_ODEBase2->setMaterial(matBase);
    m_ODEBase2->buildDynamicModel();
    m_ODEBase2->setLocalPos(-0.015, 0.012, 0.0375);


    // create a new ODE object that is automatically added to the ODE world
    double radius = 0.012;

    m_ODEBody0 = new cBulletSphere(m_bulletWorld, radius);
    m_bulletWorld->addChild(m_ODEBody0);

    m_ODEBody1 = new cBulletSphere(m_bulletWorld, radius);
    m_bulletWorld->addChild(m_ODEBody1);

    m_ODEBody2 = new cBulletSphere(m_bulletWorld, radius);
    m_bulletWorld->addChild(m_ODEBody2);

    m_ODEBody0->createAABBCollisionDetector(m_toolRadius);
    m_ODEBody1->createAABBCollisionDetector(m_toolRadius);
    m_ODEBody2->createAABBCollisionDetector(m_toolRadius);

    // define some material properties for each cube
    cMaterial mat;
    mat.setRedIndian();
    mat.m_specular.set(0.0, 0.0, 0.0);
    mat.setStiffness(1000);
    mat.setDynamicFriction(0.9);
    mat.setStaticFriction(0.9);
    m_ODEBody0->setMaterial(mat);
    m_ODEBody1->setMaterial(mat);
    m_ODEBody2->setMaterial(mat);

    // define some mass properties for each cube
    m_ODEBody0->setMass(0.02);
    m_ODEBody1->setMass(0.02);
    m_ODEBody2->setMass(0.02);

    // estimate their inertia properties
    m_ODEBody0->estimateInertia();
    m_ODEBody1->estimateInertia();
    m_ODEBody2->estimateInertia();

    // create dynamic models
    m_ODEBody0->buildDynamicModel();
    m_ODEBody1->buildDynamicModel();
    m_ODEBody2->buildDynamicModel();

    // initialize
    init();
};


//===========================================================================
/*!
    Set stiffness of environment

    \param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo2::setStiffness(double a_stiffness)
{
    // set ground
    m_ground->setStiffness(a_stiffness, true);
    
    // set objects
    m_ODEBody0->setStiffness(a_stiffness);
    m_ODEBody1->setStiffness(a_stiffness);
    m_ODEBody2->setStiffness(a_stiffness);

    m_ODEBase0->setStiffness(a_stiffness);
    m_ODEBase1->setStiffness(a_stiffness);
    m_ODEBase2->setStiffness(a_stiffness);
};


//===========================================================================
/*!
    Initialize position of objects
*/
//===========================================================================
void cDemo2::init()
{
    m_ODEBody0->setLocalPos(0.04,-0.05, 0.04);
    m_ODEBody1->setLocalPos(0.04, 0.00, 0.04);
    m_ODEBody2->setLocalPos(0.04, 0.05, 0.04);
}
