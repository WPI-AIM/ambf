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
    \version   3.2.0 $Rev: 1928 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGenericDemo.h"
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((a_resourceRoot+string(p)).c_str())


//===========================================================================
/*!
    Constructor of cGenericDemo.
*/
//===========================================================================
cGenericDemo::cGenericDemo(const string a_resourceRoot,
                           const int a_numDevices,
                           shared_ptr<cGenericHapticDevice> a_hapticDevice0,
                           shared_ptr<cGenericHapticDevice> a_hapticDevice1)
{
    // display is not mirrored
    m_mirroredDisplay = false;

    // torque gain
    m_torqueGain = 2.0;

    // initialize tool radius
    m_toolRadius = 0.0025;

    // create world
    m_bulletWorld = new cBulletWorld();;

    // set background color
    m_bulletWorld->m_backgroundColor.setWhite();

    // set shadow level
    m_bulletWorld->setShadowIntensity(0.3);

    // create camera
    m_camera = new cCamera(m_bulletWorld);
    m_bulletWorld->addChild(m_camera);

    // position and oriente the camera
    m_camera->set( cVector3d (0.3, 0.0, 0.2),    // camera position (eye)
                   cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                   cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    m_camera->setClippingPlanes(0.01, 4.0);
    m_camera->setUseMultipassTransparency(true);
    m_camera->setStereoEyeSeparation(0.005);
    m_camera->setStereoFocalLength(0.7);

    // create a positional light
    m_light0 = new cSpotLight(m_bulletWorld);
    m_bulletWorld->addChild(m_light0);                      // attach light to camera
    m_light0->setEnabled(true);                       // enable light source
    m_light0->setLocalPos(0.4, 0.4, 0.3);             // position the light source
    m_light0->setDir(-0.4, -0.4, -0.3);               // define the direction of the light beam
    m_light0->m_ambient.set(0.6, 0.6, 0.6);
    m_light0->m_diffuse.set(0.8, 0.8, 0.8);
    m_light0->m_specular.set(0.8, 0.8, 0.8);
    m_light0->m_shadowMap->setEnabled(true);
    m_light0->setCutOffAngleDeg(40);
    m_light0->m_shadowMap->setQualityVeryHigh();
  
    // create a directional light
    cDirectionalLight* m_light1 = new cDirectionalLight(m_bulletWorld);
    m_bulletWorld->addChild(m_light1);                   // attach light to camera
    m_light1->setEnabled(true);                    // enable light source
    m_light1->setDir(-1.0, 0.0, -1.0);             // define the direction of the light beam
    m_light1->m_ambient.set(0.3, 0.3, 0.3);
    m_light1->m_diffuse.set(0.6, 0.6, 0.6);
    m_light1->m_specular.set(0.0, 0.0, 0.0);

    // create a background
    cBackground* background = new cBackground();
    m_camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00, 1.00, 1.00),
                                cColorf(1.00, 1.00, 1.0),
                                cColorf(0.90, 0.90, 0.90),
                                cColorf(0.90, 0.90, 0.90));

    // create a ground
    m_ground = new cMesh();
    m_bulletWorld->addChild(m_ground);
    
    cCreatePlane(m_ground, 0.2, 0.3);
    m_ground->m_material->setGrayLevel(0.5);
    m_ground->m_material->setStiffness(1000);
    m_ground->m_material->setDynamicFriction(0.2);
    m_ground->m_material->setStaticFriction(0.2);
    m_ground->createAABBCollisionDetector(m_toolRadius);
    m_ground->setShowEnabled(false);

    // create a base
    m_base = new cMultiMesh();
    m_bulletWorld->addChild(m_base);
    
    bool fileload = m_base->loadFromFile(RESOURCE_PATH("../resources/models/base/base.obj"));
    if (!fileload)
    {
        fileload = m_base->loadFromFile("../../../bin/resources/models/base/base.obj");
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
    }

    m_base->scale(0.001);
    m_base->setShowFrame(false);
    m_base->setShowBoundaryBox(false);
    m_base->setLocalPos(-0.05, 0.0, 0.001);
    m_base->setUseDisplayList(true);

    // create tools
    m_tools[0] = NULL;
    m_tools[1] = NULL;
    m_tool0 = NULL;
    m_tool1 = NULL;

    m_numTools = cMin(a_numDevices, 2);

    cMesh* mesh = new cMesh();
    cMatrix3d rot;
    rot.identity();
    rot.rotateAboutLocalAxisDeg(cVector3d(1,0,0), 90);
    cCreateRing(mesh, 0.001, 0.005, 4, 16, cVector3d(0,0,0), rot);
    mesh->m_material->setWhite();
    mesh->setTransparencyLevel(0.4f);

    if (m_numTools > 0)
    {
        m_tool0 = new cToolGripper(m_bulletWorld);
        m_bulletWorld->addChild(m_tool0);
        m_tool0->setHapticDevice(shared_ptr<cGenericHapticDevice>(a_hapticDevice0));
        m_tool0->setRadius(m_toolRadius);
        m_tool0->enableDynamicObjects(true);
        m_tool0->setWaitForSmallForce(true);
        m_tool0->setWorkspaceRadius(0.1);
        m_tool0->start();
        m_tools[0] = m_tool0;

        cMesh* mesh0 = mesh->copy();
        m_tool0->m_hapticPointThumb->m_sphereProxy->addChild(mesh0);
        
        cMesh* mesh1 = mesh->copy();
        m_tool0->m_hapticPointFinger->m_sphereProxy->addChild(mesh1);
    }

    if (m_numTools > 1)
    {
        m_tool1 = new cToolGripper(m_bulletWorld);
        m_bulletWorld->addChild(m_tool1);
        m_tool1->setHapticDevice(a_hapticDevice1);
        m_tool1->setRadius(m_toolRadius);
        m_tool1->enableDynamicObjects(true);
        m_tool1->setWaitForSmallForce(true);
        m_tool1->setWorkspaceRadius(0.1);
        m_tool1->start();
        m_tools[1] = m_tool1;

        cMesh* mesh0 = mesh->copy();
        m_tool1->m_hapticPointThumb->m_sphereProxy->addChild(mesh0);

        cMesh* mesh1 = mesh->copy();
        m_tool1->m_hapticPointFinger->m_sphereProxy->addChild(mesh1);
    }

    // set some gravity
    m_bulletWorld->setGravity(cVector3d(0.0, 0.0,-9.8));

    // we create 6 static walls to contains the 3 cubes within a limited workspace
    double d = 0.14 / 2.0;
    double w = 0.20 / 2.0;
    double h = 0.60 / 2.0;

    m_bulletInvisibleWall0 = new cBulletStaticPlane(m_bulletWorld, cVector3d(0.0, 0.0,-1.0),-h);
    m_bulletInvisibleWall1 = new cBulletStaticPlane(m_bulletWorld, cVector3d(0.0, 0.0, 1.0), 0);
    m_bulletInvisibleWall2 = new cBulletStaticPlane(m_bulletWorld, cVector3d(0.0,-1.0, 0.0),-w);
    m_bulletInvisibleWall3 = new cBulletStaticPlane(m_bulletWorld, cVector3d(0.0, 1.0, 0.0),-w);
    m_bulletInvisibleWall4 = new cBulletStaticPlane(m_bulletWorld, cVector3d(-1.0,0.0, 0.0),-d);
    m_bulletInvisibleWall5 = new cBulletStaticPlane(m_bulletWorld, cVector3d( 1.0,0.0, 0.0),-d);
}


//===========================================================================
/*!
    Update graphics.

    \param  a_width  Width of viewport.
    \param  a_height  Height of viewport.
*/
//===========================================================================
void cGenericDemo::updateGraphics(int a_width, int a_height) 
{ 
    // update shadow maps (if any)
    m_bulletWorld->updateShadowMaps(false, m_mirroredDisplay);

    // render view
    m_camera->renderView(a_width, a_height); 
}


//===========================================================================
/*!
    Update haptics.
*/
//===========================================================================
void cGenericDemo::updateHaptics() 
{ 
    // compute global reference frames for each object
    m_bulletWorld->computeGlobalPositions(true);

    // update positions
    for (int i=0; i<m_numTools; i++)
    {
        m_tools[i]->updateFromDevice();
        cMatrix3d rot = m_tools[i]->getDeviceGlobalRot();
        m_tools[i]->m_hapticPointFinger->m_sphereProxy->setLocalRot(rot);
        m_tools[i]->m_hapticPointThumb->m_sphereProxy->setLocalRot(rot);
    }
   
    // compute interaction forces
    for (int i=0; i<m_numTools; i++)
    {
        m_tools[i]->computeInteractionForces();
    }
   
    // apply forces to haptic devices
    for (int i=0; i<m_numTools; i++)
    {
        m_tools[i]->setDeviceGlobalTorque(m_torqueGain * m_tools[i]->getDeviceGlobalTorque());
        m_tools[i]->applyToDevice();
    }

    // apply forces to ODE objects
    for (int i=0; i<m_numTools; i++)
    {
        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        int numInteractionPoints = m_tools[i]->getNumHapticPoints();
        for (int j=0; j<numInteractionPoints; j++)
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = m_tools[i]->getHapticPoint(j);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();
            for (int k=0; k<numContacts; k++)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(k);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to Bullet object
                cBulletGenericObject* bulletobject = dynamic_cast<cBulletGenericObject*>(object);

                // if Bullet object, we apply interaction forces
                if (bulletobject != NULL)
                {
                    const double Kf = 1.0;
                    bulletobject->addExternalForceAtPoint(-Kf * interactionPoint->getLastComputedForce(),
                                                           collisionEvent->m_globalPos - object->getLocalPos());
                }
            }
        }
    }

    // retrieve simulation time and compute next interval
    double time = simClock.getCurrentTimeSeconds();
    double nextSimInterval = cClamp(time, 0.0001, 0.001);

    // reset clock
    simClock.reset();
    simClock.start();

    // update simulation
    m_bulletWorld->updateDynamics(nextSimInterval);
}


//===========================================================================
/*!
    Set device offset.
*/
//===========================================================================
void cGenericDemo::setOffset(double a_offset)
{
    if (m_tool0 != NULL)
    {
        cVector3d pos(0.0, a_offset, 0.0);
        m_tool0->setLocalPos(pos);
    }
    if (m_tool1 != NULL)
    {
        cVector3d pos(0.0,-a_offset, 0.0);
        m_tool1->setLocalPos(pos);
    }
}


//===========================================================================
/*!
    Set torque gain.
*/
//===========================================================================
void cGenericDemo::setTorqueGain(double a_torqueGain)
{
    m_torqueGain = a_torqueGain;
}
