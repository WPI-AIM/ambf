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
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// GENERAL SETTINGS
//---------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;


//---------------------------------------------------------------------------
// BULLET MODULE VARIABLES
//---------------------------------------------------------------------------

// bullet world
cBulletWorld* bulletWorld;

// bullet objects
cBulletBox* bulletBox0;
cBulletBox* bulletBox1;
cBulletBox* bulletBox2;

// bullet static walls and ground
cBulletStaticPlane* bulletInvisibleWall1;
cBulletStaticPlane* bulletInvisibleWall2;
cBulletStaticPlane* bulletInvisibleWall3;
cBulletStaticPlane* bulletInvisibleWall4;
cBulletStaticPlane* bulletInvisibleWall5;
cBulletStaticPlane* bulletGround;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//===========================================================================
/*
    DEMO:    01-bullet-cube.cpp

    This example illustrates the use of the Bullet framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 3
    blocks to which we assign dynamic properties. Haptic interactions
    are computed and forces are applied to the dynamic models.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 01-bullet-cube" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[g] - Enable/Disable gravity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a dynamic world.
    bulletWorld = new cBulletWorld();

    // set the background color of the environment
    bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(bulletWorld);
    bulletWorld->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d (2.5, 0.0, 0.3),    // camera position (eye)
                cVector3d (0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(bulletWorld);

    // attach light to camera
    bulletWorld->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.0, 1.2);

    // define the direction of the light beam
    light->setDir(0.0, 0.0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (gripper or pointer)
    if (hapticDeviceInfo.m_actuatedGripper)
    {
        tool = new cToolGripper(bulletWorld);
    }
    else
    {
        tool = new cToolCursor(bulletWorld);
    }

    // insert tool into world
    bulletWorld->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the virtual tool contact points (sphere)
    double toolRadius = 0.06;
    tool->setRadius(toolRadius, toolRadius);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);


    //-----------------------------------------------------------------------
    // SETUP BULLET WORLD AND OBJECTS
    //-----------------------------------------------------------------------

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // set some gravity
    bulletWorld->setGravity(0.0, 0.0,-9.8);


    //////////////////////////////////////////////////////////////////////////
    // 3 BULLET BLOCKS
    //////////////////////////////////////////////////////////////////////////
    double size = 0.4;

    // create three objects that are added to the world
    bulletBox0 = new cBulletBox(bulletWorld, size, size, size);
    bulletWorld->addChild(bulletBox0);

    bulletBox1 = new cBulletBox(bulletWorld, size, size, size);
    bulletWorld->addChild(bulletBox1);

    bulletBox2 = new cBulletBox(bulletWorld, size, size, size);
    bulletWorld->addChild(bulletBox2);

    // define some material properties for each cube
    cMaterial mat0, mat1, mat2;
    mat0.setRedIndian();
    mat0.setStiffness(0.3 * maxStiffness);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    bulletBox0->setMaterial(mat0);

    mat1.setBlueRoyal();
    mat1.setStiffness(0.3 * maxStiffness);
    mat1.setDynamicFriction(0.6);
    mat1.setStaticFriction(0.6);
    bulletBox1->setMaterial(mat1);

    mat2.setGreenDarkSea();
    mat2.setStiffness(0.3 * maxStiffness);
    mat2.setDynamicFriction(0.6);
    mat2.setStaticFriction(0.6);
    bulletBox2->setMaterial(mat2);

    // define some mass properties for each cube
    bulletBox0->setMass(0.05);
    bulletBox1->setMass(0.05);
    bulletBox2->setMass(0.05);

    // estimate their inertia properties
    bulletBox0->estimateInertia();
    bulletBox1->estimateInertia();
    bulletBox2->estimateInertia();

    // create dynamic models
    bulletBox0->buildDynamicModel();
    bulletBox1->buildDynamicModel();
    bulletBox2->buildDynamicModel();

    // create collision detector for haptic interaction
    bulletBox0->createAABBCollisionDetector(toolRadius);
    bulletBox1->createAABBCollisionDetector(toolRadius);
    bulletBox2->createAABBCollisionDetector(toolRadius);

    // set friction values
    bulletBox0->setSurfaceFriction(0.4);
    bulletBox1->setSurfaceFriction(0.4);
    bulletBox2->setSurfaceFriction(0.4);

    // set position of each cube
    bulletBox0->setLocalPos(0.0,-0.6, 0.5);
    bulletBox1->setLocalPos(0.0, 0.6, 0.5);
    bulletBox2->setLocalPos(0.0, 0.0, 0.5);

    // rotate central cube 45 degrees around z-axis
    bulletBox0->rotateAboutGlobalAxisDeg(0,0,1, 45);


    //////////////////////////////////////////////////////////////////////////
    // INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 5 static walls to contain the dynamic objects within a limited workspace
    double planeWidth = 1.0;
    bulletInvisibleWall1 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, -1.0), -2.0 * planeWidth);
    bulletInvisibleWall2 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, -1.0, 0.0), -planeWidth);
    bulletInvisibleWall3 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 1.0, 0.0), -planeWidth);
    bulletInvisibleWall4 = new cBulletStaticPlane(bulletWorld, cVector3d(-1.0, 0.0, 0.0), -planeWidth);
    bulletInvisibleWall5 = new cBulletStaticPlane(bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.8 * planeWidth);


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create ground plane
    bulletGround = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, 1.0), -planeWidth);

    // add plane to world as we will want to make it visibe
    bulletWorld->addChild(bulletGround);

    // create a mesh plane where the static plane is located
    cCreatePlane(bulletGround, 3.0, 3.0, bulletGround->getPlaneConstant() * bulletGround->getPlaneNormal());

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(0.3 * maxStiffness);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3);
    bulletGround->setMaterial(matGround);

    // setup collision detector for haptic interaction
    bulletGround->createAABBCollisionDetector(toolRadius);

    // set friction values
    bulletGround->setSurfaceFriction(0.4);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable gravity
    else if (a_key == GLFW_KEY_G)
    {
        if (bulletWorld->getGravity().length() > 0.0)
        {
            bulletWorld->setGravity(0.0, 0.0, 0.0);
        }
        else
        {
            bulletWorld->setGravity(0.0, 0.0,-9.8);
        }
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete bulletWorld;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    bulletWorld->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        bulletWorld->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        int numInteractionPoints = tool->getNumHapticPoints();
        for (int i=0; i<numInteractionPoints; i++)
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(i);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();
            for (int i=0; i<numContacts; i++)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the Bullet object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to Bullet object
                cBulletGenericObject* bulletobject = dynamic_cast<cBulletGenericObject*>(object);

                // if Bullet object, we apply interaction forces
                if (bulletobject != NULL)
                {
                    bulletobject->addExternalForceAtPoint(-interactionPoint->getLastComputedForce(),
                                                           collisionEvent->m_globalPos - object->getLocalPos());
                }
            }
        }

        // update simulation
        bulletWorld->updateDynamics(timeInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

