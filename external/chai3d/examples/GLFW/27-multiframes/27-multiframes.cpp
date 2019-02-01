//==============================================================================
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
    \version   3.2.0 $Rev: 1925 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// four cameras to render the world from different perspectives
cCamera* cameraMain;
cCamera* cameraView1;
cCamera* cameraView2;
cCamera* cameraView3;
cCamera* cameraView4;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a virtual multimesh like object
cMultiMesh* object;

// a font for rendering text
cFontPtr font;

// Four colored background
cBackground* background1;
cBackground* background2;
cBackground* background3;
cBackground* background4;

// Four view panels
cViewPanel* viewPanel1;
cViewPanel* viewPanel2;
cViewPanel* viewPanel3;
cViewPanel* viewPanel4;

// Four framebuffer
cFrameBufferPtr frameBuffer1;
cFrameBufferPtr frameBuffer2;
cFrameBufferPtr frameBuffer3;
cFrameBufferPtr frameBuffer4;

//Four labels to display information
cLabel* labelView1;
cLabel* labelView2;
cLabel* labelView3;
cLabel* labelView4;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// root resource path
string resourceRoot;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

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


//==============================================================================
/*
    DEMO:    27-multiview.cpp

    This example illustrates how to crate four different panels and associated
    framebuffer to render a scene from different points of view.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 27-multiview" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[s] - Save screenshots of all framebuffers to file" << endl;
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

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();


    //--------------------------------------------------------------------------
    // CAMERAS
    //--------------------------------------------------------------------------

    // create camera main
    cameraMain = new cCamera(NULL);

    // create camera 1
    cameraView1 = new cCamera(world);
    world->addChild(cameraView1);

    // set position and orientation
    cameraView1->set(cVector3d(0.0, 1.0, 0.0),    // camera position (eye)
                     cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                     cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set orthographic view
    cameraView1->setOrthographicView(1.2);

    // set camera settings
    cameraView1->setClippingPlanes(0.01, 10.0);

    // create camera 2
    cameraView2 = new cCamera(world);
    world->addChild(cameraView2);

    // set position and orientation
    cameraView2->set(cVector3d(1.0, 0.0, 0.0),    // camera position (eye)
                     cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                     cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set camera settings
    cameraView2->setClippingPlanes(0.01, 10.0);

    // set orthographic view
    cameraView2->setOrthographicView(1.2);

    // create camera 3
    cameraView3 = new cCamera(world);
    world->addChild(cameraView3);

    // set position and orientation
    cameraView3->set(cVector3d(0.0, 0.0, 1.0),    // camera position (eye)
                     cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                     cVector3d(-1.0, 0.0, 0.0));  // direction of the (up) vector

    // set orthographic view
    cameraView3->setOrthographicView(1.2);

    // set camera settings
    cameraView3->setClippingPlanes(0.01, 10.0);

    // create camera 4
    cameraView4 = new cCamera(world);
    world->addChild(cameraView4);

    // set position and orientation
    cameraView4->set(cVector3d(1.0, 0.0, 0.6),    // camera position (eye)
                     cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                     cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set camera settings
    cameraView4->setClippingPlanes(0.01, 10.0);


    //--------------------------------------------------------------------------
    // LIGHT SOURCES
    //--------------------------------------------------------------------------

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0,-1.0,-1.0);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a tool (cursor)
    tool = new cToolCursor(world);

    // add tool to world
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE SCENE
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // 3D OBJECT
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    object = new cMultiMesh();

    // add object to world
    world->addChild(object);

    // load an object file
    bool fileload;
    fileload = object->loadFromFile(RESOURCE_PATH("../resources/models/engine/engine.3ds"));

    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object->loadFromFile("../../../bin/resources/models/engine/engine.3ds");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly" << endl;
        close();
        return (-1);
    }

    // disable culling so that faces are rendered on both sides
    object->setUseCulling(false);

    // get dimensions of object
    object->computeBoundaryBox(true);
    double size = cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0.001)
    {
        object->scale(1.0 / size);
    }

    // compute collision detection algorithm
    object->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    object->setStiffness(0.2 * maxStiffness, true);

    // define some haptic friction properties
    object->setFriction(0.1, 0.2, true);

    // enable display list for faster graphic rendering
    object->setUseDisplayList(true);


    /////////////////////////////////////////////////////////////////////////
    // TEXTURE
    /////////////////////////////////////////////////////////////////////////

    // load an environmental texture file
    cTexture2dPtr texture = cTexture2d::create();
    fileload = texture->loadFromFile(RESOURCE_PATH("../resources/images/grayball.jpg"));
    if (!fileload)
    {
        fileload = texture->loadFromFile("../../../bin/resources/images/grayball.jpg");
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // enable spherical mapping
    texture->setSphericalMappingEnabled(true);

    // setup texture and material properties to each mesh that composes the multimesh object
    int numMeshes = object->getNumMeshes();
    for (int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = object->getMesh(i);
        mesh->setTexture(texture, true);
        mesh->setUseTexture(true, true);
        mesh->m_material->setWhite();
    }


    //--------------------------------------------------------------------------
    // FRAMEBUFFERS
    //--------------------------------------------------------------------------

    // create framebuffer for view 1
    frameBuffer1 = cFrameBuffer::create();
    frameBuffer1->setup(cameraView1);

    // create framebuffer for view 2
    frameBuffer2 = cFrameBuffer::create();
    frameBuffer2->setup(cameraView2);

    // create framebuffer for view 3
    frameBuffer3 = cFrameBuffer::create();
    frameBuffer3->setup(cameraView3);

    // create framebuffer for view 4
    frameBuffer4 = cFrameBuffer::create();
    frameBuffer4->setup(cameraView4);


    //--------------------------------------------------------------------------
    // VIEW PANELS
    //--------------------------------------------------------------------------

    // create and setup view panel 1
    viewPanel1 = new cViewPanel(frameBuffer1);
    cameraMain->m_frontLayer->addChild(viewPanel1);

    // create and setup view panel 2
    viewPanel2 = new cViewPanel(frameBuffer2);
    cameraMain->m_frontLayer->addChild(viewPanel2);

    // create and setup view panel 3
    viewPanel3 = new cViewPanel(frameBuffer3);
    cameraMain->m_frontLayer->addChild(viewPanel3);

    // create and setup view panel 4
    viewPanel4 = new cViewPanel(frameBuffer4);
    cameraMain->m_frontLayer->addChild(viewPanel4);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label for view 1
    labelView1 = new cLabel(font);
    cameraView1->m_frontLayer->addChild(labelView1);
    labelView1->setLocalPos(10, 10);
    labelView1->m_fontColor.setBlack();
    labelView1->setText("Side View");

    // create a label for view 2
    labelView2 = new cLabel(font);
    cameraView2->m_frontLayer->addChild(labelView2);
    labelView2->setLocalPos(10, 10);
    labelView2->m_fontColor.setBlack();
    labelView2->setText("Front View");

    // create a label for view 3
    labelView3 = new cLabel(font);
    cameraView3->m_frontLayer->addChild(labelView3);
    labelView3->setLocalPos(10, 10);
    labelView3->m_fontColor.setBlack();
    labelView3->setText("Top View");

    // create a label for view 4
    labelView4 = new cLabel(font);
    cameraView4->m_frontLayer->addChild(labelView4);
    labelView4->setLocalPos(10, 10);
    labelView4->m_fontColor.setBlack();
    labelView4->setText("Perspective View");

    // create a background
    background1 = new cBackground();
    cameraView1->m_backLayer->addChild(background1);

    // set background properties
    background1->setCornerColors(cColorf(0.62f, 0.60f, 0.67f),
                                 cColorf(0.62f, 0.60f, 0.67f),
                                 cColorf(0.85f, 0.87f, 0.88f),
                                 cColorf(0.85f, 0.87f, 0.88f));

    // create background copies for other cameras
    background2 = background1->copy();
    cameraView2->m_backLayer->addChild(background2);

    background3 = background1->copy();
    cameraView3->m_backLayer->addChild(background3);

    background4 = background1->copy();
    cameraView4->m_backLayer->addChild(background4);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);


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

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    int halfW = width / 2;
    int halfH = height / 2;
    int offset = 1;

    // update display panel sizes and positions
    viewPanel1->setLocalPos(0.0, 0.0);
    viewPanel1->setSize(halfW, halfH);

    viewPanel2->setLocalPos(halfW+offset, 0.0);
    viewPanel2->setSize(halfW, halfH);

    viewPanel3->setLocalPos(0.0, halfH+offset);
    viewPanel3->setSize(halfW, halfH);

    viewPanel4->setLocalPos(halfW+offset, halfH+offset);
    viewPanel4->setSize(halfW, halfH);

    // update frame buffer sizes
    frameBuffer1->setSize(halfW, halfH);
    frameBuffer2->setSize(halfW, halfH);
    frameBuffer3->setSize(halfW, halfH);
    frameBuffer4->setSize(halfW, halfH);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

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

    // option - save screenshot to file
    else if (a_key == GLFW_KEY_S)
    {
        cImagePtr image1 = cImage::create();
        cImagePtr image2 = cImage::create();
        cImagePtr image3 = cImage::create();
        cImagePtr image4 = cImage::create();
        frameBuffer1->copyImageBuffer(image1);
        frameBuffer2->copyImageBuffer(image2);
        frameBuffer3->copyImageBuffer(image3);
        frameBuffer4->copyImageBuffer(image4);
        image1->saveToFile("screenshot1.png");
        image2->saveToFile("screenshot2.png");
        image3->saveToFile("screenshot3.png");
        image4->saveToFile("screenshot4.png");
        cout << "> Saved screenshots to file.       \r";
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
        cameraMain->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

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
    delete world;
    delete handler;

}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render all framebuffers
    frameBuffer1->renderView();
    frameBuffer2->renderView();
    frameBuffer3->renderView();
    frameBuffer4->renderView();

    // render world
    cameraMain->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics(void)
{
    cMode state = IDLE;
    cGenericObject* object = NULL;
    cTransform tool_T_object;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

 
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // compute transformation from world to tool (haptic device)
        cTransform world_T_tool = tool->getDeviceGlobalTransform();

        // get status of user switch
        bool button = tool->getUserSwitch(0);

        //
        // STATE 1:
        // Idle mode - user presses the user switch
        //
        if ((state == IDLE) && (button == true))
        {
            // check if at least one contact has occurred
            if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
            {
                // get contact event
                cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

                // get object from contact event
                object = collisionEvent->m_object;

                // get transformation from object
                cTransform world_T_object = object->getGlobalTransform();

                // compute inverse transformation from contact point to object 
                cTransform tool_T_world = world_T_tool;
                tool_T_world.invert();

                // store current transformation tool
                tool_T_object = tool_T_world * world_T_object;

                // update state
                state = SELECTION;
            }
        }


        //
        // STATE 2:
        // Selection mode - operator maintains user switch enabled and moves object
        //
        else if ((state == SELECTION) && (button == true))
        {
            // compute new transformation of object in global coordinates
            cTransform world_T_object = world_T_tool * tool_T_object;

            // compute new transformation of object in local coordinates
            cTransform parent_T_world = object->getParent()->getLocalTransform();
            parent_T_world.invert();
            cTransform parent_T_object = parent_T_world * world_T_object;

            // assign new local transformation to object
            object->setLocalTransform(parent_T_object);

            // set zero forces when manipulating objects
            tool->setDeviceGlobalForce(0.0, 0.0, 0.0);

            // reset proxy
            tool->initialize();
        }

        //
        // STATE 3:
        // Finalize Selection mode - operator releases user switch.
        //
        else
        {
            state = IDLE;
        }


        /////////////////////////////////////////////////////////////////////////
        // FINALIZE
        /////////////////////////////////////////////////////////////////////////

        // send forces to haptic device
        tool->applyToDevice();  
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
