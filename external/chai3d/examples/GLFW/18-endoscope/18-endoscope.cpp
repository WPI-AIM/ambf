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
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a camera attached to the endocope object
cCamera* cameraScope;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a virtual object
cMultiMesh* heart;

// a virtual object
cMultiMesh* scope;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a first window
GLFWwindow* window0 = NULL;
int width0 = 0;
int height0 = 0;

// a second window
GLFWwindow* window1 = NULL;
int width1 = 0;
int height1 = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback0(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window display is resized
void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics0(void);

// this function renders the scene
void updateGraphics1(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:   18-endoscope.cpp

    This example demonstrates the use of dual display contexts. In the first
    window we illustrate a tool exploring a virtual heart. A second camera 
    is attached at the extremity of the tools and the image rendered in a
    second viewport.
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
    cout << "Demo: 18-endoscope" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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
    int space = 10;
    int w = 0.5 * mode->height;
    int h = 0.5 * mode->height;
    int x0 = 0.5 * mode->width - w - space;
    int y0 = 0.5 * (mode->height - h);
    int x1 = 0.5 * mode->width + space;
    int y1 = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);


    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 0
    ////////////////////////////////////////////////////////////////////////////

    // create display context
    window0 = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window0)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window0, &width0, &height0);

    // set position of window
    glfwSetWindowPos(window0, x0, y0);

    // set key callback
    glfwSetKeyCallback(window0, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window0, windowSizeCallback0);

    // set current display context
    glfwMakeContextCurrent(window0);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);


    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 1
    ////////////////////////////////////////////////////////////////////////////

    // create display context and share GPU data with window 0
    window1 = glfwCreateWindow(w, h, "CHAI3D", NULL, window0);
    if (!window1)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window1, &width1, &height1);

    // set position of window
    glfwSetWindowPos(window1, x1, y1);

    // set key callback
    glfwSetKeyCallback(window1, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window1, windowSizeCallback1);

    // set current display context
    glfwMakeContextCurrent(window1);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);


    ////////////////////////////////////////////////////////////////////////////
    // GLEW
    ////////////////////////////////////////////////////////////////////////////

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(1.5, 0.0, 1.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 100);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0,-1.0, -1.0);

    // set lighting conditions
    light->m_ambient.set(0.4f, 0.4f, 0.4f);
    light->m_diffuse.set(0.8f, 0.8f, 0.8f);
    light->m_specular.set(1.0f, 1.0f, 1.0f);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(false, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // OBJECT "HEART"
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    heart = new cMultiMesh();

    // add object to world
    world->addChild(heart);

    // load an object file
    bool fileload;
    fileload = heart->loadFromFile(RESOURCE_PATH("../resources/models/heart/heart.3ds"));

    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = heart->loadFromFile("../../../bin/resources/models/heart/heart.3ds");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }    

    // disable culling so that faces are rendered on both sides
    heart->setUseCulling(true);

    // scale model
    heart->scale(0.06);

    // compute collision detection algorithm
    heart->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    heart->setStiffness(0.1 * maxStiffness, true);

    // use display list for faster rendering
    heart->setUseDisplayList(true);

    // position and orient object in scene
    heart->setLocalPos(0.0, -0.2, 0.2);
    heart->rotateExtrinsicEulerAnglesDeg(90, 0, 0, C_EULER_ORDER_XYZ);

    cMaterial mat;
    mat.setHapticTriangleSides(true, true);
    heart->setMaterial(mat);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT "SCOPE"
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    scope = new cMultiMesh();

    // attach scope to tool
    tool->m_image = scope;

    // load an object file
    fileload = scope->loadFromFile(RESOURCE_PATH("../resources/models/endoscope/endoscope.3ds"));

    if (!fileload)
    {
#if defined(_MSVC)
        fileload = scope->loadFromFile("../../../bin/resources/models/endoscope/endoscope.3ds");
#endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }    

    // disable culling so that faces are rendered on both sides
    scope->setUseCulling(false);

    // scale model
    scope->scale(0.1);

    // use display list for faster rendering
    scope->setUseDisplayList(true);

    // position object in scene
    scope->rotateExtrinsicEulerAnglesDeg(0, 0, 0, C_EULER_ORDER_XYZ);


    /////////////////////////////////////////////////////////////////////////
    // CAMERA "SCOPE"
    /////////////////////////////////////////////////////////////////////////

    // create a camera and insert it into the virtual world
    cameraScope = new cCamera(world);
    scope->addChild(cameraScope);

    // position and orient the camera
    cameraScope->setLocalPos(-0.03, 0.0, 0.1);

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    cameraScope->setClippingPlanes(0.01, 100);



    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.9f, 0.9f, 0.9f),
                                cColorf(0.9f, 0.9f, 0.9f));

    // create a frontground for the endoscope
    cBackground* frontground = new cBackground();
    cameraScope->m_frontLayer->addChild(frontground);

    // load an texture map
    fileload = frontground->loadFromFile(RESOURCE_PATH("../resources/images/scope.png"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = frontground->loadFromFile("../../../bin/resources/images/scope.png");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Image failed to load correctly." << endl;
        close();
        return (-1);
    }


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback0(window0, width0, height0);
    windowSizeCallback1(window1, width1, height1);

    // main graphic loop
    while ((!glfwWindowShouldClose(window0)) && (!glfwWindowShouldClose(window1)))
    {
        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 0
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window0);

        // get width and height of window
        glfwGetWindowSize(window0, &width0, &height0);

        // render graphics
        updateGraphics0();

        // swap buffers
        glfwSwapBuffers(window0);


        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 1
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window1);

        // get width and height of window
        glfwGetWindowSize(window1, &width1, &height1);

        // render graphics
        updateGraphics1();

        // swap buffers
        glfwSwapBuffers(window1);


        ////////////////////////////////////////////////////////////////////////
        // FINALIZE
        ////////////////////////////////////////////////////////////////////////

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close windows
    glfwDestroyWindow(window0);
    glfwDestroyWindow(window1);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback0(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width0  = a_width;
    height0 = a_height;
}

//------------------------------------------------------------------------------

void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width1  = a_width;
    height1 = a_height;
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
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics0(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width0 - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width0, height0);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateGraphics1(void)
{
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    cameraScope->renderView(width1, height1);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
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

        // send forces to haptic device
        tool->applyToDevice();  
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
