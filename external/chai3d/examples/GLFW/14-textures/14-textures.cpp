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


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few mesh objects
cMesh* object0;
cMesh* object1;
cMesh* object2;
cMesh* object3;

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

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

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
    DEMO:   14-textures.cpp

    This example illustrates the use of haptic textures projected onto mesh
    surfaces.
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
    cout << "Demo: 14-textures" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
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

    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.0, 0.0, 1.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 1.0, 0.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(1.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.0, 0.7);

    // define the direction of the light beam
    light->setDir(0.0, 0.0, -1.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(40);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    camera->addChild(tool);

    // position tool in respect to camera
    tool->setLocalPos(-1.0, 0.0, 0.0);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // set radius of tool
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

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
    // OBJECT 0:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object0 = new cMesh();

    // create plane
    cCreatePlane(object0, 0.3, 0.3);

    /*
    // The following code shows how to create a plane composed of four vertices
    // and two triangles. This code is equivalent to caling cCreatePlane(object0, 0.2, 0.2)

    // create vertices
    int vertex0 = object0->newVertex();
    int vertex1 = object0->newVertex();
    int vertex2 = object0->newVertex();
    int vertex3 = object0->newVertex();

    // set position, surface normal, and texture coordinate for each vertex
    object0->m_vertices->setLocalPos(vertex0,-0.1,-0.1, 0.0);
    object0->m_vertices->setNormal(vertex0, 0.0, 0.0, 1.0);
    object0->m_vertices->setTexCoord(vertex0, 0.0, 0.0);

    object0->m_vertices->setLocalPos(vertex1, 0.1,-0.1, 0.0);
    object0->m_vertices->setNormal(vertex1, 0.0, 0.0, 1.0);
    object0->m_vertices->setTexCoord(vertex1, 1.0, 0.0);

    object0->m_vertices->setLocalPos(vertex2, 0.1, 0.1, 0.0);
    object0->m_vertices->setNormal(vertex2, 0.0, 0.0, 1.0);
    object0->m_vertices->setTexCoord(vertex2, 1.0, 1.0);

    object0->m_vertices->setLocalPos(vertex3,-0.1, 0.1, 0.0);
    object0->m_vertices->setNormal(vertex3, 0.0, 0.0, 1.0);
    object0->m_vertices->setTexCoord(vertex3, 0.0, 1.0);

    // create two triangles by assigning their vertex IDs
    object0->m_triangles->newTriangle(vertex0, vertex1, vertex2);
    object0->m_triangles->newTriangle(vertex0, vertex2, vertex3);
    */

    // create collision detector
    object0->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object0);

    // set the position of the object
    object0->setLocalPos(-0.2, -0.2, 0.0);

    // set graphic properties
    bool fileload;
    object0->m_texture = cTexture2d::create();
    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/sand.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object0->m_texture->loadFromFile("../../../bin/resources/images/sand.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // enable texture mapping
    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // create normal map from texture data
    cNormalMapPtr normalMap0 = cNormalMap::create();
    normalMap0->createMap(object0->m_texture);
    object0->m_normalMap = normalMap0;
    
    // set haptic properties
    object0->m_material->setStiffness(0.8 * maxStiffness);
    object0->m_material->setStaticFriction(0.3);
    object0->m_material->setDynamicFriction(0.2);
    object0->m_material->setTextureLevel(1.0);
    object0->m_material->setHapticTriangleSides(true, false);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 1:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object1 = new cMesh();

    // create plane
    cCreatePlane(object1, 0.3, 0.3);

    // create collision detector
    object1->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object1);

    // set the position of the object
    object1->setLocalPos(0.2, -0.2, 0.0);

    // set graphic properties
    object1->m_texture = cTexture2d::create();
    fileload = object1->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/whitefoam.jpg"));
    if (!fileload)
    {
            #if defined(_MSVC)
            fileload = object1->m_texture->loadFromFile("../../../bin/resources/images/whitefoam.jpg");
            #endif
    }
    if (!fileload)
    {
            cout << "Error - Texture image failed to load correctly." << endl;
            close();
            return (-1);
    }

    // enable texture mapping
    object1->setUseTexture(true);
    object1->m_material->setWhite();

    // create normal map from texture data
    cNormalMapPtr normalMap1 = cNormalMap::create();
    normalMap1->createMap(object1->m_texture);
    object1->m_normalMap = normalMap1;

    // set haptic properties
    object1->m_material->setStiffness(0.1 * maxStiffness);
    object1->m_material->setStaticFriction(0.0);
    object1->m_material->setDynamicFriction(0.3);
    object1->m_material->setTextureLevel(1.5);
    object1->m_material->setHapticTriangleSides(true, false);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 2:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object2 = new cMesh();

    // create plane
    cCreatePlane(object2, 0.3, 0.3);

    // create collision detector
    object2->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object2);

    // set the position of the object
    object2->setLocalPos(0.2, 0.2, 0.0);

    // set graphic properties
    object2->m_texture = cTexture2d::create();
    fileload = object2->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/brownboard.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object2->m_texture->loadFromFile("../../../bin/resources/images/brownboard.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // enable texture mapping
    object2->setUseTexture(true);
    object2->m_material->setWhite();

    // create normal map from texture data
    cNormalMapPtr normalMap2 = cNormalMap::create();
    normalMap2->createMap(object2->m_texture);
    object2->m_normalMap = normalMap2;

    // set haptic properties
    object2->m_material->setStiffness(0.4 * maxStiffness);
    object2->m_material->setStaticFriction(0.2);
    object2->m_material->setDynamicFriction(0.2);
    object2->m_material->setTextureLevel(0.2);
    object2->m_material->setHapticTriangleSides(true, false);
    

    /////////////////////////////////////////////////////////////////////////
    // OBJECT 3:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object3 = new cMesh();
    
    // create plane
    cCreatePlane(object3, 0.3, 0.3);

    // create collision detector
    object3->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object3);

    // set the position of the object
    object3->setLocalPos(-0.2, 0.2, 0.0);

    // set graphic properties
    object3->m_texture = cTexture2d::create();
    fileload = object3->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/blackstone.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object3->m_texture->loadFromFile("../../../bin/resources/images/blackstone.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // enable texture mapping
    object3->setUseTexture(true);
    object3->m_material->setWhite();

    // create normal map from texture data
    cNormalMapPtr normalMap3 = cNormalMap::create();
    normalMap3->createMap(object3->m_texture);
    object3->m_normalMap = normalMap3;
    normalMap3->setTextureUnit(GL_TEXTURE0_ARB);

    // set haptic properties
    object3->m_material->setStiffness(0.7 * maxStiffness);
    object3->m_material->setStaticFriction(0.4);
    object3->m_material->setDynamicFriction(0.3);
    object3->m_material->setTextureLevel(0.5);
    object3->m_material->setHapticTriangleSides(true, false);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(0.3, 0.3, 0.3),
                                cColorf(0.2, 0.2, 0.2),
                                cColorf(0.1, 0.1, 0.1),
                                cColorf(0.0, 0.0, 0.0));


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
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

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
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
