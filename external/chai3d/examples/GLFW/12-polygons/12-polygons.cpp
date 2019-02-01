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

// a virtual mesh like object
cMesh* object;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a label to describe application
cLabel* labelDescription;

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
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback to handle mouse click
void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:   12-polygons.cpp

    This example illustrates how to build an object composed of triangle
    with individual colors. A finger-proxy algorithm is used to compute
    the interaction force between the tool and the object.
    The color of each triangle can be switched to red by selecting and clicking
    the window display with the computer mouse.
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
    cout << "Demo: 12-polygons" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Wireframe (ON/OFF)" << endl;
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

    // set mouse button callback
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

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
    camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(1.3);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.5, 0.0);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(20);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.1;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

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

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a virtual mesh
    object = new cMesh();

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

    // Since we want to see our polygons from both sides, we disable culling.
    object->setUseCulling(false);

    // create a circular collection of triangles
    double ang = 0;
    double angStep = 5;
    while (ang < 360)
    {
        double radius = 0.2;
        double radiusStep = 0.1;
        double tang = ang;
        while (radius < 1.0)
        {
            // define triangle points
            cVector3d p0 = cVector3d(0.0, radius * cCosDeg(tang), radius * cSinDeg(tang));
            cVector3d p1 = cVector3d(0.0, (radius + radiusStep) * cCosDeg(tang+angStep), (radius + radiusStep) * cSinDeg(tang+angStep));
            cVector3d p2 = cVector3d(0.0, (radius + radiusStep) * cCosDeg(tang-angStep), (radius + radiusStep) * cSinDeg(tang-angStep));

            // define a triangle color
            cColorf color;
            color.set(ang / 360, 1.0 - ang / 360, radius, 0.5);

            // create three new vertices
            int vertex0 = object->newVertex();
            int vertex1 = object->newVertex();
            int vertex2 = object->newVertex();

            // set position of each vertex
            object->m_vertices->setLocalPos(vertex0, p0);
            object->m_vertices->setLocalPos(vertex1, p1);
            object->m_vertices->setLocalPos(vertex2, p2);

            // assign color to each vertex
            object->m_vertices->setColor(vertex0, color);
            object->m_vertices->setColor(vertex1, color);
            object->m_vertices->setColor(vertex2, color);

            // create new triangle from vertices
            object->newTriangle(vertex0, vertex1, vertex2);

            // increment the different counters
            radiusStep = 1.1 * radiusStep;
            radius = radius + 1.2 * radiusStep;
            tang = tang + 0.5 * angStep;
        }

        ang = ang + 3 * angStep;
    }

    // compute surface normals
    object->computeAllNormals();

    // we indicate that we ware rendering triangles by using specific colors for each of them (see above)
    object->setUseVertexColors(true);

    // we indicate that we also using material properties. If you set this parameter to 'false'
    // you will notice that only vertex colors are used to render triangle, and lighting
    // will not longer have any effect.
    object->setUseMaterial(true);
    
    // compute a boundary box
    object->computeBoundaryBox(true);

    // compute collision detection algorithm
    object->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    object->m_material->setStiffness(0.3 * maxStiffness);

    // render triangles hapticaly on both sides
    object->m_material->setHapticTriangleSides(true, true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    // create a label that describes how to operate the application
    labelDescription = new cLabel(font);
    camera->m_frontLayer->addChild(labelDescription);
    labelDescription->setText("use mouse to select a triangle");
    labelDescription->m_fontColor.setWhite();

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background color properties
    // set background properties
    background->setCornerColors(cColorf(0.44f, 0.44f, 0.88f),
                                cColorf(0.44f, 0.44f, 0.88f),
                                cColorf(0.00f, 0.00f, 0.00f),
                                cColorf(0.00f, 0.00f, 0.00f));


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
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    // update position of label
    labelDescription->setLocalPos((int)(0.5 * (width - labelDescription->getWidth())), 40);
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

    // option - enable/disable wireframe mode
    else if (a_key == GLFW_KEY_1)
    {
        bool useWireMode = !object->getWireMode();
        object->setWireMode(useWireMode);
        if (useWireMode)
            cout << "> Wire mode enabled  \r";
        else
            cout << "> Wire mode disabled \r";
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

void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods)
{
    if (a_button == GLFW_MOUSE_BUTTON_LEFT && a_action == GLFW_PRESS)
    {
        // store mouse position
        double x,y;
        glfwGetCursorPos(window, &x, &y);

        // detect for any collision between mouse and scene
        cCollisionRecorder recorder;
        cCollisionSettings settings;

        bool hit = camera->selectWorld(x, (height-y), width, height, recorder, settings);
        if (hit)
        {
            // create red color
            cColorf color;
            color.setRed();

            // retrieve triangle selected by mouse
            int index = recorder.m_nearestCollision.m_index;
            cTriangleArrayPtr triangles = recorder.m_nearestCollision.m_triangles;

            if (index > -1)
            {
                // get indices to all three vertices
                unsigned int index0 = triangles->getVertexIndex0(index);
                unsigned int index1 = triangles->getVertexIndex1(index);
                unsigned int index2 = triangles->getVertexIndex2(index);

                // for each vertex we attribute the new color;in this example
                // the color is the same for all vertices, but it is of course
                // possible to define a specific color for each vertex
                triangles->m_vertices->setColor(index0, color);
                triangles->m_vertices->setColor(index1, color);
                triangles->m_vertices->setColor(index2, color);
            }
        }
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
    // angular velocity of object
    cVector3d angVel(0.0, 0.0, 0.4);

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // some constants
        const double INERTIA = 0.4;
        const double MAX_ANG_VEL = 10.0;
        const double DAMPING = 0.1;

        // get position of cursor in global coordinates
        cVector3d toolPos = tool->getDeviceGlobalPos();

        // get position of object in global coordinates
        cVector3d objectPos = object->getGlobalPos();

        // compute a vector from the center of mass of the object (point of rotation) to the tool
        cVector3d v = cSub(toolPos, objectPos);

        // compute angular acceleration based on the interaction forces
        // between the tool and the object
        cVector3d angAcc(0,0,0);
        if (v.length() > 0.0)
        {
            // get the last force applied to the cursor in global coordinates
            // we negate the result to obtain the opposite force that is applied on the
            // object
            cVector3d toolForce = -tool->getDeviceGlobalForce();

            // compute the effective force that contributes to rotating the object.
            cVector3d force = toolForce - cProject(toolForce, v);

            // compute the resulting torque
            cVector3d torque = cMul(v.length(), cCross( cNormalize(v), force));

            // update rotational acceleration
            angAcc = (1.0 / INERTIA) * torque;
        }

        // update rotational velocity
        angVel.add(timeInterval * angAcc);

        // set a threshold on the rotational velocity term
        double vel = angVel.length();
        if (vel > MAX_ANG_VEL)
        {
            angVel.mul(MAX_ANG_VEL / vel);
        }

        // add some damping too
        angVel.mul(1.0 - DAMPING * timeInterval);

        // if user switch is pressed, set angular velocity to zero
        if (tool->getUserSwitch(0) == 1)
        {
            angVel.zero();
        }

        // compute the next rotation configuration of the object
        if (angVel.length() > C_SMALL)
        {
            object->rotateAboutGlobalAxisRad(cNormalize(angVel), timeInterval * angVel.length());
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
