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
    \version   3.2.0 $Rev: 1907 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"
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


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// two light sources to illuminate the objects in the world
cDirectionalLight *light1;
cDirectionalLight *light2;

// a haptic device handler
cHapticDeviceHandler* handler;

// a haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

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

// root resource path
string resourceRoot;


//---------------------------------------------------------------------------
// GEL
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject;

// dynamic nodes
cGELSkeletonNode* nodes[10][10];

// haptic device model
cShapeSphere* device;
double deviceRadius;

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
double stiffness;


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

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness);

// Build deformable model of huge cell
void BuildDynamicModel();


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//===========================================================================
/*
    DEMO:    GEM_cell.cpp

    This application illustrates the use of the GEM libraries to simulate
    deformable object. In this example we load a large mesh object and
    build a dynamic skeleton composed of volumetric spheres and 3 dimensional
    springs which model torsion, flexion and elongation properties.
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
    cout << "Demo: 51-GEL-cell" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

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
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // set background color
    world->m_backgroundColor.setBlack();

    // position and orient the camera
    camera->set(cVector3d(1.8, 0.0, 0.2),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a first directional light source
    light1 = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light1);

    // enable light source
    light1->setEnabled(true);                   

    // define direction of light beam
    light1->setDir(-1.0,-1.0, 0.0);

    // create a second directional light source
    light2 = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light2);

    // enable light source
    light2->setEnabled(true);                   

    // define direction of light beam
    light2->setDir(-1.0, 1.0, 0.0); 


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // open connection to haptic device
    hapticDevice->open();

    // desired workspace radius of the cursor
    cursorWorkspaceRadius = 0.7;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 0.15 * hapticDeviceInfo.m_maxLinearForce;

    // create a large sphere that represents the haptic device
    deviceRadius = 0.08;
    device = new cShapeSphere(deviceRadius);
    world->addChild(device);
    device->m_material->m_ambient.set(1.0f, 0.4f, 0.4f, 0.5f);
    device->m_material->m_diffuse.set(1.0f, 0.7f, 0.7f, 0.5f);
    device->m_material->m_specular.set(1.0f, 1.0f, 1.0f, 0.5f);
    device->m_material->setShininess(100);
    device->setUseTransparency(true);

    // interaction stiffness between tool and deformable model 
    stiffness = 20;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------
    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);
    world->rotateAboutGlobalAxisRad(cVector3d(0,1,0), cDegToRad(50));

    // create a deformable mesh
    defObject = new cGELMesh();
    defWorld->m_gelMeshes.push_front(defObject);
    bool fileload;
    fileload = defObject->loadFromFile(RESOURCE_PATH("../resources/models/cell/cell.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = defObject->loadFromFile("../../../bin/resources/models/cell/cell.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // compute size of object
    defObject->computeBoundaryBox(true);
    defObject->setUseCulling(true, true);

    cVector3d min = defObject->getBoundaryMin();
    cVector3d max = defObject->getBoundaryMax();
    
    // This is the "size" of the object
    cVector3d span = cSub(max, min);
    double size = cMax(span.x(), cMax(span.y(), span.z()));

    // We'll center all vertices, then multiply by this amount,
    // to scale to the desired size.
    if (size > 0)
    {
        double scaleFactor = 1.0 / size;
        defObject->scale(scaleFactor);
    }

    // update bounding box again
    defObject->computeBoundaryBox(true);

    // setup default values for nodes
    cGELSkeletonNode::s_default_radius        = 0.04;//3;
    cGELSkeletonNode::s_default_kDampingPos   = 0.4;
    cGELSkeletonNode::s_default_kDampingRot   = 0.1;
    cGELSkeletonNode::s_default_mass          = 0.06;  // [kg]
    cGELSkeletonNode::s_default_showFrame     = false;
    cGELSkeletonNode::s_default_color.set(0.6, 0.6, 0.0);
    cGELSkeletonNode::s_default_useGravity      = false;
    cGELSkeletonNode::s_default_gravity.set(0.00, 0.00, -1.00);
    radius = cGELSkeletonNode::s_default_radius;

    // setup default values for links
    cGELSkeletonLink::s_default_kSpringElongation = 100.0; // [N/m]
    cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
    cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
    cGELSkeletonLink::s_default_color.set(0.2, 0.2, 1.0);

    // build dynamic vertices
    defObject->buildVertices();

    // create dynamic model (GEM)
    BuildDynamicModel();

    // connect skin to skeleton
    defObject->connectVerticesToSkeleton(true);

    // show/hide underlying dynamic skeleton model
    defObject->m_showSkeletonModel = false;

    // use internal skeleton as deformable model
    defObject->m_useSkeletonModel = true;

    // create anchors
    cGELSkeletonLink::s_default_kSpringElongation = 5.0; // [N/m]
    list<cGELSkeletonNode*>::iterator i;
    int num = 0;
    for(i = defObject->m_nodes.begin(); i != defObject->m_nodes.end(); ++i)
    {
        num++;
    }

    int counter1 = 0;
    int counter2 = 0;
    for(i = defObject->m_nodes.begin(); i != defObject->m_nodes.end(); ++i)
    {
        if (counter1 <= num)
        {
            if (counter2 > 3)
            {
                cGELSkeletonNode* nextItem = *i;
                cGELSkeletonNode* newNode = new cGELSkeletonNode();
                newNode->m_fixed = true;
                newNode->m_pos = nextItem->m_pos;
                cGELSkeletonLink* newLink = new cGELSkeletonLink(nextItem, newNode); defObject->m_links.push_front(newLink);
                newLink->m_kSpringElongation = 5;
                counter2 = 0;
            }
            counter2 ++;
            counter1++;
        }
    }


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);
    labelRates->m_fontColor.setBlack();

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    fileload = background->loadFromFile(RESOURCE_PATH("../resources/images/bio.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = background->loadFromFile("../../../bin/resources/images/bio.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Image failed to load correctly." << endl;
        close();
        return (-1);
    }


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
    width  = a_width;
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
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
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
    // UPDATE DEFORMABLE MODELS
    /////////////////////////////////////////////////////////////////////

    // update skins deformable objects
    defWorld->updateSkins(true);


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
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // stop clock
        double time = cMin(0.001, clock.stop());

        // restart clock
        clock.start(true);

        // read position from haptic device
        cVector3d pos;
        hapticDevice->getPosition(pos);
        pos.mul(workspaceScaleFactor);
        device->setLocalPos(pos);

        // clear all external forces
        defWorld->clearExternalForces();

        // compute reaction forces
        cVector3d force(0.0, 0.0, 0.0);
        list<cGELSkeletonNode*>::iterator i;
        for(i = defObject->m_nodes.begin(); i != defObject->m_nodes.end(); ++i)
        {
            cGELSkeletonNode* nextItem = *i;

            cVector3d nodePos = nextItem->m_pos;
            cVector3d f = computeForce(pos, 0, nodePos, deviceRadius+nextItem->m_radius, stiffness);
            force.add(f);
            cVector3d tmpfrc = -1.0 * f;
            nextItem->setExternalForce(tmpfrc);
        }

        // integrate dynamics
        defWorld->updateDynamics(time);

        // scale force
        force.mul(deviceForceScale);

        // send forces to haptic device
        hapticDevice->setForce(force);

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness)
{

    // In the following we compute the reaction forces between the tool and the
    // sphere.
    cVector3d force;
    force.zero();

    cVector3d vSphereCursor = a_cursor - a_spherePos;


    if (vSphereCursor.length() < 0.0000001)
    {
        return (force);
    }

    if (vSphereCursor.length() > (a_cursorRadius + a_radius))
    {
        return (force);
    }

    // compute penetration distance between tool and surface of sphere
    double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
    cVector3d forceDirection = cNormalize(vSphereCursor);
    force = cMul( penetrationDistance * a_stiffness, forceDirection);

    return (force);
}

//---------------------------------------------------------------------------

void BuildDynamicModel()
{
    cGELSkeletonNode* newNode;
    cGELSkeletonNode* prevNode;
    cGELSkeletonLink* newLink;

    //-----------------------------------
    newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.296,0.280,-0.213);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.286,0.213,-0.254);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.241,0.148,-0.274);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.200,0.084,-0.267);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.043,-0.266,-0.226);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.114,-0.281,-0.151);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.120,-0.306,-0.073);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.085,-0.343,-0.023);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.013,-0.372,-0.006);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.030,-0.392,0.053);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.087,-0.399,0.094);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.129,-0.388,0.127);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.192,-0.357,0.119);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.386,-0.220,0.174);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.340,-0.267,0.161);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.308,-0.304,0.122);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.300,-0.329,0.059);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.319,-0.331,-0.018);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.363,-0.325,-0.080);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.386,-0.299,-0.117);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.380,-0.266,-0.162);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.375,-0.232,-0.204);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.374,-0.217,-0.249);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.418,-0.145,-0.266);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.362,-0.182,-0.277);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.309,-0.227,-0.277);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.255,-0.259,-0.286);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.199,-0.285,-0.284);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.123,-0.299,-0.271);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.023,-0.279,-0.245);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.029,-0.228,-0.286);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.145,-0.214,-0.336);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.197,-0.177,-0.329);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.243,-0.115,-0.310);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.198,0.011,-0.247);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.155,0.033,-0.161);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.080,-0.015,-0.105);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.042,-0.064,-0.035);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.030,-0.078,0.038);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.005,-0.111,0.099);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.097,0.015,-0.021);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.020,-0.003,-0.041);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.003,-0.065,-0.072);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.007,-0.122,-0.102);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.061,-0.151,-0.103);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.118,-0.171,-0.051);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.277,-0.175,-0.029);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.336,-0.148,-0.094);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.395,-0.107,-0.138);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.415,-0.059,-0.181);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.524,-0.003,-0.106);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.494,-0.035,-0.072);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.435,-0.052,-0.037);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.377,-0.048,0.004);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.328,0.003,0.069);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.288,0.059,0.004);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.236,0.095,-0.048);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.195,0.151,-0.059);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.157,0.184,-0.077);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.097,0.245,-0.062);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.055,0.283,-0.036);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.045,0.318,0.028);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.081,0.359,0.107);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.163,0.371,0.149);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.238,0.358,0.126);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.312,0.344,0.113);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.335,0.301,0.179);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.331,0.247,0.237);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.305,0.193,0.298);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.267,0.145,0.337);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.228,0.101,0.366);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.183,0.059,0.374);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.096,-0.011,0.335);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.003,-0.041,0.273);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.215,-0.124,0.358);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.296,-0.109,0.343);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.374,-0.081,0.322);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.462,-0.073,0.263);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.501,-0.125,0.190);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.491,-0.179,0.133);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.437,-0.232,0.082);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.370,-0.264,0.027);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.313,-0.234,-0.033);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.329,-0.178,-0.008);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.347,-0.095,0.016);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.345,-0.034,0.042);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.314,-0.011,0.104);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.261,-0.035,0.167);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.220,-0.057,0.234);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.172,-0.095,0.306);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.129,-0.131,0.329);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.059,-0.128,0.261);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.030,-0.115,0.199);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.001,-0.112,0.109);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.045,-0.129,0.035);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.055,-0.168,-0.043);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.052,-0.200,-0.157);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.027,-0.210,-0.235);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.035,-0.141,-0.243);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(-0.062,-0.050,-0.269);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.009,-0.032,-0.292);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.104,-0.004,-0.326);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.212,0.026,-0.323);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.292,0.045,-0.258);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.275,0.037,-0.184);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.248,0.069,-0.115);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.229,0.044,-0.048);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.208,-0.017,-0.022);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.173,-0.131,-0.020);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.305,0.366,0.054);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.300,0.399,-0.006);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.274,0.395,-0.079);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.230,0.343,-0.142);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.161,0.280,-0.147);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.161,0.227,-0.121);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.174,0.186,-0.076);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.151,0.194,-0.067);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.124,0.220,-0.059);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.385,0.342,-0.098);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.328,0.370,-0.044);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.258,0.351,0.014);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.193,0.341,0.045);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.139,0.316,0.068);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.109,0.272,0.079);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.082,0.224,0.094);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.087,0.232,0.153);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.152,0.233,0.188);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.209,0.208,0.148);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.210,0.193,0.093);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.191,0.188,0.041);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.163,0.177,-0.004);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.126,0.185,-0.046);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.099,0.205,-0.093);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.112,0.224,-0.131);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.122,0.241,-0.189);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.157,0.229,-0.227);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.226,0.211,-0.240);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.224,0.188,-0.190);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.280,0.178,-0.154);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.394,0.206,-0.105);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.473,0.202,-0.060);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.522,0.173,-0.028);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.542,0.120,0.005);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.546,0.059,0.032);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.544,0.001,0.072);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.525,-0.053,0.094);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.488,-0.144,0.094);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.488,-0.192,0.040);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.519,-0.195,-0.029);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.528,-0.198,-0.116);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.506,-0.187,-0.194);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.477,-0.149,-0.241);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.414,-0.125,-0.256);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.408,-0.069,-0.228);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.436,-0.002,-0.181);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.475,0.066,-0.175);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.482,0.136,-0.177);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.476,0.203,-0.185);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.443,0.272,-0.176);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.402,0.324,-0.133);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.378,0.326,-0.085);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.385,0.262,-0.040);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.369,0.200,-0.004);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.341,0.155,0.018);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.290,0.105,0.040);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.230,0.053,0.054);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.149,0.004,0.092);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.111,-0.018,0.147);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.101,-0.075,0.186);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.089,-0.144,0.192);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.101,-0.183,0.168);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.116,-0.183,0.105);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.121,-0.169,0.028);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.165,-0.189,-0.005);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.225,-0.217,0.008);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.298,-0.205,0.009);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.366,-0.206,0.011);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.439,-0.209,-0.021);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.508,-0.200,-0.043);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
}



