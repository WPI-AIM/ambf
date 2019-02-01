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
    \version   3.2.0 $Rev: 2049 $
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

// state machine states
const int STATE_IDLE            = 1;
const int STATE_MODIFY_MAP      = 2;
const int STATE_MOVE_CAMERA     = 3;


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

// radius of tool for graphic and haptic representation
double hapticRadius;
double displayRadius;

// a virtual mesh like object
cMesh* object;

// a small magnetic line used to constrain the tool along the vertical axis
cShapeLine* magneticLine;

// two sphere placed at both end of the magnetic line
cShapeSphere* sphereA;
cShapeSphere* sphereB;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a label to explain what is happening
cLabel* labelMessage;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// state machine 
int state = STATE_IDLE;

// camera status
bool flagCameraInMotion = false;

// informs the graphic display callback to update the display list
// when the topology of the 3D height map is changed.
bool flagMarkForUpdate = false;

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

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// loads a bitmap file and create 3D height map based on pixel color
int loadHeightMap();


//==============================================================================
/*
    DEMO:    20-map.cpp

    This example illustrates the construction a triangle based object.
    The application first loads a bitmap texture image. For each pixel,
    we then define a height by computing the gray-scale value. A vertex
    is created for each pixel and triangles are then generated to connect
    the array of vertices together. This example also demonstrates the
    use of mouse callback commands to allow the operator to control the
    position of the virtual camera. The operator can also use the haptic
    device (user switch command) to move the camera or grasp a point on the 
    surface and deform the terrain.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
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
    cout << "Demo: 20-map" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Instructions:" << endl << endl;
    cout << "- Use haptic device and user switch to move" << endl;
    cout << "  camera or deform terrain." << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Haptic Shading (ON/OFF)" << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[3] - Save map to 3D files: map3d.obj, map3d.3ds, map3d.stl" << endl;
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
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // define a basis in spherical coordinates for the camera
    camera->setSphericalReferences(cVector3d(0,0,0),    // origin
                                   cVector3d(0,0,1),    // zenith direction
                                   cVector3d(1,0,0));   // azimuth direction

    camera->setSphericalDeg(3.5,    // spherical coordinate radius
                            60,     // spherical coordinate polar angle
                            5);     // spherical coordinate azimuth angle

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(2.5, 0.0, 2.0);

    // define the direction of the light beam
    light->setDir(-1.0, 0.0,-1.0);             

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(25);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the device has a gripper, then enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the camera
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // set color of tool
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhiteAliceBlue();

    // set the physical radius of the proxy.
    hapticRadius  = 0.00;
    displayRadius = 0.05;
    tool->setRadius(displayRadius, hapticRadius);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // oriente tool with camera
    tool->setLocalRot(camera->getLocalRot());

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // initialize tool by connecting to haptic device
    tool->start();


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    double maxLinearForce = hapticDeviceInfo.m_maxLinearForce;
    double maxLinearDamping = hapticDeviceInfo.m_maxLinearDamping;
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // MAP
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    object = new cMesh();

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

    // Since we want to see our polygons from both sides, we disable culling.
    object->setUseCulling(false);

    // load default map
    loadHeightMap();

    // set color properties
    object->m_material->setBlueCornflower();

    // set stiffness
    object->m_material->setStiffness(0.5 * maxStiffness);

    // enable haptic shading
    object->m_material->setUseHapticShading(true);

    // use display list to increase graphical rendering performance
    object->setUseDisplayList(true);


    /////////////////////////////////////////////////////////////////////////
    // MAGNETIC LINE
    /////////////////////////////////////////////////////////////////////////

    // create a small vertical white magnetic line that will be activated when the
    // user deforms the mesh.
    magneticLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

    // add line to world
    world->addChild(magneticLine);

    // set color of line
    magneticLine->m_colorPointA.setGrayDark();
    magneticLine->m_colorPointB.setGrayDark();

    // line is not yet enabled
    magneticLine->setHapticEnabled(false);
    magneticLine->setShowEnabled(false);

    // set haptic properties
    magneticLine->m_material->setStiffness(0.05 * maxStiffness);
    magneticLine->m_material->setMagnetMaxForce(maxLinearForce);
    magneticLine->m_material->setMagnetMaxDistance(0.25);
    magneticLine->m_material->setViscosity(0.05 * maxLinearDamping);

    // create a haptic magnetic effect
    cEffectMagnet* newEffect = new cEffectMagnet(magneticLine);
    magneticLine->addEffect(newEffect);

    // create two sphere that will be added at both ends of the line
    sphereA = new cShapeSphere(0.02);
    sphereB = new cShapeSphere(0.02);

    // add spheres to world
    world->addChild(sphereA);
    world->addChild(sphereB);

    // disable spheres for now
    sphereA->setShowEnabled(false);
    sphereB->setShowEnabled(false);

    // define some material properties for spheres
    cMaterial matSphere;
    matSphere.setWhiteAliceBlue();

    // assign material properties to both spheres
    sphereA->setMaterial(matSphere);
    sphereB->setMaterial(matSphere);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    // set font color
    labelRates->m_fontColor.setBlack();

    // create a label with a small message
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // set font color
    labelMessage->m_fontColor.setBlack();

    // set text message
    labelMessage->setText("touch surface - press user switch to deform surface or move camera");

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.8f, 0.8f, 0.8f),
                                cColorf(0.8f, 0.8f, 0.8f));


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

    // option - haptic shading:
    else if (a_key == GLFW_KEY_1)
    {
        bool useHapticShading = !object->m_material->getUseHapticShading();
        object->m_material->setUseHapticShading(useHapticShading);
        if (useHapticShading)
            cout << "> Haptic shading enabled     \r";
        else
            cout << "> Haptic shading disabled    \r";
    }

    // option - wire mode
    else if (a_key == GLFW_KEY_2)
    {
        bool useWireMode = !object->getWireMode();
        object->setWireMode(useWireMode);
        if (useWireMode)
            cout << "> Wire mode enabled          \r";
        else
            cout << "> Wire mode disabled         \r";
    }

    // option - save to file
    else if (a_key == GLFW_KEY_3)
    {
        cMultiMesh* multimesh = new cMultiMesh();
        cMesh* mesh = object->copy();
        multimesh->addMesh(mesh);
        multimesh->saveToFile("map3d.3ds");
        multimesh->saveToFile("map3d.obj");
        multimesh->saveToFile("map3d.stl");
        delete multimesh;
        cout << "> 3D Map has been saved to files \r";
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

    // update position of message label
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 50);


    /////////////////////////////////////////////////////////////////////
    // UPDATE MODEL
    /////////////////////////////////////////////////////////////////////

    // update object normals
    if (state == STATE_MODIFY_MAP)
    {
        object->computeAllNormals();
    }

    // if the mesh has been modified we update the display list
    if (flagMarkForUpdate)
    {
        object->markForUpdate(false);
        flagMarkForUpdate = false;
    }


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
    // initialize state to idle
    state = STATE_IDLE;  

    // current tool position
    cVector3d toolGlobalPos;        // global world coordinates
    cVector3d toolLocalPos;         // local coordinates

    // previous tool position
    cVector3d prevToolGlobalPos;    // global world coordinates
    cVector3d prevToolLocalPos;     // local coordinates

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

        // read user switch
        bool userSwitch = tool->getUserSwitch(0);

        // update tool position
        toolGlobalPos = tool->getDeviceGlobalPos();
        toolLocalPos  = tool->getDeviceLocalPos();

        if ((state == STATE_MOVE_CAMERA) && (!userSwitch))
        {
            state = STATE_IDLE;

            // enable haptic interaction with map
            object->setHapticEnabled(true, true);
        }

        else if (((state == STATE_MODIFY_MAP) && (!userSwitch)) ||
                 ((state == STATE_MODIFY_MAP) && (!tool->isInContact(magneticLine))))
        {
            state = STATE_IDLE;

            // disable magnetic line
            magneticLine->setHapticEnabled(false);
            magneticLine->setShowEnabled(false);

            // disable spheres
            sphereA->setShowEnabled(false);
            sphereB->setShowEnabled(false);

            // enable haptic interaction with map
            object->setHapticEnabled(true, true);

            // disable forces
            tool->setForcesOFF();

            // update bounding box (can take a little time)
            object->createAABBCollisionDetector(1.01 * hapticRadius);
            
            // enable forces again
            tool->setForcesON();
        }

        // user clicks with the mouse
        else if ((state == STATE_IDLE) && (userSwitch))
        {
            // start deforming object
            if (tool->isInContact(object))
            {
                state = STATE_MODIFY_MAP;

                // update position of line
                cVector3d posA = toolGlobalPos;
                posA.z(-0.7);

                cVector3d posB = toolGlobalPos;
                posB.z(0.7);

                magneticLine->m_pointA = posA;
                magneticLine->m_pointB = posB;

                // update position of spheres
                sphereA->setLocalPos(posA);
                sphereB->setLocalPos(posB);

                // enable spheres
                sphereA->setShowEnabled(true);
                sphereB->setShowEnabled(true);

                // enable magnetic line
                magneticLine->setHapticEnabled(true);
                magneticLine->setShowEnabled(true);

                // disable haptic interaction with map
                object->setHapticEnabled(false, true);
            }

            // start moving camera
            else
            {
                state = STATE_MOVE_CAMERA;
                
                // disable haptic interaction with map
                object->setHapticEnabled(false, true);
            }
        }

        // modify map
        else if (state == STATE_MODIFY_MAP)
        {
            // compute tool offset
            cVector3d offset = toolGlobalPos - prevToolGlobalPos;

            // map offset on z axis
            double offsetHeight = offset.z();

            // apply offset to all vertices through a weighted function
            int numVertices = object->getNumVertices();
            for (int i=0; i<numVertices; i++)
            {
                // compute distance between vertex and tool
                cVector3d posTool = tool->m_hapticPoint->getGlobalPosProxy();
                cVector3d posVertex = object->m_vertices->getLocalPos(i);
                double distance = cDistance(posTool, posVertex);

                // compute factor
                double RADIUS = 0.4;
                double relativeDistance = distance / RADIUS;
                double clampedRelativeDistance = cClamp01(relativeDistance);
                double w = 0.5 + 0.5 * cos(clampedRelativeDistance * C_PI);

                // apply offset
                double offsetVertexHeight = w * offsetHeight;
                posVertex.z(posVertex.z() + offsetVertexHeight);
                object->m_vertices->setLocalPos(i, posVertex);

                // mesh has been modified, inform the graphic rendering call back to
                // update the display list of the map.
                flagMarkForUpdate = true;
            }
        }

        // move camera
        else if (state == STATE_MOVE_CAMERA)
        {
            // compute tool offset
            cVector3d offset = toolLocalPos - prevToolLocalPos;

            // compute new coordinates for camera in spherical coordinates
            double radius = camera->getSphericalRadius() - 2 * offset.x();
            double azimuthDeg = camera->getSphericalAzimuthDeg() - 40 * offset.y();
            double polarDeg = camera->getSphericalPolarDeg() + 40 * offset.z();

            // update coordinates
            camera->setSphericalDeg(radius, polarDeg, azimuthDeg);

            // oriente tool with camera
            tool->setLocalRot(camera->getLocalRot());
        }

        // store tool position
        prevToolLocalPos  = toolLocalPos;
        prevToolGlobalPos = toolGlobalPos;

        // send forces to haptic device
        tool->applyToDevice();

        // update frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

int loadHeightMap()
{
    // create an image
    cImage image;

    // load a file
    bool fileload = image.loadFromFile(RESOURCE_PATH("../resources/images/map.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = image.loadFromFile("../../../bin/resources/images/map.jpg");
        #endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // get the size of the image
    int sizeX = image.getWidth();
    int sizeY = image.getHeight();

    // check size of image
    if ((sizeX < 1) || (sizeY < 1)) { return (false); }

    // we look for the largest side
    int largestSide = cMax(sizeX, sizeY);

    // scale the image to fit the world
    double scale = 1.0 / (double)largestSide;

    // we will create an triangle based object. For centering puposes we
    // compute an offset for axis X and Y corresponding to the half size
    // of the image map.
    double offsetX = 0.5 * (double)sizeX * scale;
    double offsetY = 0.5 * (double)sizeY * scale;

    // allocate vertices for this map
    object->m_vertices->newVertices(sizeX*sizeY);
    
    // set position of each vertex
    int x,y, index;
    index = 0;
    for (y=0; y<sizeY; y++)
    {
        for (x=0; x<sizeX; x++)
        {
            // get color of image pixel
            cColorb color;
            image.getPixelColor(x, y, color);

            // compute vertex height by averaging the color components RGB and scaling the value.
            const double HEIGHT_SCALE = 0.03;

            double height = HEIGHT_SCALE * (color.getLuminance() / 255.0);

            // compute the position of the vertex
            double px = scale * (double)x - offsetX;
            double py = scale * (double)y - offsetY;

            // set vertex position
            object->m_vertices->setLocalPos(index, px, py, height);
            index++;
        }
    }

    // Create a triangle based map using the above pixels
    for (x=0; x<(sizeX-1); x++)
    {
        for (y=0; y<(sizeY-1); y++)
        {
            // get the indexing numbers of the next four vertices
            unsigned int index00 = ((y + 0) * sizeX) + (x + 0);
            unsigned int index01 = ((y + 0) * sizeX) + (x + 1);
            unsigned int index10 = ((y + 1) * sizeX) + (x + 0);
            unsigned int index11 = ((y + 1) * sizeX) + (x + 1);

            // create two new triangles
            object->newTriangle(index00, index01, index10);
            object->newTriangle(index10, index01, index11);
        }
    }

    // compute normals
    object->computeAllNormals();

    // compute boundary box
    object->computeBoundaryBox(true);
    cVector3d min = object->getBoundaryMin();
    cVector3d max = object->getBoundaryMax();

    // compute size of object (largest side)
    cVector3d span = cSub(max, min);
    double size = cMax(span.x(), cMax(span.y(), span.z()));

    // scale object
    const double DESIRED_MESH_SIZE = 2.0;
    double scaleFactor = DESIRED_MESH_SIZE / size;
    object->scale(scaleFactor);

    // compute boundary box again
    object->computeBoundaryBox(true);

    // create collision detector for haptics interaction
    object->createAABBCollisionDetector(1.01 * hapticRadius);

    // success
    return (0);
}
