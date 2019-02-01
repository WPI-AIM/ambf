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
    \author    Sonny Chan
    \author    Francois Conti
    \version   3.2.0 $Rev: 1292 $
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
// STATES
//------------------------------------------------------------------------------
enum MouseState
{
    MOUSE_IDLE,
    MOUSE_MOVE_CAMERA
};


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a virtual voxel like object
cVoxelObject* object;

// 3D image data
cMultiImagePtr image;

// resolution of voxel model
int voxelModelResolution = 64;

// mutex to object
cMutex mutexObject;

// mutex to voxel
cMutex mutexVoxel;

// 3D texture object
cTexture3dPtr texture;

// region of voxels being updated
cCollisionAABBBox volumeUpdate;

// a mesh object to store the converted volume using the marching cube algorithm
cMultiMesh* surfaceModel;

// flag that indicates that voxels have been updated
bool flagMarkVolumeForUpdate = false;

// flag that indicates if voxel object is a cube or spheroid
bool flagModelCube = false;

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

// mouse state
MouseState mouseState = MOUSE_IDLE;

// last mouse position
double mouseX, mouseY;

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

// callback to handle mouse click
void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods);

// callback to handle mouse motion
void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY);

// callback to handle mouse scroll
void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// build voxel shape
void buildVoxelShape(double a_radiusSphere, double a_radiusCylinder);

// build voxel cube
void buildVoxelCube();


//==============================================================================
/*
    DEMO:    28-voxel-basic.cpp

    This demonstration illustrates the use of a 3D voxels.
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
    cout << "Demo: 28-voxel-basic" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Standard rendering (no shaders)" << endl;
    cout << "[2] - Basic voxel rendering" << endl;
    cout << "[3] - Isosurface with material" << endl;
    cout << "[4] - Isosurface with colors" << endl << endl;
    cout << "[7] - Resolution 32x32" << endl;
    cout << "[8] - Resolution 64x64" << endl;
    cout << "[9] - Resolution 128x128" << endl << endl;
    cout << "[v,w] Adjust quality of graphic rendering" << endl;
    cout << "[5] - Decrease voxel opacity (mode Basic voxel rendering only)" << endl;
    cout << "[6] - Increase voxel opacity (mode Basic voxel rendering only)" << endl;
    cout << "[c] - Swap between Cube or Spheroid model" << endl;
    cout << "[s] - Save volume image to disk (.png)" << endl;
    cout << "[p] - Polygonize model and save to file (.stl)" << endl;
    cout << "[u] - Trigger display of polygon model" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    string resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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

    // set mouse position callback
    glfwSetCursorPosCallback(window, mouseMotionCallback);

    // set mouse button callback
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    // set mouse scroll callback
    glfwSetScrollCallback(window, mouseScrollCallback);

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

    // define a basis in spherical coordinates for the camera
    camera->setSphericalReferences(cVector3d(0,0,0),    // origin
                                   cVector3d(0,0,1),    // zenith direction
                                   cVector3d(1,0,0));   // azimuth direction

    camera->setSphericalDeg(2.3,    // spherical coordinate radius
                            60,     // spherical coordinate polar angle
                            10);    // spherical coordinate azimuth angle

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.1, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.5, 0.5, 0.5);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(1.0, 1.0, 1.0);


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

    // if the haptic device has a gripper, enable it as a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.05);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // oriente tool with camera
    tool->setLocalRot(camera->getLocalRot());

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

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
    
    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    //--------------------------------------------------------------------------
    // CREATE VOXEL OBJECT
    //--------------------------------------------------------------------------

    // create a volumetric model
    object = new cVoxelObject();

    // add object to world
    world->addChild(object);
    
    // set the dimensions by assigning the position of the min and max corners
    object->m_minCorner.set(-0.5,-0.5,-0.5);
    object->m_maxCorner.set( 0.5, 0.5, 0.5);

    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set material color
    object->m_material->setOrangeCoral();

    // set stiffness property
    object->setStiffness(0.2 * maxStiffness);

    // show/hide boundary box
    object->setShowBoundaryBox(false);


    //--------------------------------------------------------------------------
    // CREATE VOXEL DATA
    //--------------------------------------------------------------------------

    // create multi image data structure
    image = cMultiImage::create();

    // allocate 3D image data
    image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);

    // create texture
    texture = cTexture3d::create();

    // assign texture to voxel object
    object->setTexture(texture);

    // assign volumetric image to texture
    texture->setImage(image);

    // set quality of graphic rendering
    object->setQuality(0.5);

    // draw some 3D volumetric object
    if (flagModelCube)
    {
        buildVoxelCube();
    }
    else
    {
        buildVoxelShape(0.5, 0.2);
    }

    // set default rendering mode
    object->setRenderingModeIsosurfaceMaterial();


    //--------------------------------------------------------------------------
    // SPECIAL SHADER (OPTIONAL)
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    /*
        The following section has been commented, but illustrates how one
        can load a custom shader to render a volumetric object.
    */
    //--------------------------------------------------------------------------

    /*
    // create vertex shader
    cShaderPtr vertexShader = cShader::create(C_VERTEX_SHADER);
    bool fileload = vertexShader->loadSourceFile("./resources/shaders/isosurface.vert");
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = vertexShader->loadSourceFile("../../../bin/resources/shaders/isosurface.vert");
#endif
    }

    // create fragment shader
    cShaderPtr fragmentShader = cShader::create(C_FRAGMENT_SHADER);
    
    fileload = fragmentShader->loadSourceFile("./resources/shaders/isosurface.frag");
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = fragmentShader->loadSourceFile("../../../bin/resources/shaders/isosurface.frag");
#endif
    }
    
    // create program shader
    cShaderProgramPtr voxelShader = cShaderProgram::create();
    
    // assign vertex shader to program shader
    voxelShader->attachShader(vertexShader);
    
    // assign fragment shader to program shader
    voxelShader->attachShader(fragmentShader);
    
    // assign program shader to object
    object->setShaderProgram(voxelShader);
    
    // link program shader
    voxelShader->linkProgram();

    // set custom rendering mode
    object->setRenderingModeCustom();
    */


    //--------------------------------------------------------------------------
    // SURFACE MODEL
    //--------------------------------------------------------------------------

    // create a multi-mesh object to store surface model generated from volume
    surfaceModel = new cMultiMesh;

    // add surface model as child of volume object for overlay display
    object->addChild(surfaceModel);


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

    // create a small message
    labelMessage = new cLabel(font);
    labelMessage->m_fontColor.setBlack();
    labelMessage->setText("press user switch to remove voxels (drilling)");
    camera->m_frontLayer->addChild(labelMessage);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.7f, 0.7f, 0.7f),
                                cColorf(0.7f, 0.7f, 0.7f));


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

    // option - select basic rendering mode
    else if (a_key == GLFW_KEY_1)
    {
        object->setRenderingModeBasic();
        cout << "> Standard rendering                                  \r";
    }

    // option - select shader based rendering mode
    else if (a_key == GLFW_KEY_2)
    {
        object->setRenderingModeVoxelColors();
        cout << "> Shader rendering                                    \r";
    }

    // option - select isosurface rendering mode (material)
    else if (a_key == GLFW_KEY_3)
    {
        object->setRenderingModeIsosurfaceMaterial();
        cout << "> Isosurface with material                            \r";
    }

    // option - select isosurface rendering mode (voxel colors)
    else if (a_key == GLFW_KEY_4)
    {
        object->setRenderingModeIsosurfaceColors();
        cout << "> Isosurface with colours                              \r";
    }

    // option - low resolution model
    else if (a_key == GLFW_KEY_7)
    {
        voxelModelResolution = 32;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> Low resolution                                  \r";
    }

    // option - medium resolution model
    else if (a_key == GLFW_KEY_8)
    {
        voxelModelResolution = 64;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> Medium resolution                                  \r";
    }

    // option - high resolution model
    else if (a_key == GLFW_KEY_9)
    {
        voxelModelResolution = 128;
        mutexObject.acquire();
        image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);
        mutexObject.release();
        texture->markForDeleteAndUpdate();
        if (flagModelCube)
        {
            buildVoxelCube();
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
        }
        cout << "> High resolution                                  \r";
    }

    // option - decrease opacity
    else if (a_key == GLFW_KEY_5)
    {
        float opacity = object->getVoxelOpacity();
        opacity = opacity - 0.01;
        object->setVoxelOpacity(opacity);
        cout << "> Decrease opacity   " << cStr(object->getVoxelOpacity(), 2) << "                                  \r";
    }

    // option - increase opacity
    else if (a_key == GLFW_KEY_6)
    {
        float opacity = object->getVoxelOpacity();
        opacity = opacity + 0.01;
        object->setVoxelOpacity(opacity);
        cout << "> Increase opacity: " << cStr(object->getVoxelOpacity(), 2) << "                                  \r";
    }

    // option - toggle model between cube and spheroid
    else if (a_key == GLFW_KEY_C)
    {
        flagModelCube = !flagModelCube;

        if (flagModelCube)
        {
            buildVoxelCube();
            cout << "> Render Cube                                  \r";
        }
        else
        {
            buildVoxelShape(0.5, 0.2);
            cout << "> Render Spheroid                              \r";
        }
    }

    // option - save voxel data to disk
    else if (a_key == GLFW_KEY_S)
    {
        mutexObject.acquire();
        image->saveToFiles("volume", "png");
        mutexObject.release();
        cout << "> Volume image saved to disk                       \r";
    }

    // option - polygonize model and save mesh to file
    else if (a_key == GLFW_KEY_P)
    {
        // clear model
        surfaceModel->deleteAllMeshes();

        // convert volume to surface model
        object->polygonize(surfaceModel);

        // set material property
        cMaterial mat;
        mat.setBlueCornflower();
        surfaceModel->setMaterial(mat);
        surfaceModel->setTransparencyLevel(0.5);

        // enable display of surface model
        surfaceModel->setShowEnabled(true);

        // save file to disk
        surfaceModel->saveToFile("volume(m).stl");

        // save a scaled version (x 0.1) and expressed in millimeters for 3D printing
        cMultiMesh* temp = surfaceModel->copy(true, true, true, false);
        double SCALE = 0.1;
        double METERS_TO_MILLIMETERS = 1000.0;
        temp->scale(SCALE * METERS_TO_MILLIMETERS);
        temp->saveToFile("volume-scaled(mm).stl");
        delete temp;

        cout << "> Volume has been polygonized and saved to disk    \r";
    }

    // option - trigger display of surface model
    else if (a_key == GLFW_KEY_U)
    {
        surfaceModel->setShowEnabled(!surfaceModel->getShowEnabled());
    }

    // option - decrease quality of graphic rendering
    else if (a_key == GLFW_KEY_V)
    {
        double value = object->getQuality();
        object->setQuality(value - 0.1);
        cout << "> Quality set to " << cStr(object->getQuality(), 1) << "                                  \r";
    }

    // option - increase quality of graphic rendering
    else if (a_key == GLFW_KEY_W)
    {
        double value = object->getQuality();
        object->setQuality(value + 0.1);
        cout << "> Quality set to " << cStr(object->getQuality(), 1) << "                                  \r";
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
        // update mouse state
        mouseState = MOUSE_IDLE;

        // store mouse position
        glfwGetCursorPos(window, &mouseX, &mouseY);

        // check for collision
        cCollisionRecorder recorder;
        cCollisionSettings settings;

        bool hit = camera->selectWorld(mouseX, (height - mouseY), width, height, recorder, settings);
        if (hit)
        {
            // check if hit involves voxel object
            if (recorder.m_nearestCollision.m_object == object)
            {
                // get selected voxel
                int voxelX = recorder.m_nearestCollision.m_voxelIndexX;
                int voxelY = recorder.m_nearestCollision.m_voxelIndexY;
                int voxelZ = recorder.m_nearestCollision.m_voxelIndexZ;

                // set color to black
                cColorb color(0x00, 0x00, 0x00, 0x00);

                // set color to voxel
                object->m_texture->m_image->setVoxelColor(voxelX, voxelY, voxelZ, color);

                // update voxel data
                object->m_texture->markForUpdate();
            }
        }
    }

    else if (a_button == GLFW_MOUSE_BUTTON_RIGHT && a_action == GLFW_PRESS)
    {
        // store mouse position
        glfwGetCursorPos(window, &mouseX, &mouseY);

        // update mouse state
        mouseState = MOUSE_MOVE_CAMERA;
    }

    else
    {
        // update mouse state
        mouseState = MOUSE_IDLE;
    }
}

//------------------------------------------------------------------------------

void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY)
{
    if (mouseState == MOUSE_MOVE_CAMERA)
    {
        // compute mouse motion
        int dx = a_posX - mouseX;
        int dy = a_posY - mouseY;
        mouseX = a_posX;
        mouseY = a_posY;

        // compute new camera angles
        double azimuthDeg = camera->getSphericalAzimuthDeg() - 0.5 * dx;
        double polarDeg = camera->getSphericalPolarDeg() - 0.5 * dy;

        // assign new angles
        camera->setSphericalAzimuthDeg(azimuthDeg);
        camera->setSphericalPolarDeg(polarDeg);

        // oriente tool with camera
        tool->setLocalRot(camera->getLocalRot());
    }
}

//------------------------------------------------------------------------------

void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY)
{
    double r = camera->getSphericalRadius();
    r = cClamp(r + 0.1 * a_offsetY, 0.5, 3.0);
    camera->setSphericalRadius(r);
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
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 40);


    /////////////////////////////////////////////////////////////////////
    // VOLUME UPDATE
    /////////////////////////////////////////////////////////////////////

    // update region of voxels to be updated
    if (flagMarkVolumeForUpdate)
    {
        mutexVoxel.acquire();
        cVector3d min = volumeUpdate.m_min;
        cVector3d max = volumeUpdate.m_max;
        volumeUpdate.setEmpty();
        mutexVoxel.release();
        texture->markForPartialUpdate(min, max);
        flagMarkVolumeForUpdate = false;
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

        // read user switch
        int userSwitches = tool->getUserSwitches();

        // acquire mutex
        if (mutexObject.tryAcquire())
        {
            // compute interaction forces
            tool->computeInteractionForces();

            // check if tool is in contact with voxel object
            if (tool->isInContact(object) && (userSwitches > 0))
            {
                // retrieve contact event
                cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);

                // update voxel color
                cColorb color(0x00, 0x00, 0x00, 0x00);
                object->m_texture->m_image->setVoxelColor(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ, color);

                // mark voxel for update
                mutexVoxel.acquire();
                volumeUpdate.enclose(cVector3d(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ));
                mutexVoxel.release();
                flagMarkVolumeForUpdate = true;
            }

            // release mutex
            mutexObject.release();
        }

        // send forces to haptic device
        tool->applyToDevice();
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

void buildVoxelShape(double a_radiusSphere, double a_radiusCylinder)
{
    mutexVoxel.acquire();

    // setup dimension of shape in voxel resolution
    double center = (double)voxelModelResolution / 2.0;
    double radiusSphere = (double)voxelModelResolution * a_radiusSphere;
    double radiusCylinder = (double)voxelModelResolution * a_radiusCylinder;
    double k = 255.0 / (double)voxelModelResolution;
    
    // draw voxels
    for (int z=0; z<voxelModelResolution; z++)
    {
        for (int y=0; y<voxelModelResolution; y++)
        {
            for (int x=0; x<voxelModelResolution; x++)
            {
                double px = (double)x;
                double py = (double)y;
                double pz = (double)z;

                // set translucent voxel
                cColorb color;
                color.set(0x00, 0x00, 0x00, 0x00);

                double distance = sqrt(cSqr(px-center) + cSqr(py-center) + cSqr(pz-center));
                if (distance < radiusSphere)
                {
                    double distanceAxisZ = sqrt(cSqr(px-center) + cSqr(py-center));
                    double distanceAxisY = sqrt(cSqr(px-center) + cSqr(pz-center));
                    double distanceAxisX = sqrt(cSqr(py-center) + cSqr(pz-center));
                
                    if ((distanceAxisZ > radiusCylinder) && (distanceAxisY > radiusCylinder) && (distanceAxisX > radiusCylinder))
                    {
                        // assign color to voxel
                        //color.set(0xff, 0xff, 0xff, 0xff);
                        color.set((GLubyte)(k * fabs(px-center)), (GLubyte)(k * fabs(py-center)), (GLubyte)(k * fabs(pz-center)), 0xff);
                    }
                }

                image->setVoxelColor(x, y, z, color);
            }
        }
    }

    texture->markForUpdate();

    mutexVoxel.release();
}

//------------------------------------------------------------------------------

void buildVoxelCube()
{
    mutexVoxel.acquire();

    // fill all voxels
    for (int z=0; z<voxelModelResolution; z++)
    {
        for (int y=0; y<voxelModelResolution; y++)
        {
            for (int x=0; x<voxelModelResolution; x++)
            {
                double r =       (double)x / (double)voxelModelResolution;
                double g = 1.0 - (double)y / (double)voxelModelResolution;
                double b =       (double)z / (double)voxelModelResolution;

                cColorb color;
                color.setf(r, g, b, 1.0);

                image->setVoxelColor(x, y, z, color);
            }
        }
    }

    texture->markForUpdate();

    mutexVoxel.release();
}

//------------------------------------------------------------------------------
