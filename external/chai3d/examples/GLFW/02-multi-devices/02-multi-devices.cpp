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
    \version   3.2.0 $Rev: 1869 $
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

// maximum number of devices supported by this application
const int MAX_DEVICES = 16;


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
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// labels to display each haptic device model
cLabel* labelHapticDeviceModel[MAX_DEVICES];

// labels to display the position [m] of each haptic device
cLabel* labelHapticDevicePosition[MAX_DEVICES];

// global variable to store the position [m] of each haptic device
cVector3d hapticDevicePosition[MAX_DEVICES];

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// some small spheres (cursor) representing position of each haptic device
cShapeSphere* cursor[MAX_DEVICES];

// some lines representing the velocity vector of each haptic device
cShapeLine* velocity[MAX_DEVICES];

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
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
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


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
    DEMO:   02-multi-devices.cpp

    This application illustrates how to program forces, torques and gripper
    forces on multiple haptic device.

    In this example the application opens an OpenGL window and displays a
    3D cursor for each device connected to your computer. If the user presses 
    onto the user button (if available on your haptic device), the color of 
    the cursor changes from blue to green.

    This example is very similar to 01-devices, but extends support for multiple
    haptic devices
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
    cout << "Demo: 02-multi-devices" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
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
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d(0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0); 

    // create a font
    font = NEW_CFONTCALIBRI20();


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    numHapticDevices = handler->getNumDevices();

    // setup each haptic device
    for (int i=0; i<numHapticDevices; i++)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice[i], i);

        // open a connection to haptic device
        hapticDevice[i]->open();

        // calibrate device (if necessary)
        hapticDevice[i]->calibrate();

        // retrieve information about the current haptic device
        cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

        // create a sphere (cursor) to represent the haptic device
        cursor[i] = new cShapeSphere(0.01);

        // insert cursor inside world
        world->addChild(cursor[i]);

        // create small line to illustrate the velocity of the haptic device
        velocity[i] = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

        // insert line inside world
        world->addChild(velocity[i]);


        // display a reference frame if haptic device supports orientations
        if (info.m_sensedRotation == true)
        {
            // display reference frame
            cursor[i]->setShowFrame(true);

            // set the size of the reference frame
            cursor[i]->setFrameSize(0.05);
        }

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(true);

        // create a label to display the haptic device model
        labelHapticDeviceModel[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDeviceModel[i]);
        labelHapticDeviceModel[i]->setText(info.m_modelName);

        // create a label to display the position of haptic device
        labelHapticDevicePosition[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDevicePosition[i]);
    }

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


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
    width  = a_width;
    height = a_height;

    // update position of widgets
    int step = 40;
    for (int i=0; i<numHapticDevices; i++)
    {
        // update position of label
        labelHapticDeviceModel[i]->setLocalPos(20, height - step, 0);
        step += 20;

        // update position of label
        labelHapticDevicePosition[i]->setLocalPos(20, height - step, 0);
        step += 25;
    }
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
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable force field
    if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField)
            cout << "> Enable force field     \r";
        else
            cout << "> Disable force field    \r";
    }

    // option - enable/disable damping
    if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \r";
        else
            cout << "> Disable damping        \r";
    }

    // option - toggle fullscreen
    if (a_key == GLFW_KEY_F)
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
    if (a_key == GLFW_KEY_M)
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
    for (int i=0; i<numHapticDevices; i++)
    {
        hapticDevice[i]->close();
    }

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

    // update position data
    for (int i=0; i<numHapticDevices; i++)
    {
        labelHapticDevicePosition[i]->setText(hapticDevicePosition[i].str(3));
    }

    // update haptic and graphic rate data
    if (numHapticDevices == 0)
    {
        labelRates->setText("no haptic device detected");
    }
    else
    {
        labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
            cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    }

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
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
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
        for (int i=0; i<numHapticDevices; i++)
        {
            /////////////////////////////////////////////////////////////////////
            // READ HAPTIC DEVICE
            /////////////////////////////////////////////////////////////////////

            // read position 
            cVector3d position;
            hapticDevice[i]->getPosition(position);

            // read orientation 
            cMatrix3d rotation;
            hapticDevice[i]->getRotation(rotation);

            // read gripper position
            double gripperAngle;
            hapticDevice[i]->getGripperAngleRad(gripperAngle);

            // read linear velocity 
            cVector3d linearVelocity;
            hapticDevice[i]->getLinearVelocity(linearVelocity);

            // read angular velocity
            cVector3d angularVelocity;
            hapticDevice[i]->getAngularVelocity(angularVelocity);

            // read gripper angular velocity
            double gripperAngularVelocity;
            hapticDevice[i]->getGripperAngularVelocity(gripperAngularVelocity);

            // read user-switch status (button 0)
            bool button0, button1, button2, button3;
            button0 = false;
            button1 = false;
            button2 = false;
            button3 = false;

            hapticDevice[i]->getUserSwitch(0, button0);
            hapticDevice[i]->getUserSwitch(1, button1);
            hapticDevice[i]->getUserSwitch(2, button2);
            hapticDevice[i]->getUserSwitch(3, button3);


            /////////////////////////////////////////////////////////////////////
            // UPDATE 3D CURSOR MODEL
            /////////////////////////////////////////////////////////////////////
       
            // update arrow
            velocity[i]->m_pointA = position;
            velocity[i]->m_pointB = cAdd(position, linearVelocity);

            // update position and orientation of cursor
            cursor[i]->setLocalPos(position);
            cursor[i]->setLocalRot(rotation);

            // adjust the  color of the cursor according to the status of
            // the user-switch (ON = TRUE / OFF = FALSE)
            if (button0)
            {
                cursor[i]->m_material->setGreenMediumAquamarine(); 
            }
            else if (button1)
            {
                cursor[i]->m_material->setYellowGold();
            }
            else if (button2)
            {
                cursor[i]->m_material->setOrangeCoral();
            }
            else if (button3)
            {
                cursor[i]->m_material->setPurpleLavender();
            }
            else
            {
                cursor[i]->m_material->setBlueRoyal();
            }

            // update global variable for graphic display update
            hapticDevicePosition[i] = position;


            /////////////////////////////////////////////////////////////////////
            // COMPUTE AND APPLY FORCES
            /////////////////////////////////////////////////////////////////////
            
            // desired position
            cVector3d desiredPosition;
            desiredPosition.set(0.0, 0.0, 0.0);

            // desired orientation
            cMatrix3d desiredRotation;
            desiredRotation.identity();

            // variables for forces    
            cVector3d force (0,0,0);
            cVector3d torque (0,0,0);
            double gripperForce = 0.0;

            // apply force field
            if (useForceField)
            {
                // compute linear force
                double Kp = 25; // [N/m]
                cVector3d forceField = Kp * (desiredPosition - position);
                force.add(forceField);

                // compute angular torque
                double Kr = 0.05; // [N/m.rad]
                cVector3d axis;
                double angle;
                cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
                deltaRotation.toAxisAngle(axis, angle);
                torque = rotation * ((Kr * angle) * axis);
            }
    
            // apply damping term
            if (useDamping)
            {
                cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

                // compute linear damping force
                double Kv = 1.0 * info.m_maxLinearDamping;
                cVector3d forceDamping = -Kv * linearVelocity;
                force.add(forceDamping);

                // compute angular damping force
                double Kvr = 1.0 * info.m_maxAngularDamping;
                cVector3d torqueDamping = -Kvr * angularVelocity;
                torque.add(torqueDamping);

                // compute gripper angular damping force
                double Kvg = 1.0 * info.m_maxGripperAngularDamping;
                gripperForce = gripperForce - Kvg * gripperAngularVelocity;
            }

            // send computed force, torque, and gripper force to haptic device
            hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
        }

        // update frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
