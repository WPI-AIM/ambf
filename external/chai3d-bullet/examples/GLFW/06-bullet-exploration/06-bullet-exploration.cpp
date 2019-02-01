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
    \version   3.2.0 $Rev: 1869 $
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
#include "CGenericDemo.h"
#include "CDemo1.h"
#include "CDemo2.h"
#include "CDemo3.h"
#include "CDemo4.h"
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

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> m_hapticDevice0;
shared_ptr<cGenericHapticDevice> m_hapticDevice1;


//---------------------------------------------------------------------------
// DEMOS
//---------------------------------------------------------------------------

//! currently active camera
cGenericDemo* m_demo;

//! Demos
cDemo1* m_demo1;
cDemo2* m_demo2;
cDemo3* m_demo3;
cDemo4* m_demo4;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

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

// initialize demos
void initDemo1();
void initDemo2();
void initDemo3();
void initDemo4();


//===========================================================================
/*
    DEMO:    06-bullet-exploration.cpp

    This example illustrates the use of the Bullet framework for simulating
    haptic interaction with dynamic bodies. 
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
    cout << "Demo: 06-bullet-exploration" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Select Demo 1" << endl;
    cout << "[2] - Select Demo 2" << endl;
    cout << "[3] - Select Demo 3" << endl;
    cout << "[4] - Select Demo 4" << endl;
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
    // HAPTIC DEVICES 
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    int numDevices = handler->getNumDevices();


    //-----------------------------------------------------------------------
    // DEMOS
    //-----------------------------------------------------------------------

    // get access to the haptic devices found
    if (numDevices > 0)
    {
        handler->getDevice(m_hapticDevice0, 0);    
    }

    if (numDevices > 1)
    {
        handler->getDevice(m_hapticDevice1, 1);
    }

    // setup demos
    m_demo1 = new cDemo1(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1);
    m_demo2 = new cDemo2(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1);
    m_demo3 = new cDemo3(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1);
    m_demo4 = new cDemo4(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1);

    // default stiffness of scene objects
    double maxStiffness = 1000.0;

    // get access to the haptic devices found
    if (numDevices > 0)
    {
        double stiffness = 0.1 * m_hapticDevice0->getSpecifications().m_maxLinearStiffness / m_demo1->m_tool0->getWorkspaceScaleFactor();
        maxStiffness = cMin(stiffness, maxStiffness);
    }

    if (numDevices > 1)
    {
        double stiffness = 0.1 * m_hapticDevice1->getSpecifications().m_maxLinearStiffness / m_demo1->m_tool1->getWorkspaceScaleFactor();
        maxStiffness = cMin(stiffness, maxStiffness);    
    }


    // set stereo mode
    m_demo1->m_camera->setStereoMode(stereoMode);
    m_demo2->m_camera->setStereoMode(stereoMode);
    m_demo3->m_camera->setStereoMode(stereoMode);
    m_demo4->m_camera->setStereoMode(stereoMode);

    // set object stiffness in demos
    m_demo1->setStiffness(maxStiffness);
    m_demo2->setStiffness(maxStiffness);
    m_demo3->setStiffness(maxStiffness);
    m_demo4->setStiffness(maxStiffness);

    // initialize demo 1
    initDemo1();


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
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void initDemo1()
{
    m_demo = m_demo1;
    m_demo->init();
}

//---------------------------------------------------------------------------

void initDemo2()
{
    m_demo = m_demo2;
    m_demo->init();
}

//---------------------------------------------------------------------------

void initDemo3()
{
    m_demo = m_demo3;
    m_demo->init();
}

//---------------------------------------------------------------------------

void initDemo4()
{
    m_demo = m_demo4;
    m_demo->init();
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//---------------------------------------------------------------------------

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
        m_demo1->m_camera->setMirrorVertical(mirroredDisplay);
        m_demo2->m_camera->setMirrorVertical(mirroredDisplay);
        m_demo3->m_camera->setMirrorVertical(mirroredDisplay);
        m_demo4->m_camera->setMirrorVertical(mirroredDisplay);
        m_demo1->m_mirroredDisplay = mirroredDisplay;
        m_demo2->m_mirroredDisplay = mirroredDisplay;
        m_demo3->m_mirroredDisplay = mirroredDisplay;
        m_demo4->m_mirroredDisplay = mirroredDisplay;
    }

    // option - start demo 1
    else if (a_key == GLFW_KEY_1)
    {
        initDemo1();
    }

    // option - start demo 2
    else if (a_key == GLFW_KEY_2)
    {
        initDemo2();
    }

    // option - start demo 3
    else if (a_key == GLFW_KEY_3)
    {
        initDemo3();
    }

    // option - start demo 4
    else if (a_key == GLFW_KEY_4)
    {
        initDemo4();
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hapticsThread;
    delete m_demo1;
    delete m_demo2;
    delete m_demo3;
    delete m_demo4;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // render world
    m_demo->updateGraphics(width, height);

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
    simulationRunning = true;

    // main haptic simulation loop
    while(simulationRunning)
    {
        m_demo->updateHaptics();
    }

    // shutdown haptic devices
    if (m_demo->m_tool0 != NULL)
    {
        m_demo->m_tool0->stop();
    }
    if (m_demo->m_tool1 != NULL)
    {
        m_demo->m_tool1->stop();
    }

    // exit haptics thread
    simulationFinished = true;
}

