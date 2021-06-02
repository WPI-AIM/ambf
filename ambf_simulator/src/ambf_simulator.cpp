//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)

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

    * Neither the name of authors nor the names of its contributors may
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \courtesy:  Starting point CHAI3D-BULLET examples by Francois Conti from <www.chai3d.org>
    \version   1.0$
*/
//==============================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
#include <afConversions.h>
#include "adf_loader_interface.h"

//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
#include <signal.h>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
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


struct CommandLineOptions{
    // Control whether to use a fixed physics timestep or not
    bool useFixedPhxTimeStep;
    // Control whether to use a fixed haptics timestep or not
    bool useFixedHtxTimeStep;
    // Physics Update Frequency
    int phxFrequency;
    // Haptics Update Frequency
    int htxFrequency;
    // Enable Force Feedback
    bool enableForceFeedback = true;
    // Number of Devices to Load
    int numDevicesToLoad;
    // A string of device indexes to load
    std::string devicesToLoad;
    // A string list of multibody indexes to load
    std::string multiBodiesToLoad;
    // A string list of multibody files to load
    std::string multiBodyFilesToLoad;
    // A string of Path of launch file to load
    std::string launchFilePath;
    // Control whether to run headless or not
    bool showGUI; //
    // Override the default world namespace
    std::string prepend_namespace;
    // The running speed of the simulation. 1.0 indicates a stepping of one second.
    double simulation_speed;

};

// Global struct for command line options
CommandLineOptions g_cmdOpts;

// Info for mouse events in case a body is picked
bool g_pickBody = false;
cVector3d g_pickFrom, g_pickTo;
afRigidBodyPtr g_lastClickedBody;

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool g_simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool g_simulationFinished = true;

// Flag to toggle between inverted/non_inverted mouse pitch with mouse
bool g_mouse_inverted_y = false;

// Ratio between Window Height and Width to Frame Buffer Height and Width
double g_winWidthRatio = 1.0;

double g_winHeightRatio = 1.0;

bool g_enableGrippingAssist = true;

bool g_enableNormalMapping = true;

// haptic thread
std::vector<cThread*> g_hapticsThreads;

// bullet simulation thread
cThread* g_physicsThread;

// swap interval for the display context (vertical synchronization)
int g_swapInterval = 0;

bool g_mousePickingEnabled = false;

// Vector of WindowCamera Handles Struct
std::vector<afCamera*> g_cameras;

// Global iterator for WindowsCamera Handle
std::vector<afCamera*>::iterator g_cameraIt;

// List of input devices
std::shared_ptr<afCollateralControlManager> g_inputDevices;

ADFLoaderInterface* g_adfLoaderPtr;

afWorld *g_afWorld;

afRenderOptions g_afRenderOptions;

afSimulatorPluginGroup g_simulatorPluginGroup;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

//callback for mouse buttons
void mouseBtnsCallback(GLFWwindow* a_window, int a_button, int a_action, int a_modes);

//callback for mouse positions
void mousePosCallback(GLFWwindow* a_window, double x_pos, double y_pos);

//callback for mouse positions
void mouseScrollCallback(GLFWwindow* a_window, double x_pos, double y_pos);

// Drag and drop callback
void dragDropCallback(GLFWwindow* window, int count, const char** paths);

// Copied from CommonRidiBodyBase.h of Bullet Physics by Erwin Coumans with
// Ray Tracing for Camera Pick and Place
cVector3d getRayTo(int x, int y, afCameraPtr a_camera);

// this function contains the main haptics simulation loop
void updateHapticDevice(void*);

//this function contains the main Bullet Simulation loop
void updatePhysics(void);

// this function closes the application
void close(void);

// this function renders the scene
void updateGraphics();

// Function to update labels
void updateLabels();

// Bullet pretick callback
void preTickCallBack(btDynamicsWorld* world, btScalar timeStep);

// Exit Handler
void exitHandler(int s);

void deleteModelAttribsInternalData(afModelAttributes* modelAttribs);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief main: This Application allows multi-manual tasks using several haptics devices.
/// Each device can perturb or control the dynamic bodies in the simulation
/// environment. The objects in the simulation are exposed via Asynchoronous
/// Framework (AMBF) to allow query and control via external applications.
/// \param argc
/// \param argv
/// \return
///
int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------
    namespace p_opt = boost::program_options;

    p_opt::options_description cmd_opts("InputDevices Application Usage");
    cmd_opts.add_options()
            ("help,h", "Show help")
            ("ndevs,n", p_opt::value<int>()->default_value(0), "Number of Haptic Devices to Load")
            ("load_devices,i", p_opt::value<std::string>()->default_value(""), "Index number of devices to load which is specified in input_device.yaml")
            ("enableforces,e", p_opt::value<bool>()->default_value(false), "Enable Force Feedback on Haptic Devices")
            ("phx_frequency,p", p_opt::value<int>()->default_value(1000), "Physics Update Frequency (default: 1000 Hz)")
            ("htx_frequency,d", p_opt::value<int>()->default_value(1000), "Haptics Update Frequency (default: 1000 Hz)")
            ("fixed_phx_timestep,t", p_opt::value<bool>()->default_value(false), "Use Fixed Time-Step for Physics (default: False)")
            ("fixed_htx_timestep,f", p_opt::value<bool>()->default_value(false), "Use Fixed Time-Step for Haptics (default: False)")
            ("load_multibody_files,a", p_opt::value<std::string>()->default_value(""), "Description Filenames of Multi-Body(ies) to Launch, .e.g. -a <path>"
                                                                    "/test.yaml, <another_path>/test2.yaml will load multibodies test.yaml"
                                                                    " and test2.yaml if they are valid files")
            ("load_multibodies,l", p_opt::value<std::string>()->default_value("1"), "Index of Multi-Body(ies) to Launch, .e.g. "
                                                                "-l 1,2,3 will load multibodies at indexes 1,2,3. See launch.yaml file")
            ("launch_file", p_opt::value<std::string>()->default_value("../../ambf_models/descriptions/launch.yaml"), "Launch file path to load (default: ../../ambf_models/descriptions/launch.yaml")
            ("show_gui,g", p_opt::value<bool>()->default_value(true), "Show GUI")
            ("ns", p_opt::value<std::string>()->default_value(""), "Override the default (or specified in ADF) world namespace")
            ("sim_speed_factor,s", p_opt::value<double>()->default_value(1.0), "Override the speed of \"NON REAL-TIME\" simulation by a specified factor (Default 1.0)");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("help")){ std::cout<< cmd_opts << std::endl; return 0;}

    if(var_map.count("ndevs")){ g_cmdOpts.numDevicesToLoad = var_map["ndevs"].as<int>();}

    if(var_map.count("load_devices")){ g_cmdOpts.devicesToLoad = var_map["load_devices"].as<std::string>();}

    if(var_map.count("phx_frequency")){ g_cmdOpts.phxFrequency = var_map["phx_frequency"].as<int>();}

    if(var_map.count("htx_frequency")){ g_cmdOpts.htxFrequency = var_map["htx_frequency"].as<int>();}

    if(var_map.count("fixed_phx_timestep")){ g_cmdOpts.useFixedPhxTimeStep = var_map["fixed_phx_timestep"].as<bool>();}

    if(var_map.count("fixed_htx_timestep")){ g_cmdOpts.useFixedHtxTimeStep = var_map["fixed_htx_timestep"].as<bool>();}

    if(var_map.count("enableforces")){ g_cmdOpts.enableForceFeedback = var_map["enableforces"].as<bool>();}

    if(var_map.count("load_multibody_files")){ g_cmdOpts.multiBodyFilesToLoad = var_map["load_multibody_files"].as<std::string>();}

    if(var_map.count("load_multibodies")){ g_cmdOpts.multiBodiesToLoad = var_map["load_multibodies"].as<std::string>();}

    if(var_map.count("launch_file")){ g_cmdOpts.launchFilePath = var_map["launch_file"].as<std::string>();}

    if(var_map.count("show_gui")){ g_cmdOpts.showGUI = var_map["show_gui"].as<bool>();}

    if(var_map.count("ns")){g_cmdOpts.prepend_namespace = var_map["ns"].as<std::string>();}

    if(var_map.count("sim_speed_factor")){g_cmdOpts.simulation_speed = var_map["sim_speed_factor"].as<double>();}

    // Process the loadMultiBodies string

    cout << endl;
    cout << "____________________________________________________________" << endl << endl;
    cout << "ASYNCHRONOUS MULTI-BODY FRAMEWORK SIMULATOR (AMBF Simulator)" << endl;
    cout << endl << endl;
    cout << "\t\t(www.aimlab.wpi.edu)" << endl;
    cout << "\t\t  (Copyright 2019-2021)" << endl;
    cout << "____________________________________________________________" << endl << endl;
    cout << "STARTUP COMMAND LINE OPTIONS: " << endl << endl;
    cout << cmd_opts << std::endl << std::endl;
    cout << "------------------------------------------------------------" << endl << endl << endl;
    cout << endl;

    if(var_map.count("sim_speed_factor")){
        if (g_cmdOpts.useFixedPhxTimeStep){
            std::cerr << "INFO! SETTING SIMULATION SPEED FACTOR TO: " << g_cmdOpts.simulation_speed << std::endl;
        }
        else{
            std::cerr << "WARNING! SIMULATION SPEED FACTOR IS ONLY CONSIDERED IN NON REAL-TIME SIMULATION\n";
        }
    }

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    if (g_cmdOpts.showGUI){

        // initialize GLFW library
        if (!glfwInit())
        {
            cout << "ERROR! FAILED TO INITIALIZE GLFW LIBRARY" << endl;
            cSleepMs(1000);
            return 1;
        }

        // set error callback
        glfwSetErrorCallback(errorCallback);

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
    }


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    g_adfLoaderPtr = new ADFLoaderInterface();

    afLaunchAttributes launchAttribs;
    afWorldAttributes worldAttribs;
    std::vector<afTeleRoboticUnitAttributes> tuAttribs;
    std::vector<afModelAttributes> modelsAttribs;
    if (g_adfLoaderPtr->loadLaunchFileAttribs(g_cmdOpts.launchFilePath, &launchAttribs) == false){
        // Safely Return the program
        return -1;
    }

    launchAttribs.resolveRelativePathAttribs();

    g_adfLoaderPtr->loadWorldAttribs(launchAttribs.m_worldFilepath.c_str(), &worldAttribs);

    std::vector<int> devIndexes;
    if (!g_cmdOpts.devicesToLoad.empty()){
        std::string loadDevIndices = g_cmdOpts.devicesToLoad;
        loadDevIndices.erase(std::remove(loadDevIndices.begin(), loadDevIndices.end(), ' '), loadDevIndices.end());
        std::stringstream ss(loadDevIndices);
        while(ss.good() )
        {
            string devIndex;
            getline( ss, devIndex, ',' );
            devIndexes.push_back(std::stoi(devIndex));
        }
    }
    else{
        for (int i = 0 ; i < g_cmdOpts.numDevicesToLoad ; i++){
            devIndexes.push_back(i);
        }
    }

    g_adfLoaderPtr->loadTeleRoboticUnitsAttribs(launchAttribs.m_inputDevicesFilepath.c_str(), &tuAttribs, devIndexes);

    // create a dynamic world.
    g_afWorld = new afWorld(g_cmdOpts.prepend_namespace);
    g_afWorld->m_physicsFrequency = g_cmdOpts.phxFrequency;
    g_afWorld->m_hapticsFrequency = g_cmdOpts.htxFrequency;
    g_afWorld->m_updateCounterLimit = g_cmdOpts.phxFrequency * 2;

    // set the background color of the environment
    g_afWorld->m_chaiWorld->m_backgroundColor.setWhite();

    //////////////////////////////////////////////////////////////////////////
    // AF MULTIBODY HANDLER
    //////////////////////////////////////////////////////////////////////////
    // The world loads the lights and cameras + windows
    g_afWorld->createFromAttribs(&worldAttribs);

    g_cameras = g_afWorld->getCameras();

    // Process the loadMultiBodyFiles string
    if (!g_cmdOpts.multiBodyFilesToLoad.empty()){
        std::vector<std::string> mbFileNames;
        std::string loadMBFilenames = g_cmdOpts.multiBodyFilesToLoad;
        std::stringstream ss(loadMBFilenames);
        while(ss.good() )
        {
            string mbFilename;
            getline( ss, mbFilename, ',' );
            mbFileNames.push_back(mbFilename);
        }
        for (uint idx = 0 ; idx < mbFileNames.size() ; idx++){
            afModelAttributes modelAttribs;
            if (g_adfLoaderPtr->loadModelAttribs(mbFileNames[idx], &modelAttribs)){
                modelsAttribs.push_back(modelAttribs);
            }
        }
    }

    // Process the Multi-body index files
    if (!g_cmdOpts.multiBodiesToLoad.empty()){
        std::vector<uint> mbIndexes;
        std::string loadMBs = g_cmdOpts.multiBodiesToLoad;
        loadMBs.erase(std::remove(loadMBs.begin(), loadMBs.end(), ' '), loadMBs.end());
        std::stringstream ss(loadMBs);
        while(ss.good() )
        {
            string mbIdx;
            getline( ss, mbIdx, ',' );
            mbIndexes.push_back(std::stoi(mbIdx));
        }
        for (uint idx = 0 ; idx < mbIndexes.size() ; idx++){
            uint fileIdx = mbIndexes[idx];
            if (fileIdx >= launchAttribs.m_modelFilepaths.size()){
                cerr << "ERROR: -l " << fileIdx << " greater than total number of files " << launchAttribs.m_modelFilepaths.size() << ". Ignoring \n";
                continue;
            }
            else{
                afModelAttributes modelAttribs;
                if (g_adfLoaderPtr->loadModelAttribs(launchAttribs.m_modelFilepaths[fileIdx].c_str(), &modelAttribs)){
                    modelsAttribs.push_back(modelAttribs);
                }
            }
        }
    }

    for (int idx = 0 ; idx < modelsAttribs.size() ; idx++){
        afModelPtr model = new afModel(g_afWorld);
        if (model->createFromAttribs(&modelsAttribs[idx])){
            g_afWorld->addModel(model);
        }
    }

    g_afWorld->m_bulletWorld->setInternalTickCallback(preTickCallBack, 0, true);

    //-----------------------------------------------------------------------------------------------------------
    // START: INTIALIZE SEPERATE WINDOWS FOR EACH WINDOW-CAMRERA PAIR
    //-----------------------------------------------------------------------------------------------------------
    for(g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        GLFWwindow* windowPtr = (*g_cameraIt)->m_window;
        GLFWmonitor* monitorPtr = (*g_cameraIt)->m_monitor;
        if (!windowPtr)
        {
            cout << "ERROR! FAILED TO CREATE OPENGL WINDOW" << endl;
            cSleepMs(1000);
            glfwTerminate();
            return 1;
        }

        //        // get width and height of window
        //        int _width, _height;
        //        glfwGetWindowSize(windowPtr, &_width, &_height);

        //        (*g_cameraIt)->m_width = _width;
        //        (*g_cameraIt)->m_height = _height;

        //        // set position of window
        //        glfwSetWindowPos(windowPtr, (*g_cameraIt)->m_win_x, (*g_cameraIt)->m_win_y);

        // set key callback
        glfwSetKeyCallback(windowPtr, keyCallback);

        // set mouse buttons callback
        glfwSetMouseButtonCallback(windowPtr, mouseBtnsCallback);

        //set mouse buttons callback
        glfwSetCursorPosCallback(windowPtr, mousePosCallback);

        //set mouse scroll callback
        glfwSetScrollCallback(windowPtr, mouseScrollCallback);

        // set resize callback
        glfwSetWindowSizeCallback(windowPtr, windowSizeCallback);

        // set drag and drop callback
        glfwSetDropCallback(windowPtr, dragDropCallback);

        // set the current context
        glfwMakeContextCurrent(windowPtr);

        glfwSwapInterval(g_swapInterval);

        windowSizeCallback((*g_cameraIt)->m_window, (*g_cameraIt)->m_width, (*g_cameraIt)->m_height);

        //        // initialize GLEW library
        //#ifdef GLEW_VERSION
        //        if (glewInit() != GLEW_OK)
        //        {
        //            cout << "ERROR! FAILED TO INITIALIZE GLEW LIBRARY" << endl;
        //            glfwTerminate();
        //            return 1;
        //        }
        //#endif
    }

    //-----------------------------------------------------------------------------------------------------------
    // END: INTIALIZE SEPERATE WINDOWS FOR EACH WINDOW-CAMRERA PAIR
    //-----------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------
    // START: INITIALIZE THREADS FOR ALL REQUIRED HAPTIC DEVICES AND PHYSICS THREAD
    //-----------------------------------------------------------------------------------------------------------
    g_inputDevices = std::make_shared<afCollateralControlManager>(g_afWorld);
    g_inputDevices->createFromAttribs(&tuAttribs);

    //-----------------------------------------------------------------------------------------------------------
    // END: SEARCH FOR CONTROLLING DEVICES FOR CAMERAS IN AMBF AND ADD THEM TO RELEVANT WINDOW-CAMERA PAIR
    //-----------------------------------------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    for (int gIdx = 0 ; gIdx < g_inputDevices->m_numDevices; gIdx++){
        g_hapticsThreads.push_back(new cThread());
        g_hapticsThreads[gIdx]->start(updateHapticDevice, CTHREAD_PRIORITY_HAPTICS, &g_inputDevices->m_collateralControlUnits[gIdx]);
    }

    //create a thread which starts the Bullet Simulation loop
    g_physicsThread = new cThread();
    g_physicsThread->start(updatePhysics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);
    //-----------------------------------------------------------------------------------------------------------
    // END: INITIALIZE THREADS FOR ALL REQUIRED HAPTIC DEVICES AND PHYSICS THREAD
    //-----------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // Assign Exit Handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    g_simulatorPluginGroup.add("libtest_simulator_plugin.so", "test_simulator_plugin");

    g_simulatorPluginGroup.init(argc, argv, g_afWorld);

    //    signal (SIGINT, exitHandler);

    // Compute the window width and height ratio
    if (g_cmdOpts.showGUI){
        int winH, winW;
        glfwGetWindowSize(g_cameras[0]->m_window, &winW, &winH);

        int buffH, buffW;
        glfwGetFramebufferSize(g_cameras[0]->m_window, &buffW, &buffH);

        g_winWidthRatio = double(buffW) / double(winW);
        g_winHeightRatio = double(buffH) / double(winH);

        // Load the skybox if defined.
        g_afWorld->loadSkyBox();

    }

    afRate graphicsRate(120);

    // main graphic loop
    while (!g_afRenderOptions.m_windowClosed)
    {
        if (g_cmdOpts.showGUI){
            // Call the update graphics method
            updateGraphics();

            // process events
            glfwPollEvents();

            // signal frequency counter
            g_afWorld->m_freqCounterGraphics.signal(1);
        }

        else{
            std::cerr << "\nRunning Headless (-g option provided) t = " << g_afWorld->g_wallClock.getCurrentTimeSeconds() << " sec" << std::endl;
            sleep(1.0);
        }
        graphicsRate.sleep();
    }

    // close window
    for (g_cameraIt = g_cameras.begin(); g_cameraIt !=  g_cameras.end() ; ++ g_cameraIt){
        glfwDestroyWindow((*g_cameraIt)->m_window);
    }


    // terminate GLFW library
    glfwTerminate();

    for (int idx = 0 ; idx < modelsAttribs.size() ; idx++){
        deleteModelAttribsInternalData(&modelsAttribs[idx]);
    }

    // exit
    return 0;
}

///
/// \brief updateGraphics
///
void updateGraphics()
{
    g_afRenderOptions.m_IIDModeStr = g_inputDevices->m_mode_str;
    g_afRenderOptions.m_IIDBtnActionStr = g_inputDevices->m_btn_action_str;

    g_afWorld->render(g_afRenderOptions);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    g_simulationRunning = true;
}


///
/// \brief updatePhysics
///
void updatePhysics(){

    // Wait for the graphics loop to start the sim
    while (g_simulationRunning == false){
        cSleepMs(1);
    }

    g_simulationFinished = false;

    // start haptic device
    g_afWorld->g_wallClock.start(true);

    afRate phxSleep(g_cmdOpts.phxFrequency);
    bool bodyPicked = false;

    double dt_fixed = 1.0 / g_cmdOpts.phxFrequency;

    cVector3d torque, torque_prev;
    torque.set(0, 0, 0);
    torque_prev.set(0, 0, 0);
    while(g_simulationRunning)
    {
        g_afWorld->m_freqCounterHaptics.signal(1);

        // Take care of any picked body by mouse
        if (g_pickBody){
            if (bodyPicked == false){
                bodyPicked = true;
                g_afWorld->pickBody(g_pickFrom, g_pickTo);
            }
            else{
                g_afWorld->movePickedBody(g_pickFrom, g_pickTo);
            }
        }
        else{
            bodyPicked = false;
            g_afWorld->removePickingConstraint();
        }

        double step_size;
        if (g_cmdOpts.useFixedPhxTimeStep){
            step_size = g_cmdOpts.simulation_speed * (1.0 / g_cmdOpts.phxFrequency);
        }
        else{
            step_size = g_afWorld->computeStepSize(true);
        }

        for (unsigned int devIdx = 0 ; devIdx < g_inputDevices->m_numDevices ; devIdx++){
            // update position of simulate gripper
            afSimulatedDevice * simDev = g_inputDevices->m_collateralControlUnits[devIdx].m_simulatedDevicePtr;
            afRigidBodyPtr rootLink = simDev->m_rootLink;
            simDev->updateGlobalPose();

            if (g_enableGrippingAssist){
                for (int sIdx = 0 ; sIdx < rootLink->getSensors().size() ; sIdx++){
                    afSensorPtr sensorPtr = rootLink->getSensors()[sIdx];
                    if (sensorPtr->m_sensorType == afSensorType::RAYTRACER){
                        afProximitySensor* proximitySensorPtr = (afProximitySensor*) sensorPtr;
                        if (simDev->m_gripper_angle < 0.5){
                            for (int i = 0 ; i < proximitySensorPtr->getCount() ; i++){
                                if (proximitySensorPtr->getSensedBodyType(i) == afBodyType::RIGID_BODY){
                                    // Check if the sensed body belongs to the gripper, in which case ignore
                                    afRigidBodyPtr afBody = proximitySensorPtr->getSensedRigidBody(i);
                                    if (rootLink->isChild(afBody->m_bulletRigidBody) == true){
                                        continue;
                                    }
                                }
                                simDev->m_grippingConstraint->actuate(sensorPtr);
                            }
                        }
                        else{
                            simDev->m_grippingConstraint->deactuate();
                        }
                    }
                }
            }

            cVector3d pCommand, rCommand;
            // ts is to prevent the saturation of forces
            double ts = dt_fixed / step_size;
            pCommand = simDev->m_rootLink->m_controller.computeOutput<cVector3d>(simDev->getPos(), simDev->getPosRef(), step_size, 1);
            pCommand = simDev->P_lc_ramp * pCommand;

            rCommand = simDev->m_rootLink->m_controller.computeOutput<cVector3d>(simDev->getRot(), simDev->getRotRef(), step_size, 1);
            rCommand = simDev->P_ac_ramp * rCommand;

            if (simDev->m_rootLink->m_controller.m_positionOutputType == afControlType::FORCE){
                simDev->applyForce(pCommand);
            }
            else{
                simDev->m_rootLink->m_bulletRigidBody->setLinearVelocity(to_btVector(pCommand));
            }

            if (simDev->m_rootLink->m_controller.m_orientationOutputType == afControlType::FORCE){
                simDev->applyTorque(rCommand);
            }
            else{
                simDev->m_rootLink->m_bulletRigidBody->setAngularVelocity(to_btVector(rCommand));
            }

            // Control simulated body joints only if joint control of this physical device has been enabled
            if (simDev->isJointControlEnabled()){
                simDev->setGripperAngle(simDev->m_gripper_angle);
            }

            if (simDev->P_lc_ramp < 1.0)
            {
                simDev->P_lc_ramp = simDev->P_lc_ramp + 0.5 * step_size;
            }
            else
            {
                simDev->P_lc_ramp = 1.0;
            }

            if (simDev->P_ac_ramp < 1.0)
            {
                simDev->P_ac_ramp = simDev->P_ac_ramp + 0.5 * step_size;
            }
            else
            {
                simDev->P_ac_ramp = 1.0;
            }
        }
        g_afWorld->updateDynamics(step_size, g_afWorld->g_wallClock.getCurrentTimeSeconds(), g_afWorld->m_freqCounterHaptics.getFrequency(), g_inputDevices->m_numDevices);
        phxSleep.sleep();
    }
    g_simulationFinished = true;
}

///
/// \brief updateHaptics
/// \param a_arg
///
void updateHapticDevice(void* ccuPtr){
    afCollateralControlUnit ccu = *(afCollateralControlUnit*) ccuPtr;

    // Wait for the graphics loop to start the sim
    while(g_simulationRunning == false){
        cSleepMs(1);
    }

    // simulation in now running
    g_simulationFinished = false;

    afRate htxSleep(g_cmdOpts.htxFrequency);

    // update position and orientation of simulated gripper
    std::string identifyingName = ccu.m_name;
    afPhysicalDevice *phyDev = ccu.m_physicalDevicePtr;
    afSimulatedDevice* simDev = ccu.m_simulatedDevicePtr;
    std::vector<afCameraPtr> devCams = ccu.m_cameras;
    cLabel* devFreqLabel = ccu.m_devFreqLabel;
    if (ccu.m_cameras.size() == 0){
        cerr << "WARNING: DEVICE HAPTIC LOOP \"" << phyDev->m_hInfo.m_modelName << "\" NO WINDOW-CAMERA PAIR SPECIFIED, USING DEFAULT" << endl;
        devCams = g_cameras;
    }

    phyDev->setPosClutched(cVector3d(0.0,0.0,0.0));
    phyDev->getRot();
    phyDev->setRotClutched(cMatrix3d(0, 0, 0));
    simDev->setRotRefOrigin(phyDev->getRot());

    double P_lc_offset = 10;
    double P_ac_offset = 1;
    double D_lc_offset = 1;
    double D_ac_offset = 1;
    double P_lh_offset = 5;
    double P_ah_offset = 1;

    double wait_time = 3.0;
    if (std::strcmp(phyDev->m_hInfo.m_modelName.c_str(), "Razer Hydra") == 0 ){
        wait_time = 5.0;
    }

    cVector3d force, torque;
    force.set(0,0,0);
    torque.set(0,0,0);

    // main haptic simulation loop
    while(g_simulationRunning)
    {
        if (!g_afWorld->isPhysicsPaused() || g_afWorld->getManualSteps() > 0){
            phyDev->m_freq_ctr.signal(1);
            if (devFreqLabel != NULL){
                devFreqLabel->setText(identifyingName + " [" + phyDev->m_hInfo.m_modelName + "] " + ": " + cStr(phyDev->m_freq_ctr.getFrequency(), 0) + " Hz");
            }
            // Adjust time dilation by computing dt from clockWorld time and the simulationTime
            double dt;
            if (g_cmdOpts.useFixedHtxTimeStep){

                dt = g_cmdOpts.simulation_speed * (1.0 / g_cmdOpts.htxFrequency);
            }
            else{
                dt = g_afWorld->computeStepSize();
            }

            if(phyDev->m_gripper_pinch_btn >= 0){
                if(phyDev->isButtonPressed(phyDev->m_gripper_pinch_btn)){
                    phyDev->enableForceFeedback(true);
                }
            }
            if (phyDev->m_hInfo.m_sensedGripper){
                simDev->m_gripper_angle = phyDev->getGripperAngle();
            }
            else if (phyDev->m_buttons.G1 > 0){
                // Some devices may have a gripper button instead of a continous gripper.
                simDev->m_gripper_angle = !phyDev->isButtonPressed(phyDev->m_buttons.G1);
            }
            else{
                simDev->m_gripper_angle = 0.5;
            }

            if(phyDev->isButtonPressRisingEdge(phyDev->m_buttons.NEXT_MODE)) g_inputDevices->nextMode();
            if(phyDev->isButtonPressRisingEdge(phyDev->m_buttons.PREV_MODE)) g_inputDevices->prevMode();

            bool btn_1_rising_edge = phyDev->isButtonPressRisingEdge(phyDev->m_buttons.A1);
            bool btn_1_falling_edge = phyDev->isButtonPressFallingEdge(phyDev->m_buttons.A1);
            bool btn_2_rising_edge = phyDev->isButtonPressRisingEdge(phyDev->m_buttons.A2);
            bool btn_2_falling_edge = phyDev->isButtonPressFallingEdge(phyDev->m_buttons.A2);

            double gripper_offset = 0;
            switch (g_inputDevices->m_simModes){
            case MODES::CAM_CLUTCH_CONTROL:
                g_inputDevices->m_clutch_btn_pressed  = phyDev->isButtonPressed(phyDev->m_buttons.A1);
                g_inputDevices->m_cam_btn_pressed     = phyDev->isButtonPressed(phyDev->m_buttons.A2);
                if(g_inputDevices->m_clutch_btn_pressed) g_inputDevices->m_btn_action_str = "Clutch Pressed";
                if(g_inputDevices->m_cam_btn_pressed)   {g_inputDevices->m_btn_action_str = "Cam Pressed";}
                if(btn_1_falling_edge || btn_2_falling_edge) g_inputDevices->m_btn_action_str = "";
                break;
            case MODES::GRIPPER_JAW_CONTROL:
                if (btn_1_rising_edge) gripper_offset = 0.1;
                if (btn_2_rising_edge) gripper_offset = -0.1;
                simDev->offsetGripperAngle(gripper_offset);
                break;
            case MODES::CHANGE_CONT_LIN_GAIN:
                if(btn_1_rising_edge) g_inputDevices->increment_P_lc(P_lc_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_P_lc(-P_lc_offset);
                break;
            case MODES::CHANGE_CONT_ANG_GAIN:
                if(btn_1_rising_edge) g_inputDevices->increment_P_ac(P_ac_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_P_ac(-P_ac_offset);
                break;
            case MODES::CHANGE_CONT_LIN_DAMP:
                if(btn_1_rising_edge) g_inputDevices->increment_D_lc(D_lc_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_D_lc(-D_lc_offset);
                break;
            case MODES::CHANGE_CONT_ANG_DAMP:
                if(btn_1_rising_edge) g_inputDevices->increment_D_ac(D_ac_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_D_ac(-D_ac_offset);
                break;
            case MODES::CHANGE_DEV_LIN_GAIN:
                if(btn_1_rising_edge) g_inputDevices->increment_K_lh(P_lh_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_K_lh(-P_lh_offset);
                break;
            case MODES::CHANGE_DEV_ANG_GAIN:
                if(btn_1_rising_edge) g_inputDevices->increment_K_ah(P_ah_offset);
                if(btn_2_rising_edge) g_inputDevices->increment_K_ah(-P_ah_offset);
                break;
            }

            devCams[0]->m_cam_pressed = phyDev->isButtonPressed(phyDev->m_buttons.A2);
            if(devCams[0]->m_cam_pressed && g_inputDevices->m_simModes == MODES::CAM_CLUTCH_CONTROL){
                double scale = 0.01;
                for (int dcIdx = 0 ; dcIdx < devCams.size() ; dcIdx++){
                    devCams[dcIdx]->setLocalPos(devCams[dcIdx]->getLocalPos() + cMul(scale, devCams[dcIdx]->getLocalRot() * phyDev->getLinVel() ) );
                    devCams[dcIdx]->setLocalRot(phyDev->getRotCamPreClutch() * cTranspose(phyDev->getRotPreClutch()) * phyDev->getRot());
                }

            }
            if (!devCams[0]->m_cam_pressed){
                phyDev->setRotCamPreClutch( devCams[0]->getLocalRot() );
                phyDev->setRotPreClutch( phyDev->getRot() );
            }


            if (g_afWorld->g_wallClock.getCurrentTimeSeconds() < wait_time){
                phyDev->setPosClutched(phyDev->getPos());
            }

            if(g_inputDevices->m_cam_btn_pressed){
                if(phyDev->btn_cam_rising_edge){
                    phyDev->btn_cam_rising_edge = false;
                    simDev->setPosRefOrigin(simDev->getPosRef());
                    simDev->setRotRefOrigin(simDev->getRotRef());
                }
                phyDev->setPosClutched(phyDev->getPos());
                phyDev->setRotClutched(phyDev->getRot());
            }
            else{
                phyDev->btn_cam_rising_edge = true;
            }
            if(g_inputDevices->m_clutch_btn_pressed){
                if(phyDev->btn_clutch_rising_edge){
                    phyDev->btn_clutch_rising_edge = false;
                    simDev->setPosRefOrigin(simDev->getPosRef());
                    simDev->setRotRefOrigin(simDev->getRotRef());
                }
                phyDev->setPosClutched(phyDev->getPos());
                phyDev->setRotClutched(phyDev->getRot());
            }
            else{
                phyDev->btn_clutch_rising_edge = true;
            }

            simDev->setPosRef(simDev->getPosRefOrigin() + phyDev->m_workspaceScale * (devCams[0]->getLocalRot() * (phyDev->getPos() - phyDev->getPosClutched() ) ) ) ;
            if (!g_inputDevices->m_use_cam_frame_rot){
                simDev->setRotRef(simDev->getRotRefOrigin() * devCams[0]->getLocalRot() *
                        cTranspose(phyDev->getRotClutched()) * phyDev->getRot() *
                        cTranspose(devCams[0]->getLocalRot()));
            }
            else{
                simDev->setRotRef(simDev->getSimRotInitial() * phyDev->getSimRotOffset() * phyDev->getRot() * phyDev->getSimRotOffsetInverse());
            }

            if (phyDev->m_showMarker){
                phyDev->m_refSphere->setLocalPos(simDev->getPosRef());
                phyDev->m_refSphere->setLocalRot(simDev->getRotRef());
            }

            // update position of simulated gripper
            simDev->updateGlobalPose();

            force  = - !g_inputDevices->m_clutch_btn_pressed * g_cmdOpts.enableForceFeedback * phyDev->P_lh_ramp * (phyDev->m_controller.computeOutput<cVector3d, cVector3d>(simDev->getPos(), simDev->getPosRef(), dt));
            torque = - !g_inputDevices->m_clutch_btn_pressed * g_cmdOpts.enableForceFeedback * phyDev->P_ah_ramp * (phyDev->m_controller.computeOutput<cVector3d, cMatrix3d>(simDev->getRot(), simDev->getRotRef(), dt));

            // Orient the wrench along the camera view
            force = cTranspose(devCams[0]->getLocalRot()) * force;
            torque = cTranspose(devCams[0]->getLocalRot()) * torque;

            if (force.length() < phyDev->m_deadBand){
                force.set(0,0,0);
            }

            if (force.length() > phyDev->m_maxForce){
                force.normalize();
                force = force * phyDev->m_maxForce;
            }

            if (torque.length() < phyDev->m_deadBand){
                torque.set(0,0,0);
            }

            phyDev->applyWrench(force, torque);

            if (phyDev->P_lh_ramp < 1.0)
            {
                phyDev->P_lh_ramp = phyDev->P_lh_ramp + 0.1 * dt * phyDev->m_controller.P_lin;
            }
            else
            {
                phyDev->P_lh_ramp = 1.0;
            }

            if (phyDev->P_ah_ramp < 1.0)
            {
                phyDev->P_ah_ramp = phyDev->P_ah_ramp + 0.1 * dt * phyDev->m_controller.P_ang;
            }
            else
            {
                phyDev->P_ah_ramp = 1.0;
            }

        }
        htxSleep.sleep();
    }
    // exit haptics thread
}

void exitHandler(int s){
    std::cerr << "\n(CTRL-C) Caught Signal " << s << std::endl;
    g_afRenderOptions.m_windowClosed = true;
}

void deleteModelAttribsInternalData(afModelAttributes* modelAttribs){
    for (int i = 0 ; i < modelAttribs->m_sensorAttribs.size() ; i++){
        delete modelAttribs->m_sensorAttribs[i];
    }
    for (int i = 0 ; i < modelAttribs->m_actuatorAttribs.size() ; i++){
        delete modelAttribs->m_actuatorAttribs[i];
    }
}


///
/// \brief windowSizeCallback
/// \param a_window
/// \param a_width
/// \param a_height
///
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    std::vector<afCameraPtr>::iterator cameraIt;

    for(cameraIt = g_cameras.begin(); cameraIt != g_cameras.end() ; ++cameraIt){
        if( (*cameraIt)->m_window == a_window){
            // update window size
            (*cameraIt)->m_width = a_width;
            (*cameraIt)->m_height = a_height;
        }
    }

}

///
/// \brief dragDropCallback
/// \param window
/// \param count
/// \param paths
///
void dragDropCallback(GLFWwindow* windowPtr, int count, const char** paths){
    int i;
    for (i = 0;  i < count;  i++){
        // Check if the file name ends in a .yaml or .ambf file. Only then attempt
        // to load other file, otherwise throw a warning that we are ignoring this
        // file
        std::string extension = paths[i];
        unsigned long int findIdx = extension.find_last_of(".");
        std::cerr << "FIND IDX == " << findIdx << std::endl;
        if (findIdx != std::string::npos){
            extension = extension.substr(findIdx);

            if (! extension.compare(".yaml") || ! extension.compare(".YAML") || ! extension.compare(".ambf") || ! extension.compare(".AMBF") ){
                std::cerr << "LOADING DRAG AND DROPPED FILE NAMED: " << paths[i] << std::endl;
                g_afWorld->pausePhysics(true);
                afModelAttributes modelAttribs;
                if (g_adfLoaderPtr->loadModelAttribs(paths[i], &modelAttribs)){
                    afModelPtr model = new afModel(g_afWorld);
                    if (model->createFromAttribs(&modelAttribs)){
                        g_afWorld->addModel(model);
                    }
                    else{
                        delete model;
                    }
                }
            }
            else{
                std::cerr << "INVALID EXTENSION: \"" << paths[i] << "\". ONLY \".AMBF\" OR \".YAML\" SUPPORTED \n";
            }
        }
        else{
            std::cerr << "INVALID DRAG AND DROP OF DIRECTORY \"" << paths[i] << "\". PLEASE ONLY D&D FILES \n";
        }
    }
    g_afWorld->pausePhysics(false);
}

///
/// \brief errorCallback
/// \param a_error
/// \param a_description
///
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

///
/// \brief keyCallback
/// \param a_window
/// \param a_key
/// \param a_scancode
/// \param a_action
/// \param a_mods
///
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

    if (a_mods == GLFW_MOD_CONTROL){
        // MODS IF THE CTRL KEY IS PRESSED
        // option - If CTRL R is pressed, reset the simulation
        if (a_key == GLFW_KEY_R){
            printf("Resetting the Simulation\n");
            g_afWorld->resetDynamicBodies();

            // Reset the clutched position of all Physical devices to their
            // simulated dynamic end-effectors
            std::vector<afCollateralControlUnit*> ccu_vec  = g_inputDevices->getAllCollateralControlUnits();
            for (int ccuIdx = 0 ; ccuIdx < ccu_vec.size() ; ccuIdx++){
                afPhysicalDevice* pDev = ccu_vec[ccuIdx]->m_physicalDevicePtr;
                afSimulatedDevice* sDev = ccu_vec[ccuIdx]->m_simulatedDevicePtr;
                pDev->setPosClutched(pDev->getPos());
                pDev->setRotClutched(pDev->getRot());
                sDev->setPosRef(sDev->m_rootLink->getInitialTransform().getLocalPos());
                sDev->setRotRef(sDev->m_rootLink->getInitialTransform().getLocalRot());
                sDev->setPosRefOrigin(sDev->m_rootLink->getInitialTransform().getLocalPos() / pDev->m_workspaceScale);
                sDev->setRotRefOrigin(sDev->m_rootLink->getInitialTransform().getLocalRot());
            }
        }

        // option - If CTRL V is pressed, reset the simulation
        else if (a_key == GLFW_KEY_V){
            printf("Resetting Camera Views\n");
            g_afWorld->resetCameras();
        }


        // option - If CTRL X is pressed, reset the simulation
        else if (a_key == GLFW_KEY_X){
            g_afWorld->pausePhysics(true);
            if (g_afWorld->m_pickedRigidBody != nullptr){
                printf("Removing Last Picked Body Named: \"%s\"\n", g_afWorld->m_pickedRigidBody->m_name.c_str());
                g_afWorld->m_pickedRigidBody->remove();
            }
            else{
                printf("Last Picked Body Not Valid for Removal\n");
            }
            g_afWorld->pausePhysics(false);
        }

        // option - Pause Physics
        else if (a_key == GLFW_KEY_P)
        {
            bool pause_phx = g_afWorld->isPhysicsPaused();
            // Toggle;
            pause_phx = !pause_phx;
            g_afWorld->pausePhysics(pause_phx);
            printf("Pausing Physics: %i\n", pause_phx);
        }

        // option - Step Physics
        else if (a_key == GLFW_KEY_SPACE)
        {
            if(g_afWorld->isPhysicsPaused()){
                g_afWorld->stepPhysicsManually(10);
                printf("Stepping Physics by 10 Step \n");
            }
        }

    }
    else if (a_mods == GLFW_MOD_SHIFT){
        if (a_key == GLFW_KEY_N){
            afRigidBodyVec rbVec = g_afWorld->getRigidBodies();
            g_enableNormalMapping = ! g_enableNormalMapping;
            printf("Toggling Normal Mapping ON/OFF %d \n", g_enableNormalMapping);
            for (int i = 0 ; i < rbVec.size() ; i++){
                if (rbVec[i]->isShaderPgmDefined()){
                    if (rbVec[i]->m_visualMesh->getShaderProgram()){
                        rbVec[i]->m_visualMesh->getShaderProgram()->setUniformi("vEnableNormalMapping", g_enableNormalMapping);
                    }
                }
            }
        }

        // option - Toogle visibility of body frames and softbody skeleton
        else if (a_key == GLFW_KEY_V){
            printf("Toggling Frame Visibility ON/OFF\n");
            if (g_afWorld->m_pickedRigidBody != nullptr){
                g_afWorld->m_pickedRigidBody->toggleFrameVisibility();
            }
            else{
            }
        }
    }
    else{

        // option - grippers orientation w.r.t contextual camera
        if (a_key == GLFW_KEY_C){
            g_inputDevices->m_use_cam_frame_rot = true;
            printf("Gripper Rotation w.r.t Camera Frame:\n");
        }

        // option - toggle fullscreen
        if (a_key == GLFW_KEY_F)
        {
            // toggle state variable
            fullscreen = !fullscreen;

            std::vector<afCameraPtr>::iterator cameraIt;
            for (cameraIt = g_cameras.begin() ; cameraIt != g_cameras.end() ; ++cameraIt){

                // get handle to monitor
                GLFWmonitor* monitor = (*cameraIt)->m_monitor;

                // get handle to window
                GLFWwindow* window = (*cameraIt)->m_window;

                // get information about monitor
                const GLFWvidmode* mode = glfwGetVideoMode(monitor);

                // set fullscreen or window mode
                if (fullscreen)
                {
                    glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
                    glfwSwapInterval(g_swapInterval);
                }
                else
                {
                    int w = 0.8 * mode->height;
                    int h = 0.5 * mode->height;
                    int x = 0.5 * (mode->width - w);
                    int y = 0.5 * (mode->height - h);
                    glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
                    glfwSwapInterval(g_swapInterval);
                }
            }
        }

        // option - help menu
        else if (a_key == GLFW_KEY_H)
        {
            cout << "Keyboard Options:" << endl << endl;
            cout << "[h] - Display help menu" << endl;
            cout << "[1] - Enable gravity" << endl;
            cout << "[2] - Disable gravity" << endl << endl;
            cout << "[3] - decrease linear haptic gain" << endl;
            cout << "[4] - increase linear haptic gain" << endl;
            cout << "[5] - decrease angular haptic gain" << endl;
            cout << "[6] - increase angular haptic gain" << endl << endl;
            cout << "[7] - decrease linear stiffness" << endl;
            cout << "[8] - increase linear stiffness" << endl;
            cout << "[9] - decrease angular stiffness" << endl;
            cout << "[0] - increase angular stiffness" << endl;
            cout << "[PgUp] - increase linear damping" << endl;
            cout << "[PgDown] - decrease linear damping" << endl;
            cout << "[Home] - increate angular damping" << endl;
            cout << "[End] - decrease angular damping" << endl << endl;
            cout << "[v] - toggle frame/skeleton visualization" << endl;
            cout << "[s] - toggle sensors visibility" << endl;
            cout << "[w] - use world frame for orientation clutch" << endl;
            cout << "[c] - use9999 camera frame for orientation clutch" << endl;
            cout << "[n] - next device mode" << endl << endl;
            cout << "[i] - toogle inverted y for camera control via mouse" << endl << endl;
            cout << "[t] - toogle gripper picking constraints" << endl << endl;
            cout << "[p] - toogle mouse picking constraints" << endl << endl;
            cout << "[u] - toogle update of labels" << endl << endl;
            cout << "[CTRL + R] - Reset the Dynamic Bodies" << endl << endl;
            cout << "[CTRL + X] - Remove Last Picked Body" << endl << endl;
            cout << "[CTRL + V] - Reset the Camera Views" << endl << endl;
            cout << "[q] - Exit application\n" << endl;
            cout << endl << endl;
        }

        // option - Toggle Inverted Y axis of mouse for camera control
        else if (a_key == GLFW_KEY_I){
            g_mouse_inverted_y = !g_mouse_inverted_y;
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_M){
            g_afRenderOptions.m_mirroredDisplay = ! g_afRenderOptions.m_mirroredDisplay;
            std::vector<afCameraPtr>::iterator cameraIt;
            for (cameraIt = g_cameras.begin() ; cameraIt != g_cameras.end() ; ++cameraIt){
                (*cameraIt)->setMirrorVertical( g_afRenderOptions.m_mirroredDisplay);
            }
        }

        // option - Change to next device mode
        else if (a_key == GLFW_KEY_N){
            g_inputDevices->nextMode();
            printf("Changing to next device mode:\n");
        }

        // option - Toogle mouse picking
        else if (a_key == GLFW_KEY_P){
            g_mousePickingEnabled = !g_mousePickingEnabled;
        }

        // option - Toggle Visibility of Sensors
        else if (a_key == GLFW_KEY_S){
            auto sMap = g_afWorld->getSensorMap();
            afBaseObjectMap::const_iterator sIt;
            for (sIt = sMap->begin() ; sIt != sMap->end(); ++sIt){
                ((afSensorPtr)sIt->second)->toggleSensorVisibility();
            }
        }

        // option - Toggle Ray Test for Gripper Picking
        else if (a_key == GLFW_KEY_T){
            g_enableGrippingAssist = !g_enableGrippingAssist;
        }

        // option - Toggle visibility of label updates
        else if (a_key == GLFW_KEY_U){
            g_afRenderOptions.m_updateLabels = !g_afRenderOptions.m_updateLabels;
        }

        // option - Toogle visibility of body frames and softbody skeleton
        else if (a_key == GLFW_KEY_V){
            auto cMap = g_afWorld->getChildrenMap();
            afChildrenMap::const_iterator cIt;
            for (cIt = cMap->begin() ; cIt != cMap->end() ; ++cIt){
                afBaseObjectMap objMap = cIt->second;
                afBaseObjectMap::iterator objIt;
                for (objIt = objMap.begin(); objIt != objMap.end(); ++objIt){
                    objIt->second->toggleFrameVisibility();
                }
            }
        }

        // option - grippers orientation w.r.t world
        else if (a_key == GLFW_KEY_W){
            g_inputDevices->m_use_cam_frame_rot = false;
            printf("Gripper Rotation w.r.t World Frame:\n");
        }
        // option - enable gravity
        else if (a_key == GLFW_KEY_1)
        {
            // enable gravity
            afVector3d g(0.0, 0.0, -9.8);
            g_afWorld->setGravity(g);
            printf("gravity ON:\n");
        }

        // option - disable gravity
        else if (a_key == GLFW_KEY_2)
        {
            // disable gravity
            afVector3d g(0.0, 0.0, 0.0);
            g_afWorld->setGravity(g);
            printf("gravity OFF:\n");
        }

        // option - decrease linear haptic gain
        else if (a_key == GLFW_KEY_3)
        {
            printf("linear haptic gain:  %f\n", g_inputDevices->increment_K_lh(-0.05));
        }

        // option - increase linear haptic gain
        else if (a_key == GLFW_KEY_4)
        {
            printf("linear haptic gain:  %f\n", g_inputDevices->increment_K_lh(0.05));
        }

        // option - decrease angular haptic gain
        else if (a_key == GLFW_KEY_5)
        {
            printf("angular haptic gain:  %f\n", g_inputDevices->increment_K_ah(-0.05));
        }

        // option - increase angular haptic gain
        else if (a_key == GLFW_KEY_6)
        {
            printf("angular haptic gain:  %f\n", g_inputDevices->increment_K_ah(0.05));
        }

        // option - decrease linear stiffness
        else if (a_key == GLFW_KEY_7)
        {
            printf("linear stiffness:  %f\n", g_inputDevices->increment_P_lc(-50));
        }

        // option - increase linear stiffness
        else if (a_key == GLFW_KEY_8)
        {
            printf("linear stiffness:  %f\n", g_inputDevices->increment_P_lc(50));
        }

        // option - decrease angular stiffness
        else if (a_key == GLFW_KEY_9)
        {
            printf("angular stiffness:  %f\n", g_inputDevices->increment_P_ac(-1));
        }

        // option - increase angular stiffness
        else if (a_key == GLFW_KEY_0)
        {
            printf("angular stiffness:  %f\n", g_inputDevices->increment_P_ac(1));
        }

        // option - decrease linear damping
        else if (a_key == GLFW_KEY_PAGE_DOWN)
        {
            printf("linear damping:  %f\n", g_inputDevices->increment_D_lc(-0.1));
        }

        // option - increase linear damping
        else if (a_key == GLFW_KEY_PAGE_UP)
        {
            printf("linear damping:  %f\n", g_inputDevices->increment_D_lc(0.1));
        }

        // option - decrease angular damping
        else if (a_key == GLFW_KEY_END)
        {
            printf("angular damping:  %f\n", g_inputDevices->increment_D_ac(-0.1));
        }

        // option - increase angular damping
        else if (a_key == GLFW_KEY_HOME)
        {
            printf("angular damping:  %f\n", g_inputDevices->increment_D_ac(0.1));
        }

        // option - step physics
        else if (a_key == GLFW_KEY_SPACE)
        {
            if(g_afWorld->isPhysicsPaused()){
                g_afWorld->stepPhysicsManually(1);
                printf("Stepping Physics by 1 Step \n");
            }
        }
    }

    g_simulatorPluginGroup.keyboardUpdate(a_key, a_scancode, a_action, a_mods);
}

///
/// \brief mouseBtnCallback
/// \param window
/// \param a_button1
/// \param a_button2
/// \param a_button3
/// \param a_button4
///
void mouseBtnsCallback(GLFWwindow* a_window, int a_button, int a_clicked, int a_modes){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            if (a_button == GLFW_MOUSE_BUTTON_1){
                (*g_cameraIt)->mouse_l_clicked = a_clicked;
                //                (*g_cameraIt)->showTargetPos(true);
                if (a_clicked){
                    if (g_mousePickingEnabled){
                        cVector3d rayFrom = (*g_cameraIt)->getGlobalTransform().getLocalPos();
                        double x_pos, y_pos;
                        glfwGetCursorPos(a_window, &x_pos, &y_pos);
                        cVector3d rayTo = getRayTo(x_pos, y_pos, *g_cameraIt);
                        g_pickFrom = rayFrom;
                        g_pickTo = rayTo;
                        g_pickBody = true;
                    }
                }
                else{
                    g_pickBody = false;
                }
            }
            else if (a_button == GLFW_MOUSE_BUTTON_2){
                (*g_cameraIt)->mouse_r_clicked = a_clicked;
            }
            else if (a_button == GLFW_MOUSE_BUTTON_3){
                int state = glfwGetKey(a_window, GLFW_KEY_LEFT_SHIFT);
                if (state == GLFW_PRESS){
                    (*g_cameraIt)->mouse_l_clicked = a_clicked;
                }
                else{
                    (*g_cameraIt)->mouse_scroll_clicked = a_clicked;
                }
            }
        }
    }
}


///
/// \brief mousePosCallback
/// \param a_window
/// \param a_xpos
/// \param a_ypos
///
void mousePosCallback(GLFWwindow* a_window, double a_xpos, double a_ypos){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            int state = glfwGetKey(a_window, GLFW_KEY_LEFT_CONTROL);
            double speed_scale = 1.0;
            if (state == GLFW_PRESS)
            {
                speed_scale = 0.1;
            }
            afCameraPtr devCam = (*g_cameraIt);
            (*g_cameraIt)->mouse_x[1] = (*g_cameraIt)->mouse_x[0];
            (*g_cameraIt)->mouse_x[0] = a_xpos;
            (*g_cameraIt)->mouse_y[1] = (*g_cameraIt)->mouse_y[0];
            (*g_cameraIt)->mouse_y[0] = a_ypos;

            if( devCam->mouse_l_clicked ){
                if(g_mousePickingEnabled){
                    cVector3d rayFrom = (*g_cameraIt)->getGlobalTransform().getLocalPos();
                    cVector3d rayTo = getRayTo(a_xpos, a_ypos, (*g_cameraIt));
                    g_pickFrom = rayFrom;
                    g_pickTo = rayTo;
                }
                else{
                    double scale = 0.01;
                    double x_vel = speed_scale * scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]);
                    double y_vel = speed_scale * scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]);
                    if (g_mouse_inverted_y){
                        y_vel = -y_vel;
                    }
                    cVector3d camVel(0, -x_vel, y_vel);
                    cVector3d dPos = devCam->getLocalPos() + devCam->getLocalRot() * camVel;
                    devCam->setLocalPos(dPos);
                }
            }

            if( devCam->mouse_r_clicked ){
                cMatrix3d camRot;
                double scale = 0.3;
                double yawVel = speed_scale * scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]); // Yaw
                double pitchVel = speed_scale * scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]); // Pitch
                if (g_mouse_inverted_y){
                    pitchVel = -pitchVel;
                }

                cVector3d nz(0, 0, 1);
                cVector3d ny(0, 1, 0);

                cMatrix3d camViewWithoutPitch(cCross(devCam->getRightVector(), nz), devCam->getRightVector() ,nz);
                cMatrix3d camViewPitchOnly;
                // Use the look vector to avoid locking view at horizon
                double pitchAngle = cAngle(nz, devCam->getLookVector()) - (C_PI/2);
                camViewPitchOnly.setAxisAngleRotationRad(ny, -pitchAngle);
                camRot.setIntrinsicEulerRotationDeg(0, pitchVel, yawVel, cEulerOrder::C_EULER_ORDER_XYZ);
                (*g_cameraIt)->camRot = camRot;

                devCam->setLocalRot( camViewWithoutPitch * (*g_cameraIt)->camRot * camViewPitchOnly );
            }
            else{
                devCam->camRotPre = (*g_cameraIt)->camRot;
            }

            if( devCam->mouse_scroll_clicked){
                //                devCam->showTargetPos(true);
                double scale = 0.03;
                double horizontalVel = speed_scale * scale * ( (*g_cameraIt)->mouse_x[0] - (*g_cameraIt)->mouse_x[1]);
                double verticalVel = speed_scale * scale * ( (*g_cameraIt)->mouse_y[0] - (*g_cameraIt)->mouse_y[1]);
                if (g_mouse_inverted_y){
                    verticalVel = -verticalVel;
                }
                cVector3d nz(0, 0, 1);

                // Use the look vector to avoid locking view at horizon
                double pitchAngle = cAngle(nz, devCam->getLookVector()) - (C_PI/2);

                // Clamp the +ve vertical arc ball to 1.5 Radians
                if (pitchAngle >= 1.5 && verticalVel > 0.0){
                    verticalVel = 0.0;
                }
                // Clamp the -ve vertical arc ball to -1.5 Radians
                if (pitchAngle <= -1.5 && verticalVel < 0.0){
                    verticalVel = 0.0;
                }

                cVector3d deltaVel(0, -horizontalVel, verticalVel);

                cVector3d newPos = devCam->getLocalPos() + devCam->getLocalRot() * deltaVel;
                devCam->setView(newPos, devCam->getTargetPosLocal(), cVector3d(0,0,1));
            }
            //            else{
            //                devCam->showTargetPos(false);
            //            }

        }
    }
}


///
/// \brief mouseScrollCallback
/// \param a_window
/// \param a_xpos
/// \param a_ypos
///
void mouseScrollCallback(GLFWwindow *a_window, double a_xpos, double a_ypos){
    for (g_cameraIt = g_cameras.begin() ; g_cameraIt != g_cameras.end() ; ++g_cameraIt){
        if (a_window == (*g_cameraIt)->m_window){
            int state = glfwGetKey(a_window, GLFW_KEY_LEFT_SHIFT);
            double speed_scale = 1.0;
            if (state == GLFW_PRESS)
            {
                speed_scale = 0.1;
            }
            afCameraPtr cameraPtr = (*g_cameraIt);
            (*g_cameraIt)->mouse_scroll[1] = (*g_cameraIt)->mouse_scroll[0];
            (*g_cameraIt)->mouse_scroll[0] = -a_ypos;

            double scale = 0.1;
            cVector3d camVelAlongLook(speed_scale * scale * (*g_cameraIt)->mouse_scroll[0], 0, 0);
            cVector3d newTargetPos = cameraPtr->getTargetPosLocal();
            cVector3d newPos = cameraPtr->getLocalTransform() * camVelAlongLook;
            cVector3d dPos = newPos - newTargetPos;
            if(dPos.length() < 0.5){
                newTargetPos = newTargetPos + cameraPtr->getLocalRot() * camVelAlongLook;
            }

            if (cameraPtr->isOrthographic()){
                cameraPtr->getInternalCamera()->setOrthographicView(cameraPtr->getInternalCamera()->getOrthographicViewWidth() + (speed_scale * scale * (*g_cameraIt)->mouse_scroll[0]));
                cameraPtr->setLocalPos( cameraPtr->getLocalTransform() * camVelAlongLook );
            }
            else{
                cameraPtr->setLocalPos( cameraPtr->getLocalTransform() * camVelAlongLook );
            }
            cameraPtr->setTargetPos(newTargetPos);
        }
    }
}

// The following functions have been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief getRayTo
/// \param x
/// \param y
/// \param a_cameraPtr
/// \return
///
cVector3d getRayTo(int x, int y, afCameraPtr a_cameraPtr)
{
    float top = 1.f;
    float bottom = -1.0f;
    float nearPlane = 1.f;
    float tanFov = (top - bottom) * 0.5f / nearPlane;
    float fov = (a_cameraPtr->getFieldViewAngle() * 2 / 1.57079) * btAtan(tanFov);

    cVector3d rayFrom = a_cameraPtr->getGlobalTransform().getLocalPos();
    cVector3d rayForward = a_cameraPtr->getTargetPosGlobal() - a_cameraPtr->getGlobalTransform().getLocalPos();
    rayForward.normalize();
    float farPlane = 10000.f;
    rayForward *= farPlane;

    cVector3d vertical = a_cameraPtr->getUpVector();
    if (a_cameraPtr->getParentObject()){
        vertical = a_cameraPtr->getParentObject()->getGlobalTransform().getLocalRot() * vertical;
    }

    cVector3d hor;
    hor = cCross(rayForward, vertical);
    hor.normalize();
    vertical = cCross(hor, rayForward);
    vertical.normalize();

    float tanfov = tanf(0.5f * fov);

    hor *= 2.f * farPlane * tanfov;
    vertical *= 2.f * farPlane * tanfov;

    double aspect;
    float width = float(a_cameraPtr->m_width);
    float height = float(a_cameraPtr->m_height);

    aspect = width / height;

    hor *= aspect;

    cVector3d rayToCenter = rayFrom + rayForward;
    cVector3d dHor = hor * 1.f / width;
    cVector3d dVert = vertical * 1.f / height;

    cVector3d rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
    rayTo += double(g_winWidthRatio*x) * dHor;
    rayTo -= double(g_winHeightRatio*y) * dVert;

    return rayTo;
}


///
/// \brief preTickCallBack: This function is there to account for all the
/// desired features of AMBF that we need from Bullet but are either not
/// implemented or difficult to wrap around from AMBF
/// \param world
/// \param timeStep
///
void preTickCallBack(btDynamicsWorld *world, btScalar timeStep){
    // Check if a softbody has been picked
    if (g_afWorld->m_pickedSoftBody){
        if (g_afWorld->m_pickedNode->m_im == 0.0){
            g_afWorld->m_pickedNode->m_x << g_afWorld->m_pickedNodeGoal;
        }
        else{
            btVector3 delta = to_btVector(g_afWorld->m_pickedNodeGoal) - g_afWorld->m_pickedNode->m_x;
            static const double maxdrag = 10;
            if (delta.length() > (maxdrag * maxdrag))
            {
                delta.normalize();
                delta = delta * maxdrag;
            }
            g_afWorld->m_pickedNode->m_v += delta / timeStep;
        }
    }

//    afJointMap::const_iterator it;
//    for (it = g_afWorld->getAFJointMap()->begin() ; it != g_afWorld->getAFJointMap()->end() ; ++it){
//        afJointPtr jnt = (it->second);
//        jnt->applyDamping(timeStep);
//    }

//    std::vector<afSensorPtr> afSensors = g_afWorld->getAFSensors();
//    // Update the data for sensors
//    for (int i=0 ; i < afSensors.size() ; i++){
//        afSensors[i]->updateSensor();
//    }
}

///
/// \brief close
///
void close(void)
{
    // stop the simulation
    g_simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!g_simulationFinished) { cSleepMs(100); }
    g_physicsThread->stop();
    delete g_physicsThread;

    for(int i = 0 ; i < g_inputDevices->m_numDevices ; i ++){
        g_hapticsThreads[i]->stop();
    }

    // delete resources
    g_inputDevices->closeDevices();
    for(int i = 0 ; i < g_inputDevices->m_numDevices ; i ++){
        delete g_hapticsThreads[i];
    }
    delete g_afWorld;

    delete g_adfLoaderPtr;
}
