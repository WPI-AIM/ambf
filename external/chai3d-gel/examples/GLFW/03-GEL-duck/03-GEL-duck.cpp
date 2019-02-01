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
    \author    Sonny Chan
    \version   3.2.0 $Rev: 1907 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"
#include "tetgen.h"
#include <set>
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

// TetGen switches
char TETGEN_SWITCHES[]    = "pq1.414a0.002";


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

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

// ground level height
double groundLevel = -0.4;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// GEL 3D
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// water
cGELMesh* ground;

// object mesh
cGELMesh* defObject;

// haptic device  model
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

// create a filling sphere skeleton model for duck 2
bool createSkeletonMesh(cGELMesh *a_object, char *a_filename, char *a_filenameHighRes);


//===========================================================================
/*
    DEMO:    GEM_duck.cpp

    This application illustrates the use of the GEM libraries to simulate
    deformable objects. 

    In this example we load our polygonal surface model, use the TetGen
    implementations of the tetrahedralization algorithms to compute a solid
    mesh, and build our simulation data structures from the tetrahedralized
    output. In addition, TetGen gives us the capability to control both the
    maximum circumradius to shortest edge ratio and the maximum volume
    of the generated tetrahedra, which is very useful for controlling the
    number of elements created for any given input mesh.

    TetGen (http://tetgen.berlios.de/) is a quality tetrahedral mesh generator
    and threedimensional Delaunay triangulator created by Hang Si at the
    Weierstrass Institute for Applied Analysis and Stochastics. It provides
    the ability to compute an exact 3D Delaunay tetrahedralization and its dual
    Voronoi diagram from a set of 3D points. More importantly, it also has the
    capability to compute both a constrained and a conforming Delaunay
    tetrahedralization from a given piecewise linear complex (of which a
    polygonal surface is a special case).

    Haptic feedback is performed by computing at every integration step the
    interaction between the cursor (large sphere) with each mass-point or
    skeleton node composing the physical models of each respective body.
    The approach is not optimal for large models and will need to be improved
    in the future. However, performing point location on deformable bodies
    is a non trivial problem since the collision structure needs to be
    continuously updated as the body deforms.
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
    cout << "Demo: 52-GEL-duck" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[s] - Show/Hide GEL Skeleton" << endl;
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

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(1.0, 1.0, 1.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(1.5, 0.0, 1.6),    // camera position (eye)
                cVector3d(0.5, 0.0, 0.0),    // lookat position (target)
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

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define direction of light beam
    light->setDir(-1.0,-1.0,-1.0); 


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
    cursorWorkspaceRadius = 0.8;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 0.05 * hapticDeviceInfo.m_maxLinearForce;

    // create a large sphere that represents the haptic device
    deviceRadius = 0.12;
    device = new cShapeSphere(deviceRadius);
    world->addChild(device);
    device->m_material->setWhite();
    device->m_material->setShininess(100);

    // interaction stiffness between tool and deformable model 
    stiffness = 10;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);


    /////////////////////////////////////////////////////////////////////////
    // COMPOSE WATER BED
    /////////////////////////////////////////////////////////////////////////

    // create GEL mesh object
    ground = new cGELMesh();

    // add mesh to dynamic world
    defWorld->m_gelMeshes.push_back(ground);

    // enable particle model (mesh vertices)
    ground->m_useMassParticleModel = true;

    // set default physical properties for each mass node
    cGELMassParticle::s_default_mass = 0.010;
    cGELMassParticle::s_default_kDampingPos = 0.4;
    cGELMassParticle::s_default_gravity.set(0,0,0);

    // create mesh object
    cMesh* mesh = ground->newMesh();

    // create a array of polygons that simulate water
    int RESOLUTION = 15;
    double SIZE = 5.0;
    for (int v=0; v<RESOLUTION; v++)
    {
        for (int u=0; u<RESOLUTION; u++)
        {
            double px, py, tu, tv;

            // compute the position of the vertex
            px = SIZE / (double)RESOLUTION * (double)u - (SIZE/2.0);
            py = SIZE / (double)RESOLUTION * (double)v - (SIZE/2.0);

            // create new vertex
            unsigned int index = mesh->newVertex(px, py, groundLevel);

            // compute texture coordinate
            tu = (double)u / (double)RESOLUTION;
            tv = (double)v / (double)RESOLUTION;
            mesh->m_vertices->setTexCoord(index, tu, tv);
            mesh->m_vertices->setColor(index, cColorf(1.0, 0.0, 0.1));
        }
    }
    
    // build particles for each vertex
    ground->buildVertices();

    // set all particle at edge of map as fixed
    for (int v=0; v<RESOLUTION; v++)
    {
        for (int u=0; u<RESOLUTION; u++)
        {
            if ((u == 0) || (v == 0) || (u == (RESOLUTION-1)) || (v == (RESOLUTION-1)))
            {
                unsigned int index = ((v + 0) * RESOLUTION) + (u + 0);
                ground->m_gelVertices[index].m_massParticle->m_fixed = true;
            }
        }
    }

    // set default physical properties for spring
    cGELLinearSpring::s_default_kSpringElongation = 10.0; // [N/m]
    
    // create springs between particles
    for (int v=0; v<(RESOLUTION-1); v++)
    {
        for (int u=0; u<(RESOLUTION-1); u++)
        {
            // get the indexing numbers of the next four vertices
            unsigned int index00 = ((v + 0) * RESOLUTION) + (u + 0);
            unsigned int index01 = ((v + 0) * RESOLUTION) + (u + 1);
            unsigned int index10 = ((v + 1) * RESOLUTION) + (u + 0);
            unsigned int index11 = ((v + 1) * RESOLUTION) + (u + 1);

            // create two new triangles
            mesh->newTriangle(index00, index01, index10);
            mesh->newTriangle(index10, index01, index11);

            cGELMassParticle* m0 = ground->m_gelVertices[index00].m_massParticle;
            cGELMassParticle* m1 = ground->m_gelVertices[index01].m_massParticle;
            cGELMassParticle* m2 = ground->m_gelVertices[index10].m_massParticle;
            cGELMassParticle* m3 = ground->m_gelVertices[index11].m_massParticle;

            cGELLinearSpring* spring0 = new cGELLinearSpring(m0, m1);
            cGELLinearSpring* spring1 = new cGELLinearSpring(m0, m2);
            ground->m_linearSprings.push_back(spring0);
            ground->m_linearSprings.push_back(spring1);

            if ((u == (RESOLUTION-2)) || (v == (RESOLUTION-2)))
            {
                cGELLinearSpring* spring2 = new cGELLinearSpring(m3, m1);
                cGELLinearSpring* spring3 = new cGELLinearSpring(m3, m2);
                ground->m_linearSprings.push_back(spring2);
                ground->m_linearSprings.push_back(spring3);
            }
        }
    }

    // ajust material and transparency
    ground->setUseMaterial(true);
    ground->m_material->setGrayLevel(0.1);
    ground->setTransparencyLevel(0.7);
    ground->setUseTransparency(true);

    // create water texture
    shared_ptr<cTexture2d> textureGround(new cTexture2d());
    ground->setTexture(textureGround);
    ground->setUseTexture(true, true);
    
    // load water texture
    bool fileload;
    fileload = textureGround->loadFromFile(RESOURCE_PATH("../resources/images/water.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = textureGround->loadFromFile("../../../bin/resources/images/water.jpg" );
        #endif
        if (!fileload)
        {
            cout << "Error - 3D Model failed to load correctly." << endl;
            close();
            return (-1);
        }
    }

    // enable environmental texturing
    textureGround->setEnvironmentMode(GL_DECAL);
    textureGround->setSphericalMappingEnabled(true);
    

    /////////////////////////////////////////////////////////////////////////
    // CREATE DUCK
    /////////////////////////////////////////////////////////////////////////

    // create a deformable mesh
    defObject = new cGELMesh();

    // add deformable mesh to dynamic world
    defWorld->m_gelMeshes.push_back(defObject);
    
    // create a skeleton composed of mass particles
    fileload = createSkeletonMesh(defObject, RESOURCE_PATH("../resources/models/ducky/duck-200.off"), RESOURCE_PATH("../resources/models/ducky/duck-full.obj"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = createSkeletonMesh(defObject, "../../../bin/resources/models/ducky/duck-200.off", "../../../bin/resources/models/ducky/duck-full.obj");
#endif
        if (!fileload)
        {
            cout << "Error - 3D Model failed to load correctly." << endl;
            close();
            return (-1);
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
    labelRates->m_fontColor.setWhite();

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    fileload = background->loadFromFile(RESOURCE_PATH("../resources/images/stone.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = background->loadFromFile("../../../bin/resources/images/stone.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Image failed to load correctly." << endl;
        close();
        return (-1);
    }
    
    // set aspect ration of background image a constant
    background->setFixedAspectRatio(true);


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

bool createSkeletonMesh(cGELMesh *a_object, char *a_filename, char *a_filenameHighRes)
{
    a_object->m_useSkeletonModel = true;
    a_object->m_useMassParticleModel = false;
    a_object->loadFromFile(a_filenameHighRes);

    cGELMesh* model = new cGELMesh();
    cMesh* mesh = model->newMesh();

    tetgenio input;
    if (input.load_off(a_filename))
    {
        // use TetGen to tetrahedralize our mesh
        tetgenio output;
        tetrahedralize(TETGEN_SWITCHES, &input, &output);

        // create a vertex in the object for each point of the result
        for (int p = 0, pi = 0; p < output.numberofpoints; ++p, pi += 3)
        {
            cVector3d point;
            point.x(output.pointlist[pi+0]);
            point.y(output.pointlist[pi+1]);
            point.z(output.pointlist[pi+2]);
            mesh->newVertex(point);
        }

        // create a triangle for each face on the surface
        for (int t = 0, ti = 0; t < output.numberoftrifaces; ++t, ti += 3)
        {
            cVector3d p[3];
            unsigned int vi[3];
            for (int i = 0; i < 3; ++i)
            {
                int tc = output.trifacelist[ti+i];
                vi[i] = tc;
                int pi = tc*3;
                p[i].x(output.pointlist[pi+0]);
                p[i].y(output.pointlist[pi+1]);
                p[i].z(output.pointlist[pi+2]);
            }
            //unsigned int index = a_object->newTriangle(p[1], p[0], p[2]);
            mesh->newTriangle(vi[1], vi[0], vi[2]);
        }

        // find out exactly which vertices are on the inside and outside
        set<int> inside, outside;
        for (int t = 0; t < output.numberoftrifaces * 3; ++t)
        {
            outside.insert(output.trifacelist[t]);
        }
        for (int p = 0; p < output.numberofpoints; ++p)
        {
            if (outside.find(p) == outside.end())
                inside.insert(p);
        }

        model->computeAllNormals();

        // compute a boundary box
        model->computeBoundaryBox(true);

        // get dimensions of object
        double size = cSub(model->getBoundaryMax(), model->getBoundaryMin()).length();

        // resize object to screen
        if (size > 0)
        {
            model->scale( 1.5 / size);
            a_object->scale( 1.5 / size);
        }

        // setup default values for nodes
        cGELSkeletonNode::s_default_radius        = 0.05;
        cGELSkeletonNode::s_default_kDampingPos   = 0.3;
        cGELSkeletonNode::s_default_kDampingRot   = 0.1;
        cGELSkeletonNode::s_default_mass          = 0.002;  // [kg]
        cGELSkeletonNode::s_default_showFrame     = false;
        cGELSkeletonNode::s_default_color.set(1.0, 0.6, 0.6);
        cGELSkeletonNode::s_default_useGravity    = true;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00, -3.45);
        radius = cGELSkeletonNode::s_default_radius;

        a_object->buildVertices();
        model->buildVertices();

        vector<cGELSkeletonNode*> nodes;
        int i=0;
        for (set<int>::iterator it = inside.begin(); it != inside.end(); ++it)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            a_object->m_nodes.push_front(newNode);

            unsigned int vertexIndex = 0;
            cMesh* mesh = NULL;
            
            if (model->getVertex(*it, mesh, vertexIndex))
            {
                newNode->m_pos = mesh->m_vertices->getLocalPos(vertexIndex);
                newNode->m_rot.identity();
                newNode->m_radius = 0.1;
                newNode->m_fixed = false;
                mesh->m_vertices->setUserData(vertexIndex, i);
                i++;
                nodes.push_back(newNode);
            }
        }

        // get all the edges of our tetrahedra
        set< pair<int,int> > springs;
        for (int t = 0, ti = 0; t < output.numberoftetrahedra; ++t, ti += 4)
        {
            // store each edge of the tetrahedron as a pair of indices
            for (int i = 0; i < 4; ++i) {
                int v0 = output.tetrahedronlist[ti+i];
                for (int j = i+1; j < 4; ++j) {
                    int v1 = output.tetrahedronlist[ti+j];

                    // connect only if both points are inside
                    if (inside.find(v0) != inside.end() && inside.find(v1) != inside.end())
                        springs.insert(pair<int,int>(min(v0,v1), max(v0,v1)));
                }
            }
        }

        // setup default values for links
        cGELSkeletonLink::s_default_kSpringElongation = 100.0; // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.set(0.2, 0.2, 1.0);

        for (set< pair<int,int> >::iterator it = springs.begin(); it != springs.end(); ++it)
        {
            unsigned int vertexIndex0 = 0;
            unsigned int vertexIndex1 = 0;
            cMesh* mesh0 = NULL;
            cMesh* mesh1 = NULL;

            model->getVertex(it->first, mesh0, vertexIndex0);
            model->getVertex(it->second, mesh1, vertexIndex1);
            cGELSkeletonNode* n0 = nodes[mesh0->m_vertices->getUserData(vertexIndex0)];
            cGELSkeletonNode* n1 = nodes[mesh1->m_vertices->getUserData(vertexIndex1)];
            cGELSkeletonLink* newLink = new cGELSkeletonLink(n0, n1);
            a_object->m_links.push_front(newLink);
        }

        a_object->connectVerticesToSkeleton(false);

        cMaterial mat;
        mat.m_ambient.set(0.7, 0.7, 0.7);
        mat.m_diffuse.set(0.8, 0.8, 0.8);
        mat.m_specular.set(0.0, 0.0, 0.0);
        a_object->setMaterial(mat, true);

        // cleanup
        delete model;

        return (true);
    }
    return (false);
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
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

    // option - show/hide skeleton
    else if (a_key == GLFW_KEY_S)
    {
        defObject->m_showSkeletonModel = !defObject->m_showSkeletonModel;

        if (defObject->m_showSkeletonModel)
        {
            defObject->setWireMode(true, true);
        }
        else
        {
            defObject->setWireMode(false, true);
        }
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

    // simulation time
    double time = 0.0;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;


    // main haptic simulation loop
    while(simulationRunning)
    {
        // stop clock
        double interval = 0.001; // cMin(0.001, clock.stop());

        // restart clock
        clock.start(true);

        // update simulation time
        time = time + interval;

        // read position from haptic device
        cVector3d pos;
        hapticDevice->getPosition(pos);
        pos.mul(workspaceScaleFactor);
        device->setLocalPos(pos);

        // compute reaction forces
        cVector3d force(0.0, 0.0, 0.0);

        // compute reaction forces
        list<cGELMesh*>::iterator i;

        // model water groundLevel
        for(i = defWorld->m_gelMeshes.begin(); i != defWorld->m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;

            if (nextItem->m_useMassParticleModel)
            {
                int numVertices = (int)(nextItem->m_gelVertices.size());
                for (int i=0; i<numVertices; i++)
                {
                   cVector3d nodePos = nextItem->m_gelVertices[i].m_massParticle->m_pos;

                   double forceWave = 0.001 * sin(1.0 * (time + nodePos.x() + nodePos.y()));

                   cVector3d force = cVector3d(-0.002 * nodePos.x(), -0.002 * nodePos.y(), forceWave);
                   if (nodePos.z() < groundLevel)
                   {
                        double depth = nodePos.z() - groundLevel;
                        force.add(cVector3d(0,0,-100*depth));
                   }
                   nextItem->m_gelVertices[i].m_massParticle->setExternalForce(force);
                }
            }

            if (nextItem->m_useSkeletonModel)
            {
                list<cGELSkeletonNode*>::iterator i;
                for(i = nextItem->m_nodes.begin(); i != nextItem->m_nodes.end(); ++i)
                {
                    cGELSkeletonNode* node = *i;
                    cVector3d nodePos = node->m_pos;
                    double radius = node->m_radius;
                    cVector3d force = cVector3d(-0.01 * nodePos.x(), -0.01 * (nodePos.y()), 0.0);

                    if ((nodePos.z()-radius) < groundLevel)
                    {
                        double depth = (nodePos.z()-radius) - groundLevel;
                        force.add(cVector3d(0,0,-1.0 * depth));
                        node->m_vel.mul(0.95);
                    }

                    double forceWave = 0.001 * sin(time);
                    force.add(0.0, forceWave, 0.0);

                    node->setExternalForce(force);
                }
            }
        }

        // compute haptic feedback
        for(i = defWorld->m_gelMeshes.begin(); i != defWorld->m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;

            if (nextItem->m_useMassParticleModel)
            {
                int numVertices = (int)(nextItem->m_gelVertices.size());
                for (int i=0; i<numVertices; i++)
                {
                cVector3d nodePos = nextItem->m_gelVertices[i].m_massParticle->m_pos;
                cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness);
                if (f.lengthsq() > 0)
                {
                    cVector3d tmpfrc = cNegate(f);
                    nextItem->m_gelVertices[i].m_massParticle->setExternalForce(tmpfrc);
                }
                force.add(cMul(1.0, f));
                }
            }

            if (nextItem->m_useSkeletonModel)
            {
                list<cGELSkeletonNode*>::iterator i;
                for(i = nextItem->m_nodes.begin(); i != nextItem->m_nodes.end(); ++i)
                {
                    cGELSkeletonNode* node = *i;
                    cVector3d nodePos = node->m_pos;
                    double radius = node->m_radius;
                    cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness);
                    if (f.lengthsq() > 0)
                    {
                        cVector3d tmpfrc = cNegate(f);
                        node->setExternalForce(tmpfrc);
                    }
                    force.add(cMul(4.0, f));
                }
            }
        }
        
        // integrate dynamics
        defWorld->updateDynamics(interval);

        // scale force
        force.mul(deviceForceScale);

        // water viscosity
        if ((pos.z() - deviceRadius) < groundLevel)
        {
            // read damping properties of haptic device
            cHapticDeviceInfo info = hapticDevice->getSpecifications();
            double Kv = 0.8 * info.m_maxLinearDamping;

            // read device velocity
            cVector3d linearVelocity;
            hapticDevice->getLinearVelocity(linearVelocity);

            // compute a scale factor [0,1] proportional to percentage
            // of tool volume immersed in the water
            double val = (groundLevel - (pos.z() - deviceRadius)) / (2.0 * deviceRadius);
            double scale = cClamp(val, 0.1, 1.0);

            // compute force
            cVector3d forceDamping = cMul(-Kv * scale, linearVelocity);
            force.add(forceDamping);
        }

        // send forces to haptic device
        hapticDevice->setForce(force);

        // update frequency counter
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

    // compute the reaction forces between the tool and the ith sphere.
    cVector3d force;
    force.zero();
    cVector3d vSphereCursor = a_cursor - a_spherePos;

    // check if both objects are intersecting
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

    // return result
    return (force);
}
