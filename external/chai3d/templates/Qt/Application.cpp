//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "Application.h"
//------------------------------------------------------------------------------
#include <QFile>
//------------------------------------------------------------------------------
using namespace chai3d;
//------------------------------------------------------------------------------

ApplicationWidget::ApplicationWidget (QWidget *parent)
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    // initialize variables
    m_parent  = (Interface*)(void*)parent;
    m_running = false;
    m_timerID = 0;
    m_mouseMoveCamera = false;

    // reset frequency counters
    m_graphicRate.reset();
    m_hapticRate.reset();


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    m_world = new cWorld();

    // set the background color of the environment
    m_world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    m_camera = new cCamera(m_world);
    m_world->addChild(m_camera);

    // define a basis in spherical coordinates for the camera
    m_camera->setSphericalReferences(cVector3d(0,0,0),    // origin
                                     cVector3d(0,0,1),    // zenith direction
                                     cVector3d(1,0,0));   // azimuth direction

    m_camera->setSphericalDeg(4.0,    // spherical coordinate radius
                              0,      // spherical coordinate azimuth angle
                              0);     // spherical coordinate polar angle

    // set the near and far clipping planes of the camera
    m_camera->setClippingPlanes (0.01, 20.0);

    // create a light source
    m_light = new cDirectionalLight (m_world);

    // add light to camera
    m_camera->addChild(m_light);

    // enable light source
    m_light->setEnabled(true);

    // define the direction of the light beam
    m_light->setDir(-1.0,-1.0,-0.5);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    cHapticDeviceHandler* handler = new cHapticDeviceHandler ();

    // get access to the first available haptic device found
    handler->getDevice (m_device, 0);

    // create a tool (cursor) and insert into the world
    m_tool = new cToolCursor (m_world);
    m_world->addChild (m_tool);

    // connect the haptic device to the virtual tool
    m_tool->setHapticDevice(m_device);

    // define the radius of the tool (sphere)
    double toolRadius = 0.1;
    m_tool->setRadius(toolRadius);
 
    // map the physical workspace of the haptic device to a larger virtual workspace.
    m_tool->setWorkspaceRadius(1.0);

    // start the haptic tool
    m_tool->start();


    //--------------------------------------------------------------------------
    // OBJECTS
    //--------------------------------------------------------------------------

    // create a sphere
    m_object = new cMultiMesh();
    
    // add object to world
    m_world->addChild (m_object);

    // position object
    m_object->setLocalPos(0.0, 0.0, 0.0);
    
    // create a first ring (red)
    cMesh* mesh1 = m_object->newMesh();
    cCreatePipe(mesh1, 0.1, 0.80, 1.00, 72, 1, cVector3d(-0.05, 0.0, 0.0), cMatrix3d(cDegToRad(0), cDegToRad(90), cDegToRad(0), C_EULER_ORDER_XYZ));
    mesh1->m_material->setOrangeRed();
    mesh1->m_material->setStiffness(100.0);

    // create a second ring (green)
    cMesh* mesh2 = m_object->newMesh();
    cCreatePipe(mesh2, 0.1, 0.79, 1.01, 72, 1, cVector3d(0.0,-0.05, 0.0), cMatrix3d(cDegToRad(-90), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ));
    mesh2->m_material->setGreenForest();
    mesh2->m_material->setStiffness(100.0);

    // create a third ring (blue)
    cMesh* mesh3 = m_object->newMesh();
    cCreatePipe(mesh3, 0.1, 0.78, 1.02, 72, 1, cVector3d(0.0, 0.0,-0.05), cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ));
    mesh3->m_material->setBlueRoyal();
    mesh3->m_material->setStiffness(100.0);

    // create collision detector
    m_object->createAABBCollisionDetector(toolRadius);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a background
    cBackground* background = new cBackground();
    m_camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(0.00f, 0.00f, 0.00f),
                                cColorf(0.00f, 0.00f, 0.00f),
                                cColorf(0.44f, 0.44f, 0.88f),
                                cColorf(0.44f, 0.44f, 0.88f));
};

//------------------------------------------------------------------------------

ApplicationWidget::~ApplicationWidget ()
{
    delete m_world;
}

//------------------------------------------------------------------------------

void* ApplicationWidget::hapticThread ()
{
    // acquire run lock
    m_runLock.acquire();

    // update state
    m_running = true;

    while (m_running) 
    {
        // compute global reference frames for each object
        m_world->computeGlobalPositions(true);

        // update position and orientation of tool
        m_tool->updateFromDevice();

        // compute interaction forces
        m_tool->computeInteractionForces();

        // send forces to haptic device
        m_tool->applyToDevice();

        // update frequency counter
        m_hapticRate.signal(1);
    }

    // disable forces
    m_device->setForceAndTorqueAndGripperForce (cVector3d(0.0, 0.0, 0.0),
                                                cVector3d(0.0, 0.0, 0.0),
                                                0.0);

    // update state
    m_running = false;

    // release run lock
    m_runLock.release();

    // exit thread
    return (NULL);
}

//------------------------------------------------------------------------------

void ApplicationWidget::initializeGL ()
{
#ifdef GLEW_VERSION
    glewInit ();
#endif
}

//------------------------------------------------------------------------------

void ApplicationWidget::paintGL ()
{
    if (!m_running) return;

    m_worldLock.acquire();

    // render world
    m_camera->renderView(m_width, m_height);

    // wait until all GL commands are completed
    glFinish();

    m_graphicRate.signal(1);

    m_worldLock.release();
}

//------------------------------------------------------------------------------

void ApplicationWidget::resizeGL (int a_width,  int a_height)
{
    m_worldLock.acquire ();

    m_width = a_width;
    m_height = a_height;

    m_worldLock.release ();
}

//------------------------------------------------------------------------------

int ApplicationWidget::start ()
{
    // start graphic rendering
    m_timerID = startTimer(20);

    // start haptic thread
    m_thread.start (_hapticThread, CTHREAD_PRIORITY_HAPTICS, this);

    return(0);
}

//------------------------------------------------------------------------------

int ApplicationWidget::stop ()
{
    // stop the simulation thread and wait it to join
    m_running = false;
    m_runLock.acquire();
    m_runLock.release();

    m_tool->stop ();

    killTimer (m_timerID);

    return 0;
}

//------------------------------------------------------------------------------

void ApplicationWidget::wheelEvent (QWheelEvent *event)
{
    double radius = m_camera->getSphericalRadius() + (double)(event->delta())*5e-4;
    m_camera->setSphericalRadius(radius);

    // tell GUI widgets to reflect the new simulation parameters
    m_parent->SyncUI ();
}

//---------------------------------------------------------------------------

void ApplicationWidget::mousePressEvent(QMouseEvent *event)
{
    m_mouseX = event->pos().x();
    m_mouseY = event->pos().y();
    m_mouseMoveCamera = true;
}

//---------------------------------------------------------------------------

void ApplicationWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_mouseMoveCamera)
    {
        int x = event->pos().x();
        int y = event->pos().y();

        // compute mouse motion
        int dx = x - m_mouseX;
        int dy = y - m_mouseY;
        m_mouseX = x;
        m_mouseY = y;

        // compute new camera angles
        double azimuthDeg = m_camera->getSphericalAzimuthDeg() + (0.5 * dy);
        double polarDeg = m_camera->getSphericalPolarDeg() + (-0.5 * dx);

        // assign new angles
        m_camera->setSphericalAzimuthDeg(azimuthDeg);
        m_camera->setSphericalPolarDeg(polarDeg);

        // line up tool with camera
        m_tool->setLocalRot(m_camera->getLocalRot());
    }
}

//---------------------------------------------------------------------------

void ApplicationWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_mouseMoveCamera = false;
}

//------------------------------------------------------------------------------

void _hapticThread (void *arg)
{
    ((ApplicationWidget*)arg)->hapticThread();
}

//------------------------------------------------------------------------------
