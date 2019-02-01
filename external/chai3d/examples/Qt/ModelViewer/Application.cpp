//==============================================================================
/*
    \author    Sebastien Grange
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "Application.h"
//------------------------------------------------------------------------------
#include <QFile>
#include <QString>
#include <QMessageBox>
//------------------------------------------------------------------------------
using namespace std;
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
    m_timer = new QBasicTimer;
    m_mouseMoveCamera = false;
    m_toolRadius = 0.02;
    m_object = NULL;

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
    cHapticDeviceHandler handler;

    // get access to the first available haptic device found
    handler.getDevice (m_device, 0);

    // create a tool (cursor) and insert into the world
    m_tool = new cToolCursor (m_world);
    m_world->addChild (m_tool);

    // connect the haptic device to the virtual tool
    m_tool->setHapticDevice(m_device);

    // define the radius of the tool (sphere)
    m_tool->setRadius(m_toolRadius);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    m_tool->setWorkspaceRadius(1.0);

    // start the haptic tool
    m_tool->start();

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = m_device->getSpecifications();

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = m_tool->getWorkspaceScaleFactor();

    // stiffness properties
    m_maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // add empty object for now
    m_mesh = new cMultiMesh();
    m_point = new cMultiPoint();
    m_world->addChild(m_mesh);
    m_world->addChild(m_point);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a background
    cBackground* background = new cBackground();
    m_camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(0.85f, 0.85f, 0.85f),
                                cColorf(0.85f, 0.85f, 0.85f));
};

//------------------------------------------------------------------------------

ApplicationWidget::~ApplicationWidget ()
{
    stop();
    delete m_world;
    delete m_timer;
}

//------------------------------------------------------------------------------

bool
ApplicationWidget::loadModel (string filename)
{
    // remove object from the world, so we can now safely modify it
    m_worldLock.acquire();
    m_modelLock.acquire();
    m_world->removeChild(m_mesh);
    m_world->removeChild(m_point);
    m_modelLock.release();
    m_worldLock.release();

    // reset object
    m_mesh->clear();
    m_point->clear();

    // reset camera and tool
    m_camera->setSphericalDeg(4.0,    // spherical coordinate radius
                              0,      // spherical coordinate azimuth angle
                              0);     // spherical coordinate polar angle
    m_tool->setLocalRot(m_camera->getLocalRot());

    // try and load as 3D model
    if (m_mesh->loadFromFile(filename) && m_mesh->getNumTriangles() > 0)
    {
        // disable culling so that faces are rendered on both sides
        m_mesh->setUseCulling(false);

        // get dimensions of object
        m_mesh->computeBoundaryBox(true);
        double size = cSub(m_mesh->getBoundaryMax(), m_mesh->getBoundaryMin()).length();

        // resize object to screen
        m_mesh->scale(2.0 / size);

        // since we scaled, we need to recompute the boundary box
        m_mesh->computeBoundaryBox(true);

        // compute collision detection algorithm
        m_mesh->createAABBCollisionDetector(m_toolRadius);

        // define a default stiffness for the object
        m_mesh->setStiffness(0.2 * m_maxStiffness, true);

        // define some haptic friction properties
        m_mesh->setFriction(0.1, 0.2, true);

        // enable display list for faster graphic rendering
        m_mesh->setUseDisplayList(true);

        // compute all edges of object for which adjacent triangles have more than 15 degree angle
        m_mesh->computeAllEdges(15.0);

        // set line width of edges and color
        cColorf colorEdges;
        colorEdges.setBlack();
        m_mesh->setEdgeProperties(1, colorEdges);

        // set normal properties for display
        cColorf colorNormals;
        colorNormals.setOrangeTomato();
        m_mesh->setNormalsProperties(0.01, colorNormals);

        // set view properties
        m_mesh->setShowTriangles(true);
        m_mesh->setShowEdges(false);
        m_mesh->setShowBoundaryBox(false);
        m_mesh->setShowFrame(false);

        // add object to world and center in scene
        m_worldLock.acquire();
        m_modelLock.acquire();
        m_world->addChild(m_mesh);
        m_mesh->setLocalPos(-1.0 * m_mesh->getBoundaryCenter());
        m_object = m_mesh;
        m_modelLock.release();
        m_worldLock.release();

        // start haptic device
        m_tool->start();

        return (true);
    }

    // try and load as point cloud
    if (m_point->loadFromFile(filename))
    {
         // get dimensions of object
        m_point->computeBoundaryBox(true);
        double size = cSub(m_point->getBoundaryMax(), m_point->getBoundaryMin()).length();

        // resize object to screen
        m_point->scale(2.0 / size);

        // since we scaled, we need to recompute the boundary box
        m_point->computeBoundaryBox(true);

        // compute collision detection algorithm
        m_point->createAABBCollisionDetector(m_toolRadius);

        // define a default stiffness for the object
        m_point->setStiffness(0.2 * m_maxStiffness, true);

        // enable display list for faster graphic rendering
        m_point->setUseDisplayList(true);

        // set point size for graphic rendering
        m_point->setPointSize(5.0);

        // add object to world and center in scene
        m_worldLock.acquire();
        m_modelLock.acquire();
        m_world->addChild(m_point);
        m_point->setLocalPos(-1.0 * m_point->getBoundaryCenter());
        m_object = m_point;
        m_modelLock.release();
        m_worldLock.release();

        // start haptic device
        m_tool->start();

        return (true);
    }

    // failure
    QMessageBox::warning(this, "ModelViewer", QString("Failed to load model %1").arg(filename.c_str()), QMessageBox::Ok);
    return false;
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
        // update position and orientation of tool
        m_tool->updateFromDevice();

        // make sure the model does not change during rendering
        m_modelLock.acquire();

        // compute global reference frames for each object
        m_world->computeGlobalPositions(true);

        // compute interaction forces
        m_tool->computeInteractionForces();

        // it is now safe to modify the model
        m_modelLock.release();

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

    // enable anti-aliasing
    QGLWidget::setFormat(QGLFormat(QGL::SampleBuffers));
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
    m_timer->start(25, this);

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

    // stop haptics
    m_tool->stop();

    // stop graphics
    m_timer->stop();

    return 0;
}

//------------------------------------------------------------------------------

void ApplicationWidget::wheelEvent (QWheelEvent *event)
{
    double radius = m_camera->getSphericalRadius() + (double)(event->delta())*5e-4;
    m_camera->setSphericalRadius(radius);
}

//------------------------------------------------------------------------------

void ApplicationWidget::mousePressEvent(QMouseEvent *event)
{
    m_mouseX = event->pos().x();
    m_mouseY = event->pos().y();
    m_mouseMoveCamera = true;
}

//------------------------------------------------------------------------------

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
        double polarDeg = m_camera->getSphericalPolarDeg() -0.5 * dy;
        double azimuthDeg = m_camera->getSphericalAzimuthDeg() - 0.5 * dx;

        // assign new angles
        m_camera->setSphericalPolarDeg(cClamp(polarDeg, 1.0, 179.0));
        m_camera->setSphericalAzimuthDeg(azimuthDeg);

        // line up tool with camera
        m_tool->setLocalRot(m_camera->getLocalRot());
    }
}

//------------------------------------------------------------------------------

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
