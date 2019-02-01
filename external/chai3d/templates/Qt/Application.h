//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================


//------------------------------------------------------------------------------
#ifndef APPLICATION_H
#define APPLICATION_H
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#pragma warning(disable: 4100)
#endif
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include "Interface.h"
//------------------------------------------------------------------------------
#include <QWheelEvent>
//------------------------------------------------------------------------------

void _hapticThread (void *arg);

//------------------------------------------------------------------------------

class ApplicationWidget : public QGLWidget
{

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    ApplicationWidget (QWidget *parent);
    virtual ~ApplicationWidget ();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    int start();
    int stop();
    void waitForStop();
    void* hapticThread();
    bool isRunning() { return m_running; }

    double getGraphicRate() { return (m_graphicRate.getFrequency()); }
    double getHapticRate() { return  (m_hapticRate.getFrequency()); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    void initializeGL();
    void resizeGL(int a_width, int a_height);
    void paintGL();
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void timerEvent(QTimerEvent *event) { updateGL(); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    // application control
    Interface* m_parent;
    chai3d::cMutex m_worldLock;
    chai3d::cMutex m_runLock;
    chai3d::cThread m_thread;
    chai3d::cFrequencyCounter m_graphicRate;
    chai3d::cFrequencyCounter m_hapticRate;

    int m_timerID;
    bool m_running;
    int m_width;
    int m_height;
    int m_mouseX;
    int m_mouseY;
    bool m_mouseMoveCamera;

    // CHAI3D world
    chai3d::cGenericHapticDevicePtr m_device;
    chai3d::cWorld* m_world;
    chai3d::cCamera* m_camera;
    chai3d::cDirectionalLight* m_light;
    chai3d::cToolCursor* m_tool;
    chai3d::cMultiMesh* m_object;
};

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
