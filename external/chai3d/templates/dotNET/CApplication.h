//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================


//------------------------------------------------------------------------------
#ifndef CAPPLICATION_H
#define CAPPLICATION_H
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#pragma warning(disable: 4100)
#endif
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------

void _hapticThread (void *arg);

//------------------------------------------------------------------------------

class cApplication
{

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    cApplication();
    virtual ~cApplication();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    int start();
    int stop();

    void* hapticThread();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    void resizeGL(int a_width, int a_height);
    void paintGL();


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    // application control
    int m_timerID;
    int m_width;
    int m_height;
    bool m_running;
    bool m_finished;

    // CHAI3D world
    chai3d::cGenericHapticDevicePtr m_device;
    chai3d::cWorld* m_world;
    chai3d::cCamera* m_camera;
    chai3d::cDirectionalLight* m_light;
    chai3d::cToolCursor* m_tool;
    chai3d::cMultiMesh* m_object;
    chai3d::cFrequencyCounter m_graphicRate;
    chai3d::cFrequencyCounter m_hapticRate;
    chai3d::cThread m_thread;
};

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------