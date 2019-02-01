//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef INTERFACE_H
#define INTERFACE_H
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#pragma warning(disable: 4100)
#endif
//------------------------------------------------------------------------------
#include "ui_Interface.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <QGLWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QShortcut>
//------------------------------------------------------------------------------
class ApplicationWidget;
//------------------------------------------------------------------------------

namespace Ui
{
    class InterfaceClass;
}

//------------------------------------------------------------------------------

class Interface : public QMainWindow
{
    Q_OBJECT

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    Interface (QWidget *parent = 0, Qt::WindowFlags flags = 0);
    ~Interface ();


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS - UI:
    //--------------------------------------------------------------------------

private:

    Ui::InterfaceClass ui;
    QShortcut *EscKey;
    QShortcut *FKey;
    QShortcut *SKey;
    QShortcut *QKey;
    QTimer *StatusTimer;
    ApplicationWidget *Application;
    QLabel GraphicRate;
    QLabel HapticRate;


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //--------------------------------------------------------------------------

private:

    int AbortRequest;


    //--------------------------------------------------------------------------
    // PRIVATE SLOTS:
    //--------------------------------------------------------------------------

private slots:

    void on_sliderZoom_valueChanged(int val);
    void EnterFullScreen();
    void ExitFullScreen();
    void ToggleFullScreen();
    void SetFullScreen(bool fs);
    void ToggleSettings();
    void ShowSettings(bool show);
    void UpdateStatus();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    int  Start();
    void Stop();
    void SyncUI();
};


//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------