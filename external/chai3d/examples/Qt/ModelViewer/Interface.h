//==============================================================================
/*
    \author    Sebastien Grange
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
#include <QFileSystemModel>
#include <QGLWidget>
#include <QLabel>
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
    QFileSystemModel *dirModel;


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //--------------------------------------------------------------------------

private:

    int AbortRequest;


    //--------------------------------------------------------------------------
    // PRIVATE SLOTS:
    //--------------------------------------------------------------------------

private slots:

    void on_showWireframe_stateChanged(int val);
    void on_showEdges_stateChanged(int val);
    void on_showNormals_stateChanged(int val);
    void on_showBoundaryBox_stateChanged(int val);
    void on_showTriangles_stateChanged(int val);
    void on_showFrame_stateChanged(int val);
    void on_folderList_doubleClicked(const QModelIndex &index);
    void on_folderList_clicked(const QModelIndex &index);

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
};


//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
