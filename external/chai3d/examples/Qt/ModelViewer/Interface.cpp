//==============================================================================
/*
    \author    Sebastien Grange
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "Interface.h"
#include "Application.h"
//------------------------------------------------------------------------------
using std::string;
//------------------------------------------------------------------------------

Interface::Interface (QWidget        *parent,
                      Qt::WindowFlags flags) : QMainWindow (parent, flags)
{
    // setup ui
    ui.setupUi(this);

    // setup display keyboard shortcuts
    EscKey   = new QShortcut (Qt::Key_Escape, this, SLOT(ExitFullScreen()));
    FKey     = new QShortcut (Qt::Key_F,      this, SLOT(ToggleFullScreen()));
    SKey     = new QShortcut (Qt::Key_S,      this, SLOT(ToggleSettings()));
    QKey     = new QShortcut (Qt::Key_Q,      this, SLOT(close()));

    // create CHAI3D application widget
    Application = new ApplicationWidget (this);
    if (Application) 
    {
        Application->setSizePolicy (QSizePolicy::Expanding, QSizePolicy::Expanding);
        centralWidget()->layout()->addWidget (Application);
    }
    else 
    {
        QMessageBox::information (this, "Application", "Cannot start application.", QMessageBox::Ok);
        close();
    }

    // configure timers
    StatusTimer = new QTimer (this);
    connect(StatusTimer, SIGNAL(timeout()), this, SLOT(UpdateStatus()));
    StatusTimer->start (1000);

    // initialize status bar
    GraphicRate.setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ui.statusBar->addPermanentWidget (&GraphicRate);
    HapticRate.setFrameStyle(QFrame::Panel | QFrame::Sunken);
    ui.statusBar->addPermanentWidget (&HapticRate);

    // set folder list
    QStringList filters;
    filters << "*.3ds" << "*.obj" << "*.stl";
    dirModel = new QFileSystemModel(this);
    dirModel->setFilter(QDir::NoDot | QDir::AllDirs | QDir::Files);

    dirModel->setRootPath("/");
    dirModel->setNameFilters(filters);
    dirModel->setNameFilterDisables(false);
    ui.folderList->setModel(dirModel);
    ui.folderList->setRootIndex(dirModel->index(QCoreApplication::applicationDirPath()));

    // show settings by default
    ShowSettings(true);
}

//------------------------------------------------------------------------------

int Interface::Start()
{
    // start haptic thread
    if (Application->start () < 0)
    {
        QMessageBox::warning (this, "CHAI3D", "No device found.", QMessageBox::Ok);
        return (-1);
    }

    return (0);
}

//------------------------------------------------------------------------------

void Interface::Stop()
{
    Application->stop();
}

//------------------------------------------------------------------------------

Interface::~Interface()
{
    Stop();
    delete Application;
}

//------------------------------------------------------------------------------

void Interface::EnterFullScreen()
{
    showFullScreen ();
}

//------------------------------------------------------------------------------

void Interface::ExitFullScreen()
{
    showNormal ();
}

//------------------------------------------------------------------------------

void Interface::ToggleFullScreen()
{
    if (isFullScreen())
    {
        ExitFullScreen();
    }
    else
    {
        EnterFullScreen();
    }
}

//------------------------------------------------------------------------------

void Interface::SetFullScreen(bool fs)
{
    if( fs && !isFullScreen())
    {
        EnterFullScreen();
    }
    else if (!fs &&  isFullScreen())
    {
        ExitFullScreen();
    }
}

//------------------------------------------------------------------------------

void Interface::ToggleSettings()
{
      bool show = !ui.Settings->isVisible();
      ui.Settings->setVisible (show);
}

//------------------------------------------------------------------------------
void Interface::ShowSettings(bool show)
{
      ui.Settings->setVisible (show);
}

//------------------------------------------------------------------------------

void Interface::on_showWireframe_stateChanged(int val)
{
    Application->m_object->setWireMode(val == Qt::Checked);
    if (val == Qt::Checked)
    {
        ui.showTriangles->setChecked(true);
        ui.showTriangles->setEnabled(false);
    }
    else
    {
        ui.showTriangles->setEnabled(true);
    }
}

//------------------------------------------------------------------------------

void Interface::on_showEdges_stateChanged(int val)
{
    Application->m_mesh->setShowEdges(val == Qt::Checked);
}

//------------------------------------------------------------------------------

void Interface::on_showNormals_stateChanged(int val)
{
    Application->m_mesh->setShowNormals(val == Qt::Checked);
}

//------------------------------------------------------------------------------

void Interface::on_showBoundaryBox_stateChanged(int val)
{
    Application->m_object->setShowBoundaryBox(val == Qt::Checked);
}

//------------------------------------------------------------------------------

void Interface::on_showTriangles_stateChanged(int val)
{
    Application->m_mesh->setShowTriangles(val == Qt::Checked);
}

//------------------------------------------------------------------------------

void Interface::on_showFrame_stateChanged(int val)
{
    Application->m_object->setShowFrame(val == Qt::Checked);
}

//------------------------------------------------------------------------------

void Interface::on_folderList_clicked(const QModelIndex &index)
{
    QFileInfo info = dirModel->fileInfo(index);
    if (info.isFile())
    {
        if (Application) Application->loadModel(info.absoluteFilePath().toStdString());

        bool mesh = (Application->m_mesh == Application->m_object);

        ui.showWireframe->setEnabled(mesh);
        ui.showTriangles->setEnabled(mesh);
        ui.showEdges->setEnabled(mesh);
        ui.showNormals->setEnabled(mesh);

        ui.showFrame->setChecked(false);
        ui.showBoundaryBox->setChecked(false);
        ui.showTriangles->setChecked(mesh);
        ui.showWireframe->setChecked(false);
        ui.showEdges->setChecked(false);
        ui.showNormals->setChecked(false);
    }
}

//------------------------------------------------------------------------------

void Interface::on_folderList_doubleClicked(const QModelIndex &index)
{
    QFileInfo info = dirModel->fileInfo(index);
    if (info.isDir())
    {
        ui.folderList->setModel(dirModel);
        ui.folderList->setRootIndex(dirModel->index(info.absoluteFilePath()));
    }
}

//------------------------------------------------------------------------------

void Interface::UpdateStatus()
{
    if (Application)
    {
        GraphicRate.setText(QString("graphic: %1 Hz").arg((int)(Application->getGraphicRate()), 3));
        HapticRate.setText(QString("haptic: %1 Hz").arg((int)(Application->getHapticRate()), 4));
    }
    else 
    {
        GraphicRate.setText(QString("graphic: --- Hz"));
        HapticRate.setText(QString("haptic: ---- Hz"));
    }
}

//------------------------------------------------------------------------------
