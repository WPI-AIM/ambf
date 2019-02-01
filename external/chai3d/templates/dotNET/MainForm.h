//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

#pragma once

//------------------------------------------------------------------------------
#include "CApplication.h"
#include "CViewport.h"
//------------------------------------------------------------------------------

namespace MyApplication {

    using namespace System;
    using namespace System::ComponentModel;
    using namespace System::Collections;
    using namespace System::Windows::Forms;
    using namespace System::Data;
    using namespace System::Drawing;
    
    /// <summary>
    /// Summary for MainForm
    /// </summary>
    public ref class MainForm : public System::Windows::Forms::Form
    {
    public:
        MainForm(void)
        {
            InitializeComponent();

            //------------------------------------------------------------------
            // CHAI3D - INITIALIZATION
            //------------------------------------------------------------------

            // intialize variables
            moveCamera = false;

            // create application
            application= new cApplication();
            
            // initilize display size
            application->resizeGL(this->panelView->Size.Width, this->panelView->Size.Height);

            // create OpenGL viewport 
            viewport = new cViewport((HWND)this->panelView->Handle.ToPointer());

            // initialize GLEW library (OpenGL)
            viewport->preRender();
            glewInit();
            viewport->postRender();

            // start haptics thread
            application->start();
        }

    protected:
        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        ~MainForm()
        {
            if (components)
            {
                delete components;
            }
        }

    private: System::Windows::Forms::Panel^  panelView;
    private: System::Windows::Forms::GroupBox^  groupBox;
    private: System::Windows::Forms::MenuStrip^  menuStrip;
    private: System::Windows::Forms::ToolStripMenuItem^  menuToolStripMenuItem;
    private: System::ComponentModel::IContainer^  components;

    private:
        /// <summary>
        /// Required designer variable.
        /// </summary>

        //------------------------------------------------------------------
        // CHAI3D - OBJECTS
        //------------------------------------------------------------------

        // application
        cApplication* application;

        // mouse information
        int mouseX;
        int mouseY;
        bool moveCamera;

        // display viewport
        cViewport* viewport;
    private: System::Windows::Forms::ToolStripMenuItem^  exitToolStripMenuItem;
    private: System::Windows::Forms::Label^  label1;
    private: System::Windows::Forms::TrackBar^  trackBar1;
    private: System::Windows::Forms::StatusStrip^  statusStrip;
    private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel;


        // timer for graphic rendering
        System::Windows::Forms::Timer^  timer1;


#pragma region Windows Form Designer generated code
        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        void InitializeComponent(void)
        {
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(MainForm::typeid));
			this->panelView = (gcnew System::Windows::Forms::Panel());
			this->groupBox = (gcnew System::Windows::Forms::GroupBox());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->menuStrip = (gcnew System::Windows::Forms::MenuStrip());
			this->menuToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->exitToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->statusStrip = (gcnew System::Windows::Forms::StatusStrip());
			this->toolStripStatusLabel = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->groupBox->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
			this->menuStrip->SuspendLayout();
			this->statusStrip->SuspendLayout();
			this->SuspendLayout();
			// 
			// panelView
			// 
			this->panelView->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->panelView->ForeColor = System::Drawing::SystemColors::AppWorkspace;
			this->panelView->Location = System::Drawing::Point(217, 27);
			this->panelView->Name = L"panelView";
			this->panelView->Size = System::Drawing::Size(572, 478);
			this->panelView->TabIndex = 1;
			this->panelView->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::panelView_MouseDown);
			this->panelView->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::panelView_MouseMove);
			this->panelView->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::panelView_MouseUp);
			this->panelView->Resize += gcnew System::EventHandler(this, &MainForm::panelView_Resize);
			// 
			// groupBox
			// 
			this->groupBox->Anchor = static_cast<System::Windows::Forms::AnchorStyles>(((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left));
			this->groupBox->Controls->Add(this->trackBar1);
			this->groupBox->Controls->Add(this->label1);
			this->groupBox->Location = System::Drawing::Point(11, 27);
			this->groupBox->Name = L"groupBox";
			this->groupBox->Size = System::Drawing::Size(200, 478);
			this->groupBox->TabIndex = 2;
			this->groupBox->TabStop = false;
			this->groupBox->Text = L"Options";
			// 
			// trackBar1
			// 
			this->trackBar1->Location = System::Drawing::Point(47, 19);
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Size = System::Drawing::Size(147, 45);
			this->trackBar1->TabIndex = 1;
			this->trackBar1->Scroll += gcnew System::EventHandler(this, &MainForm::trackBar1_Scroll);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(7, 20);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(34, 13);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Zoom";
			// 
			// menuStrip
			// 
			this->menuStrip->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->menuToolStripMenuItem });
			this->menuStrip->Location = System::Drawing::Point(0, 0);
			this->menuStrip->Name = L"menuStrip";
			this->menuStrip->Size = System::Drawing::Size(789, 24);
			this->menuStrip->TabIndex = 4;
			this->menuStrip->Text = L"menuStrip1";
			// 
			// menuToolStripMenuItem
			// 
			this->menuToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->exitToolStripMenuItem });
			this->menuToolStripMenuItem->Name = L"menuToolStripMenuItem";
			this->menuToolStripMenuItem->Size = System::Drawing::Size(37, 20);
			this->menuToolStripMenuItem->Text = L"File";
			// 
			// exitToolStripMenuItem
			// 
			this->exitToolStripMenuItem->Name = L"exitToolStripMenuItem";
			this->exitToolStripMenuItem->Size = System::Drawing::Size(92, 22);
			this->exitToolStripMenuItem->Text = L"Exit";
			this->exitToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::exitToolStripMenuItem_Click);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &MainForm::timer1_Tick);
			// 
			// statusStrip
			// 
			this->statusStrip->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->toolStripStatusLabel });
			this->statusStrip->Location = System::Drawing::Point(0, 508);
			this->statusStrip->Name = L"statusStrip";
			this->statusStrip->Size = System::Drawing::Size(789, 22);
			this->statusStrip->TabIndex = 3;
			this->statusStrip->Text = L"statusStrip1";
			// 
			// toolStripStatusLabel
			// 
			this->toolStripStatusLabel->Name = L"toolStripStatusLabel";
			this->toolStripStatusLabel->Size = System::Drawing::Size(12, 17);
			this->toolStripStatusLabel->Text = L"-";
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(789, 530);
			this->Controls->Add(this->statusStrip);
			this->Controls->Add(this->menuStrip);
			this->Controls->Add(this->groupBox);
			this->Controls->Add(this->panelView);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^>(resources->GetObject(L"$this.Icon")));
			this->MainMenuStrip = this->menuStrip;
			this->Name = L"MainForm";
			this->Text = L"CHAI3D";
			this->groupBox->ResumeLayout(false);
			this->groupBox->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
			this->menuStrip->ResumeLayout(false);
			this->menuStrip->PerformLayout();
			this->statusStrip->ResumeLayout(false);
			this->statusStrip->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

    //--------------------------------------------------------------------------
    // CHAI3D - METHODS
    //--------------------------------------------------------------------------
    private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e)
             {
                 UNREFERENCED_PARAMETER(sender);
                 UNREFERENCED_PARAMETER(e);

                 // update status bar
                 this->toolStripStatusLabel->Text = String::Format("Graphics: {0} Hz   Haptics: {1} Hz", 
                                                    this->application->m_graphicRate.getFrequency(), 
                                                    this->application->m_hapticRate.getFrequency());
                 // render scene
                 viewport->preRender();
                 application->paintGL();
                 viewport->postRender();
             }

    private: System::Void panelView_Resize(System::Object^  sender, System::EventArgs^  e) 
            {
                int w = this->panelView->Size.Width;
                int h = this->panelView->Size.Height;
                this->application->resizeGL(w, h);
            }

    private: System::Void exitToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) 
            {
                Close();
            }

    private: System::Void trackBar1_Scroll(System::Object^  sender, System::EventArgs^  e) 
            {
                double value = 0.5 * (double)(this->trackBar1->Value);
                this->application->m_camera->setSphericalRadius(value);
            }

    private: System::Void panelView_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
             {
                 moveCamera = true;
                 mouseX = e->X;
                 mouseY = e->Y;
             }

    private: System::Void panelView_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
             {
                 if (moveCamera)
                 {
                    // compute mouse motion
                    int dx = e->X - mouseX;
                    int dy = e->Y - mouseY;
                    mouseX = e->X;
                    mouseY = e->Y;

                    // compute new camera angles
                    double azimuthDeg = this->application->m_camera->getSphericalAzimuthDeg() + (0.5 * dy);
                    double polarDeg = this->application->m_camera->getSphericalPolarDeg() + (-0.5 * dx);

                    // assign new angles
                    this->application->m_camera->setSphericalAzimuthDeg(azimuthDeg);
                    this->application->m_camera->setSphericalPolarDeg(polarDeg);

                    // line up tool with camera
                    this->application->m_tool->setLocalRot(this->application->m_camera->getLocalRot());
                 }
             }

    private: System::Void panelView_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
             {
                 moveCamera = false;
             }
};
}
