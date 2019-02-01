//===========================================================================
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
    \version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGELSkeletonNodeH
#define CGELSkeletonNodeH
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELSkeletonNode.h

    \brief
    Implementation of a skeleton mass node.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELSkeletonNode
    \ingroup    GEL

    \brief
    This class implements a mass node with mass and inertia properties.
*/
//===========================================================================
class cGELSkeletonNode
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELSkeletonNode.
    cGELSkeletonNode();

    //! Destructor of cGELSkeletonNode.
    ~cGELSkeletonNode();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method sets the mass of the node.
    void setMass(double a_mass);

    //! This method adds a force to the node.
    inline void addForce(chai3d::cVector3d &a_force)
    {
        m_force.add(a_force);
    }

    //! This method adds a torque to the node.
    inline void addTorque(chai3d::cVector3d &a_torque)
    {
        m_torque.add(a_torque);
    }

    //! This method sets an external force to the node.
    inline void setExternalForce(chai3d::cVector3d &a_force)
    {
        m_externalForce = a_force;
    }

    //! This method sets an external torque to the node.
    inline void setExternalTorque(chai3d::cVector3d &a_torque)
    {
        m_externalTorque = a_torque;
    }

    //! This method update the position of the node over a time interval.
    inline void computeNextPose(double a_timeInterval)
    {
        if (!m_fixed)
        {
            // Euler double integration for position
            chai3d::cVector3d damping;
            m_vel.mulr(-m_kDampingPos * m_mass, damping);
            m_force.add(damping);
            m_acc = cDiv(m_mass, cAdd(m_force, m_externalForce));
            m_vel = cAdd(m_vel, cMul(a_timeInterval, m_acc));
            m_nextPos = cAdd(m_pos, cMul(a_timeInterval, m_vel));
        }
        else
        {
            m_nextPos = m_pos;
        }

        // Euler double integration for rotation
        chai3d::cVector3d dampingAng;
        m_angVel.mulr(-m_kDampingRot * m_mass, dampingAng);
        m_torque.add(dampingAng);
        m_angAcc = cMul((1/m_inertia), m_torque);
        m_angVel = cAdd(m_angVel, cMul(a_timeInterval, m_angAcc));

        double normAngVel = m_angVel.length();
        if (normAngVel < 0.000001)
        {
            m_nextRot = m_rot;
        }
        else
        {
            m_nextRot = m_rot;
            m_nextRot.rotateAboutGlobalAxisRad(m_angVel, a_timeInterval * normAngVel);
        }
    }

    //! This method updates the position and orientation of this node with the latest computed values.
    inline void applyNextPose()
    {
        m_pos = m_nextPos;
        m_rot = m_nextRot;
    }

    //! This method clears the current forces and torques.
    inline void clearForces()
    {
        if (m_useGravity)
        {
            m_force = m_gravity;
            m_force.mul(m_mass);
        }
        else
        {
            m_force.zero();
        }
        m_torque.zero();
    }

    //! This method clears the external forces and torques.
    inline void clearExternalForces()
    {
        m_externalForce.zero();
        m_externalTorque.zero();
    }

    //! This method renders this node in OpenGL.
    inline void render()
    {
        // set pose
        chai3d::cTransform mat;
        mat.set(m_pos, m_rot);
        glPushMatrix();
        glMultMatrixd( (const double *)mat.getData() );

        // draw node
        m_color.render();
        chai3d::cDrawSphere(m_radius, 12, 12);

        // draw frame
        if (s_default_showFrame == true)
        {
            double frameScale = 3.0 * m_radius;
            chai3d::cDrawFrame(frameScale);
        }

        // pop OpenGL matrix
        glPopMatrix();

        // render external forces
        glColor4fv( (const float *)&m_color);
        chai3d::cVector3d v = cAdd(m_pos, cMul(1.0/50.0, m_externalForce));
        glBegin(GL_LINES);
            glVertex3dv( (const double *)&m_pos);
            glVertex3dv( (const double *)&v);
        glEnd();
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Color used to display nodes.
    chai3d::cColorf m_color;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Radius of mass node.
    double m_radius;

    //! Mass property.
    double m_mass;

    //! Current force applied on node.
    chai3d::cVector3d m_force;

    //! Current torque applies on node.
    chai3d::cVector3d m_torque;

    //! Instant acceleration at node.
    chai3d::cVector3d m_acc;

    //! Instant angular acceleration at node.
    chai3d::cVector3d m_angAcc;

    //! Instant velocity at node.
    chai3d::cVector3d m_vel;

    //! Instant angular velocity at node.
    chai3d::cVector3d m_angVel;

    //! Linear damping.
    double m_kDampingPos;

    //! Angular damping.
    double m_kDampingRot;

    //! Inertia (to be completed).
    double m_inertia;

    //! If __true__, then mass is fixed in space and can not move.
    bool m_fixed;

    //! If __true__, then gravity is enabled.
    bool m_useGravity;

    //! Gravity field.
    chai3d::cVector3d m_gravity;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - POSITION & ORIENTATION:
    //-----------------------------------------------------------------------

public:

    //! Position computed.
    chai3d::cVector3d m_pos;

    //! Rotation computed.
    chai3d::cMatrix3d m_rot;

    //! Next position computed.
    chai3d::cVector3d m_nextPos;

    //! Next rotation computed.
    chai3d::cMatrix3d m_nextRot;


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS - EXTERNAL FORCES:
    //-----------------------------------------------------------------------

private:

    //! External force.
    chai3d::cVector3d m_externalForce;

    //! External torque.
    chai3d::cVector3d m_externalTorque;


    //-----------------------------------------------------------------------
    // STATIC MEMBERS - DEFAULT SETTINGS:
    //-----------------------------------------------------------------------

public:

    //! Default value for node radius.
    static double s_default_radius;

    //! Default value for linear damping.
    static double s_default_kDampingPos;

    //! Default value for revolute damping.
    static double s_default_kDampingRot;

    //! Default value for node mass.
    static double s_default_mass;

    //! Default value - Is gravity field enabled?
    static bool s_default_useGravity;

    //! Default value - Gravity field magnitude and direction.
    static chai3d::cVector3d s_default_gravity;

    //! Default value - Is the node reference frame displayed?
    static bool s_default_showFrame;

    //! Default color used to render the node.
    static chai3d::cColorf s_default_color;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


