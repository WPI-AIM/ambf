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
#ifndef CGELMassParticleH
#define CGELMassParticleH
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELMassParticle.h

    \brief
    Implementation of a mass particle.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELMassParticle
    \ingroup    GEL

    \brief
    This class implements a simple mass particle point.
*/
//===========================================================================
class cGELMassParticle
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELMassParticle.
    cGELMassParticle();

    //! Destructor of cGELMassParticle.
    virtual ~cGELMassParticle();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! This method assigns the mass of this particle.
    void setMass(double a_mass);

    //! This method adds a force to this mass particle.
    inline void addForce(chai3d::cVector3d &a_force)
    {
        m_force.add(a_force);
    }

    //! This method assigns an external force to this mass particle.
    inline void setExternalForce(chai3d::cVector3d &a_force)
    {
        m_externalForce = a_force;
    }

    //! This method updates the simulation over a specified time interval.
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
    }

    //! This method updates the position with its latest computed values.
    inline void applyNextPose()
    {
        m_pos = m_nextPos;
    }

    //! This method clears all forces.
    inline void clearForces()
    {
        if (m_useGravity)
        {
            m_gravity.mulr(m_mass, m_force);
        }
        else
        {
            m_force.zero();
        }
    }

    //! This method clears all external forces.
    inline void clearExternalForces()
    {
        m_externalForce.zero();
    }

    //! This method renders this node in OpenGL.
    inline void render()
    {
        // draw point
        m_color.render();
        glBegin(GL_POINTS);
          glVertex3dv( (const double *)&m_pos);
        glEnd();

        // render external forces
        glColor4fv( (const float *)&m_color);
        chai3d::cVector3d v = cAdd(m_pos, cMul(s_scale_force_vector_display, m_externalForce));
        glBegin(GL_LINES);
          glVertex3dv( (const double *)&m_pos);
          glVertex3dv( (const double *)&v);
        glEnd();
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Mass property.
    double m_mass;

    //! Current force being applied.
    chai3d::cVector3d m_force;

    //! Instant acceleration.
    chai3d::cVector3d m_acc;

    //! Instant velocity.
    chai3d::cVector3d m_vel;

    //! Linear damping.
    double m_kDampingPos;

    //! If __true__, then mass is fixed in space and cannot move.
    bool m_fixed;

    //! If __true__, then gravity field is taken into account.
    bool m_useGravity;

    //! Gravity field.
    chai3d::cVector3d m_gravity;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Color used to display nodes.
    chai3d::cColorf m_color;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - POSITION:
    //-----------------------------------------------------------------------

public:

    //! Position of particle.
    chai3d::cVector3d m_pos;

    //! Next computed position of particle.
    chai3d::cVector3d m_nextPos;

    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS - EXTERNAL FORCES:
    //-----------------------------------------------------------------------

private:

    //! External force applied onto this particle.
    chai3d::cVector3d m_externalForce;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Default property - linear damping.
    static double s_default_kDampingPos;

    //! Default property - mass.
    static double s_default_mass;

    //! Default property - color.
    static chai3d::cColorf s_default_color;

    //! Default property - use of gravity field.
    static bool s_default_useGravity;

    //! Default property - gravity field.
    static chai3d::cVector3d s_default_gravity;

    //! Default property - force display status.
    static bool s_show_forces;

    //! Default property - force vector display magnitude.
    static bool s_scale_force_vector_display;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
