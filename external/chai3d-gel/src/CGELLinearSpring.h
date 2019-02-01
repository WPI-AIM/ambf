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
#ifndef CGELLinearSpringH
#define CGELLinearSpringH
//---------------------------------------------------------------------------
#include "CGELMassParticle.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELLinearSpring.h

    \brief
    Implementation on a linear spring.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELLinearSpring
    \ingroup    GEL

    \brief
    This class implements a linear spring between two mass particles.
*/
//===========================================================================
class cGELLinearSpring
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELLinearSpring.
    cGELLinearSpring(cGELMassParticle* a_node0, cGELMassParticle* a_node1);

    //! Destructor of cGELLinearSpring.
    ~cGELLinearSpring();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //-----------------------------------------------------------------------
    /*!
        \brief
         This method graphically renders this spring using OpenGL.
    */
    //-----------------------------------------------------------------------
    inline void render()
    {
        // render link
        glColor4fv( (const float *)&m_color);
        glBegin(GL_LINES);
          glVertex3dv( (const double *)&m_node0->m_pos);
          glVertex3dv( (const double *)&m_node1->m_pos);
        glEnd();
    }


    //-----------------------------------------------------------------------
    /*!
        \brief
         This method computes the forces that are applied on the mass nodes.
    */
    //-----------------------------------------------------------------------
    inline void computeForces()
    {
        // update basic parameters of current link
        chai3d::cVector3d m_link01 = cSub(m_node1->m_pos, m_node0->m_pos);
        double m_length = m_link01.length();

        //-------------------------------------------------------------
        // ELONGATION:
        //-------------------------------------------------------------
        // if distance too small, no forces are applied
        if (m_length < 0.000001) { return; }

        // elongation compute force
        double f = m_kSpringElongation * (m_length - m_length0);

        // apply force
        if (m_length > 0.000001)
        {
            chai3d::cVector3d force = cMul(f/m_length, m_link01);
            m_node0->addForce(force);
            chai3d::cVector3d tmpfrc = cMul(-1, force);
            m_node1->addForce(tmpfrc);
        }
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Color property used for displaying link.
    chai3d::cColorf m_color;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - MASS NODES:
    //-----------------------------------------------------------------------
    
public:

    //! Node 0 of current link.
    cGELMassParticle *m_node0;

    //! Node 1 of current link.
    cGELMassParticle *m_node1;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Linear spring constant.
    double m_kSpringElongation;

    //! Is link enabled?
    bool m_enabled;

    //! Linear damping.
    double m_kDamperElongation;

    //! Initial length of link.
    double m_length0;


    //-----------------------------------------------------------------------
    // STATIC MEMBERS - DEFAULT SETTINGS:
    //-----------------------------------------------------------------------

public:

    //! Default property - linear stiffness.
    static double s_default_kSpringElongation;   // [N/m]

    //! Default property - color property.
    static chai3d::cColorf s_default_color;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

