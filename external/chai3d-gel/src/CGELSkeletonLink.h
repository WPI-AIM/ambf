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
#ifndef CGELSkeletonLinkH
#define CGELSkeletonLinkH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGELSkeletonNode.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELSkeletonLink.h

    \brief
    Implementation of an elastic link that connects two skeleton nodes
    together.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELSkeletonLink
    \ingroup    GEL

    \brief
    This class models an elastic link between two skeleton nodes.
*/
//===========================================================================
class cGELSkeletonLink
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cGELSkeletonLink.
    cGELSkeletonLink(cGELSkeletonNode* a_node0, cGELSkeletonNode* a_node1);

    //! Destructor of cGELSkeletonLink.
    ~cGELSkeletonLink();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

public:

    //-----------------------------------------------------------------------
    /*!
        \brief
         This method graphically renders this  skeleton link using OpenGL.
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
         This method computes the forces applied on the two mass nodes
         that this link connects.
    */
    //-----------------------------------------------------------------------
    inline void computeForces()
    {
        // update basic parameters of current link
        m_wLink01 = cSub(m_node1->m_pos, m_node0->m_pos);
        m_wLink10 = cMul(-1, m_wLink01);
        m_length = m_wLink01.length();

        m_wzLink01 = cMul((m_node0->m_rot), m_nzLink01);
        m_wzLink10 = cMul((m_node1->m_rot), m_nzLink10);

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
            chai3d::cVector3d force = cMul(f/m_length, m_wLink01);
            m_node0->addForce(force);
            chai3d::cVector3d tmpfrc = cMul(-1, force);
            m_node1->addForce(tmpfrc);
        }

        //-------------------------------------------------------------
        // FLEXION:
        //-------------------------------------------------------------
        // compute flexion forces and torques
        chai3d::cVector3d torqueDir0 = cCross(m_wzLink01, m_wLink01);
        chai3d::cVector3d torqueDir1 = cCross(m_wzLink10, m_wLink10);
        double angle0 = cAngle(m_wzLink01, m_wLink01);
        double angle1 = cAngle(m_wzLink10, m_wLink10);
        double torqueMag0 = angle0 * m_kSpringFlexion;
        double torqueMag1 = angle1 * m_kSpringFlexion;

        if ((m_length > 0.000001) && (m_enabled))
        {
            if (torqueMag0 > 0.0001)
            {
                chai3d::cVector3d torque0 = cMul(torqueMag0, cNormalize(torqueDir0));
                m_node0->addTorque(torque0);
                chai3d::cVector3d force1 = cMul((torqueMag0/m_length), cNormalize(cCross(m_wLink01, torque0)));
                m_node1->addForce(force1);
                chai3d::cVector3d tmpfrc = cMul(-1,force1);
                m_node0->addForce(tmpfrc);
            }

            if (torqueMag1 > 0.0001)
            {
                chai3d::cVector3d torque1 = cMul(torqueMag1, cNormalize(torqueDir1));
                m_node1->addTorque(torque1);
                chai3d::cVector3d force0 = cMul((torqueMag1/m_length), cNormalize(cCross(m_wLink10, torque1)));
                m_node0->addForce(force0);
                chai3d::cVector3d tmpfrc = cMul(-1,force0);
                m_node1->addForce(tmpfrc);
            }
        }

        //-----------------------------------------------------------------------
        // TORSION:
        //-----------------------------------------------------------------------

        if (m_enabled)
        {
            // update frame vectors in world coordinates.
            m_node0->m_rot.mulr(m_A0, m_wA0);
            m_node0->m_rot.mulr(m_B0, m_wB0);
            m_node1->m_rot.mulr(m_A1, m_wA1);

            // compute torsional angle and torque
            chai3d::cVector3d v0, v1;
            m_wA0.crossr(m_wLink01, v0);
            m_wA1.crossr(m_wLink01, v1);
            v0.normalize();
            v1.normalize();

            chai3d::cVector3d torque;
            v0.crossr(v1, m_torsionAxisAngle);
            m_torsionAxisAngle.mulr(m_kSpringTorsion, torque);

            m_node0->addTorque(torque);
            chai3d::cVector3d tmptrq = -1.0 * torque;
            m_node1->addTorque(tmptrq);
        }
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - MASS NODES:
    //-----------------------------------------------------------------------

public:

    //! Node 0 of current link.
    cGELSkeletonNode *m_node0;

    //! Node 1 of current link.
    cGELSkeletonNode *m_node1;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Color used to display this link.
    chai3d::cColorf m_color;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Elongation spring constant.
    double m_kSpringElongation;

    //! Angular spring constant.
    double m_kSpringFlexion;

    //! Torsional spring constant.
    double m_kSpringTorsion;

    //! Is link enabled.
    bool m_enabled;

    //! Linear damping constant.
    //double m_kDamperElongation;

    //! Angular damping constant.
    //double m_kDamperFlexion;

    //! Torsion damping constant.
    //double m_kDamperTorsion;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Initial link vector between node 0 and node 1 in node0 reference frame.
    chai3d::cVector3d m_nzLink01;

    //! Initial link vector between node 0 and node 1 in world coordinates.
    chai3d::cVector3d m_wzLink01;

    //! Initial link vector between node 1 and node 0 in node1 reference frame.
    chai3d::cVector3d m_nzLink10;

    //! Initial link vector between node 1 and node 0 in world coordinates.
    chai3d::cVector3d m_wzLink10;

    //! Current link between node 0 and node 1 in world coordinates.
    chai3d::cVector3d m_wLink01;

    //! Current link between node 1 and node 0 in world coordinates.
    chai3d::cVector3d m_wLink10;

    //! Torsional angle.
    chai3d::cVector3d m_torsionAxisAngle;

    //! Reference vector frame A on sphere 0 (local coordinates).
    chai3d::cVector3d m_A0;

    //! Reference vector frame A on sphere 0 (world coordinates).
    chai3d::cVector3d m_wA0;

    //! Reference vector frame B on sphere 0 (local coordinates).
    chai3d::cVector3d m_B0;

    //! Reference vector frame B on sphere 0 (world coordinates).
    chai3d::cVector3d m_wB0;

    //! Reference vector frame A on sphere 1 (local coordinates).
    chai3d::cVector3d m_A1;

    //! Reference vector frame A on sphere 1 (world coordinates).
    chai3d::cVector3d m_wA1;

    //! Initial length of link.
    double m_length0;

    //! Current length of link.
    double m_length;


    //-----------------------------------------------------------------------
    // STATIC MEMBERS - DEFAULT SETTINGS:
    //-----------------------------------------------------------------------

public:

    //! Default property - linear stiffness.
    static double s_default_kSpringElongation;   // [N/m]

    //! Default property - angular stiffness.
    static double s_default_kSpringFlexion;      // [Nm/RAD]

    //! Default property - torsional stiffness.
    static double s_default_kSpringTorsion;      // [Nm/RAD]

    //! Default property - color property.
    static chai3d::cColorf s_default_color;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
