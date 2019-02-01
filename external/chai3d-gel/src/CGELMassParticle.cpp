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
#include "CGELMassParticle.h"
//---------------------------------------------------------------------------
using namespace chai3d;
//---------------------------------------------------------------------------

//===========================================================================
// DEFINITION - DEFAULT VALUES:
//===========================================================================

// Physical properties:
double cGELMassParticle::s_default_kDampingPos   = 5.00;
double cGELMassParticle::s_default_mass          = 0.1;  // [kg]

// Graphic properties:
cColorf cGELMassParticle::s_default_color(1.0f, 1.0f, 0.4f);

// Gravity field:
bool cGELMassParticle::s_default_useGravity = true;
cVector3d cGELMassParticle::s_default_gravity(0.00, 0.00, -9.81);

// Default rendering mode
bool cGELMassParticle::s_show_forces = true;
bool cGELMassParticle::s_scale_force_vector_display = true;


//===========================================================================
/*!
    Constructor of cGELMassParticle.
*/
//===========================================================================
cGELMassParticle::cGELMassParticle()
{
    m_pos.zero();
    m_color         = s_default_color;
    m_externalForce.zero();
    m_force.zero();
    m_acc.zero();
    m_vel.zero();
    m_kDampingPos   = s_default_kDampingPos;
    m_gravity       = s_default_gravity;
    m_useGravity    = s_default_useGravity;
    m_fixed         = false;
    setMass(s_default_mass);
}


//===========================================================================
/*!
    Destructor of cGELMassParticle.
*/
//===========================================================================
cGELMassParticle::~cGELMassParticle()
{
}


//===========================================================================
/*!
    This method assigns a mass value passed as argument to this particle.

    \param  a_mass  Mass of particle.
*/
//===========================================================================
void cGELMassParticle::setMass(double a_mass)
{
    m_mass = a_mass;
}
