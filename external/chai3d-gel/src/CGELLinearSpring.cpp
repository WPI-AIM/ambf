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
#include "CGELLinearSpring.h"
//---------------------------------------------------------------------------
#include "CGELMesh.h"
//---------------------------------------------------------------------------
using namespace chai3d;
//---------------------------------------------------------------------------

//===========================================================================
// DEFINITION - DEFAULT VALUES:
//===========================================================================

// Physical properties:
double cGELLinearSpring::s_default_kSpringElongation = 100.0; // [N/m]

// Graphic properties:
cColorf cGELLinearSpring::s_default_color(1.0f, 0.2f, 0.2f);


//===========================================================================
/*!
    Constructor of cGELLinearSpring.

    \param  a_node0  Node 0.
    \param  a_node1  Node 1.
*/
//===========================================================================
cGELLinearSpring::cGELLinearSpring(cGELMassParticle* a_node0, cGELMassParticle* a_node1)
{
    // set nodes:
    m_node0 = a_node0;
    m_node1 = a_node1;
    m_enabled = true;

    // set default color
    m_color = s_default_color;

    // compute initial length
    m_length0 = cDistance(m_node1->m_pos, m_node0->m_pos);

    // set elongation spring constant [N/M]
    m_kSpringElongation = s_default_kSpringElongation;
}


//===========================================================================
/*!
    Destructor of cGELLinearSpring.
*/
//===========================================================================
cGELLinearSpring::~cGELLinearSpring()
{
}
