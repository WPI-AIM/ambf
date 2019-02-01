//==============================================================================
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
    \author    Chris Sewell
    \author    Francois Conti
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionBruteH
#define CCollisionBruteH
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "graphics/CGenericArray.h"
#include "graphics/CVertexArray.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionBrute.h

    \brief
    Implements a brute force approach for collision detection.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionBrute
    \ingroup    collisions

    \brief
    This class implements a brute force approach for collision detection.

    \details
    This class implements a brute force approach to check for all intersections
    between a line segment and an object composedcpoints, segments, or
    triangles.
*/
//==============================================================================
class cCollisionBrute : public cGenericCollision
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionBrute.
    cCollisionBrute(cGenericArrayPtr a_entities);

    //! Destructor of cCollisionBrute.
    virtual ~cCollisionBrute() {}


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method computes all collisions between a segment passed as argument and the attributed 3D object.
    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings);


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Pointer to the list of elements (points, segments, triangles).
    cGenericArrayPtr m_elements;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

