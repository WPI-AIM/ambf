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
    \version   3.2.0 $Rev: 2173 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericCollisionH
#define CGenericCollisionH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file   CGenericCollision.h
    
    \brief
    Implements a base class for programming collision detectors that identify 
    intersections between segments points, lines, and triangles.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericCollision
    \ingroup    collisions

    \brief
    This class implements a base class for collision detection.

    \details
    This class implements a base class for programming collision-detection
    algorithms between objects and line segments defined by two points.\n\n

    A collision detector is instantiated for a specific object (e.g. mesh)
    passed by reference in the constructor. The collision detector must then 
    be initialized by calling method initialize() which computes and builds the 
    necessary data structures for the particular object. (e.g. collision tree)\n\n

    The optional initialize() method takes one argument named a_radius which defines
    a boundary distance around every triangle (an enclosing shell). When the 
    collision inquiry method is called \ref computeCollision(), collisions are 
    searched between a segment, passed as argument, and the shells that cover every 
    triangle. If this radius is set to zero, then the shells are equal to the 
    triangles themselves. This option is used by the finger-proxy force rendering 
    algorithm to compute the intersection between a sphere (haptic point) and 
    the surface of a mesh.\n\n

    If the shape of the object is modified (e.g triangles are added or removed
    from a mesh), then the \ref update() command of the collision detector
    must be called again. The method is responsible for deallocating any 
    previously built data structures.\n\n

    Please note that this class does not support collision detection 
    between objects themselves.\n\n
*/
//==============================================================================
class cGenericCollision
{

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericCollision.
    cGenericCollision();

    //! Destructor of cGenericCollision.
    virtual ~cGenericCollision() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This methods updates the collision detector and should be called if the 3D model it represents is modified.
    virtual void update() {}

    //! This method computes all collisions between a segment passed as argument and the attributed 3D object.
    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings)
                                  { return (false); }

    //! This method renders a visual representation of the collision tree.
    virtual void render(cRenderOptions& a_options) {};

    //! This method returns the radius of the boundary shell that covers every triangles.
    double getBoundaryRadius() const { return (m_radiusAroundElements); }

    //! This method sets the level of the collision tree to display.
    void setDisplayDepth(const int a_depth) { m_displayDepth = a_depth; }

    //! This method returns the level inside the collision tree being displayed. (root = 0).
    double getDisplayDepth() const { return (m_displayDepth); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

    //! Color property used to render the collision detector graphically.
    cColorf m_color;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    /*!
        Level of collision tree to render. Negative values force rendering
        up to and including this level, positive values render just this level.
    */
    int m_displayDepth;

    /*!
        Radius boundary around elements. This value is must be equal or larger
        than the physical radius of the proxy.
    */
    double m_radiusAroundElements;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
