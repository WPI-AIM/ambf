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
    \author    Francois Conti
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericWidgetH
#define CGenericWidgetH
//------------------------------------------------------------------------------
#include "world/CMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericWidget.h

    \brief
    Implements a base class for widgets.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericWidget
    \ingroup    widgets

    \brief
    This class implements a base class for widgets.

    \details
    cGenericWidget implements a base class for widgets. Widgets can be 
    positioned anywhere on the front and back layers of the camera.
*/
//==============================================================================
class cGenericWidget : public cMesh
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericWidget.
    cGenericWidget();

    //! Destructor of cGenericWidget.
    virtual ~cGenericWidget() {};


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cGenericWidget* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS: SIZES
    //--------------------------------------------------------------------------

public:

    //! This method returns the width of the widget.
    virtual double getWidth() const { return (m_width); }

    //! This method returns the height of the widget.
    virtual double getHeight() const { return (m_height); }

    //! This method updates the bounding box of the widget.
    virtual void updateBoundaryBox()
    {
        m_boundaryBoxMin.set(0.0, 0.0, 0.0);
        m_boundaryBoxMax.set(m_width, m_height, 0.0);
    }


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS: ROTATING WIDGETS
    //--------------------------------------------------------------------------

public:

    //! This method rotates the widget around the z-axis with an angle defined in degrees.
    virtual void rotateWidgetDeg(double a_angleDeg) { rotateAboutLocalAxisDeg(cVector3d(0,0,1), a_angleDeg); }

    //! This method rotates the widget around the z-axis with an angle defined in radians.
    virtual void rotateWidgetRad(double a_angleRad) { rotateAboutLocalAxisRad(cVector3d(0,0,1), a_angleRad); }

    //! This method rotates the widget around its center point with an angle defined in degrees.
    virtual void rotateWidgetAroundCenterDeg(double a_angleDeg) { rotateWidgetAroundCenterRad(cDegToRad(a_angleDeg)); }

    //! This method rotates the widget around its center point with an angle defined in radians.
    virtual void rotateWidgetAroundCenterRad(double a_angleRad);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method copies all properties of this object to another.
    void copyGenericWidgetProperties(cGenericWidget* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Width of this widget.
    double m_width;

    //! Height of this widget.
    double m_height;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
