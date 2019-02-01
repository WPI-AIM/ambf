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
#ifndef CPanelH
#define CPanelH
//------------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPanel.h

    \brief 
    Implements a panel widget.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cPanel
    \ingroup    widgets

    \brief
    This class implements a 2D panel widget.

    \details
    This class implements a 2D panel widget. A panel is defined by a width and 
    a height. Radius values and color properties are defined for each of its 
    four corners. A panel is modeled as a mesh. If the panel is disabled, then
    no mesh is constructed when the internal \ref updatePanelMesh() method
    is called.
*/
//==============================================================================
class cPanel : public cGenericWidget
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cPanel.
    cPanel();

    //! Destructor of cPanel.
    virtual ~cPanel() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cPanel* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false,
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);

    //! This method sets the width, height, and radius of each corner of this panel.
    virtual void set(const double& a_width, 
        const double& a_height,
        const double& a_radiusTopLeft = 0,
        const double& a_radiusTopRight = 0,
        const double& a_radiusBottomLeft = 0,
        const double& a_radiusBottomRight = 0);

    //! This method sets the width and height of this panel.
    virtual void setSize(const double& a_width, 
        const double& a_height);

    //! This method sets the radius for each corner of this panel.
    virtual void setCornerRadius(const double& a_radiusTopLeft = 0,
        const double& a_radiusTopRight = 0,
        const double& a_radiusBottomLeft = 0,
        const double& a_radiusBottomRight = 0);

    //! This method sets the panel color.
    virtual void setColor(const cColorf& a_panelColor);

    //! This method sets different colors for all four corners of the panel.
    virtual void setCornerColors(const cColorf& a_panelColorTopLeft, 
        const cColorf& a_panelColorTopRight, 
        const cColorf& a_panelColorBottomLeft, 
        const cColorf& a_panelColorBottomRight);

    //! This method sets a vertical gradient color.
    virtual void setPanelColorVerticalGradient(cColorf a_topColor, 
        cColorf a_bottomColor);

    //! This method sets a horizontal gradient color.
    virtual void setHorizontalLinearGradient(cColorf a_leftColor, 
        cColorf a_rightColor);

    //! This method returns the color at top left corner of this panel.
    inline cColorf getColorTopLeft() const { return (m_panelColorTopLeft); }

    //! This method returns the color at top right corner of this panel.
    inline cColorf getColorTopRight() const { return (m_panelColorTopRight); }

    //! This method returns the color at to left corner  of this panel.
    inline cColorf getColorBottomLeft() const { return (m_panelColorBottomLeft); }

    //! This method returns the color at to left corner  of this panel.
    inline cColorf getColorBottomRight() const { return (m_panelColorBottomRight); }

    //! This method enables or disables the modeling of the panel.
    void setShowPanel(const bool a_showPanel) { m_showPanel = a_showPanel; updatePanelMesh(); }

    //! This method returns __true__ is this panel is enabled, __false__ otherwise.
    bool getShowPanel() { return (m_showPanel); }


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS: MARGINS
    //--------------------------------------------------------------------------

public:

    //! This method assigns margin values to this widget.
    virtual void setMargins(const double a_marginTop,
        const double a_marginBottom,
        const double a_marginLeft,
        const double a_marginRight);

    //! This method assigns a top margin value.
    virtual void setMarginTop(const double a_marginTop);

    //! This method returns the value of the top margin.
    virtual double getMarginTop() { return (m_marginTop); }

    //! This method assigns a bottom margin value.
    virtual void setMarginBottom(const double a_marginBottom);

    //! This method returns the value of the bottom margin.
    virtual double getMarginBottom() { return (m_marginBottom); }

    //! This method assigns a left margin value.
    virtual void setMarginLeft(const double a_marginLeft);

    //! This method returns the value of the left margin.
    virtual double getMarginLeft() { return (m_marginLeft); }

    //! This method assigns a right margin value.
    virtual void setMarginRight(const double a_marginRight);

    //! This method returns the value of the right margin.
    virtual double getMarginRight() { return (m_marginRight); }

    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method assigns a transparency level to this panel.
    void setTransparencyLevel(const float a_level,
        const bool a_applyToVertices = false,
        const bool a_applyToTextures = false,
        const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Radius of top left corner of this panel.
    double m_panelRadiusTopLeft;

    //! Radius of top right corner of this panel.
    double m_panelRadiusTopRight;

    //! Radius of bottom left corner of this panel.
    double m_panelRadiusBottomLeft;

    //! Radius of bottom right corner of this panel.
    double m_panelRadiusBottomRight;

    //! Number of segments used to render each corner of  thispanel.
    int m_numPanelSegmentsPerCorner;

    //! Left margin that is used to render internal content.
    double m_marginLeft;

    //! Right margin that is used to render internal content.
    double m_marginRight;

    //! Top margin that is used to render internal content.
    double m_marginTop;

    //! Bottom margin that is used to render internal content.
    double m_marginBottom;

    //! If __true__, panel is enabled for display. __false__ otherwise.
    bool m_showPanel;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method updates the mesh model of this panel.
    virtual void updatePanelMesh();

    //! This method copies all properties of this panel to another.
    void copyPanelProperties(cPanel* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Panel color at top left corner.
    cColorf m_panelColorTopLeft;

    //! Panel color at top right corner.
    cColorf m_panelColorTopRight;

    //! Panel color at bottom left corner.
    cColorf m_panelColorBottomLeft;

    //! Panel color at bottom right corner.
    cColorf m_panelColorBottomRight;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
