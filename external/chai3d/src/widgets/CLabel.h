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
    \version   3.2.0 $Rev: 2153 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CLabelH
#define CLabelH
//------------------------------------------------------------------------------
#include "widgets/CPanel.h"
#include "graphics/CFont.h"
#include "graphics/CColor.h"
#include "system/CString.h"
#include <string>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CLabel.h

    \brief 
    Implements a 2D text label.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cLabel
    \ingroup    widgets

    \brief
    This class implements a 2D label widget to display one line of text.

    \details
    This class implements a 2D label widget to display one line of text.
*/
//==============================================================================
class cLabel : public cPanel
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cLabel.
    cLabel(cFontPtr a_font);

    //! Destructor of cLabel.
    virtual ~cLabel();


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Font type.
    cFontPtr m_font;

    //! Font color.
    cColorf m_fontColor;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! This method assign a string text to this label.
    void setText(const std::string a_text);

    //! This method returns the text associated with this label.
    std::string getText() const { return (m_text); }

    //! This method sets the font scale factor.
    void setFontScale(const double a_scale);

    //! This method returns the font scale factor.
    double getFontScale() const { return (m_fontScale); }

    //! This method sets the letter spacing.
    void setLetterSpacing(const double a_letterSpacing) { m_letterSpacing = a_letterSpacing; }

    //! This method returns the current letter spacing value.
    double getLetterSpacing() { return (m_letterSpacing); }

    //! This method sets the line spacing when multiple lines of text are used.
    void setLineSpacing(const double a_lineSpacing) { m_lineSpacing = a_lineSpacing; }

    //! This method returns the current line spacing value.
    double getLineSpacing() { return (m_lineSpacing); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! This method computes any collisions between a segment and this object.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                cVector3d& a_segmentPointB,
                                                cCollisionRecorder& a_recorder,
                                                cCollisionSettings& a_settings);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Scale factor of fonts.
    double m_fontScale;

    //! Letter spacing. Default value is 1.0
    double m_letterSpacing;

    //! Line spacing. Default value is 1.0
    double m_lineSpacing;

    //! Text to be displayed.
    std::string m_text;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of this object.
    cLabel* copy(const bool a_duplicateMaterialData = false,
                 const bool a_duplicateTextureData = false, 
                 const bool a_duplicateMeshData = false,
                 const bool a_buildCollisionDetector = false);

    //! This method returns the width of the text in pixels.
    virtual double getTextWidth() const;

    //! This method returns the height of the text in pixels.
    virtual double getTextHeight() const;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method renders the object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method copies all properties of this object to another.
    void copyLabelProperties(cLabel* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
