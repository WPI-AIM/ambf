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
    \version   3.2.0 $Rev: 2173 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CFontH
#define CFontH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CFont.h

    \brief
    Implements support for fonts.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cFont;
typedef std::shared_ptr<cFont> cFontPtr;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

struct cFontCharDescriptor
{
    int m_x, m_y;
    int m_width, m_height;
    int m_xOffset, m_yOffset;
    int m_xAdvance;
    int m_page;

    double m_tu0, m_tu1, m_tu2, m_tu3;
    double m_tv0, m_tv1, m_tv2, m_tv3;
    double m_px0, m_px1, m_px2, m_px3;
    double m_py0, m_py1, m_py2, m_py3;

    cFontCharDescriptor()
    { 
        initialize();
    }

    void initialize()
    {
        m_x         = 0;
        m_y         = 0;
        m_width     = 0;
        m_height    = 0;
        m_xOffset   = 0;
        m_yOffset   = 0;
        m_xAdvance  = 0;
        m_page      = 0;
    }
};

//------------------------------------------------------------------------------

struct cFontCharset
{
    unsigned int m_lineHeight;
    unsigned int m_base;
    unsigned int m_width, m_height;
    unsigned int m_pages;
    std::string m_fileName;

    cFontCharDescriptor m_chars[256];

    cFontCharset()
    { 
        initialize();
    }

    void initialize()
    {
        m_lineHeight = 0;
        m_base       = 0;
        m_width      = 0;
        m_height     = 0;
        m_pages      = 0;

        for (int i=0; i<256; i++)
            m_chars[i].initialize();
    }

    void preProcess();
};

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    \class      cFont
    \ingroup    graphics

    \brief
    This class implements support for 2D fonts.

    \details
    This class provides support for font files generated from AngelSoft's 
    Bitmap Font Generator application. This program will allow you to generate
    bitmap fonts from TrueType fonts. The application generates both image 
    files and character descriptions that can be read by a game for easy 
    rendering of fonts. These files can then be loaded into the cFont class
    by using the loadFromFile() method.
*/
//==============================================================================
class cFont
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cFont.
    cFont();

    //! Destructor of cFont.
    virtual ~cFont();
    
    //! Shared cFont allocator.
    static cFontPtr create() { return (std::make_shared<cFont>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads a font file by passing a path and file name as argument.
    bool loadFromFile(const std::string& a_filename);

    //! This method returns the filename from which this font was last loaded or saved.
    std::string getFilename() const { return (m_filename); }
    

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FONT SETTINGS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the current font size.
    double getPointSize() const { return (m_charset.m_lineHeight); }

    //! This method returns the width of a particular string using this font.
    double getTextWidth(const std::string& a_text, const double a_letterSpacing);

    //! This method returns the heigth of a particular string using this font.
    double getTextHeight(const std::string& a_text, const double a_lineSpacing);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - OPENGL RENDERING:
    //--------------------------------------------------------------------------

public:

    //! This method renders a single string of text.
    double renderText(const std::string& a_text,
        const cColorf& a_color,
        const double a_fontScale,
        const double a_letterSpacing,
        const double a_lineSpacing,
        cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Texture image containing the full bitmap representation of the font.
    cTexture2d* m_texture;

    //! Information about each character in the set.
    cFontCharset m_charset;


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method parses a font file.
    void parseFont(std::istream& a_stream, cFontCharset& a_charsetDesc);

    //! This method cleans all memory.
    void cleanup();


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Current font filename loaded into memory.
    std::string m_filename;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
