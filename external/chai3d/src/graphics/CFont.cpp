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
#include "graphics/CFont.h"
//------------------------------------------------------------------------------
#include <fstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of class cFont.
*/
//==============================================================================
cFont::cFont()
{
    // create new texture object
    m_texture = new cTexture2d();

    // initialize all data structures
    cleanup();
}


//==============================================================================
/*!
    Destructor of class cFont.
*/
//==============================================================================
cFont::~cFont()
{
    delete m_texture;
}


//==============================================================================
/*!
    This method cleans up all data structures.
*/
//==============================================================================
void cFont::cleanup()
{
    m_texture->m_image->clear();
    m_filename  = "";
    m_charset.initialize();
}


//==============================================================================
/*!
    This method loads a font from the specified file.

    \param  a_filename  Font filename.
*/
//==============================================================================
bool cFont::loadFromFile(const string& a_filename)
{
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // verify file extension
    if (fileType != "fnt")
    {
        return (false);
    }

    // load font file into memory
    std::ifstream  fontFile;
    fontFile.open(a_filename.c_str(), std::ios::binary);
    
    // failed to load file
    if (fontFile.good() == false)
    {
        return (false);
    }

    // font file has been loaded, we can now cleanup previous data
    cleanup();

    // parse font file
    parseFont(fontFile, m_charset);
    m_charset.preProcess();

    // close file
    fontFile.close();

    // compute texture image filename by replacing the extension with .png
    //string imageFile = cGetDirectory(a_filename)+"/"+m_charset.m_fileName;

    int length = (int)(a_filename.length() - extension.length());
    string imageFile = a_filename.substr(0, length-1);
    string imageFile1 = imageFile+".png";
    string imageFile2 = imageFile+"_0.png";

    // load font image into memory
    if (!m_texture->m_image->loadFromFile(imageFile1))
    {
        if (!m_texture->m_image->loadFromFile(imageFile2))
        {
            return (false);
        }
    }

    // store file name
    m_filename = a_filename;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method parses a font from the specified stream.

    \param  a_stream       Font filename
    \param  a_charsetDesc  Description of the new charset.
*/
//==============================================================================
void cFont::parseFont(std::istream& a_stream, 
                      cFontCharset& a_charsetDesc)
{
    std::string Line;
    std::string Read, Key, Value;
    std::size_t i;
    while( !a_stream.eof() )
    {
        std::stringstream LineStream;
        std::getline( a_stream, Line );
        LineStream << Line;

        //read the line's type
        LineStream >> Read;
        if( Read == "common" )
        {
            //this holds common data
            while( !LineStream.eof() )
            {
                std::stringstream Converter;
                LineStream >> Read;
                i = Read.find( '=' );
                Key = Read.substr( 0, i );
                Value = Read.substr( i + 1 );

                //assign the correct value
                Converter << Value;
                if( Key == "lineHeight" )
                    Converter >> a_charsetDesc.m_lineHeight;
                else if( Key == "base" )
                    Converter >> a_charsetDesc.m_base;
                else if( Key == "scaleW" )
                    Converter >> a_charsetDesc.m_width;
                else if( Key == "scaleH" )
                    Converter >> a_charsetDesc.m_height;
                else if( Key == "pages" )
                    Converter >> a_charsetDesc.m_pages;
                else if( Key == "file" )
                    Converter >> a_charsetDesc.m_fileName;
            }
        }
        else if( Read == "char" )
        {
            //this is data for a specific char
            unsigned short CharID = 0;

            while( !LineStream.eof() )
            {
                std::stringstream Converter;
                LineStream >> Read;
                i = Read.find( '=' );
                Key = Read.substr( 0, i );
                Value = Read.substr( i + 1 );

                //assign the correct value
                Converter << Value;
                if( Key == "id" )
                    Converter >> CharID;
                else if( Key == "x" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_x;
                else if( Key == "y" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_y;
                else if( Key == "width" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_width;
                else if( Key == "height" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_height;
                else if( Key == "xoffset" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_xOffset;
                else if( Key == "yoffset" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_yOffset;
                else if( Key == "xadvance" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_xAdvance;
                else if( Key == "page" )
                    Converter >> a_charsetDesc.m_chars[CharID].m_page;
            }
        }
    }
}


//==============================================================================
/*!
    This method renders a single string and returns its length.

    \param  a_text           String to be rendered.
    \param  a_color          Font Color
    \param  a_fontScale      Font scale factor
    \param  a_letterSpacing  Spacing between letters
    \param  a_lineSpacing    Spacing between lines
    \param  a_options        Rendering options.

    \return Length of string.
*/
//==============================================================================
double cFont::renderText(const string& a_text,
                         const cColorf& a_color,
                         const double a_fontScale,
                         const double a_letterSpacing,
                         const double a_lineSpacing,
                         cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL
    // sanity check
    if (m_texture == nullptr)
    {
        return (0);
    }

    // get length of text
    int length = (int)(a_text.length());
    if (length == 0) { return(0.0); }

    // get height of single line
    double lineHeight = this->getPointSize();

    // count number of lines
    int numLines = 1;
    for (int i = 0; i<length; i++)
    {
        int index = a_text[i];
        while (index < 0) { index = 256 + index; }
        if (index == '\n')
        {
            numLines++;
        }
    }

    // initialization
    double cursorX = 0;
    double cursorY = (numLines - 1) * (a_fontScale * lineHeight * a_lineSpacing);

    // render texture
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // render texture
    m_texture->renderInitialize(a_options);

    // render font texture
    m_texture->setEnvironmentMode(GL_MODULATE);
    
    // render font color
    a_color.render();

    // render a quad for each character in the string
    glBegin(GL_QUADS);

    // get reference to texture unit
    GLenum textureUnit = m_texture->getTextureUnit();

    // get size of font.
    double offset = getPointSize();

     // render each character
    for (int i=0; i<length; i++)
    {
        int index = a_text[i];

        // seb's magic formula for unicode > 127 (which is not extended ASCII)
        if (index < 0) index = 381 + index + a_text[++i];

        if (index == '\n')
        {
            cursorY = cursorY - (a_fontScale * lineHeight * a_lineSpacing);
            cursorX = 0;
        }

        cFontCharDescriptor *ch = &(m_charset.m_chars[index]);

        // render character
        glNormal3d(0.0, 0.0, 1.0);
        glMultiTexCoord2dARB(textureUnit, ch->m_tu0, ch->m_tv0);
        glVertex3d((double)(a_fontScale * ch->m_px0 + cursorX), (double)(a_fontScale * (ch->m_py0 + offset) + cursorY), 0.0);
        glMultiTexCoord2dARB(textureUnit, ch->m_tu1, ch->m_tv1);
        glVertex3d((double)(a_fontScale * ch->m_px1 + cursorX), (double)(a_fontScale * (ch->m_py1 + offset) + cursorY), 0.0);
        glMultiTexCoord2dARB(textureUnit, ch->m_tu2, ch->m_tv2);
        glVertex3d((double)(a_fontScale * ch->m_px2 + cursorX), (double)(a_fontScale * (ch->m_py2 + offset) + cursorY), 0.0);
        glMultiTexCoord2dARB(textureUnit, ch->m_tu3, ch->m_tv3);
        glVertex3d((double)(a_fontScale * ch->m_px3 + cursorX), (double)(a_fontScale * (ch->m_py3 + offset) + cursorY), 0.0);

        // increment cursor
        cursorX += a_fontScale * m_charset.m_chars[index].m_xAdvance * a_letterSpacing;
    }

    // all character quads have been rendered
    glEnd();

    // disable texture
    m_texture->renderFinalize(a_options);

    // cleanup
    glDisable(GL_BLEND);

    // return position of cursor
    return (cursorX);

#else
    return (0);
#endif
}


//==============================================================================
/*!
    This method computes the width of a string \p a_text passed as argument.

    \param  a_text  Input string.
    \param  a_letterSpacing  Letter spacing factor

    \return Width of input text.
*/
//==============================================================================
double cFont::getTextWidth(const string& a_text, const double a_letterSpacing)
{
    // get width of string
    int length = (int)(a_text.length());
    if (length == 0) { return(0.0); }

    // initialization
    double cursorX = 0;
    double width = 0;

    // parse each character
    for (int i=0; i<length; i++)
    {
        int index = a_text[i];
        while (index < 0) { index = 256 + index; }
        if (index == '\n')
        {
            cursorX = 0;
        }
        else
        {
            cursorX += (double)(m_charset.m_chars[index].m_xAdvance) * a_letterSpacing;
            width = cMax(width, cursorX);
        }
    }

    // return total length
    return (width);
}


//==============================================================================
/*!
    This method returns the heigth of a particular string using this font.

    \param  a_text  Input string.
    \param  a_lineSpacing  Line spacing. Default value if 1.0

    \return Height of input text.
*/
//==============================================================================
double cFont::getTextHeight(const std::string& a_text, const double a_lineSpacing)
{
    int numLines = 1;

    // get height of single line
    double lineHeight = this->getPointSize();
    
    // get length of text
    int length = (int)(a_text.length());

    // count number of lines
    for (int i = 0; i<length; i++)
    {
        int index = a_text[i];
        while (index < 0) { index = 256 + index; }
        if (index == '\n')
        {
            numLines++;
        }
    }

    // compute total height (number of lines and line spacing between lines=
    double height = lineHeight + (numLines - 1)  * lineHeight * a_lineSpacing;

    // return total height
    return (height);
}


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*
    Font Processing (internal use only).
*/
//==============================================================================
void cFontCharset::preProcess()
{
    for(int index=0; index<256; index++)
    {
        int charX   = m_chars[index].m_x;
        int charY   = m_chars[index].m_y;
        int width   = m_chars[index].m_width;
        int height  = m_chars[index].m_height;
        int offsetX = m_chars[index].m_xOffset;
        int offsetY = m_chars[index].m_yOffset;

        // lower left
        m_chars[index].m_tu0 = (double)charX / (double) m_width;
        m_chars[index].m_tv0 = 1.0 - ((double)(charY+height) / (double) m_height);
        m_chars[index].m_px0 = offsetX;
        m_chars[index].m_py0 = -(height + offsetY);

        // lower right
        m_chars[index].m_tu1 = (double)(charX+width) / (double) m_width;
        m_chars[index].m_tv1 = 1.0 - ((double)(charY+height) / (double) m_height);
        m_chars[index].m_px1 = width + offsetX;
        m_chars[index].m_py1 = -(height + offsetY);

        // upper right
        m_chars[index].m_tu2 = (double)(charX+width) / (double) m_width;
        m_chars[index].m_tv2 = 1.0 - ((double)charY / (double) m_height);
        m_chars[index].m_px2 = width + offsetX;
        m_chars[index].m_py2 = -offsetY;

        // upper left
        m_chars[index].m_tu3 = (double)charX / (double) m_width;
        m_chars[index].m_tv3 = 1.0 - ((double)charY / (double) m_height);
        m_chars[index].m_px3 = offsetX;
        m_chars[index].m_py3 = -offsetY;
    }
}

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
