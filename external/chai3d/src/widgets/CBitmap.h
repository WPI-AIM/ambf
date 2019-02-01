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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBitmapH
#define CBitmapH
//------------------------------------------------------------------------------
#include "widgets/CPanel.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBitmap.h

    \brief
    Implements a 2D bimap widget
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBitmap
    \ingroup    widgets

    \brief
    This class implements a 2D bitmap widget.

    \details
    This class provides functionalities to display a bitmap image. A bitmap
    image can be zoomed along its width and/or height.
*/
//==============================================================================
class cBitmap : public cPanel
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBitmap.
    cBitmap();

    //! Destructor of cBitmap.
    virtual ~cBitmap();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cBitmap* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);

    //! This method loads a bitmap image from a file.
    bool loadFromFile(std::string a_filename);

    //! This method loads a bitmap image from an image structure.
    bool loadFromImage(cImagePtr a_image);

    //! This method sets the horizontal and vertical zoom factors.
    void setZoom(const float a_zoomWidth,
                 const float a_zoomHeight);

    //! This method returns the zoom factor along the horizontal axis.
    double getZoomHeight() const { return (m_zoomHeight); }

    //! This method returns the zoom factor along the vertical axis.
    double getZoomWidth() const { return (m_zoomWidth); }

    //! This method updates the mesh model of this bitmap.
    virtual void updateBitmapMesh();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Horizontal zoom factor.
    double m_zoomWidth;

    //! Vertical zoom factor.
    double m_zoomHeight;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS: 
    //--------------------------------------------------------------------------

protected:

    //! This method copies all properties of this object to another.
    void copyBitmapProperties(cBitmap* a_obj,
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
