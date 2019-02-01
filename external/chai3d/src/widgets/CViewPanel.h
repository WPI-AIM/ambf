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
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CViewPanelH
#define CViewPanelH
//------------------------------------------------------------------------------
#include "display/CFrameBuffer.h"
#include "widgets/CPanel.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CViewPanel.h

    \brief
    Implements a 2D display panel associated with a framebuffer.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cViewPanel
    \ingroup    widgets

    \brief
    This class implements a 2D display panel associated with a framebuffer.

    \details
    This class implements a 2D display panel that displays the image coming
    from a framebuffer.
*/
//==============================================================================
class cViewPanel : public cPanel
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cViewPanel.
    cViewPanel(cFrameBufferPtr a_frameBuffer);

    //! Destructor of cViewPanel.
    virtual ~cViewPanel();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method assigns a framebuffer to this object.
    void setFrameBuffer(cFrameBufferPtr a_frameBuffer);

    //! This method returns a pointer to the current framebuffer.
    cFrameBufferPtr getFrameBuffer() { return (m_frameBuffer); }

    //! This method creates a copy of itself.
    virtual cViewPanel* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Framebuffer.
    cFrameBufferPtr m_frameBuffer;

    //--------------------------------------------------------------------------
    // PROTECTED METHODS: 
    //--------------------------------------------------------------------------

protected:

    //! This method copies properties of this object to another.
    void copyViewPanelProperties(cViewPanel* a_obj,
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
