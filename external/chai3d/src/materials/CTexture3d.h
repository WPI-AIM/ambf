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
#ifndef CTexture3dH
#define CTexture3dH
//------------------------------------------------------------------------------
#include "graphics/CMultiImage.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CTexture3d.h

    \brief
    Implements 3D textures.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cTexture3d;
typedef std::shared_ptr<cTexture3d> cTexture3dPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cTexture3d
    \ingroup    materials

    \brief
    This class implements a 3D texture map.

    \details
    This class implements a 3D texture map.
*/
//==============================================================================
class cTexture3d : public cTexture2d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cTexture3d.
    cTexture3d();

    //! Destructor of cTexture3d.
    virtual ~cTexture3d();

    //! Shared cTexture3d allocator.
    static cTexture3dPtr create() { return (std::make_shared<cTexture3d>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    cTexture3dPtr copy();

    //! This method enables texturing and sets this texture as the current texture.
    virtual void renderInitialize(cRenderOptions& a_options);

    //! This method disables texture once triangles have been rendered.
    virtual void renderFinalize(cRenderOptions& a_options);

    //! This method set the texture wrap mode of __r__.
    virtual void setWrapModeR(const GLint& a_wrapModeR);

    //! This method returns the texture wrap mode of __r__.
    GLint getWrapModeR() const { return (m_wrapModeR); }

    //! This method marks this texture for partial GPU update from RAM.
    void markForPartialUpdate(const cVector3d a_voxelUpdateMin, const cVector3d a_voxelUpdateMax);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method resets internal variables. This function should be called only by constructors.
    virtual void reset();

    //! This method updates this texture to GPU.
    virtual void update(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ the partial update is performed.
    bool m_markPartialUpdate;

    //! Minimum voxel coordinate to be updated.
    cVector3d m_voxelUpdateMin;

    //! Maximum voxel coordinate to be updated.
    cVector3d m_voxelUpdateMax;

    //! Texture wrap parameter along __r__ (GL_REPEAT or GL_CLAMP).
    GLint m_wrapModeR;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
