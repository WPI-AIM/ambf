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
    \author    Francois Conti, Sonny Chan
    \version   3.2.0 $Rev: 2187 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CShaderH
#define CShaderH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "system/CString.h"
#include "math/CConstants.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#include "graphics/COpenGLHeaders.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CShader.h

    \brief
    Implements an OpenGL shader primitive.
*/
//==============================================================================

//------------------------------------------------------------------------------
// vertex array bindings
#define C_VB_INDEX_BUFFER       0
#define C_VB_POSITION           1
#define C_VB_NORMAL             2
#define C_VB_TEXCOORD           3
#define C_VB_COLOR              4
#define C_VB_TANGENT            5
#define C_VB_BITANGENT          6

// texture unit bindings
#define C_TU_TEXTURE            0
#define C_TU_SHADOWMAP          1
#define C_TU_NORMALMAP          2
#define C_TU_SPECULARMAP        3

#define C_TU_IMAGEBUFFER        2
#define C_TU_DEPTHBUFFER        3

#define C_TU_IBL_ENVIRONMENT    4
#define C_TU_IBL_FILTERED       5
#define C_TU_IBL_BRDF           6
     

       
#define C_TU_UNALLOCATED        7

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
enum cShaderType
{
    C_VERTEX_SHADER          = 0x0001,
    C_FRAGMENT_SHADER        = 0x0002,
    C_GEOMETRY_SHADER        = 0X0003
};
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cShader;
typedef std::shared_ptr<cShader> cShaderPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cShader
    \ingroup    shaders
    
    \brief
    This class implements a shader primitive

    \details
    This class implements the basic capabilities to load and control an OpenGL
    shader.
*/
//==============================================================================
class cShader
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShader.
    cShader(cShaderType a_type);

    //! Destructor of cShader.
    virtual ~cShader(){};

    //! Shared cShader allocator.
    static cShaderPtr create(cShaderType a_type) { return (std::make_shared<cShader>(a_type)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method loads the shader content from a char array.
    bool loadSourceCode(const char *a_source);

    //! This method loads the shader content from a string.
    bool loadSourceCode(const std::string& a_source);

    //! This method loads the shader content from a file.
    bool loadSourceFile(const std::string& a_fileName);

    //! This method compiles a shader and display any problems if compilation fails.
    bool compile(); 

    //! This method returns __true__ if shader has been compiled successfully, __false__ otherwise.
    bool isCompiled() const { return (m_compiled); }
    
    //! This method returns the log file generated from compilation.
    std::string getLog() const { return (m_log); }

    //! This method returns the shader OpenGL ID.
    GLuint getId() const { return (m_id); }

    //! This method returns the source code of the shader.
    std::string getSource() { return (m_source); }

    //! This method returns the type of the shader.
    cShaderType getType() const { return (m_type); }


    //--------------------------------------------------------------------------
    // STATIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns __true__ if shaders are supported on this computer, __false__ otherwise.
    static bool hasOpenGLShaders(cShaderType a_type);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Shader type (C_VERTEX_SHADER, C_FRAGMENT_SHADER, or C_GEOMETRY_SHADER).
    cShaderType m_type;

    //! Handle ID for the shader.
    GLuint m_id;

    //! The shader source code (i.e. the GLSL code itself).
    std::string m_source; 

    //! Log message following compilation.
    std::string m_log;

    //! If __true__ then shader was compiled successfully, __false__ otherwise.
    bool m_compiled;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
