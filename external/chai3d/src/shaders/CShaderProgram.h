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
#ifndef CShaderProgramH
#define CShaderProgramH
//------------------------------------------------------------------------------
#include "shaders/CShader.h"
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------
#include <iostream>
#include <map>
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CShaderProgram.h

    \brief
    Implements a shader program primitive.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cShaderProgram;
typedef std::shared_ptr<cShaderProgram> cShaderProgramPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cShaderProgram
    \ingroup    shaders
    
    \brief
    This class implements a shader program primitive

    \details
    This class implements a shader program that contain a list of shader 
    primitives. Shader programs can be assigned to cGenericObject entities
    to control their graphical output.
*/
//==============================================================================
class cShaderProgram
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShaderProgram.
    cShaderProgram();

    //! Constructor of cShaderProgram.
    cShaderProgram(const std::string& a_vertexShader, const std::string& a_fragmentShader);

    //! Destructor of cShaderProgram.
    virtual ~cShaderProgram();

    //! Shared cShaderProgram allocator.
    static cShaderProgramPtr create() { return (std::make_shared<cShaderProgram>()); }

    //! Shared cShaderProgram allocator.
    static cShaderProgramPtr create(const std::string& a_vertexShader, const std::string& a_fragmentShader) { return (std::make_shared<cShaderProgram>(a_vertexShader, a_fragmentShader)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method attaches a shader to the shader program.
    void attachShader(cShaderPtr a_shader);

    //! This method links the shader program and returns the link status.
    bool linkProgram();

    //! This method enables the shader program.
    void use(cGenericObject* a_object, cRenderOptions& a_options);

    //! This method disables the shader program.
    void disable();

    //! This method returns __true__ if this shader is currently in use. __false__ otherwise.
    bool isUsed() { return (m_enabled); }

    //! This method binds an attribute to a name.
    void bindAttributeLocation(const unsigned int a_index, const char *a_name);

    //! This method returns the location of a specified attribute.
    int getAttributeLocation(const char *a_name);

    //! This method returns the location of a specified uniform.
    int getUniformLocation(const char *a_name);

    //! This method returns the program ID.
    GLint getId() { return (m_id); }

    //! Set the geometry shader input primitype type.
    void setGeometryInputType(GLenum a_type) { m_geometryInputType = a_type; }

    //! Set the geometry shader output primitype type.
    void setGeometryOutputType(GLenum a_type) { m_geometryOutputType = a_type; }

    //! Set the maximum number of vertices emitted by the gemoetry shader.
    void setGeometryVerticesOut(int a_numVertices) { m_geometryVerticesOut = a_numVertices; }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS: (UNIFORMS)
    //--------------------------------------------------------------------------

public:

    //! This method sets an __int__ uniform to a specified value.
    void setUniformi(const char* a_name, const GLint a_value);

    //! This method sets a __float__ uniform to a specified value.
    void setUniformf(const char* a_name, const GLfloat a_value);

    //! This method sets a \ref cVector3d uniform to a specified value.
    void setUniform(const char* a_name, cVector3d& a_value);

    //! This method sets a \ref cMatrix3d uniform to a specified value.
    void setUniform(const char* a_name, cMatrix3d& a_value, bool a_transposed);

    //! This method sets a \ref cTransform uniform to a specified value.
    void setUniform(const char* a_name, cTransform& a_value, bool a_transposed);

    //! This method sets an __int__ array uniform to specified values.
    void setUniformiv(const char* a_name, const GLint *a_values, const int a_count);

    //! This method sets a __float__ array uniform to specified values.
    void setUniformfv(const char* a_name, const GLfloat *a_values, const int a_count);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! OpenGL handle ID for the shader program.
    GLuint m_id; 

    //! If __true__ then shader program was successfully linked, __false__ otherwise.
    bool m_linked;

    //! If __true__ then program shader is currently enabled.
    bool m_enabled;

    //! Number of shaders that are attached to the shader program.
    GLuint m_shaderCount;

    //! Map of attributes and their binding locations.
    std::map<std::string, int> m_attributeLocList;

    //! Map of uniforms and their binding locations.
    std::map<std::string, int> m_uniformLocList;

    //! List of attached shaders.
    std::vector<cShaderPtr> m_shaders;

    //! Whether or not this program has a geometry shader attached
    bool m_geometryShaderAttached;

    //! Geometry shader input primitive type, if applicable
    GLenum m_geometryInputType;

    //! Geometry shader output primitive type, if applicable
    GLenum m_geometryOutputType;

    //! Maximum number of vertices generated by the geometry shader
    GLint m_geometryVerticesOut;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
