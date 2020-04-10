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
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------
using std::vector;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShaderProgram.
*/
//==============================================================================
cShaderProgram::cShaderProgram()
{
    // initialization
    m_id = 0;
    m_enabled = false;
    m_linked = false;
    m_shaders.clear();
    m_shaderCount = 0;

    m_geometryShaderAttached = false;
    m_geometryInputType = GL_TRIANGLES;
    m_geometryOutputType = GL_TRIANGLE_STRIP;
    m_geometryVerticesOut = 24;
}


//==============================================================================
/*!
    Constructor of cShaderProgram.

    \param  a_vertexShader  Vertex shader code.
    \param  a_vertexShader  Fragment shader code.
*/
//==============================================================================
cShaderProgram::cShaderProgram(const std::string& a_vertexShader, const std::string& a_fragmentShader)
{
    // initialization
    m_id = 0;
    m_enabled = false;
    m_linked = false;
    m_shaders.clear();
    m_shaderCount = 0;

    m_geometryShaderAttached = false;
    m_geometryInputType = GL_TRIANGLES;
    m_geometryOutputType = GL_TRIANGLE_STRIP;
    m_geometryVerticesOut = 24;

    // create vertex shader
    cShaderPtr vertexShader = cShader::create(C_VERTEX_SHADER);

    // load source code
    vertexShader->loadSourceCode(a_vertexShader);

    // create fragment shader
    cShaderPtr fragmentShader = cShader::create(C_FRAGMENT_SHADER);

    // load source code
    fragmentShader->loadSourceCode(a_fragmentShader);

    // assign vertex shader to program shader
    attachShader(vertexShader);

    // assign fragment shader to program shader
    attachShader(fragmentShader);

    // link program shader
    linkProgram();
}


//==============================================================================
/*!
    Destructor of cShaderProgram.
*/
//==============================================================================
cShaderProgram::~cShaderProgram()
{
#ifdef C_USE_OPENGL
    if (m_id > 0)
    {
        glDeleteProgram(m_id);
    }
#endif
}


//==============================================================================
/*!
    This method attaches a shader to the shader program.

    \param  a_shader  Shader object.
*/
//==============================================================================
void cShaderProgram::attachShader(cShaderPtr a_shader)
{
    // add shader to list
    m_shaders.push_back(a_shader);

    // increment the number of shaders we have associated with the program
    m_shaderCount++;

    // if attaching a geometry shader, make note to set parameters later
    if (a_shader->getType() == C_GEOMETRY_SHADER)
        m_geometryShaderAttached = true;
}


//==============================================================================
/*!
    This method links the shader program and return the link status.

    \return __true__ if the operation succeeded, __false__otherwise.
*/
//==============================================================================
bool cShaderProgram::linkProgram()
{
#ifdef C_USE_OPENGL
    // check for necessary OpenGL extensions
#ifdef GLEW_VERSION
    if(!(GLEW_ARB_shading_language_100))
    {
        printf("GLEW has not been initialized. Please call glewInit() before initializing any shader.\n");
        return (C_ERROR);
    }
#endif

    // check if program shader has already been linked
    if (m_linked) { return (C_SUCCESS); }

    // Generate a unique Id / handle for the shader program if needed
    if (m_id == 0)
    {
        m_id = glCreateProgram();
    }

    // If we have at least two shaders (like a vertex shader and a fragment shader)...
    if (m_shaderCount > 0)
    {
        // attach all shaders to program shader
        vector<cShaderPtr>::iterator it;
        for (it = m_shaders.begin(); it < m_shaders.end(); it++)
        {
            // make sure shader is compiled
            if ((*it)->isCompiled() == false)
            {
                (*it)->compile();
            }

            // attach shader
            if ((*it)->isCompiled() == true)
            {
                glAttachShader(m_id, (*it)->getId());
            }
        }

        // assign default location
        glBindAttribLocation(m_id, C_VB_POSITION, "aPosition");
        glBindAttribLocation(m_id, C_VB_NORMAL, "aNormal");
        glBindAttribLocation(m_id, C_VB_TEXCOORD, "aTexCoord");
        glBindAttribLocation(m_id, C_VB_COLOR, "aColor");
        glBindAttribLocation(m_id, C_VB_TANGENT, "aTangent");
        glBindAttribLocation(m_id, C_VB_BITANGENT, "aBitangent");

        // set geometry shader input/output types
        if (m_geometryShaderAttached)
        {
            glProgramParameteriEXT(m_id, GL_GEOMETRY_INPUT_TYPE_EXT, m_geometryInputType);
            glProgramParameteriEXT(m_id, GL_GEOMETRY_OUTPUT_TYPE_EXT, m_geometryOutputType);
            glProgramParameteriEXT(m_id, GL_GEOMETRY_VERTICES_OUT_EXT, m_geometryVerticesOut);
        }

        // perform the linking process
        glLinkProgram(m_id);

        // check the status
        GLint linkStatus;
        glGetProgramiv(m_id, GL_LINK_STATUS, &linkStatus);
        if (linkStatus == GL_FALSE)
        {
            m_linked = false;
            return (C_ERROR);
        }
        else
        {
            m_linked = true;
            return (C_SUCCESS);
        }
    }
    else
    {
        m_linked = false;
        return (C_ERROR);
    }
#else
    return (C_ERROR);
#endif
}


//==============================================================================
/*!
    This method enables the shader program.

    \param  a_object   Object on which the program shader will apply
    \param  a_options  Rendering options.
*/
//==============================================================================
void cShaderProgram::use(cGenericObject* a_object, cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL
    // use program
    if (m_linked)
    {
        glUseProgram(m_id);
        m_enabled = true;
    }
#endif
}


//==============================================================================
/*!
    This method disables the shader program.
*/
//==============================================================================
void cShaderProgram::disable()
{
#ifdef C_USE_OPENGL
    glUseProgram(0);
    m_enabled = false;
#endif
}


//==============================================================================
/*!
    This method binds an attribute to a name.

    \param  a_index  Index of the attribute to be bound
    \param  a_name   Name of the attribute.
*/
//==============================================================================
void cShaderProgram::bindAttributeLocation(const unsigned int a_index, const char *a_name)
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // get location
    glUseProgram(m_id);
    glBindAttribLocation(m_id, a_index, a_name);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method returns the location of a specified attribute.

    \param  a_name  Name of the attribute.

    \return Attribute location.
*/
//==============================================================================
int cShaderProgram::getAttributeLocation(const char *a_name) 
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return(C_ERROR); } 

    // get location
    glUseProgram(m_id);
    int attributeLocation = glGetAttribLocation(m_id, a_name);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }

    return (attributeLocation);
#else
    return (0);
#endif
}


//==============================================================================
/*!
    This method returns the location of a specified uniform.

    \param  a_name  Name of the uniform.

    \return Uniform location.
*/
//==============================================================================
int cShaderProgram::getUniformLocation(const char *a_name) 
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return(C_ERROR); } 

    // get location
    glUseProgram(m_id);
    int attributeLocation = glGetUniformLocation(m_id, a_name);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }

    return (attributeLocation);
#else
    return (0);
#endif
}


//==============================================================================
/*!
    This method sets an __int__ uniform to a specified value.

    \param  a_name   Name of the uniform.
    \param  a_value  New value of the uniform.
*/
//==============================================================================
void cShaderProgram::setUniformi(const char* a_name, const GLint a_value)
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    glUseProgram(m_id);
    glUniform1i(location, a_value);
    
    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets a __float__ uniform to a specified value.

    \param  a_name   Name of the uniform.
    \param  a_value  New value of the uniform.
*/
//==============================================================================
void cShaderProgram::setUniformf(const char* a_name, const GLfloat a_value)
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    glUseProgram(m_id);
    glUniform1f(location, a_value);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets a \ref cVector3d uniform to a specified value.

    \param  a_name   Name of the uniform.
    \param  a_value  New value of the uniform.
*/
//==============================================================================
void cShaderProgram::setUniform(const char* a_name, cVector3d& a_value) 
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    glUseProgram(m_id);
    glUniform3f(location, (GLfloat)a_value(0), (GLfloat)a_value(1), (GLfloat)a_value(2));

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets a \ref cMatrix3d uniform to a specified value.

    \param  a_name        Name of the uniform.
    \param  a_value       New 3x3 matrix of the uniform.
    \param  a_transposed  Is the matrix transposed?
*/
//==============================================================================
void cShaderProgram::setUniform(const char* a_name, cMatrix3d& a_value, bool a_transposed)
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    GLfloat m[9] = {
        GLfloat(a_value(0,0)), GLfloat(a_value(0,1)), GLfloat(a_value(0,2)),
        GLfloat(a_value(1,0)), GLfloat(a_value(1,1)), GLfloat(a_value(1,2)),
        GLfloat(a_value(2,0)), GLfloat(a_value(2,1)), GLfloat(a_value(2,2)),
    };
    glUseProgram(m_id);
    glUniformMatrix3fv(location, 1, a_transposed, m);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets a \ref cTransform uniform to a specified value.

    \param  a_name        Name of the uniform.
    \param  a_value       New 4x4 matrix of the uniform.
    \param  a_transposed  Is the matrix transposed?
*/
//==============================================================================
void cShaderProgram::setUniform(const char* a_name, cTransform& a_value, bool a_transposed)
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    double *matrixData = a_value.getData();
    GLfloat m[16];
    for (int i = 0; i < 16; ++i)
    {
        m[i] = GLfloat(matrixData[i]);
    }

    glUseProgram(m_id);
    glUniformMatrix4fv(location, 1, a_transposed, m);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets an __int__ array uniform to specified values.

    \param  a_name    Name of the uniform.
    \param  a_values  Pointer to an array with the new values.
    \param  a_count   Number of values in the given array.
*/
//==============================================================================
void cShaderProgram::setUniformiv(const char *a_name, const GLint *a_values, const int a_count) 
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; } 

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    glUseProgram(m_id);
    glUniform1iv(location, a_count, a_values);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//==============================================================================
/*!
    This method sets a __float__ array uniform to specified values.

    \param  a_name    Name of the uniform.
    \param  a_values  Pointer to an array with the new values.
    \param  a_count   Number of values in the given array.
*/
//==============================================================================
void cShaderProgram::setUniformfv(const char *a_name, const GLfloat *a_values, const int a_count) 
{
#ifdef C_USE_OPENGL
    // check program
    if (!m_linked) { return; }

    // check location
    GLint location = glGetUniformLocation(m_id, a_name);
    if (location < 0) { return; }

    // assign value
    glUseProgram(m_id);
    glUniform1fv(location, a_count, a_values);

    // finalize
    if (!m_enabled)
    {
        glUseProgram(0);
    }
#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
