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
    \version   3.2.0 $Rev: 2174 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "shaders/CShader.h"
//------------------------------------------------------------------------------
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
//------------------------------------------------------------------------------
using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::ifstream;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShader.
*/
//==============================================================================
cShader::cShader(cShaderType a_type)
{
    // initialization
    m_id = 0;
    m_compiled = false;

    // store shader type
    m_type = a_type;
}


//==============================================================================
/*!
    This method loads the shader content from a char array.

    \param  a_source  Source code.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cShader::loadSourceCode(const char *a_source)
{
    // keep hold of a copy of the source
    m_source = a_source;

    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method loads the shader content from a string.

    \param  a_source  Source code.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cShader::loadSourceCode(const std::string& a_source)
{
    // keep hold of a copy of the source
    m_source = a_source;

    return (C_SUCCESS); 
}

//==============================================================================
/*!
    This method loads the shader content from a file.

    \param  a_fileName  Filename containing source code.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cShader::loadSourceFile(const std::string& a_fileName)
{
    ifstream file;

    file.open(a_fileName.c_str());

    if (!file.good() )
    {
        return (C_ERROR);
    }

    // create a string stream
    stringstream stream;

    // dump the contents of the file into it
    stream << file.rdbuf();

    // close the file
    file.close();

    // convert the StringStream into a string
    m_source = stream.str();

    return (C_SUCCESS); 
}


//==============================================================================
/*!
    This method compile a shader and displays any problems if compilation fails.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cShader::compile()
{
#ifdef C_USE_OPENGL
    // check if shader has already been compiled
    if (m_compiled) { return (C_SUCCESS); }

    // create shader if not yet allocated
    if (m_id == 0)
    {
        switch (m_type)
        {
        case C_VERTEX_SHADER:
            m_id = glCreateShader(GL_VERTEX_SHADER);
            break;

        case C_FRAGMENT_SHADER:
            m_id = glCreateShader(GL_FRAGMENT_SHADER);
            break;

        case C_GEOMETRY_SHADER:
            m_id = glCreateShader(GL_GEOMETRY_SHADER_EXT);
            break;
        }
    }

    // get the source string as a pointer to an array of characters
    const char *sourceChars = m_source.c_str();

    // associate the source with the shader id
    glShaderSource(m_id, 1, &sourceChars, NULL);

    // compile the shader
    glCompileShader(m_id);

    // check the compilation status and report any errors
    GLint shaderStatus;
    glGetShaderiv(m_id, GL_COMPILE_STATUS, &shaderStatus);

    // if the shader failed to compile, display the info log and quit out
    if (shaderStatus == GL_FALSE)
    {
        GLint infoLogLength;
        glGetShaderiv(m_id, GL_INFO_LOG_LENGTH, &infoLogLength);

        GLchar *strInfoLog = new GLchar[infoLogLength + 1];
        glGetShaderInfoLog(m_id, infoLogLength, NULL, strInfoLog);

        // display error message
        cout << "shader compilation failed:" << endl << strInfoLog << endl;

        printf("shader compilation failed:\n");
        string error = strInfoLog;
        printf("%s", error.c_str());

        // store log
        stringstream stream;
        stream << strInfoLog << endl;
        m_log = stream.str();
        
        // compilation failed
        m_compiled = false;

        delete[] strInfoLog;
        return (C_ERROR);
    }
    else
    {
        m_compiled = true;
        return (C_SUCCESS);
    }
#else
    return (C_ERROR);
#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
