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
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVertexArrayH
#define CVertexArrayH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CColor.h"
#include "shaders/CShader.h"
//------------------------------------------------------------------------------
#include <vector>
#include <list>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVertexArray.h

    \brief
    Implements an array of 3D vertices.
*/
//==============================================================================


//==============================================================================
/*!
    \struct     cVertexArrayOptions
    \ingroup    graphics

    \brief
    This structure manages user options when creating vertex arrays.
*/
//==============================================================================
struct cVertexArrayOptions
{

public:

    cVertexArrayOptions(const bool a_useNormalData,
                        const bool a_useTexCoordData,
                        const bool a_useColorData,
                        const bool a_useTangentData,
                        const bool a_useBitangentData,
                        const bool a_useUserData)
    {
        m_useNormalData     = a_useNormalData;
        m_useTexCoordData   = a_useTexCoordData;
        m_useColorData      = a_useColorData;
        m_useTangentData    = a_useTangentData;
        m_useBitangentData  = a_useBitangentData;
        m_useUserData       = a_useUserData;
    }

    bool m_useNormalData;
    bool m_useTexCoordData;
    bool m_useColorData;
    bool m_useTangentData;
    bool m_useBitangentData;
    bool m_useUserData; 
};

//------------------------------------------------------------------------------
class cVertexArray;
typedef std::shared_ptr<cVertexArray> cVertexArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cVertexArray
    \ingroup    graphics

    \brief
    This class implements array of 3D vertices.

    \details
    cVertexArray defines a an array of 3D vertices (points) that can include 
    properties such as position, color, texture coordinate, and surface 
    normals. \n

    New vertices can be added to the array by calling \ref newVertex() or
    \ref newVertices(). Once points have been allocated, they cannot be removed,
    unless the entire array is cleared by calling \ref clear(). \n

    The properties of each vertex can be modified by calling the appropriate 
    methods and by passing the vertex index as argument with the associated
    data. \n
*/
//==============================================================================
class cVertexArray
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cVertexArray. You may define which type of data this vertex
        array will store. By setting the flag, every time a new vertex is created,
        the associated data will be allocated and initialized.

        \param  a_options  Data allocation options.
    */
    //--------------------------------------------------------------------------
    cVertexArray(const cVertexArrayOptions& a_options)
    {
        clear();
        m_useNormalData     = a_options.m_useNormalData;
        m_useTexCoordData   = a_options.m_useTexCoordData;
        m_useColorData      = a_options.m_useColorData;
        m_useTangentData    = a_options.m_useTangentData;
        m_useBitangentData  = a_options.m_useBitangentData;
        m_useUserData       = a_options.m_useUserData;
        m_flagPositionData  = false;
        m_flagNormalData    = false;
        m_flagTexCoordData  = false;
        m_flagColorData     = false;
        m_flagTangentData   = false;
        m_flagBitangentData = false;
        m_flagUserData      = false;
        m_flagBufferResize  = true;
        m_positionBuffer    = (GLuint)(-1);
        m_normalBuffer      = (GLuint)(-1);
        m_texCoordBuffer    = (GLuint)(-1);
        m_colorBuffer       = (GLuint)(-1);
        m_tangentBuffer     = (GLuint)(-1);
        m_bitangentBuffer   = (GLuint)(-1);
    }


    //--------------------------------------------------------------------------
    /*!
        Destructor of cVertexArray.
    */
    //--------------------------------------------------------------------------
    ~cVertexArray() {};


    //--------------------------------------------------------------------------
    /*!
        This method create an instance of cVertexArrayPtr. You may define which type of data this vertex
        array will store. By setting the flag, every time a new vertex is created,
        the associated data will be allocated and initialized.

        \param  a_useNormalData     If __true__ then normal data is allocated.
        \param  a_useTexCoordData   If __true__ then texture coordinate data is allocated.
        \param  a_useColorData      If __true__ then vertex color data is allocated.
        \param  a_useTangentData    If __true__ then surface tangent data is allocated.
        \param  a_useBitangentData  If __true__ then surface bitangent data is allocated.
        \param  a_useUserData       If __true__ then user data is allocated.
    */
    //--------------------------------------------------------------------------
    static cVertexArrayPtr create(const bool a_useNormalData,
                                  const bool a_useTexCoordData,
                                  const bool a_useColorData,
                                  const bool a_useTangentData,
                                  const bool a_useBitangentData,
                                  const bool a_useUserData) 
    { 
        const cVertexArrayOptions options(a_useNormalData, 
                                               a_useTexCoordData, 
                                               a_useColorData, 
                                               a_useTangentData, 
                                               a_useBitangentData,
                                               a_useUserData);

        return (std::make_shared<cVertexArray>(options)); 
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    /*!
        This method clears all vertex data.
     */
    //--------------------------------------------------------------------------
    inline void clear()
    {
        m_localPos.clear();
        m_globalPos.clear();
        m_normal.clear();
        m_texCoord.clear();
        m_color.clear();
        m_tangent.clear();
        m_bitangent.clear();
        m_userData.clear();
        m_numVertices = 0;
        m_flagBufferResize = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method creates copy of this vertex array.
     */
    //--------------------------------------------------------------------------
    cVertexArrayPtr copy()
    {
        cVertexArrayPtr vertexArray = cVertexArray::create(m_useNormalData, 
                                                           m_useTexCoordData, 
                                                           m_useColorData,
                                                           m_useTangentData,
                                                           m_useBitangentData,
                                                           m_useUserData);

        // copy arrays
        vertexArray->m_localPos = m_localPos;
        vertexArray->m_globalPos = m_globalPos;
        vertexArray->m_normal = m_normal;
        vertexArray->m_texCoord = m_texCoord;
        vertexArray->m_color = m_color;
        vertexArray->m_tangent = m_tangent;
        vertexArray->m_bitangent = m_bitangent;
        vertexArray->m_userData = m_userData;

        // copy data
        vertexArray->m_useNormalData = m_useNormalData;
        vertexArray->m_useTexCoordData = m_useTexCoordData;
        vertexArray->m_useColorData = m_useColorData;
        vertexArray->m_useTangentData = m_useTangentData;
        vertexArray->m_useBitangentData = m_useBitangentData;
        vertexArray->m_useUserData = m_useUserData;
        vertexArray->m_numVertices = m_numVertices;

        // return new vertex array
        return (vertexArray);
    }


    //--------------------------------------------------------------------------
    /*!
        This method creates a new vertex.

        \return  Index number of the new allocated vertex.
     */
    //--------------------------------------------------------------------------
    inline int newVertex()
    {
        return (newVertices(1));
    }


    //--------------------------------------------------------------------------
    /*!
        This method creates a number of new vertices.

        \param  a_numberOfVertices  Number of vertices to create.

        \return  Index number of the first new allocated vertex.
     */
    //--------------------------------------------------------------------------
    inline int newVertices(unsigned int a_numberOfVertices)
    {
        // sanity check
        if (a_numberOfVertices == 0) { return (-1); }

        // store current index
        int index = m_numVertices;

        // allocate new vertices
        allocateData(a_numberOfVertices, m_useNormalData, m_useTexCoordData, m_useColorData, m_useTangentData, m_useBitangentData, m_useUserData);

        // return index of first allocated vertex
        return (index);
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the local position data for selected vertex.

        \param  a_vertexIndex  Vertex index number.
        \param  a_x            X position value.
        \param  a_y            Y position value.
        \param  a_z            Z position value.
    */
    //--------------------------------------------------------------------------
    inline void setLocalPos(const unsigned int a_vertexIndex,
                            const double& a_x, 
                            const double& a_y, 
                            const double& a_z)
    {
        m_localPos[a_vertexIndex].set(a_x, a_y, a_z);
        m_flagPositionData = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the local position value for selected vertex.

        \param  a_vertexIndex  Vertex index number.
        \param  a_pos          Local position of vertex.
    */
    //--------------------------------------------------------------------------
    inline void setLocalPos(const unsigned int a_vertexIndex,
                            const cVector3d& a_pos)
    {
        m_localPos[a_vertexIndex] = a_pos;
        m_flagPositionData = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method translates a selected vertex by defining a translation 
        offset passed as argument.

        \param  a_vertexIndex  Vertex index number.
        \param  a_translation  Translation offset.
    */
    //--------------------------------------------------------------------------
    inline void translate(const unsigned int a_vertexIndex,
                          const cVector3d& a_translation)
    {
        m_localPos[a_vertexIndex].add(a_translation);
        m_flagPositionData = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the local position for a selected vertex.

        \param  a_vertexIndex  Vertex index number.
        \return Position of vertex.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getLocalPos(const unsigned int a_vertexIndex) const 
    { 
        return (m_localPos[a_vertexIndex]); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the global position of a selected vertex. This value 
        is only correct if the computeGlobalPositions() method has been called 
        previously.

        \param  a_vertexIndex  Vertex index number.
        \return Global position of vertex in world coordinates.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getGlobalPos(const unsigned int a_vertexIndex) const 
    { 
        return (m_globalPos[a_vertexIndex]); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the surface normal vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_normal       Normal vector.
    */
    //--------------------------------------------------------------------------
    inline void setNormal(const unsigned int a_vertexIndex,
                          const cVector3d& a_normal)
    {
        if (m_useNormalData)
        {
            m_normal[a_vertexIndex] = a_normal;
            m_flagNormalData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the surface normal vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_x            X component of normal.
        \param  a_y            Y component of normal.
        \param  a_z            Z component of normal.
    */
    //--------------------------------------------------------------------------
    inline void setNormal(const unsigned int a_vertexIndex,
                          const double& a_x,
                          const double& a_y,
                          const double& a_z)
    {
        if (m_useNormalData)
        {
            m_normal[a_vertexIndex].set(a_x, a_y, a_z);
            m_flagNormalData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the surface normal vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return Normal vector.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getNormal(const unsigned int a_vertexIndex) const
    {
        return (m_normal[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the texture coordinate for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_texCoord     Texture coordinate.
    */
    //--------------------------------------------------------------------------
    inline void setTexCoord(const unsigned int a_vertexIndex,
                            const cVector3d& a_texCoord)
    {
        if (m_useTexCoordData)
        {
            m_texCoord[a_vertexIndex] = a_texCoord;
            m_flagTexCoordData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the texture coordinate for selected vertex by passing its 
        coordinates as parameters.

        \param  a_vertexIndex  Vertex index.
        \param  a_tx           X component of texture coordinate.
        \param  a_ty           Y component of texture coordinate.
        \param  a_tz           Z component of texture coordinate.
    */
    //--------------------------------------------------------------------------
    inline void setTexCoord(const unsigned int a_vertexIndex,
                            const double& a_tx, 
                            const double& a_ty = 0.0,
                            const double& a_tz = 0.0)
    {
        if (m_useTexCoordData)
        {
            m_texCoord[a_vertexIndex].set(a_tx, a_ty,a_tz);
            m_flagTexCoordData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the texture coordinate for a selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return Texture coordinate.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getTexCoord(const unsigned int a_vertexIndex) const 
    { 
        return (m_texCoord[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method set the color for a selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_color        Color.
    */
    //--------------------------------------------------------------------------
    inline void setColor(const unsigned int a_vertexIndex,
                         const cColorf& a_color) 
    { 
        if (m_useColorData)
        {
            m_color[a_vertexIndex] = a_color;
            m_flagColorData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method set the color of a selected vertex by passing as argument 
        the individual color components.

        \param  a_vertexIndex  Vertex index.
        \param  a_red          Red component.
        \param  a_green        Green component.
        \param  a_blue         Blue component.
        \param  a_alpha        Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void setColor(const unsigned int a_vertexIndex,
                         const float& a_red, 
                         const float& a_green,
                         const float& a_blue, 
                         const float& a_alpha = 1.0 )
    {
        if (m_useColorData)
        {
            m_color[a_vertexIndex].set(a_red, a_green, a_blue, a_alpha);
            m_flagColorData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method set color for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_color  Color.
    */
    //--------------------------------------------------------------------------
    inline void setColor(const unsigned int a_vertexIndex,
                         const cColorb& a_color)
    {
        if (m_useColorData)
        {
            m_color[a_vertexIndex] = a_color.getColorf();
            m_flagColorData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the color for a selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return Color of vertex.
    */
    //--------------------------------------------------------------------------
    inline cColorf getColor(const unsigned int a_vertexIndex) const
    {
        return (m_color[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the tangent vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_tangent      Tangent vector.
    */
    //--------------------------------------------------------------------------
    inline void setTangent(const unsigned int a_vertexIndex,
        const cVector3d& a_tangent)
    {
        if (m_useTangentData)
        {
            m_tangent[a_vertexIndex] = a_tangent;
            m_flagTangentData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the tangent vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_x            X component of tangent.
        \param  a_y            Y component of tangent.
        \param  a_z            Z component of tangent.
    */
    //--------------------------------------------------------------------------
    inline void setTangent(const unsigned int a_vertexIndex,
        const double& a_x,
        const double& a_y,
        const double& a_z)
    {
        if (m_useTangentData)
        {
            m_tangent[a_vertexIndex].set(a_x, a_y, a_z);
            m_flagTangentData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the surface tangent for a selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return Tangent vector.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getTangent(const unsigned int a_vertexIndex) const
    {
        return (m_tangent[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the bi-tangent vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_bitangent    Bi-tangent vector.
    */
    //--------------------------------------------------------------------------
    inline void setBitangent(const unsigned int a_vertexIndex,
        const cVector3d& a_bitangent)
    {
        if (m_useBitangentData)
        {
            m_bitangent[a_vertexIndex] = a_bitangent;
            m_flagBitangentData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the bi-tangent vector for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_x            X component of bi-tangent.
        \param  a_y            Y component of bi-tangent.
        \param  a_z            Z component of bi-tangent.
    */
    //--------------------------------------------------------------------------
    inline void setBitangent(const unsigned int a_vertexIndex,
        const double& a_x,
        const double& a_y,
        const double& a_z)
    {
        if (m_useBitangentData)
        {
            m_bitangent[a_vertexIndex].set(a_x, a_y, a_z);
            m_flagBitangentData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method  returns the surface bitangent for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return Bitangent vector.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getBitangent(const unsigned int a_vertexIndex) const
    {
        return (m_bitangent[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the user data field for selected vertex.

        \param  a_vertexIndex  Vertex index.
        \param  a_userData     User data.
    */
    //--------------------------------------------------------------------------
    inline void setUserData(const unsigned int a_vertexIndex,
                           const int a_userData) 
    { 
        if (m_useUserData)
        {
            m_userData[a_vertexIndex] = a_userData;
            m_flagUserData = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the user data value for a selected vertex.

        \param  a_vertexIndex  Vertex index.
        \return User data.
    */
    //--------------------------------------------------------------------------
    inline int getUserData(const unsigned int a_vertexIndex) const
    {
        return (m_userData[a_vertexIndex]);
    }


    //--------------------------------------------------------------------------
    /*!
        This method computes the global position of vertex given the global 
        position and global rotation matrix of the parent object.

        \param  a_vertexIndex  Index number of vertex.
        \param  a_globalPos    Global position vector of parent.
        \param  a_globalRot    Global rotation matrix of parent.
    */
    //--------------------------------------------------------------------------
    inline void computeGlobalPosition(const unsigned int a_vertexIndex,
                                      const cVector3d& a_globalPos, 
                                      const cMatrix3d& a_globalRot)
    {
        a_globalRot.mulr(m_localPos[a_vertexIndex], m_globalPos[a_vertexIndex]);
        m_globalPos[a_vertexIndex].add(a_globalPos);
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the number of vertices allocated in this array.

        \return  Number of vertices.
    */
    //--------------------------------------------------------------------------
    inline unsigned int getNumElements() const
    { 
        return (m_numVertices);
    }


    //--------------------------------------------------------------------------
    /*!
        This method checks if normal data is allocated for each vertex in this
        array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseNormalData() const 
    { 
        return (m_useNormalData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method check if texture coordinate data is allocated for each 
        vertex in this array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseTexCoordData() const 
    { 
        return (m_useTexCoordData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method checks if color data is allocated for each vertex in this 
        array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseColorData() const 
    { 
        return (m_useColorData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method checks if tangent data is allocated for each vertex in this 
        array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseTangentData() const 
    { 
        return (m_useTangentData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method checks if bitangent data is allocated for each vertex in 
        this array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseBitangentData() const 
    { 
        return (m_useBitangentData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method checks if user data is allocated for each vertex in this 
        array.

        \return __true__ if data is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getUseUserData() const 
    { 
        return (m_useUserData); 
    }


    //--------------------------------------------------------------------------
    /*!
        This method allocates or updates all OpenGL buffers.
    */
    //--------------------------------------------------------------------------
    inline void renderInitialize()
    { 
#ifdef C_USE_OPENGL
        // create buffers first time
        if (m_positionBuffer == (GLuint)(-1))
        {
            glGenBuffers(1, &m_positionBuffer);
        }
        
        if ((m_normalBuffer == (GLuint)(-1)) && (m_useNormalData))
        {
            glGenBuffers(1, &m_normalBuffer);
        }
        
        if ((m_texCoordBuffer == (GLuint)(-1)) && (m_useTexCoordData))
        {
            glGenBuffers(1, &m_texCoordBuffer);
        }
        
        if ((m_colorBuffer == (GLuint)(-1)) && (m_useColorData))
        {
            glGenBuffers(1, &m_colorBuffer);
        }
        
        if ((m_tangentBuffer == (GLuint)(-1)) && (m_useTangentData))
        {
            glGenBuffers(1, &m_tangentBuffer);
        }
        
        if ((m_bitangentBuffer == (GLuint)(-1)) && (m_useBitangentData))
        {
            glGenBuffers(1, &m_bitangentBuffer);
        }

        // resize buffers
        if (m_flagBufferResize)
        {
            if (true)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_positionBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cVector3d), &(m_localPos[0]), GL_STATIC_DRAW);
            }

            if (m_useNormalData)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_normalBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cVector3d), &(m_normal[0]), GL_STATIC_DRAW);
            }

            if (m_useTexCoordData)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_texCoordBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cVector3d), &(m_texCoord[0]), GL_STATIC_DRAW);
            }

            if (m_useColorData)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cColorf), &(m_color[0]), GL_STATIC_DRAW);
            }

            if (m_useTangentData)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_tangentBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cVector3d), &(m_tangent[0]), GL_STATIC_DRAW);
            }
        
            if (m_useBitangentData)
            {
                glBindBuffer(GL_ARRAY_BUFFER, m_bitangentBuffer);
                glBufferData(GL_ARRAY_BUFFER, m_numVertices * sizeof(cVector3d), &(m_bitangent[0]), GL_STATIC_DRAW);
            }

            m_flagBufferResize = false;
        }

        // update buffers if needed
        if (m_flagPositionData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_positionBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cVector3d), &(m_localPos[0]));
            m_flagPositionData = false;
        }
        if (m_flagNormalData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_normalBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cVector3d), &(m_normal[0]));
            m_flagNormalData = false;
        }
        if (m_flagTexCoordData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_texCoordBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cVector3d), &(m_texCoord[0]));
            m_flagTexCoordData = false;
        }
        if (m_flagColorData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cColorf), &(m_color[0]));
            m_flagColorData = false;
        }
        if (m_flagTangentData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_tangentBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cVector3d), &(m_tangent[0]));
            m_flagTangentData = false;
        }
        if (m_flagBitangentData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_bitangentBuffer);
            glBufferSubData(GL_ARRAY_BUFFER, 0, m_numVertices * sizeof(cVector3d), &(m_bitangent[0]));
            m_flagBitangentData = false;
        }

        // bind buffers and set client state
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_positionBuffer);
            glEnableVertexAttribArray(C_VB_POSITION);
            glVertexAttribPointer(C_VB_POSITION, 3, GL_DOUBLE, GL_FALSE, 0, 0);
            glVertexPointer(3, GL_DOUBLE, 0, 0);
        }
        
        if (m_useNormalData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_normalBuffer);
            glEnableVertexAttribArray(C_VB_NORMAL);
            glVertexAttribPointer(C_VB_NORMAL, 3, GL_DOUBLE, GL_FALSE, 0, 0);
        }
        else
        {
            glDisableVertexAttribArray(C_VB_NORMAL);
        }

        if (m_useTexCoordData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_texCoordBuffer);
            glEnableVertexAttribArray(C_VB_TEXCOORD);
            glVertexAttribPointer(C_VB_TEXCOORD, 3, GL_DOUBLE, GL_FALSE, 0, 0);
        }
        else
        {
            glDisableVertexAttribArray(C_VB_TEXCOORD);
        }

        if (m_useColorData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
            glEnableVertexAttribArray(C_VB_COLOR);
            glVertexAttribPointer(C_VB_COLOR, 4, GL_FLOAT, GL_FALSE, sizeof(cColorf), 0);
        }
        else
        {
            glDisableVertexAttribArray(C_VB_COLOR);
        }

        if (m_useTangentData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_tangentBuffer);
            glEnableVertexAttribArray(C_VB_TANGENT);
            glVertexAttribPointer(C_VB_TANGENT, 3, GL_DOUBLE, GL_FALSE, 0, 0);
        }
        else
        {
            glDisableVertexAttribArray(C_VB_TANGENT);
        }

        if (m_useBitangentData)
        {
            glBindBuffer(GL_ARRAY_BUFFER, m_bitangentBuffer);
            glEnableVertexAttribArray(C_VB_BITANGENT);
            glVertexAttribPointer(C_VB_BITANGENT, 3, GL_DOUBLE, GL_FALSE, 0, 0);
        }
        else
        {
            glDisableVertexAttribArray(C_VB_BITANGENT);
        }

        glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
    }


    //--------------------------------------------------------------------------
    /*!
        This method finalizes rendering by disabling all OpenGL buffers.
    */
    //--------------------------------------------------------------------------
    inline void renderFinalize()
    { 
#ifdef C_USE_OPENGL
        glDisableVertexAttribArray(C_VB_INDEX_BUFFER);
        glDisableVertexAttribArray(C_VB_POSITION);
        glDisableVertexAttribArray(C_VB_NORMAL);
        glDisableVertexAttribArray(C_VB_TEXCOORD);
        glDisableVertexAttribArray(C_VB_COLOR);
        glDisableVertexAttribArray(C_VB_TANGENT);
        glDisableVertexAttribArray(C_VB_BITANGENT);
#endif
    }

    //--------------------------------------------------------------------------
    /*!
        This method allocate data for vertex array.

        \param  a_numberOfVertices  Number of new vertices to be allocated
        \param  a_useNormalData     If __true__ then normal data is allocated.
        \param  a_useTexCoordData   If__true__ then texture coordinate data is allocated.
        \param  a_useColorData      If __true__ then vertex color data is allocated.
        \param  a_useTangentData    If __true__ then surface tangent data is allocated.
        \param  a_useBitangentData  If __true__ then surface bitangent data is allocated.
        \param  a_useUserData       If __true__ then user data data is allocated.
     */
    //--------------------------------------------------------------------------
    inline void allocateData(const int a_numberOfVertices,
                             const bool a_useNormalData,
                             const bool a_useTexCoordData,
                             const bool a_useColorData,
                             const bool a_useTangentData,
                             const bool a_useBitangentData,
                             const bool a_useUserData)
    {
        // increment counter
        m_numVertices = m_numVertices + a_numberOfVertices;

        // update position data allocation
        cVector3d pos(0.0, 0.0, 0.0);
        m_localPos.resize(m_numVertices, pos);
        m_globalPos.resize(m_numVertices, pos);

        // update normal data allocation
        m_useNormalData = a_useNormalData;
        if (m_useNormalData)
        {
            cVector3d normal(1.0, 0.0, 0.0);
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_normal.resize(m_numVertices, normal);
            }
            else
            {
                m_normal.push_back(normal);
            }
            m_flagNormalData = true;
        }
        else
        {
            m_normal.clear();
        }

        // texture coordinate data allocation
        m_useTexCoordData = a_useTexCoordData;
        if (m_useTexCoordData)
        {
            cVector3d texCoord(0.0, 0.0, 0.0);
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_texCoord.resize(m_numVertices, texCoord);
            }
            else
            {
                m_texCoord.push_back(texCoord);
            }
            m_flagTexCoordData = true;
        }
        else
        {
            m_texCoord.clear();
        }

        // update color data allocation
        m_useColorData = a_useColorData;
        if (m_useColorData)
        {
            cColorf color(0.0, 0.0, 0.0, 1.0);
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_color.resize(m_numVertices, color);
            }
            else
            {
                m_color.push_back(color);
            }
            m_flagColorData = true;
        }
        else
        {
            m_color.clear();
        }

        // update tangent data allocation
        m_useTangentData = a_useTangentData;
        if (m_useTangentData)
        {
            cVector3d tangent(1.0, 0.0, 0.0);
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_tangent.resize(m_numVertices, tangent);
            }
            else
            {
                m_tangent.push_back(tangent);
            }
            m_flagTangentData = true;
        }
        else
        {
            m_tangent.clear();
        }

        // update bitangent data allocation
        m_useBitangentData = a_useBitangentData;
        if (m_useBitangentData)
        {
            cVector3d bitangent(0.0, 1.0, 0.0);
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_bitangent.resize(m_numVertices, bitangent);
            }
            else
            {
                m_bitangent.push_back(bitangent);
            }
            m_flagBitangentData = true;
        }
        else
        {
            m_bitangent.clear();
        }

        // update user data allocation
        m_useUserData = a_useUserData;
        if (m_useUserData)
        {
            int data = 0;
            if ((a_numberOfVertices > 1) || (a_numberOfVertices == 0))
            {
                m_userData.resize(m_numVertices, data);
            }
            else
            {
                m_userData.push_back(data);
            }
            m_flagUserData = true;
        }
        else
        {
            m_userData.clear();
        }

        // update buffer size
        m_flagBufferResize = true;
    }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Local position of vertices.
    std::vector<cVector3d> m_localPos;

    //! Global position of vertices in world coordinates.
    std::vector<cVector3d> m_globalPos;

    //! Surface normal of vertices.
    std::vector<cVector3d> m_normal;

    //! Texture coordinate (U,V,W) of vertices.
    std::vector<cVector3d> m_texCoord;

    //! Color of vertices.
    std::vector<cColorf> m_color;

    //! Surface tangent of vertices.
    std::vector<cVector3d> m_tangent;

    //! Surface bitangent of vertices.
    std::vector<cVector3d> m_bitangent;

    //! User data of vertices.
    std::vector<int> m_userData;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Number of vertices.
    unsigned int m_numVertices;

    //! If __true__ then normal data will be allocated for each new vertex.
    bool m_useNormalData;

    //! If __true__ then texture coordinate data will be allocated for each new vertex.
    bool m_useTexCoordData;

    //! If __true__ then vertex color data will be allocated for each new vertex.
    bool m_useColorData;

    //! If __true__ then surface tangent data will be allocated for each new vertex.
    bool m_useTangentData;

    //! If __true__ then surface bitangent data will be allocated for each new vertex.
    bool m_useBitangentData;

    //! If __true__ then surface bitangent data will be allocated for each new vertex.
    bool m_useUserData;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! If __true__ then position data has been modified.
    bool m_flagPositionData;

    //! If __true__ then normal data has been modified.
    bool m_flagNormalData;

    //! If __true__ then texture coordinate data has been modified.
    bool m_flagTexCoordData;

    //! If __true__ then vertex color data has been modified.
    bool m_flagColorData;

    //! If __true__ then surface tangent data has been modified.
    bool m_flagTangentData;

    //! If __true__ then surface bitangent data has been modified.
    bool m_flagBitangentData;

    //! If __true__ then user data has been modified.
    bool m_flagUserData;

    //! If true, then data buffer need to be updated in size.
    bool m_flagBufferResize;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS: (OPENGL)
    //--------------------------------------------------------------------------

public:

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_positionBuffer;

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_normalBuffer;

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_texCoordBuffer;

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_colorBuffer;

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_tangentBuffer;

    //! OpenGL Buffer for storing triangle indices.
    GLuint m_bitangentBuffer;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
