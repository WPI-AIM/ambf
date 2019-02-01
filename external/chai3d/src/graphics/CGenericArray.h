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
#ifndef CGenericArrayH
#define CGenericArrayH
//------------------------------------------------------------------------------
#include "math/CGeometry.h"
#include "graphics/CVertexArray.h"
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericObject;
class cCollisionRecorder;
struct cCollisionSettings;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file   CGenericArray.h

    \brief
    Implements an abstract class for describing elements composed of vertices.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cGenericArray;
typedef std::shared_ptr<cGenericArray> cGenericArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cGenericArray
    \ingroup    graphics

    \brief
    This class implements an abstract class for describing geometrical elements
    composed of vertices.

    \details
    cGenericArray implements an abstract class for describing geometrical 
    elements composed of vertices.\n
    Example of elements include points, segments, or triangles. Each element
    shall be composed on one or more vertices (point=1, segment=2, triangle=3).
*/
//==============================================================================
class cGenericArray
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cGenericArray.

        \param  a_vertexArray  Array of vertices used to describe the elements.
    */
    //--------------------------------------------------------------------------
    cGenericArray(cVertexArrayPtr a_vertexArray)
    {
        // clear all elements
        clear();

        // initialize OpenGL buffer ID 
        m_elementBuffer         = 0;
    }


    //--------------------------------------------------------------------------
    /*!
        Destructor of cGenericArray.
    */
    //--------------------------------------------------------------------------
    ~cGenericArray(){}


    //! This method clear all elements of array.
    virtual void clear()
    {
        m_allocated.clear();
        m_indices.clear();
        m_freeElements.clear();
        m_flagMarkForUpdate     = true;
        m_flagMarkForResize     = true;
    }


    //! Shared cGenericArray allocator.
    static cGenericArrayPtr create(cVertexArrayPtr a_vertexArray) { return (std::make_shared<cGenericArray>(a_vertexArray)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of allocated elements.
    virtual unsigned int getNumElements() { return (unsigned int)(m_allocated.size()); }

    //! This method returns the number of vertices per element.
    virtual unsigned int getNumVerticesPerElement() { return (1); }

    //! This method removes non used elements. This compresses the array.
    void compress();


    //--------------------------------------------------------------------------
    /*!
        Is the element passed by argument currently allocated.

        \return __true__ if the element is allocated, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool getAllocated(const unsigned int a_index) const
    {
        return (m_allocated[a_index]);
    };


    //--------------------------------------------------------------------------
    /*!
         This method returns the index number of a vertex for a selected element. 
         The element is identified by passing its index number as argument. 
         The vertex is selected by passing its vertex number. For a triangle. 
         this number shall be 0,1, or 2.

        \param  a_elementIndex  Index number of the element.
        \param  a_vertexNumber  Vertex number.

        \return Index number to the requested vertex.
    */
    //--------------------------------------------------------------------------
    virtual unsigned int getVertexIndex(const unsigned int a_elementIndex,
                                        const unsigned int a_vertexNumber) const
    {
        return (0);
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the texture coordinate at the nearest point on the 
        element from an input location.

        \param  a_elementIndex  Index number of selected element.
        \param  a_localPos      Location of point on element.

        \return Interpolated texture coordinate at desired location.
    */
    //--------------------------------------------------------------------------
    cVector3d getTexCoordAtPosition(const unsigned int a_elementIndex, 
                                    const cVector3d& a_localPos)
    {
        cVector3d texCoord(0.0, 0.0, 0.0);
        return (texCoord);
    }


    //! This method render the OpenGL vertex buffer object.
    virtual void render()
    { 
    }

    //! This method checks if the given line segment intersects a selected element from this array.
    virtual bool computeCollision(const unsigned int a_elementIndex,
                                  cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings) const { return (false); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Vertex array that contains all vertices used to describe the elements of this array.
    cVertexArrayPtr m_vertices;

    //! Element indices to vertices.
    std::vector<unsigned int> m_indices;

    //! Element allocation flags.
    std::vector<bool> m_allocated;

    //! If __true__ then element data has been modified.
    bool m_flagMarkForUpdate;

    //! If __true__ then element array size has changed.
    bool m_flagMarkForResize;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS: (OPENGL)
    //--------------------------------------------------------------------------

public:

    //! OpenGL Buffer for storing elements.
    GLuint m_elementBuffer;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! List of free elements.
    std::list<unsigned int> m_freeElements;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
