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
#ifndef CPointArrayH
#define CPointArrayH
//------------------------------------------------------------------------------
#include "graphics/CGenericArray.h"
//------------------------------------------------------------------------------

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
    \file   CPointArray.h
    
    \brief
    Implements arrays of 3D points.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cPointArray;
typedef std::shared_ptr<cPointArray> cPointArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cPointArray
    \ingroup    graphics

    \brief
    This class implements an array of 3D points.

    \details
    cPointArray defines an array of points, each composed of a single vertex.\n

    A point array is created by first passing a reference to an array of vertices 
    (\ref cVertexArray) from which the points are composed. \n
    
    For graphic rendering purposes and memory efficiency, points are stored 
    in large arrays. The number of allocated points in an array can be
    retrieved by calling \ref getNumElements().\n

    New points can be added or removed by calling \ref newPoint() and 
    \ref removePoint(). respectively.\n
    
    When a point is removed from the array, its vertex index is set to zero 
    and the point index number is added to a list of free points for future 
    allocation.

    To avoid accumulating large numbers of free points, it is possible to 
    compress the array by calling \ref compress(). This method removes all
    non used points from memory.\n
*/
//==============================================================================
class cPointArray : public cGenericArray
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cPointArray.

        \param  a_vertexArray  Array of vertices used to describe the points.
    */
    //--------------------------------------------------------------------------
    cPointArray(cVertexArrayPtr a_vertexArray) : cGenericArray(a_vertexArray)
    {
        // clear all points
        clear();

        // store pointer to vertex array
        m_vertices = a_vertexArray;

        // initialize OpenGL buffer ID
        m_elementBuffer         = 0;
    }


    //--------------------------------------------------------------------------
    /*!
        Destructor of cPointArray.
    */
    //--------------------------------------------------------------------------
    ~cPointArray(){}


    //! This method clears all points from the array.
    void clear()
    {
        m_allocated.clear();
        m_indices.clear();
        m_freeElements.clear();
        m_flagMarkForResize     = true;
    }


    //! Shared cPointArray allocator.
    static cPointArrayPtr create(cVertexArrayPtr a_vertexArray) { return (std::make_shared<cPointArray>(a_vertexArray)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of vertices that compose a point.
    virtual unsigned int getNumVerticesPerElement() { return (1); }

    //! This method copies point data and return new point array.
    cPointArrayPtr copy();

    //! This method compresses the array by removing non used points.
    void compress();


    //--------------------------------------------------------------------------
    /*!
        This method creates a new point.

        \param  a_vertexIndex0  index of vertex 0.

        \return Index number of new point.
    */
    //--------------------------------------------------------------------------
    int newPoint(const unsigned int a_vertexIndex0)
    {
        int index;

        // check if there is an available slot on the free point list
        if (m_freeElements.size() > 0)
        {
            index = m_freeElements.front();
            m_allocated[index] = true;
            m_freeElements.pop_front();
            setVertex(index, a_vertexIndex0);
        }
        else
        {
            // store new indices
            m_indices.push_back(a_vertexIndex0);
            m_allocated.push_back(true);

            m_flagMarkForResize = true;

            // store index of new point
            index = (int)(m_allocated.size())-1;
        }

        // mark for update
        m_flagMarkForUpdate = true;

        // return index number to new point
        return (index);
    }


    //--------------------------------------------------------------------------
    /*!
        This method deallocates a selected point from the array.
        Its vertex index is set to zero, and the point is added to the free list
        for future allocation.\n

        \param  a_pointIndex  Index number of selected point.
    */
    //--------------------------------------------------------------------------
    void removePoint(const unsigned int a_pointIndex)
    {
        // sanity check
        if (m_allocated[a_pointIndex])
        {
            // disable point
            m_allocated[a_pointIndex] = false;

            // reset index to vertex
            setVertex(a_pointIndex, 0);

            // add point to free list
            m_freeElements.push_back(a_pointIndex);

            // mark for update
            m_flagMarkForUpdate = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the vertex of a selected point by passing its index 
        number.

        \param  a_pointIndex   Index of selected point.
        \param  a_vertexIndex  Index of vertex.
    */
    //--------------------------------------------------------------------------
    inline void setVertex(const unsigned int a_pointIndex,
                           const unsigned int a_vertexIndex)
    {
        m_indices[a_pointIndex] = a_vertexIndex;

        // mark for update
        m_flagMarkForUpdate = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the index number of the vertex that describes a selected point.

        \param  a_pointIndex  Index of point.

        \return Index to vertex.
    */
    //--------------------------------------------------------------------------
    inline unsigned int getVertexIndex0(const unsigned int a_pointIndex) const
    {
        return (m_indices[a_pointIndex]);
    };


    //--------------------------------------------------------------------------
    /*!
         This method reads the index number of the vertex of a selected point.
         In the case of a point \p a_vertexNumber only takes value 0, as a point 
         is only composed of a single vertex.

        \param  a_elementIndex  Index number of selected point.
        \param  a_vertexNumber  Vertex number.

        \return Index number of the requested vertex.
    */
    //--------------------------------------------------------------------------
    virtual unsigned int getVertexIndex(const unsigned int a_elementIndex,
                                        const unsigned int a_vertexNumber) const
    {
        switch (a_vertexNumber)
        {
            case 0 : return (m_indices[a_elementIndex]);
            default: return (0);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the texture coordinate at the point. For points, 
        a_localPos is meaningless and ignore. This value is used however for 
        segments or triangles.

        \param  a_pointIndex  Index number of selected point.
        \param  a_localPos    Value ignored.

        \return Interpolated texture coordinate at desired location.
    */
    //--------------------------------------------------------------------------
    cVector3d getTexCoordAtPosition(const unsigned int a_pointIndex, 
                                    const cVector3d& a_localPos)
    {
        cVector3d texCoord;

        if (m_vertices->getUseTexCoordData())
        {
            texCoord = m_vertices->getTexCoord(getVertexIndex0(a_pointIndex));
        }
        else
        {
            texCoord.set(0.0, 0.0, 0.0);
        }

        // return result
        return (texCoord);
    }


    //--------------------------------------------------------------------------
    /*!
        This method renders the OpenGL vertex buffer object.
    */
    //--------------------------------------------------------------------------
    inline void render()
    { 
#ifdef C_USE_OPENGL
        unsigned int numPoints = getNumElements();

        // allocate object and buffers if needed
        if (m_elementBuffer == 0)
        {
            glGenBuffers(1, &m_elementBuffer);
        }

        // bind buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBuffer);

        // update buffer size if needed
        if (m_flagMarkForResize)
        {
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, numPoints * sizeof(unsigned int), &(m_indices[0]), GL_STATIC_DRAW);
            m_flagMarkForResize = true;
        }

        // update data if needed
        if (m_flagMarkForUpdate)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBuffer);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, numPoints * sizeof(unsigned int), &(m_indices[0]));

            m_flagMarkForUpdate = false;
        }

        // render object
        m_vertices->renderInitialize();
        
        glEnableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBuffer);
        glDrawElements(GL_POINTS, numPoints, GL_UNSIGNED_INT, (void*)0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        m_vertices->renderFinalize();
#endif
    }

    //! This method checks if a given line segment intersects a selected point from this array.
    virtual bool computeCollision(const unsigned int a_elementIndex,
                                  cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings) const;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------


