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
#ifndef CSegmentArrayH
#define CSegmentArrayH
//------------------------------------------------------------------------------
#include "graphics/CGenericArray.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file   CSegmentArray.h
    
    \brief
    Implement arrays of 3D segments.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cSegmentArray;
typedef std::shared_ptr<cSegmentArray> cSegmentArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cSegmentArray
    \ingroup    graphics

    \brief
    This class implements an array of 3D segments.

    \details
    This class defines an array of segments each composed of two vertices.\n

    A segment array is created by first passing a reference to an array of vertices 
    (\ref cVertexArray) from which the segments are composed. \n

    Every segment is described by a set of two vertex indices. The indices point 
    to the vertices that describe the segment. \n

    For graphic rendering purposes and memory efficiency, segments are stored 
    in large arrays. The number of allocated segments in an array can be
    retrieved by call \ref getNumElements().\n
    
    New segments can be added or removed by calling \ref newSegment() and 
    \ref removeSegment(). respectively.\n
    
    When a segment is removed from the array, the vertex indices are set to zero 
    and the segment index number is added to a list of free segments for future 
    allocation. Segments are therefore never removed from memory, but become a 
    point until they are converted to an active segment again.\n

    To avoid accumulating large numbers of free segments, it is possible to 
    compress the array by calling \ref compress(). This method removes all
    non used segments from memory.\n
*/
//==============================================================================
class cSegmentArray : public cGenericArray
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cSegmentArray.

        \param  a_vertexArray  Array of vertices used to describe the segments.
    */
    //--------------------------------------------------------------------------
    cSegmentArray(cVertexArrayPtr a_vertexArray) : cGenericArray(a_vertexArray)
    {
        // clear all segments
        clear();

        // store pointer to vertex array
        m_vertices = a_vertexArray;

        // initialize OpenGL buffer ID 
        m_elementBuffer         = 0;
    }


    //--------------------------------------------------------------------------
    /*!
        Destructor of cSegmentArray.
    */
    //--------------------------------------------------------------------------
    ~cSegmentArray(){}


    //! This method clears all segments from array.
    void clear()
    {
        m_allocated.clear();
        m_indices.clear();
        m_freeElements.clear();
        m_flagMarkForUpdate     = true;
        m_flagMarkForResize     = true;
    }


    //! Shared cSegmentArray allocator.
    static cSegmentArrayPtr create(cVertexArrayPtr a_vertexArray) { return (std::make_shared<cSegmentArray>(a_vertexArray)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method returns the number of vertices that compose a segment.
    virtual unsigned int getNumVerticesPerElement() { return (2); }

    //! This method create a copy of all segment data and returns a new segment array.
    cSegmentArrayPtr copy();

    //! This method compresses the array by removing non used segments.
    void compress();


    //--------------------------------------------------------------------------
    /*!
        This method creates a new segment.

        \param  a_vertexIndex0  index of vertex 0.
        \param  a_vertexIndex1  index of vertex 1.

        \return Index number of new segment.
    */
    //--------------------------------------------------------------------------
    int newSegment(const unsigned int a_vertexIndex0,
                   const unsigned int a_vertexIndex1)
    {
        int index;

        // check if there is an available slot on the free segment list
        if (m_freeElements.size() > 0)
        {
            index = m_freeElements.front();
            m_allocated[index] = true;
            m_freeElements.pop_front();
            setVertices(index, a_vertexIndex0, a_vertexIndex1);
        }
        else
        {
            // store new indices
            m_indices.push_back(a_vertexIndex0);
            m_indices.push_back(a_vertexIndex1);
            m_allocated.push_back(true);

            m_flagMarkForResize = true;

            // store index of new segment
            index = (int)(m_allocated.size())-1;
        }

        // mark for update
        m_flagMarkForUpdate = true;

        // return index to new segment
        return (index);
    }


    //--------------------------------------------------------------------------
    /*!
        This method deallocates a selected segment from the array. The two vertex 
        indices of the segment are set to zero, and the segment is added to the 
        free list for future allocation.\n

        \param  a_segmentIndex  Index number of selected segment.
    */
    //--------------------------------------------------------------------------
    void removeSegment(const unsigned int a_segmentIndex)
    {
        // sanity check
        if (m_allocated[a_segmentIndex])
        {
            // disable segment
            m_allocated[a_segmentIndex] = false;

            // reset indices to vertices
            setVertices(a_segmentIndex, 0, 0);

            // add segment to free list
            m_freeElements.push_back(a_segmentIndex);

            // mark for update
            m_flagMarkForUpdate = true;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method sets the vertices of a selected segment by passing their
        index numbers.

        \param  a_segmentIndex  Index of selected segment.
        \param  a_vertexIndex0  Index of vertex 0.
        \param  a_vertexIndex1  Index of vertex 1.
    */
    //--------------------------------------------------------------------------
    inline void setVertices(const unsigned int a_segmentIndex,
                            const unsigned int a_vertexIndex0,
                            const unsigned int a_vertexIndex1)
    {
        m_indices[2*a_segmentIndex+0] = a_vertexIndex0;
        m_indices[2*a_segmentIndex+1] = a_vertexIndex1;

        // mark for update
        m_flagMarkForUpdate = true;
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the index of vertex 0 of a selected segment.

        \param  a_segmentIndex  Index of segment.

        \return Index to vertex 0.
    */
    //--------------------------------------------------------------------------
    inline unsigned int getVertexIndex0(const unsigned int a_segmentIndex) const
    {
        return (m_indices[2*a_segmentIndex+0]);
    };


    //--------------------------------------------------------------------------
    /*!
        This method returns index of vertex 1 of a selected segment.

        \param  a_segmentIndex  Index of selected segment.

        \return Index to vertex 1.
    */
    //--------------------------------------------------------------------------
    inline unsigned int getVertexIndex1(const unsigned int a_segmentIndex) const
    {
        return (m_indices[2*a_segmentIndex+1]);
    };


    //--------------------------------------------------------------------------
    /*!
         This method returns the index number of a selected vertex for a selected 
         segment. In the case of a segment a_vertexNumber takes either 0 or 1.

        \param  a_elementIndex  Index number of selected segment.
        \param  a_vertexNumber  Vertex number.

        \return Index number of the requested vertex.
    */
    //--------------------------------------------------------------------------
    virtual unsigned int getVertexIndex(const unsigned int a_elementIndex,
                                        const unsigned int a_vertexNumber) const
    {
        switch (a_vertexNumber)
        {
            case 0: return (m_indices[2*a_elementIndex+0]);
            case 1: return (m_indices[2*a_elementIndex+1]);
            default: return (0);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        This method returns the texture coordinate at the nearest point on the 
        segment from a location passed as argument.

        \param  a_segmentIndex  Index number of selected segment.
        \param  a_localPos      Local point along the segment.

        \return Interpolated texture coordinate at desired location.
    */
    //--------------------------------------------------------------------------
    cVector3d getTexCoordAtPosition(const unsigned int a_segmentIndex, 
                                    const cVector3d& a_localPos)
    {
        // get vertices of contact segments
        cVector3d vertex0 = m_vertices->getLocalPos(getVertexIndex0(a_segmentIndex));
        cVector3d vertex1 = m_vertices->getLocalPos(getVertexIndex1(a_segmentIndex));

        // project desired point on segment
        cVector3d point = cProjectPointOnSegment(a_localPos, vertex0, vertex1);

        // compute weights based on position of contact point
        double c0, c1;

        double distance01 = cDistance(vertex0, vertex1);
        if (distance01 < C_SMALL)
        {
            c0 = 1.0;
            c1 = 0.0;
        }
        else
        {
            double distanceP0 = cDistance(vertex0, point);
            c1 = distanceP0 / distance01;
            c0 = 1.0 - c1;
        }

        cVector3d texCoord;

        if (m_vertices->getUseTexCoordData())
        {
            // retrieve the texture coordinate for each segment vertex
            cVector3d texCoord0 = m_vertices->getTexCoord(getVertexIndex0(a_segmentIndex));
            cVector3d texCoord1 = m_vertices->getTexCoord(getVertexIndex1(a_segmentIndex));

            // compute the exact texture coordinate at the contact point
            texCoord = cAdd( cMul(c0, texCoord0), cMul(c1, texCoord1));
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
        unsigned int numSegments = getNumElements();

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
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * numSegments * sizeof(unsigned int), &(m_indices[0]), GL_STATIC_DRAW);
            m_flagMarkForResize = true;
        }

        // update data if needed
        if (m_flagMarkForUpdate)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBuffer);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, 2 * numSegments * sizeof(unsigned int), &(m_indices[0]));

            m_flagMarkForUpdate = false;
        }

        // render object
        m_vertices->renderInitialize();
        
        glEnableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBuffer);
        glDrawElements( GL_LINES, 2 * numSegments, GL_UNSIGNED_INT, (void*)0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

         m_vertices->renderFinalize();
#endif
    }

    //! This method checks if the given line segment intersects a selected segment from this array.
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


