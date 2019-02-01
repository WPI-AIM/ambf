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
#ifndef CMultiSegmentH
#define CMultiSegmentH
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "graphics/CColor.h"
//------------------------------------------------------------------------------
#include <vector>
#include <list>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cWorld;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMultiSegment.h

    \brief
    Implements a collection of 3D segments.
*/
//==============================================================================

//------------------------------------------------------------------------------
//! Array of vertices.
class cVertexArray;
typedef std::shared_ptr<cVertexArray> cVertexArrayPtr;

//! Array of segments.
class cSegmentArray;
typedef std::shared_ptr<cSegmentArray> cSegmentArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!    
    \class      cMultiSegment
    \ingroup    world

    \brief
    This class implements a collection 3D segments.

    \details
    This class implements a collection 3D segments.
*/
//==============================================================================
class cMultiSegment : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cMultiSegment.
    cMultiSegment();

    //! Destructor of cMultiSegment.
    virtual ~cMultiSegment();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cMultiSegment* copy(const bool a_duplicateMaterialData = false,
                        const bool a_duplicateTextureData = false, 
                        const bool a_duplicateMeshData = false,
                        const bool a_buildCollisionDetector = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - VERTICES
    //--------------------------------------------------------------------------

public:


    //! This method creates a new vertex and adds it to the vertex list.
    unsigned int newVertex(const double a_x = 0.0, 
                           const double a_y = 0.0, 
                           const double a_z = 0.0);

    //! This method creates a new vertex and adds it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cColorf& a_color = cColorf(0,0,0,1));

    //! This method returns the number of stored vertices.
    inline unsigned int getNumVertices() const { return (unsigned int)(m_vertices->getNumElements()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - SEGMENTS
    //--------------------------------------------------------------------------

public:

    //! This method creates a new segment by passing two vertex indices.
    unsigned int newSegment(const unsigned int a_indexVertex0,
                            const unsigned int a_indexVertex1);

    //! This method creates a new segment by passing the position of its end points and color.
    unsigned int newSegment(const cVector3d& a_vertex0 = cVector3d(0,0,0),
                            const cVector3d& a_vertex1 = cVector3d(0,0,0),
                            const cColorf& a_colorVertex0 = cColorf(0,0,0,1),
                            const cColorf& a_colorVertex1 = cColorf(0,0,0,1));

    //! This method removes a selected segment.
    bool removeSegment(const unsigned int a_index);

    //! This method returns the number of stored segments.
    unsigned int getNumSegments();

    //! This method clears all segments and vertices.
    void clear();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method sets the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
        const bool a_applyToVertices = false,
        const bool a_applyToTextures = false,
        const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - DISPLAY LISTS:
    //-----------------------------------------------------------------------

public:

    //! This method invalidates any existing display lists and marks the mesh for update.
    virtual void markForUpdate(const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - LINE GRAPHIC PROPERTIES
    //--------------------------------------------------------------------------

public:

    //! This method sets the color of all segments.
    void setLineColor(const cColorf& a_color);

    //! This method sets the line width of all segments.
    inline void setLineWidth(const double a_lineWidth) { m_lineWidth = fabs(a_lineWidth); }

    //! This method returns the line width.
    inline double getLineWidth() const { return (m_lineWidth); }

    //! This method specifies the line stipple pattern.
    void setLineStipple(const GLint a_stippleFactor, const GLushort a_stipplePattern);

    //! This method returns the line stipple factor value.
    GLint getLineStippleFactor() { return (m_stippleFactor); }

    //! This method returns the line stipple pattern value.
    GLushort  getLineStipplePattern() { return (m_stipplePattern); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //--------------------------------------------------------------------------

public:

    //! This method builds a brute force collision detector for this mesh.
    virtual void createBruteForceCollisionDetector();

    //! This method builds an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(const double a_radius);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GEOMETRY:
    //--------------------------------------------------------------------------

public:

    //! This method scales this object by using different scale factors along X,Y and Z axes.
    void scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ);

    //! This method shifts all vertex positions by the specified amount.
    virtual void offsetVertices(const cVector3d& a_offset, 
                                const bool a_updateCollisionDetector = true);

    //! This method computes the center of mass of this object, based on vertex positions.
    virtual cVector3d getCenterOfMass();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL
    //--------------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method rendres all segments.
    virtual void renderSegments(cRenderOptions& a_options);

    //! This method updates the global position of each vertex.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! This method updates the boundary box dimensions based on the vertices.
    virtual void updateBoundaryBox();

    //! This method copies all properties of this object to another.
    void copyMultiSegmentProperties(cMultiSegment* a_obj, 
        const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);

    //! This method scales this object by a scale factor.
    virtual void scaleObject(const double& a_scaleFactor) { scaleXYZ(a_scaleFactor, a_scaleFactor, a_scaleFactor); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DISPLAY PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, then segments are displayed.
    bool m_showSegments;

    //! Line width.
    double m_lineWidth;

    //! Specifies a multiplier for each bit in the line stipple pattern.
    GLint m_stippleFactor;

    //! Specifies a 16-bit integer whose bit pattern determine which fragments of a line will be drawn when the line is rasterized.
    GLushort m_stipplePattern;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - SEGMENT AND VERTEX DATA:
    //--------------------------------------------------------------------------

public:

    //! Array of vertices.
    cVertexArrayPtr m_vertices;

    //! Array of triangles.
    cSegmentArrayPtr m_segments;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

