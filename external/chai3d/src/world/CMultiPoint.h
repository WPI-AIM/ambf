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
#ifndef CMultiPointH
#define CMultiPointH
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
    \file       CMultiPoint.h

    \brief
    Implements a 3D point cloud object.
*/
//==============================================================================

//------------------------------------------------------------------------------
//! Array of vertices.
class cVertexArray;
typedef std::shared_ptr<cVertexArray> cVertexArrayPtr;

//! Array of points.
class cPointArray;
typedef std::shared_ptr<cPointArray> cPointArrayPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!    
    \class      cMultiPoint
    \ingroup    world

    \brief
    This class implements 3D point cloud object.

    \details
    This class implements 3D point cloud object.
*/
//==============================================================================
class cMultiPoint : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cMultiPoint.
    cMultiPoint();

    //! Destructor of cMultiPoint.
    virtual ~cMultiPoint();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cMultiPoint* copy(const bool a_duplicateMaterialData = false,
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
    // PUBLIC METHODS - POINTS
    //--------------------------------------------------------------------------

public:

    //! This method creates a new point by passing a vertex index.
    unsigned int newPoint(const unsigned int a_indexVertex0);

    //! This method creates a new point by passing a vertex position and color.
    unsigned int newPoint(const cVector3d& a_vertex0 = cVector3d(0,0,0),
                          const cColorf& a_colorVertex0 = cColorf(0,0,0,1));

    //! This method removed a selected point.
    bool removePoint(const unsigned int a_index);

    //! This method returns the number of stored points.
    unsigned int getNumPoints();

    //! This method clears all points and vertices.
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
    // PUBLIC METHODS - POINTS GRAPHIC PROPERTIES
    //--------------------------------------------------------------------------

public:

    //! This method sets a color for all points.
    void setPointColor(const cColorf& a_color);

    //! This method sets the point size that is used to graphically render the points.
    inline void setPointSize(const double a_pointSize) { m_pointSize = fabs(a_pointSize); }

    //! This method returns the point size.
    inline double getPointSize() const { return (m_pointSize); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //--------------------------------------------------------------------------

public:

    //! This method builds a brute force collision detector for this mesh.
    virtual void createBruteForceCollisionDetector();

    //! This method builds an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(const double a_radius);


    //--------------------------------------------------------------------------
    // PUBLIC VIRTUAL METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads a 3D point cloud from a file.
    virtual bool loadFromFile(std::string a_filename);

    //! This method saves a 3D point cloud to a file.
    virtual bool saveToFile(std::string a_filename);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GEOMETRY:
    //--------------------------------------------------------------------------

public:

    //! This method scales this point cloud by using different scale factors along X, Y and Z axes.
    void scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ);

    //! This method shifts all vertex positions by the specified amount.
    virtual void offsetVertices(const cVector3d& a_offset, 
        const bool a_updateCollisionDetector = true);

    //! This method computes the center of mass of this point cloud, based on vertex positions.
    virtual cVector3d getCenterOfMass();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL
    //--------------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method renders all points.
    virtual void renderPoints(cRenderOptions& a_options);

    //! This method updates the global position of each vertex.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! This method updates the boundary box dimensions based on the vertices.
    virtual void updateBoundaryBox();

    //! This method copies all properties of this point cloud to another.
    void copyMultiPointProperties(cMultiPoint* a_obj, 
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
    bool m_showPoints;

    //! Display size of point.
    double m_pointSize;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - POINT AND VERTEX DATA:
    //--------------------------------------------------------------------------

public:

    //! Array of vertices.
    cVertexArrayPtr m_vertices;

    //! Array of points.
    cPointArrayPtr m_points;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
