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
    \author    Dan Morris
    \author    Chris Sewell
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//------------------------------------------------------------------------------
#ifndef CMeshH
#define CMeshH
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
    \file       CMesh.h

    \brief
    Implements a class to model polygonal meshes.
*/
//==============================================================================
//------------------------------------------------------------------------------

// Array of vertices.
class cVertexArray;
typedef std::shared_ptr<cVertexArray> cVertexArrayPtr;

// Array of triangles.
class cTriangleArray;
typedef std::shared_ptr<cTriangleArray> cTriangleArrayPtr;

// Edges
struct cEdge;

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cMesh
    \ingroup    world

    \brief
    This class implements a 3D polygonal mesh

    \details
    This class implements a 3D polygonal mesh. A mesh is composed of a collection 
    of vertices, triangles, materials, and texture properties that can be 
    rendered graphically and haptically.
*/
//==============================================================================
class cMesh : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cMesh.
    cMesh(cMaterialPtr a_material = cMaterialPtr());

    //! Destructor of cMesh.
    virtual ~cMesh();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cMesh* copy(const bool a_duplicateMaterialData = false,
                        const bool a_duplicateTextureData = false, 
                        const bool a_duplicateMeshData = false,
                        const bool a_buildCollisionDetector = false);


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
    // PUBLIC METHODS - VERTICES
    //--------------------------------------------------------------------------

public:

    //! This method creates a new vertex and adds it to the vertex list.
    unsigned int newVertex(const double a_x = 0.0, 
                           const double a_y = 0.0, 
                           const double a_z = 0.0,
                           const double a_normalX = 1.0, 
                           const double a_normalY = 0.0, 
                           const double a_normalZ = 0.0,
                           const double a_textureCoordX = 0.0,
                           const double a_textureCoordY = 0.0,
                           const double a_textureCoordZ = 0.0);

    //! This method creates a new vertex and adds it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal = cVector3d(1,0,0),
                           const cVector3d& a_textureCoord = cVector3d(0,0,0),
                           const cColorf& a_color = cColorf(0,0,0,1));

    //! This method returns the number of stored vertices.
    inline unsigned int getNumVertices() const { return (unsigned int)(m_vertices->getNumElements()); }

    //! This method sets the color of each vertex.
    void setVertexColor(const cColorf& a_color);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRIANGLES
    //--------------------------------------------------------------------------

public:

    //! This method creates a new triangle by passing vertex indices.
    unsigned int newTriangle(const unsigned int a_indexVertex0,
                             const unsigned int a_indexVertex1, 
                             const unsigned int a_indexVertex2);

    //! This method creates a new triangle and three new vertices by passing vertex positions, normals and texture coordinates.
    unsigned int newTriangle(const cVector3d& a_vertex0 = cVector3d(0,0,0), 
                             const cVector3d& a_vertex1 = cVector3d(0,0,0),
                             const cVector3d& a_vertex2 = cVector3d(0,0,0),
                             const cVector3d& a_normal0 = cVector3d(1,0,0), 
                             const cVector3d& a_normal1 = cVector3d(1,0,0),
                             const cVector3d& a_normal2 = cVector3d(1,0,0),
                             const cVector3d& a_textureCoord0 = cVector3d(0,0,0), 
                             const cVector3d& a_textureCoord1 = cVector3d(0,0,0),
                             const cVector3d& a_textureCoord2 = cVector3d(0,0,0),
                             const cColorf& a_colorVertex0 = cColorf(0,0,0,1),
                             const cColorf& a_colorVertex1 = cColorf(0,0,0,1),
                             const cColorf& a_colorVertex2 = cColorf(0,0,0,1));

    //! This method removes a triangle from the triangle array.
    bool removeTriangle(const unsigned int a_index);

    //! This method returns the number of stored triangles.
    unsigned int getNumTriangles();

    //! This method clears all triangles and vertices of mesh.
    void clear();

    //! This method enables or disables the rendering of triangles.
    void setShowTriangles(const bool a_showTriangles) { m_showTriangles = a_showTriangles; }

    //! This method returns whether rendering of triangles is enabled.
    bool getShowTriangles() const { return (m_showTriangles); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - EDGES
    //--------------------------------------------------------------------------

public:

    //! This method creates a list of edges by providing a threshold angle in degrees.
    void computeAllEdges(double a_angleThresholdDeg = 40.0);

    //! This method clears all edges
    void clearAllEdges();

    //! This method enables or disables the rendering of edges.
    void setShowEdges(const bool a_showEdges) { m_showEdges = a_showEdges; }

    //! This method returns whether rendering of edges is enabled.
    bool getShowEdges() const { return (m_showEdges); }

    //! This method sets the graphic properties for edge-rendering.
    void setEdgeProperties(const double a_width, 
        const cColorf& a_color);

    //! This method sets the line width of edges.
    void setEdgeLineWidth(const double a_width) { setEdgeProperties(a_width, m_edgeLineColor); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - SURFACE NORMALS, TANGENTS, BITANGENTS
    //--------------------------------------------------------------------------

public:

    //! This method enables or disables the rendering of vertex normals.
    void setShowNormals(const bool a_showNormals) { m_showNormals = a_showNormals; }

    //! This method returns whether rendering of normals is enabled.
    bool getShowNormals() const { return (m_showNormals); }

    //! This method sets the graphic properties for normal-rendering.
    void setNormalsProperties(const double a_length, const cColorf& a_color);

    //! This method set the length of normals for display purposes.
    void setNormalsLength(const double a_length) { setNormalsProperties(a_length, m_normalsColor); }

    //! This method computes all triangle normals.
    void computeAllNormals();

    //! This method reverses all surface normals.
    virtual void reverseAllNormals();

    //! This method computes the normal matrix vectors for all triangles.
    void computeBTN();

    //! This method enables or disables the rendering of tangents and bi-tangents.
    void setShowTangents(const bool a_showTangents) { m_showTangents = a_showTangents; }

    //! This method returns whether rendering of tangents and bi-tangents is enabled.
    bool getShowTangents() const { return (m_showTangents); }


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

    //! This method scale this mesh by using different scale factors along X, Y and Z axes.
    void scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ);

    //! This method shifts all vertex positions by the specified amount.
    virtual void offsetVertices(const cVector3d& a_offset, 
                                const bool a_updateCollisionDetector = true);

    //! This method computes the center of mass of this mesh, based on vertex positions.
    virtual cVector3d getCenterOfMass();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL
    //--------------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method draws a small line for each vertex normal.
    virtual void renderNormals(cRenderOptions& a_options);

    //! This method draws a small line for each tangent and bi-tangent.
    virtual void renderTangents(cRenderOptions& a_options);

    //! This method draws all edges of the mesh.
    virtual void renderEdges(cRenderOptions& a_options);

    //! This method renders all triangles, material and texture properties.
    virtual void renderMesh(cRenderOptions& a_options);

    //! This method updates the global position of each vertex.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! This method updates the boundary box dimensions based on the vertices.
    virtual void updateBoundaryBox();

    //! This method copies all properties of this mesh to another.
    void copyMeshProperties(cMesh* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);

    //! This method scales this object by a scale factor.
    virtual void scaleObject(const double& a_scaleFactor) { scaleXYZ(a_scaleFactor, a_scaleFactor, a_scaleFactor); }

    //! This method updates the relationship between the tool and the current object.
    void computeLocalInteraction(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DISPLAY PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, then triangles are displayed.
    bool m_showTriangles;

    //! If __true__, then normals are displayed.
    bool m_showNormals;

    //! If __true__, then tangents and bitangents are displayed.
    bool m_showTangents;

    //! Length of each normal (for graphic rendering of normals).
    double m_normalsLength;

    //! If __true__, then show edges.
    bool m_showEdges;

    //! Width of edge lines.
    double m_edgeLineWidth;

    //! Display list for edges.
    cDisplayList m_displayListEdges;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - DISPLAY PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Color used to render lines representing normals.
    cColorf m_normalsColor;

    //! Color used to render lines representing edges.
    cColorf m_edgeLineColor;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - TRIANGLE AND VERTEX DATA:
    //--------------------------------------------------------------------------

public:

    //! Array of vertices.
    cVertexArrayPtr m_vertices;

    //! Array of triangles.
    cTriangleArrayPtr m_triangles;

    //! Array of Edges.
    std::vector<cEdge> m_edges;
};


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \struct     cEdge
    \ingroup    graphics

    \brief
    3D Edge (defined by two vertices).

    \details
    cEdge is defined by two vertices (points) in 3 dimensional space.
*/
//==============================================================================
struct cEdge
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cEdge.
    cEdge() { m_parent = NULL; m_triangle = 0; m_vertex0 = 0; m_vertex1 = 0; }

    //! Constructor of cEdge.
    cEdge(cMesh* a_parent, 
          int a_vertex0, 
          int a_vertex1);
   
    //! Destructor of cEdge.
    ~cEdge() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method sets the edge points.
    void set(cMesh* a_parent, 
             int a_vertex0, 
             int a_vertex1);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:
    //! Index of vertex 0.
    int m_vertex0;

    //! Index of vertex 1.
    int m_vertex1;

    //! Index of triangle.
    int m_triangle;

    //! Parent mesh.
    cMesh* m_parent;
};


//------------------------------------------------------------------------------
// OPERTAOR - (IMPLEMENTED FOR EDGE SORTING PURPOSES)
//------------------------------------------------------------------------------
inline bool operator<(const cEdge& a_edge0, const cEdge& a_edge1)
{
    double tol = 0.0;

    cVector3d edge0Vertex0 = a_edge0.m_parent->m_vertices->getLocalPos(a_edge0.m_vertex0);
    cVector3d edge0Vertex1 = a_edge0.m_parent->m_vertices->getLocalPos(a_edge0.m_vertex1);
    cVector3d edge1Vertex0 = a_edge1.m_parent->m_vertices->getLocalPos(a_edge1.m_vertex0);
    cVector3d edge1Vertex1 = a_edge1.m_parent->m_vertices->getLocalPos(a_edge1.m_vertex1);


    if (cAbs(edge0Vertex0.x() - edge1Vertex0.x()) <= tol)
    {
        if (cAbs(edge0Vertex0.y() - edge1Vertex0.y()) <= tol)
        {
            if (cAbs(edge0Vertex0.z() - edge1Vertex0.z()) <= tol)
            {
                if (cAbs(edge0Vertex1.x() - edge1Vertex1.x()) <= tol)
                {
                    if (cAbs(edge0Vertex1.y() - edge1Vertex1.y()) <= tol)
                    {
                        return (edge0Vertex1.z() < edge1Vertex1.z());
                    }
                    else
                    {
                        return (edge0Vertex1.y() < edge1Vertex1.y());
                    }
                }
                else
                {
                    return (edge0Vertex1.x() < edge1Vertex1.x());
                }
            }
            else
            {
                return (edge0Vertex0.z() < edge1Vertex0.z());
            }
        }
        else
        {
            return (edge0Vertex0.y() < edge1Vertex0.y());
        }
    }
    else
    {
        return (edge0Vertex0.x() < edge1Vertex0.x());
    }
}

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

