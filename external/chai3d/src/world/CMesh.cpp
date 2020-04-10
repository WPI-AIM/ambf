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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CMesh.h"
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
#include <algorithm>
#include <vector>
#include <list>
#include <utility>
#include <set>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMesh.

    \param  a_material  Material property to be applied to object.
*/
//==============================================================================
cMesh::cMesh(cMaterialPtr a_material)
{
    // create vertex array
    m_vertices = cVertexArray::create(true, true, true, true, true, false);

    // create triangle array
    m_triangles = cTriangleArray::create(m_vertices);

    // should triangles be displayed?
    m_showTriangles = true;

    // should normals be displayed?
    m_showNormals = false;

    // should tangents and bitangents be displayed?
    m_showTangents = false;

    // if normals are displayed, this value defines their length.
    m_normalsLength = 0.1;

    // if normals are displayed, this defines their color
    m_normalsColor.set(0.0, 0.0, 1.0);

    // should edges be displayed?
    m_showEdges = true;

    // Color used to render lines representing edges
    m_edgeLineColor.setBlack();

    // width of edge lines
    m_edgeLineWidth = 1.0;

    // should the frame (X-Y-Z) be displayed?
    m_showFrame = false;

    // set default collision detector
    m_collisionDetector = NULL;

    // display lists disabled by default
    m_useDisplayList = false;

    // set material properties
    if (a_material == nullptr)
    {
        m_material = cMaterial::create();
    }
    else
    {
        m_material = a_material;
    }
}


//==============================================================================
/*!
    Destructor of cMesh.
*/
//==============================================================================
cMesh::~cMesh()
{
    // delete any allocated display lists
    m_displayList.invalidate();
    m_displayListEdges.invalidate();
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return Pointer to new object.
*/
//==============================================================================
cMesh* cMesh::copy(const bool a_duplicateMaterialData,
                   const bool a_duplicateTextureData, 
                   const bool a_duplicateMeshData,
                   const bool a_buildCollisionDetector)
{
    // create new instance
    cMesh* obj = new cMesh();

    // copy properties of cMesh
    copyMeshProperties(obj,
                       a_duplicateMaterialData,
                       a_duplicateTextureData,
                       a_duplicateMeshData,
                       a_buildCollisionDetector);

    // return new mesh
    return (obj);
}


//==============================================================================
/*!
    This method copies all properties of this mesh to another.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cMesh::copyMeshProperties(cMesh* a_obj,
    const bool a_duplicateMaterialData,
    const bool a_duplicateTextureData, 
    const bool a_duplicateMeshData,
    const bool a_buildCollisionDetector)
{
    // copy properties of cGenericObject
    copyGenericObjectProperties(a_obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // copy properties of cMesh
    if (a_duplicateMeshData)
    {
        // duplicate vertex data
        a_obj->m_vertices = m_vertices->copy();

        // duplicate triangle data
        a_obj->m_triangles = m_triangles->copy();

        // build collision detector
        if (a_buildCollisionDetector)
        {
            double radius = 0.0;
            if (m_collisionDetector)
            {
                radius = m_collisionDetector->getBoundaryRadius();
            }
            a_obj->createAABBCollisionDetector(radius);
        }
    }
    else
    {
        // share mesh data
        a_obj->m_vertices = m_vertices;
        a_obj->m_triangles = m_triangles;
        if (a_buildCollisionDetector)
        {   
            if (m_collisionDetector)
            {
                a_obj->m_collisionDetector = m_collisionDetector;
            }
            else
            {
                a_obj->createAABBCollisionDetector(0.0);
            }
        }
    }

    // extras
    a_obj->m_normalsColor = m_normalsColor;
    a_obj->m_normalsLength = m_normalsLength;
    a_obj->m_useVertexColors = m_useVertexColors;
}


//==============================================================================
/*!
    This method computes the center of mass of this mesh, based on vertex 
    positions.

    \return Center of mass.
*/
//==============================================================================
cVector3d cMesh::getCenterOfMass()
{
    cVector3d centerOfMass(0,0,0);
    int numVertices = m_vertices->getNumElements();

    if (numVertices > 0) 
    {
        for(int i=0; i<numVertices; i++)
        {
            cVector3d pos = m_vertices->getLocalPos(i);
            centerOfMass += pos;
        }
        centerOfMass.mul(1.0 / numVertices);
    }

    return (centerOfMass);
}


//==============================================================================
/*!
    This method returns the number of stored triangles.

    \return Number of triangles.
*/
//==============================================================================
unsigned int cMesh::getNumTriangles()
{
    return (unsigned int)(m_triangles->getNumElements()); 
}


//==============================================================================
/*!
    This method creates a new vertex and adds it to the vertex list.

    \param  a_x              X coordinate of vertex.
    \param  a_y              Y coordinate of vertex.
    \param  a_z              Z coordinate of vertex.
    \param  a_normalX        X coordinate of normal associated with vertex.
    \param  a_normalY        Y coordinate of normal associated with vertex.
    \param  a_normalZ        Z coordinate of normal associated with vertex.
    \param  a_textureCoordX  X component of texture coordinate.
    \param  a_textureCoordY  Y component of texture coordinate.
    \param  a_textureCoordZ  Z component of texture coordinate.

    \return Index number of new vertex.
*/
//==============================================================================
unsigned int cMesh::newVertex(const double a_x, 
                              const double a_y, 
                              const double a_z,
                              const double a_normalX, 
                              const double a_normalY, 
                              const double a_normalZ,
                              const double a_textureCoordX,
                              const double a_textureCoordY,
                              const double a_textureCoordZ)
{
    unsigned int index = m_vertices->newVertex();

    // set data
    m_vertices->setLocalPos(index, a_x, a_y, a_z);
    m_vertices->setNormal(index, a_normalX, a_normalY, a_normalZ);
    m_vertices->setTexCoord(index, a_textureCoordX, a_textureCoordY, a_textureCoordZ);

    // return vertex index
    return (index);
}


//==============================================================================
/*!
    This method creates a new vertex and adds it to the vertex list.

    \param  a_pos           Position of new vertex.
    \param  a_normal        Normal vector associated with new vertex.
    \param  a_textureCoord  Texture coordinate of new vertex.
    \param  a_color         Color of new vertex.

    \return Index number of new vertex.
*/
//==============================================================================
unsigned int cMesh::newVertex(const cVector3d& a_pos, 
                              const cVector3d& a_normal,
                              const cVector3d& a_textureCoord,
                              const cColorf& a_color)
{
    unsigned int index = m_vertices->newVertex();

    // set data
    m_vertices->setLocalPos(index, a_pos);
    m_vertices->setNormal(index, a_normal);
    m_vertices->setTexCoord(index, a_textureCoord);
    m_vertices->setColor(index, a_color);

    // return vertex index
    return (index);
}


//==============================================================================
/*!
    This method creates a new triangle by passing vertex indices.

    \param  a_indexVertex0  Index position of vertex 0.
    \param  a_indexVertex1  Index position of vertex 1.
    \param  a_indexVertex2  Index position of vertex 2.

    \return Index number of new triangle.
*/
//==============================================================================
unsigned int cMesh::newTriangle(const unsigned int a_indexVertex0, 
                                const unsigned int a_indexVertex1,
                                const unsigned int a_indexVertex2)
{
    int index = m_triangles->newTriangle(a_indexVertex0, a_indexVertex1, a_indexVertex2);

    // mark mesh for update
    markForUpdate(false);

    // return the index at which I inserted this triangle in my triangle array
    return (index);
}


//==============================================================================
/*!
    This method creates a new triangle and three new vertices by passing vertex 
    positions, normals and texture coordinates.

    \param  a_vertex0        Position of vertex 0.
    \param  a_vertex1        Position of vertex 1.
    \param  a_vertex2        Position of vertex 2.
    \param  a_normal0        Normal of vertex 0.
    \param  a_normal1        Normal position of vertex 1.
    \param  a_normal2        Normal position of vertex 2.
    \param  a_textureCoord0  Texture coordinate of vertex 0.
    \param  a_textureCoord1  Texture coordinate of vertex 1.
    \param  a_textureCoord2  Texture coordinate of vertex 2.
    \param  a_colorVertex0   Color at vertex 0.
    \param  a_colorVertex1   Color at vertex 1.
    \param  a_colorVertex2   Color at vertex 2.

    \return Index number of new triangle.
*/
//==============================================================================
unsigned int cMesh::newTriangle(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cVector3d& a_vertex2,
                                const cVector3d& a_normal0, 
                                const cVector3d& a_normal1,
                                const cVector3d& a_normal2,
                                const cVector3d& a_textureCoord0, 
                                const cVector3d& a_textureCoord1,
                                const cVector3d& a_textureCoord2,
                                const cColorf& a_colorVertex0,
                                const cColorf& a_colorVertex1,
                                const cColorf& a_colorVertex2)
{

    unsigned int indexVertex0 = m_vertices->newVertex();
    unsigned int indexVertex1 = m_vertices->newVertex();
    unsigned int indexVertex2 = m_vertices->newVertex();

    int index = m_triangles->newTriangle(indexVertex0, indexVertex1, indexVertex2);

    m_vertices->setLocalPos(indexVertex0, a_vertex0);
    m_vertices->setLocalPos(indexVertex1, a_vertex1);
    m_vertices->setLocalPos(indexVertex2, a_vertex2);

    m_vertices->setNormal(indexVertex0, a_normal0);
    m_vertices->setNormal(indexVertex1, a_normal1);
    m_vertices->setNormal(indexVertex2, a_normal2);

    m_vertices->setTexCoord(indexVertex0, a_textureCoord0);
    m_vertices->setTexCoord(indexVertex1, a_textureCoord1);
    m_vertices->setTexCoord(indexVertex2, a_textureCoord2);

    m_vertices->setColor(indexVertex0, a_colorVertex0);
    m_vertices->setColor(indexVertex1, a_colorVertex1);
    m_vertices->setColor(indexVertex2, a_colorVertex2);

    // mark mesh for update
    markForUpdate(false);

    // return result
    return (index);
}


//==============================================================================
/*!
    This method removes a triangle by passing its index number.

    \param  a_index  Index number of triangle.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMesh::removeTriangle(const unsigned int a_index)
{
    m_triangles->removeTriangle(a_index);

    // clear edges
    clearAllEdges();

    // mark mesh for update
    markForUpdate(false);

    // return success
    return (true);
}


//==============================================================================
/*!
    This method clears all triangles and vertices.
*/
//==============================================================================
void cMesh::clear()
{
    // clear all triangles
    m_triangles->clear();

    // clear all vertices
    m_vertices->clear();

    // clear all edges
    m_edges.clear();

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method computes all surface normals for every vertex in the mesh, by 
    averaging the face normals of the triangle that include each vertex.
*/
//==============================================================================
void cMesh::computeAllNormals()
{
    // read number of vertices and triangles of object
    unsigned int numTriangles = m_triangles->getNumElements();
    unsigned int numVertices = m_vertices->getNumElements();

    // initialize all normals to zero
    for (unsigned int i=0; i<numVertices; i++)
    {
        m_vertices->setNormal(i, 0, 0, 0);
    }

    // compute the normal of each triangle, add contribution to each vertex
    for (unsigned int i=0; i<numTriangles; i++)
    {
        unsigned int vertexIndex0 = m_triangles->getVertexIndex0(i);
        unsigned int vertexIndex1 = m_triangles->getVertexIndex1(i);
        unsigned int vertexIndex2 = m_triangles->getVertexIndex2(i);

        cVector3d vertex0 = m_vertices->getLocalPos(vertexIndex0);
        cVector3d vertex1 = m_vertices->getLocalPos(vertexIndex1);
        cVector3d vertex2 = m_vertices->getLocalPos(vertexIndex2);

        // compute normal vector
        cVector3d normal, v01, v02;
        vertex1.subr(vertex0, v01);
        vertex2.subr(vertex0, v02);
        v01.crossr(v02, normal);
        double length = normal.length();
        if (length > 0.0)
        {
            normal.div(length);
            m_vertices->m_normal[vertexIndex0].add(normal);
            m_vertices->m_normal[vertexIndex1].add(normal);
            m_vertices->m_normal[vertexIndex2].add(normal);
        }
    }

    // normalize all triangles
    for (unsigned int i=0; i<numVertices; i++)
    {
        if (m_vertices->m_normal[i].length() > 0.000000001)
        {
            m_vertices->m_normal[i].normalize();
        }
    }
}


//==============================================================================
/*!
    This method computes the normal matrix vectors for all triangles.
*/
//==============================================================================
void cMesh::computeBTN()
{
    m_triangles->computeBTN();
}


//==============================================================================
/*!
    This method scales this object by using different scale factors along 
    X,Y and Z axes.

    \param  a_scaleX  Scale factor along X axis.
    \param  a_scaleY  Scale factor along Y axis.
    \param  a_scaleZ  Scale factor along Z axis.
*/
//==============================================================================
void cMesh::scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ)
{
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->m_localPos[i].mul(a_scaleX, a_scaleY, a_scaleZ);
    }

    m_boundaryBoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
    m_boundaryBoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method computes the global position of all vertices.

    \param  a_frameOnly  If __false__, then the global position of all vertices is computed.
*/
//==============================================================================
void cMesh::updateGlobalPositions(const bool a_frameOnly)
{
    if (a_frameOnly) return;

    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->computeGlobalPosition(i, m_globalPos, m_globalRot);
    }
}


//==============================================================================
/*!
    This method invalidates any existing display lists. You should call this on
    if you are using display lists and you modify mesh options, vertex 
    positions, etc.

    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMesh::markForUpdate(const bool a_affectChildren)
{
    // mark triangles for update
    m_triangles->m_flagMarkForUpdate = true;

    // invalidate display list
    m_displayList.invalidate();
    m_displayListEdges.invalidate();

    // update display list of cGenericObject and children
    cGenericObject::markForUpdate(a_affectChildren);
}


//==============================================================================
/*!
    This method sets the alpha value at each vertex, in all of its material 
    colors, optionally propagating the operation to my children.

    \param  a_level            Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToVertices  If __true__, then apply changes to vertex colors.
    \param  a_applyToTextures  If __true__, then apply changes to texture.
    \param  a_affectChildren   If __true__, then children are updated too.
*/
//==============================================================================
void cMesh::setTransparencyLevel(const float a_level,
    const bool a_applyToVertices,
    const bool a_applyToTextures,
    const bool a_affectChildren)
{
    cGenericObject::setTransparencyLevel(a_level, a_applyToVertices, a_applyToTextures, a_affectChildren);

    // apply the new value to all vertex colors
    if (a_applyToVertices)
    {    
        int numVertices = m_vertices->getNumElements();
        for (int i = 0; i < numVertices; i++)
        {
            m_vertices->m_color[i].setA(a_level);
        }

        // mark for update
        markForUpdate(true);
    }
}


//==============================================================================
/*!
    This method sets the color of each vertex.

    \param  a_color  New color to be applied to each vertex.
*/
//==============================================================================
void cMesh::setVertexColor(const cColorf& a_color)
{
    // apply color to all vertex colors
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->setColor(i, a_color);
    }

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method shifts all vertex positions by the specified amount.

    \param  a_offset                   Translation to apply to each vertex.
    \param  a_updateCollisionDetector  If __true__, then update collision detector.
*/
//==============================================================================
void cMesh::offsetVertices(const cVector3d& a_offset,
                           const bool a_updateCollisionDetector)
{
    // offset all vertices
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->m_localPos[i].add(a_offset);
    }

    // update boundary box
    m_boundaryBoxMin+=a_offset;
    m_boundaryBoxMax+=a_offset;

    // update collision detector if requested
    if (a_updateCollisionDetector && m_collisionDetector)
    {
        m_collisionDetector->update();
    }

    // mark for update
    markForUpdate(false);
}


//==============================================================================
/*!
    This method reverses the normal for every vertex on this model. Useful 
    for models that started with inverted faces and thus gave inward-pointing
    normals.
*/
//==============================================================================
void cMesh::reverseAllNormals()
{
    // reverse normals for this object
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->m_normal[i].negate();
    }
}


//==============================================================================
/*!
    This method defines the way normals are graphically rendered.

    \param  a_length  Length of normals.
    \param  a_color   Color of normals.
*/
//==============================================================================
void cMesh::setNormalsProperties(const double a_length, 
                                 const cColorf& a_color)
{
    m_normalsLength = cClamp0(a_length);
    m_normalsColor = a_color;
}


//==============================================================================
/*!
    This method compute the axis-aligned boundary box that encloses all
    triangles in this mesh.
*/
//==============================================================================
void cMesh::updateBoundaryBox()
{
    unsigned int numTriangles = m_triangles->getNumElements();

    if (numTriangles == 0)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
        return;
    }

    double xMin = C_LARGE;
    double yMin = C_LARGE;
    double zMin = C_LARGE;
    double xMax = -C_LARGE;
    double yMax = -C_LARGE;
    double zMax = -C_LARGE;
    bool flag = false;

    // loop over all my triangles
    for(unsigned int i=0; i<numTriangles; i++)
    {
        if (m_triangles->m_allocated[i])
        {
            cVector3d tVertex0 = m_vertices->getLocalPos(m_triangles->getVertexIndex0(i));
            xMin = cMin(tVertex0(0) , xMin);
            yMin = cMin(tVertex0(1) , yMin);
            zMin = cMin(tVertex0(2) , zMin);
            xMax = cMax(tVertex0(0) , xMax);
            yMax = cMax(tVertex0(1) , yMax);
            zMax = cMax(tVertex0(2) , zMax);

            cVector3d tVertex1 = m_vertices->getLocalPos(m_triangles->getVertexIndex1(i));
            xMin = cMin(tVertex1(0) , xMin);
            yMin = cMin(tVertex1(1) , yMin);
            zMin = cMin(tVertex1(2) , zMin);
            xMax = cMax(tVertex1(0) , xMax);
            yMax = cMax(tVertex1(1) , yMax);
            zMax = cMax(tVertex1(2) , zMax);

            cVector3d tVertex2 = m_vertices->getLocalPos(m_triangles->getVertexIndex2(i));
            xMin = cMin(tVertex2(0) , xMin);
            yMin = cMin(tVertex2(1) , yMin);
            zMin = cMin(tVertex2(2) , zMin);
            xMax = cMax(tVertex2(0) , xMax);
            yMax = cMax(tVertex2(1) , yMax);
            zMax = cMax(tVertex2(2) , zMax);

            flag = true;
        }
    }

    if (flag)
    {
        m_boundaryBoxMin.set(xMin, yMin, zMin);
        m_boundaryBoxMax.set(xMax, yMax, zMax);
        m_boundaryBoxEmpty = false;
    }
    else
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
    }
}


//==============================================================================
/*!
    This method creates a list of edges by providing a threshold angle in 
    degrees. All triangles for which the angle between their respective surface
    normals are greater than the select angle threshold are added to the list of 
    edges.

    \param  a_angleThresholdDeg  Threshold angle in degrees.
*/
//==============================================================================
void cMesh::computeAllEdges(double a_angleThresholdDeg)
{
    // clear current list of edges.
    clearAllEdges();

    // initialize variables
    int numtriangles = getNumTriangles();

    multiset<cEdge> edges;
    edges.clear();
    cEdge edge;
    multiset<cEdge>::iterator it;

    // setup angle threshold
    double angleThresholdRad = cDegToRad(a_angleThresholdDeg);

    // process all triangles
    for (int i=0; i<numtriangles; i++)
    {
        int v0 = m_triangles->getVertexIndex0(i);
        int v1 = m_triangles->getVertexIndex1(i);
        int v2 = m_triangles->getVertexIndex2(i);

        cVector3d z0 = m_vertices->getLocalPos(v0);
        cVector3d z1 = m_vertices->getLocalPos(v1);
        cVector3d z2 = m_vertices->getLocalPos(v2);

        cVector3d n0 = cComputeSurfaceNormal(z0, z1, z2);

        if (n0.length() > 0)
        {
            //////////////////////////////////////////////////////////////////
            // store reference to triangle
            //////////////////////////////////////////////////////////////////
            edge.m_triangle = i;
            
            //////////////////////////////////////////////////////////////////
            // edge 01 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(this, v0, v1);
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cVector3d n1 = cComputeSurfaceNormal(m_vertices->getLocalPos(m_triangles->getVertexIndex0((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex1((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex2((*it).m_triangle)));

                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= angleThresholdRad)
                    {
                        m_edges.push_back(edge);
                    }
                }

                // remove edge as we have already found it dual
                edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }


            //////////////////////////////////////////////////////////////////
            // edge 02 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(this, v0, v2);
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cVector3d n1 = cComputeSurfaceNormal(m_vertices->getLocalPos(m_triangles->getVertexIndex0((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex1((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex2((*it).m_triangle)));


                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= angleThresholdRad)
                    {
                        m_edges.push_back(edge);
                    }
                }

                // remove edge as we have already found it dual
                edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }

            //////////////////////////////////////////////////////////////////
            // edge 12 of triangle.
            //////////////////////////////////////////////////////////////////
            edge.set(this, v1, v2);
            it = edges.find(edge);
            if (it!=edges.end())
            {
                cVector3d n1 = cComputeSurfaceNormal(m_vertices->getLocalPos(m_triangles->getVertexIndex0((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex1((*it).m_triangle)),
                                                     m_vertices->getLocalPos(m_triangles->getVertexIndex2((*it).m_triangle)));


                if (n1.length() > 0.0)
                {
                    if (cAngle(n0, n1) >= angleThresholdRad)
                    {
                        m_edges.push_back(edge);
                    }
                }

                // remove edge as we have already found it dual
                edges.erase(it);
            }
            else
            {
                edges.insert(edge);
            }
        }
    }

    // store all edges with no dual in the edge list
    multiset<cEdge>::iterator it2;
    for (it2 = edges.begin(); it2 != edges.end(); it2++)
    {
        m_edges.push_back(*it2);
    }
}


//==============================================================================
/*!
    This method clears all edges.
*/
//==============================================================================
void cMesh::clearAllEdges()
{
    // clear all edges
    m_edges.clear();
    m_displayListEdges.invalidate();
}


//==============================================================================
/*!
    This method sets the graphic properties for edge-rendering.

    \param  a_lineWidth  Width of edge lines.
    \param  a_lineColor  Color of edge lines.
*/
//==============================================================================
void cMesh::setEdgeProperties(const double a_lineWidth, 
                              const cColorf& a_lineColor)
{
    m_edgeLineWidth = cMax(1.0, a_lineWidth);
    m_edgeLineColor = a_lineColor;
}


//==============================================================================
/*!
     This method builds a brute Force collision detector for this mesh.
*/
//==============================================================================
void cMesh::createBruteForceCollisionDetector()
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create brute collision detector
    m_collisionDetector = new cCollisionBrute(m_triangles);
}


//==============================================================================
/*!
    This method builds an AABB collision detector for this mesh.

    \param  a_radius  Bounding radius.
*/
//==============================================================================
void cMesh::createAABBCollisionDetector(const double a_radius)
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create AABB and initialize collision detector 
    cCollisionAABB* collisionDetector = new cCollisionAABB();
    collisionDetector->initialize(m_triangles, a_radius);

    // assign new collision detector
    m_collisionDetector = collisionDetector;
}


//==============================================================================
/*!
    This method uses the position of the tool and searches for the nearest point
    located at the surface of the current object and identifies if the point is
    located inside or outside of the object.

    For mesh objects, this information is computed by the virtual tool
    when computing the finger-proxy model. More information can be found in 
    file cToolCursor.cpp under method \ref cToolCursor::computeInteractionForces()
    Both variables m_interactionProjectedPoint and m_interactionInside are
    assigned values based on the objects encountered by the proxy.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN      Identification number of the force algorithm.
*/
//==============================================================================
void cMesh::computeLocalInteraction(const cVector3d& a_toolPos,
                                    const cVector3d& a_toolVel,
                                    const unsigned int a_IDN)
{
    // m_interactionProjectedPoint
    // m_interactionInside
}


//==============================================================================
/*!
    This method renders this mesh using OpenGL. This method actually just 
    prepares some OpenGL state, and uses renderMesh to actually do the rendering.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMesh::render(cRenderOptions& a_options)
{
    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // render normals
        // (note that we will not render the normals if we are currently creating
        // a shadow map). 
        if (m_showNormals && !a_options.m_creating_shadow_map) 
        {
            renderNormals(a_options);
        }

        // render tangents
        // (note that we will not render the bitangents if we are currently creating
        // a shadow map). 
        if (m_showTangents && !a_options.m_creating_shadow_map) 
        {
            renderTangents(a_options);
        }

        // render edges
        if (m_showEdges && !a_options.m_creating_shadow_map)
        {
            renderEdges(a_options);
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that use material properties
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
    {
        if (m_showTriangles)
        {
            renderMesh(a_options);
        }
    }
}


//==============================================================================
/*!
    This method renders a graphic representation of each normal of the mesh.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMesh::renderNormals(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // check if any normals to render
    unsigned int numtriangles = m_triangles->getNumElements();

    if (numtriangles == 0)
    {
        return;
    }

    // disable lighting
    glDisable(GL_LIGHTING);

    // set line width
    glLineWidth(1.0);

    // set color
    glColor4fv( (const float *)&m_normalsColor);

    // render normals
    glBegin(GL_LINES);
    for(unsigned int i=0; i<numtriangles; i++) 
    {
        if (m_triangles->getAllocated(i))
        {
            cVector3d v, n;

            // normal 0
            v = m_vertices->getLocalPos(m_triangles->getVertexIndex0(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex0(i));
            glVertex3d(v(0) ,v(1) ,v(2) );
            n.mul(m_normalsLength);
            n.add(v);
            glVertex3d(n(0) ,n(1) ,n(2) );

            // normal 1
            v = m_vertices->getLocalPos(m_triangles->getVertexIndex1(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex1(i));
            glVertex3d(v(0) ,v(1) ,v(2) );
            n.mul(m_normalsLength);
            n.add(v);
            glVertex3d(n(0) ,n(1) ,n(2) );

            // normal 2
            v = m_vertices->getLocalPos(m_triangles->getVertexIndex2(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex2(i));
            glVertex3d(v(0) ,v(1) ,v(2) );
            n.mul(m_normalsLength);
            n.add(v);
            glVertex3d(n(0) ,n(1) ,n(2) );
        }
    }
    glEnd();

    // enable lighting
    glEnable(GL_LIGHTING);

#endif
}


//==============================================================================
/*!
    This method draws a small line for each tangent and bi-tangent.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMesh::renderTangents(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    // get number of triangles
    unsigned int numtriangles = m_triangles->getNumElements();

    // check if any normals to render
    if (numtriangles == 0)
    {
        return;
    }

    // check if tangents and bi-tangents are defined
    if (!(m_vertices->getUseTangentData() && m_vertices->getUseBitangentData()))
    {
        return;
    }

    // disable lighting
    glDisable(GL_LIGHTING);

    // set line width
    glLineWidth(1.0);

    // render normals slightly above triangle surface
    glBegin(GL_LINES);
    for(unsigned int i=0; i<numtriangles; i++) 
    {
        if (m_triangles->getAllocated(i))
        {
            cVector3d p, n, u, v;

            // tangent and bitangent 0
            p = m_vertices->getLocalPos(m_triangles->getVertexIndex0(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex0(i));
            u = m_vertices->getTangent(m_triangles->getVertexIndex0(i));
            v = m_vertices->getBitangent(m_triangles->getVertexIndex0(i));

            n.mul(-0.01 * m_normalsLength);
            p.add(n); // offset point to be above triangle
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            u.mul(m_normalsLength);
            u.add(p);
            glVertex3d(u(0), u(1), u(2));
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            v.mul(m_normalsLength);
            v.add(p);
            glVertex3d(v(0), v(1), v(2));

            // tangent and bitangent 1
            p = m_vertices->getLocalPos(m_triangles->getVertexIndex1(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex1(i));
            u = m_vertices->getTangent(m_triangles->getVertexIndex1(i));
            v = m_vertices->getBitangent(m_triangles->getVertexIndex1(i));

            n.mul(-0.01 * m_normalsLength);
            p.add(n); // offset point to be above triangle
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            u.mul(m_normalsLength);
            u.add(p);
            glVertex3d(u(0), u(1), u(2));
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            v.mul(m_normalsLength);
            v.add(p);
            glVertex3d(v(0), v(1), v(2));

            // tangent and bitangent 2
            p = m_vertices->getLocalPos(m_triangles->getVertexIndex2(i));
            n = m_vertices->getNormal(m_triangles->getVertexIndex2(i));
            u = m_vertices->getTangent(m_triangles->getVertexIndex2(i));
            v = m_vertices->getBitangent(m_triangles->getVertexIndex2(i));

            n.mul(-0.01 * m_normalsLength);
            p.add(n); // offset point to be above triangle
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            u.mul(m_normalsLength);
            u.add(p);
            glVertex3d(u(0), u(1), u(2));
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(p(0) ,p(1) ,p(2) );
            v.mul(m_normalsLength);
            v.add(p);
            glVertex3d(v(0), v(1), v(2));
        }
    }
    glEnd();

    // enable lighting
    glEnable(GL_LIGHTING);

#endif
}


//==============================================================================
/*!
    This method renders all edges.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMesh::renderEdges(cRenderOptions& a_options)
{

#ifdef C_USE_OPENGL

    if (m_edges.size() == 0) { return; }

    /////////////////////////////////////////////////////////////////////////
    // SET PROPERTIES
    /////////////////////////////////////////////////////////////////////////

    // turn-off lighting
    glDisable(GL_LIGHTING);
    
    // setup color for rendering line edges
    m_edgeLineColor.render();

    // width of line edges
    glLineWidth((GLfloat)m_edgeLineWidth);

    // render all edges as triangles so that we can perform polygon offset
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset (-10.0f, -100.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // disable culling
    glDisable(GL_CULL_FACE);


    /////////////////////////////////////////////////////////////////////////
    // RENDER LINES
    /////////////////////////////////////////////////////////////////////////

    // render all lines
    if (!m_displayListEdges.render(m_useDisplayList))
    {
        m_displayListEdges.begin(m_useDisplayList);
   
        glBegin(GL_TRIANGLES);
            vector<cEdge>::iterator i;
            for(i = m_edges.begin(); i != m_edges.end(); i++)
            {
                cVector3d v0 = (*i).m_parent->m_vertices->getLocalPos((*i).m_vertex0);
                cVector3d v1 = (*i).m_parent->m_vertices->getLocalPos((*i).m_vertex1);

                glVertex3dv(&v0(0));
                glVertex3dv(&v1(0));
                glVertex3dv(&v1(0));
            }
        glEnd();

        m_displayListEdges.end(true);
    }


    /////////////////////////////////////////////////////////////////////////
    // FINALIZE
    /////////////////////////////////////////////////////////////////////////

    if (m_cullingEnabled)
    {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {
        glDisable(GL_CULL_FACE);
    }

    // restore OpenGL properties
    glPolygonOffset(0.0, 0.0);
    glDisable(GL_POLYGON_OFFSET_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, m_triangleMode);
    glEnable(GL_LIGHTING);

#endif
}


//==============================================================================
/*!
    This method renders the mesh itself. This method is declared public to 
    allow sharing of data among meshes, which is not possible given most
    implementations of 'protected'.  But it should only be accessed
    from within render() or derived versions of render().

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMesh::renderMesh(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    unsigned int numVertices = m_vertices->getNumElements();
    unsigned int numTriangles = m_triangles->getNumElements();

    // check if object contains any triangles or vertices
    if ((numVertices == 0) || (numTriangles == 0))
    {
        return;
    }

    // initialize some OpenGL flags
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_EDGE_FLAG_ARRAY);
    glDisable(GL_COLOR_MATERIAL);


    //--------------------------------------------------------------------------
    // RENDER MATERIAL
    //--------------------------------------------------------------------------

    // render material properties if enabled
    if (m_useMaterialProperty && a_options.m_render_materials)
    {
        m_material->render(a_options);
    }

    //--------------------------------------------------------------------------
    // RENDER TEXTURE
    //--------------------------------------------------------------------------

    // check if texture is available
    if (m_texture == nullptr)
    {
        m_useTextureMapping = false;
    }

    // render texture if enabled
    if ((m_texture != nullptr) && (m_useTextureMapping) && (a_options.m_render_materials))
    {
        m_texture->renderInitialize(a_options);
    }


    //--------------------------------------------------------------------------
    // RENDER VERTEX COLORS
    //--------------------------------------------------------------------------
    /*
    if vertex colors (m_useVertexColors) is enabled, we render the colors 
    defined for each individual vertex.
    
    if material properties (m_useMaterialProperty) has also been enabled, 
    then we combine vertex colors and materials together with OpenGL lighting 
    enabled.
    
    if material properties are disabled then lighting is disabled and
    we use the pure color defined at each vertex to render the object
    */

    if (m_useVertexColors && a_options.m_render_materials)
    {
        // Clear the effects of material properties...
        if (m_useMaterialProperty || a_options.m_rendering_shadow)
        {
            // enable vertex colors
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable(GL_COLOR_MATERIAL);
            glEnable(GL_LIGHTING);
        }
        else
        {
            glDisable(GL_LIGHTING);
        }
    }
    else
    {
        glDisable(GL_COLOR_MATERIAL);
    }


    //--------------------------------------------------------------------------
    // FOR OBJECTS WITH NO DEFINED COLOR/MATERIAL SETTINGS
    //--------------------------------------------------------------------------
    /*
    A default color for objects that don't have vertex colors or
    material properties (otherwise they're invisible)...
    If texture mapping is enabled, then just turn off lighting
    */

    if (((!m_useVertexColors) && (!m_useMaterialProperty)) && a_options.m_render_materials)
    {
        if (m_useTextureMapping && !a_options.m_rendering_shadow)
        {
            glDisable(GL_LIGHTING);
        }
        else
        {
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glColor4f(1.0,1.0,1.0,1.0);
        }
    }


    //--------------------------------------------------------------------------
    // RENDER OBJECT (OBJECT BUFFER + SHADERS)
    //--------------------------------------------------------------------------
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // enable shader
        m_shaderProgram->use(this, a_options);

        // render normal texture if enabled
        if (m_normalMap != nullptr)
        {
            if (m_shaderProgram->isUsed())
            {
                m_normalMap->renderInitialize(a_options);
            }
        }

        // initialize rendering of VBO
        m_triangles->renderInitialize();

        // finalize rendering of VBO
        m_triangles->renderFinalize();

        // disable normal map is available
        if (m_normalMap != nullptr)
        {
            m_normalMap->renderFinalize(a_options);
        }

        // disable shader
        m_shaderProgram->disable();
    }

    //--------------------------------------------------------------------------
    // RENDER OBJECT (OLD METHOD)
    //--------------------------------------------------------------------------
    else if (!m_displayList.render(m_useDisplayList))   
    {
        // get texture unit
        GLenum textureUnit = GL_TEXTURE1_ARB;
        if (m_texture != nullptr)
        {
            textureUnit = m_texture->getTextureUnit();
        }

        // if requested, begin creating display list
        m_displayList.begin(m_useDisplayList);

        //-------------------------------------------------------------------
        // RENDER ALL TRIANGLES
        //-------------------------------------------------------------------

        /////////////////////////////////////////////////////////////////////
        // RENDER TRIANGLES USING CLASSIC OPENGL COMMANDS
        /////////////////////////////////////////////////////////////////////
        {
            // begin rendering triangles
            glBegin(GL_TRIANGLES);

            // render all active triangles
            if (((!m_useTextureMapping) && (!m_useVertexColors)))
            {
                for (unsigned int i=0; i<numTriangles; i++)
                {
                    if (m_triangles->m_allocated[i])
                    {
                        unsigned int index0 = m_triangles->getVertexIndex0(i);
                        unsigned int index1 = m_triangles->getVertexIndex1(i);
                        unsigned int index2 = m_triangles->getVertexIndex2(i);

                        // render vertex 0
                        glNormal3dv(&m_vertices->m_normal[index0](0));
                        glVertex3dv(&m_vertices->m_localPos[index0](0));

                        // render vertex 1
                        glNormal3dv(&m_vertices->m_normal[index1](0));
                        glVertex3dv(&m_vertices->m_localPos[index1](0));

                        // render vertex 2
                        glNormal3dv(&m_vertices->m_normal[index2](0));
                        glVertex3dv(&m_vertices->m_localPos[index2](0));
                    }
                }
            }
            else if ((m_useTextureMapping) && (!m_useVertexColors))
            {
                if (m_texture->m_image->getImageCount() > 1)
                {
                    for (unsigned int i=0; i<numTriangles; i++)
                    {
                        if (m_triangles->m_allocated[i])
                        {
                            unsigned int index0 = m_triangles->getVertexIndex0(i);
                            unsigned int index1 = m_triangles->getVertexIndex1(i);
                            unsigned int index2 = m_triangles->getVertexIndex2(i);

                            // render vertex 0
                            glNormal3dv(&m_vertices->m_normal[index0](0));
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index0](0));
                            glVertex3dv(&m_vertices->m_localPos[index0](0));

                            // render vertex 1
                            glNormal3dv(&m_vertices->m_normal[index1](0));
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index1](0));
                            glVertex3dv(&m_vertices->m_localPos[index1](0));

                            // render vertex 2
                            glNormal3dv(&m_vertices->m_normal[index2](0));
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index2](0));
                            glVertex3dv(&m_vertices->m_localPos[index2](0));
                        }
                    }
                }
                else
                {
                    for (unsigned int i=0; i<numTriangles; i++)
                    {
                        if (m_triangles->m_allocated[i])
                        {
                            unsigned int index0 = m_triangles->getVertexIndex0(i);
                            unsigned int index1 = m_triangles->getVertexIndex1(i);
                            unsigned int index2 = m_triangles->getVertexIndex2(i);

                            // render vertex 0
                            glNormal3dv(&m_vertices->m_normal[index0](0));
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index0](0));
                            glVertex3dv(&m_vertices->m_localPos[index0](0));

                            // render vertex 1
                            glNormal3dv(&m_vertices->m_normal[index1](0));
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index1](0));
                            glVertex3dv(&m_vertices->m_localPos[index1](0));

                            // render vertex 2
                            glNormal3dv(&m_vertices->m_normal[index2](0));
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index2](0));
                            glVertex3dv(&m_vertices->m_localPos[index2](0));
                        }
                    }
                }

            }
            else if ((!m_useTextureMapping) && (m_useVertexColors))
            {
                for (unsigned int i=0; i<numTriangles; i++)
                {
                    if (m_triangles->m_allocated[i])
                    {
                        unsigned int index0 = m_triangles->getVertexIndex0(i);
                        unsigned int index1 = m_triangles->getVertexIndex1(i);
                        unsigned int index2 = m_triangles->getVertexIndex2(i);

                        // render vertex 0
                        glNormal3dv(&m_vertices->m_normal[index0](0));
                        glColor4fv(m_vertices->m_color[index0].getData());
                        glVertex3dv(&m_vertices->m_localPos[index0](0));

                        // render vertex 1
                        glNormal3dv(&m_vertices->m_normal[index1](0));
                        glColor4fv(m_vertices->m_color[index1].getData());
                        glVertex3dv(&m_vertices->m_localPos[index1](0));

                        // render vertex 2
                        glNormal3dv(&m_vertices->m_normal[index2](0));
                        glColor4fv(m_vertices->m_color[index2].getData());
                        glVertex3dv(&m_vertices->m_localPos[index2](0));
                    }
                }
            }
            else if ((m_useTextureMapping) && (m_useVertexColors))
            {
                if (m_texture->m_image->getImageCount() > 1)
                {
                    for (unsigned int i=0; i<numTriangles; i++)
                    {
                        if (m_triangles->m_allocated[i])
                        {
                            unsigned int index0 = m_triangles->getVertexIndex0(i);
                            unsigned int index1 = m_triangles->getVertexIndex1(i);
                            unsigned int index2 = m_triangles->getVertexIndex2(i);

                            // render vertex 0
                            glNormal3dv(&m_vertices->m_normal[index0](0));
                            glColor4fv(m_vertices->m_color[index0].getData());
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index0](0));
                            glVertex3dv(&m_vertices->m_localPos[index0](0));

                            // render vertex 1
                            glNormal3dv(&m_vertices->m_normal[index1](0));
                            glColor4fv(m_vertices->m_color[index1].getData());
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index1](0));
                            glVertex3dv(&m_vertices->m_localPos[index1](0));

                            // render vertex 2
                            glNormal3dv(&m_vertices->m_normal[index2](0));
                            glColor4fv(m_vertices->m_color[index2].getData());
                            glMultiTexCoord3dv(textureUnit, &m_vertices->m_texCoord[index2](0));
                            glVertex3dv(&m_vertices->m_localPos[index2](0));
                        }
                    }
                }
                else
                {
                    for (unsigned int i=0; i<numTriangles; i++)
                    {
                        if (m_triangles->m_allocated[i])
                        {
                            unsigned int index0 = m_triangles->getVertexIndex0(i);
                            unsigned int index1 = m_triangles->getVertexIndex1(i);
                            unsigned int index2 = m_triangles->getVertexIndex2(i);

                            // render vertex 0
                            glNormal3dv(&m_vertices->m_normal[index0](0));
                            glColor4fv(m_vertices->m_color[index0].getData());
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index0](0));
                            glVertex3dv(&m_vertices->m_localPos[index0](0));

                            // render vertex 1
                            glNormal3dv(&m_vertices->m_normal[index1](0));
                            glColor4fv(m_vertices->m_color[index1].getData());
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index1](0));
                            glVertex3dv(&m_vertices->m_localPos[index1](0));

                            // render vertex 2
                            glNormal3dv(&m_vertices->m_normal[index2](0));
                            glColor4fv(m_vertices->m_color[index2].getData());
                            glMultiTexCoord2dv(textureUnit, &m_vertices->m_texCoord[index2](0));
                            glVertex3dv(&m_vertices->m_localPos[index2](0));
                        }
                    }
                }
            }

            // finalize rendering list of triangles
            glEnd();
        }

        //-------------------------------------------------------------------
        // FINALIZE DISPLAY LIST
        //-------------------------------------------------------------------

        // if being created, finalize display list
        m_displayList.end(true);
    }


    //--------------------------------------------------------------------------
    // RESTORE OPENGL
    //--------------------------------------------------------------------------

    // turn off texture rendering
    if ((m_texture != nullptr) && (m_useTextureMapping))
    {
        m_texture->renderFinalize(a_options);
    }

    // restore OpenGL variables
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

#endif
}


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cEdge.

    \param  a_parent   Mesh parent.
    \param  a_vertex0  Vertex 0 of edge.
    \param  a_vertex1  Vertex 1 of edge.
*/
//==============================================================================
cEdge::cEdge(cMesh* a_parent,
             int a_vertex0,
             int a_vertex1)
{
    set(a_parent, a_vertex0, a_vertex1);
}


//==============================================================================
/*!
    This method sets the two points of this edge.

    \param  a_parent   Mesh parent.
    \param  a_vertex0  Vertex 0 of edge.
    \param  a_vertex1  Vertex 1 of edge.
*/
//==============================================================================
void cEdge::set(cMesh* a_parent, 
                int a_vertex0, 
                int a_vertex1)
{
    // sanity check
    if (a_parent == NULL)
    {
        return;
    }

    m_parent = a_parent;

    // sort order of points
    if (m_parent->m_vertices->getLocalPos(a_vertex0).x() > m_parent->m_vertices->getLocalPos(a_vertex1).x())
    {
        m_vertex0 = a_vertex0;
        m_vertex1 = a_vertex1;
    }
    else if (m_parent->m_vertices->getLocalPos(a_vertex0).x() < m_parent->m_vertices->getLocalPos(a_vertex1).x())
    {
        m_vertex1 = a_vertex0;
        m_vertex0 = a_vertex1;
    }
    else
    {
        if (m_parent->m_vertices->getLocalPos(a_vertex0).y() > m_parent->m_vertices->getLocalPos(a_vertex1).y())
        {
            m_vertex0 = a_vertex0;
            m_vertex1 = a_vertex1;
        }
        else if (m_parent->m_vertices->getLocalPos(a_vertex0).y() < m_parent->m_vertices->getLocalPos(a_vertex1).y())
        {
            m_vertex1 = a_vertex0;
            m_vertex0 = a_vertex1;
        }
        else
        {
            if (m_parent->m_vertices->getLocalPos(a_vertex0).z() > m_parent->m_vertices->getLocalPos(a_vertex1).z())
            {
                m_vertex0 = a_vertex0;
                m_vertex1 = a_vertex1;
            }
            else if (m_parent->m_vertices->getLocalPos(a_vertex0).z() < m_parent->m_vertices->getLocalPos(a_vertex1).z())
            {
                m_vertex1 = a_vertex0;
                m_vertex0 = a_vertex1;
            }
            else
            {
                m_vertex0 = a_vertex0;
                m_vertex1 = a_vertex1;
            }
        }
    }
}

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
