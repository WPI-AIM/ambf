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
    \version   3.2.0 $Rev: 2173 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CPrimitives.h"
//------------------------------------------------------------------------------
#include "graphics/CTriangleArray.h"
#include "math/CBezier.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function creates a plane by defining its size along the X and Y axes. 
    Texture coordinates are defined so that the bitmap image maps the entire
    plane.

    \param  a_mesh     Mesh object in which primitive is created.
    \param  a_lengthX  Size along X axis.
    \param  a_lengthY  Size along Y axis.
    \param  a_pos      Position where to build the new primitive.
    \param  a_rot      Orientation of the new primitive.
    \param  a_color    Color of vertices.
*/
//==============================================================================
void cCreatePlane(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreatePlane2(a_mesh,
                  a_lengthX,
                  a_lengthY,
                  a_pos,
                  a_rot,
                  a_color,
                  a_color,
                  a_color,
                  a_color);
}


//==============================================================================
/*!
    This function creates a plane by defining its size along the X and Y axes.
    Texture coordinates are defined so that the bitmap image maps the entire 
    plane.

    \param  a_mesh              Mesh object in which primitive is created.
    \param  a_lengthX           Size along X axis.
    \param  a_lengthY           Size along Y axis.
    \param  a_pos               Position where to build the new primitive.
    \param  a_rot               Orientation of the new primitive.
    \param  a_colorTopLeft      Color of top left vertex.
    \param  a_colorTopRight     Color of top right vertex.
    \param  a_colorBottomLeft   Color of bottom left vertex.
    \param  a_colorBottomRight  Color of bottom right vertex.
*/
//==============================================================================
void cCreatePlane2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_colorTopLeft,
    const cColorf& a_colorTopRight,
    const cColorf& a_colorBottomLeft,
    const cColorf& a_colorBottomRight)
{
    // sanity check
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // compute half edges
    double half_length_X = 0.5 * a_lengthX;
    double half_length_Y = 0.5 * a_lengthY;;

    // compute position of vertices
    cVector3d v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y, 0.0)));
    cVector3d v1 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y, 0.0)));
    cVector3d v2 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y, 0.0)));
    cVector3d v3 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y, 0.0)));

    // compute surface normal
    cVector3d n = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // create new vertices
    int vertexIndex0 = a_mesh->newVertex(v0);
    int vertexIndex1 = a_mesh->newVertex(v1);
    int vertexIndex2 = a_mesh->newVertex(v2);
    int vertexIndex3 = a_mesh->newVertex(v3);

    // vertex (bottom left)
    a_mesh->m_vertices->setNormal(vertexIndex0, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex0, 0.0, 0.0);
    a_mesh->m_vertices->setColor(vertexIndex0, a_colorBottomLeft);

    // vertex (bottom right)
    a_mesh->m_vertices->setNormal(vertexIndex1, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex1, 1.0, 0.0);
    a_mesh->m_vertices->setColor(vertexIndex1, a_colorBottomRight);

    // vertex (top right)
    a_mesh->m_vertices->setNormal(vertexIndex2, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex2, 1.0, 1.0);
    a_mesh->m_vertices->setColor(vertexIndex2, a_colorTopRight);

    // vertex (top left)
    a_mesh->m_vertices->setNormal(vertexIndex3, n);
    a_mesh->m_vertices->setTexCoord(vertexIndex3, 0.0, 1.0);
    a_mesh->m_vertices->setColor(vertexIndex3, a_colorTopLeft);

    // create triangles
    a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    a_mesh->newTriangle(vertexIndex0, vertexIndex2, vertexIndex3);
}


//==============================================================================
/*!
    This function creates a 2D map by defining the size along the X and Y axes, 
    and the number of sides along each axis. For instance, a map containing 3 
    sides along the X axis and 4 sides along the Y axis will contains: 12 squares 
    composed each of two triangles. The total number of vertices will be equal 
    to 20, respectively (3+1)x(4+1). By modifying the Z component of each vertex 
    you can easily create height maps for instance.Texture coordinates are defined 
    so that the bitmap image covers the entire map.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_lengthX    Size along X axis.
    \param  a_lengthY    Size along Y axis.
    \param  a_numSidesX  Number of elements along X axis.
    \param  a_numSidesY  Number of elements along Y axis.
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateMap(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const unsigned int a_numSidesX,
    const unsigned int a_numSidesY,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if ((a_numSidesX < 1) || (a_numSidesY < 1)) { return; }
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // compute half sides
    double half_length_X = a_lengthX / 2.0;
    double half_length_Y = a_lengthY / 2.0;

    // compute size of single element
    double size_X = a_lengthX / (double)a_numSidesX;
    double size_Y = a_lengthY / (double)a_numSidesY;

    // compute map normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // get current vertex index
    int index = a_mesh->getNumVertices();

    // create vertices
    int numVerticesX = a_numSidesX + 1;
    int numVerticesY = a_numSidesY + 1;
    for (int ix=0; ix<numVerticesX; ix++)
    {   
        double posX = -half_length_X + (double)(ix) * size_X;
        double textureCoordX = (double)(ix)/(double)(a_numSidesX);
        for (int iy=0; iy<numVerticesY; iy++)
        {   
            double posY = -half_length_Y + (double)(iy) * size_Y;
            cVector3d pos = cAdd(a_pos, cMul(a_rot, cVector3d(posX, posY, 0.0)));
            double textureCoordY = (double)(iy)/(double)(a_numSidesY);
            cVector3d textCoord(textureCoordX, textureCoordY, 0.0);
            a_mesh->newVertex(pos, normal, textCoord, a_color);
        }
    }

    // create triangles
    for (unsigned int ix=0; ix<a_numSidesX; ix++)
    {   
        for (unsigned int iy=0; iy<a_numSidesY; iy++)
        {   
            unsigned int index00 = index + (ix * numVerticesY) + iy;
            unsigned int index01 = index + (ix * numVerticesY) + (iy+1);
            unsigned int index10 = index + ((ix+1) * numVerticesY) + iy;
            unsigned int index11 = index + ((ix+1) * numVerticesY) + (iy+1);
            
            a_mesh->newTriangle(index00, index10, index11);
            a_mesh->newTriangle(index00, index11, index01);
        }
    } 
}



//==============================================================================
/*!
    This function creates a disk by defining its radius properties along axis X 
    and axis Y.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radiusX    Radius of sphere along axis X.
    \param  a_radiusY    Radius of sphere along axis Y.
    \param  a_numSlices  Specifies the number of slices composing the disc.
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateDisk(cMesh* a_mesh, 
    const double& a_radiusX, 
    const double& a_radiusY, 
    const unsigned int a_numSlices,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_numSlices < 3) { return; }
    if (a_radiusX <= 0) { return; }
    if (a_radiusY <= 0) { return; }

    // compute offset
    double delta_a = C_TWO_PI / (double)a_numSlices;

    // compute normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // create first vertex
    cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, 0.0)));
    unsigned int index = a_mesh->newVertex(p);

    // set texture coordinate
    a_mesh->m_vertices->setTexCoord(index, cVector3d(0.5, 0.5, 0.0));

    // set normal
    a_mesh->m_vertices->setNormal(index, normal);

    // create first round of points
    double angle = 0.0;
    for (unsigned int i=0; i<a_numSlices; i++)
    {   
        // set position
        p = cAdd(a_pos, cMul(a_rot, cVector3d(a_radiusX * cos(angle), a_radiusY * sin(angle), 0.0)));
        int vertexID = a_mesh->newVertex(p);

        // set normal
        a_mesh->m_vertices->setNormal(vertexID, normal);

        // set texture coordinate
        a_mesh->m_vertices->setTexCoord(vertexID, cVector3d(0.5 + 0.5 * cos(angle), 0.5 + 0.5 * sin(angle), 0.0));

        angle = angle + delta_a;
    }

    // create triangles
    for (unsigned int i=1; i<a_numSlices; i++)
    {   
        unsigned int index0 = index;
        unsigned int index1 = index + i;
        unsigned int index2 = index + i + 1;
        a_mesh->newTriangle(index0, index1, index2);
    }
    a_mesh->newTriangle(index, index + a_numSlices, index + 1);
}


//==============================================================================
/*!
    This function creates a panel with optional rounded corners.

    \param  a_mesh                  Mesh object in which primitive is created.
    \param  a_lengthX               Size along X axis.
    \param  a_lengthY               Size along Y axis.
    \param  a_radiusCorners         Radius of corners.
    \param  a_numSegmentsPerCorner  Number of segments per rounded corner.
    \param  a_pos                   Position where to build the new primitive.
    \param  a_rot                   Orientation of the new primitive.
    \param  a_color                 Color of vertices.
*/
//==============================================================================
void cCreatePanel(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_radiusCorners,
    const int& a_numSegmentsPerCorner,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreatePanel2(a_mesh, 
                  a_lengthX, 
                  a_lengthY, 
                  a_radiusCorners,
                  a_radiusCorners,
                  a_radiusCorners,
                  a_radiusCorners,
                  a_numSegmentsPerCorner,
                  a_pos,
                  a_rot,
                  a_color,
                  a_color,
                  a_color,
                  a_color);
}


//==============================================================================
/*!
    This function creates a panel with optional rounded corners.

    \param  a_mesh                     Mesh object in which primitive is created.
    \param  a_lengthX                  Size along X axis.
    \param  a_lengthY                  Size along Y axis.
    \param  a_cornerTopLeftRadius      Radius of top left corner.
    \param  a_cornerTopRightRadius     Radius of top right corner.
    \param  a_cornerBottomLeftRadius   Radius of bottom left corner.
    \param  a_cornerTopLeftRadius      Radius of bottom right corner.
    \param  a_cornerBottomRightRadius  Number of segments per rounded corner.
    \param  a_numSegmentsPerCorner     Number of segments composing the circular corners.
    \param  a_pos                      Position where to build the new primitive.
    \param  a_rot                      Orientation of the new primitive.
    \param  a_colorTopLeft             Color of top left vertex.
    \param  a_colorTopRight            Color of top right vertex.
    \param  a_colorBottomLeft          Color of bottom left vertex.
    \param  a_colorBottomRight         Color of bottom right vertex.
*/
//==============================================================================
void cCreatePanel2(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_cornerTopLeftRadius,
    const double& a_cornerTopRightRadius,
    const double& a_cornerBottomLeftRadius,
    const double& a_cornerBottomRightRadius,
    const int& a_numSegmentsPerCorner,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_colorTopLeft,
    const cColorf& a_colorTopRight,
    const cColorf& a_colorBottomLeft,
    const cColorf& a_colorBottomRight)
{
    // sanity check
    if (a_cornerTopLeftRadius < 0) { return; }
    if (a_cornerTopRightRadius < 0) { return; }
    if (a_cornerBottomLeftRadius < 0) { return; }
    if (a_cornerBottomRightRadius < 0) { return; }
    if (a_lengthX <= 0) { return; }
    if (a_lengthY <= 0) { return; }

    // sharp corners
    if ( (a_cornerTopLeftRadius == 0) &&
         (a_cornerTopRightRadius == 0) &&
         (a_cornerBottomLeftRadius == 0) &&
         (a_cornerBottomRightRadius == 0) )
    {
        cCreatePlane2(a_mesh,
                      a_lengthX,
                      a_lengthY,
                      a_pos,
                      a_rot,
                      a_colorTopLeft,
                      a_colorTopRight,
                      a_colorBottomLeft,
                      a_colorBottomRight);

        return;
    }

    // create a panel with corners
    double maxRadius = cMax(cMax(a_cornerTopLeftRadius, a_cornerTopRightRadius), 
                            cMax(a_cornerBottomLeftRadius, a_cornerBottomRightRadius));
    double minLength = cMin(a_lengthX, a_lengthY) / 2.0;
    maxRadius = cMin(maxRadius, minLength);

    // compute inner rectangle dimensions
    double hx0 = (a_lengthX / 2.0) - a_cornerTopRightRadius;
    double hy0 = (a_lengthY / 2.0) - a_cornerTopRightRadius;

    double hx1 = (a_lengthX / 2.0) - a_cornerTopLeftRadius;
    double hy1 = (a_lengthY / 2.0) - a_cornerTopLeftRadius;

    double hx2 = (a_lengthX / 2.0) - a_cornerBottomLeftRadius;
    double hy2 = (a_lengthY / 2.0) - a_cornerBottomLeftRadius;

    double hx3 = (a_lengthX / 2.0) - a_cornerBottomRightRadius;
    double hy3 = (a_lengthY / 2.0) - a_cornerBottomRightRadius;

    double sx = 0.5 / (a_lengthX / 2.0);
    double sy = 0.5 / (a_lengthY / 2.0);

    // compute surface normal
    cVector3d normal = cMul(a_rot, cVector3d(0.0, 0.0, 1.0));

    // setup variables
    int numVerticesPerCorner = a_numSegmentsPerCorner + 1;
    double deltaAng = C_PI_DIV_2 / (double(a_numSegmentsPerCorner));

    // create center vertex
    cVector3d pos = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, 0.0)));
    int vertexIndex0 = a_mesh->newVertex(pos);
    a_mesh->m_vertices->setNormal(vertexIndex0, normal);
    a_mesh->m_vertices->setTexCoord(vertexIndex0, 0.5, 0.5);

    // compute at center of panel
    cColorf color;
    color.setR(0.25f * (a_colorTopLeft.getR() + a_colorTopRight.getR() + a_colorBottomLeft.getR() + a_colorBottomRight.getR()));
    color.setG(0.25f * (a_colorTopLeft.getG() + a_colorTopRight.getG() + a_colorBottomLeft.getG() + a_colorBottomRight.getG()));
    color.setB(0.25f * (a_colorTopLeft.getB() + a_colorTopRight.getB() + a_colorBottomLeft.getB() + a_colorBottomRight.getB()));
    color.setA(0.25f * (a_colorTopLeft.getA() + a_colorTopRight.getA() + a_colorBottomLeft.getA() + a_colorBottomRight.getA()));
    a_mesh->m_vertices->setColor(vertexIndex0, color);

    // create vertices (top right corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (double)(i) * deltaAng;
        double x = hx0 + a_cornerTopRightRadius * cos(angle);
        double y = hy0 + a_cornerTopRightRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorTopRight);
    }

    // create vertices (top left corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (1 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x =-hx1 + a_cornerTopLeftRadius * cos(angle);
        double y = hy1 + a_cornerTopLeftRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorTopLeft);
    }

    // create vertices (bottom left corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (2 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x =-hx2 + a_cornerBottomLeftRadius * cos(angle);
        double y =-hy2 + a_cornerBottomLeftRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorBottomLeft);
    }

    // create vertices (bottom right corner)
    for (int i=0; i<numVerticesPerCorner; i++)
    {
        // compute position of vertex
        double angle = (3 * C_PI_DIV_2) + (double)(i) * deltaAng;
        double x = hx3 + a_cornerBottomRightRadius * cos(angle);
        double y =-hy3 + a_cornerBottomRightRadius * sin(angle);
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(x, y, 0.0)));

        // compute texture coordinate
        cVector3d texCoord(0.5 + sx*x, 0.5 + sy*y, 0.0);

        // create new vertex
        a_mesh->newVertex(p, normal, texCoord, a_colorBottomRight);
    }

    // create triangles
    int numTriangles = 4 * numVerticesPerCorner;
    int v = vertexIndex0 + 1;
    for (int i=0; i<numTriangles; i++)
    {
        int vertexIndex1 = v + i;
        int vertexIndex2 = v + ((i+1) % (numTriangles));

        a_mesh->newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
    }
}


//==============================================================================
/*!
    This function creates a box by defining its size along the x, y and z axis.
    Texture coordinates are defined so that the bitmap image is displayed
    on each face of the box.

    \param  a_mesh     Mesh object in which primitive is created.
    \param  a_lengthX  Size along X axis.
    \param  a_lengthY  Size along Y axis.
    \param  a_lengthZ  Size along Z axis.
    \param  a_pos      Position where to build the new primitive.
    \param  a_rot      Orientation of the new primitive.
    \param  a_color    Color of vertices.
*/
//==============================================================================
void cCreateBox(cMesh* a_mesh, 
    const double& a_lengthX, 
    const double& a_lengthY, 
    const double& a_lengthZ,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_lengthX < 0) { return; }
    if (a_lengthY < 0) { return; }
    if (a_lengthZ < 0) { return; }

    // compute half edges
    double half_length_X = a_lengthX / 2.0;
    double half_length_Y = a_lengthY / 2.0;
    double half_length_Z = a_lengthZ / 2.0;

    // create texture coordinates
    cVector3d t00(0.0, 0.0, 0.0);
    cVector3d t10(1.0, 0.0, 0.0);
    cVector3d t01(0.0, 1.0, 0.0);
    cVector3d t11(1.0, 1.0, 0.0);

    // compute position of vertices
    cVector3d v000 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y, -half_length_Z)));
    cVector3d v100 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y, -half_length_Z)));
    cVector3d v110 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y, -half_length_Z)));
    cVector3d v010 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y, -half_length_Z)));
    cVector3d v001 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X, -half_length_Y,  half_length_Z)));
    cVector3d v101 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X, -half_length_Y,  half_length_Z)));
    cVector3d v111 = cAdd(a_pos, cMul(a_rot, cVector3d( half_length_X,  half_length_Y,  half_length_Z)));
    cVector3d v011 = cAdd(a_pos, cMul(a_rot, cVector3d(-half_length_X,  half_length_Y,  half_length_Z)));

    // compute normals
    cVector3d nx0 = cMul(a_rot, cVector3d(-1.0, 0.0, 0.0));
    cVector3d nx1 = cMul(a_rot, cVector3d( 1.0, 0.0, 0.0));
    cVector3d ny0 = cMul(a_rot, cVector3d( 0.0,-1.0, 0.0));
    cVector3d ny1 = cMul(a_rot, cVector3d( 0.0, 1.0, 0.0));
    cVector3d nz0 = cMul(a_rot, cVector3d( 0.0, 0.0,-1.0));
    cVector3d nz1 = cMul(a_rot, cVector3d( 0.0, 0.0, 1.0));

    // create triangles
    a_mesh->newTriangle(v011, v010, v000, nx0, nx0, nx0, t11, t10, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v011, v000, v001, nx0, nx0, nx0, t11, t00, t01, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v100, v110, nx1, nx1, nx1, t01, t00, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v110, v111, nx1, nx1, nx1, t01, t10, t11, a_color, a_color, a_color);

    a_mesh->newTriangle(v101, v001, v000, ny0, ny0, ny0, t11, t01, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v101, v000, v100, ny0, ny0, ny0, t11, t00, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v111, v110, v010, ny1, ny1, ny1, t11, t10, t00, a_color, a_color, a_color);
    a_mesh->newTriangle(v111, v010, v011, ny1, ny1, ny1, t11, t00, t01, a_color, a_color, a_color);

    a_mesh->newTriangle(v000, v010, v110, nz0, nz0, nz0, t00, t01, t11, a_color, a_color, a_color);
    a_mesh->newTriangle(v000, v110, v100, nz0, nz0, nz0, t00, t11, t10, a_color, a_color, a_color);
    a_mesh->newTriangle(v001, v101, v111, nz1, nz1, nz1, t00, t10, t11, a_color, a_color, a_color);
    a_mesh->newTriangle(v001, v111, v011, nz1, nz1, nz1, t00, t11, t01, a_color, a_color, a_color);
}


//==============================================================================
/*!
    This function creates a cylinder by defining its radius and height. 
    The user may  also decide if the top and bottom discs should be included. 
    Texture coordinates are defined so that the bitmap image wraps around 
    the cylinder. The texture coordinates for the top part of the cylinder 
    are set to (0.0, 0.0, 0.0). The texture coordinates for the bottom part of 
    the cylinder are set to (1.0, 1.0, 0.0). When texture is enabled, 
    the colors defining the top and bottoms sections of the cylinder 
    are defined by both texels.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of the cylinder.
    \param  a_radius             Radius of the cylinder.
    \param  a_numSides           Number of sides composing the cylinder.
    \param  a_numHeightSegments  Number of segments along the cylinder axis.
    \param  a_numRings           Number of rings that compose the top and bottom disks.
    \param  a_includeTop         If __true__, then the top disc is included.
    \param  a_includeBottom      If __true__, then the bottom disc is included.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreateCylinder(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_radius,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const unsigned int a_numRings,
    const bool a_includeTop,
    const bool a_includeBottom,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreateCone(a_mesh, 
                a_height,  
                a_radius,
                a_radius,
                a_numSides,
                a_numHeightSegments,
                a_numRings,
                a_includeBottom,
                a_includeTop,
                a_pos,
                a_rot,
                a_color);
}


//==============================================================================
/*!
    This function creates a cone by defining its height and bottom radius. By defining 
    a top radius larger than zero, it is possible to create a truncated
    cone. Top and bottom parts can also be included or not.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of cone.
    \param  a_radiusBottom       Bottom radius of cone.
    \param  a_radiusTop          Top radius of cone. Apply 0 value for non truncated cone.
    \param  a_numSides           Number of sides composing the cone.
    \param  a_numHeightSegments  Number of segments along the cone axis.
    \param  a_numRings           Number of rings that compose the top and bottom disks.
    \param  a_includeTop         If __true__, then the top disc is included. (truncated cone)
    \param  a_includeBottom      If __true__, then the bottom disc is included.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreateCone(cMesh* a_mesh,
    const double& a_height,
    const double& a_radiusBottom,
    const double& a_radiusTop,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const unsigned int a_numRings,
    const bool a_includeBottom,
    const bool a_includeTop,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numH = cMax((unsigned int)1, a_numHeightSegments);

    // compute shared values
    double deltaAng = C_TWO_PI / (double)numS;
    double deltaLen = a_height    / (double)numH;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    if (a_height > 0.0)
    {
        // create vertices
        double nz = (a_radiusBottom - a_radiusTop) / a_height;
        double deltaRadius = (a_radiusTop - a_radiusBottom) / ((double)(numH));
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d np = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, nz));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                double radius = a_radiusBottom + (double)(j) * deltaRadius; 
                np.mulr(radius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }
    }

     // build cylinder bottom - create vertices and triangles
    if ((a_includeBottom) && (a_radiusBottom > 0.0))
    {
        double radius = a_radiusBottom / (double)(cMax((unsigned int)1, a_numRings));
        cVector3d t(0.0, 0.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,-1));
        cVector3d p = a_pos; 
        unsigned int vertex0 = a_mesh->newVertex(p, n, t, a_color);

        for (unsigned int i=0; i<numS; i++)
        {
            double ang = -(double)(i) * deltaAng;
            p = cAdd(a_pos, cMul(a_rot, cVector3d(radius * cos(ang), radius * sin(ang), 0.0)));
            a_mesh->newVertex(p, n, t, a_color);
        }

        vertexBaseID = vertex0 + 1;
        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex1 = vertexBaseID + i;
            unsigned int vertex2 = vertexBaseID + (i + 1)%numS; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
        }

        if (a_numRings > 1)
        {
            int rings = a_numRings - 1;
            for (int i=0; i<rings; i++)
            {
                // create vertices
                for (unsigned int j=0; j<numS; j++)
                {
                    double ang = -(double)(j) * deltaAng;
                    p = cAdd(a_pos, cMul(a_rot, cVector3d((i+2) * radius * cos(ang), (i+2) * radius * sin(ang), 0.0)));
                    a_mesh->newVertex(p, n, t, a_color);
                }

                // create triangles
                for (unsigned int j=0; j<numS; j++)
                {
                    unsigned int vertex00 = vertexBaseID + j;
                    unsigned int vertex01 = vertexBaseID + (j + 1)%numS;
                    unsigned int vertex10 = vertexBaseID + numS + j;
                    unsigned int vertex11 = vertexBaseID + numS + (j + 1)%numS; 
                    a_mesh->newTriangle(vertex00, vertex11, vertex01);
                    a_mesh->newTriangle(vertex00, vertex10, vertex11);
                }

                vertexBaseID = vertexBaseID + numS;
            }
        }
    }

    // build cylinder top - create vertices and triangles
    if ((a_includeTop) && (a_radiusTop > 0.0))
    {
        double radius = a_radiusTop / (double)(cMax((unsigned int)1, a_numRings));
        cVector3d t(1.0, 1.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,1));
        cVector3d p = cAdd(a_pos, cMul(a_rot, cVector3d(0,0,a_height))); 
        unsigned int vertex0 = a_mesh->newVertex(p, n, t, a_color);

        for (unsigned int i=0; i<numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            p = cAdd(a_pos, cMul(a_rot, cVector3d(radius * cos(ang), radius * sin(ang), a_height)));
            a_mesh->newVertex(p, n, t, a_color);
        }

        vertexBaseID = vertex0 + 1;
        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex1 = vertexBaseID + i;
            unsigned int vertex2 = vertexBaseID + (i + 1)%numS; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
        }

        if (a_numRings > 1)
        {
            int rings = a_numRings - 1;
            for (int i=0; i<rings; i++)
            {
                // create vertices
                for (unsigned int j=0; j<numS; j++)
                {
                    double ang = (double)(j) * deltaAng;
                    p = cAdd(a_pos, cMul(a_rot, cVector3d((i+2) * radius * cos(ang), (i+2) * radius * sin(ang), a_height)));
                    a_mesh->newVertex(p, n, t, a_color);
                }

                // create triangles
                for (unsigned int j=0; j<numS; j++)
                {
                    unsigned int vertex00 = vertexBaseID + j;
                    unsigned int vertex01 = vertexBaseID + (j + 1)%numS;
                    unsigned int vertex10 = vertexBaseID + numS + j;
                    unsigned int vertex11 = vertexBaseID + numS + (j + 1)%numS; 
                    a_mesh->newTriangle(vertex00, vertex11, vertex01);
                    a_mesh->newTriangle(vertex00, vertex10, vertex11);
                }

                vertexBaseID = vertexBaseID + numS;
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a pipe by defining a height, inner radius and outer 
    radius.

    \param  a_mesh               Mesh object in which primitive is created.
    \param  a_height             Height of the pipe.
    \param  a_innerRadius        Inner radius of the pipe.
    \param  a_outerRadius        Outer radius of the pipe.
    \param  a_numSides           Number of sides composing the pipe.
    \param  a_numHeightSegments  Number of segments along the cylinder axis.
    \param  a_pos                Position where to build the new primitive.
    \param  a_rot                Orientation of the new primitive.
    \param  a_color              Color of vertices.
*/
//==============================================================================
void cCreatePipe(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides,
    const unsigned int a_numHeightSegments,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numH = cMax((unsigned int)1, a_numHeightSegments);
    double innerRadius = cMin(a_innerRadius, a_outerRadius);
    double outerRadius = cMax(a_innerRadius, a_outerRadius);

    // compute shared values
    double deltaAng = C_TWO_PI / (double)numS;
    double deltaLen = a_height    / (double)numH;

    // get vertex base id
    int vertexBaseID;

    if (a_height > 0.0)
    {
        vertexBaseID = a_mesh->getNumVertices();

        // create cylinder vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d n = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, 0.0));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                n.mulr(outerRadius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create cylinder triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }

        vertexBaseID = a_mesh->getNumVertices();

        // create cylinder vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p;
            double ang = (double)(i) * deltaAng;
            double cosAng = cos(ang);
            double sinAng = sin(ang);
            cVector3d n = cVector3d(cosAng, sinAng, 0.0);
            cVector3d n_ = cMul(a_rot, cVector3d(cosAng, sinAng, 0.0));
            n_.normalize();

            for (unsigned int j=0; j<=numH; j++)
            {
                cVector3d offset(0.0, 0.0, (double)j * deltaLen);
                n.mulr(innerRadius, p);
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p + offset));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create cylinder triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index11, index01);
                a_mesh->newTriangle(index00, index10, index11);
            }
        }
    }

    // create extremities
    double deltaRadius = outerRadius - innerRadius;
    if (deltaRadius > 0.0)
    {
        // create top
        vertexBaseID = a_mesh->getNumVertices();

        cVector3d t(0.0, 0.0, 0.0);
        cVector3d n = cMul(a_rot, cVector3d(0,0,-1));

        for (unsigned int i=0; i<=numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            cVector3d p0 = cAdd(a_pos, cMul(a_rot, cVector3d(outerRadius * cos(ang), outerRadius * sin(ang), 0.0))); 
            cVector3d p1 = cAdd(a_pos, cMul(a_rot, cVector3d(innerRadius * cos(ang), innerRadius * sin(ang), 0.0))); 
            a_mesh->newVertex(p0, n, t, a_color);
            a_mesh->newVertex(p1, n, t, a_color);
        }

        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex0 = vertexBaseID + (2*i);
            unsigned int vertex1 = vertexBaseID + (2*i) + 1; 
            unsigned int vertex2 = vertexBaseID + (2*i) + 2;
            unsigned int vertex3 = vertexBaseID + (2*i) + 3; 
            a_mesh->newTriangle(vertex0, vertex1, vertex2);
            a_mesh->newTriangle(vertex1, vertex3, vertex2);
        }

        // create bottom
        vertexBaseID = a_mesh->getNumVertices();

        t.set(1.0, 1.0, 0.0);
        n = cMul(a_rot, cVector3d(0,0,1));

        for (unsigned int i=0; i<=numS; i++)
        {
            double ang = (double)(i) * deltaAng;
            cVector3d p0 = cAdd(a_pos, cMul(a_rot, cVector3d(outerRadius * cos(ang), outerRadius * sin(ang), a_height))); 
            cVector3d p1 = cAdd(a_pos, cMul(a_rot, cVector3d(innerRadius * cos(ang), innerRadius * sin(ang), a_height))); 
            a_mesh->newVertex(p0, n, t, a_color);
            a_mesh->newVertex(p1, n, t, a_color);
        }

        for (unsigned int i=0; i<numS; i++)
        {
            unsigned int vertex0 = vertexBaseID + (2*i);
            unsigned int vertex1 = vertexBaseID + (2*i) + 1; 
            unsigned int vertex2 = vertexBaseID + (2*i) + 2;
            unsigned int vertex3 = vertexBaseID + (2*i) + 3; 
            a_mesh->newTriangle(vertex0, vertex2, vertex1);
            a_mesh->newTriangle(vertex1, vertex2, vertex3);
        }
    }
}


//==============================================================================
/*!
    This function creates sphere by defining its radius.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radius     Radius of sphere.
    \param  a_numSlices  Specifies the number of subdivisions around the z axis (similar to lines of longitude).
    \param  a_numStacks  Specifies the number of subdivisions along the z axis (similar to lines of latitude).
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateSphere(cMesh* a_mesh, 
    const double& a_radius,  
    const unsigned int a_numSlices,   
    const unsigned int a_numStacks,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    cCreateEllipsoid(a_mesh, 
                     a_radius, 
                     a_radius, 
                     a_radius, 
                     a_numSlices, 
                     a_numStacks, 
                     a_pos, 
                     a_rot,
                     a_color);
}


//==============================================================================
/*!
    This function creates an ellipsoid by defining the radius properties along 
    each axis X, Y and Z.

    \param  a_mesh       Mesh object in which primitive is created.
    \param  a_radiusX    Radius along X axis.
    \param  a_radiusY    Radius along Y axis.
    \param  a_radiusZ    Radius along Z axis.
    \param  a_numSlices  Specifies the number of subdivisions around the z axis (similar to lines of longitude).
    \param  a_numStacks  Specifies the number of subdivisions along the z axis (similar to lines of latitude).
    \param  a_pos        Position where to build the new primitive.
    \param  a_rot        Orientation of the new primitive.
    \param  a_color      Color of vertices.
*/
//==============================================================================
void cCreateEllipsoid(cMesh* a_mesh, 
    const double& a_radiusX,
    const double& a_radiusY,
    const double& a_radiusZ,
    const unsigned int a_numSlices,
    const unsigned int a_numStacks,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSlices);
    unsigned int numH = cMax((unsigned int)2, a_numStacks);

    if (a_radiusX <= 0.0) { return; }
    if (a_radiusY <= 0.0) { return; }
    if (a_radiusZ <= 0.0) { return; }

    // compute shared values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngH = C_PI / (double)(numH);
    double cX = 1.0 / a_radiusX;
    double cY = 1.0 / a_radiusY;
    double cZ = 1.0 / a_radiusZ;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    if (a_radiusZ > 0.0)
    {
        // create vertices
        for (unsigned int i=0; i<=numS; i++)
        {
            cVector3d p, n;
            double ang = (double)(i) * deltaAngS;
            double cosAng = cos(ang);
            double sinAng = sin(ang);

            for (unsigned int j=0; j<=numH; j++)
            {
                double angH = -C_PI_DIV_2 + (double)(j) * deltaAngH;
                double sinAngH = sin(angH);
                double cosAngH = cos(angH);
                p.set(a_radiusX * cosAng * cosAngH, a_radiusY * sinAng * cosAngH, a_radiusZ * sinAngH);
                n.set(cX * cosAng * cosAngH, cY * sinAng * cosAngH, cZ * sinAngH);
                n.normalize();
                cVector3d n_ = cMul(a_rot, n);
                n_.normalize();
                cVector3d p_ = cAdd(a_pos, cMul(a_rot, p));
                cVector3d t_((double)(i)/(double)(numS), (double)(j)/(double)(numH), 0.0);
                a_mesh->newVertex(p_, n_, t_, a_color);
            }
        }

        // create triangles
        for (unsigned int i=0; i<numS; i++)
        {
            for (unsigned int j=0; j<numH; j++)
            {

                int index00 = vertexBaseID + ((i  ) * (numH+1)) + j;
                int index01 = vertexBaseID + ((i+1) * (numH+1)) + j;
                int index10 = vertexBaseID + ((i  ) * (numH+1)) + j+1;
                int index11 = vertexBaseID + ((i+1) * (numH+1)) + j+1;
                a_mesh->newTriangle(index00, index01, index11);
                a_mesh->newTriangle(index00, index11, index10);
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a torus by defining the inner and outer radius values.

    \param  a_mesh         Mesh object in which primitive is created.
    \param  a_innerRadius  Inner radius of the torus.
    \param  a_outerRadius  Outer radius of the torus.
    \param  a_numSides     Number of sides for each radial section.
    \param  a_numRings     Number of radial divisions for the torus.
    \param  a_pos          Position where to build the new primitive.
    \param  a_rot          Orientation of the new primitive.
    \param  a_color        Color of vertices.
*/
//==============================================================================
void cCreateRing(cMesh* a_mesh,
    const double& a_innerRadius,
    const double& a_outerRadius,
    const unsigned int a_numSides,
    const unsigned int a_numRings,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check 
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numR = cMax((unsigned int)3, a_numRings);
    if (a_innerRadius < 0.0) { return; }
    if (a_outerRadius < 0.0) { return; }

    // compute step values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngR = C_TWO_PI / (double)numR;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    // create vertices
    for (unsigned int i=0; i<=numR; i++)
    {
        double angR = (double)(i) * deltaAngR;
        double cosAngR = cos(angR);
        double sinAngR = sin(angR);
        cVector3d pos(-a_outerRadius * sinAngR, a_outerRadius * cosAngR, 0.0);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisRad(cVector3d(0.0, 0.0, 1.0), angR);

        for (unsigned int j=0; j<=numS; j++)
        {
            double angS = (double)(j) * deltaAngS;
            double cosAngS = cos(angS);
            double sinAngS = sin(angS);
            cVector3d p_ = cAdd(a_pos, cMul(a_rot, cAdd(pos, cMul(rot, cVector3d(0.0, a_innerRadius * cosAngS, a_innerRadius * sinAngS)))));
            cVector3d n_ = cMul(a_rot, cMul(rot, cVector3d(0.0, cosAngS, sinAngS)));
            cVector3d t_((double)(j)/(double)(numS), (double)(i)/(double)(numR), 0.0);
            a_mesh->newVertex(p_, n_, t_, a_color);
        }
    }

    // create triangles
    for (unsigned int i=0; i<numR; i++)
    {
        for (unsigned int j=0; j<numS; j++)
        {
            int index00 = vertexBaseID + ((i  ) * (numS+1)) + j;
            int index01 = vertexBaseID + ((i+1) * (numS+1)) + j;
            int index10 = vertexBaseID + ((i  ) * (numS+1)) + j+1;
            int index11 = vertexBaseID + ((i+1) * (numS+1)) + j+1;
            a_mesh->newTriangle(index00, index01, index11);
            a_mesh->newTriangle(index00, index11, index10);
        }
    }

}


//==============================================================================
/*!
    This function creates a torus by defining the inner and outer radius values.

    \param  a_mesh                   Mesh object in which primitive is created.
    \param  a_innerRadius0           Inner radius of the ring at the beginning extremity.
    \param  a_innerRadius1           Inner radius of the ring at the end extremity.
    \param  a_outerRadius            Outer radius of the torus.
    \param  a_coverageAngleDEG       Coverage angle in degrees (from 0 to 360).
    \param  a_includeExtremityFaces  Include flat surfaces at extremities of ring section.
    \param  a_numSides               Number of sides for each radial section.
    \param  a_numRings               Number of radial divisions for the torus.
    \param  a_pos                    Position where to build the new primitive.
    \param  a_rot                    Orientation of the new primitive.
    \param  a_color                  Color of vertices.
*/
//==============================================================================
void cCreateRingSection(cMesh* a_mesh, 
    const double& a_innerRadius0,
    const double& a_innerRadius1,
    const double& a_outerRadius,    
    const double& a_coverageAngleDEG,
    const bool a_includeExtremityFaces,
    const unsigned int a_numSides,   
    const unsigned int a_numRings,  
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    unsigned int numS = cMax((unsigned int)3, a_numSides);
    unsigned int numR = cMax((unsigned int)3, a_numRings);

    double coverageAngle = cClamp(a_coverageAngleDEG, 0.0, 360.0);
    if (coverageAngle == 0.0) { return; }
    
    bool includeExtremityFaces = a_includeExtremityFaces;
    if (coverageAngle == 360)
    {
        includeExtremityFaces = false;
    }
    
    if (a_innerRadius0 < 0.0) { return; }
    if (a_innerRadius1 < 0.0) { return; }
    if (a_outerRadius < 0.0) { return; }

    // compute step values
    double deltaAngS = C_TWO_PI / (double)numS;
    double deltaAngR = (coverageAngle / 360) * (C_TWO_PI / (double)numR);
    double deltaInnerRadius = (a_innerRadius1 - a_innerRadius0) / a_numRings;

    // get vertex base id
    int vertexBaseID = a_mesh->getNumVertices();

    // create vertices
    for (unsigned int i=0; i<=numR; i++)
    {
        double angR = (double)(i) * deltaAngR;
        double cosAngR = cos(angR);
        double sinAngR = sin(angR);
        cVector3d pos(-a_outerRadius * sinAngR, a_outerRadius * cosAngR, 0.0);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisRad(cVector3d(0.0, 0.0, 1.0), angR);

        double innerRadius = a_innerRadius0 + deltaInnerRadius * i;
        for (unsigned int j=0; j<=numS; j++)
        {
            double angS = (double)(j) * deltaAngS;
            double cosAngS = cos(angS);
            double sinAngS = sin(angS);
            cVector3d p_ = cAdd(a_pos, cMul(a_rot, cAdd(pos, cMul(rot, cVector3d(0.0, innerRadius * cosAngS, innerRadius * sinAngS)))));
            cVector3d n_ = cMul(a_rot, cMul(rot, cVector3d(0.0, cosAngS, sinAngS)));
            cVector3d t_((double)(j)/(double)(numS), (double)(i)/(double)(numR), 0.0);
            a_mesh->newVertex(p_, n_, t_, a_color);
        }
    }

    // create ring triangles 
    for (unsigned int i=0; i<numR; i++)
    {
        for (unsigned int j=0; j<numS; j++)
        {
            int index00 = vertexBaseID + ((i  ) * (numS+1)) + j;
            int index01 = vertexBaseID + ((i+1) * (numS+1)) + j;
            int index10 = vertexBaseID + ((i  ) * (numS+1)) + j+1;
            int index11 = vertexBaseID + ((i+1) * (numS+1)) + j+1;
            a_mesh->newTriangle(index00, index01, index11);
            a_mesh->newTriangle(index00, index11, index10);
        }
    }

    // create extremity triangles
    if (includeExtremityFaces)
    {
        // extremity 0:
        if (a_innerRadius0 > 0.0)
        {
            for (unsigned int j=0; j<numS; j++)
            {
                int index0 = vertexBaseID;
                int index1 = vertexBaseID + j;
                int index2 = vertexBaseID + j+1;
                cVector3d pos0 = a_mesh->m_vertices->getLocalPos(index0);
                cVector3d pos1 = a_mesh->m_vertices->getLocalPos(index1);
                cVector3d pos2 = a_mesh->m_vertices->getLocalPos(index2);
                int triangleIndex = a_mesh->newTriangle(pos0, pos1, pos2);
                a_mesh->m_triangles->computeNormal(triangleIndex, true);
            }
        }

        // extremity 1:
        if (a_innerRadius1 > 0.0)
        {
            for (unsigned int j=0; j<numS; j++)
            {
                int index0 = vertexBaseID + (numR * numS);
                int index1 = vertexBaseID + (numR * numS) + j;
                int index2 = vertexBaseID + (numR * numS) + j+1;
                cVector3d pos0 = a_mesh->m_vertices->getLocalPos(index0);
                cVector3d pos1 = a_mesh->m_vertices->getLocalPos(index1);
                cVector3d pos2 = a_mesh->m_vertices->getLocalPos(index2);
                int triangleIndex = a_mesh->newTriangle(pos0, pos1, pos2);
                a_mesh->m_triangles->computeNormal(triangleIndex, true);
            }
        }
    }
}


//==============================================================================
/*!
    This function creates a square pyramid.

    \param  a_mesh           Mesh object in which primitive is created.
    \param  a_height         Height of square pyramid.
    \param  a_baseSize       Size of a base of the pyramid.
    \param  a_includeBottom  If __true__, then the bottom (square) of the pyramid is included.
    \param  a_pos            Position where to build the new primitive.
    \param  a_rot            Orientation of the new primitive.
    \param  a_color          Color of vertices.
*/
//==============================================================================
void cCreateSquarePyramid(cMesh* a_mesh, 
    const double& a_height,  
    const double& a_baseSize,
    const bool a_includeBottom,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // sanity check
    if (a_baseSize == 0.0) { return; }

    // temp variables
    cVector3d v0, v1, v2, v3;
    cVector3d t0, t1, t2, t3;
    cVector3d n;
    double s = a_baseSize / 2.0;

    // create texture coordinates
    t0.set(0.0, 0.0, 0.0);
    t1.set(1.0, 0.0, 0.0);
    t2.set(0.5, 1.0, 0.0);

    // create face 1
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
    v2 = cAdd(a_pos, cMul(a_rot, cVector3d(0.0, 0.0, a_height)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 2
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 3
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create face 4
    v0 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
    v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
    n = cMul(a_rot, cNormalize(cCross(cSub(v1, v0), cSub(v2, v1))));
    a_mesh->newTriangle(v0, v1, v2, n, n, n, t0, t1, t2, a_color, a_color, a_color);

    // create bottom
    if (a_includeBottom)
    {
        v0 = cAdd(a_pos, cMul(a_rot, cVector3d( s, -s, 0.0)));
        v1 = cAdd(a_pos, cMul(a_rot, cVector3d( s,  s, 0.0)));
        v2 = cAdd(a_pos, cMul(a_rot, cVector3d(-s,  s, 0.0)));
        v3 = cAdd(a_pos, cMul(a_rot, cVector3d(-s, -s, 0.0)));
        n = cMul(a_rot, cVector3d(0.0, 0.0,-1.0));

        t0.set(0.0, 1.0, 0.0);
        t1.set(1.0, 1.0, 0.0);
        t2.set(1.0, 0.0, 0.0);
        t3.set(0.0, 0.0, 0.0);

        a_mesh->newTriangle(v0, v3, v2, n, n, n, t0, t3, t2, a_color, a_color, a_color);
        a_mesh->newTriangle(v0, v2, v1, n, n, n, t0, t2, t1, a_color, a_color, a_color);
    }
}


//==============================================================================
/*!
    This function creates a copy of the famous OpenGL tea pot.

    \param  a_mesh        Mesh object in which primitive is created.
    \param  a_size        Size of the tea pot.
    \param  a_resolution  Number of divisions per patch.
    \param  a_pos         Position where to build the new primitive.
    \param  a_rot         Orientation of the new primitive.
    \param  a_color       Color of vertices.
*/
//==============================================================================
void cCreateTeaPot(cMesh* a_mesh,
    const double& a_size,
    const int& a_resolution,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // Teapot data

    const static int kTeapotNumPatches = 32;
    const static int kTeapotNumVertices = 306;
    const int teapotPatches[kTeapotNumPatches][16] = {
        {   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16 },
        {   4,  17,  18,  19,   8,  20,  21,  22,  12,  23,  24,  25,  16,  26,  27,  28 },
        {  19,  29,  30,  31,  22,  32,  33,  34,  25,  35,  36,  37,  28,  38,  39,  40 },
        {  31,  41,  42,   1,  34,  43,  44,   5,  37,  45,  46,   9,  40,  47,  48,  13 },
        {  13,  14,  15,  16,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60 },
        {  16,  26,  27,  28,  52,  61,  62,  63,  56,  64,  65,  66,  60,  67,  68,  69 },
        {  28,  38,  39,  40,  63,  70,  71,  72,  66,  73,  74,  75,  69,  76,  77,  78 },
        {  40,  47,  48,  13,  72,  79,  80,  49,  75,  81,  82,  53,  78,  83,  84,  57 },
        {  57,  58,  59,  60,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96 },
        {  60,  67,  68,  69,  88,  97,  98,  99,  92, 100, 101, 102,  96, 103, 104, 105 },
        {  69,  76,  77,  78,  99, 106, 107, 108, 102, 109, 110, 111, 105, 112, 113, 114 },
        {  78,  83,  84,  57, 108, 115, 116,  85, 111, 117, 118,  89, 114, 119, 120,  93 },
        { 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136 },
        { 124, 137, 138, 121, 128, 139, 140, 125, 132, 141, 142, 129, 136, 143, 144, 133 },
        { 133, 134, 135, 136, 145, 146, 147, 148, 149, 150, 151, 152,  69, 153, 154, 155 },
        { 136, 143, 144, 133, 148, 156, 157, 145, 152, 158, 159, 149, 155, 160, 161,  69 },
        { 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177 },
        { 165, 178, 179, 162, 169, 180, 181, 166, 173, 182, 183, 170, 177, 184, 185, 174 },
        { 174, 175, 176, 177, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197 },
        { 177, 184, 185, 174, 189, 198, 199, 186, 193, 200, 201, 190, 197, 202, 203, 194 },
        { 204, 204, 204, 204, 207, 208, 209, 210, 211, 211, 211, 211, 212, 213, 214, 215 },
        { 204, 204, 204, 204, 210, 217, 218, 219, 211, 211, 211, 211, 215, 220, 221, 222 },
        { 204, 204, 204, 204, 219, 224, 225, 226, 211, 211, 211, 211, 222, 227, 228, 229 },
        { 204, 204, 204, 204, 226, 230, 231, 207, 211, 211, 211, 211, 229, 232, 233, 212 },
        { 212, 213, 214, 215, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245 },
        { 215, 220, 221, 222, 237, 246, 247, 248, 241, 249, 250, 251, 245, 252, 253, 254 },
        { 222, 227, 228, 229, 248, 255, 256, 257, 251, 258, 259, 260, 254, 261, 262, 263 },
        { 229, 232, 233, 212, 257, 264, 265, 234, 260, 266, 267, 238, 263, 268, 269, 242 },
        { 270, 270, 270, 270, 279, 280, 281, 282, 275, 276, 277, 278, 271, 272, 273, 274 },
        { 270, 270, 270, 270, 282, 289, 290, 291, 278, 286, 287, 288, 274, 283, 284, 285 },
        { 270, 270, 270, 270, 291, 298, 299, 300, 288, 295, 296, 297, 285, 292, 293, 294 },
        { 270, 270, 270, 270, 300, 305, 306, 279, 297, 303, 304, 275, 294, 301, 302, 271 }
    };

    const double teapotVertices[kTeapotNumVertices][3] = {
        { 1.4000,  0.0000,  2.4000 }, { 1.4000, -0.7840,  2.4000 }, { 0.7840, -1.4000,  2.4000 },
        { 0.0000, -1.4000,  2.4000 }, { 1.3375,  0.0000,  2.5312 }, { 1.3375, -0.7490,  2.5312 },
        { 0.7490, -1.3375,  2.5312 }, { 0.0000, -1.3375,  2.5312 }, { 1.4375,  0.0000,  2.5312 },
        { 1.4375, -0.8050,  2.5312 }, { 0.8050, -1.4375,  2.5312 }, { 0.0000, -1.4375,  2.5312 }, 
        { 1.5000,  0.0000,  2.4000 }, { 1.5000, -0.8400,  2.4000 }, { 0.8400, -1.5000,  2.4000 },
        { 0.0000, -1.5000,  2.4000 }, {-0.7840, -1.4000,  2.4000 }, {-1.4000, -0.7840,  2.4000 },
        {-1.4000,  0.0000,  2.4000 }, {-0.7490, -1.3375,  2.5312 }, {-1.3375, -0.7490,  2.5312 },
        {-1.3375,  0.0000,  2.5312 }, {-0.8050, -1.4375,  2.5312 }, {-1.4375, -0.8050,  2.5312 },
        {-1.4375,  0.0000,  2.5312 }, {-0.8400, -1.5000,  2.4000 }, {-1.5000, -0.8400,  2.4000 },
        {-1.5000,  0.0000,  2.4000 }, {-1.4000,  0.7840,  2.4000 }, {-0.7840,  1.4000,  2.4000 },
        { 0.0000,  1.4000,  2.4000 }, {-1.3375,  0.7490,  2.5312 }, {-0.7490,  1.3375,  2.5312 },
        { 0.0000,  1.3375,  2.5312 }, {-1.4375,  0.8050,  2.5312 }, {-0.8050,  1.4375,  2.5312 },
        { 0.0000,  1.4375,  2.5312 }, {-1.5000,  0.8400,  2.4000 }, {-0.8400,  1.5000,  2.4000 },
        { 0.0000,  1.5000,  2.4000 }, { 0.7840,  1.4000,  2.4000 }, { 1.4000,  0.7840,  2.4000 },
        { 0.7490,  1.3375,  2.5312 }, { 1.3375,  0.7490,  2.5312 }, { 0.8050,  1.4375,  2.5312 },
        { 1.4375,  0.8050,  2.5312 }, { 0.8400,  1.5000,  2.4000 }, { 1.5000,  0.8400,  2.4000 },
        { 1.7500,  0.0000,  1.8750 }, { 1.7500, -0.9800,  1.8750 }, { 0.9800, -1.7500,  1.8750 },
        { 0.0000, -1.7500,  1.8750 }, { 2.0000,  0.0000,  1.3500 }, { 2.0000, -1.1200,  1.3500 },
        { 1.1200, -2.0000,  1.3500 }, { 0.0000, -2.0000,  1.3500 }, { 2.0000,  0.0000,  0.9000 },
        { 2.0000, -1.1200,  0.9000 }, { 1.1200, -2.0000,  0.9000 }, { 0.0000, -2.0000,  0.9000 },
        {-0.9800, -1.7500,  1.8750 }, {-1.7500, -0.9800,  1.8750 }, {-1.7500,  0.0000,  1.8750 },
        {-1.1200, -2.0000,  1.3500 }, {-2.0000, -1.1200,  1.3500 }, {-2.0000,  0.0000,  1.3500 },
        {-1.1200, -2.0000,  0.9000 }, {-2.0000, -1.1200,  0.9000 }, {-2.0000,  0.0000,  0.9000 },
        {-1.7500,  0.9800,  1.8750 }, {-0.9800,  1.7500,  1.8750 }, { 0.0000,  1.7500,  1.8750 },
        {-2.0000,  1.1200,  1.3500 }, {-1.1200,  2.0000,  1.3500 }, { 0.0000,  2.0000,  1.3500 },
        {-2.0000,  1.1200,  0.9000 }, {-1.1200,  2.0000,  0.9000 }, { 0.0000,  2.0000,  0.9000 },
        { 0.9800,  1.7500,  1.8750 }, { 1.7500,  0.9800,  1.8750 }, { 1.1200,  2.0000,  1.3500 },
        { 2.0000,  1.1200,  1.3500 }, { 1.1200,  2.0000,  0.9000 }, { 2.0000,  1.1200,  0.9000 },
        { 2.0000,  0.0000,  0.4500 }, { 2.0000, -1.1200,  0.4500 }, { 1.1200, -2.0000,  0.4500 },
        { 0.0000, -2.0000,  0.4500 }, { 1.5000,  0.0000,  0.2250 }, { 1.5000, -0.8400,  0.2250 },
        { 0.8400, -1.5000,  0.2250 }, { 0.0000, -1.5000,  0.2250 }, { 1.5000,  0.0000,  0.1500 },
        { 1.5000, -0.8400,  0.1500 }, { 0.8400, -1.5000,  0.1500 }, { 0.0000, -1.5000,  0.1500 },
        {-1.1200, -2.0000,  0.4500 }, {-2.0000, -1.1200,  0.4500 }, {-2.0000,  0.0000,  0.4500 },
        {-0.8400, -1.5000,  0.2250 }, {-1.5000, -0.8400,  0.2250 }, {-1.5000,  0.0000,  0.2250 },
        {-0.8400, -1.5000,  0.1500 }, {-1.5000, -0.8400,  0.1500 }, {-1.5000,  0.0000,  0.1500 },
        {-2.0000,  1.1200,  0.4500 }, {-1.1200,  2.0000,  0.4500 }, { 0.0000,  2.0000,  0.4500 },
        {-1.5000,  0.8400,  0.2250 }, {-0.8400,  1.5000,  0.2250 }, { 0.0000,  1.5000,  0.2250 },
        {-1.5000,  0.8400,  0.1500 }, {-0.8400,  1.5000,  0.1500 }, { 0.0000,  1.5000,  0.1500 },
        { 1.1200,  2.0000,  0.4500 }, { 2.0000,  1.1200,  0.4500 }, { 0.8400,  1.5000,  0.2250 },
        { 1.5000,  0.8400,  0.2250 }, { 0.8400,  1.5000,  0.1500 }, { 1.5000,  0.8400,  0.1500 },
        {-1.6000,  0.0000,  2.0250 }, {-1.6000, -0.3000,  2.0250 }, {-1.5000, -0.3000,  2.2500 },
        {-1.5000,  0.0000,  2.2500 }, {-2.3000,  0.0000,  2.0250 }, {-2.3000, -0.3000,  2.0250 },
        {-2.5000, -0.3000,  2.2500 }, {-2.5000,  0.0000,  2.2500 }, {-2.7000,  0.0000,  2.0250 },
        {-2.7000, -0.3000,  2.0250 }, {-3.0000, -0.3000,  2.2500 }, {-3.0000,  0.0000,  2.2500 },
        {-2.7000,  0.0000,  1.8000 }, {-2.7000, -0.3000,  1.8000 }, {-3.0000, -0.3000,  1.8000 },
        {-3.0000,  0.0000,  1.8000 }, {-1.5000,  0.3000,  2.2500 }, {-1.6000,  0.3000,  2.0250 },
        {-2.5000,  0.3000,  2.2500 }, {-2.3000,  0.3000,  2.0250 }, {-3.0000,  0.3000,  2.2500 },
        {-2.7000,  0.3000,  2.0250 }, {-3.0000,  0.3000,  1.8000 }, {-2.7000,  0.3000,  1.8000 },
        {-2.7000,  0.0000,  1.5750 }, {-2.7000, -0.3000,  1.5750 }, {-3.0000, -0.3000,  1.3500 },
        {-3.0000,  0.0000,  1.3500 }, {-2.5000,  0.0000,  1.1250 }, {-2.5000, -0.3000,  1.1250 },
        {-2.6500, -0.3000,  0.9375 }, {-2.6500,  0.0000,  0.9375 }, {-2.0000, -0.3000,  0.9000 },
        {-1.9000, -0.3000,  0.6000 }, {-1.9000,  0.0000,  0.6000 }, {-3.0000,  0.3000,  1.3500 },
        {-2.7000,  0.3000,  1.5750 }, {-2.6500,  0.3000,  0.9375 }, {-2.5000,  0.3000,  1.1250 },
        {-1.9000,  0.3000,  0.6000 }, {-2.0000,  0.3000,  0.9000 }, { 1.7000,  0.0000,  1.4250 },
        { 1.7000, -0.6600,  1.4250 }, { 1.7000, -0.6600,  0.6000 }, { 1.7000,  0.0000,  0.6000 },
        { 2.6000,  0.0000,  1.4250 }, { 2.6000, -0.6600,  1.4250 }, { 3.1000, -0.6600,  0.8250 },
        { 3.1000,  0.0000,  0.8250 }, { 2.3000,  0.0000,  2.1000 }, { 2.3000, -0.2500,  2.1000 },
        { 2.4000, -0.2500,  2.0250 }, { 2.4000,  0.0000,  2.0250 }, { 2.7000,  0.0000,  2.4000 },
        { 2.7000, -0.2500,  2.4000 }, { 3.3000, -0.2500,  2.4000 }, { 3.3000,  0.0000,  2.4000 },
        { 1.7000,  0.6600,  0.6000 }, { 1.7000,  0.6600,  1.4250 }, { 3.1000,  0.6600,  0.8250 },
        { 2.6000,  0.6600,  1.4250 }, { 2.4000,  0.2500,  2.0250 }, { 2.3000,  0.2500,  2.1000 },
        { 3.3000,  0.2500,  2.4000 }, { 2.7000,  0.2500,  2.4000 }, { 2.8000,  0.0000,  2.4750 },
        { 2.8000, -0.2500,  2.4750 }, { 3.5250, -0.2500,  2.4938 }, { 3.5250,  0.0000,  2.4938 },
        { 2.9000,  0.0000,  2.4750 }, { 2.9000, -0.1500,  2.4750 }, { 3.4500, -0.1500,  2.5125 },
        { 3.4500,  0.0000,  2.5125 }, { 2.8000,  0.0000,  2.4000 }, { 2.8000, -0.1500,  2.4000 },
        { 3.2000, -0.1500,  2.4000 }, { 3.2000,  0.0000,  2.4000 }, { 3.5250,  0.2500,  2.4938 },
        { 2.8000,  0.2500,  2.4750 }, { 3.4500,  0.1500,  2.5125 }, { 2.9000,  0.1500,  2.4750 },
        { 3.2000,  0.1500,  2.4000 }, { 2.8000,  0.1500,  2.4000 }, { 0.0000,  0.0000,  3.1500 },
        { 0.0000, -0.0020,  3.1500 }, { 0.0020,  0.0000,  3.1500 }, { 0.8000,  0.0000,  3.1500 },
        { 0.8000, -0.4500,  3.1500 }, { 0.4500, -0.8000,  3.1500 }, { 0.0000, -0.8000,  3.1500 },
        { 0.0000,  0.0000,  2.8500 }, { 0.2000,  0.0000,  2.7000 }, { 0.2000, -0.1120,  2.7000 },
        { 0.1120, -0.2000,  2.7000 }, { 0.0000, -0.2000,  2.7000 }, {-0.0020,  0.0000,  3.1500 },
        {-0.4500, -0.8000,  3.1500 }, {-0.8000, -0.4500,  3.1500 }, {-0.8000,  0.0000,  3.1500 },
        {-0.1120, -0.2000,  2.7000 }, {-0.2000, -0.1120,  2.7000 }, {-0.2000,  0.0000,  2.7000 },
        { 0.0000,  0.0020,  3.1500 }, {-0.8000,  0.4500,  3.1500 }, {-0.4500,  0.8000,  3.1500 },
        { 0.0000,  0.8000,  3.1500 }, {-0.2000,  0.1120,  2.7000 }, {-0.1120,  0.2000,  2.7000 },
        { 0.0000,  0.2000,  2.7000 }, { 0.4500,  0.8000,  3.1500 }, { 0.8000,  0.4500,  3.1500 },
        { 0.1120,  0.2000,  2.7000 }, { 0.2000,  0.1120,  2.7000 }, { 0.4000,  0.0000,  2.5500 },
        { 0.4000, -0.2240,  2.5500 }, { 0.2240, -0.4000,  2.5500 }, { 0.0000, -0.4000,  2.5500 },
        { 1.3000,  0.0000,  2.5500 }, { 1.3000, -0.7280,  2.5500 }, { 0.7280, -1.3000,  2.5500 },
        { 0.0000, -1.3000,  2.5500 }, { 1.3000,  0.0000,  2.4000 }, { 1.3000, -0.7280,  2.4000 },
        { 0.7280, -1.3000,  2.4000 }, { 0.0000, -1.3000,  2.4000 }, {-0.2240, -0.4000,  2.5500 },
        {-0.4000, -0.2240,  2.5500 }, {-0.4000,  0.0000,  2.5500 }, {-0.7280, -1.3000,  2.5500 },
        {-1.3000, -0.7280,  2.5500 }, {-1.3000,  0.0000,  2.5500 }, {-0.7280, -1.3000,  2.4000 },
        {-1.3000, -0.7280,  2.4000 }, {-1.3000,  0.0000,  2.4000 }, {-0.4000,  0.2240,  2.5500 },
        {-0.2240,  0.4000,  2.5500 }, { 0.0000,  0.4000,  2.5500 }, {-1.3000,  0.7280,  2.5500 },
        {-0.7280,  1.3000,  2.5500 }, { 0.0000,  1.3000,  2.5500 }, {-1.3000,  0.7280,  2.4000 },
        {-0.7280,  1.3000,  2.4000 }, { 0.0000,  1.3000,  2.4000 }, { 0.2240,  0.4000,  2.5500 },
        { 0.4000,  0.2240,  2.5500 }, { 0.7280,  1.3000,  2.5500 }, { 1.3000,  0.7280,  2.5500 },
        { 0.7280,  1.3000,  2.4000 }, { 1.3000,  0.7280,  2.4000 }, { 0.0000,  0.0000,  0.0000 },
        { 1.5000,  0.0000,  0.1500 }, { 1.5000,  0.8400,  0.1500 }, { 0.8400,  1.5000,  0.1500 },
        { 0.0000,  1.5000,  0.1500 }, { 1.5000,  0.0000,  0.0750 }, { 1.5000,  0.8400,  0.0750 },
        { 0.8400,  1.5000,  0.0750 }, { 0.0000,  1.5000,  0.0750 }, { 1.4250,  0.0000,  0.0000 },
        { 1.4250,  0.7980,  0.0000 }, { 0.7980,  1.4250,  0.0000 }, { 0.0000,  1.4250,  0.0000 },
        {-0.8400,  1.5000,  0.1500 }, {-1.5000,  0.8400,  0.1500 }, {-1.5000,  0.0000,  0.1500 },
        {-0.8400,  1.5000,  0.0750 }, {-1.5000,  0.8400,  0.0750 }, {-1.5000,  0.0000,  0.0750 },
        {-0.7980,  1.4250,  0.0000 }, {-1.4250,  0.7980,  0.0000 }, {-1.4250,  0.0000,  0.0000 },
        {-1.5000, -0.8400,  0.1500 }, {-0.8400, -1.5000,  0.1500 }, { 0.0000, -1.5000,  0.1500 },
        {-1.5000, -0.8400,  0.0750 }, {-0.8400, -1.5000,  0.0750 }, { 0.0000, -1.5000,  0.0750 },
        {-1.4250, -0.7980,  0.0000 }, {-0.7980, -1.4250,  0.0000 }, { 0.0000, -1.4250,  0.0000 },
        { 0.8400, -1.5000,  0.1500 }, { 1.5000, -0.8400,  0.1500 }, { 0.8400, -1.5000,  0.0750 },
        { 1.5000, -0.8400,  0.0750 }, { 0.7980, -1.4250,  0.0000 }, { 1.4250, -0.7980,  0.0000 }};

    // get index of first triangle and vertex
    int indexTriangle = a_mesh->m_triangles->getNumElements();
    int indexVertex = a_mesh->m_vertices->getNumElements();

    // compute scale factor by taking intto account original size of object
    double scale = (1.0 / 6.4335) * a_size;

    // create patches
    cVector3d controlPoints[16];
    for (int np = 0; np < kTeapotNumPatches; ++np) 
    {
        // set the control points for the current patch
        for (int i = 0; i < 16; ++i)
        {
            controlPoints[i].set(scale * teapotVertices[teapotPatches[np][i] - 1][0],
                scale * teapotVertices[teapotPatches[np][i] - 1][1],
                scale * teapotVertices[teapotPatches[np][i] - 1][2]);
        }

        // generate patch
        cCreateBezierPatch(a_mesh, controlPoints, a_resolution, a_pos, a_rot, a_color);
    }

    // flip triangles
    int numTriangles = a_mesh->getNumTriangles();
    for (int i = indexTriangle; i < numTriangles; i++)
    {
        a_mesh->m_triangles->flip(i);
    }

    // offset and translate model
    int numVertices = a_mesh->getNumVertices();
    for (int i = indexVertex; i < numVertices; i++)
    {
        cVector3d v = cAdd(a_pos, cMul(a_rot, a_mesh->m_vertices->getLocalPos(i)));
        a_mesh->m_vertices->setLocalPos(i, v);
        cVector3d n = cMul(a_rot, a_mesh->m_vertices->getNormal(i));
        a_mesh->m_vertices->setNormal(i, n);
    }
}


//==============================================================================
/*!
    This function creates a linear arrow.

    \param  a_mesh                          Mesh object in which primitive is created.
    \param  a_length                        Length of arrow.
    \param  a_radiusShaft                   Radius of arrow shaft.
    \param  a_lengthTip                     Length or arrow tip.
    \param  a_radiusTip                     Radius of arrow tip.
    \param  a_includeTipsAtBothExtremities  Include tip at both extremities of arrow.
    \param  a_numSides                      Number of sides for each radial section.
    \param  a_direction                     Direction of arrow.
    \param  a_pos                           Position where to build the new primitive.
    \param  a_color                         Color of vertices.
*/
//==============================================================================
void cCreateArrow(cMesh* a_mesh, 
    const double& a_length,
    const double& a_radiusShaft,
    const double& a_lengthTip,
    const double& a_radiusTip,
    const bool a_includeTipsAtBothExtremities,
    const unsigned int a_numSides,
    const cVector3d& a_direction,
    const cVector3d& a_pos,
    const cColorf& a_color)
{
    // sanity check
    if ((a_direction.length() == 0.0)   ||
        (a_length < 0)                  ||
        (a_radiusShaft < 0)             ||
        (a_lengthTip < 0)               ||
        (a_radiusTip > 360)) 
    { return; }
    
    // create rotation frame from direction vector
    cVector3d vx,vy,vz,t0,t1, t;;
    t0.set(1,0,0);
    t1.set(0,1,0);
    vz = cNormalize(a_direction);
    double ang0 = cAngle(vz, t0);
    double ang1 = cAngle(vz, t1);
    if (ang0>ang1)
    {
        t = cCross(vz, t0);  
    }
    else
    {
        t = cCross(vz, t1);
    }
    vy = cNormalize(t);
    vx = cNormalize(cCross(vy, vz));

    cMatrix3d rot;
    rot.setCol(vx, vy, vz);

    cVector3d offset0, offset1, offset2;
    double length;
    if (a_includeTipsAtBothExtremities)
    {
        length = a_length - 2.0 * a_lengthTip;
        offset0.set(0, 0, length + a_lengthTip);
        offset1.set(0, 0, a_lengthTip);
    }
    else
    {
        length = a_length - 1.0 * a_lengthTip;
        offset0.set(0, 0, length);
        offset1.set(0, 0, 0);
    }
    
    // create first tip
    cVector3d pos0 = cAdd(a_pos, cMul(rot, offset0));
    cMatrix3d rot0 = rot;

    cCreateCone(a_mesh, 
                a_lengthTip,  
                a_radiusTip,
                0.0,
                a_numSides,
                1,
                1,
                true,
                false,
                pos0,
                rot0,
                a_color);

    // create arrow shaft
    cVector3d pos1 = cAdd(a_pos, cMul(rot, offset1));
    cMatrix3d rot1 = rot;

    if (length > 0)
    {
        cCreateCylinder(a_mesh, 
                        length,  
                        a_radiusShaft,
                        a_numSides,
                        1,
                        1,
                        true,
                        true,
                        pos1,
                        rot1,
                        a_color);
    }

    // create possibly second tip
    if (a_includeTipsAtBothExtremities)
    {
        offset2.set(0, 0, a_lengthTip);
        cVector3d pos2 = cAdd(a_pos, cMul(rot, offset2));  
        cMatrix3d rot2p;
        rot2p.identity();
        rot2p.rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 180);
        cMatrix3d rot2 = cMul(rot,rot2p);

        cCreateCone(a_mesh, 
                    a_lengthTip,  
                    a_radiusTip,
                    0.0,
                    a_numSides,
                    1,
                    1,
                    true,
                    false,
                    pos2,
                    rot2,
                    a_color);
    }
}


//==============================================================================
/*!
    This function creates a circular arrow.

    \param  a_mesh                          Mesh object in which primitive is created.
    \param  a_innerRadius0                  Length of arrow.
    \param  a_innerRadius1                  Radius of arrow shaft.
    \param  a_outerRadius                   Radius of arrow shaft.
    \param  a_lengthTip                     Length or arrow tip.
    \param  a_radiusTip                     Radius of arrow tip.
    \param  a_coverageAngleDeg              Coverage angle of the arrow (0-360 degrees)
    \param  a_includeTipsAtBothExtremities  Include tip at both extremities of arrow.
    \param  a_numSides                      Number of sides for each radial section.
    \param  a_numRings                      Number of radial divisions for the circular shaft.
    \param  a_direction                     Direction of circular arrow plane.
    \param  a_pos                           Position where to build the new primitive.
    \param  a_color                         Color of vertices.
*/
//==============================================================================
void cCreateCircularArrow(cMesh* a_mesh,
    const double& a_innerRadius0,
    const double& a_innerRadius1,
    const double& a_outerRadius,
    const double& a_lengthTip,
    const double& a_radiusTip,
    const double& a_coverageAngleDeg,
    const bool a_includeTipsAtBothExtremities,
    const unsigned int a_numSides,
    const unsigned int a_numRings, 
    const cVector3d& a_direction,
    const cVector3d& a_pos,
    const cColorf& a_color)
{
    // sanity check
    if ((a_direction.length() == 0.0)   ||
        (a_innerRadius0 < 0)            ||
        (a_innerRadius1 < 0)            ||
        (a_coverageAngleDeg < 0)        ||
        (a_coverageAngleDeg > 360)      ||
        (a_outerRadius < 0)             ||
        (a_outerRadius < 0))  
    { return; }
    
    // create rotation frame from direction vector
    cVector3d vx,vy,vz,t0,t1, t;;
    t0.set(1,0,0);
    t1.set(0,1,0);
    vz = cNormalize(a_direction);
    double ang0 = cAngle(vz, t0);
    double ang1 = cAngle(vz, t1);
    if (ang0>ang1)
    {
        t = cCross(vz, t0);
    }
    else
    {
        t = cCross(vz, t1);
    }
    vy = cNormalize(t);
    vx = cNormalize(cCross(vy, vz));

    cMatrix3d rot;
    rot.setCol(vx, vy, vz);

    // create ring
    cCreateRingSection(a_mesh,
                       a_innerRadius0,
                       a_innerRadius1,
                       a_outerRadius,
                       a_coverageAngleDeg, 
                       true,
                       a_numSides,
                       a_numRings,
                       a_pos,
                       rot,
                       a_color);

    cVector3d offset0, offset1, pos0, pos1;
    cMatrix3d rot0, rot0p, rot1, rot1p;

    offset0.set(a_outerRadius * cCosDeg(a_coverageAngleDeg + 90),
                a_outerRadius * cSinDeg(a_coverageAngleDeg + 90),
                0.0);
    pos0 = cAdd(a_pos, cMul(rot, offset0));
    rot0p.identity();
    rot0p.rotateAboutGlobalAxisDeg(cVector3d(0,1,0), -90);
    rot0p.rotateAboutGlobalAxisDeg(cVector3d(0,0,1), a_coverageAngleDeg);
    rot0 = cMul(rot,rot0p);

    // create first arrow tip
    cCreateCone(a_mesh, 
                a_lengthTip,  
                a_radiusTip,
                0.0,
                a_numSides,
                1,
                1,
                true,
                false,
                pos0,
                rot0,
                a_color);

    // create second arrow tip
    if (a_includeTipsAtBothExtremities)
    {
        offset1.set(0, a_outerRadius, 0);
        pos1 = cAdd(a_pos, cMul(rot, offset1));
        rot1p.identity();
        rot1p.rotateAboutGlobalAxisDeg(cVector3d(0,1,0), 90);
        rot1 = cMul(rot, rot1p);

        // create first arrow tip
        cCreateCone(a_mesh,
                    a_lengthTip,
                    a_radiusTip,
                    0.0,
                    a_numSides,
                    1,
                    1,
                    true,
                    false,
                    pos1,
                    rot1,
                    a_color);
    }
}


//==============================================================================
/*!
    This function creates a Bezier patch from a set of control points.

    \param  a_mesh            Mesh object in which primitive is created.
    \param  a_controlPoints   Bezier control points (16).
    \param  a_numDivisions    Number of division along each side of the patch.
    \param  a_pos             Position where to build the new primitive.
    \param  a_rot             Orientation of the new primitive.
    \param  a_color           Color of vertices.
*/
//==============================================================================
void cCreateBezierPatch(cMesh* a_mesh,
    const cVector3d *a_controlPoints,
    const int a_numDivisions,
    const cVector3d& a_pos,
    const cMatrix3d& a_rot,
    const cColorf& a_color)
{
    // get index of first vertex
    int index = a_mesh->getNumVertices();

    // create patch
    cCreateMap(a_mesh, 1.0, 1.0, a_numDivisions, a_numDivisions, a_pos, a_rot, a_color);

    // compute position of all points
    for (int j = 0, k = index; j <= a_numDivisions; ++j)
    {
        double v = j / (double)a_numDivisions;
        for (int i = 0; i <= a_numDivisions; ++i, ++k)
        {
            double u = i / (double)a_numDivisions;
            cVector3d p = cEvalBezierPatch(a_controlPoints, u, v);
            cVector3d dU = cDerivUBezier(a_controlPoints, u, v);
            cVector3d dV = cDerivVBezier(a_controlPoints, u, v);
            cVector3d n = cNormalize(cCross(dU, dV));

            a_mesh->m_vertices->setLocalPos(k, p);
            a_mesh->m_vertices->setNormal(k, n);
            a_mesh->m_vertices->setTangent(k, dU);
            a_mesh->m_vertices->setTangent(k, dV);
        }
    }
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
