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
    \version   3.2.0 $Rev: 2097 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileModelSTL.h"
//------------------------------------------------------------------------------
#include "stdint.h"
#include <stdio.h>
#include <math.h>
#include <fstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

struct cHeaderSTL
{
    char m_header[80];
    unsigned int m_numTriangles;
};

struct cTriangleSTL
{
    float m_normal[3];
    float m_vertex0[3];
    float m_vertex1[3];
    float m_vertex2[3];
    int m_attribute;
};

//------------------------------------------------------------------------------
#endif // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    This function loads an STL (binary format) 3D model from a file into a cMultiMesh structure.
    If the operation succeeds, then the functions returns __true__ and the
    3D model is loaded into cMultiMesh as a single mesh.
    If the operation fails, then the function returns __false__.

    \param  a_object    Multimesh object.
    \param  a_filename  Filename.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileSTL(cMultiMesh* a_object, const std::string& a_filename)
{
    // sanity check
    if (a_object == NULL)
        return (C_ERROR);

    // open file
    ifstream file(a_filename.c_str(), ios::binary);
    if (!file)
        return (C_ERROR);

    // get length of file
    file.seekg (0, file.end);
    int length = (int)(file.tellg());
    file.seekg (0, file.beg);
    if (length < 84)
    {
        file.close();
        return (C_ERROR);
    }

    // read header
    cHeaderSTL header;
    file.read((char*)(&header), 84);

    // read number of triangles
    unsigned int numTriangles = header.m_numTriangles;
    if (numTriangles == 0)
    {
        file.close();
        return (C_ERROR);
    }

    // check if the length of the file correspond to the number of triangles found
    double num = (double)(length - sizeof(cHeaderSTL)) / 50.0;
    if ((num != floor(num)) && ((int)num != numTriangles))
    {
        file.close();
        return (C_ERROR);
    }

    // create mesh
    cMesh* mesh = a_object->newMesh();

    // load triangles
    for (int i=0; i<(int)numTriangles; i++)
    {
        cTriangleSTL triangle;

        // read triangle data from file
        file.read((char*)(&triangle), 50);

        // create triangle entity
        mesh->newTriangle(cVector3d(triangle.m_vertex0[0], triangle.m_vertex0[1], triangle.m_vertex0[2]),
                          cVector3d(triangle.m_vertex1[0], triangle.m_vertex1[1], triangle.m_vertex1[2]),
                          cVector3d(triangle.m_vertex2[0], triangle.m_vertex2[1], triangle.m_vertex2[2]));
    }

    // compute normals
    mesh->computeAllNormals();

    // close file
    file.close();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This function saves an STL (binary format) 3D model from a cMultiMesh object to a file.
    If the operation succeeds, then the functions returns __true__ and the 
    model data is saved to a file.
    If the operation fails, then the function returns __false__.

    \param  a_object    Multimesh object.
    \param  a_filename  Filename.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileSTL(cMultiMesh* a_object, const std::string& a_filename)
{
    // sanity check
    if (a_object == NULL) 
        return (C_ERROR);

    // write file
    ofstream file(a_filename.c_str(), ios::binary);
    if (!file)
        return (C_ERROR);

    // get number of triangles
    unsigned int numTriangles = a_object->getNumTriangles();

    // write header
    cHeaderSTL header;
    string message = "CHAI3D STL Converter";
    strcpy(header.m_header, message.c_str());
    header.m_numTriangles = numTriangles;
    file.write((char*)(&header), 84);

    // write all triangles
    int numMeshes = a_object->getNumMeshes();
    for (int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = a_object->getMesh(i);
        int numTri = mesh->getNumTriangles();

        // compute global position of vertices
        mesh->computeGlobalPositions(false);

        for (int j=0; j<numTri; j++)
        {
            // retrieve vertex positions
            cVector3d vertex0 = mesh->m_vertices->getGlobalPos(mesh->m_triangles->getVertexIndex0(j));
            cVector3d vertex1 = mesh->m_vertices->getGlobalPos(mesh->m_triangles->getVertexIndex1(j));
            cVector3d vertex2 = mesh->m_vertices->getGlobalPos(mesh->m_triangles->getVertexIndex2(j));

            // compute surface normal
            cVector3d normal = cComputeSurfaceNormal(vertex0, vertex1, vertex2);

            // construct STL data structure
            cTriangleSTL triangle;
            triangle.m_vertex0[0] = (float)vertex0(0);
            triangle.m_vertex0[1] = (float)vertex0(1);
            triangle.m_vertex0[2] = (float)vertex0(2);
            
            triangle.m_vertex1[0] = (float)vertex1(0);
            triangle.m_vertex1[1] = (float)vertex1(1);
            triangle.m_vertex1[2] = (float)vertex1(2);
            
            triangle.m_vertex2[0] = (float)vertex2(0);
            triangle.m_vertex2[1] = (float)vertex2(1);
            triangle.m_vertex2[2] = (float)vertex2(2);

            triangle.m_normal[0] = (float)normal(0);
            triangle.m_normal[1] = (float)normal(1);
            triangle.m_normal[2] = (float)normal(2);

            triangle.m_attribute = 0;

            // write data
            file.write((char*)(&triangle), 50);
        }
    }

    // close file
    file.close();

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
