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
    \author    Tim Schroeder
    \author    Francois Conti
    \version   3.2.0 $Rev: 2187 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileModelOBJ.h"
//------------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
#include <ostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <cstring>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
bool g_objLoaderShouldGenerateExtraVertices = false;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads an OBJ 3D model from a file into a cMultiMesh structure.
    If the operation succeeds, then the functions returns __true__ and the
    3D model is loaded into cMultiMesh.
    If the operation fails, then the function returns __false__.

    \param  a_object    Multimesh object.
    \param  a_filename  Filename.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileOBJ(cMultiMesh* a_object, const std::string& a_filename)
{
    try
    {
        cOBJModel fileObj;

        // load file into memory. If an error occurs, exit.
        if (!fileObj.LoadModel(a_filename.c_str())) { return (false); }

        // clear all vertices and triangle of current mesh
        a_object->deleteAllMeshes();

        // get information about file
        int numMaterials = fileObj.m_OBJInfo.m_materialCount;

        // extract materials
        vector<cMaterial> materials;

        // object has no material properties
        if (numMaterials == 0)
        {
            // create a new child
            cMesh *newMesh = a_object->newMesh();
            newMesh->setUseMaterial(true);
            newMesh->setUseTransparency(false);
        }

        // object has material properties. Create a child for each material
        // property.
        else
        {
            int i = 0;
            bool found_transparent_material = false;

            while (i < numMaterials)
            {
                // create a new child
                cMesh *newMesh = a_object->newMesh();

                // use materials
                newMesh->setUseMaterial(true);

                // get next material
                cMaterialInfo material = fileObj.m_pMaterials[i];

                int textureId = material.m_textureID;
                if (textureId >= 1)
                {
                    cTexture2dPtr newTexture = cTexture2d::create();
                    bool result = newTexture->loadFromFile(material.m_texture);

                    // If this didn't work out, try again in the obj file's path
                    if (result == false) 
                    {
                        string model_dir = cGetDirectory(a_filename);

                        char new_texture_path[1024];
                        sprintf(new_texture_path,"%s/%s",model_dir.c_str(),material.m_texture);

                        result = newTexture->loadFromFile(new_texture_path);
                    }

                    if (result)
                    {
                        newMesh->setTexture(newTexture);
                        newMesh->setUseTexture(true);
                    }
                }

                float alpha = material.m_alpha;
                if (alpha < 1.0) 
                {
                    newMesh->setUseTransparency(true, false);
                    found_transparent_material = true;
                }

                // get ambient component:
                newMesh->m_material->m_ambient.setR(material.m_ambient[0]);
                newMesh->m_material->m_ambient.setG(material.m_ambient[1]);
                newMesh->m_material->m_ambient.setB(material.m_ambient[2]);
                newMesh->m_material->m_ambient.setA(alpha);

                // get diffuse component:
                newMesh->m_material->m_diffuse.setR(material.m_diffuse[0]);
                newMesh->m_material->m_diffuse.setG(material.m_diffuse[1]);
                newMesh->m_material->m_diffuse.setB(material.m_diffuse[2]);
                newMesh->m_material->m_diffuse.setA(alpha);

                // get specular component:
                newMesh->m_material->m_specular.setR(material.m_specular[0]);
                newMesh->m_material->m_specular.setG(material.m_specular[1]);
                newMesh->m_material->m_specular.setB(material.m_specular[2]);
                newMesh->m_material->m_specular.setA(alpha);

                // get emissive component:
                newMesh->m_material->m_emission.setR(material.m_emmissive[0]);
                newMesh->m_material->m_emission.setG(material.m_emmissive[1]);
                newMesh->m_material->m_emission.setB(material.m_emmissive[2]);
                newMesh->m_material->m_emission.setA(alpha);

                // get shininess
                newMesh->m_material->setShininess((GLuint)(1.28 * material.m_shininess));

                i++;
            }

            // Enable material property rendering
            a_object->setUseVertexColors(false, true);
            a_object->setUseMaterial(true, true);

            // Mark the presence of transparency in the root mesh; don't
            // modify the value stored in children...
            a_object->setUseTransparency(found_transparent_material, false);
        }

        // Keep track of vertex mapping in each mesh; maps "old" vertices
        // to new vertices
        int nMeshes = a_object->getNumMeshes();
        vertexIndexSet_uint_map* vertexMaps = new vertexIndexSet_uint_map[nMeshes];
        vertexIndexSet_uint_map::iterator vertexMapIter;

        // build object
        {
            int i = 0;

            // get triangles
            int numTriangles = fileObj.m_OBJInfo.m_faceCount;
            int j = 0;
            if (numTriangles > 0)
            {
                while (j < numTriangles)
                {
                    // get next face
                    cFace face = fileObj.m_pFaces[j];

                    // get material index attributed to the face
                    int objIndex = face.m_materialIndex;

                    // the mesh that we're reading this triangle into
                    cMesh* curMesh = a_object->getMesh(objIndex);

                    // create a name for this mesh if necessary (over-writing a previous
                    // name if one has been written)
                    if ( (face.m_groupIndex >= 0) && (fileObj.m_groupNames.size() > 0) )
                    {
                        curMesh->m_name = fileObj.m_groupNames[face.m_groupIndex];
                    }

                    // get the vertex map for this mesh
                    vertexIndexSet_uint_map* curVertexMap = &(vertexMaps[objIndex]);

                    // number of vertices on face
                    int vertCount = face.m_numVertices;

                    if (vertCount >= 3) 
                    {
                        int indexV1 = face.m_pVertexIndices[0];

                        if (g_objLoaderShouldGenerateExtraVertices==false) 
                        {
                            vertexIndexSet vis(indexV1);
                            if (face.m_pNormals != NULL) vis.nIndex = face.m_pNormalIndices[0];
                            if (face.m_pTexCoords != NULL) vis.tIndex = face.m_pTextureIndices[0];
                            indexV1 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                        }                

                        for (int triangleVert = 2; triangleVert < vertCount; triangleVert++)
                        {
                            int indexV2 = face.m_pVertexIndices[triangleVert-1];
                            int indexV3 = face.m_pVertexIndices[triangleVert];
                            if (g_objLoaderShouldGenerateExtraVertices==false) 
                            {
                                vertexIndexSet vis(indexV2);
                                if (face.m_pNormals != NULL) vis.nIndex = face.m_pNormalIndices[triangleVert-1];
                                if (face.m_pTexCoords) vis.tIndex = face.m_pTextureIndices[triangleVert-1];
                                indexV2 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                                vis.vIndex = indexV3;
                                if (face.m_pNormals != NULL) vis.nIndex = face.m_pNormalIndices[triangleVert];
                                if (face.m_pTexCoords) vis.tIndex = face.m_pTextureIndices[triangleVert];
                                indexV3 = getVertexIndex(curMesh, &fileObj, curVertexMap, vis);
                            }

                            // for debugging, I want to look for degenerate triangles, but
                            // I don't want to assert here.
                            if (indexV1 == indexV2 || indexV2 == indexV3 || indexV1 == indexV3) 
                            {
                            }

                            unsigned int indexTriangle;

                            // create triangle:
                            if (g_objLoaderShouldGenerateExtraVertices==false) 
                            {
                                indexTriangle = curMesh->newTriangle(indexV1,indexV2,indexV3);
                                curMesh->m_triangles->computeNormal(indexTriangle, true);
                            }
                            else 
                            {
                                indexTriangle = curMesh->newTriangle(
                                    fileObj.m_pVertices[indexV1],
                                    fileObj.m_pVertices[indexV2],
                                    fileObj.m_pVertices[indexV3]
                                );
                                curMesh->m_triangles->computeNormal(indexTriangle, true);
                            }

                            // assign normals:
                            if (face.m_pNormals != NULL)
                            {
                                // set normals
                                curMesh->m_vertices->setNormal(curMesh->m_triangles->getVertexIndex0(indexTriangle), face.m_pNormals[0]);
                                curMesh->m_vertices->setNormal(curMesh->m_triangles->getVertexIndex1(indexTriangle), face.m_pNormals[triangleVert-1]);
                                curMesh->m_vertices->setNormal(curMesh->m_triangles->getVertexIndex2(indexTriangle), face.m_pNormals[triangleVert]);
                            }

                            // assign texture coordinates
                            if (face.m_pTexCoords != NULL)
                            {
                                // set texture coordinates
                                curMesh->m_vertices->setTexCoord(curMesh->m_triangles->getVertexIndex0(indexTriangle), face.m_pTexCoords[0]);
                                curMesh->m_vertices->setTexCoord(curMesh->m_triangles->getVertexIndex1(indexTriangle), face.m_pTexCoords[triangleVert-1]);
                                curMesh->m_vertices->setTexCoord(curMesh->m_triangles->getVertexIndex2(indexTriangle), face.m_pTexCoords[triangleVert]);
                            }
                        }
                    }
                    else 
                    {
                        // This faces doesn't have 3 vertices... this line is just
                        // here for debugging, since this should never happen, but
                        // I don't want to assert here.
                    }

                    j++;
                }
                i++;
            }
            else
            {
                // get number of vertices
                int numVertices = fileObj.m_OBJInfo.m_vertexCount;
                int j = 0;

                // get main mesh
                cMesh* mesh = a_object->getMesh(0);

                // copy vertex and color data
                while (j < numVertices)
                {
                    cColorf color = fileObj.m_pColors[j];
                    cVector3d vertex = fileObj.m_pVertices[j];
                    mesh->newVertex(vertex, cVector3d(1,0,0), cVector3d(0,0,0), color);
                    j++;
                }

                if (numVertices > 0)
                {
                    cColorf color = fileObj.m_pColors[j];
                    if (color.m_flag_color)
                    {
                        mesh->setUseVertexColors(true);
                    }
                    else
                    {
                        mesh->setUseVertexColors(false);
                    }
                }
            }
        }
        delete [] vertexMaps;

        // compute boundary boxes
        a_object->computeBoundaryBox(true);

        // update global position in world
        a_object->computeGlobalPositionsFromRoot(true);

        // return success
        return (C_SUCCESS);
    }

    catch (...)
    {
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This function saves an OBJ 3D model from a cMultiMesh object to a file.
    If the operation succeeds, then the functions returns __true__ and the 
    model data is saved to a file.
    If the operation fails, then the function returns __false__.

    \param  a_object    Multimesh object.
    \param  a_filename  Filename.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileOBJ(cMultiMesh* a_object, const std::string& a_filename)
{
    /////////////////////////////////////////////////////////////////////////
    // INITIALIZATION
    /////////////////////////////////////////////////////////////////////////

    // get number of mesh objects composing multimesh
    unsigned int numMeshes = a_object->getNumMeshes();

    // get name of file. remove path and file extension
    string fileName = cGetFilename(a_filename, false);
    if (fileName == "") { return (C_ERROR); }

    // get path
    string filePath = cGetDirectory(a_filename);


    /////////////////////////////////////////////////////////////////////////
    // MATERIAL FILE
    /////////////////////////////////////////////////////////////////////////

    // create material file
    string str = filePath + fileName + ".mat";

    // create file
    ofstream fileMat(str.c_str());

    fileMat << "#" << endl;
    fileMat << "# Wavefront material file" << endl;
    fileMat << "# CHAI3D" << endl;
    fileMat << "# http://www.chai3d.org" << endl;
    fileMat << "#" << endl;
    fileMat << endl << endl;

    // copy data from all material files
    for (unsigned int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = a_object->getMesh(i);
        cMaterialPtr mat = mesh->m_material;

        string textureName = "";
        cTexture1dPtr texture = mesh->m_texture;
        if (texture != nullptr)
        {
            cImagePtr image = texture->m_image;
            if (image != nullptr)
            {
                textureName = fileName  + "-" + cStr(i) + ".png";
            }
        }

        if (mat != nullptr)
        {
            fileMat << "newmtl MATERIAL"+cStr(i) << endl;
            fileMat << "Ka " << cStr(mat->m_ambient.getR(), 3) << " " << cStr(mat->m_ambient.getG(), 3) << " " << cStr(mat->m_ambient.getB(), 3) << endl;
            fileMat << "Kd " << cStr(mat->m_diffuse.getR(), 3) << " " << cStr(mat->m_diffuse.getG(), 3) << " " << cStr(mat->m_diffuse.getB(), 3) << endl;
            fileMat << "Ks " << cStr(mat->m_specular.getR(), 3) << " " << cStr(mat->m_specular.getG(), 3) << " " << cStr(mat->m_specular.getB(), 3) << endl;
            fileMat << "Ns " << cStr((1.0/1.28)*mat->getShininess()) << endl;

            float transparency = mat->m_ambient.getA();
            if (transparency < 1.0)
            {
                fileMat << "d" << cStr(transparency, 3);
            }
            
            fileMat << "illum 2" << endl;
            fileMat << "map_Kd " << textureName << endl;
            fileMat << "map_bump" << endl;
            fileMat << "bump" << endl;
            fileMat << "map_opacity" << endl;
            fileMat << "map_d" << endl;
            fileMat << "refl" << endl;
            fileMat << "map_kS" << endl;
            fileMat << "map_kA" << endl;
            fileMat << "map_Ns" << endl;
            fileMat << endl;
        }
    }

    // close file
    fileMat.close();

    /////////////////////////////////////////////////////////////////////////
    // TEXTURE FILES
    /////////////////////////////////////////////////////////////////////////

    // save image files
    for (unsigned int i=0; i<numMeshes; i++)
    {
        cMesh* mesh = a_object->getMesh(i);
        cTexture1dPtr texture = mesh->m_texture;
        if (texture != nullptr)
        {
            cImagePtr image = texture->m_image;
            if (image != nullptr)
            {
                string textureFileName = filePath + fileName + "-" + cStr(i) + ".png";
                image->saveToFile(textureFileName);
            }
        }
    }


    /////////////////////////////////////////////////////////////////////////
    // MESH DATA
    /////////////////////////////////////////////////////////////////////////

    // vertex counter
    int vertexCounter = 0;

    // compute global positions for all vertices.
    a_object->computeGlobalPositions(false);

    // create OBJ file
    str = filePath + fileName + ".obj";
    ofstream fileObj(str.c_str());

    fileObj << "#" << endl;
    fileObj << "# Wavefront object file" << endl;
    fileObj << "# CHAI3D" << endl;
    fileObj << "# http://www.chai3d.org" << endl;
    fileObj << "#" << endl;
    fileObj << endl << endl;

    // basic information
    fileObj << "mtllib " << fileName << ".mat" << endl;
    fileObj << "# object " << fileName << endl;
    fileObj << "g " << fileName << endl << endl;

    // copy mesh data
    for (unsigned int i=0; i<numMeshes; i++)
    {
        // get next mesh object
        cMesh* mesh = a_object->getMesh(i);
        unsigned int numVertices = mesh->getNumVertices();

        // store vertex positions
        fileObj << "# mesh-" << cStr(i) << ": vertices" << endl;
        for (unsigned int j=0; j<numVertices; j++)
        {
            cVector3d pos = mesh->m_vertices->getGlobalPos(j);
            cColorf color = mesh->m_vertices->getColor(j);
            if (mesh->getUseVertexColors())
            {
                fileObj << "v " << cStr(pos(0), 5) << " " << cStr(pos(1), 5) << " " << cStr(pos(2), 5) << " " << cStr(color.getR(), 3) << " " << cStr(color.getG(), 3) << " " << cStr(color.getB(), 3) <<endl;
            }
            else
            {
                fileObj << "v " << cStr(pos(0), 5) << " " << cStr(pos(1), 5) << " " << cStr(pos(2), 5) << endl;
            }
        }
        fileObj << endl;

        // store texture coordinates
        if (mesh->getUseTexture())
        {
            fileObj << "# mesh-" << cStr(i) << ": texture coordinates" << endl;
            for (unsigned int j=0; j<numVertices; j++)
            {
                cVector3d pos = mesh->m_vertices->getTexCoord(j);
                fileObj << "vt " << cStr(pos(0), 5) << " " << cStr(pos(1), 5) << endl;//" " << cStr(pos[2], 6) << endl;
            }
            fileObj << endl;
        }

        // store normals
        fileObj << "# mesh-" << cStr(i) << ": normals" << endl;
        for (unsigned int j=0; j<numVertices; j++)
        {
            cVector3d normal = cMul(mesh->getLocalRot(), mesh->m_vertices->getNormal(j));
            fileObj << "vn " << cStr(normal(0), 5) << " " << cStr(normal(1), 5) << " " << cStr(normal(2), 5) << endl;
        }
        fileObj << endl;

        // faces
        fileObj << "# mesh-" << cStr(i) << ": faces" << endl;
        fileObj << "usemtl MATERIAL" << cStr(i) << endl;
        unsigned numTriangles = mesh->getNumTriangles();
        for (unsigned int j=0; j<numTriangles; j++)
        {  
            int indexV0 = mesh->m_triangles->getVertexIndex0(j) + vertexCounter + 1;
            int indexV1 = mesh->m_triangles->getVertexIndex1(j) + vertexCounter + 1;
            int indexV2 = mesh->m_triangles->getVertexIndex2(j) + vertexCounter + 1;
            int indexT0 = indexV0;
            int indexT1 = indexV1;
            int indexT2 = indexV2;
            int indexN0 = indexV0;
            int indexN1 = indexV1;
            int indexN2 = indexV2;

            fileObj << "f " << cStr(indexV0) << "/" << cStr(indexT0) << "/" << cStr(indexN0) << " " << cStr(indexV1 ) << "/" << cStr(indexT1) << "/" << cStr(indexN1) << " " << cStr(indexV2) << "/" << cStr(indexT2) << "/" << cStr(indexN2) << endl;
        }

        fileObj << endl;

        // update vertex counter
        vertexCounter = vertexCounter + mesh->getNumVertices();
    }


    // close file
    fileObj.close();

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------
//==============================================================================
// OBJ PARSER IMPLEMENTATION:
//==============================================================================

cOBJModel::cOBJModel()
{
    m_pVertices = NULL;
    m_pColors = NULL;
    m_pFaces = NULL;
    m_pNormals = NULL;
    m_pTexCoords = NULL;
    m_pMaterials = NULL;
}

//------------------------------------------------------------------------------

cOBJModel::~cOBJModel()
{
    if (m_pVertices) delete [] m_pVertices;
    if (m_pColors) delete [] m_pColors;
    if (m_pNormals) delete [] m_pNormals;
    if (m_pTexCoords) delete [] m_pTexCoords;
    if (m_pMaterials) delete [] m_pMaterials;
    if (m_pFaces)
    {
        for (unsigned int i=0; i<m_OBJInfo.m_faceCount; i++)
        {
            // delete every pointer in the face structure
            if (m_pFaces[i].m_pNormals) delete [] m_pFaces[i].m_pNormals;
            if (m_pFaces[i].m_pNormalIndices) delete [] m_pFaces[i].m_pNormalIndices;
            if (m_pFaces[i].m_pTexCoords)  delete [] m_pFaces[i].m_pTexCoords;
            if (m_pFaces[i].m_pTextureIndices) delete [] m_pFaces[i].m_pTextureIndices;
            if (m_pFaces[i].m_pVertices)  delete [] m_pFaces[i].m_pVertices;
            if (m_pFaces[i].m_pColors)  delete [] m_pFaces[i].m_pColors;
            if (m_pFaces[i].m_pVertexIndices)  delete [] m_pFaces[i].m_pVertexIndices;
        }
        delete [] m_pFaces;
    }
    for(unsigned int i=0; i<m_groupNames.size(); i++) 
    {
        delete [] m_groupNames[i];
    }
}

//------------------------------------------------------------------------------

bool cOBJModel::LoadModel(const char a_fileName[])
{
    /////////////////////////////////////////////////////////////////////////
    // LOAD A OBJ FILE AND RENDER ITS DATA INTO A DISPLAY LIST
    /////////////////////////////////////////////////////////////////////////

    cOBJFileInfo currentIndex;          // current array index
    char str[C_OBJ_MAX_STR_SIZE] = "";  // buffer string for reading the file
    char basePath[C_OBJ_SIZE_PATH];     // path were all paths in the OBJ start
    unsigned int curMaterial = 0;       // current material

    // get base path
    strcpy(basePath, a_fileName);
    makePath(basePath);

    /////////////////////////////////////////////////////////////////////////
    // OPEN THE OBJ FILE
    /////////////////////////////////////////////////////////////////////////
    FILE *hFile = fopen(a_fileName, "r");

    // success opening file?
    if (!hFile)
    {
        return (false);
    }

    /////////////////////////////////////////////////////////////////////////
    // ALLOCATE SPACE FOR STRUCTURES THAT HOLD THE MODEL DATA
    /////////////////////////////////////////////////////////////////////////

    // which data types are stored in the file? how many of each type?
    getFileInfo(hFile, &m_OBJInfo, basePath);

    // vertices and faces
    if (m_pVertices) delete [] m_pVertices;
    if (m_pColors) delete [] m_pColors;
    if (m_pFaces) delete [] m_pFaces;
    m_pVertices = new cVector3d[m_OBJInfo.m_vertexCount];
    m_pColors = new cColorf[m_OBJInfo.m_vertexCount];
    m_pFaces = new cFace[m_OBJInfo.m_faceCount];

    // allocate space for optional model data only if present.
    if (m_pNormals) { delete [] m_pNormals; m_pNormals = NULL; }
    if (m_pTexCoords) { delete [] m_pTexCoords; m_pTexCoords = NULL; }
    if (m_pMaterials) { delete [] m_pMaterials; m_pMaterials = NULL; }
    if (m_OBJInfo.m_normalCount)
        m_pNormals = new cVector3d[m_OBJInfo.m_normalCount];
    if (m_OBJInfo.m_texCoordCount)
        m_pTexCoords = new cVector3d[m_OBJInfo.m_texCoordCount];
    if (m_OBJInfo.m_materialCount)
        m_pMaterials = new cMaterialInfo[m_OBJInfo.m_materialCount];

    // init structure that holds the current array index
    currentIndex.init();

    /////////////////////////////////////////////////////////////////////////
    // READ THE FILE CONTENTS
    /////////////////////////////////////////////////////////////////////////

    // start reading the file from the start
    rewind(hFile);

    // quit reading when end of file has been reached
    while (!feof(hFile))
    {
        // get next string
        readNextString(str, sizeof(str), hFile);

        // next three elements are floats of a vertex
        if (!strncmp(str, C_OBJ_VERTEX_ID, sizeof(C_OBJ_VERTEX_ID)))
        {
            // read three floats out of the file
            float fx = 0.0f, fy = 0.0f, fz = 0.0f, cr = 0.0f, cg = 0.0f, cb = 0.0f;
            if (fscanf(hFile, "%f %f %f %f %f %f", &fx, &fy, &fz, &cr, &cg, &cb) > 0)
            {
                m_pVertices[currentIndex.m_vertexCount](0)  = fx;
                m_pVertices[currentIndex.m_vertexCount](1)  = fy;
                m_pVertices[currentIndex.m_vertexCount](2)  = fz;
                m_pColors[currentIndex.m_vertexCount].set(cr, cg, cb);
                m_pColors[currentIndex.m_vertexCount].m_flag_color = true;
            }
            else if (fscanf(hFile, "%f %f %f", &fx, &fy, &fz) > 0)
            {
                m_pVertices[currentIndex.m_vertexCount](0)  = fx;
                m_pVertices[currentIndex.m_vertexCount](1)  = fy;
                m_pVertices[currentIndex.m_vertexCount](2)  = fz;
                m_pColors[currentIndex.m_vertexCount].set(0,0,0);
                m_pColors[currentIndex.m_vertexCount].m_flag_color = false;
            }

            // next vertex
            currentIndex.m_vertexCount++;
        }

        // next two elements are floats of a texture coordinate
        else if (!strncmp(str, C_OBJ_TEXCOORD_ID, sizeof(C_OBJ_TEXCOORD_ID)))
        {
            // read two floats out of the file
            float fx = 0.0f, fy = 0.0f, fz = 0.0f;
            if (fscanf(hFile, "%f %f %f", &fx, &fy, &fz) > 0)
            {
                m_pTexCoords[currentIndex.m_texCoordCount](0)  = fx;
                m_pTexCoords[currentIndex.m_texCoordCount](1)  = fy;
                m_pTexCoords[currentIndex.m_texCoordCount](2)  = fz;
            }

            // next texture coordinate
            currentIndex.m_texCoordCount++;
        }

        // next three elements are floats of a vertex normal
        else if (!strncmp(str, C_OBJ_NORMAL_ID, sizeof(C_OBJ_NORMAL_ID)))
        {
            // read three floats out of the file
            float fx = 0.0f, fy = 0.0f, fz = 0.0f;
            if (fscanf(hFile, "%f %f %f", &fx, &fy, &fz) > 0)
            {
                m_pNormals[currentIndex.m_normalCount](0)  = fx;
                m_pNormals[currentIndex.m_normalCount](1)  = fy;
                m_pNormals[currentIndex.m_normalCount](2)  = fz;
            }

            // next normal
            currentIndex.m_normalCount++;
        }

        // rest of the line contains face information
        else if (!strncmp(str, C_OBJ_FACE_ID, sizeof(C_OBJ_FACE_ID)))
        {
            // read the rest of the line (the complete face)
            getTokenParameter(str, sizeof(str) ,hFile);

            // convert string into a face structure
            parseFaceString(str, &m_pFaces[currentIndex.m_faceCount],
            m_pVertices, m_pNormals, m_pTexCoords, curMaterial);
            
            // next face
            currentIndex.m_faceCount++;
        }

        // rest of the line contains face information
        else if (!strncmp(str, C_OBJ_NAME_ID, sizeof(C_OBJ_NAME_ID)))
        {
            // read the rest of the line (the complete face)
            getTokenParameter(str, sizeof(str) ,hFile);

            char* name = new char[strlen(str)+1];
            strcpy(name,str);
            m_groupNames.push_back(name);
        }

        // process material information only if needed
        if (m_pMaterials)
        {
            // rest of the line contains the name of a material
            if (!strncmp(str, C_OBJ_USE_MTL_ID, sizeof(C_OBJ_USE_MTL_ID)))
            {
                // read the rest of the line (the complete material name)
                getTokenParameter(str, sizeof(str), hFile);

                // are any materials loaded ?
                if (m_pMaterials)
                {
                    // find material array index for the material name
                    for (unsigned i=0; i<m_OBJInfo.m_materialCount; i++)
                    if (!strncmp(m_pMaterials[i].m_name, str, sizeof(str)))
                    {
                        curMaterial = i;
                        break;
                    }
                }
            }

            // rest of the line contains the filename of a material library
            else if (!strncmp(str, C_OBJ_MTL_LIB_ID, sizeof(C_OBJ_MTL_LIB_ID)))
            {
                // read the rest of the line (the complete filename)
                getTokenParameter(str, sizeof(str), hFile);

                // append material library filename to the model's base path
                char libraryFile[C_OBJ_SIZE_PATH];
                strcpy(libraryFile, basePath);
                strcat(libraryFile, str);

                // append .mtl
                // strcat(szLibraryFile, ".mtl");

                // load the material library
                loadMaterialLib(libraryFile, m_pMaterials,
                    &currentIndex.m_materialCount, basePath);
            }
        }
    }

    // close OBJ file
    fclose(hFile);

    /////////////////////////////////////////////////////////////////////////
    // SUCCESS
    /////////////////////////////////////////////////////////////////////////
    return (true);
}

//------------------------------------------------------------------------------

void cOBJModel::parseFaceString(char a_faceString[], cFace *a_faceOut,
                const cVector3d *a_pVertices,
                const cVector3d *a_pNormals,
                const cVector3d *a_pTexCoords,
                const unsigned int a_materialIndex)
{
    /////////////////////////////////////////////////////////////////////////
    // CONVERT FACE STRING FROM THE OBJ FILE INTO A FACE STRUCTURE
    /////////////////////////////////////////////////////////////////////////

    unsigned int i;
    int iVertex = 0;
    int iTextureCoord = 0;
    int iNormal = 0;

    // pointer to the face string. will be incremented later to
    // advance to the next triplet in the string.
    char *pFaceString = a_faceString;

    // save the string positions of all triplets
    int iTripletPos[C_OBJ_MAX_VERTICES];
    int iCurTriplet = 0;

    // init the face structure
    a_faceOut->init();

    // the first vertex always starts at position 0 in the string
    iTripletPos[0] = 0;
    a_faceOut->m_numVertices = 1;
    iCurTriplet++;
      
    if (m_groupNames.size() > 0) a_faceOut->m_groupIndex = (int)(m_groupNames.size() - 1);
    else a_faceOut->m_groupIndex = -1;

    /////////////////////////////////////////////////////////////////////////
    // GET NUMBER OF VERTICES IN THE FACE
    /////////////////////////////////////////////////////////////////////////

    // loop trough the whole string
    bool detectedSpace = false;
    for (i=0; i<strlen(a_faceString); i++)
    {
        if (!detectedSpace)
        {
            // each triplet is separated by spaces
            if (a_faceString[i] == ' ')
            {
                detectedSpace = true;
            }
        }
        else
        {
            if (a_faceString[i] != ' ')
            {
                // one more vertex
                a_faceOut->m_numVertices++;
                
                // save position of triplet
                iTripletPos[iCurTriplet] = i;
                
                // next triplet
                iCurTriplet++;

                // reset for next group of spaces
                detectedSpace = false;
            }
        }
    }


    /////////////////////////////////////////////////////////////////////////
    // CHECK DATA FORMAT OF FACE: (i, i/j, i/j/k, i//k)
    /////////////////////////////////////////////////////////////////////////

    // mode
    int mode = 0;

    // copy a vertex block
    char vertexStr[100];
    strncpy (vertexStr, pFaceString, iTripletPos[1]);

    char* p = strstr(vertexStr, "//");
    if (p != NULL)
    {
        // format: i//k
        mode = 3;
    }
    else
    {
        char* first = strstr(vertexStr, "/");
        if (first == NULL)
        {
            // format i
            mode = 1;
        }
        else
        {
            char* second = strstr(first, "/");
            if (second != NULL)
            {
                // format i/j/k
                mode = 4;
            }
            else
            {
                // format: i/j
                mode = 2;
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // ALLOCATE SPACE FOR STRUCTURES THAT HOLD THE FACE DATA
    /////////////////////////////////////////////////////////////////////////

    // vertices
    a_faceOut->m_pVertices = new cVector3d[a_faceOut->m_numVertices];
    a_faceOut->m_pVertexIndices = new int[a_faceOut->m_numVertices];

    // allocate space for normals and texture coordinates only if present
    if ((mode == 3) || (mode == 4))
    {
        a_faceOut->m_pNormals = new cVector3d[a_faceOut->m_numVertices];
        a_faceOut->m_pNormalIndices = new int[a_faceOut->m_numVertices];
    }
    else
    {
        a_faceOut->m_pNormals = NULL;
        a_faceOut->m_pNormalIndices = NULL;
    }

    // allocate space for texture coordinates
    if ((mode == 2) || (mode == 4)) 
    {  
        a_faceOut->m_pTexCoords = new cVector3d[a_faceOut->m_numVertices];
        a_faceOut->m_pTextureIndices = new int[a_faceOut->m_numVertices];
    }
    else 
    {
        a_faceOut->m_pTexCoords = NULL;
        a_faceOut->m_pTextureIndices = NULL;
    }

    /////////////////////////////////////////////////////////////////////////
    // COPY VERTEX, NORMAL, MATERIAL AND TEXTURE DATA INTO THE STRUCTURE
    /////////////////////////////////////////////////////////////////////////

    // set material
    a_faceOut->m_materialIndex = a_materialIndex;

    // process per-vertex data
    for (i=0; i<(unsigned int) a_faceOut->m_numVertices; i++)
    {
        // read one triplet from the face string

        // are vertices, normals and texture coordinates present ?
        if (mode == 4)
        {
            sscanf(pFaceString, "%i/%i/%i",
            &iVertex, &iTextureCoord, &iNormal);
        }
        else if (mode == 3)
        {
            // vertices and normals but no texture coordinates
            sscanf(pFaceString, "%i//%i", &iVertex, &iNormal);
        }
        else if (mode == 2)
        {
            // vertices and texture coordinates but no normals
            sscanf(pFaceString, "%i/%i", &iVertex, &iTextureCoord);
        }
        else if (mode == 1)
        {
            // only vertices
            sscanf(pFaceString, "%i", &iVertex);
        }

        // copy vertex into the face. Also check for normals and texture
        // coordinates and copy them if present.
        memcpy(&a_faceOut->m_pVertices[i], &m_pVertices[iVertex - 1],
        sizeof(cVector3d));
        a_faceOut->m_pVertexIndices[i] = iVertex-1;

        if ((mode == 2) || (mode == 4)) 
        {    
            memcpy(&a_faceOut->m_pTexCoords[i],
            &m_pTexCoords[iTextureCoord - 1], sizeof(cVector3d));
            a_faceOut->m_pTextureIndices[i] = iTextureCoord-1;
        }
        if ((mode == 3) || (mode == 4)) 
        {
            memcpy(&a_faceOut->m_pNormals[i],
            &m_pNormals[iNormal - 1], sizeof(cVector3d));
            a_faceOut->m_pNormals[i].normalize();
            a_faceOut->m_pNormalIndices[i] = iNormal-1;
        }

        // set string pointer to the next triplet
        pFaceString = &a_faceString[iTripletPos[i+1]];
    }
}

//------------------------------------------------------------------------------

bool cOBJModel::loadMaterialLib(const char a_fileName[],
                cMaterialInfo* a_pMaterials,
                unsigned int* a_curMaterialIndex,
                char a_basePath[])
{
    //----------------------------------------------------------------------
    // loads a material library file (.mtl)
    //----------------------------------------------------------------------

    char str[C_OBJ_MAX_STR_SIZE];       // buffer used while reading the file.
    bool bFirstMaterial = true;         // only increase index after first
                                        // material has been set.

    /////////////////////////////////////////////////////////////////////////
    // OPEN LIBRARY FILE
    /////////////////////////////////////////////////////////////////////////

    FILE *hFile = fopen(a_fileName, "r");

    // success ?
    if (!hFile)
    {
        return (false);
    }

    /////////////////////////////////////////////////////////////////////////
    // READ ALL MATERIAL DEFINITIONS
    /////////////////////////////////////////////////////////////////////////

    // quit reading when end of file has been reached
    while (!feof(hFile))
    {
        // get next string
        readNextString(str, sizeof(str), hFile);

        // is it a "new material" identifier ?
        if (!strncmp(str, C_OBJ_NEW_MTL_ID, sizeof(C_OBJ_NEW_MTL_ID)))
        {
            // only increase index after first material has been set
            if (bFirstMaterial == true)
            {
                // first material has been set
                bFirstMaterial = false;
            }
            else
            {
                // use next index
                (*a_curMaterialIndex)++;
            }

            // read material name
            getTokenParameter(str, sizeof(str), hFile);

            // store material name in the structure
            strcpy(m_pMaterials[*a_curMaterialIndex].m_name, str);
        }

        // transparency
        if (!strncmp(str, C_OBJ_MTL_ALPHA_ID_ALT, sizeof(C_OBJ_MTL_ALPHA_ID_ALT)))
        {
            // read into current material
            if (fscanf(hFile, "%f", &m_pMaterials[*a_curMaterialIndex].m_alpha) < 0) return(false);
            m_pMaterials[*a_curMaterialIndex].m_alpha = 1.0 - m_pMaterials[*a_curMaterialIndex].m_alpha;
        }

        // opacity
        if (!strncmp(str, C_OBJ_MTL_ALPHA_ID, sizeof(C_OBJ_MTL_ALPHA_ID)))
        {
            // read into current material
            if (fscanf(hFile, "%f", &m_pMaterials[*a_curMaterialIndex].m_alpha) < 0) return(false);
        }

        // ambient material properties
        if (!strncmp(str, C_OBJ_MTL_AMBIENT_ID, sizeof(C_OBJ_MTL_AMBIENT_ID)))
        {
            // read into current material
            if (fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_ambient[0],
                &m_pMaterials[*a_curMaterialIndex].m_ambient[1],
                &m_pMaterials[*a_curMaterialIndex].m_ambient[2]) < 0) return(false);
        }

        // diffuse material properties
        if (!strncmp(str, C_OBJ_MTL_DIFFUSE_ID, sizeof(C_OBJ_MTL_DIFFUSE_ID)))
        {
            // read into current material
            if (fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[0],
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[1],
                &m_pMaterials[*a_curMaterialIndex].m_diffuse[2]) < 0) return(false);
        }

        // specular material properties
        if (!strncmp(str, C_OBJ_MTL_SPECULAR_ID, sizeof(C_OBJ_MTL_SPECULAR_ID)))
        {
            // read into current material
            if (fscanf(hFile, "%f %f %f",
                &m_pMaterials[*a_curMaterialIndex].m_specular[0],
                &m_pMaterials[*a_curMaterialIndex].m_specular[1],
                &m_pMaterials[*a_curMaterialIndex].m_specular[2]) < 0) return(false);
        }

        // texture map name
        if (!strncmp(str, C_OBJ_MTL_TEXTURE_ID, sizeof(C_OBJ_MTL_TEXTURE_ID)))
        {
            // read texture filename
            getTokenParameter(str, sizeof(str), hFile);

            // append material library filename to the model's base path
            char textureFile[C_OBJ_SIZE_PATH];
            strcpy(textureFile, a_basePath);
            strcat(textureFile, str);
            
            // store texture filename in the structure
            strcpy(m_pMaterials[*a_curMaterialIndex].m_texture, textureFile);
            
            // load texture and store its ID in the structure
            m_pMaterials[*a_curMaterialIndex].m_textureID = 1;//LoadTexture(szTextureFile);
        }

        // shininess
        if (!strncmp(str, C_OBJ_MTL_SHININESS_ID, sizeof(C_OBJ_MTL_SHININESS_ID)))
        {
            // read into current material
            if (fscanf(hFile, "%f", &m_pMaterials[*a_curMaterialIndex].m_shininess) > 0)
            {
                // OBJ files use a shininess from 0 to 1000; Scale for OpenGL
                m_pMaterials[*a_curMaterialIndex].m_shininess *= 1.28f;
            }
        }
    }

    fclose(hFile);

    // increment index cause LoadMaterialLib() assumes that the passed
    // index is always empty
    (*a_curMaterialIndex)++;

    return (true);
}

//------------------------------------------------------------------------------

void cOBJModel::getFileInfo(FILE *a_hStream, cOBJFileInfo *a_info, const char a_constBasePath[])
{
    /////////////////////////////////////////////////////////////////////////
    // READ THE COUNT OF ALL IMPORTANT IDENTIFIERS OUT OF THE GIVEN STREAM
    /////////////////////////////////////////////////////////////////////////

    char str[C_OBJ_MAX_STR_SIZE];    // Buffer for reading the file
    char basePath[C_OBJ_SIZE_PATH];  // Needed to append a filename to the base path

    // init structure
    a_info->init();

    // rollback the stream
    rewind(a_hStream);

    // quit reading when end of file has been reached
    while (!feof(a_hStream))
    {
        // get next string
        readNextString(str, sizeof(str), a_hStream);

        // vertex?
        if (!strncmp(str, C_OBJ_VERTEX_ID, sizeof(C_OBJ_VERTEX_ID)))
        a_info->m_vertexCount++;

        // texture coordinate?
        if (!strncmp(str, C_OBJ_TEXCOORD_ID, sizeof(C_OBJ_TEXCOORD_ID)))
        a_info->m_texCoordCount++;
        
        // vertex normal?
        if (!strncmp(str, C_OBJ_NORMAL_ID, sizeof(C_OBJ_NORMAL_ID)))
        a_info->m_normalCount++;
        
        // face?
        if (!strncmp(str, C_OBJ_FACE_ID, sizeof(C_OBJ_FACE_ID)))
        a_info->m_faceCount++;

        // material library definition?
        if (!strncmp(str, C_OBJ_MTL_LIB_ID, sizeof(C_OBJ_MTL_LIB_ID)))
        {
            // read the filename of the library
            getTokenParameter(str, sizeof(str), a_hStream);

            // copy the model's base path into a none-constant string
            strcpy(basePath, a_constBasePath);

            // append material library filename to the model's base path
            strcat(basePath, str);

            // append .mtl
            //strcat(szBasePath, ".mtl");

            // open the library file
            FILE *hMaterialLib = fopen(basePath, "r");

            // success?
            if (hMaterialLib)
            {
                // quit reading when end of file has been reached
                while (!feof(hMaterialLib))
                {
                    // read next string
                    if (fscanf(hMaterialLib, "%1024s" ,str) > 0)
                    {

                        // is it a "new material" identifier ?
                        if (!strncmp(str, C_OBJ_NEW_MTL_ID, sizeof(C_OBJ_NEW_MTL_ID)))
                        {
                            // one more material defined
                            a_info->m_materialCount++;
                        }
                    }
                }

                // close material library
                fclose(hMaterialLib);
            }
        }

       // clear string two avoid counting something twice
       memset(str, '\0', sizeof(str));
    }
}

//------------------------------------------------------------------------------

void cOBJModel::readNextString(char *a_str, int a_size, FILE *a_hStream)
{
    /////////////////////////////////////////////////////////////////////////
    // READ THE NEXT STRING THAT ISN'T A COMMENT
    /////////////////////////////////////////////////////////////////////////

    bool bSkipLine;

    // skip all strings that contain comments
    do
    {
        bSkipLine = false; // skip the current line ?

        // read new string
        if (fscanf(a_hStream, "%1024s", a_str) > 0)
        {
            int len = (int)(strlen(a_str));
            if (len>0 && a_str[len-1]==13) a_str[len-1] = 0;

            // is rest of the line a comment ?
            if (!strncmp(a_str, C_OBJ_COMMENT_ID, sizeof(C_OBJ_COMMENT_ID)))
            {
                // skip the rest of the line
                if (fgets(a_str, a_size, a_hStream) != NULL)
                {
                    bSkipLine = true;
                }
            }
        }
    }
    while (bSkipLine == true);
}

//------------------------------------------------------------------------------

void cOBJModel::makePath(char a_fileAndPath[])
{
    /////////////////////////////////////////////////////////////////////////
    // RIPS THE FILENAMES OUT OF A PATH AND ADDS A SLASH (IF NEEDED)
    /////////////////////////////////////////////////////////////////////////

    // get string length
    int iNumChars = (int)(strlen(a_fileAndPath));

    // loop from the last to the first char
    for (int iCurChar=iNumChars-1; iCurChar>=0; iCurChar--)
    {
        // if the current char is a slash / backslash
        if (a_fileAndPath[iCurChar] == char('\\') ||
        a_fileAndPath[iCurChar] == char('/'))
        {
            // terminate the the string behind the slash / backslash
            a_fileAndPath[iCurChar + 1] = char('\0');
            return;
        }
    }

    // no slash there, set string length to zero
    a_fileAndPath[0] = char('\0');
}

//------------------------------------------------------------------------------

void cOBJModel::getTokenParameter(char a_str[],
                                  const unsigned int a_strSize, 
                                  FILE *a_hFile)
{
    char str[C_OBJ_MAX_STR_SIZE];

    /////////////////////////////////////////////////////////////////////////
    // READ THE PARAMETER OF A TOKEN, REMOVE SPACE AND NEWLINE CHARACTER
    /////////////////////////////////////////////////////////////////////////

    // read the parameter after the token
    if (fgets(str, C_OBJ_MAX_STR_SIZE, a_hFile) != NULL)
    {
        // remove newline character after the token
        int len = (int)(strlen(str));
        if (len>1 && str[len-2]==13) str[len-2] = 0;
        else                         str[len-1] = 0;

        char* first_non_whitespace_character = str;
        while( *first_non_whitespace_character == ' ' ) first_non_whitespace_character++;

        // remove space before the token
        strcpy (a_str, first_non_whitespace_character);
    }
}

//------------------------------------------------------------------------------

unsigned int getVertexIndex(cMesh* a_mesh, cOBJModel* a_model,
                            vertexIndexSet_uint_map* a_vertexMap, vertexIndexSet& vis) 
{
    unsigned int index;

    // have we seen this vertex before?
    vertexIndexSet_uint_map::iterator vertexMapIter = a_vertexMap->find(vis);

    // if we have, just grab the new index for this vertex
    if (vertexMapIter != a_vertexMap->end()) 
    {
        index = (*vertexMapIter).second;
        return (index);
    }

    // otherwise create a new vertex and put the mapping in our map
    else 
    {
        index = a_mesh->newVertex(a_model->m_pVertices[vis.vIndex]);
        (*a_vertexMap)[vis] = index;
        return (index);
    }
}


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
