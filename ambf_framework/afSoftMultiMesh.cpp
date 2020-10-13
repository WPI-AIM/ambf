//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)

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

    * Neither the name of authors nor the names of its contributors may
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \courtesy: Extended from cBulletMultiMesh Class by Francois Conti
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "afSoftMultiMesh.h"
//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//------------------------------------------------------------------------------

namespace ambf {
using namespace chai3d;

///
/// \brief afSoftMultiMesh::setLocalPos: This method assigns a desired position to the object.
/// \param a_position
///
void afSoftMultiMesh::setLocalPos(const cVector3d& a_position)
{
    m_gelMesh.setLocalPos(a_position);

    // get transformation matrix of object
    btTransform trans;
    btVector3 pos;
    btQuaternion q;

    // set new position
    pos[0] = a_position(0);
    pos[1] = a_position(1);
    pos[2] = a_position(2);

    // set new orientation
    cQuaternion quaternion;
    quaternion.fromRotMat(m_localRot);

    q.setW(quaternion.w);
    q.setX(quaternion.x);
    q.setY(quaternion.y);
    q.setZ(quaternion.z);

    // set new transform
    trans.setOrigin(pos);
    trans.setRotation(q);

    if (m_bulletMotionState)
        m_bulletMotionState->setWorldTransform(trans);
    if (m_bulletSoftBody)
        m_bulletSoftBody->translate(pos);
}


///
/// \brief afSoftMultiMesh::setLocalRot This method assigns a desired rotation to the object.
/// \param a_rotation
///
void afSoftMultiMesh::setLocalRot(const cMatrix3d& a_rotation)
{
    m_gelMesh.setLocalRot(a_rotation);

    // get transformation matrix of object
    btTransform trans;
    btVector3 pos;
    btQuaternion q;

    // set new position
    pos[0] = m_localPos(0);
    pos[1] = m_localPos(1);
    pos[2] = m_localPos(2);

    // set new orientation
    cQuaternion quaternion;
    quaternion.fromRotMat(a_rotation);

    q.setW(quaternion.w);
    q.setX(quaternion.x);
    q.setY(quaternion.y);
    q.setZ(quaternion.z);

    // set new transform
    trans.setOrigin(pos);
    trans.setRotation(q);

    if (m_bulletMotionState)
        m_bulletMotionState->setWorldTransform(trans);
    if (m_bulletSoftBody)
        m_bulletSoftBody->rotate(q);
}

///
/// \brief updateMins: Updated the min and max bounds for a mesh
/// \param x
/// \param y
/// \param z
/// \param v
///
void updateMins(double &x, double &y, double &z, cVector3d &v){
    x = cMin(x,v.x());
    y = cMin(y,v.y());
    z = cMin(z,v.z());
}

///
/// \brief updateMins: Updates the min and max bounds for a mesh
/// \param vMin
/// \param v
///
void updateMins(cVector3d &vMin, cVector3d &v){
    vMin.x(cMin(vMin.x(),v.x()));
    vMin.y(cMin(vMin.y(),v.y()));
    vMin.z(cMin(vMin.z(),v.z()));
}

///
/// \brief updateMaxs
/// \param x
/// \param y
/// \param z
/// \param v
///
void updateMaxs(double &x, double &y, double &z, cVector3d &v){
    x = cMax(x,v.x());
    y = cMax(y,v.y());
    z = cMax(z,v.z());
}

///
/// \brief updateMaxs
/// \param vMax
/// \param v
///
void updateMaxs(cVector3d &vMax, cVector3d &v){
    vMax.x(cMax(vMax.x(),v.x()));
    vMax.y(cMax(vMax.y(),v.y()));
    vMax.z(cMax(vMax.z(),v.z()));
}


///
/// \brief updateMesh: Updates the visuals of a mesh from softbody
/// \param mesh
/// \param sb
/// \param tree
///
void updateMesh(cMesh* mesh, btSoftBody* sb, std::vector<VertexTree>* tree){
    btVector3 bVec;
    cVector3d cVec;
    for (int i = 0 ; i < tree->size() ; i++){
        bVec = sb->m_nodes[i].m_x;
        cVec.set(bVec.x(), bVec.y(), bVec.z());
        for (int j = 0 ; j < (*tree)[i].vertexIdx.size() ; j++){
            int idx = (*tree)[i].vertexIdx[j];
            mesh->m_vertices->setLocalPos(idx, cVec);
        }
    }
    mesh->computeAllNormals();
}

///
/// \brief afSoftMultiMesh::render renders the mesh
/// \param a_options
///
void afSoftMultiMesh::render(cRenderOptions &a_options){
    m_gelMesh.updateVertexPosition();
    m_gelMesh.computeAllNormals();
    m_gelMesh.render(a_options);
}


///
/// \brief afSoftMultiMesh::updatePositionFromDynamics: This method updates the
/// position and orientation data from the Bullet representation to the CHAI3D representation.
///
void afSoftMultiMesh::updatePositionFromDynamics()
{
    if (m_bulletSoftBody)
    {
        updateGELSkeletonFrombtSoftBody();
    }

    // update Transform data for m_rosObj
#ifdef C_ENABLE_CHAI_ENV_SUPPORT
    if(m_afObjectPtr.get() != nullptr){
        m_afObjectPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
        m_afObjectPtr->cur_orientation(q.x, q.y, q.z, q.w);
    }
#endif
}

//bool isPresentInGrid(int cntIdx, cVector3d &v, cVector3d &vMin, cVector3d &vBounds, bool* vtxCheckGrid, int* vtxIdxGrid){
//    int xIdx, yIdx, zIdx;
//    xIdx = (v.x() + vMin.x()) / vBounds.x();
//    yIdx = (v.y() + vMin.y()) / vBounds.y();
//    zIdx = (v.z() + vMin.z()) / vBounds.z();

//    if (vtxCheckGrid[xIdx + yIdx + zIdx] == false){

//    }
//}

///
/// \brief clearArrays
/// \param vtxChkBlock
/// \param vtxIdxBlock
/// \param blockSize
///
void clearArrays(bool * vtxChkBlock, int * vtxIdxBlock, int blockSize){
    int s = blockSize*blockSize*blockSize;
    memset(vtxChkBlock, false, s*sizeof(bool)); // Initialize all the vtx check blocks to 0
    memset(vtxIdxBlock, -1, s*sizeof(int)); // Initialize all the vtx index blocks to -1
}

///
/// \brief afSoftMultiMesh::loadFromFile
/// \param a_filename
/// \return
///
bool afSoftMultiMesh::loadFromFile(std::string a_filename){
    return m_gelMesh.loadFromFile(a_filename);
}

///
/// \brief afSoftMultiMesh::computeUniqueVerticesandTriangles
/// \param mesh
/// \param outputVertices
/// \param outputTriangles
/// \param outputLines
/// \param print_debug_info
///
void afSoftMultiMesh::computeUniqueVerticesandTriangles(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<int>* outputTriangles, std::vector< std::vector<int> >* outputLines, bool print_debug_info){

    // read number of triangles of the object
    int numTriangles = mesh->m_triangles->getNumElements();
    int numVertices = mesh->m_vertices->getNumElements();

    if (print_debug_info){
        printf("# Triangles %d, # Vertices %d \n", numTriangles, numVertices);
    }

    // The max number of vertices to check per block
    int vtxBlockSize = 60;
    // Number of default blocks
    int numBlocks = 1;
    //Define bound for lowest value of vertices
    cVector3d vMin(9999,9999,9999);
    //Define bound for max value of vertices
    cVector3d vMax(-9999,-9999,-9999);
    // Length of the bounds (max - min) for each x,y,z
    cVector3d vBounds;

    // Update the min and max value (x,y,z) of vertices to get bounds
    for (int x = 0 ; x < numVertices ; x++){
        cVector3d v = mesh->m_vertices->getLocalPos(x);
        updateMins(vMin, v); // Iterative search to get the min distance
        updateMaxs(vMax, v); // Iterative search to get the max distance
    }
    // Update magnitude of bound
    vBounds = vMax - vMin;
    if (print_debug_info){
        printf("***************************************\n");
        printf("Vmin = [%f, %f, %f] \n", vMin.x(), vMin.y(), vMin.z());
        printf("Vmax = [%f, %f, %f] \n", vMax.x(), vMax.y(), vMax.z());
        printf("VBounds = [%f, %f, %f] \n", vBounds.x(), vBounds.y(), vBounds.z());
        printf("***************************************\n");
    }
    // Place holder for count of repeat and duplicate vertices
    int uniqueVtxCount = 0;
    int duplicateVtxCount = 0;

    // If number of vertices is greater the vertices per block, increase no of blocks
    // This is to prevent memory exhaustion
    if (numVertices > vtxBlockSize){
        numBlocks = std::ceil((float)numVertices / (float)vtxBlockSize);
    }

    if (print_debug_info){
        printf("Using %d blocks \n", numBlocks);
    }
    // Copy over the vertices to process without altering the original data
    auto vtxArrCopy = mesh->m_vertices->copy();
    // This data-structure is to store the unaltered indices in the first row vertices referring to their
    // original copy in the second row. The third row contains the indexes to the vertices after
    // the unique vertices have been placed in the outputVertices array
    // . E.g. if a vertex at idx 5 was a repeat of vtx at idx 3, orderedVtxList[5][0] = 5 ; orderedVtxList[5][1] = 3;
    // and if the vertex was added to the array of unique vertices at Idx 2 then orderedVtxList[5][2] = 2;
    int orderedVtxList [numVertices][3];
    memset(orderedVtxList, -1, numVertices*3*sizeof(int));

    // This forms a 3D block with all value initiazlied to false
    // If we visit a specific 3D idx, it's set to true to know that we have been there
    bool vtxChkBlock[vtxBlockSize][vtxBlockSize][vtxBlockSize];
    // This forms a 3D block with all values init to -1
    // What ever 3D idx we visited we set the corresponding corrected idx value in this 3D block
    int vtxIdxBlock[vtxBlockSize][vtxBlockSize][vtxBlockSize];
    // To reduce computational cost, if we have already checked a vertex, we can mark it
    bool vtxAlreadyChkd[numVertices];
    // Initialize all the vertex check index to false
    memset(vtxAlreadyChkd, false, numVertices*sizeof(bool));
    // Upper a lower bound for block in x direction
    int xblockLowerBound;
    int xblockUpperBound;
    // Upper a lower bound for block in y direction
    int yblockLowerBound;
    int yblockUpperBound;
    // Upper a lower bound for block in z direction
    int zblockLowerBound;
    int zblockUpperBound;

    int vxKey; // X key to look up in the block
    int vyKey; // Y key to look up in the block
    int vzKey; // Z ket to look up in the block

    double xRes; // X Resolution
    double yRes; // Y Resolution
    double zRes; // X Resolution

    cVector3d vPos; // The position of a vertex
    if(vBounds.x() == 0){
        xRes = 0; // If planar in x direction, set x res to 0
    }
    else{

        xRes = (double) (numVertices - 1) / vBounds.x();
    }
    if(vBounds.y() == 0){
        yRes = 0; // If planar in y direction, set x res to 0
    }
    else{

        yRes = (double) (numVertices - 1) / vBounds.y();
    }
    if(vBounds.z() == 0){
        zRes = 0; // If planar in z direction, set x res to 0
    }
    else{
        zRes = (double) (numVertices - 1) / vBounds.z();
    }

    // Begin the loop to create a hash grid and check for unique vertices
    for (int xblockNum = 0 ; xblockNum < numBlocks ; xblockNum ++){
        xblockLowerBound = xblockNum * vtxBlockSize;
        xblockUpperBound = xblockLowerBound + vtxBlockSize;
        for (int yblockNum = 0 ; yblockNum < numBlocks ; yblockNum ++){
            yblockLowerBound = yblockNum * vtxBlockSize;
            yblockUpperBound = yblockLowerBound + vtxBlockSize;
            for (int zblockNum = 0 ; zblockNum < numBlocks ; zblockNum ++){
                zblockLowerBound = zblockNum * vtxBlockSize;
                zblockUpperBound = zblockLowerBound + vtxBlockSize;
                if (print_debug_info) {printf("Block Num [%d, %d, %d] \n", xblockNum, yblockNum, zblockNum);}
                // Clear the 3D idx and chk arrays to be reused for the new block
                clearArrays(&vtxChkBlock[0][0][0], &vtxIdxBlock[0][0][0], vtxBlockSize);
                for(int idx = 0; idx < numVertices ; idx++){
                    if (!vtxAlreadyChkd[idx]){
                        vPos = vtxArrCopy->getLocalPos(idx);
                        // Generate keys to parse the 3D idx and chk block
                        vxKey = xRes * (vPos.x() - vMin.x());
                        vyKey = yRes * (vPos.y() - vMin.y());
                        vzKey = zRes * (vPos.z() - vMin.z());
                        // Check if the generated keys are in the bounds of the current block
                        if (vxKey >= xblockLowerBound && vyKey >= yblockLowerBound && vzKey >= zblockLowerBound){
                            if (vxKey <= xblockUpperBound && vyKey <= yblockUpperBound && vzKey <= zblockUpperBound){
                                // If the key lies inside the block, offset the value to the block bounds
                                vxKey -= xblockLowerBound; vyKey -= yblockLowerBound; vzKey -= zblockLowerBound;
                                // Mark that we already checked this vertex, so we don't have to check it again
                                vtxAlreadyChkd[idx] = true;
                                // Set the vertexIdx Pair value
                                orderedVtxList[idx][0] = idx;
                                // Check if the key is already set in the chk block
                                if (vtxChkBlock[vxKey][vyKey][vzKey] == false){
                                    // Unique vertex, so mark it as such in the corresponding blocks
                                    vtxChkBlock[vxKey][vyKey][vzKey] = true;
                                    // Set the idx block to the original idx
                                    vtxIdxBlock[vxKey][vyKey][vzKey] = idx;
                                    orderedVtxList[idx][1] = idx;
                                    uniqueVtxCount ++;
                                }
                                else{
                                    // This is not a unique vertex, so get the original idx
                                    // and set it in the corresponding blocks
                                    orderedVtxList[idx][1] = vtxIdxBlock[vxKey][vyKey][vzKey];
                                    duplicateVtxCount++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    //Resize once to save on iterative push/pop time
    outputVertices->resize(uniqueVtxCount*3);
    outputTriangles->resize(numTriangles*3);
    m_vertexTree.resize(uniqueVtxCount);

    // In this loop we append the index of the newly resized array containing
    // the unique vertices to the index of the original array of duplicated vertices.
    // This is an example of the orderedVtxList might look like for usual run
    // After above steps
    // orderedVtxList[:][0] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  4,  2,  1,  7,  4,  7, 10,  4}
    // orderedVtxList[:][1] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
    // And we want:
    // orderedVtxList[:][0] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  4,  2,  1,  7,  4,  7, 10,  4}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  3,  2,  1,  4,  3,  4,  5,  3}
    int vtxCounted = 0;
    for (int aIdx = 0 ; aIdx < numVertices ; aIdx++){
        if (orderedVtxList[aIdx][1] == orderedVtxList[aIdx][0] && orderedVtxList[aIdx][2] == -1){ // A unique vertex
            vPos = mesh->m_vertices->getLocalPos(aIdx);
            (*outputVertices)[3*vtxCounted + 0] = vPos.x();
            (*outputVertices)[3*vtxCounted + 1] = vPos.y();
            (*outputVertices)[3*vtxCounted + 2] = vPos.z();

            orderedVtxList[aIdx][2] = vtxCounted; // Record the index in queue where the unique vertex is added
            m_vertexTree[vtxCounted].vertexIdx.push_back(aIdx);
            vtxCounted++; // Increase the queue idx by 1
        }
        else if(orderedVtxList[aIdx][1] < orderedVtxList[aIdx][0]){ // Not a unique vertex
            int bIdx = orderedVtxList[aIdx][1];
            int cIdx = orderedVtxList[bIdx][2];
            if (orderedVtxList[bIdx][1] != orderedVtxList[bIdx][0] || cIdx == -1){
                // This shouldn't happend. This means that we haven't assigned the third row
                // and row 1 is greater than row 2
                throw "Algorithm Failed for (b[i] < a[i]), a[b[i]] != b[b[i]] : %d and c[b[i]] != -1";
            }
            orderedVtxList[aIdx][2] = cIdx;
            m_vertexTree[cIdx].vertexIdx.push_back(aIdx);
        }
        else if(orderedVtxList[aIdx][1] > orderedVtxList[aIdx][0]){
            int bIdx = orderedVtxList[aIdx][1];
            if (orderedVtxList[bIdx][1] != orderedVtxList[bIdx][0]){
                throw "Algorithm Failed for (b[i] > a[i]), a[b[i]] != b[b[i]] : %d";
            }
            if (orderedVtxList[bIdx][2] == -1){
                vPos = mesh->m_vertices->getLocalPos(bIdx);
                vtxCounted++;
                (*outputVertices)[3*vtxCounted + 0] = vPos.x();
                (*outputVertices)[3*vtxCounted + 1] = vPos.y();
                (*outputVertices)[3*vtxCounted + 2] = vPos.z();
                orderedVtxList[bIdx][2] = vtxCounted;
            }
            orderedVtxList[aIdx][2] = orderedVtxList[bIdx][2];
        }
    }

    // This last loop iterates over the triangle idxes and assigns the re-idxd vertices from the
    // third row of orderedVtxList
    for (int i = 0 ; i < mesh->m_triangles->m_indices.size() ; i++){
        int triIdx = mesh->m_triangles->m_indices[i];
        if ( triIdx >= numVertices){
            std::cerr << "ERROR ! Triangle Vtx Index " << triIdx << " >= # Vertices " << numVertices << std::endl;
        }
        else{
            (*outputTriangles)[i] = orderedVtxList[triIdx][2];
        }
    }

    // This last loop iterates over the lines and assigns the re-idxd vertices to the
    // lines
    if (outputLines){
        for (int i = 0 ; i < outputLines->size() ; i++){
            std::vector<int> originalLine = (*outputLines)[i];
            std::vector<int> reIndexedLine = originalLine;
            for (int vtx = 0 ; vtx < originalLine.size() ; vtx++){
                reIndexedLine[vtx] = orderedVtxList[ originalLine[vtx] ][2];
            }
            (*outputLines)[i].clear();
            (*outputLines)[i] = reIndexedLine;
        }
    }

    if (print_debug_info){
        printf("*** PARALLEL COMPUTE UNIQUE VERTICES AND TRIANGLE INDICES ***\n");

        for (int i = 0 ; i < uniqueVtxCount; i ++){
            printf("Vertex %d = [%f, %f, %f] \n", i, (*outputVertices)[3*i + 0], (*outputVertices)[3*i + 1], (*outputVertices)[3*i + 2]);
        }

        for (int i = 0 ; i < uniqueVtxCount; i ++){
            printf("%d) Children = [", i );
            for (int j = 0 ; j < m_vertexTree[i].vertexIdx.size(); j++){
                printf(" %d", m_vertexTree[i].vertexIdx[j]);
            }
            printf(" ]\n");
        }

        for (int i = 0 ; i < numTriangles; i ++){
            printf("Triangle %d = [%d, %d, %d] \n", i, (*outputTriangles)[3*i], (*outputTriangles)[3*i+1], (*outputTriangles)[3*i+2]);
        }

        for (int i = 0 ; i < numVertices ; i++){
            printf("%d) v[0] = %d \t v[1] = %d \t v[2] = %d \n", i, orderedVtxList[i][0], orderedVtxList[i][1], orderedVtxList[i][2]);
        }
    }

    printf("Unique Vertices Found = %d, Duplicate Vertices Found = %d\n", uniqueVtxCount, duplicateVtxCount);
}



void afSoftMultiMesh::computeUniqueVerticesandTrianglesSequential(cMesh *mesh, std::vector<btScalar> *outputVertices, std::vector<int> *outputTriangles, std::vector<std::vector<int> > *outputLines, bool print_debug_info){
    // read number of triangles of the object
    int numTriangles = mesh->m_triangles->getNumElements();
    int numVertices = mesh->m_vertices->getNumElements();

    // Place holder for count of repeat and duplicate vertices
    int uniqueVtxCount = 0;
    int duplicateVtxCount = 0;

    if (print_debug_info){
        printf("# Triangles %d, # Vertices %d \n", numTriangles, numVertices);
    }

    int orderedVtxList[numVertices][3];


    orderedVtxList[0][0] = 0;
    orderedVtxList[0][1] = 0;
    orderedVtxList[0][2] = -1;

    cVector3d v1Pos, v2Pos;
    for (int i = 0 ; i < numVertices ; i++){
        orderedVtxList[i][0] = i;
        orderedVtxList[i][1] = -1;
        orderedVtxList[i][2] = -1;
    }

    for (int aIdx = 0 ; aIdx < numVertices - 1 ; aIdx++){
        if (orderedVtxList[aIdx][1] == -1){
            orderedVtxList[aIdx][1] = aIdx;
            uniqueVtxCount++;
        }
        else{
            duplicateVtxCount++;
        }
        for (int bIdx = aIdx + 1 ; bIdx < numVertices ; bIdx++){
            v1Pos = mesh->m_vertices->getLocalPos(aIdx);
            v2Pos = mesh->m_vertices->getLocalPos(bIdx);

            if (orderedVtxList[bIdx][1] == -1){
                if ( (v1Pos - v2Pos).length() == 0 ){
                    orderedVtxList[bIdx][1] = aIdx;
                }
            }
        }
    }

    // Check if the last vtx index was assigned
    if (orderedVtxList[numVertices-1][1] == -1){
        orderedVtxList[numVertices-1][1] = orderedVtxList[numVertices-1][0];
        uniqueVtxCount++;
    }

    outputVertices->resize(uniqueVtxCount*3);
    outputTriangles->resize(numTriangles*3);
    m_vertexTree.resize(uniqueVtxCount);

    // In this loop we append the index of the newly resized array containing
    // the unique vertices to the index of the original array of duplicated vertices.
    // This is an example of the orderedVtxList might look like for usual run
    // After above steps
    // orderedVtxList[:][0] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  4,  2,  1,  7,  4,  7, 10,  4}
    // orderedVtxList[:][1] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
    // And we want:
    // orderedVtxList[:][0] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  4,  2,  1,  7,  4,  7, 10,  4}
    // orderedVtxList[:][1] = { 0,  1,  2,  1,  3,  2,  1,  4,  3,  4,  5,  3}
    int vtxCounted = 0;
    cVector3d vPos;
    for (int aIdx = 0 ; aIdx < numVertices ; aIdx++){
        if (orderedVtxList[aIdx][1] == orderedVtxList[aIdx][0] && orderedVtxList[aIdx][2] == -1){ // A unique vertex
            vPos = mesh->m_vertices->getLocalPos(aIdx);
            (*outputVertices)[3*vtxCounted + 0] = vPos.x();
            (*outputVertices)[3*vtxCounted + 1] = vPos.y();
            (*outputVertices)[3*vtxCounted + 2] = vPos.z();

            orderedVtxList[aIdx][2] = vtxCounted; // Record the index in queue where the unique vertex is added
            m_vertexTree[vtxCounted].vertexIdx.push_back(aIdx);
            vtxCounted++; // Increase the queue idx by 1
        }
        else if(orderedVtxList[aIdx][1] < orderedVtxList[aIdx][0]){ // Not a unique vertex
            int bIdx = orderedVtxList[aIdx][1];
            int cIdx = orderedVtxList[bIdx][2];
            if (orderedVtxList[bIdx][1] != orderedVtxList[bIdx][0] || cIdx == -1){
                // This shouldn't happend. This means that we haven't assigned the third row
                // and row 1 is greater than row 2
                throw "Algorithm Failed for (b[i] < a[i]), a[b[i]] != b[b[i]] : and c[b[i]] != -1";
            }
            orderedVtxList[aIdx][2] = cIdx;
            m_vertexTree[cIdx].vertexIdx.push_back(aIdx);
        }
        else if(orderedVtxList[aIdx][1] > orderedVtxList[aIdx][0]){
            int bIdx = orderedVtxList[aIdx][1];
            if (orderedVtxList[bIdx][1] != orderedVtxList[bIdx][0]){
                throw "Algorithm Failed for (b[i] > a[i]), a[b[i]] != b[b[i]] : %d";
            }
            if (orderedVtxList[bIdx][2] == -1){
                vPos = mesh->m_vertices->getLocalPos(bIdx);
                vtxCounted++;
                (*outputVertices)[3*vtxCounted + 0] = vPos.x();
                (*outputVertices)[3*vtxCounted + 1] = vPos.y();
                (*outputVertices)[3*vtxCounted + 2] = vPos.z();
                orderedVtxList[bIdx][2] = vtxCounted;
            }
            orderedVtxList[aIdx][2] = orderedVtxList[bIdx][2];
        }
    }

    // This last loop iterates over the triangle idxes and assigns the re-idxd vertices from the
    // third row of orderedVtxList
    for (int i = 0 ; i < mesh->m_triangles->m_indices.size() ; i++){
        int triIdx = mesh->m_triangles->m_indices[i];        if ( triIdx >= numVertices){
            std::cerr << "ERROR ! Triangle Vtx Index " << triIdx << " >= # Vertices " << numVertices << std::endl;
        }
        else{
            (*outputTriangles)[i] = orderedVtxList[triIdx][2];
        }
    }

    // This last loop iterates over the lines and assigns the re-idxd vertices to the
    // lines
    if (outputLines){
        for (int i = 0 ; i < outputLines->size() ; i++){
            std::vector<int> originalLine = (*outputLines)[i];
            std::vector<int> reIndexedLine = originalLine;
            for (int vtx = 0 ; vtx < originalLine.size() ; vtx++){
                reIndexedLine[vtx] = orderedVtxList[ originalLine[vtx] ][2];
            }
            (*outputLines)[i].clear();
            (*outputLines)[i] = reIndexedLine;
        }
    }

    if(print_debug_info){
        printf("*** SEQUENTIAL COMPUTE UNIQUE VERTICES AND TRIANGLE INDICES ***\n");

        printf("# Unique Vertices = %d, # Duplicate Vertices = %d\n", uniqueVtxCount, duplicateVtxCount);

        for (int i = 0 ; i < numVertices ; i++){
            std::cerr << i << ")\t" << orderedVtxList[i][0] << " ,\t" << orderedVtxList[i][1] << " ,\t" << orderedVtxList[i][2] << std::endl;
        }
    }

}


///
/// \brief afSoftMultiMesh::createGELSkeleton: This method creates a GEL Skeleton
///  based on the underlying bullet softbody
///
void afSoftMultiMesh::createGELSkeleton(){
    int n_btNodes = m_bulletSoftBody->m_nodes.size();
    std::vector<cGELSkeletonNode*> gelNodes;
    gelNodes.resize(n_btNodes);
    m_afSoftNodes.resize(n_btNodes);
    for (int i = 0 ; i < n_btNodes ; i++){
        btSoftBody::Node* btNode = &m_bulletSoftBody->m_nodes[i];

        cGELSkeletonNode* gelNode = new cGELSkeletonNode;
        gelNode->m_pos.set(btNode->m_x.x(), btNode->m_x.y(), btNode->m_x.z());
        gelNode->m_nextRot.identity();
        gelNodes[i] = gelNode;
        m_gelMesh.m_nodes.push_back(gelNode);

        m_afSoftNodes[i].m_gelNode = gelNode;
        m_afSoftNodes[i].m_btNode = btNode;

        for (int j = 0 ; j < m_bulletSoftBody->m_links.size() ; j++){
            btSoftBody::Link* btLink = &m_bulletSoftBody->m_links[j];
            if (btNode == btLink->m_n[0]){
                m_afSoftNodes[i].m_btLinks.push_back(btLink);
            }
        }
    }

    for (int i = 0 ; i < m_trianglesPtr.size()/3 ; i++){
        int nodeIdx0 = m_trianglesPtr[3*i + 0];
        int nodeIdx1 = m_trianglesPtr[3*i + 1];
        int nodeIdx2 = m_trianglesPtr[3*i + 2];
        if (m_bulletSoftBody->checkLink(nodeIdx0, nodeIdx1)){
            cGELSkeletonLink* link = new cGELSkeletonLink(gelNodes[nodeIdx0], gelNodes[nodeIdx1]);
            m_gelMesh.m_links.push_back(link);
            // Store the link the afNode DS so that it can be used later
            m_afSoftNodes[nodeIdx0].m_gelLinks.push_back(link);
        }
        if (m_bulletSoftBody->checkLink(nodeIdx1, nodeIdx2)){
            cGELSkeletonLink* link = new cGELSkeletonLink(gelNodes[nodeIdx1], gelNodes[nodeIdx2]);
            m_gelMesh.m_links.push_back(link);
            // Store the link the afNode DS so that it can be used later
            m_afSoftNodes[nodeIdx1].m_gelLinks.push_back(link);
        }
        if (m_bulletSoftBody->checkLink(nodeIdx2, nodeIdx0)){
            cGELSkeletonLink* link = new cGELSkeletonLink(gelNodes[nodeIdx2], gelNodes[nodeIdx0]);
            m_gelMesh.m_links.push_back(link);
            // Store the link the afNode DS so that it can be used later
            m_afSoftNodes[nodeIdx2].m_gelLinks.push_back(link);
        }
    }

    m_gelMesh.m_showSkeletonModel = false;
    m_gelMesh.m_useSkeletonModel = true;
}

btVector3 cVec2bVec(cVector3d &cVec){
    btVector3 bVec(cVec.x(), cVec.y(), cVec.z());
    return bVec;
}

cVector3d bVec2cVec(btVector3 &bVec){
    cVector3d cVec(bVec.x(), bVec.y(), bVec.z());
    return cVec;
}

///
/// \brief afSoftMultiMesh::updateGELSkeletonFrombtSoftBody
///
void afSoftMultiMesh::updateGELSkeletonFrombtSoftBody(){

    for (int i = 0 ; i < m_afSoftNodes.size() ; i++){
//        int lastLinkIdx = m_afSoftNodes[i].m_btLinks.size() - 1 ;
//        btSoftBody::Link* btLink = m_afSoftNodes[i].m_btLinks[lastLinkIdx];
        cVector3d vPos =  bVec2cVec(m_afSoftNodes[i].m_btNode->m_x);
//        btVector3 dPos = btLink->m_n[1]->m_x - btLink->m_n[0]->m_x;
//        cVector3d vZ = bVec2cVec(m_afSoftNodes[i].m_btNode->m_n);
//        vZ.normalize();
//        cVector3d vX = bVec2cVec(dPos);
//        vX.normalize();
        m_afSoftNodes[i].m_gelNode->m_nextPos.set(vPos.x(), vPos.y(), vPos.z());
//        cVector3d vY = cCross(vZ, vX);
//        vY.normalize();
//        vX = cCross(vY, vZ);
//        m_afSoftNodes[i].m_gelNode->m_nextRot.setCol0(vX);
//        m_afSoftNodes[i].m_gelNode->m_nextRot.setCol1(vY);
//        m_afSoftNodes[i].m_gelNode->m_nextRot.setCol2(vZ);

    }

    std::list<cGELSkeletonNode*>::iterator n;
    for(n = m_gelMesh.m_nodes.begin(); n != m_gelMesh.m_nodes.end(); ++n)
    {
        (*n)->applyNextPose();
    }
}


///
/// \brief afSoftMultiMesh::createLinksFromLineIdxs
/// \param a_sb
/// \param a_line
/// \return
///
bool afSoftMultiMesh::createLinksFromLines(btSoftBody *a_sb, std::vector< std::vector<int> > *a_lines, cMesh* a_mesh){
    if (a_sb && a_lines){
//        btSoftBody::Material* pm = a_sb->appendMaterial();
//                    pm->m_kLST = 1;
//                    pm->m_kAST = 1;
//                    pm->m_kVST = 1;
//                    pm->m_flags = btSoftBody::fMaterial::Default;
        for(int lIdx = 0 ; lIdx < a_lines->size() ; lIdx++){
            std::vector<int> line = (*a_lines)[lIdx];

            for(int vIdx = 0 ; vIdx < line.size() - 1 ; vIdx++){
                // The indexes start at zero, correct this
                int node0Idx = line[vIdx];
                int node1Idx = line[vIdx+1];
                int nodesSize = a_sb->m_nodes.size();

                if (node0Idx >= nodesSize){
                    int originalVtxIdx = a_mesh->m_lines[lIdx][vIdx];
                    cVector3d pos = a_mesh->m_vertices->getLocalPos(originalVtxIdx);
                    btVector3 vPos = cVec2bVec(pos);
                    btSoftBody::Node n;
                    n.m_im = 1;
                    n.m_im = 1 / n.m_im;
                    n.m_x = vPos;
                    n.m_q = n.m_x;
                    n.m_n = btVector3(0, 0, 1);
                    n.m_leaf = m_bulletSoftBody->m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x, 0.1), &n);
                    n.m_material = m_bulletSoftBody->m_materials[0];
                    a_sb->m_nodes.push_back(n);
                    node0Idx = a_sb->m_nodes.size() - 1;
                }

                if (node1Idx >= nodesSize){
                    int originalVtxIdx = a_mesh->m_lines[lIdx][vIdx];
                    cVector3d pos = a_mesh->m_vertices->getLocalPos(originalVtxIdx);
                    btVector3 vPos = cVec2bVec(pos);
                    btSoftBody::Node n;
                    n.m_im = 1;
                    n.m_im = 1 / n.m_im;
                    n.m_x = vPos;
                    n.m_q = n.m_x;
                    n.m_n = btVector3(0, 0, 1);
                    n.m_leaf = m_bulletSoftBody->m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x, 0.1), &n);
                    n.m_material = m_bulletSoftBody->m_materials[0];
                    a_sb->m_nodes.push_back(n);
                    node1Idx = a_sb->m_nodes.size() - 1;
                }
                a_sb->appendLink(node0Idx, node1Idx);
            }
        }
    }
}



/// Copied from btSoftBodyHelpers with few modifications
///
///
btSoftBody* CreateFromMesh(btSoftBodyWorldInfo& worldInfo, const btScalar* vertices, int nNodes,
                                                 const int* triangles,
                                                 int ntriangles, bool randomizeConstraints=true)
{
    int maxidx = 0;
    int i, j, ni;

    for (i = 0, ni = ntriangles * 3; i < ni; ++i)
    {
        maxidx = btMax(triangles[i], maxidx);
    }
    ++maxidx;
    btAlignedObjectArray<bool> chks;
    btAlignedObjectArray<btVector3> vtx;
    chks.resize(maxidx * maxidx, false);
    vtx.resize(nNodes);
    for (i = 0, j = 0; i < nNodes * 3; ++j, i += 3)
    {
        vtx[j] = btVector3(vertices[i], vertices[i + 1], vertices[i + 2]);
    }
    btSoftBody* psb = new btSoftBody(&worldInfo, vtx.size(), &vtx[0], 0);
    for (i = 0, ni = ntriangles * 3; i < ni; i += 3)
    {
        const int idx[] = {triangles[i], triangles[i + 1], triangles[i + 2]};
#define IDX(_x_, _y_) ((_y_)*maxidx + (_x_))
        for (int j = 2, k = 0; k < 3; j = k++)
        {
            if (!chks[IDX(idx[j], idx[k])])
            {
                chks[IDX(idx[j], idx[k])] = true;
                chks[IDX(idx[k], idx[j])] = true;
                psb->appendLink(idx[j], idx[k]);
            }
        }
#undef IDX
        psb->appendFace(idx[0], idx[1], idx[2]);
    }

    if (randomizeConstraints)
    {
        psb->randomizeConstraints();
    }

    return (psb);
}

///
/// \brief afSoftMultiMesh::buildContactTriangles: This method creates a Bullet collision model for this object.
/// \param a_margin
/// \param lowResMesh
///
void afSoftMultiMesh::buildContactTriangles(const double a_margin, cMultiMesh* lowResMesh)
{
    m_gelMesh.buildVertices();
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    std::vector<cMesh*> *v_meshes;
    if (lowResMesh ){
        v_meshes = lowResMesh->m_meshes;
    }
    else{
        v_meshes = m_gelMesh.m_meshes;
    }

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // read number of triangles of the object
        int numTriangles = mesh->m_triangles->getNumElements();
        std::vector<std::vector <int> > _lines = mesh->m_lines;
//        computeUniqueVerticesandTriangles(mesh, &m_verticesPtr, &m_trianglesPtr, &_lines, true);
        computeUniqueVerticesandTrianglesSequential(mesh, &m_verticesPtr, &m_trianglesPtr, &_lines, false);
        if (m_trianglesPtr.size() > 0){
            m_bulletSoftBody = CreateFromMesh(*m_dynamicWorld->m_bulletSoftBodyWorldInfo,
                                                                    m_verticesPtr.data(), m_verticesPtr.size() / 3, m_trianglesPtr.data(), numTriangles);
            createLinksFromLines(m_bulletSoftBody, &_lines, mesh);
        }
        else{
            m_bulletSoftBody = new btSoftBody(m_dynamicWorld->m_bulletSoftBodyWorldInfo);
            /* Default material	*/
            btSoftBody::Material* pm = m_bulletSoftBody->appendMaterial();
            pm->m_kLST = 1;
            pm->m_kAST = 1;
            pm->m_kVST = 1;
            pm->m_flags = btSoftBody::fMaterial::Default;
            if (m_bulletSoftBody){
                m_bulletSoftBody->m_nodes.resize(mesh->m_vertices->getNumElements());
                for(int nIdx = 0 ; nIdx < m_bulletSoftBody->m_nodes.size() ; nIdx++){
                    cVector3d pos = mesh->m_vertices->getLocalPos(nIdx);
                    btVector3 vPos = cVec2bVec(pos);
                    btSoftBody::Node& n = m_bulletSoftBody->m_nodes[nIdx];
                    n.m_im = 1;
                    n.m_im = 1 / n.m_im;
                    n.m_x = vPos;
                    n.m_q = n.m_x;
                    n.m_n = btVector3(0, 0, 1);
                    n.m_leaf = m_bulletSoftBody->m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x, 0.1), &n);
                    n.m_material = m_bulletSoftBody->m_materials[0];
                }
            }
            createLinksFromLines(m_bulletSoftBody, &mesh->m_lines, mesh);
        }
        m_bulletSoftBody->getCollisionShape()->setMargin(a_margin);
        // Set the default radius of the GEL Skeleton Node
        cGELSkeletonNode::s_default_radius = m_bulletSoftBody->getCollisionShape()->getMargin();
        createGELSkeleton();
        m_gelMesh.connectVerticesToSkeleton(false);
        // add to compound object
        btTransform localTrans;
        btVector3 pos;
        btQuaternion q;

        // set new position
        cVector3d posMesh = mesh->getLocalPos();
        pos[0] = posMesh(0);
        pos[1] = posMesh(1);
        pos[2] = posMesh(2);

        // set new orientation
        cMatrix3d rotMesh = mesh->getLocalRot();
        cQuaternion quaternion;
        quaternion.fromRotMat(rotMesh);

        q.setW(quaternion.w);
        q.setX(quaternion.x);
        q.setY(quaternion.y);
        q.setZ(quaternion.z);

        // set new transform
        localTrans.setOrigin(pos);
        localTrans.setRotation(q);

        // Apply the inertial transform offset
        localTrans *= getInverseInertialOffsetTransform();
    }
    if(lowResMesh){
        lowResMesh->m_meshes->clear();
    }
}


///
/// \brief afSoftMultiMesh::buildContactConvexTriangles
/// \param a_margin
///
void afSoftMultiMesh::buildContactConvexTriangles(const double a_margin)
{

}


///
/// \brief afSoftMultiMesh::buildContactHull: This method creates a Bullet collision model for this object.
/// \param a_margin
///
void afSoftMultiMesh::buildContactHull(const double a_margin)
{
    m_gelMesh.buildVertices();
    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // read number of triangles of the object
        int numVertices = mesh->m_vertices->getNumElements();

        m_verticesVecPtr.resize(numVertices);

        // add all triangles to Bullet model
        for (int i=0; i<numVertices; i++)
        {
            auto vPos = mesh->m_vertices->getLocalPos(i);
            m_verticesVecPtr[i].setValue(vPos.x(), vPos.y(), vPos.z());

        }

        m_bulletSoftBody = btSoftBodyHelpers::CreateFromConvexHull(*m_dynamicWorld->m_bulletSoftBodyWorldInfo, m_verticesVecPtr.data(), numVertices);

    }
}

///
/// \brief afSoftMultiMesh::buildDynamicModel: Build the dynamic model of the bullet soft body.
///
void afSoftMultiMesh::buildDynamicModel(){
    // add collision shape to compound
    m_bulletSoftBody->setTotalMass(m_mass, false);
    m_bulletSoftBody->getCollisionShape()->setUserPointer(m_bulletSoftBody);
    btSoftRigidDynamicsWorld *softWorld = (btSoftRigidDynamicsWorld*) m_dynamicWorld->m_bulletWorld;
    softWorld->addSoftBody(m_bulletSoftBody);
    m_dynamicWorld->m_bulletSoftBodyWorldInfo->m_sparsesdf.Reset();
}

///
/// \brief afSoftMultiMesh::scale
/// \param a_scaleFactor
/// \param a_affectChildren
///
void afSoftMultiMesh::scale(const double &a_scaleFactor, const bool a_affectChildren){
    m_gelMesh.scale(a_scaleFactor);
}


//------------------------------------------------------------------------------
} // namespace ambf
//------------------------------------------------------------------------------
