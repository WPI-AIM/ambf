//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (www.aimlab.wpi.edu)

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

    \author:    <http://www.aimlab.wpi.edu>
    \author:    <amunawar@wpi.edu>
    \author:    Adnan Munawar
    \coutesy:   Extended from cBulletMultiMesh Class by Francois Conti
    \version:   $
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
    memset(vtxChkBlock, false, s*sizeof(bool));
    memset(vtxIdxBlock, -1, s*sizeof(int));
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
/// \param print_debug_info
///
void afSoftMultiMesh::computeUniqueVerticesandTriangles(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<int>* outputTriangles, bool print_debug_info){

    // read number of triangles of the object
    int numTriangles = mesh->m_triangles->getNumElements();
    int numVertices = mesh->m_vertices->getNumElements();

    if (print_debug_info){
        printf("# Triangles %d, # Vertices %d \n", numTriangles, numVertices);
    }

    // The max number of vertices to check per block
    int blockSize = 60;
    // Number of default blocks
    int numBlocks = 1;
    //Define bound for lowest value of vertices
    cVector3d vMin(9999,9999,9999);
    //Define bound for max value of vertices
    cVector3d vMax(-9999,-9999,-9999);
    cVector3d vBounds;

    // Update the min and max value x,y,z value of vertices to get bounds
    for (int x = 0 ; x < numVertices ; x++){
        cVector3d v = mesh->m_vertices->getLocalPos(x);
        updateMins(vMin, v);
        updateMaxs(vMax, v);
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
    if (numVertices > blockSize){
        numBlocks = std::ceil((float)numVertices / (float)blockSize);
    }

    if (print_debug_info){
        printf("Using %d blocks \n", numBlocks);
    }
    // Copy over the vertices to process without altering the original data
    auto vtxArrCopy = mesh->m_vertices->copy();
    // This tri vector is to store the unaltered indices in the first row vertices referring to their
    // original copy in the second row. The third row contains the index to the vertices after
    // the unique vertices have been place in the outputVertices array
    // . E.g. if a vertex at idx 5 was a repeat of vtx at idx 3, vtxIdxPair[5][0] = 5 ; vtxIdxPair[5][1] = 3;
    // and if the vertex was added to the array of unique vertices at Idx 2 then vtxIdxPair[5][2] = 2;
    int vtxIdxTriPair [numVertices][3];
    memset(vtxIdxTriPair, -1, numVertices*3*sizeof(int));

    // This forms a 3D block with all value init to false
    // If we visit a specific 3D idx, its set to true to know that we have been there
    bool vtxChkBlock[blockSize][blockSize][blockSize];
    // This forms a 3D block with all values init to -1
    // What ever 3D idx we visited we set the corresponding corrected idx value in this 3D block
    int vtxIdxBlock[blockSize][blockSize][blockSize];
    // To reduce computational cost, if we have already checked a vertex, we can mark it
    bool vtxAlreadyChkd[numVertices];
    memset(vtxAlreadyChkd, false, numVertices*sizeof(bool));
    int xblockLowerBound; int xblockUpperBound;
    int yblockLowerBound; int yblockUpperBound;
    int zblockLowerBound; int zblockUpperBound;
    int vxKey;
    int vyKey;
    int vzKey;
    cVector3d vPos;
    double xCoeff;
    double yCoeff;
    double zCoeff;
    if(vBounds.x() == 0){
        xCoeff = 0;
    }
    else{

        xCoeff = (double) (numVertices - 1) / vBounds.x();
    }
    if(vBounds.y() == 0){
        yCoeff = 0;
    }
    else{

        yCoeff = (double) (numVertices - 1) / vBounds.y();
    }
    if(vBounds.z() == 0){
        zCoeff = 0;
    }
    else{

        zCoeff = (double) (numVertices - 1) / vBounds.z();
    }
    for (int xblockNum = 0 ; xblockNum < numBlocks ; xblockNum ++){
        xblockLowerBound = xblockNum * blockSize;
        xblockUpperBound = xblockLowerBound + blockSize;
        for (int yblockNum = 0 ; yblockNum < numBlocks ; yblockNum ++){
            yblockLowerBound = yblockNum * blockSize;
            yblockUpperBound = yblockLowerBound + blockSize;
            for (int zblockNum = 0 ; zblockNum < numBlocks ; zblockNum ++){
                zblockLowerBound = zblockNum * blockSize;
                zblockUpperBound = zblockLowerBound + blockSize;
                if (print_debug_info) {printf("Block Num [%d, %d, %d] \n", xblockNum, yblockNum, zblockNum);}
                // Clear the 3D idx and chk arrays to be reused for the new block
                clearArrays(&vtxChkBlock[0][0][0], &vtxIdxBlock[0][0][0], blockSize);
                for(int idx = 0; idx < numVertices ; idx++){
                    if (!vtxAlreadyChkd[idx]){
                        vPos = vtxArrCopy->getLocalPos(idx);
                        // Generate keys to parse the 3D idx and chk block
                        vxKey = xCoeff * (vPos.x() - vMin.x());
                        vyKey = yCoeff * (vPos.y() - vMin.y());
                        vzKey = zCoeff * (vPos.z() - vMin.z());
                        // Check if the generated keys are in the bounds of the current block
                        if (vxKey >= xblockLowerBound && vyKey >= yblockLowerBound && vzKey >= zblockLowerBound){
                            if (vxKey <= xblockUpperBound && vyKey <= yblockUpperBound && vzKey <= zblockUpperBound){
                                // If the key lies inside the block, offset the value to the block bounds
                                vxKey -= xblockLowerBound; vyKey -= yblockLowerBound; vzKey -= zblockLowerBound;
                                // Mark that we already checked this vertex, so we don't have to check it again
                                vtxAlreadyChkd[idx] = true;
                                // Check if the key is already set in the chk block
                                if (vtxChkBlock[vxKey][vyKey][vzKey] == false){
                                    // Unique vertex, so mark it as such in the corresponding blocks
                                    vtxChkBlock[vxKey][vyKey][vzKey] = true;
                                    // Set the idx block to the original idx
                                    vtxIdxBlock[vxKey][vyKey][vzKey] = idx;
                                    // Set the vertexIdx Pair value
                                    vtxIdxTriPair[idx][0] = idx;
                                    vtxIdxTriPair[idx][1] = idx;
                                    uniqueVtxCount ++;
                                }
                                else{
                                    // This is not a unique vertex, so get the original idx
                                    // and set it in the corresponding blocks
                                    vtxIdxTriPair[idx][0] = idx;
                                    vtxIdxTriPair[idx][1] = vtxIdxBlock[vxKey][vyKey][vzKey];
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

    // This loop uses a logic that appends the index of the new resized array containing
    // the unique vertices to the index of the original array of duplicated vertices.
    int vtxCounted = -1;
    for (int i = 0 ; i < numVertices ; i++){
        if (vtxIdxTriPair[i][1] == vtxIdxTriPair[i][0] && vtxIdxTriPair[i][2] == -1){
            vPos = mesh->m_vertices->getLocalPos(i);
            vtxCounted++;
            (*outputVertices)[3*vtxCounted + 0] = vPos.x();
            (*outputVertices)[3*vtxCounted + 1] = vPos.y();
            (*outputVertices)[3*vtxCounted + 2] = vPos.z();

            vtxIdxTriPair[i][2] = vtxCounted;
            m_vertexTree[vtxCounted].vertexIdx.push_back(i);
        }
        else if(vtxIdxTriPair[i][1] < vtxIdxTriPair[i][0]){
            int bi = vtxIdxTriPair[i][1];
            int ci = vtxIdxTriPair[bi][2];
            if (vtxIdxTriPair[bi][1] != vtxIdxTriPair[bi][0] || ci == -1){
                throw 'Algorithm Failed for (b[i] < a[i]), a[b[i]] != b[b[i]] : %d and c[b[i]] != -1';
            }
            vtxIdxTriPair[i][2] = ci;
            m_vertexTree[ci].vertexIdx.push_back(i);
        }
        else if(vtxIdxTriPair[i][1] > vtxIdxTriPair[i][0]){
            int bi = vtxIdxTriPair[i][1];
            if (vtxIdxTriPair[bi][1] != vtxIdxTriPair[bi][0]){
                throw 'Algorithm Failed for (b[i] > a[i]), a[b[i]] != b[b[i]] : %d';
            }
            if (vtxIdxTriPair[bi][2] == -1){
                vPos = mesh->m_vertices->getLocalPos(bi);
                vtxCounted++;
                (*outputVertices)[3*vtxCounted + 0] = vPos.x();
                (*outputVertices)[3*vtxCounted + 1] = vPos.y();
                (*outputVertices)[3*vtxCounted + 2] = vPos.z();
                vtxIdxTriPair[bi][2] = vtxCounted;
            }
            vtxIdxTriPair[i][2] = vtxIdxTriPair[bi][2];
        }
    }

    // This last loop iterates over the triangle idxes and assigns the re-idxd vertices from the
    // third row of vtxIdxTriPair
    for (int i = 0 ; i < numTriangles*3 ; i++){
        (*outputTriangles)[i] = vtxIdxTriPair[i][2];
    }

    if (print_debug_info){
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

        for (int i = 0 ; i < numTriangles*3 ; i++){
            printf("v[0] = %d \t v[1] = %d \t v[2] = %d \n", vtxIdxTriPair[i][0], vtxIdxTriPair[i][1], vtxIdxTriPair[i][2]);
        }
    }

    printf("Unique Vertices Found = %d, Duplicate Vertices Found = %d\n", uniqueVtxCount, duplicateVtxCount);
}


///
/// \brief afSoftMultiMesh::createGELSkeleton: This method creates a GEL Skeleton
///  based on the underlying bullet softbody
///
void afSoftMultiMesh::createGELSkeleton(){
    int n_btLinks = m_bulletSoftBody->m_links.size();
    int n_btNodes = m_bulletSoftBody->m_nodes.size();
    std::vector<cGELSkeletonNode*> gelNodes;
    gelNodes.resize(n_btNodes);
    m_afSoftNodes.resize(n_btNodes);
    for (int i = 0 ; i < n_btNodes ; i++){
        btSoftBody::Node* btNode = &m_bulletSoftBody->m_nodes[i];
        btSoftBody::Link btLink = m_bulletSoftBody->m_links[i];

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
        computeUniqueVerticesandTriangles(mesh, &m_verticesPtr, &m_trianglesPtr);
        m_bulletSoftBody = btSoftBodyHelpers::CreateFromTriMesh(*m_dynamicWorld->m_bulletSoftBodyWorldInfo,
                                                                m_verticesPtr.data(), m_trianglesPtr.data(), numTriangles);
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
        localTrans *= m_inertialOffsetTransform.inverse();
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
