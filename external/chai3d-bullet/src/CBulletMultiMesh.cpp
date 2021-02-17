//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
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
    \author    <amunawar@wpi.edu>
    \author    Francois Conti, Adnan Munawar
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletMultiMesh.h"
//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


//==============================================================================
/*!
    This method assigns a desired position to the object.

    \param  a_position  New desired position.
*/
//==============================================================================
void cBulletMultiMesh::setLocalPos(const cVector3d& a_position)
{
    m_localPos = a_position;

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
    if (m_bulletRigidBody)
        m_bulletRigidBody->setCenterOfMassTransform(trans);
}


//==============================================================================
/*!
    This method assigns a desired rotation to the object.

    \param  a_rotation  New desired orientation.
*/
//==============================================================================
void cBulletMultiMesh::setLocalRot(const cMatrix3d& a_rotation)
{
    m_localRot = a_rotation;

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
    if (m_bulletRigidBody)
        m_bulletRigidBody->setCenterOfMassTransform(trans);
}


//==============================================================================
/*!
    This method updates the position and orientation data from the Bullet
    representation to the CHAI3D representation.
*/
//==============================================================================
void cBulletMultiMesh::updatePositionFromDynamics()
{
    if (m_bulletRigidBody)
    {
        // get transformation matrix of object
        btTransform trans;
        m_bulletRigidBody->getMotionState()->getWorldTransform(trans);
        trans *= getInverseInertialOffsetTransform();

        btVector3 pos = trans.getOrigin();
        btQuaternion q = trans.getRotation();

       // set new position
        m_localPos.set(pos[0],pos[1],pos[2]);

        // set new orientation
        cQuaternion quaternion(q.getW(), q.getX(), q.getY(), q.getZ());
        quaternion.toRotMat(m_localRot);

        // orthogonalize frame
        m_localRot.orthogonalize();
    }
}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletMultiMesh::buildContactTriangles(const double a_margin, cMultiMesh* lowResMesh)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    std::vector<cMesh*> *v_meshes;
    if (lowResMesh ){
        v_meshes = lowResMesh->m_meshes;
    }
    else{
        v_meshes = m_meshes;
    }

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = mesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int i=0; i<numTriangles; i++)
        {
            unsigned int vertexIndex0 = mesh->m_triangles->getVertexIndex0(i);
            unsigned int vertexIndex1 = mesh->m_triangles->getVertexIndex1(i);
            unsigned int vertexIndex2 = mesh->m_triangles->getVertexIndex2(i);

            cVector3d vertex0 = mesh->m_vertices->getLocalPos(vertexIndex0);
            cVector3d vertex1 = mesh->m_vertices->getLocalPos(vertexIndex1);
            cVector3d vertex2 = mesh->m_vertices->getLocalPos(vertexIndex2);

            bulletMesh->addTriangle(btVector3(vertex0(0), vertex0(1), vertex0(2)),
                btVector3(vertex1(0), vertex1(1), vertex1(2)),
                btVector3(vertex2(0), vertex2(1), vertex2(2)));
        }

        // create mesh collision model
        btGImpactMeshShape* collisionShape = new btGImpactMeshShape(bulletMesh);

        // assign settings
        collisionShape->setMargin(a_margin);
        collisionShape->updateBound();

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

        // add collision shape to compound
        compound->addChildShape(localTrans, collisionShape);
    }
}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletMultiMesh::buildContactConvexTriangles(const double a_margin, cMultiMesh* lowResMesh)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    std::vector<cMesh*> *v_meshes;
    if (lowResMesh ){
        v_meshes = lowResMesh->m_meshes;
    }
    else{
        v_meshes = m_meshes;
    }

    int nMesh = 0;
    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        std::cerr << "CREATING CONVEX TRIANGLE MESH FROM MESH NUMBER " << nMesh << std::endl;
        nMesh++;
        cMesh* mesh = (*it);

        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = mesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int i=0; i<numTriangles; i++)
        {
            unsigned int vertexIndex0 = mesh->m_triangles->getVertexIndex0(i);
            unsigned int vertexIndex1 = mesh->m_triangles->getVertexIndex1(i);
            unsigned int vertexIndex2 = mesh->m_triangles->getVertexIndex2(i);

            cVector3d vertex0 = mesh->m_vertices->getLocalPos(vertexIndex0);
            cVector3d vertex1 = mesh->m_vertices->getLocalPos(vertexIndex1);
            cVector3d vertex2 = mesh->m_vertices->getLocalPos(vertexIndex2);

            bulletMesh->addTriangle(btVector3(vertex0(0), vertex0(1), vertex0(2)),
                btVector3(vertex1(0), vertex1(1), vertex1(2)),
                btVector3(vertex2(0), vertex2(1), vertex2(2)));
        }

        // create mesh collision model
        btConvexTriangleMeshShape* collisionShape = new btConvexTriangleMeshShape(bulletMesh);

        // assign settings
        collisionShape->setMargin(a_margin);

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

        // add collision shape to compound
        compound->addChildShape(localTrans, collisionShape);
    }
}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletMultiMesh::buildContactHull(const double a_margin, cMultiMesh* lowResMesh)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    std::vector<cMesh*> *v_meshes;
    if (lowResMesh ){
        v_meshes = lowResMesh->m_meshes;
    }
    else{
        v_meshes = m_meshes;
    }

    std::cerr << "INFO! NUMBER OF MESHES FOR CONVEX HULL " << lowResMesh->getNumMeshes() << std::endl;

    int nMesh = 0;
    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        std::cerr << "CREATING CONVEX HULL FROM MESH NUMBER " << nMesh << std::endl;
        nMesh++;
        cMesh* mesh = (*it);

        // create convex hull
        btConvexHullShape* collisionShape = new btConvexHullShape((double*)(&mesh->m_vertices->m_localPos[0]), mesh->m_vertices->getNumElements(), sizeof(cVector3d));

        // set margin
        collisionShape->setMargin(a_margin);

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

        // add collision shape to compound
        compound->addChildShape(localTrans, collisionShape);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
