//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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
*/
//==============================================================================

//------------------------------------------------------------------------------
#include <chrono>
#include "afFramework.h"
#include "afConversions.h"
#include "afShaders.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
//------------------------------------------------------------------------------

#include "ros_comm_plugin/ObjectCommPlugin.h"
#include "ros_comm_plugin/WorldCommPlugin.h"
#include "ros_comm_plugin/VideoStreamerPlugin.h"
#include "ros_comm_plugin/DepthStreamerPlugin.h"

//------------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Declare Static Variables

double afWorld::m_enclosureL;
double afWorld::m_enclosureW;
double afWorld::m_enclosureH;
int afWorld::m_maxIterations;

GLFWwindow* afCamera::s_mainWindow = nullptr;
int afCamera::s_numWindows = 0;
int afCamera::s_cameraIdx = 0;
int afCamera::s_windowIdx = 0;

// afComm static vars
bool afComm::s_globalOverride = false;
int afComm::s_maxFreq = 1000;
int afComm::s_minFreq = 50;
string afComm::s_global_namespace_prefix = "";

btGhostPairCallback* afGhostObject::m_bulletGhostPairCallback = nullptr;
//------------------------------------------------------------------------------


/// End declare static variables

cMesh *afShapeUtils::createVisualShape(const afPrimitiveShapeAttributes *a_primitiveShape){
    cMesh* primMesh = new cMesh();
    int dx = 32; // Default x resolution for shape
    int dy = 32; // Default y resolution for shape
    int dz = 5; // Default z resolution for shape

    switch (a_primitiveShape->getShapeType()) {
    case afPrimitiveShapeType::BOX:{
        cCreateBox(primMesh, a_primitiveShape->getDimensions()(0), a_primitiveShape->getDimensions()(1), a_primitiveShape->getDimensions()(2));
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        cCreateSphere(primMesh, a_primitiveShape->getRadius(), dx, dy);
        break;
    }
    case afPrimitiveShapeType::CYLINDER:{
        cCreateCylinder(primMesh, a_primitiveShape->getHeight(), a_primitiveShape->getRadius(), dx, dy, dz, true, true, cVector3d(0.0, 0.0,-0.5 * a_primitiveShape->getHeight()));
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        cCreateEllipsoid(primMesh, a_primitiveShape->getRadius(), a_primitiveShape->getRadius(), a_primitiveShape->getHeight(), dx, dy);
        break;
    }
    case afPrimitiveShapeType::CONE:{
        cCreateCone(primMesh, a_primitiveShape->getHeight(), a_primitiveShape->getRadius(), 0, dx, dy, dz, true, true, cVector3d(0.0, 0.0, -0.5 * a_primitiveShape->getHeight()));
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        cVector3d planeNormal = to_cVector3d(a_primitiveShape->getPlaneNormal());
        cVector3d pos = a_primitiveShape->getPlaneConstant() * planeNormal;
        cQuaternion rot_quat = afUtils::getRotBetweenVectors<cQuaternion, cVector3d>(cVector3d(0, 0, 1), planeNormal);
        cMatrix3d rot;
        rot_quat.toRotMat(rot);
        cCreatePlane(primMesh, 100, 100, pos, rot);
        break;
    }
    default:
        cerr << "ERROR!, VISUAL SHAPE PRIMITIVE TYPE NOT UNDERSTOOD " << endl;
        break;
    }
    return primMesh;
}


btCollisionShape *afShapeUtils::createCollisionShape(const afPrimitiveShapeAttributes *a_primitiveShape, double a_margin){
    btCollisionShape* collisionShape;

    switch (a_primitiveShape->m_shapeType) {
    case afPrimitiveShapeType::BOX:{
        afVector3d dims = a_primitiveShape->getDimensions();
        btVector3 halfExtents(dims(0)/2, dims(1)/2, dims(2)/2);
        collisionShape = new btBoxShape(halfExtents);
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        collisionShape = new btSphereShape(a_primitiveShape->getRadius());
        break;
    }
    case afPrimitiveShapeType::CYLINDER:{
        double radius = a_primitiveShape->getRadius();
        double height = a_primitiveShape->getHeight();
        switch (a_primitiveShape->getAxisType()) {
        case afAxisType::X:{
            btVector3 halfExtents(height/2, radius, radius);
            collisionShape = new btCylinderShapeX(halfExtents);
            break;
        }
        case afAxisType::Y:{
            btVector3 halfExtents(radius, height/2, radius);
            collisionShape = new btCylinderShape(halfExtents);
            break;
        }
        case afAxisType::Z:{
            btVector3 halfExtents(radius, radius, height/2);
            collisionShape = new btCylinderShapeZ(halfExtents);
            break;
        }
        default:
            cerr << "ERROR! AXIS TYPE FOR COLLISION SHAPE NOT UNDERSTOOD" << endl;
            break;
        }
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        double radius = a_primitiveShape->getRadius();
        double height = a_primitiveShape->getHeight();
        // Adjust for height as bullet treats the height as the distance
        // between the two spheres forming the capsule's ends.
        height = height - 2 * radius;
        switch (a_primitiveShape->getAxisType()) {
        case afAxisType::X:{
            collisionShape = new btCapsuleShapeX(radius, height);
            break;
        }
        case afAxisType::Y:{
            collisionShape = new btCapsuleShape(radius, height);
            break;
        }
        case afAxisType::Z:{
            collisionShape = new btCapsuleShapeZ(radius, height);
            break;
        }
        default:
            cerr << "ERROR! AXIS TYPE FOR COLLISION SHAPE NOT UNDERSTOOD" << endl;
            break;
        }
        break;
    }
    case afPrimitiveShapeType::CONE:{
        double radius = a_primitiveShape->getRadius();
        double height = a_primitiveShape->getHeight();
        switch (a_primitiveShape->getAxisType()) {
        case afAxisType::X:{
            collisionShape = new btConeShapeX(radius, height);
            break;
        }
        case afAxisType::Y:{
            collisionShape = new btConeShape(radius, height);
            break;
        }
        case afAxisType::Z:{
            collisionShape = new btConeShapeZ(radius, height);
            break;
        }
        default:
            cerr << "ERROR! AXIS TYPE FOR COLLISION SHAPE NOT UNDERSTOOD" << endl;
            break;
        }
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        btVector3 planeNormal = to_btVector(a_primitiveShape->getPlaneNormal());
        collisionShape = new btStaticPlaneShape(planeNormal, a_primitiveShape->getPlaneConstant());
        break;
    }
    default:
        cerr << "ERROR! COLLISION SHAPE PRIMITIVE TYPE NOT UNDERSTOOD" << endl;
        break;
    };
    collisionShape->setMargin(a_margin);
    return collisionShape;
}


///
/// \brief cMeshTObtTriangleMesh
/// \param a_mesh
/// \return
///
btTriangleMesh* cMeshTObtTriangleMesh(const cMesh* a_mesh){

    // bullet mesh
    btTriangleMesh* bulletMesh = new btTriangleMesh();

    // read number of indices of the object
    unsigned int numIndices = a_mesh->m_triangles->getNumElements();

    // add all indices to Bullet model
    for (unsigned int i=0; i<numIndices; i++)
    {
        unsigned int vertexIndex0 = a_mesh->m_triangles->getVertexIndex0(i);
        unsigned int vertexIndex1 = a_mesh->m_triangles->getVertexIndex1(i);
        unsigned int vertexIndex2 = a_mesh->m_triangles->getVertexIndex2(i);
        bulletMesh->addTriangleIndices(vertexIndex0, vertexIndex1, vertexIndex2);
    }

    unsigned int numVertices = a_mesh->m_vertices->getNumElements();

    for (unsigned int i=0; i<numVertices; i++)
    {
        cVector3d vertex = a_mesh->m_vertices->getLocalPos(i);
        bulletMesh->findOrAddVertex(btVector3(vertex(0), vertex(1), vertex(2)), false);
    }

    return bulletMesh;
}


///
/// \brief afShapeUtils::createCollisionShape
/// \param a_collisionMesh
/// \param a_margin
/// \param a_meshType
/// \return
///
btCollisionShape* afShapeUtils::createCollisionShape(cMesh *a_collisionMesh,
                                                     double a_margin,
                                                     afCollisionMeshShapeType a_meshType)
{
    // create the collision shape
    btCollisionShape* collisionShape;

    switch (a_meshType) {
    case afCollisionMeshShapeType::CONCAVE_MESH:
    case afCollisionMeshShapeType::CONVEX_MESH:{
        // bullet mesh
        btTriangleMesh* bulletMesh = cMeshTObtTriangleMesh(a_collisionMesh);

        if (a_meshType == afCollisionMeshShapeType::CONCAVE_MESH){
        // create mesh collision model
        collisionShape = new btGImpactMeshShape(bulletMesh);
        }
        else{
            collisionShape = new btConvexTriangleMeshShape(bulletMesh);
        }
        break;
    }
    case afCollisionMeshShapeType::CONVEX_HULL:{
        // create collision detector for each mesh
        collisionShape = new btConvexHullShape((double*)(&a_collisionMesh->m_vertices->m_localPos[0]),
                a_collisionMesh->m_vertices->getNumElements(), sizeof(cVector3d));
        break;
    }
    case afCollisionMeshShapeType::POINT_CLOUD:{
        // NOT IMPLEMENTED YET
        break;
    }
    default:
        break;
    }
    collisionShape->setMargin(a_margin);
    return collisionShape;
}


///
/// \brief afShapeUtils::createCollisionShape
/// \param a_collisionMultiMesh
/// \param a_margin
/// \param m_inertialOffset
/// \param a_meshType
/// \return
///
btCompoundShape *afShapeUtils::createCollisionShape(cMultiMesh *a_collisionMultiMesh,
                                                    double a_margin,
                                                    afTransform m_inertialOffset,
                                                    afCollisionMeshShapeType a_meshType){
    // create the collision shape
    btCollisionShape* collisionShape;
    btCompoundShape* compoundCollisionShape = new btCompoundShape();
    btTransform inverseInertialOffsetTransform;
    inverseInertialOffsetTransform << m_inertialOffset.getInverse();

    switch (a_meshType) {
    case afCollisionMeshShapeType::CONCAVE_MESH:
    case afCollisionMeshShapeType::CONVEX_MESH:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
        {
            cMesh* mesh = (*it);
            btTriangleMesh* bulletMesh = cMeshTObtTriangleMesh(mesh);

            if (a_meshType == afCollisionMeshShapeType::CONCAVE_MESH){
                // create mesh collision model
                collisionShape = new btGImpactMeshShape(bulletMesh);
                collisionShape->setMargin(a_margin);
                ((btGImpactMeshShape*) collisionShape)->updateBound();
                compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
            }
            else{
                // create mesh collision model
                collisionShape = new btConvexTriangleMeshShape(bulletMesh);
                collisionShape->setMargin(a_margin);
                compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
            }
        }
        break;
    }
    case afCollisionMeshShapeType::CONVEX_HULL:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        int idx = 0;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
        {
            cMesh* mesh = (*it);
            if (mesh->m_vertices->getNumElements() < 3){
                cerr << "ERROR! COLLISION MESH \"" << a_collisionMultiMesh->m_name  << "\" AT IDX " << idx << " HAS LESS THAN 3 VERTICES. IGNORING! " << endl;
            }
            else{
                collisionShape = new btConvexHullShape((double*)(&mesh->m_vertices->m_localPos[0]), mesh->m_vertices->getNumElements(), sizeof(cVector3d));
                collisionShape->setMargin(a_margin);
                compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
            }
            idx++;
        }
        break;
    }
    case afCollisionMeshShapeType::POINT_CLOUD:{
        std::vector<cMesh*>::iterator it;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
        {
            cMesh* mesh = (*it);
            for (uint i = 0 ; i < mesh->m_vertices->getNumElements() ; i++){
                collisionShape = new btSphereShape(a_margin);
                btTransform lT;
                btVector3 btPos = to_btVector(mesh->m_vertices->getLocalPos(i));
                lT.setOrigin(btPos);
                compoundCollisionShape->addChildShape(inverseInertialOffsetTransform * lT, collisionShape);
            }
        }
        break;
    }
    default:
        break;
    }
    return compoundCollisionShape;
}


std::vector<afRayAttributes> afShapeUtils::createRayAttribs(cMultiMesh *a_contourMesh, double a_range){
    std::vector<afRayAttributes> raysAttribs;
    for (int n = 0 ; n < a_contourMesh->getNumMeshes() ; n++){
        cMesh* sourceMesh = a_contourMesh->getMesh(n);
        if (sourceMesh){
            int count = sourceMesh->m_triangles->getNumElements();
            afRayAttributes ray;
            for (uint i = 0 ; i < count ; i++ ){

                ray.m_range = a_range;

                uint vIdx0 = sourceMesh->m_triangles->getVertexIndex0(i);
                uint vIdx1 = sourceMesh->m_triangles->getVertexIndex1(i);
                uint vIdx2 = sourceMesh->m_triangles->getVertexIndex2(i);

                cVector3d v0 = sourceMesh->m_vertices->getLocalPos(vIdx0);
                cVector3d v1 = sourceMesh->m_vertices->getLocalPos(vIdx1);
                cVector3d v2 = sourceMesh->m_vertices->getLocalPos(vIdx2);

                cVector3d e1 = v1 - v0;
                cVector3d e2 = v2 - v1;

                cVector3d localStart = ( v0 + v1 + v2 ) / 3;

                cVector3d localDir = cCross(e1, e2);
                localDir.normalize();

                cVector3d localEnd = localStart + localDir * a_range;

                localDir.normalize();
                ray.m_rayFromLocal << localStart ;
                ray.m_direction << localDir;
                ray.m_rayToLocal << localEnd;
                raysAttribs.push_back(ray);
            }
        }
    }
    return raysAttribs;
}


///
/// \brief afMaterialUtils::createMaterialFromColor
/// \param a_color
/// \return
///
cMaterial afMaterialUtils::createFromAttribs(afColorAttributes *a_color)
{
    cMaterial mat;
    mat.m_diffuse.set(a_color->m_diffuse(0), a_color->m_diffuse(1), a_color->m_diffuse(2));
    mat.m_specular.set(a_color->m_specular(0), a_color->m_specular(1), a_color->m_specular(2));
    mat.m_ambient.set(a_color->m_ambient(0), a_color->m_ambient(1), a_color->m_ambient(2));
    mat.m_emission.set(a_color->m_emission(0), a_color->m_emission(1), a_color->m_emission(2));
    mat.setShininess(a_color->m_shininess);
    mat.setTransparencyLevel(a_color->m_alpha);

    return mat;
}


bool afVisualUtils::createFromAttribs(afVisualAttributes *attribs, cMultiMesh *mesh, string obj_name){
    if (attribs->m_geometryType == afGeometryType::MESH){
        if (mesh->loadFromFile(attribs->m_meshFilepath.c_str()) ){
            if (attribs->m_meshRemoveDuplicates == afStatusFlag::TRUE){
                mesh->removeDuplicateVertices();
            }
            else if (attribs->m_meshRemoveDuplicates == afStatusFlag::UNDEFINED){
                for (auto m : *(mesh->m_meshes)){
                    if (m->getNumVertices() > 10000){
                        double wd = 0.;
                        m->removeDuplicateVertices(wd);
                    }
                }
            }
            mesh->setUseDisplayList(true);
        }
        else{
            cerr << "WARNING! OBJECT "
                 << obj_name
                 << "'s mesh \"" << attribs->m_meshFilepath.c_str() << "\" not found\n";
            return false;
        }
    }
    else if(attribs->m_geometryType == afGeometryType::SINGLE_SHAPE ||
            attribs->m_geometryType == afGeometryType::COMPOUND_SHAPE){

        for(unsigned long sI = 0 ; sI < attribs->m_primitiveShapes.size() ; sI++){
            afPrimitiveShapeAttributes pS = attribs->m_primitiveShapes[sI];
            cMesh* tempMesh = afShapeUtils::createVisualShape(&pS);;
            mesh->m_meshes->push_back(tempMesh);
        }
    }

    mesh->m_name = obj_name;

    if (attribs->m_colorAttribs.m_useMaterial){
        cMaterial mat = afMaterialUtils::createFromAttribs(&attribs->m_colorAttribs);
        mesh->setMaterial(mat);
        // Important to set the transparency after setting the material, otherwise the alpha
        // channel ruins the Z-buffer depth testing in some way.
        mesh->setTransparencyLevel(attribs->m_colorAttribs.m_alpha);
        mesh->setShowEnabled(attribs->m_visible);
    }
    return true;
}


///
/// \brief afComm::getMinPublishFrequency
/// \return
///
int afComm::getMinPublishFrequency(){
    if (s_globalOverride){
        return s_minFreq;
    }
    else{
        return m_minPubFreq;
    }
}


///
/// \brief afComm::getMaxPublishFrequency
/// \return
///
int afComm::getMaxPublishFrequency(){
    if (s_globalOverride){
        return s_maxFreq;
    }
    else{
        return m_maxPubFreq;
    }
}


///
/// \brief afComm::setMinPublishFrequency
/// \param freq
///
void afComm::setMinPublishFrequency(int freq){
    if (freq > getMaxPublishFrequency()){
        cerr << "ERROR! MIN PUBLISHING FREQUENCY CANNOT BE GREATER THAN MAX PUBLISHING FREQUENCY. IGNORING!" << endl;
        return;
    }
    m_minPubFreq = freq;
}


///
/// \brief afComm::setMaxPublishFrequency
/// \param freq
///
void afComm::setMaxPublishFrequency(int freq){
    if (freq < getMinPublishFrequency()){
        cerr << "ERROR! MAX PUBLISHING FREQUENCY CANNOT BE LOWER THAN MIN PUBLISHING FREQUENCY. IGNORING!" << endl;
        return;
    }
    m_maxPubFreq = freq;
}


///
/// \brief afComm::overrideMaxPublishingFrequency
/// \param freq
///
void afComm::overrideMaxPublishingFrequency(int freq)
{
    if (freq < s_minFreq){
        cerr << "ERROR! MAX PUBLISHING FREQUENCY CANNOT BE LOWER THAN MIN PUBLISHING FREQUENCY. IGNORING!" << endl;
        return;
    }
    cerr << "INFO ! OVERRIDING MAX COMMUNICATION FREQUENCY TO: " << freq << endl;
    s_globalOverride = true;
    s_maxFreq = freq;
}


///
/// \brief afComm::overrideMinPublishingFrequency
/// \param freq
///
void afComm::overrideMinPublishingFrequency(int freq)
{
    if (freq > s_maxFreq){
        cerr << "ERROR! MIN PUBLISHING FREQUENCY CANNOT BE GREATER THAN MAX PUBLISHING FREQUENCY. IGNORING!" << endl;
        return;
    }
    cerr << "INFO ! OVERRIDING MIN COMMUNICATION FREQUENCY TO: " << freq << endl;
    s_globalOverride = true;
    s_minFreq = freq;
}


///
/// \brief afComm::setGlobalNamespacePrefix
/// \param a_global_namespace_prefix
///
void afComm::setGlobalNamespacePrefix(string a_global_namespace_prefix){
    s_global_namespace_prefix = a_global_namespace_prefix;
    if (!s_global_namespace_prefix.empty()){
        cerr << " INFO! FORCE PREPENDING GLOBAL NAMESPACE PREFIX \"" << s_global_namespace_prefix << "\" \n" ;
    }
}



///
/// \brief afCartesianController::afCartesianController
///
afCartesianController::afCartesianController(){
    m_dPos.setValue(0, 0, 0);
    m_dPos_cvec.set(0, 0, 0);
    m_dRot.setIdentity();
    m_dRot_cvec.identity();
    m_enabled = false;
}

bool afCartesianController::createFromAttribs(afCartesianControllerAttributes *a_attribs)
{
    P_lin = a_attribs->P_lin;
    I_lin = a_attribs->I_lin;
    D_lin = a_attribs->D_lin;
    P_ang = a_attribs->P_ang;
    I_ang = a_attribs->I_ang;
    D_ang = a_attribs->D_ang;

    m_positionOutputType = a_attribs->m_positionOutputType;
    m_orientationOutputType = a_attribs->m_orientationOutputType;
    setEnabled(true);
    return true;
}


////
/// \brief afCartesianController::setLinearGains
/// \param a_P
/// \param a_I
/// \param a_D
///
void afCartesianController::setLinearGains(double a_P, double a_I, double a_D){
    P_lin = a_P;
    I_lin = a_I;
    D_lin = a_D;
    // If someone sets the gains, enable the controller
    setEnabled(true);
}


///
/// \brief afCartesianController::setAngularGains
/// \param a_P
/// \param a_I
/// \param a_D
///
void afCartesianController::setAngularGains(double a_P, double a_I, double a_D){
    P_ang = a_P;
    I_ang = a_I;
    D_ang = a_D;

    setEnabled(true);
}

void afCartesianController::setOutputType(afControlType type)
{
    m_positionOutputType = type;
    m_orientationOutputType = type;
}

template <>
///
/// \brief afCartesianController::computeOutput ts is the time_scale and computed as a fraction of fixed time-step (dt-fixed) and dynamic time-step (dt)
/// \param process_val
/// \param set_point
/// \param dt
/// \param ts
/// \return
///
btVector3 afCartesianController::computeOutput<btVector3, btVector3>(const btVector3 &process_val, const btVector3 &set_point, const double &dt, const double &ts){
    btVector3 output(0, 0, 0); // Initialize the output to zero
    if (isEnabled()){
        btVector3 dPos_prev, ddPos;
        dPos_prev = m_dPos;
        m_dPos = set_point - process_val;
        ddPos = (m_dPos - dPos_prev) / dt;

        output = P_lin * (m_dPos) * ts + D_lin * (ddPos);
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
    return output;
}


template<>
///
/// \brief afCartesianController::computeOutput
/// \param process_val
/// \param set_point
/// \param dt
/// \param ts
/// \return
///
btVector3 afCartesianController::computeOutput<btVector3, btMatrix3x3>(const btMatrix3x3 &process_val, const btMatrix3x3 &set_point, const double &dt, const double &ts){
    btVector3 output(0, 0, 0);

    if (isEnabled()){
        btVector3 error_cur, error_prev;
        btMatrix3x3 dRot_prev;
        btQuaternion dRotQuat, dRotQuat_prev;
        dRot_prev = m_dRot;
        dRot_prev.getRotation(dRotQuat_prev);
        error_prev = dRotQuat_prev.getAxis() * dRotQuat_prev.getAngle();

        m_dRot = process_val.transpose() * set_point;
        m_dRot.getRotation(dRotQuat);
        error_cur = dRotQuat.getAxis() * dRotQuat.getAngle();

        output = (P_ang * error_cur * ts) + (D_ang * (error_cur - error_prev) / dt);

        // Important to transform the torque in the world frame as its represented
        // in the body frame from the above computation
        output = process_val * output;
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }

    return output;
}

template<>
///
/// \brief afCartesianController::computeOutput_cvec
/// \param process_val
/// \param set_point
/// \param dt
/// \return
///
cVector3d afCartesianController::computeOutput<cVector3d, cVector3d>(const cVector3d &process_val, const cVector3d &set_point, const double &dt, const double &ts){
    cVector3d output(0, 0, 0);
    if (isEnabled()){
        cVector3d dPos_prev, ddPos;
        dPos_prev = m_dPos_cvec;
        m_dPos_cvec = set_point - process_val;
        ddPos = (m_dPos_cvec - dPos_prev) / dt;

        output = P_lin * (m_dPos_cvec) * ts + D_lin * (ddPos);
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
    return output;
}

template<>
///
/// \brief afCartesianController::computeOutput_cvec
/// \param process_val
/// \param set_point
/// \param dt
/// \return
///
cVector3d afCartesianController::computeOutput<cVector3d, cMatrix3d>(const cMatrix3d &process_val, const cMatrix3d &set_point, const double &dt, const double &ts){
    cVector3d output(0, 0, 0);
    if (isEnabled()){
        cVector3d error_cur, error_prev;
        cMatrix3d dRot_prev;
        cVector3d e_axis, e_axis_prev;
        double e_angle, e_angle_prev;
        dRot_prev = m_dRot_cvec;
        dRot_prev.toAxisAngle(e_axis_prev, e_angle_prev);
        error_prev = e_axis_prev * e_angle_prev;

        m_dRot_cvec = cTranspose(process_val) * set_point;
        m_dRot_cvec.toAxisAngle(e_axis, e_angle);
        error_cur = e_axis * e_angle;

        output = (P_ang * error_cur * ts) + (D_ang * (error_cur - error_prev) / dt);

        // Important to transform the torque in the world frame as its represented
        // in the body frame from the above computation
        output = process_val * output;
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
    return output;
}

template<>
///
/// \brief afCartesianController::computeOutputTransform
/// \param process_val
/// \param set_point
/// \param current_time
/// \return
///
btTransform afCartesianController::computeOutput<btTransform, btTransform>(const btTransform &process_val, const btTransform &set_point, const double &dt, const double &tsf){
    // Not implemented yet
    return btTransform();
}


///
/// \brief afIdentification::afIdentification
/// \param a_type
///
afIdentification::afIdentification(afType a_type): m_type(a_type)
{
    setGlobalRemapIdx("");
}


///
/// \brief afIdentification::getTypeAsStr
/// \return
///
string afIdentification::getTypeAsStr(){
    switch (m_type) {
    case afType::ACTUATOR:
        return "ACTUATOR";
        break;
    case afType::CAMERA:
        return "CAMERA";
        break;
    case afType::GHOST_OBJECT:
        return "GHOST_OBJECT";
        break;
    case afType::INPUT_DEVICE:
        return "INPUT_DEVICE";
        break;
    case afType::INVALID:
        return "INVALID";
        break;
    case afType::JOINT:
        return "JOINT";
        break;
    case afType::LIGHT:
        return "LIGHT";
        break;
    case afType::MODEL:
        return "MODEL";
        break;
    case afType::OBJECT:
        return "OBJECT";
        break;
    case afType::POINT_CLOUD:
        return "POINT_CLOUD";
        break;
    case afType::RIGID_BODY:
        return "RIGID_BODY";
        break;
    case afType::SENSOR:
        return "SENSOR";
        break;
    case afType::SOFT_BODY:
        return "SOFT_BODY";
        break;
    case afType::VEHICLE:
        return "VEHICLE";
        break;
    case afType::WORLD:
        return "WORLD";
        break;
    default:
        cerr << "ERROR! SHOULD NOT HAPPEN" << endl;
        return "";
        break;
    }
}

string afIdentification::getName(){return m_name;}

string afIdentification::getNamespace(){return afComm::getGlobalNamespacePrefix() + m_namespace;}

string afIdentification::getQualifiedName(){return getNamespace() + m_name;}

void afIdentification::setName(string a_name){m_name = a_name;}

void afIdentification::setNamespace(string a_namespace){m_namespace = a_namespace; }

string afIdentification::getQualifiedIdentifier(){return getNamespace() + m_identifier;}


///
/// \brief afObject::afObject
/// \param a_afWorld
///
afBaseObject::afBaseObject(afType a_type, afWorldPtr a_afWorld, afModelPtr a_afModel): afIdentification(a_type){
    m_afWorld = a_afWorld;
    m_modelPtr = a_afModel;
    m_parentObject = nullptr;
}



///
/// \brief afObject::~afObject
///
afBaseObject::~afBaseObject(){
    m_pluginManager.close();
    for (int i=0; i<m_childrenSceneObjects.size(); i++){
        delete m_childrenSceneObjects[i];
    }
}


///
/// \brief afBaseObject::createFromAttribs
/// \param a_attribs
/// \return
///
bool afBaseObject::createFromAttribs(afBaseObjectAttributes* a_attribs){
    return false;
}


///
/// \brief afBaseObject::
/// \param pluginAttribs
/// \return
///
bool afBaseObject::loadPlugins(afBaseObjectPtr objPtr, afBaseObjectAttribsPtr attribs, vector<afPluginAttributes> *pluginAttribs){
    for (int i = 0 ; i < pluginAttribs->size(); i++){
        m_pluginManager.loadPlugin(objPtr, attribs, (*pluginAttribs)[i].m_filename, (*pluginAttribs)[i].m_name, (*pluginAttribs)[i].m_path.c_str());
    }

    return true;
}

void afBaseObject::update(double dt){
}

void afBaseObject::reset()
{
    setLocalTransform(getInitialTransform());
}


///
/// \brief afBaseObject::getLocalPos
/// \return
///
cVector3d afBaseObject::getLocalPos()
{
    return m_localTransform.getLocalPos();
}


///
/// \brief afBaseObject::getLocalRot
/// \return
///
cMatrix3d afBaseObject::getLocalRot()
{
    return m_localTransform.getLocalRot();
}


///
/// \brief afBaseObject::getLocalTransform
/// \return
///
cTransform afBaseObject::getLocalTransform()
{
    return m_localTransform;
}

cTransform afBaseObject::getGlobalTransform()
{
    return m_globalTransform;
}

double afBaseObject::getWallTime(){
    return m_afWorld->getWallTime();
}

double afBaseObject::getSimulationTime(){
    return m_afWorld->getSimulationTime();
}


///
/// \brief afBaseObject::setLocalPos
/// \param pos
///
void afBaseObject::setLocalPos(const cVector3d &pos)
{
    m_localTransform.setLocalPos(pos);
}

void afBaseObject::setLocalPos(const afVector3d &pos)
{
    m_localTransform.setLocalPos(cVector3d(pos(0), pos(1), pos(2)));
}

///
/// \brief afBaseObject::setLocalPos
/// \param px
/// \param py
/// \param pz
///
void afBaseObject::setLocalPos(double px, double py, double pz)
{
    cVector3d pos(px, py, pz);
    setLocalPos(pos);
}


///
/// \brief afBaseObject::setLocalRot
/// \param mat
///
void afBaseObject::setLocalRot(const cMatrix3d &a_mat)
{
    cMatrix3d mat = a_mat;
    mat.orthogonalize();
    m_localTransform.setLocalRot(mat);
}

void afBaseObject::setLocalRot(const afMatrix3d &mat)
{
    cMatrix3d rot = to_cMatrix3d(mat);
    m_localTransform.setLocalRot(rot);
}


///
/// \brief afBaseObject::setLocalRot
/// \param mat
///
void afBaseObject::setLocalRot(const cQuaternion &quat)
{
    cMatrix3d mat;
    quat.toRotMat(mat);
    setLocalRot(mat);
}


///
/// \brief afBaseObject::setLocalRot
/// \param qx
/// \param qy
/// \param qz
/// \param qw
///
void afBaseObject::setLocalRot(double qx, double qy, double qz, double qw)
{
    cQuaternion quat(qw, qx, qy, qz);
    setLocalRot(quat);
}


///
/// \brief afBaseObject::setLocalTransform
/// \param trans
///
void afBaseObject::setLocalTransform(const cTransform &trans)
{
    m_localTransform = trans;
}

void afBaseObject::clearParentObject()
{
    m_parentObject = nullptr;
}

bool afBaseObject::addChildObject(afBaseObjectPtr a_afObject)
{
    if (a_afObject == this){
        return false;
    }
    // Remove any set parent of the child object first?
    //    a_afObject->clearParentObject();
    a_afObject->setParentObject(this);
    m_childrenObjects.push_back(a_afObject);
    return true;
}

///
/// \brief afBaseObject::setParentObject
/// \param a_afObject
///
bool afBaseObject::setParentObject(afBaseObject *a_afObject)
{
    m_parentObject = a_afObject;
    return true;
}


///
/// \brief afBaseObject::toggleFrameVisibility
///
void afBaseObject::toggleFrameVisibility(){
    std::vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin(); it != m_childrenSceneObjects.end() ; ++it){
        (*it)->getChaiObject()->setShowFrame(!(*it)->getChaiObject()->getShowFrame());
    }
    //    if (m_visualMesh){
    //        m_visualMesh->setShowFrame(!m_visualMesh->getShowFrame());
    //    }
}


///
/// \brief afBaseObject::checkSceneObjectAlreadyAdded
/// \param a_object
/// \return
///
bool afBaseObject::isSceneObjectAlreadyAdded(afSceneObject *a_object){
    vector<afSceneObject*>::iterator it;
    for(it = m_childrenSceneObjects.begin() ; it != m_childrenSceneObjects.end() ; ++it){
        if ((*it) == a_object){
            return true;
        }
    }

    return false;
}


///
/// \brief afBaseObject::setScale
/// \param a_scale
///
void afBaseObject::setScale(double a_scale){
    m_scale = a_scale;
    scaleSceneObjects(m_scale);
}


///
/// \brief afBaseObject::addChild
/// \param a_visualMesh
///
bool afBaseObject::addChildSceneObject(cGenericObject *a_object, const cTransform &a_trans){

    if (a_object == nullptr){
        return false;
    }

    afSceneObject* sceneObject = new afSceneObject;
    sceneObject->setChaiObject(a_object);
    sceneObject->setLocalOffset(a_trans);
    return addChildSceneObject(sceneObject);
}

bool afBaseObject::addChildSceneObject(afSceneObject *a_object)
{
    // sanity check
    if (a_object == nullptr){
        return false;
    }

    // Check if this object is already appended as a child, If so, return true;
    if (isSceneObjectAlreadyAdded(a_object)){
        return true;
    }

    // the object does not have any parent yet, so we can add it as a child
    // to current object.
    a_object->getChaiObject()->m_name = getQualifiedName();
    m_childrenSceneObjects.push_back(a_object);
    return true;
}

void afBaseObject::scaleSceneObjects(double a_scale){
    std::vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin(); it != m_childrenSceneObjects.end(); ++it){
        (*it)->getChaiObject()->scale(a_scale);

    }
}

bool afBaseObject::removeChildSceneObject(cGenericObject *a_object, bool removeFromGraph)
{
    // sanity check
    if (a_object == nullptr) { return (false); }

    vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin(); it != m_childrenSceneObjects.end(); ++it)
    {
        if ((*it)->getChaiObject() == a_object)
        {
            if (removeFromGraph){
                (*it)->getChaiObject()->removeFromGraph();
            }
            // remove this object from my list of children
            m_childrenSceneObjects.erase(it);
            // return success
            return true;
        }
    }

    // operation failed
    return false;
}


///
/// \brief afBaseObject::removeChild
/// \param a_cObject
///
bool afBaseObject::removeChildSceneObject(afSceneObject *a_object, bool removeFromGraph){
    // sanity check
    if (a_object == nullptr) { return (false); }

    vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin(); it != m_childrenSceneObjects.end(); ++it)
    {
        if ((*it) == a_object)
        {
            if (removeFromGraph){
                (*it)->getChaiObject()->removeFromGraph();
            }
            // remove this object from my list of children
            m_childrenSceneObjects.erase(it);
            // return success
            return true;
        }
    }

    // operation failed
    return false;
}


///
/// \brief afBaseObject::removeAllChildSceneObjects
/// \param removeFromGraph
///
void afBaseObject::removeAllChildSceneObjects(bool removeFromGraph){
    std::vector<afSceneObject*>::iterator it;

    for (it = m_childrenSceneObjects.begin() ; it < m_childrenSceneObjects.end() ; ++it){
        removeChildSceneObject((*it), removeFromGraph);
    }
}


///
/// \brief afBaseObject::loadCommunicationPlugin
/// \return
///
bool afBaseObject::loadCommunicationPlugin(afBaseObjectPtr a_objPtr, afBaseObjectAttribsPtr a_attribs)
{
    bool result = false;
    if (isPassive() == false){
        afObjectCommunicationPlugin* commPlugin = new afObjectCommunicationPlugin();
        result = m_pluginManager.loadPlugin(a_objPtr, a_attribs, commPlugin);
    }

    return result;
}


///
/// \brief afBaseObject::updateSceneObjects
///
void afBaseObject::updateSceneObjects(){
    // Assuming that the global pose was computed prior to this call.
    vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin() ; it != m_childrenSceneObjects.end() ; ++it){
        cTransform globalTrans = m_globalTransform * (*it)->getOffsetTransform();
        (*it)->getChaiObject()->setLocalTransform(globalTrans);
    }
}


///
/// \brief afBaseObject::pluginsGraphicsUpdate
///
void afBaseObject::pluginsGraphicsUpdate()
{
    m_pluginManager.graphicsUpdate();
}


///
/// \brief afBaseObject::pluginsPhysicsUpdate
/// \param dt
///
void afBaseObject::pluginsPhysicsUpdate(double dt){
    m_pluginManager.physicsUpdate(dt);
}

void afBaseObject::pluginsReset()
{
    m_pluginManager.reset();
}


///
/// \brief afBaseObject::updateGlobalPose
/// \param a_forceUpdate
/// \param a_parentTransform
///
void afBaseObject::updateGlobalPose(bool a_forceUpdate, cTransform a_parentTransform){
    if ( (getParentObject() != nullptr) && (a_forceUpdate == false) ){
        // Don't update the pose as this object's parent is
        // responsible for it.
        return;
    }

    m_globalTransform = a_parentTransform * m_localTransform;

    vector<afBaseObjectPtr>::const_iterator it;

    for (it = m_childrenObjects.begin() ; it != m_childrenObjects.end() ; ++it){
        (*it)->updateGlobalPose(true, m_globalTransform);
    }
}


///
/// \brief afBaseObject::calculateFrameSize
///
void afBaseObject::calculateFrameSize()
{
    std::vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin(); it != m_childrenSceneObjects.end() ; ++it){
        // Set the size of the frame.
        cVector3d bounds = (*it)->getChaiObject()->getBoundaryMax();
        double frame_size;
        if (bounds.length() > 0.001){
            double max_axis = cMax3(bounds.x(), bounds.y(), bounds.z());
            frame_size = max_axis * 1.2;
        }
        else{
            frame_size = 0.5;
        }
        (*it)->getChaiObject()->setFrameSize(frame_size);
    }
}


afMeshObject::afMeshObject(afWorldPtr a_afWorld, afModelPtr a_afModel){
    m_world = a_afWorld;
    m_model = a_afModel;
    m_visualMesh = nullptr;
}

///
cMultiMesh *afMeshObject::getVisualObject(){
    return m_visualMesh;
}

///
/// \brief afBaseObject::isShaderProgramDefined
/// \return
///
bool afMeshObject::isShaderProgramDefined(){
    if (m_visualMesh == nullptr){
        return false;
    }

    return m_visualMesh->getShaderProgram().get() == nullptr ? 0 : 1;
}


///
/// \brief afBaseObject::isShaderProgramDefined
/// \param a_mesh
/// \return
///
bool afMeshObject::isShaderProgramDefined(cMesh *a_mesh)
{
    if (a_mesh == nullptr){
        return false;
    }

    return a_mesh->getShaderProgram().get() == nullptr ? 0 : 1;
}


///
/// \brief afBaseObject::loadShaderProgram
///
void afMeshObject::loadShaderProgram()
{
    bool valid = false;
    if (m_shaderAttribs.m_shaderDefined){
        cShaderProgramPtr shaderProgram;
        shaderProgram = afShaderUtils::createFromAttribs(&m_shaderAttribs, m_visualMesh->m_name, "OBJECT_SHADERS");
        m_visualMesh->setShaderProgram(shaderProgram);
        valid = true;
    }
    else if (m_model->m_shaderAttribs.m_shaderDefined){
        if (m_model->m_shaderProgram.get()){
            m_shaderAttribs.m_shaderDefined = true;
            m_visualMesh->setShaderProgram(m_model->m_shaderProgram);
            valid = true;
        }
    }
    else if (m_world->m_shaderAttribs.m_shaderDefined){
        if (m_world->m_shaderProgram.get()){
            m_shaderAttribs.m_shaderDefined = true;
            m_visualMesh->setShaderProgram(m_world->m_shaderProgram);
            valid = true;
        }
    }

    if (valid){
        cShaderProgramPtr shaderProgram = m_visualMesh->getShaderProgram();
        // Set the ID for shadow and normal maps.
        shaderProgram->setUniformi("shadowMap", C_TU_SHADOWMAP);
        if (isNormalTextureDefined()){
            enableShaderNormalMapping(true);
        }
        else{
            enableShaderNormalMapping(false);
        }
    }
}

///
/// \brief afBaseObject::isNormalMapDefined
/// \return
///
bool afMeshObject::isNormalMapDefined()
{
    if (m_visualMesh->m_normalMap.get() != nullptr){
        return true;
    }
    else{
        return false;
    }
}

///
/// \brief afBaseObject::isNormalMapDefined
/// \param a_mesh
/// \return
///
bool afMeshObject::isNormalMapDefined(cMesh *a_mesh)
{
    if (a_mesh->m_normalMap.get() != nullptr){
        return true;
    }
    else{
        return false;
    }
}


///
/// \brief afBaseObject::isNormalTextureDefined
/// \return
///
bool afMeshObject::isNormalTextureDefined()
{
    if (isNormalMapDefined()){
        if (m_visualMesh->m_normalMap->m_image.get() != nullptr){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}


///
/// \brief afBaseObject::isNormalTextureDefined
/// \param a_mesh
/// \return
///
bool afMeshObject::isNormalTextureDefined(cMesh *a_mesh)
{
    if (isNormalMapDefined(a_mesh)){
        if (a_mesh->m_normalMap->m_image.get() != nullptr){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

///
/// \brief afBaseObject::enableShaderNormalMapping
/// \param enable
///
void afMeshObject::enableShaderNormalMapping(bool enable)
{
    if (isShaderProgramDefined() == false){
        return;
    }

    m_visualMesh->getShaderProgram()->setUniformi("normalMap", C_TU_NORMALMAP);
    m_visualMesh->getShaderProgram()->setUniformi("vEnableNormalMapping", enable);

    for (int i = 0 ; i < m_visualMesh->getNumMeshes() ; i++){
        enableShaderNormalMapping(enable, m_visualMesh->getMesh(i));
    }
}

///
/// \brief afBaseObject::enableShaderNormalMapping
/// \param enable
/// \param a_mesh
///
void afMeshObject::enableShaderNormalMapping(bool enable, cMesh *a_mesh)
{
    if (isShaderProgramDefined(a_mesh) == false){
        return;
    }

    if (isNormalTextureDefined(a_mesh)){
        a_mesh->getShaderProgram()->setUniformi("normalMap", C_TU_NORMALMAP);
        a_mesh->getShaderProgram()->setUniformi("vEnableNormalMapping", enable);
    }
}

cShaderProgramPtr afMeshObject::getShaderProgram()
{
    if (m_visualMesh){
        return m_visualMesh->getShaderProgram();
    }
    else{
        return nullptr;
    }
}

void afMeshObject::setShaderProgram(cShaderProgramPtr a_program)
{
    if (m_visualMesh){
        m_visualMesh->setShaderProgram(a_program);
    }
    else{
        cerr << "ERROR! MESH FOR THIS VISUAL OBJECT HAS NOT BEEN INITIALIZED YET. CAN'T SET SHADER PROGRAM" << endl;
    }
}

void afMeshObject::backupShaderProgram()
{
    if (m_shaderProgramBackup == nullptr){
        m_shaderProgramBackup = m_visualMesh->getShaderProgram();
    }
    else{
        cerr << "WARNING! A SHADER PROGRAM FOR " << m_visualMesh->m_name <<
                " IS ALREADY BACKED UP. RESTORE THE SHADER PROGRAM FIRST" << endl;
    }
}

void afMeshObject::restoreShaderProgram()
{
    m_visualMesh->setShaderProgram(m_shaderProgramBackup);
    m_shaderProgramBackup = nullptr;
}

///
/// \brief afBaseObject::resolveParent
/// \param a_parentName
/// \param suppress_warning
/// \return
///
bool afBaseObject::resolveParent(string a_parentName, bool suppress_warning){
    // If the parent name is empty, return true as nothing to do
    if(a_parentName.empty()){
        return true;
    }
    else if ( (m_parentName.compare(a_parentName) == 0) && m_parentObject != nullptr ){
        // The name of the parent is the same, ignore.
        return true;
    }
    else{
        // Should generalize this to not be just a rigid body that we may parent to.
        afBaseObjectPtr pBody = m_afWorld->getBaseObject(m_parentName, suppress_warning);
//        afBaseObjectPtr pBody = m_afWorld->getRigidBody(m_parentName, suppress_warning);
        if (pBody){
            pBody->addChildObject(this);
            return true;
        }
        else{
            return false;
        }
    }
}


///
/// \brief afActuator::afActuator
/// \param a_afWorld
///
afActuator::afActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::ACTUATOR, a_afWorld, a_modelPtr){
}


///
/// \brief afConstraintActuator::afConstraintActuator
/// \param a_afWorld
///
afConstraintActuator::afConstraintActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afActuator(a_afWorld, a_modelPtr){
    m_actuatorType = afActuatorType::CONSTRAINT;
}

afConstraintActuator::~afConstraintActuator(){
    if(m_constraint){
        delete m_constraint;
    }
}

bool afConstraintActuator::createFromAttribs(afConstraintActuatorAttributes *a_attribs)
{
    storeAttributes(a_attribs);

    bool result = true;

    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_initialTransform = to_cTransform(a_attribs->m_kinematicAttribs.m_location);
    setLocalTransform(m_initialTransform);

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    setVisibleFlag(a_attribs->m_visible);
    m_visibleSize = a_attribs->m_visibleSize;

    // First search in the local space.
    m_parentBody = m_modelPtr->getRigidBody(m_parentName, true);

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getActuatorMap());
    setGlobalRemapIdx(remap_idx);

    if(!m_parentBody){
        m_parentBody = m_afWorld->getRigidBody(m_parentName + getGlobalRemapIdx());

        if (m_parentBody == nullptr){
            cerr << "ERROR! ACTUATOR'S "<< m_parentName + remap_idx << " NOT FOUND, IGNORING ACTUATOR\n";
            return 0;
        }
    }

    m_parentBody->addActuator(this);

    m_parentBody->addChildObject(this);

    m_maxImpulse = a_attribs->m_maxImpulse;
    m_tau = a_attribs->m_tau;

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    return result;
}


///
/// \brief afActuator::enableVisualization
///
void afConstraintActuator::enableVisualization()
{
    if (m_visualizationEnabled == false){
        cMesh* mesh = new cMesh();
        cCreateSphere(mesh, m_visibleSize);
        mesh->m_material->setYellowGold();
        mesh->setShowEnabled(false);
        mesh->setUseDisplayList(true);
        mesh->markForUpdate(false);
        m_actuatorVisual = mesh;
        addChildSceneObject(m_actuatorVisual, cTransform());
        m_afWorld->addSceneObjectToWorld(mesh);
    }
    m_visualizationEnabled = true;
}

void afConstraintActuator::visualize(bool show)
{
    if (m_visualizationEnabled == false){
        enableVisualization();
    }
    m_actuatorVisual->setShowEnabled(show);

    if (m_active){
        m_actuatorVisual->m_material->setGreenLime();
    }
    else{
        m_actuatorVisual->m_material->setYellowGold();
    }
}


void afConstraintActuator::actuate(string a_rigid_body_name){

    afRigidBodyPtr body = m_afWorld->getRigidBody(a_rigid_body_name);
    actuate(body);

}

void afConstraintActuator::actuate(afRigidBodyPtr a_rigidBody){
    if (a_rigidBody){
        // Since this method does not require an explicit offset, find the relative location
        // of the body and then use it as its offset for the constraint
        btTransform T_aINp = to_btTransform(getLocalTransform());
        btTransform T_pINw = m_parentBody->m_bulletRigidBody->getCenterOfMassTransform();
        btTransform T_wINc = a_rigidBody->m_bulletRigidBody->getCenterOfMassTransform().inverse();
        btTransform T_aINw = T_pINw * T_aINp;

        btTransform T_aINc = T_wINc * T_aINw;

        cVector3d P_aINc;
        P_aINc << T_aINc.getOrigin();

        actuate(a_rigidBody, T_aINc);
    }

}

void afConstraintActuator::actuate(string a_rigid_body_name, btTransform a_bodyOffset){
    afRigidBodyPtr body = m_afWorld->getRigidBody(a_rigid_body_name);
    actuate(body, a_bodyOffset);
}

void afConstraintActuator::actuate(afRigidBodyPtr a_rigidBody, btTransform a_bodyOffset){
    // Check if a constraint is already active
    if (m_constraint){
        // Check if the new requested actuation is the same as what is already
        // actuated. In this case simply ignore the request

        if (a_rigidBody == m_childRigidBody){
            // We already have the same constraint. We can ignore the new request

            //cerr << "INFO! ACTUATOR \"" << m_name << "\" IS ACTIVATED WITH THE SAME BODY AND OFFSET. THEREBY IGNORING REQUEST \n";
            return;
        }
        else{
            // Deactuate the constraint first
            deactuate();
        }
    }

    if (a_rigidBody){
        btTransform tranA = to_btTransform(getLocalTransform());
        btTransform tranB = a_bodyOffset;
        m_childRigidBody = a_rigidBody;
        m_constraint = new btFixedConstraint(*m_parentBody->m_bulletRigidBody, *m_childRigidBody->m_bulletRigidBody, tranA, tranB);
        m_afWorld->m_bulletWorld->addConstraint(m_constraint);
        m_active = true;
        return;
    }
    else{
        // We can warn that the requested body is invalid
    }
}

void afConstraintActuator::actuate(string a_softbody_name, btSoftBody::Face* face){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, btSoftBody::Face* face){
    vector<btSoftBody::Node*> nodes;
    for (int nIdx = 0; nIdx < 3 ; nIdx++){
        btSoftBody::Node* _node = face->m_n[nIdx];
        nodes.push_back(_node);
    }
    actuate(a_softBody, nodes);
}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, vector<btSoftBody::Node *> nodes)
{
    for (int i = 0 ; i < nodes.size() ; i++){
        btSoftBody::Node* node = nodes[i];
        bool node_exists = false;
        for (int j = 0 ; j < m_softBodyNodes.size() ; j++){
            if (node == m_softBodyNodes[j]){
                node_exists = true;
                break;
            }
        }

        if (node_exists){
            // Node is already anchored, so skip
            break;
        }

        btVector3 localPivot = m_parentBody->m_bulletRigidBody->getCenterOfMassTransform().inverse() * node->m_x;
        btSoftBody::Anchor anchor;
        node->m_battach = 1;
        anchor.m_body = m_parentBody->m_bulletRigidBody;
        anchor.m_node = node;
        anchor.m_influence = 1;
        anchor.m_local = localPivot;
        a_softBody->m_bulletSoftBody->m_anchors.push_back(anchor);
        m_softBodyNodes.push_back(node);
    }
    m_childSoftBody = a_softBody;
    m_active = true;
}

void afConstraintActuator::actuate(string a_softbody_name, btSoftBody::Face* face, btTransform a_bodyOffset){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, btSoftBody::Face* face, btTransform a_bodyOffset){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, vector<btSoftBody::Node *> nodes, btTransform a_bodyOffset)
{

}

void afConstraintActuator::actuate(afSensorPtr sensorPtr)
{
    if (m_active){
        // Already actuated, nothing to do, return.
        return;
    }
    if (sensorPtr->m_sensorType == afSensorType::RAYTRACER){
        afProximitySensor* proximitySensorPtr = (afProximitySensor*) sensorPtr;
        for (int i = 0 ; i < proximitySensorPtr->getCount() ; i++){
            if (proximitySensorPtr->isTriggered(i)){
                if (proximitySensorPtr->getSensedBodyType(i) == afBodyType::RIGID_BODY){
                    afRigidBodyPtr afBody = proximitySensorPtr->getSensedRigidBody(i);
                    actuate(afBody);
                }

                if (proximitySensorPtr->getSensedBodyType(i) == afBodyType::SOFT_BODY){
                    // Here we implement the softBody grasp logic. We want to move the
                    // soft body as we move the simulated end effector

                    // If we get a sensedSoftBody, we should check if it has a detected face. If a face
                    // is found, we can anchor all the connecting nodes.
                    if (proximitySensorPtr->getSensedSoftBodyFace(i)){
                        afSoftBodyPtr afSoftBody = proximitySensorPtr->getSensedSoftBody(i);
                        btSoftBody::Face* sensedFace = proximitySensorPtr->getSensedSoftBodyFace(i);
                        actuate(afSoftBody, sensedFace);
                    }
                    // Otherwise we shall directly anchor to nodes. This case
                    // arises for ropes, suturing thread etc
                    else{
                        afSoftBodyPtr afSoftBody = proximitySensorPtr->getSensedSoftBody(i);
                        vector<btSoftBody::Node*> nodes;
                        nodes.push_back(proximitySensorPtr->getSensedSoftBodyNode(i));
                        actuate(afSoftBody, nodes);
                    }
                }
            }
        }
    }
}

void afConstraintActuator::deactuate(){
    if (m_constraint){
        m_afWorld->m_bulletWorld->removeConstraint(m_constraint);
        delete m_constraint;

        m_constraint = nullptr;
        m_childRigidBody = nullptr;
    }

    if (m_childSoftBody){
        for (int nIdx = 0 ; nIdx < m_softBodyNodes.size()  ; nIdx++){
            btSoftBody::Node* nodePtr = m_softBodyNodes[nIdx];
            btSoftBody* softBody = m_childSoftBody->m_bulletSoftBody;
            btRigidBody* rigidBody = m_parentBody->m_bulletRigidBody;
            for (int aIdx = 0 ; aIdx < softBody->m_anchors.size() ; aIdx++){
                if (softBody->m_anchors[aIdx].m_body == rigidBody){
                    btSoftBody::Anchor* anchor = &softBody->m_anchors[aIdx];
                    if (anchor->m_node == nodePtr){
                        softBody->m_anchors.removeAtIndex(aIdx);
                        break;
                    }
                }
            }
        }
        m_childSoftBody = nullptr;
        m_softBodyFaceIdx = -1;
        m_softBodyNodes.clear();
    }
    m_active = false;
}


///
/// \brief afConstraintActuator::updatePositionFromDynamics
///
void afConstraintActuator::update(double dt){
    visualize(getVisibleFlag());
}


///
/// \brief afInertialObject::afInertialObject
/// \param a_afWorld
///
afInertialObject::afInertialObject(afType a_type, afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(a_type, a_afWorld, a_modelPtr), afMeshObject(a_afWorld, a_modelPtr)
{
    m_T_iINb.setIdentity();
    m_T_bINi.setIdentity();

    m_bulletRigidBody = nullptr;
    m_bulletSoftBody = nullptr;
    m_bulletCollisionShape = nullptr;
    m_bulletMotionState = nullptr;
    m_overrideGravity = false;
}


///
/// \brief afInertialObject::~afInertialObject
///
afInertialObject::~afInertialObject()
{
    if (m_bulletRigidBody){
        delete m_bulletRigidBody;
    }
    if (m_bulletSoftBody){
        delete m_bulletSoftBody;
    }
    if (m_bulletCollisionShape){
        if (m_bulletCollisionShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE){
            btCompoundShape* compoundShape = static_cast<btCompoundShape*>(m_bulletCollisionShape);
            for (int i = 0 ; i < compoundShape->getNumChildShapes(); i++){
                if (compoundShape->getChildShape(i)){
                    if (compoundShape->getChildShape(i)->getShapeType() == GIMPACT_SHAPE_PROXYTYPE){
                        btGImpactMeshShape* gimShape = static_cast<btGImpactMeshShape*>(compoundShape->getChildShape(i));
                        delete gimShape->getMeshInterface();
                    }
                    delete compoundShape->getChildShape(i);
                }
            }
        }
        delete m_bulletCollisionShape;
    }
    if (m_bulletMotionState){
        delete m_bulletMotionState;
    }
}


///
/// \brief afInertialObject::estimateInertia
///
void afInertialObject::estimateInertia()
{
    if (m_bulletCollisionShape)
    {
        // compute inertia
        m_bulletCollisionShape->calculateLocalInertia(m_mass, m_inertia);

    }
}

void afInertialObject::setGravity(const cVector3d &a_gravity)
{
    if(m_bulletRigidBody){
        m_bulletRigidBody->setGravity(to_btVector(a_gravity));
        cerr << "INFO! SETTING " << m_name << "'s GRAVITY TO: " << a_gravity.str() << endl;
    }
}

btTransform afInertialObject::getCOMTransform()
{
    // Inertial Transform
    btTransform T_iINw;
    m_bulletRigidBody->getMotionState()->getWorldTransform(T_iINw);
    return (T_iINw * getInverseInertialOffsetTransform());
}


///
/// \brief afInertialObject::getSurfaceProperties
/// \return
///
afSurfaceAttributes afInertialObject::getSurfaceProperties()
{
    afSurfaceAttributes props;
    props.m_linearDamping = m_bulletRigidBody->getLinearDamping();
    props.m_angularDamping = m_bulletRigidBody->getAngularDamping();
    props.m_staticFriction = m_bulletRigidBody->getFriction();
    props.m_rollingFriction = m_bulletRigidBody->getRollingFriction();
    props.m_restitution = m_bulletRigidBody->getRestitution();

    return props;
}

///
/// \brief afInertialObject::setInertia
/// \param ix
/// \param iy
/// \param iz
///
void afInertialObject::setInertia(afVector3d& a_inertia)
{
    m_inertia << a_inertia;
}

void afInertialObject::setInertialOffsetTransform(btTransform &a_trans)
{
    m_T_iINb = a_trans;
    // Also compute the inverse here.
    m_T_bINi = m_T_iINb.inverse();
}

void afInertialObject::setSurfaceProperties(const afSurfaceAttributes &a_props)
{
    m_bulletRigidBody->setFriction(a_props.m_staticFriction);
    m_bulletRigidBody->setRollingFriction(a_props.m_rollingFriction);
    m_bulletRigidBody->setDamping(a_props.m_linearDamping, a_props.m_angularDamping);
    m_bulletRigidBody->setRestitution(a_props.m_restitution);
}


///
/// \brief afInertialObject::computeInertialOffset
/// \param mesh
/// \return
///
btVector3 afInertialObject::computeInertialOffset(cMesh* mesh){
    cVector3d intertialOffset(0, 0, 0);
    cVector3d vPos;
    // Sanity Check
    if (mesh){
        uint nvertices = mesh->getNumVertices();
        uint i;
        double idx;
        for (i = 0, idx = 0 ; i < nvertices ; i++, idx++){
            vPos = mesh->m_vertices->getLocalPos(i);
            intertialOffset = ((( idx ) / ( idx + 1.0 )) * intertialOffset) + (( 1.0 / ( idx + 1.0 )) * vPos);
        }
    }
    else{
        cerr << "ERROR! CANNOT COMPUTE INERTIAL OFFSET FROM EMPTY MESH: " << m_name << endl;
    }
    return btVector3(intertialOffset.x(), intertialOffset.y(), intertialOffset.z());
}


///
/// \brief afInertialObject::computeInertialOffset
/// \param mMesh
/// \return
///
btVector3 afInertialObject::computeInertialOffset(cMultiMesh *mMesh)
{
    btVector3 inertialOffset(0, 0, 0);
    for (int i = 0 ; i < mMesh->getNumMeshes() ; i++){
        cMesh* mesh = mMesh->getMesh(0);
        inertialOffset += computeInertialOffset(mesh);
    }
    return inertialOffset;
}


///
/// \brief afInertialObject::checkCollisionGroupIdx
/// \param a_idx
/// \return
///
bool afInertialObject::checkCollisionGroupIdx(uint a_idx){
    bool in_group = false;
    for (uint i = 0 ; i < m_collisionGroups.size() ; i++){
        if (m_collisionGroups[i] == a_idx){
            in_group = true;
            break;
        }
    }

    return in_group;
}

///
/// \brief afInertialObject::isCommonCollisionGroupIdx
/// \param a_idx
/// \return
///
bool afInertialObject::isCommonCollisionGroupIdx(vector<uint> a_idx){
    bool in_group = false;
    for (uint i = 0 ; i < a_idx.size() ; i ++){
        for (uint j = 0 ; j < m_collisionGroups.size() ; j++){
            if (a_idx[i] == m_collisionGroups[j]){
                in_group = true;
                break;
            }
        }
    }

    return in_group;
}


///
/// \brief afInertialObject::applyForceAtPointOnBody
/// \param a_forceInWorld
/// \param a_pointInWorld
///
void afInertialObject::applyForceAtPointOnBody(const cVector3d &a_forceInWorld, const cVector3d &a_pointInWorld){
    if (m_bulletRigidBody){
        btTransform T_bodyInWorld = m_bulletRigidBody->getWorldTransform();
        btTransform T_worldInBody = T_bodyInWorld.inverse(); // Invert once here so we dont have to invert multiple times.
        btVector3 worldForce = to_btVector(a_forceInWorld);
        btVector3 worldPoint = to_btVector(a_pointInWorld);
        btVector3 localForce = T_worldInBody.getBasis() * worldForce;
        btVector3 localPoint = T_worldInBody * worldPoint;
        btVector3 localTorque = localPoint.cross(localForce);
        btVector3 worldTorque = T_worldInBody.getBasis() * localTorque;

        m_bulletRigidBody->applyCentralForce(worldForce);
        m_bulletRigidBody->applyTorque(worldTorque);
    }

}


///
/// \brief afInertialObject::applyForce
/// \param a_force
/// \param offset
///
void afInertialObject::applyForce(const cVector3d &a_force, const cVector3d &a_offset)
{
    btVector3 force = to_btVector(a_force);
    btVector3 offset = to_btVector(a_offset);
    m_bulletRigidBody->applyForce(force, offset);
}


///
/// \brief afInertialObject::applyTorque
/// \param a_torque
///
void afInertialObject::applyTorque(const cVector3d &a_torque)
{
    btVector3 torque = to_btVector(a_torque);
    m_bulletRigidBody->applyTorque(torque);
}

///
/// \brief afBody::afBody
/// \param a_world
///
afRigidBody::afRigidBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(afType::RIGID_BODY, a_afWorld, a_modelPtr){
    m_mesh_name.clear();
    m_scale = 1.0;

    m_dpos.setValue(0, 0, 0);
    btTransform identity;
    identity.setIdentity();
    m_bulletMotionState = new btDefaultMotionState(identity);
}


///
/// \brief afRigidBody::updateUpwardHeirarchyForAddition
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::updateUpwardHeirarchyForAddition(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //1a. We add the child body and all of it's children to this body
    vector<afChildJointPair> cjPairs;
    cjPairs = a_childBody->m_CJ_PairsAll;
    for (unsigned long i = 0 ; i < cjPairs.size() ; i++){
        cjPairs[i].m_directConnection = false; // Make sure to mark that these are not directly connected to the body
    }
    cjPairs.push_back(afChildJointPair(a_childBody, a_jnt, true));

    vector<afChildJointPair>::iterator cjIt;
    for (cjIt = cjPairs.begin() ; cjIt != cjPairs.end(); ++cjIt){
        bool cExists = false;
        for (size_t cjIdx = 0; cjIdx < m_CJ_PairsAll.size() ; cjIdx++){
            if (cjIt->m_childBody == m_CJ_PairsAll[cjIdx].m_childBody){
                cExists = true;
                break;
            }
        }

        if (!cExists){
            m_CJ_PairsAll.push_back(*cjIt);

            // Also populate the activeChildJointPairs vector
            if (cjIt->m_childJoint->isPassive() == false){
                m_CJ_PairsActive.push_back(*cjIt);
            }

            if (cjIt->m_childBody->m_afSensors.size() > 0){
                m_afSensors.insert(m_afSensors.end(), cjIt->m_childBody->m_afSensors.begin(), cjIt->m_childBody->m_afSensors.end());
            }
        }
        else{
            //            cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A CHILD BODY NAMED \""
            //                      << (*cBodyIt)->m_name << "\" PARALLEL LINKAGE FOUND" << endl;
        }
    }
}


///
/// \brief afRigidBody::updateDownwardHeirarchyForAddition
/// \param a_parentBody
///
void afRigidBody::updateDownwardHeirarchyForAddition(afRigidBodyPtr a_parentBody){
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //2a. We add the child body and all of it's children to this body
    vector<afRigidBodyPtr> pBodies;
    pBodies = a_parentBody->m_parentBodies;
    pBodies.push_back(a_parentBody);

    vector<afRigidBodyPtr>::iterator pBobyIt;
    for (pBobyIt = pBodies.begin() ; pBobyIt != pBodies.end(); ++pBobyIt){
        bool pExists = false;
        for (size_t pIdx = 0; pIdx < m_parentBodies.size() ; pIdx++){
            if (*pBobyIt == m_parentBodies[pIdx]){
                pExists = true;
                break;
            }
        }

        if (!pExists){
            m_parentBodies.push_back(*pBobyIt);
        }
        else{
            //            cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A CHILD BODY NAMED \""
            //                      << (*pBobyIt)->m_name << "\" PARALLEL LINKAGE FOUND" << endl;
        }
    }
}


///
/// \brief afRigidBody::remove
///
void afRigidBody::remove(){
    if (m_bulletRigidBody){
        m_bulletRigidBody->clearForces();
    }

    updateDownwardHeirarchyForRemoval();
    updateUpwardHeirarchyForRemoval();

    if (m_bulletRigidBody){
        m_afWorld->m_bulletWorld->removeRigidBody(m_bulletRigidBody);
    }

    removeAllChildSceneObjects();
}


///
/// \brief afRigidBody::updateUpwardHeirarchyForRemoval
///
void afRigidBody::updateUpwardHeirarchyForRemoval(){
    // We want to remove not only the current body from the parents list of all its children, but also all
    // the parents of this body from the parents list of its children

    vector<afRigidBodyPtr> childrensParents = m_parentBodies;
    childrensParents.push_back(this);
    vector<afRigidBodyPtr>::iterator cpIt;
    vector<afChildJointPair>::iterator cjIt;

    for (cpIt = childrensParents.begin() ; cpIt != childrensParents.end() ; ++cpIt){

        for (cjIt = m_CJ_PairsAll.begin() ; cjIt != m_CJ_PairsAll.end(); ++cjIt){
            afRigidBodyPtr childBody = cjIt->m_childBody;
            vector<afRigidBodyPtr>::iterator pIt;
            for (pIt = childBody->m_parentBodies.begin() ; pIt != childBody->m_parentBodies.end() ; ++pIt){
                if (*cpIt == *pIt){
                    childBody->m_parentBodies.erase(pIt);
                    break;
                }
            }
        }
    }

    // Also update the tree for active CJ pairs
    for (cpIt = childrensParents.begin() ; cpIt != childrensParents.end() ; ++cpIt){

        for (cjIt = m_CJ_PairsActive.begin() ; cjIt != m_CJ_PairsActive.end(); ++cjIt){
            afRigidBodyPtr childBody = cjIt->m_childBody;
            vector<afRigidBodyPtr>::iterator pIt;
            for (pIt = childBody->m_parentBodies.begin() ; pIt != childBody->m_parentBodies.end() ; ++pIt){
                if (*cpIt == *pIt){
                    childBody->m_parentBodies.erase(pIt);
                    break;
                }
            }
        }
    }
}


///
/// \brief afRigidBody::removalUpdateDownwardTree
///
void afRigidBody::updateDownwardHeirarchyForRemoval(){
    // We want to remove not only the current body from the children list of its parents but also all
    // the children of this body from the children list of its parents


    // First we want to remove
    vector<afChildJointPair> parentsChildrenJointPairs = m_CJ_PairsAll;
    parentsChildrenJointPairs.push_back( afChildJointPair(this, nullptr));
    vector<afChildJointPair>::iterator pCJIt;
    vector<afRigidBodyPtr>::iterator pIt;

    for (pCJIt = parentsChildrenJointPairs.begin() ; pCJIt != parentsChildrenJointPairs.end() ; ++pCJIt){
        for (pIt = m_parentBodies.begin() ; pIt != m_parentBodies.end(); ++pIt){
            afRigidBodyPtr parentBody = *pIt;
            vector<afChildJointPair>::iterator cjIt;
            for (cjIt = parentBody->m_CJ_PairsAll.begin() ; cjIt != parentBody->m_CJ_PairsAll.end() ; ++cjIt){
                if (pCJIt->m_childBody == cjIt->m_childBody){
                    if (this == cjIt->m_childBody){
                        // This the special case where we provide null for the joint.
                        // We want to clear this joint
                        if (cjIt->m_childJoint){
                            cjIt->m_childJoint->remove();
                        }
                    }
                    parentBody->m_CJ_PairsAll.erase(cjIt);
                    break;
                }
            }
        }
    }

    // Also make sure to remove all the directly connected joints to this body
    vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_CJ_PairsAll.begin() ; cjIt != m_CJ_PairsAll.end() ; ++cjIt){
        if (cjIt->m_directConnection){
            cjIt->m_childJoint->remove();
        }
    }

    ///
    ///
    ///
    // Also update the Active CJ Pairs tree.
    parentsChildrenJointPairs.clear();
    parentsChildrenJointPairs = m_CJ_PairsActive;
    parentsChildrenJointPairs.push_back( afChildJointPair(this, nullptr));

    for (pCJIt = parentsChildrenJointPairs.begin() ; pCJIt != parentsChildrenJointPairs.end() ; ++pCJIt){
        for (pIt = m_parentBodies.begin() ; pIt != m_parentBodies.end(); ++pIt){
            afRigidBodyPtr parentBody = *pIt;
            vector<afChildJointPair>::iterator cjIt;
            for (cjIt = parentBody->m_CJ_PairsActive.begin() ; cjIt != parentBody->m_CJ_PairsActive.end() ; ++cjIt){
                if (pCJIt->m_childBody == cjIt->m_childBody){
                    if (this == cjIt->m_childBody){
                        // This the special case where we provide null for the joint.
                        // We want to clear this joint
                        if (cjIt->m_childJoint){
                            cjIt->m_childJoint->remove();
                        }
                    }
                    parentBody->m_CJ_PairsActive.erase(cjIt);
                    break;
                }
            }
        }
    }

    // Also make sure to remove all the directly connected joints to this body
    for (cjIt = m_CJ_PairsActive.begin() ; cjIt != m_CJ_PairsActive.end() ; ++cjIt){
        if (cjIt->m_directConnection){
            cjIt->m_childJoint->remove();
        }
    }
}


///
/// \brief afRigidBody::addChildJointPair
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::addChildBodyJointPair(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
    //TODO: TEST THIS LOGIC RIGOROUSLY
    if (this == a_childBody){
        cerr << "INFO, BODY \"" << m_name << "\": CANNOT HAVE ITSELF AS ITS CHILD" << endl;
    }
    else{
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //1.. We go higher up the tree and add all the children and joints beneath to the parents
        vector<afRigidBodyPtr> pBodies;
        pBodies = this->m_parentBodies;
        pBodies.push_back(this);

        vector<afRigidBodyPtr>::iterator pIt;

        for (pIt = pBodies.begin() ; pIt != pBodies.end() ; ++pIt){
            (*pIt)->updateUpwardHeirarchyForAddition(a_childBody, a_jnt);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //2. Now we add this body as the parent of all the children of the child body
        vector<afChildJointPair> cjPairs;
        cjPairs = a_childBody->m_CJ_PairsAll;
        for (uint i = 0 ; i < cjPairs.size() ; i++){
            cjPairs[i].m_directConnection = false; // Make sure to mark that these are not directly connected to the body
        }
        cjPairs.push_back(afChildJointPair(a_childBody, a_jnt, true));

        vector<afChildJointPair>::iterator cjIt;
        for (cjIt = cjPairs.begin() ; cjIt != cjPairs.end() ; ++cjIt){
            cjIt->m_childBody->updateDownwardHeirarchyForAddition(this);
        }
    }
}


bool afRigidBody::createFromAttribs(afRigidBodyAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);
    m_visualMeshFilePath = a_attribs->m_visualAttribs.m_meshFilepath;
    m_collisionGeometryType = a_attribs->m_collisionAttribs.m_geometryType;
    m_collisionMeshFilePath = a_attribs->m_collisionAttribs.m_meshFilepath;

    m_scale = a_attribs->m_kinematicAttribs.m_scale;

    m_visualMesh = new cMultiMesh();
    m_collisionMesh = new cMultiMesh();
    m_collisionMesh->setShowEnabled(false);

    if (afVisualUtils::createFromAttribs(&a_attribs->m_visualAttribs, m_visualMesh, m_name)){
        m_visualMesh->scale(m_scale);
    }
    else{
        // FAILED TO LOAD MESH. RETURN FALSE
        return 0;
    }

    m_shaderAttribs = a_attribs->m_shaderAttribs;
    loadShaderProgram();

    // Set this now, but if we require the inertial offset to be estimated AND a collision
    // shape is a MESH, then estimate it to override this.
    btTransform inertialOffset = to_btTransform(a_attribs->m_inertialAttribs.m_inertialOffset);
    setInertialOffsetTransform(inertialOffset);

    if (m_collisionGeometryType == afGeometryType::MESH){
        if (m_collisionMesh->loadFromFile(m_collisionMeshFilePath.c_str()) ){
            m_collisionMesh->removeDuplicateVertices();
            m_collisionMesh->scale(m_scale);
            // Override the inertial offset if it is required by attribs
            if (a_attribs->m_inertialAttribs.m_estimateInertialOffset){
                btTransform inertialOffset;
                inertialOffset.setIdentity();
                inertialOffset.setOrigin(computeInertialOffset(m_collisionMesh));
                setInertialOffsetTransform(inertialOffset);
            }

            m_bulletCollisionShape = afShapeUtils::createCollisionShape(m_collisionMesh,
                                                                        a_attribs->m_collisionAttribs.m_margin,
                                                                        a_attribs->m_inertialAttribs.m_inertialOffset,
                                                                        a_attribs->m_collisionAttribs.m_meshShapeType);
        }
        else{
            cerr << "WARNING! Body "
                 << m_name
                 << "'s mesh \"" << m_collisionMeshFilePath.c_str() << "\" not found\n";
            return false;
        }
    }
    else if(m_collisionGeometryType == afGeometryType::SINGLE_SHAPE ||
            m_collisionGeometryType == afGeometryType::COMPOUND_SHAPE){

        // A bug in Bullet where a plane shape appended to a compound shape doesn't collide with soft bodies.
        // Thus instead of using a compound, use the single collision shape.
        if (a_attribs->m_collisionAttribs.m_primitiveShapes.size() == 0){
            // ERROR! NO PRIMITIVE SHAPES HAVE BEEN DEFINED.
            return false;
        }
        else if (a_attribs->m_collisionAttribs.m_primitiveShapes.size() == 1 && a_attribs->m_collisionAttribs.m_primitiveShapes[0].getShapeType() == afPrimitiveShapeType::PLANE){
            afPrimitiveShapeAttributes pS = a_attribs->m_collisionAttribs.m_primitiveShapes[0];
            m_bulletCollisionShape =  afShapeUtils::createCollisionShape(&pS, a_attribs->m_collisionAttribs.m_margin);
        }
        else{
            btCompoundShape* compoundCollisionShape = new btCompoundShape();
            for (unsigned long sI = 0 ; sI < a_attribs->m_collisionAttribs.m_primitiveShapes.size() ; sI++){
                afPrimitiveShapeAttributes pS = a_attribs->m_collisionAttribs.m_primitiveShapes[sI];
                btCollisionShape* collShape = afShapeUtils::createCollisionShape(&pS, a_attribs->m_collisionAttribs.m_margin);

                // Here again, we consider both the inertial offset transform and the
                // shape offset transfrom. This will change the legacy behavior but
                // luckily only a few ADFs (i.e. -l 16,17 etc) use the compound collision
                // shape. So they shall be updated.
                btTransform shapeOffset = to_btTransform(pS.getOffset());
                compoundCollisionShape->addChildShape(getInverseInertialOffsetTransform() * shapeOffset, collShape);
            }
            m_bulletCollisionShape = compoundCollisionShape;
        }
    }
    else{
        // No valid collision. Must be an empty object
        m_bulletCollisionShape = new btEmptyShape();
        cerr << "ERROR! " << m_name << " COLLISION TYPE NOT UNDERSTOOD" << endl;
    }

    // The collision groups are sorted by integer indices. A group is an array of
    // ridig bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // the a body can be a part of multiple groups

    for (uint gI = 0 ; gI < a_attribs->m_collisionAttribs.m_groups.size() ; gI++){
        uint group =  a_attribs->m_collisionAttribs.m_groups[gI];
        // Sanity check for the group number
        if (group >= 0 && group <= 999){
            m_afWorld->m_collisionGroups[group].push_back(this);
            m_collisionGroups.push_back(group);
        }
        else{
            cerr << "WARNING! Body "
                 << m_name
                 << "'s group number is \"" << group << "\" which should be between [0 - 999], ignoring\n";
        }
    }

    m_controller.createFromAttribs(&a_attribs->m_controllerAttribs);

    setMass(a_attribs->m_inertialAttribs.m_mass);
    if(a_attribs->m_inertialAttribs.m_estimateInertia){
        estimateInertia();
    }
    else{
        setInertia(a_attribs->m_inertialAttribs.m_inertia);
    }

    createInertialObject();

    // inertial origin in world
    cTransform T_iINw = to_cTransform(a_attribs->m_kinematicAttribs.m_location);
    cTransform T_mINw = T_iINw * to_cTransform(getInertialOffsetTransform());

    setInitialTransform(T_mINw);
    setLocalTransform(T_mINw);

    setSurfaceProperties(a_attribs->m_surfaceAttribs);

    m_publish_children_names = a_attribs->m_publishChildrenNames;
    m_publish_joint_names = a_attribs->m_publishJointNames;
    m_publish_joint_positions = a_attribs->m_publishJointPositions;

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    addChildSceneObject(m_visualMesh, cTransform());
    addChildSceneObject(m_collisionMesh, cTransform());
    m_afWorld->m_bulletWorld->addRigidBody(m_bulletRigidBody);

    if (a_attribs->m_inertialAttribs.m_overrideGravity){
        m_overrideGravity = true;
        m_gravity << a_attribs->m_inertialAttribs.m_gravity;
        setGravity(m_gravity);
    }

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getRigidBodyMap());
    setGlobalRemapIdx(remap_idx);

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    // Where to add the visual, collision and this object?
    return true;
}

void afRigidBody::createInertialObject()
{
    // create rigid body
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(m_mass, m_bulletMotionState, m_bulletCollisionShape, m_inertia);
    m_bulletRigidBody = new btRigidBody(rigidBodyCI);
    m_bulletRigidBody->setUserPointer(this);

    // by default deactivate sleeping mode
    m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_bulletRigidBody->setSleepingThresholds(0, 0);
}


///
/// \brief afRigidBody::estimateCartesianControllerGains
/// \param controller
/// \param computeLinear
/// \param computeAngular
///
void afRigidBody::estimateCartesianControllerGains(afCartesianController &controller, bool computeLinear, bool computeAngular){
    if (computeLinear == false && computeAngular == false){
        // NOTIFY THAT THIS WAS A USELESS CALL
        return;
    }

    double netMass = m_mass;
    btVector3 netInertia = m_inertia;
    vector<afChildJointPair>::iterator sjIt;

    for(sjIt = m_CJ_PairsActive.begin() ; sjIt != m_CJ_PairsActive.end() ; ++sjIt){
        netMass += sjIt->m_childBody->getMass();
        netInertia += sjIt->m_childBody->getInertia();
    }

    if (computeLinear){
        double P_lin, D_lin;
        P_lin = netMass * 20;
        D_lin = P_lin / 100;
        controller.setLinearGains(P_lin, 0, D_lin);
    }

    if (computeAngular){
        // TODO
        // Need a better way of estimating angular gains
        double P_ang, D_ang;
        P_ang = netMass * 10;
        D_ang = netMass;
        controller.setAngularGains(P_ang, 0, D_ang);
    }
}


///
/// \brief afRigidBody::updatePositionFromDynamics
///
void afRigidBody::update(double dt)
{
    if (m_bulletRigidBody)
    {
        m_localTransform << getCOMTransform();
    }
}


///
/// \brief afRigidBody::reset
///
void afRigidBody::reset()
{
    btVector3 zero(0, 0, 0);
    m_bulletRigidBody->clearForces();
    m_bulletRigidBody->setLinearVelocity(zero);
    m_bulletRigidBody->setAngularVelocity(zero);
    if (m_overrideGravity){
        setGravity(m_gravity);
    }
//    cTransform T_i = getInitialTransform();
//    setLocalTransform(T_i);
    afBaseObject::reset();
}


///
/// \brief afRigidBody::updateBodySensors
/// \param threadIdx
/// \return
///
bool afRigidBody::updateBodySensors(uint threadIdx){
    uint startIdx = threadIdx * m_sensorThreadBlockSize;
    uint endIdx = startIdx + m_sensorThreadBlockSize;

    endIdx = endIdx > m_afSensors.size() ? m_afSensors.size() : endIdx;
    while (m_keepSensorThreadsAlive){
        if (m_threadUpdateFlags[threadIdx] == true){

            for (uint idx = startIdx ; idx < endIdx ; idx++){
                m_afSensors[idx]->update(0.001);
            }

            m_threadUpdateFlags[threadIdx] = false;
        }
        usleep(1000);
    }
    return true;
}


///
/// \brief afRigidBody::afObjectSetChildrenNames
///
void afRigidBody::afObjectStateSetChildrenNames(){
}


///
/// \brief afRigidBody::afObjectStateSetJointNames
///
void afRigidBody::afObjectStateSetJointNames(){
}


///
/// \brief afRigidBody::afObjectSetJointPositions
///
void afRigidBody::afObjectSetJointPositions(){
}


///
/// \brief afRigidBody::afObjectSetJointVelocities
///
void afRigidBody::afObjectSetJointVelocities(){
}


///
/// \brief afRigidBody::afObjectSetJointVelocities
///
void afRigidBody::afObjectSetJointEfforts(){
}


///
/// \brief afRigidBody::setAngle
/// \param angle
///
void afRigidBody::setAngle(double &angle){
    if (m_parentBodies.size() == 0){
        for (size_t jnt = 0 ; jnt < m_CJ_PairsActive.size() ; jnt++){
            m_CJ_PairsActive[jnt].m_childJoint->commandPosition(angle);
        }

    }
}


///
/// \brief afRigidBody::setAngle
/// \param angles
///
void afRigidBody::setAngle(vector<double> &angles){
    if (m_parentBodies.size() == 0){
        double jntCmdSize = m_CJ_PairsActive.size() < angles.size() ? m_CJ_PairsActive.size() : angles.size();
        for (size_t jntIdx = 0 ; jntIdx < jntCmdSize ; jntIdx++){
            m_CJ_PairsActive[jntIdx].m_childJoint->commandPosition(angles[jntIdx]);
        }

    }
}


///
/// \brief afRigidBody::isChild
/// \param a_body
/// \return
///
bool afRigidBody::isChild(btRigidBody *a_body){
    bool isChild = false;
    vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_CJ_PairsAll.begin() ; cjIt != m_CJ_PairsAll.end() ; ++cjIt){
        if (a_body == cjIt->m_childBody->m_bulletRigidBody){
            isChild = true;
            break;
        }
    }

    return isChild;
}


///
/// \brief afRigidBody::isDirectChild
/// \param a_body
/// \return
///
bool afRigidBody::isDirectChild(btRigidBody *a_body){
    bool isDirectChild = false;
    vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_CJ_PairsAll.begin() ; cjIt != m_CJ_PairsAll.end() ; ++cjIt){
        if (a_body == cjIt->m_childBody->m_bulletRigidBody){
            if (cjIt->m_directConnection){
                isDirectChild = true;
            }
            break;
        }
    }

    return isDirectChild;
}

///
/// \brief afRigidBody::~afRigidBody
///
afRigidBody::~afRigidBody(){
    int numConstraints = m_bulletRigidBody->getNumConstraintRefs();
    for (int i = numConstraints - 1 ; i >=0 ; i--){
        btTypedConstraint * tConstraint = m_bulletRigidBody->getConstraintRef(i);
        m_bulletRigidBody->removeConstraintRef(tConstraint);
    }
}

void afRigidBody::setLocalTransform(const cTransform &trans)
{
    m_bulletMotionState->setWorldTransform(to_btTransform(trans));
    m_bulletRigidBody->setCenterOfMassTransform(to_btTransform(trans));
    cTransform com = trans * to_cTransform(getInverseInertialOffsetTransform());
    afBaseObject::setLocalTransform(com);
}





///
/// \brief afMeshCleanup::updateMins
/// \param vMin
/// \param v
///
void afMeshCleanup::updateMins(cVector3d &vMin, cVector3d &v){
    vMin.x(cMin(vMin.x(),v.x()));
    vMin.y(cMin(vMin.y(),v.y()));
    vMin.z(cMin(vMin.z(),v.z()));
}


///
/// \brief afMeshCleanup::updateMaxs
/// \param vMax
/// \param v
///
void afMeshCleanup::updateMaxs(cVector3d &vMax, cVector3d &v){
    vMax.x(cMax(vMax.x(),v.x()));
    vMax.y(cMax(vMax.y(),v.y()));
    vMax.z(cMax(vMax.z(),v.z()));
}


///
/// \brief afSoftBody::afSoftBody
/// \param a_afWorld
///
afSoftBody::afSoftBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(afType::SOFT_BODY, a_afWorld, a_modelPtr){
}


///
/// \brief afSoftBody::createFromAttribs
/// \param a_attribs
/// \return
///
bool afSoftBody::createFromAttribs(afSoftBodyAttributes *a_attribs)
{
    storeAttributes(a_attribs);

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_scale = a_attribs->m_kinematicAttribs.m_scale;

    btTransform iOff = to_btTransform(a_attribs->m_inertialAttribs.m_inertialOffset);
    setInertialOffsetTransform(iOff);

    m_visualMesh = new cMultiMesh();
    m_collisionMesh = new cMultiMesh();

    if (afVisualUtils::createFromAttribs(&a_attribs->m_visualAttribs, m_visualMesh, m_name)){
        m_visualMesh->scale(m_scale);
    }
    else
    {
        // If we can't find the visual mesh, we can proceed with
        // printing just an error and returning
        cerr << "ERROR! Soft Body " << m_name
             << "'s mesh " << a_attribs->m_visualAttribs.m_meshFilepath.c_str() << " not found\n";
        return 0;
    }

    if (m_collisionMesh->loadFromFile(a_attribs->m_collisionAttribs.m_meshFilepath.c_str())){
        m_collisionMesh->removeDuplicateVertices();
        m_collisionMesh->scale(m_scale);
        // Use the visual mesh for generating the softbody
        generateFromMesh(m_collisionMesh, a_attribs->m_collisionAttribs.m_margin);
    }
    else
    {
        cerr << "ERROR! Soft Body " << m_name
             << "'s mesh " << a_attribs->m_collisionAttribs.m_meshFilepath.c_str() << " not found\n";
        return 0;
    }

    setMass(a_attribs->m_inertialAttribs.m_mass);
    m_bulletSoftBody->setTotalMass(m_mass, false);
    m_bulletSoftBody->getCollisionShape()->setUserPointer(m_bulletSoftBody);

    createInertialObject();

    cTransform pose = to_cTransform(a_attribs->m_kinematicAttribs.m_location);
    setLocalTransform(pose);

    btSoftBody* softBody = m_bulletSoftBody;

    if (a_attribs->m_useMaterial){
        btSoftBody::Material *pm = softBody->appendMaterial();
        //        pm->m_kLST = a_attribs->m_kLST;
        //        pm->m_kAST = a_attribs->m_kAST;
        //        pm->m_kVST = a_attribs->m_kVST;

        softBody->m_materials[0]->m_kLST = a_attribs->m_kLST;
        softBody->m_materials[0]->m_kAST = a_attribs->m_kAST;
        softBody->m_materials[0]->m_kVST = a_attribs->m_kVST;
    }

    if (a_attribs->m_usePoseMatching){
        softBody->m_cfg.kMT = a_attribs->m_kMT;
        softBody->setPose(false, true);
    }

    softBody->m_cfg.kVCF = a_attribs->m_kVCF;

    softBody->m_cfg.kDP = a_attribs->m_kDP;
    softBody->m_cfg.kDG = a_attribs->m_kDG;
    softBody->m_cfg.kLF = a_attribs->m_kLF;
    softBody->m_cfg.kPR = a_attribs->m_kPR;
    softBody->m_cfg.kVC = a_attribs->m_kVC;
    softBody->m_cfg.kDF = a_attribs->m_kDF;

    softBody->m_cfg.kCHR = a_attribs->m_kCHR;
    softBody->m_cfg.kKHR = a_attribs->m_kKHR;
    softBody->m_cfg.kSHR = a_attribs->m_kSHR;
    softBody->m_cfg.kAHR = a_attribs->m_kAHR;

    softBody->m_cfg.kSRHR_CL = a_attribs->m_kSRHR_CL;
    softBody->m_cfg.kSKHR_CL = a_attribs->m_kSKHR_CL;
    softBody->m_cfg.kSSHR_CL = a_attribs->m_kSSHR_CL;

    softBody->m_cfg.kSR_SPLT_CL = a_attribs->m_kSR_SPLT_CL;
    softBody->m_cfg.kSK_SPLT_CL = a_attribs->m_kSK_SPLT_CL;
    softBody->m_cfg.kSS_SPLT_CL = a_attribs->m_kSS_SPLT_CL;

    softBody->m_cfg.maxvolume = a_attribs->m_maxVolume;
    softBody->m_cfg.timescale = a_attribs->m_timeScale;

    softBody->m_cfg.viterations = a_attribs->m_vIterations;
    softBody->m_cfg.piterations = a_attribs->m_pIterations;
    softBody->m_cfg.diterations = a_attribs->m_dIterations;
    softBody->m_cfg.citerations = a_attribs->m_cIterations;

    softBody->m_cfg.collisions = a_attribs->m_flags;

    if (a_attribs->m_useBendingConstraints){
        softBody->generateBendingConstraints(a_attribs->m_bendingConstraint);
    }


    // If a vertexIdx Map is defined, we can retrieve the actual indices defined in the mesh file.
    bool useOriginalIndexes = getVisualObject()->m_vtxIdxMap.size() > 0 ? true : false;

    for (uint i = 0 ; i < a_attribs->m_fixedNodes.size() ; i++){

        uint nodeIdx = a_attribs->m_fixedNodes[i];
        if ( nodeIdx > softBody->m_nodes.size()){break;}

        if (useOriginalIndexes){
            // Find the node's original vertex index
            map<int, vector<int> >::iterator nIt = getVisualObject()->m_vtxIdxMap.find(nodeIdx);
            if ( nIt != getVisualObject()->m_vtxIdxMap.end()){
                if (nIt->second.size() == 0){break;}
                int originalVtxIdx = nIt->second[0];
                unsigned int newIdx = m_collisionMesh->getMesh(0)->getNewVertexIndex(originalVtxIdx);
                if (newIdx > 0){
                    cerr << "INFO! Fixing Softbody Node. Original Node Idx: " << nodeIdx
                         << " | Old Vertex/Node Idx:  " << originalVtxIdx
                         << " | New Vertex/Node Idx: " << newIdx << endl;
                    softBody->setMass(newIdx, 0);
                }
            }
        }
        else{
            softBody->setMass(nodeIdx, 0);
        }
    }

    if(a_attribs->m_useClusters){
        softBody->generateClusters(a_attribs->m_clusters);
    }

    if (a_attribs->m_useConstraintRandomization){
        softBody->randomizeConstraints();
    }

    addChildSceneObject(m_visualMesh, cTransform());
    ((btSoftRigidDynamicsWorld*)m_afWorld->m_bulletWorld)->addSoftBody(m_bulletSoftBody);
    m_afWorld->m_bulletSoftBodyWorldInfo->m_sparsesdf.Reset();

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getSoftBodyMap());
    setGlobalRemapIdx(remap_idx);

    setPassive(true);

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    return true;
}

void afSoftBody::createInertialObject()
{
    m_bulletSoftBody->setUserPointer(this);
}

void afSoftBody::setLocalTransform(const cTransform &trans)
{
    m_bulletSoftBody->transform(to_btTransform(trans));
}

void afSoftBody::toggleSkeletalModelVisibility(){
    if (m_visualMesh->getNumMeshes() > 0){
        m_visualMesh->setShowEdges(!m_visualMesh->getMesh(0)->getShowEdges());
    }
}

void afSoftBody::updateSceneObjects(){
    cMesh * mesh = m_visualMesh->getMesh(0);

    for (int i = 0 ; i < m_bulletSoftBody->m_nodes.size() ; i++){
        mesh->setVertexLocalPosForAllDuplicates(i, m_bulletSoftBody->m_nodes[i].m_x[0], m_bulletSoftBody->m_nodes[i].m_x[1], m_bulletSoftBody->m_nodes[i].m_x[2]);
    }

    mesh->computeAllNormals();
    mesh->markForUpdate(true);
    afBaseObject::updateSceneObjects();
}

bool afSoftBody::cleanupMesh(cMultiMesh *multiMesh, std::vector<afVertexTree> &a_afVertexTree, std::vector<unsigned int> &a_triangles)
{

    bool valid = true;
    //Store the elements of the mesh first.
    cMesh* reducedMesh = new cMesh();
    cMesh* tempMesh;
    reducedMesh->m_vertices->allocateData(a_afVertexTree.size(), true, true, true, true, true, false);
    for (int i=0; i<a_afVertexTree.size();i++){
        unsigned int resolved_idx;
        if (multiMesh->getVertex(a_afVertexTree[i].vertexIdx[0], tempMesh, resolved_idx)){
            cVector3d position = tempMesh->m_vertices->getLocalPos(resolved_idx);
            cVector3d normal = tempMesh->m_vertices->getNormal(resolved_idx);
            cVector3d tex_coord = tempMesh->m_vertices->getTexCoord(resolved_idx);
            cVector3d tangent = tempMesh->m_vertices->getTangent(resolved_idx);
            cVector3d bit_tangent = tempMesh->m_vertices->getBitangent(resolved_idx);

            reducedMesh->m_vertices->setLocalPos(i, position);
            reducedMesh->m_vertices->setNormal(i, normal);
            reducedMesh->m_vertices->setTexCoord(i, tex_coord);
            reducedMesh->m_vertices->setTangent(i, tangent);
            reducedMesh->m_vertices->setBitangent(i, bit_tangent);
        }
        else{
            // THROW SOME ERROR
            cerr << "ERROR! EXPERIMENTAL! CANNOT CLEAN UP MESH FOR SOFT-BDOY, PLEASE CHECK LOGIC" << endl;
            delete reducedMesh;
            valid = false;
            break;
        }
    }

    if (valid){
        for (int i=0 ;  i < a_triangles.size()/3 ; i++){
            reducedMesh->newTriangle(a_triangles[3*i], a_triangles[3*i+1], a_triangles[3*i+2]);
        }

        reducedMesh->computeAllEdges();
        //        reducedMesh->computeAllNormals();
        reducedMesh->setMaterial(multiMesh->m_material);
        reducedMesh->setTexture(multiMesh->getMesh(0)->m_texture);
        reducedMesh->setUseTexture(multiMesh->getMesh(0)->getUseTexture());
        reducedMesh->setShaderProgram(multiMesh->getShaderProgram());
        reducedMesh->setShowEdges(false);
        multiMesh->m_meshes->clear();
        multiMesh->addMesh(reducedMesh);
    }

    return valid;
}


///
/// \brief afSoftBody::generateFromMesh
/// \param multiMesh
/// \param a_margin
/// \return
///
bool afSoftBody::generateFromMesh(cMultiMesh *multiMesh, const double a_margin)
{
    // create compound shape
    m_bulletCollisionShape = new btCompoundShape();;

    // create collision detector for each mesh
    for (int mi=0; mi < multiMesh->getNumMeshes() ; mi++)
    {
        cMesh* mesh = multiMesh->getMesh(mi);
        m_bulletSoftBody = createFromMesh(m_afWorld->m_bulletSoftBodyWorldInfo, mesh, false);
        createLinksFromLines(m_bulletSoftBody, &mesh->m_lines, mesh);
        m_bulletSoftBody->getCollisionShape()->setMargin(a_margin);
    }
    return true;
}


///
/// \brief afSoftBody::createLinksFromLines
/// \param a_softBody
/// \param a_lines
/// \param a_mesh
/// \return
///
bool afSoftBody::createLinksFromLines(btSoftBody *a_softBody, std::vector< std::vector<int> > *a_lines, cMesh* a_mesh){
    if (a_softBody && a_lines){
        for(int lIdx = 0 ; lIdx < a_lines->size() ; lIdx++){
            std::vector<int> line = (*a_lines)[lIdx];

            for(int vIdx = 0 ; vIdx < line.size() - 1 ; vIdx++){
                int node0Idx = line[vIdx];
                int node1Idx = line[vIdx+1];
                int nodesSize = a_softBody->m_nodes.size();

                if (node0Idx >= nodesSize){
                    int originalVtxIdx = a_mesh->m_lines[lIdx][vIdx];
                    btVector3 vPos = to_btVector(a_mesh->m_vertices->getLocalPos(originalVtxIdx));
                    btSoftBody::Node n;
                    n.m_im = 1;
                    n.m_im = 1 / n.m_im;
                    n.m_x = vPos;
                    n.m_q = n.m_x;
                    n.m_n = btVector3(0, 0, 1);
                    n.m_leaf = m_bulletSoftBody->m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x, 0.1), &n);
                    n.m_material = m_bulletSoftBody->m_materials[0];
                    a_softBody->m_nodes.push_back(n);
                    node0Idx = a_softBody->m_nodes.size() - 1;
                }

                if (node1Idx >= nodesSize){
                    int originalVtxIdx = a_mesh->m_lines[lIdx][vIdx];
                    btVector3 vPos = to_btVector(a_mesh->m_vertices->getLocalPos(originalVtxIdx));
                    btSoftBody::Node n;
                    n.m_im = 1;
                    n.m_im = 1 / n.m_im;
                    n.m_x = vPos;
                    n.m_q = n.m_x;
                    n.m_n = btVector3(0, 0, 1);
                    n.m_leaf = m_bulletSoftBody->m_ndbvt.insert(btDbvtVolume::FromCR(n.m_x, 0.1), &n);
                    n.m_material = m_bulletSoftBody->m_materials[0];
                    a_softBody->m_nodes.push_back(n);
                    node1Idx = a_softBody->m_nodes.size() - 1;
                }
                a_softBody->appendLink(node0Idx, node1Idx);
            }
        }
    }

    return true;
}


///
/// \brief afSoftBody::createFromMesh
/// \param worldInfo
/// \param a_mesh
/// \param randomizeConstraints
/// \return
///
btSoftBody* afSoftBody::createFromMesh(btSoftBodyWorldInfo* worldInfo, cMesh *a_mesh, bool randomizeConstraints){
    unsigned int maxidx = 0;
    int i, j, ni;

    for (i = 0, ni = a_mesh->getNumTriangles() * 3; i < ni; ++i)
    {
        maxidx = btMax(a_mesh->m_triangles->m_indices[i], maxidx);
    }
    ++maxidx;
    btAlignedObjectArray<bool> chks;
    btAlignedObjectArray<btVector3> vtx;
    chks.resize(maxidx * maxidx, false);
    vtx.resize(a_mesh->getNumVertices());
    for (i = 0; i < a_mesh->getNumVertices(); i++)
    {
        vtx[i] = to_btVector(a_mesh->m_vertices->getLocalPos(i));
    }
    btSoftBody* psb = new btSoftBody(worldInfo, vtx.size(), &vtx[0], 0);
    for (i = 0, ni = a_mesh->getNumTriangles() * 3; i < ni; i += 3)
    {
        const unsigned int idx[] = {a_mesh->m_triangles->m_indices[i], a_mesh->m_triangles->m_indices[i + 1], a_mesh->m_triangles->m_indices[i + 2]};
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
/// \brief afController::computeOutput
/// \param process_val
/// \param set_point
/// \param current_time
/// \return
///
afJointController::afJointController(){
}

bool afJointController::createFromAttribs(afJointControllerAttributes *a_attribs)
{
    m_P = a_attribs->m_P;
    m_I = a_attribs->m_I;
    m_D = a_attribs->m_D;
    m_maxImpulse = a_attribs->m_maxImpulse;
    m_outputType = a_attribs->m_outputType;

    return true;
}

double afJointController::computeOutput(double process_val, double set_point, double current_time){
    uint n = queue_length - 1;
    for (uint i = 0 ; i < n ; i++){
        t[i] = t[i+1];
        e[i] = e[i+1];
        de[i] = de[i+1];
        ie[i] = ie[i+1];
    }
    t[n] = current_time;
    e[n] = set_point - process_val;
    // Clamp freq to 10 KHZ
    double dt = t[n] - t[n-1];
    if (dt <= m_min_dt){
        cerr << "WARNING! JOINT CONTROLLER dt = " << dt << " Sec. Clamping to " << m_min_dt << " Sec." << endl;
        dt = m_min_dt;
    }
    de[n] = (e[n] - e[n-1]) / dt;
    ie[n] = Ie_sum + ((e[n] + e[n-1]) / 2 * dt);
    Ie_sum = ie[n];
    output = (m_P * e[n]) + (m_I * ie[n]) + (m_D * de[n]);
    return output;
}


///
/// \brief afController::boundImpulse
/// \param effort_cmd
/// \param effort_time
///
void afJointController::boundImpulse(double &effort_cmd){
    double impulse = ( effort_cmd - m_last_cmd ) / (t[0]- t[1]);
    //    cerr << "Before " << effort_cmd ;
    int sign = 1;
    if (impulse > m_maxImpulse){
        if (impulse < 0){
            sign = -1;
        }
        effort_cmd = m_last_cmd + (sign * m_maxImpulse * (t[0]- t[1]));
    }
    //    cerr << " - After " << effort_cmd << " Impulse: " << max_impulse << endl ;
    m_last_cmd = effort_cmd;
}


///
/// \brief afJoint::afJoint
///
afJoint::afJoint(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::JOINT, a_afWorld, a_modelPtr){
    m_posArray.resize(m_jpSize);
    m_dtArray.resize(m_jpSize);
    m_enableLimits = false;
}


bool afJoint::createFromAttribs(afJointAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afJointAttributes &attribs = *a_attribs;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;
    m_childName = a_attribs->m_hierarchyAttribs.m_childName;
    m_enableActuator = a_attribs->m_enableMotor;
    m_jointOffset = a_attribs->m_jointOffset;
    m_childOffset = a_attribs->m_childOffset;
    m_enableLimits = a_attribs->m_enableLimits;
    m_lowerLimit = a_attribs->m_lowerLimit;
    m_upperLimit = a_attribs->m_upperLimit;
    //Default joint type is revolute if not type is specified
    m_jointType = a_attribs->m_jointType;
    m_damping = a_attribs->m_damping; // Initialize damping to 0

    // First we should search in the local Model space and if we don't find the body.
    // Only then we find the world space


    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getJointMap());
    setGlobalRemapIdx(remap_idx);

    m_afParentBody = findConnectingBody(m_parentName);
    m_afChildBody = findConnectingBody(m_childName);

    if (m_afParentBody == nullptr){
        cerr <<"ERROR! JOINT: \"" << m_name << "\'s\" PARENT BODY \"" << m_parentName <<"\" NOT FOUND" << endl;
        return 0;
    }

    if (m_afChildBody == nullptr){
        cerr <<"ERROR! JOINT: \"" << m_name << "\'s\" CHILD BODY \"" << m_childName << "\" NOT FOUND" << endl;
        return 0;
    }

    m_controller.createFromAttribs(&a_attribs->m_controllerAttribs);

    m_pvtA = to_btVector(a_attribs->m_parentPivot * m_afParentBody->m_scale);
    m_axisA = to_btVector(a_attribs->m_parentAxis);
    m_axisA.normalize();
    m_pvtA = m_afParentBody->getInverseInertialOffsetTransform() * m_pvtA;
    m_axisA = m_afParentBody->getInverseInertialOffsetTransform().getBasis() * m_axisA;

    m_pvtB = to_btVector(a_attribs->m_childPivot * m_afChildBody->m_scale);
    m_axisB = to_btVector(a_attribs->m_childAxis);
    m_axisB.normalize();
    m_pvtB = m_afChildBody->getInverseInertialOffsetTransform() * m_pvtB;
    m_axisB = m_afChildBody->getInverseInertialOffsetTransform().getBasis() * m_axisB;

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    m_enableFeedback = a_attribs->m_enableFeedback;

    // Compute frameA and frameB from constraint axis data. This step is common
    // for all joints, the only thing that changes in the constraint axis which can be
    // set the appropriate joint type

    btTransform frameA, frameB;
    frameA.setIdentity();
    frameB.setIdentity();

    // Bullet takes the x axis as the default for prismatic joints
    btVector3 ax_jINp = getDefaultJointAxisInParent(m_jointType);
    // Rotation of constraint in parent axis as quaternion
    btQuaternion quat_jINp;
    quat_jINp = afUtils::getRotBetweenVectors<btQuaternion, btVector3>(ax_jINp, m_axisA);

    // Offset rotation along the parent axis
    btQuaternion quat_jOffINp;
    quat_jOffINp.setRotation(m_axisA, m_jointOffset);

    frameA.setRotation(quat_jOffINp * quat_jINp);
    frameA.setOrigin(m_pvtA);

    // Rotation of child axis in parent axis as Quaternion
    btQuaternion quat_cINp;
    quat_cINp = afUtils::getRotBetweenVectors<btQuaternion, btVector3>(m_axisB, m_axisA);

    // Offset rotation along the parent axis
    btQuaternion quat_cOffINp;
    quat_cOffINp.setRotation(m_axisA, m_childOffset);
    // We need to post-multiply frameA's rot to cancel out the shift in axis, then
    // the offset along joint axis and finally frameB's axis alignment in frameA.
    frameB.setRotation( quat_cINp.inverse() * quat_cOffINp.inverse() * quat_jOffINp * quat_jINp );
    frameB.setOrigin(m_pvtB);

    switch (m_jointType) {
    case afJointType::REVOLUTE:{
        //        m_hinge = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
        m_hinge = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_hinge->setParam(BT_CONSTRAINT_ERP, a_attribs->m_erp);
        m_hinge->setParam(BT_CONSTRAINT_CFM, a_attribs->m_cfm);
        m_hinge->enableAngularMotor(false, 0.0, a_attribs->m_maxMotorImpulse);

        if(a_attribs->m_enableLimits){
            m_hinge->setLimit(m_lowerLimit, m_upperLimit);
        }
        m_btConstraint = m_hinge;
    }
        break;
    case afJointType::PRISMATIC:{
        m_slider = new btSliderConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_slider->setParam(BT_CONSTRAINT_ERP, a_attribs->m_erp);
        m_slider->setParam(BT_CONSTRAINT_CFM, a_attribs->m_cfm);

        if(a_attribs->m_enableLimits){
            m_slider->setLowerLinLimit(m_lowerLimit);
            m_slider->setUpperLinLimit(m_upperLimit);
        }
        // Ugly hack, divide by (default) fixed timestep to max linear motor force
        // since m_slider does have a max impulse setting method.
        m_slider->setMaxLinMotorForce(a_attribs->m_maxMotorImpulse / 0.001);
        m_slider->setPoweredLinMotor(false);
        m_btConstraint = m_slider;
    }
        break;
    case afJointType::FIXED:{
        m_btConstraint = new btFixedConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB);
    }
        break;
    case afJointType::LINEAR_SPRING:
    case afJointType::TORSION_SPRING:{
        m_spring = new btGeneric6DofSpring2Constraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, RotateOrder::RO_XYZ);

        // Initialize all the 6 axes to 0 stiffness and damping and limits also set to 0-0
        for (int axIdx = 0 ; axIdx < 6 ; axIdx++){
            m_spring->setLimit(axIdx, 0.0, 0.0);
            m_spring->setStiffness(axIdx, 0.0);
            m_spring->setDamping(axIdx, 0.0);
            m_spring->enableSpring(axIdx, false);
        }

        // We treat springs along the z axes of constraint, thus chosed the appropriate axis
        // number based on if the spring is linear or torsional [0-2] -> linear, [3-5] -> rotational
        int axisIdx = -1;

        if (m_jointType == afJointType::LINEAR_SPRING){
            axisIdx = 2;
        }
        else if (m_jointType == afJointType::TORSION_SPRING){
            axisIdx = 5;
        }

        if (a_attribs->m_enableLimits){
            // Somehow bullets springs limits for rotational joints are inverted.
            // So handle them internally rather than breaking AMBF description specificaiton
            if (m_jointType == afJointType::LINEAR_SPRING){
                m_spring->setLimit(axisIdx, m_lowerLimit, m_upperLimit);
                m_spring->setEquilibriumPoint(axisIdx, a_attribs->m_equilibriumPoint);
            }
            else if (m_jointType == afJointType::TORSION_SPRING){
                m_spring->setLimit(axisIdx, -m_upperLimit, -m_lowerLimit);
                m_spring->setEquilibriumPoint(axisIdx, -a_attribs->m_equilibriumPoint);
            }

            m_spring->enableSpring(axisIdx, true);
        }

        m_spring->setStiffness(axisIdx, a_attribs->m_stiffness);
        m_spring->setDamping(axisIdx, m_damping);
        m_spring->setParam(BT_CONSTRAINT_STOP_ERP, a_attribs->m_erp, axisIdx);
        m_spring->setParam(BT_CONSTRAINT_CFM, a_attribs->m_cfm, axisIdx);
        m_btConstraint = m_spring;
    }
        break;
    case afJointType::P2P:{
        // p2p joint doesnt concern itself with rotations, its set using just the pivot information
        m_p2p = new btPoint2PointConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB);
        m_p2p->setParam(BT_CONSTRAINT_ERP, a_attribs->m_erp);
        m_p2p->setParam(BT_CONSTRAINT_CFM, a_attribs->m_cfm);
        m_btConstraint = m_p2p;

    }
        break;
    case afJointType::CONE_TWIST:{
        m_coneTwist = new btConeTwistConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB);
        m_coneTwist->setLimit(a_attribs->m_coneTwistLimits.m_Z, a_attribs->m_coneTwistLimits.m_Y, a_attribs->m_coneTwistLimits.m_X);
        m_coneTwist->setDamping(a_attribs->m_damping);
        m_btConstraint = m_coneTwist;

    }
        break;
    case afJointType::SIX_DOF:{
        m_sixDof = new btGeneric6DofConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        for (int id = 0 ; id < 3 ; id++){
            m_sixDof->setLimit(id, a_attribs->m_sixDofLimits.m_lowerLimit[id], a_attribs->m_sixDofLimits.m_upperLimit[id]);
        }
        for (int id = 3 ; id < 6 ; id++){
            // The rotational limits are inverted in Bullet
            m_sixDof->setLimit(id, -a_attribs->m_sixDofLimits.m_upperLimit[id], -a_attribs->m_sixDofLimits.m_lowerLimit[id]);
        }

        m_btConstraint = m_sixDof;

    }
        break;
    case afJointType::SIX_DOF_SPRING:{
        m_sixDofSpring = new btGeneric6DofSpring2Constraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, RotateOrder::RO_XYZ);
        for (int id = 0 ; id < 3 ; id++){
            m_sixDofSpring->setLimit(id, a_attribs->m_sixDofLimits.m_lowerLimit[id], a_attribs->m_sixDofLimits.m_upperLimit[id]);
            m_sixDofSpring->setEquilibriumPoint(id, a_attribs->m_sixDofSpringAttribs.m_equilibriumPoint[id]);
        }
        for (int id = 3 ; id < 6 ; id++){
            // The rotational limits are inverted in Bullet
            m_sixDofSpring->setLimit(id, -a_attribs->m_sixDofLimits.m_upperLimit[id], -a_attribs->m_sixDofLimits.m_lowerLimit[id]);
            m_sixDofSpring->setEquilibriumPoint(id, -a_attribs->m_sixDofSpringAttribs.m_equilibriumPoint[id]);
        }
        for (int id = 0 ; id < 6 ; id++){
            m_sixDofSpring->setDamping(id, a_attribs->m_sixDofSpringAttribs.m_damping[id]);
            m_sixDofSpring->setStiffness(id, a_attribs->m_sixDofSpringAttribs.m_stiffness[id]);
            m_sixDofSpring->enableSpring(id, true);
        }

        m_btConstraint = m_sixDofSpring;
    }
        break;
    default:
        break;
    }

    if (m_btConstraint != nullptr){
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, a_attribs->m_ignoreInterCollision);
        m_afParentBody->addChildBodyJointPair(m_afChildBody, this);

        if (m_enableFeedback){
            m_btConstraint->enableFeedback(m_enableFeedback);
            m_feedback = new btJointFeedback();
            m_btConstraint->setJointFeedback(m_feedback);
        }
    }

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

//    loadCommunicationPlugin(this, a_attribs);

    return true;
}

void afJoint::update(double dt){
    cacheState(dt);
}


afRigidBodyPtr afJoint::findConnectingBody(string body_name){
    afRigidBodyPtr connectingBody = nullptr;

    connectingBody = m_modelPtr->getRigidBody(body_name, true);
    if (connectingBody == nullptr){
        connectingBody = m_modelPtr->getRigidBody(getNamespace() + body_name, true);
        if (connectingBody == nullptr){
            connectingBody = m_afWorld->getRigidBody(getNamespace() + body_name + getGlobalRemapIdx(), true);
            // If we couldn't find the body with name_remapping, it might have been
            // Defined in another ambf file. Search without name_remapping string
            if(connectingBody == nullptr){
                connectingBody = m_afWorld->getRigidBody(body_name, true);
                // If the body is not world, print what we just did
                if (connectingBody != nullptr && !(strcmp(connectingBody->m_name.c_str(), "world") == 0)
                        && !(strcmp(connectingBody->m_name.c_str(), "World") == 0)
                        && !(strcmp(connectingBody->m_name.c_str(), "WORLD") == 0)){
                                cerr <<"INFO! JOINT: \"" << m_name << "\'s\" PARENT/CHILD BODY \"" << body_name << "\" FOUND IN ANOTHER ADF," << endl;
                }
            }
        }
    }

    return connectingBody;
}


btVector3 afJoint::getDefaultJointAxisInParent(afJointType a_type)
{
    btVector3 jINp(0, 0, 1);
    switch (a_type) {
    case afJointType::REVOLUTE:
    case afJointType::FIXED:
    case afJointType::LINEAR_SPRING:
    case afJointType::TORSION_SPRING:
    case afJointType::P2P:
    case afJointType::CONE_TWIST:
    case afJointType::SIX_DOF:
    case afJointType::SIX_DOF_SPRING:
        jINp.setValue(0, 0, 1);
        break;
    case afJointType::PRISMATIC:
        jINp.setValue(1, 0, 0);
        break;
    default:
        break;
    }
    return jINp;
}

void afJoint::cacheState(const double &dt){
    for (uint i = 0 ; i < m_jpSize-1 ; i++){
        m_posArray[i] = m_posArray[i+1];
        m_dtArray[i] = m_dtArray[i+1];
    }
    m_posArray[m_jpSize-1] = getPosition();
    m_dtArray[m_jpSize-1] = dt;
}


void afJoint::remove(){
    if (m_btConstraint){
        m_afWorld->m_bulletWorld->removeConstraint(m_btConstraint);
    }
}


///
/// \brief afJoint::applyDamping
///
void afJoint::applyDamping(const double &dt){
    double effort = - m_damping * getVelocity();
    // Since we are applying damping internally, don't override the motor
    // enable/disable as an external controller may be using the motor.
    commandEffort(effort, true);
}


///
/// \brief afJoint::commandPosition
/// \param position_cmd
///
void afJoint::commandPosition(double &position_cmd){
    // The torque commands disable the motor, so double check and re-enable the motor
    // if it was set to be enabled in the first place
    if (m_enableActuator){
        if (m_jointType == afJointType::REVOLUTE || m_jointType == afJointType::PRISMATIC){
            // Sanity check
            double position_cur = getPosition();
            if (m_enableLimits){
                btClamp(position_cmd, m_lowerLimit, m_upperLimit);
            }
            else{
                if (m_jointType == afJointType::REVOLUTE){
                    // The joint is continous. Need some optimization
                    position_cmd = getShortestAngle(position_cur, position_cmd);
                    position_cur = 0.0;
                }
            }

            double command = m_controller.computeOutput(position_cur, position_cmd, m_afWorld->getSimulationTime());
            if (m_controller.m_outputType == afControlType::FORCE){
                commandEffort(command);
            }
            else{
                commandVelocity(command);
            }
        }
    }
    else{
        cerr << "WARNING, MOTOR NOT ENABLED FOR JOINT: " << m_name << endl;
    }
}


double ambf::afJoint::afJoint::getShortestAngle(double current, double target){
    if (current < 0.0){
        current = 2 * PI + current;
    }
    double delta_angle = fmod( (target - current + 3 * PI), 2*PI) - PI;
    return delta_angle;
}

///
/// \brief afJoint::commandEffort. The option skip motor check is to ignore the enabling / diasbling of
/// motor. This is useful is one wants to apply an effort without disabling the motor if it was enabled
/// or without enabling it if was disabled.
/// \param cmd
/// \param skip_motor_check
///
void afJoint::commandEffort(double &cmd, bool skip_motor_check){
    if (m_jointType == afJointType::REVOLUTE || m_jointType == afJointType::TORSION_SPRING){
        if (m_jointType == afJointType::REVOLUTE && ! skip_motor_check){
            m_hinge->enableMotor(false);
        }
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
        m_btConstraint->getRigidBodyA().applyTorque(-hingeAxisInWorld * cmd);
        m_btConstraint->getRigidBodyB().applyTorque(hingeAxisInWorld * cmd);
    }
    else if (m_jointType == afJointType::PRISMATIC || m_jointType == afJointType::LINEAR_SPRING){
        if (m_jointType == afJointType::PRISMATIC && ! skip_motor_check){
            m_slider->setPoweredLinMotor(false);
        }
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        const btVector3 sliderAxisInWorld = trA.getBasis()*m_axisA;
        const btVector3 relPos(0,0,0);
        m_afParentBody->m_bulletRigidBody->applyForce(-sliderAxisInWorld * cmd, relPos);
        m_afChildBody->m_bulletRigidBody->applyForce(sliderAxisInWorld * cmd, relPos);
    }
}


///
/// \brief afJoint::commandVelocity
/// \param cmd
///
void afJoint::commandVelocity(double &velocity_cmd){
    if (m_jointType == afJointType::REVOLUTE){
        m_hinge->enableMotor(true);
        m_hinge->setMotorTargetVelocity(velocity_cmd);

    }
    else if (m_jointType == afJointType::PRISMATIC){
        m_slider->setPoweredLinMotor(true);
        m_slider->setTargetLinMotorVelocity(velocity_cmd);
    }
}

///
/// \brief afJoint::getPosition
/// \return
///
double afJoint::getPosition(){
    double jntPos = 0.0;
    if (m_jointType == afJointType::REVOLUTE)
        jntPos = m_hinge->getHingeAngle();
    else if (m_jointType == afJointType::PRISMATIC)
        jntPos = m_slider->getLinearPos();
    else if (m_jointType == afJointType::FIXED)
        jntPos = 0;
    else if (m_jointType == afJointType::LINEAR_SPRING){
        // Adapted form btSlider Constraint
        btGeneric6DofSpringConstraint* springConstraint = (btGeneric6DofSpringConstraint*) m_btConstraint;
        btTransform transA, transB;
        transA = m_btConstraint->getRigidBodyA().getCenterOfMassTransform();
        transB = m_btConstraint->getRigidBodyB().getCenterOfMassTransform();
        const btTransform tAINW = transA * springConstraint->getFrameOffsetA();
        const btTransform tBINW = transB * springConstraint->getFrameOffsetB();
        const btVector3 deltaPivot = tBINW.getOrigin() - tAINW.getOrigin();
        btScalar angle = deltaPivot.dot(tAINW.getBasis().getColumn(2));
        jntPos = 1.0 * angle; // Using the -1.0 since we always use bodyA as reference frame
    }
    else if (m_jointType == afJointType::TORSION_SPRING){
        // Adapted from btHingeConstraint with slight modifications for spring constraint
        btGeneric6DofSpringConstraint* springConstraint = (btGeneric6DofSpringConstraint*) m_btConstraint;
        btTransform transA, transB;
        transA = m_btConstraint->getRigidBodyA().getCenterOfMassTransform();
        transB = m_btConstraint->getRigidBodyB().getCenterOfMassTransform();
        const btVector3 refAxis0 = transA.getBasis() * springConstraint->getFrameOffsetA().getBasis().getColumn(0);
        const btVector3 refAxis1 = transA.getBasis() * springConstraint->getFrameOffsetA().getBasis().getColumn(1);
        const btVector3 swingAxis = transB.getBasis() * springConstraint->getFrameOffsetB().getBasis().getColumn(1);
        //	btScalar angle = btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
        btScalar angle = btAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
        jntPos = -1.0 * angle; // Using the -1.0 since we always use bodyA as reference frame
    }
    return jntPos;
}


///
/// \brief afJoint::getVelocity
/// \return
///
double afJoint::getVelocity(){
    // TODO: Implement higher order discrete velocity computation methods
    double p_a = m_posArray[m_jpSize - 1];
    double p_b = m_posArray[m_jpSize - 2];
    double dt_n = m_dtArray[m_jpSize - 1];
    double vel ;
    if (dt_n < m_controller.m_min_dt){
        vel = 0.0;
    }
    else{
        vel = (p_a - p_b) / dt_n;
    }
    return vel;
}


///
/// \brief afJoint::getEffort
/// \return
///
double afJoint::getEffort(){
    // Only supported if the joint feedback is enabled in ADF.
    // Only supports single DOF joints such as rev, pris, linear and torsion springs.
    return m_estimatedEffort;
}


///
/// \brief afSensor::afSensor
/// \param a_afWorld
///
afSensor::afSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::SENSOR, a_afWorld, a_modelPtr){
}


///
/// \brief afSensor::updatePositionFromDynamics
///
void afSensor::update(double dt){

}


///
/// \brief afRayTracerSensor
/// \param a_afWorld
///
afRayTracerSensor::afRayTracerSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afSensor(a_afWorld, a_modelPtr){

}


void afRayTracerResult::enableVisualization(afRayTracerSensor* sensorPtr, const afRayAttributes* attribs, double sphereRadius){
    if (m_hitSphereMesh == nullptr){
        cMesh* mesh = new cMesh();
        cCreateSphere(mesh, sphereRadius);
        mesh->m_material->setPinkHot();
        mesh->setShowEnabled(false);
        mesh->setUseDisplayList(true);
        mesh->markForUpdate(false);
        m_hitSphereMesh = mesh;
        sensorPtr->m_afWorld->addSceneObjectToWorld(mesh);
    }

    if (m_fromSphereMesh == nullptr){
        cMesh* mesh = new cMesh();
        cCreateSphere(mesh, sphereRadius);
        mesh->m_material->setRed();
        mesh->setShowEnabled(false);
        mesh->setUseDisplayList(true);
        mesh->markForUpdate(false);
        m_fromSphereMesh = mesh;
        cTransform offsetTrans;
        offsetTrans.setLocalPos(to_cVector3d(attribs->m_rayFromLocal));
        sensorPtr->addChildSceneObject(mesh, offsetTrans);
        sensorPtr->m_afWorld->addSceneObjectToWorld(mesh);
    }

    if (m_toSphereMesh == nullptr){
        cMesh* mesh = new cMesh();
        cCreateSphere(mesh, sphereRadius);
        mesh->m_material->setGreen();
        mesh->setShowEnabled(false);
        mesh->setUseDisplayList(true);
        mesh->markForUpdate(false);
        m_toSphereMesh = mesh;
        cTransform offsetTrans;
        offsetTrans.setLocalPos(to_cVector3d(attribs->m_rayToLocal));
        sensorPtr->addChildSceneObject(mesh, offsetTrans);
        sensorPtr->m_afWorld->addSceneObjectToWorld(mesh);
    }

    if (m_hitNormalMesh == nullptr){
        cMesh* mesh = new cMesh();
        cCreateArrow(mesh, sphereRadius*10,
                     sphereRadius*0.5,
                     sphereRadius*1,
                     sphereRadius*0.8,
                     false);
        mesh->m_material->setGreenForest();
        mesh->setShowEnabled(false);
        mesh->setUseDisplayList(true);
        mesh->markForUpdate(false);
        m_hitNormalMesh = mesh;
        sensorPtr->m_afWorld->addSceneObjectToWorld(mesh);
    }
}


bool afRayTracerSensor::createFromAttribs(afRayTracerSensorAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afRayTracerSensorAttributes &attribs = *a_attribs;

    bool result = true;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;
    m_localTransform << a_attribs->m_kinematicAttribs.m_location;

    m_range = a_attribs->m_range;

    if (m_range < 0.0){
        cerr << "ERROR! SENSOR RANGE CANNOT BE NEGATIVE" << endl;
        return 0;
    }

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    setVisibleFlag(a_attribs->m_visible);
    m_visibilitySphereRadius = a_attribs->m_visibleSize;

    // First search in the local space.
    m_parentBody = m_modelPtr->getRigidBody(m_parentName, true);

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getSensorMap());
    setGlobalRemapIdx(remap_idx);

    if(m_parentBody == nullptr){
        m_parentBody = m_afWorld->getRigidBody(m_parentName + getGlobalRemapIdx());
        if (m_parentBody == nullptr){
            cerr << "ERROR! SENSOR'S "<< m_parentName + remap_idx << " NOT FOUND, IGNORING SENSOR\n";
            return 0;
        }
    }

    m_parentBody->addSensor(this);
    m_parentBody->addChildObject(this);

    switch (a_attribs->m_specificationType) {
    case afSensactorSpecificationType::ARRAY:
    case afSensactorSpecificationType::PARAMETRIC:{
        m_count = a_attribs->m_raysAttribs.size();
        m_raysAttribs = a_attribs->m_raysAttribs;
        m_rayTracerResults.resize(m_count);
        break;
    }
    case afSensactorSpecificationType::MESH:{
        cMultiMesh* contourMesh = new cMultiMesh();
        if (contourMesh->loadFromFile(a_attribs->m_contourMeshFilepath.c_str())){
            m_raysAttribs = afShapeUtils::createRayAttribs(contourMesh, m_range);
            delete contourMesh;
            result = true;
        }
        else{
            cerr << "ERROR! BODY \"" << m_name << "\'s\" RESISTIVE MESH " <<
                    a_attribs->m_contourMeshFilepath.c_str() << " NOT FOUND. IGNORING\n";
            result = false;
        }
        break;
    }
    default:
        break;
    }

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    return result;
}


///
/// \brief afRayTracerSensor::visualize
///
void afRayTracerSensor::enableVisualization(){
    for (uint i = 0 ; i < m_count ; i++){
        m_rayTracerResults[i].enableVisualization(this, &m_raysAttribs[i], m_visibilitySphereRadius);
    }
    m_visualizationEnabled = true;
}

void afRayTracerSensor::visualize(bool show)
{
    if (m_visualizationEnabled == false){
        enableVisualization();
    }

    for (uint i = 0 ; i < m_count ; i++){
        m_rayTracerResults[i].m_fromSphereMesh->setShowEnabled(show);
        m_rayTracerResults[i].m_hitSphereMesh->setShowEnabled(show && m_rayTracerResults[i].m_triggered);
        m_rayTracerResults[i].m_toSphereMesh->setShowEnabled(show);
    }
}

///
/// \brief afRayTracerSensor::updatePositionFromDynamics
///
void afRayTracerSensor::update(double dt){
    if (m_parentBody == nullptr){
        return;
    }

    btTransform T_bINw = m_parentBody->getCOMTransform() * to_btTransform(m_localTransform);
    for (uint i = 0 ; i < m_count ; i++){
        btVector3 rayFromWorld, rayToWorld;
        rayFromWorld = T_bINw * to_btVector(m_raysAttribs[i].m_rayFromLocal);
        rayToWorld = T_bINw * to_btVector(m_raysAttribs[i].m_rayToLocal);

        btCollisionWorld::ClosestRayResultCallback rayCallBack(rayFromWorld, rayToWorld);
        m_afWorld->m_bulletWorld->rayTest(rayFromWorld, rayToWorld, rayCallBack);
        if (rayCallBack.hasHit()){
            if (m_visualizationEnabled){
                cVector3d Ph;
                Ph << rayCallBack.m_hitPointWorld;
                m_rayTracerResults[i].m_hitSphereMesh->setLocalPos(Ph);
            }
            m_rayTracerResults[i].m_triggered = true;
            if (rayCallBack.m_collisionObject->getInternalType()
                    == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
                m_rayTracerResults[i].m_sensedBTRigidBody = (btRigidBody*)btRigidBody::upcast(rayCallBack.m_collisionObject);
                m_rayTracerResults[i].m_sensedRigidBody = m_afWorld->getRigidBody(m_rayTracerResults[i].m_sensedBTRigidBody);
                m_rayTracerResults[i].m_sensedBodyType = afBodyType::RIGID_BODY;
            }
            else if (rayCallBack.m_collisionObject->getInternalType()
                     == btCollisionObject::CollisionObjectTypes::CO_SOFT_BODY){
                btSoftBody* sensedSoftBody = (btSoftBody*)btSoftBody::upcast(rayCallBack.m_collisionObject);

                // Now get the node which is closest to the hit point;
                btVector3 hitPoint = rayCallBack.m_hitPointWorld;
                int sensedSoftBodyNodeIdx = -1;
                int sensedSoftBodyFaceIdx = -1;

                double maxDistance = 0.1;
                for (int faceIdx = 0 ; faceIdx < sensedSoftBody->m_faces.size() ; faceIdx++){
                    btVector3 faceCenter(0, 0, 0);
                    // Iterate over all the three nodes of the face to find the this centroid to the hit
                    // point in world to store this face as the closest face
                    for (int nIdx = 0 ; nIdx < 3 ; nIdx++){
                        faceCenter += sensedSoftBody->m_faces[faceIdx].m_n[nIdx]->m_x;
                    }
                    faceCenter /= 3;
                    if ( (hitPoint - faceCenter).length() < maxDistance ){
                        sensedSoftBodyFaceIdx = faceIdx;
                        maxDistance = (hitPoint - faceCenter).length();
                    }

                }
                // If sensedBodyFaceIdx is not -1, we sensed some face. Lets capture it
                if (sensedSoftBodyFaceIdx > -1){
                    m_rayTracerResults[i].m_sensedSoftBodyFaceIdx = sensedSoftBodyFaceIdx;
                    m_rayTracerResults[i].m_sensedSoftBodyFace = &sensedSoftBody->m_faces[sensedSoftBodyFaceIdx];
                    m_rayTracerResults[i].m_sensedBTSoftBody = sensedSoftBody;
                    m_rayTracerResults[i].m_sensedSoftBody = m_afWorld->getSoftBody(m_rayTracerResults[i].m_sensedBTSoftBody);
                    m_rayTracerResults[i].m_sensedBodyType = afBodyType::SOFT_BODY;
                }
                // Reset the maxDistance for node checking
                maxDistance = 0.1;
                // Iterate over all the softbody nodes to figure out which node is closest to the
                // hit point in world
                for (int nodeIdx = 0 ; nodeIdx < sensedSoftBody->m_nodes.size() ; nodeIdx++){
                    if ( (hitPoint - sensedSoftBody->m_nodes[nodeIdx].m_x).length() < maxDistance ){
                        sensedSoftBodyNodeIdx = nodeIdx;
                        maxDistance = (hitPoint - sensedSoftBody->m_nodes[nodeIdx].m_x).length();
                    }
                }
                // If sensedBodyNodeIdx is not -1, we sensed some node. Lets capture it
                if (sensedSoftBodyNodeIdx > -1){
                    m_rayTracerResults[i].m_sensedSoftBodyNodeIdx = sensedSoftBodyNodeIdx;
                    m_rayTracerResults[i].m_sensedSoftBodyNode = &sensedSoftBody->m_nodes[sensedSoftBodyNodeIdx];
                    m_rayTracerResults[i].m_sensedBTSoftBody = sensedSoftBody;
                    m_rayTracerResults[i].m_sensedBodyType = afBodyType::SOFT_BODY;
                }
            }
            m_rayTracerResults[i].m_depthFraction = (1.0 - rayCallBack.m_closestHitFraction);
            m_rayTracerResults[i].m_contactNormal << rayCallBack.m_hitNormalWorld;
            m_rayTracerResults[i].m_sensedLocationWorld << rayCallBack.m_hitPointWorld;
        }
        else{
            m_rayTracerResults[i].m_triggered = false;
            m_rayTracerResults[i].m_depthFraction = 0;
        }

        visualize(getVisibleFlag());
    }
}

void afRayTracerSensor::setRayFromInLocal(const cVector3d &a_rayFrom, uint idx){
    m_raysAttribs[idx].m_rayFromLocal.set(a_rayFrom.x(), a_rayFrom.y(), a_rayFrom.z());
}

void afRayTracerSensor::setRayToInLocal(const cVector3d &a_rayTo, uint idx){
    m_raysAttribs[idx].m_rayToLocal.set(a_rayTo.x(), a_rayTo.y(), a_rayTo.z());
}

void afRayTracerSensor::setDirection(const cVector3d &a_direction, uint idx){
    m_raysAttribs[idx].m_direction.set(a_direction.x(), a_direction.y(), a_direction.z());
}


///
/// \brief afProximitySensor::afProximitySensor
/// \param a_afWorld
///
afProximitySensor::afProximitySensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afRayTracerSensor(a_afWorld, a_modelPtr){
    m_afWorld = a_afWorld;
    m_sensorType = afSensorType::RAYTRACER;
}


///
/// \brief afResistanceSensor::afResistanceSensor
/// \param a_afWorld
///
afResistanceSensor::afResistanceSensor(afWorld* a_afWorld, afModelPtr a_modelPtr): afRayTracerSensor(a_afWorld, a_modelPtr){
    m_lastContactPosInWorld.set(0,0,0);
    m_curContactPosInWorld.set(0,0,0);
    m_staticContactFriction = 0;
    m_dynamicFriction = 0;

    m_contactArea = 0.1;
    m_staticContactDamping = 0.1;

    m_contactNormalStiffness = 0;
    m_contactNormalDamping = 0;

    m_sensorType = afSensorType::RESISTANCE;
}


bool afResistanceSensor::createFromAttribs(afResistanceSensorAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afResistanceSensorAttributes &attribs = *a_attribs;

    bool result = false;
    // Stop the communication instance from loading.
    bool temp_passive_store = a_attribs->m_communicationAttribs.m_passive;
    a_attribs->m_communicationAttribs.m_passive = true;
    result = afRayTracerSensor::createFromAttribs(&attribs);

    if (result){

        a_attribs->m_communicationAttribs.m_passive = temp_passive_store;

        setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
        setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
        setPassive(a_attribs->m_communicationAttribs.m_passive);

        m_staticContactFriction = a_attribs->m_staticContactFriction;

        m_staticContactDamping = a_attribs->m_staticContactDamping;

        m_dynamicFriction = a_attribs->m_dynamicFriction;

        m_useVariableCoeff = a_attribs->m_useVariableCoeff;

        m_contactArea = a_attribs->m_contactArea;
        m_contactNormalStiffness = a_attribs->m_contactNormalStiffness;

        m_contactNormalDamping = a_attribs->m_contactNormalDamping;

        m_rayContactResults.resize(m_count);

        for (uint i = 0 ; i < m_count ; i++){
            m_rayContactResults[i].m_bodyAContactPointLocal.set(0,0,0);
            m_rayContactResults[i].m_bodyBContactPointLocal.set(0,0,0);

            m_rayContactResults[i].m_tangentialError.set(0,0,0);
            m_rayContactResults[i].m_tangentialErrorLast.set(0,0,0);

            m_rayContactResults[i].m_contactPointsValid = false;

        }

    }

    return result;
}


///
/// \brief afResistanceSensor::updatePositionFromDynamics
///
void afResistanceSensor::update(double dt){
    // Let's update the RayTracer Sensor First
    afRayTracerSensor::update(dt);

    if (m_parentBody == nullptr){
        return;
    }

    for (uint i = 0 ; i < m_count ; i++){

        if (isTriggered(i)){
            if (getVisibleFlag()){
                m_rayTracerResults[i].m_hitNormalMesh->setLocalPos(getSensedPoint(i));
                m_rayTracerResults[i].m_hitNormalMesh->setLocalRot(afUtils::getRotBetweenVectors<cMatrix3d,
                                                                   cVector3d>(cVector3d(0,0,1), m_rayTracerResults[i].m_contactNormal));
                m_rayTracerResults[i].m_hitNormalMesh->setShowEnabled(true);
            }

            btVector3 F_s_w(0,0,0); // Due to "stick" friction
            btVector3 F_d_w(0,0,0); // Due to "sliding" friction
            btVector3 F_n_w(0,0,0); // Force normal to contact point.

            if (getSensedBodyType(i) == afBodyType::RIGID_BODY){

                // Get the fraction of contact point penetration from the range of the sensor
                // Subscript (a) represents parent body, which is the parent of this sensor
                // Subscript (b) represents the sensed body, which is in contact with the resistive sensor
                // Subscript (c) represents contact point
                // Subscript (w) represents world
                btTransform T_aINw = getParentBody()->m_bulletRigidBody->getWorldTransform();
                btTransform T_wINa = T_aINw.inverse(); // Invert once to save computation later
                btVector3 P_cINw = to_btVector(getSensedPoint(i));
                btVector3 P_cINa = T_wINa * P_cINw;
                btVector3 vel_aINw = getParentBody()->m_bulletRigidBody->getLinearVelocity();
                btVector3 omega_aINw = getParentBody()->m_bulletRigidBody->getAngularVelocity();
                btVector3 N_a = to_btVector(m_raysAttribs[i].m_direction);
                btVector3 N_aINw = T_aINw.getBasis() * N_a;

                btTransform T_bINw = getSensedBTRigidBody(i)->getWorldTransform();
                btTransform T_wINb = T_bINw.inverse(); // Invert once to save computation later
                btVector3 P_cINb = T_wINb * P_cINw;
                btVector3 vel_bINw = getSensedBTRigidBody(i)->getLinearVelocity();
                btVector3 omega_bINw = getSensedBTRigidBody(i)->getAngularVelocity();
                btVector3 N_bINw = to_btVector(m_rayTracerResults[i].m_contactNormal);
                btVector3 N_b = T_wINb.getBasis() * N_bINw;

                double depthFractionLast = m_rayTracerResults[i].m_depthFraction;

                if (m_rayTracerResults[i].m_depthFraction < 0 || m_rayTracerResults[i].m_depthFraction > 1){
                    cerr << "LOGIC ERROR! "<< m_name <<" Depth Fraction is " << m_rayTracerResults[i].m_depthFraction <<
                            ". It should be between [0-1]" << endl;
                    cVector3d rayF, rayT, Pc_a;
                    rayF << m_raysAttribs[i].m_rayFromLocal;
                    rayT << m_raysAttribs[i].m_rayToLocal;
                    Pc_a << P_cINa;
                    cerr << "Ray Start: "<< rayF <<"\nRay End: " << rayT <<
                            "\nSensed Point: " << Pc_a << endl;
                    cerr << "----------\n";
                    m_rayTracerResults[i].m_depthFraction = 0;
                }

                // First calculate the normal contact force
                btVector3 F_n_a = ((m_contactNormalStiffness * m_rayTracerResults[i].m_depthFraction)
                                   + m_contactNormalDamping * (m_rayTracerResults[i].m_depthFraction - depthFractionLast)) * (N_a);
                F_n_w = T_aINw.getBasis() * F_n_a;

                double coeffScale = 1;
                if (m_useVariableCoeff){
                    coeffScale = F_n_w.length();
                }

                if(m_rayContactResults[i].m_contactPointsValid){
                    btVector3 P_aINw = T_aINw * to_btVector(m_rayContactResults[i].m_bodyAContactPointLocal);
                    btVector3 P_bINw = T_bINw * to_btVector(m_rayContactResults[i].m_bodyBContactPointLocal);
                    btVector3 error;
                    error = P_aINw - P_bINw;
                    btVector3 orthogonalError = N_aINw.cross(error);
                    btVector3 errorDir = orthogonalError.cross(N_aINw);
                    if (errorDir.length() > 0.0001){
                        errorDir.normalize();
                    }
                    double errorMag = errorDir.dot(error);
                    if (errorMag < 0.0){
                        cerr << errorMag << endl;
                    }

                    btVector3 tangentialError = errorMag * errorDir;
                    btVector3 tangentialErrorLast = to_btVector(m_rayContactResults[i].m_tangentialErrorLast);
                    m_rayContactResults[i].m_tangentialErrorLast = m_rayContactResults[i].m_tangentialError;
                    m_rayContactResults[i].m_tangentialError << tangentialError;

                    if (tangentialError.length() > 0.0 && tangentialError.length() <= m_contactArea){
                        F_s_w = m_staticContactFriction * coeffScale * tangentialError +
                                m_staticContactDamping * (tangentialError - tangentialErrorLast);
                    }
                    else{
                        m_rayContactResults[i].m_contactPointsValid = false;
                    }

                    //                cerr << "F Static: " << F_static << endl;
                    //                cerr << "F Normal: " << F_normal << endl;
                    //                cerr << "Depth Ra: " << m_depthFraction << endl;
                    //                cerr << "------------\n";
                }
                else{
                    cVector3d Pa, Pb;
                    Pa << T_wINa * to_btVector(getSensedPoint(i));
                    Pb << T_wINb * to_btVector(getSensedPoint(i));
                    m_rayContactResults[i].m_bodyAContactPointLocal = Pa;
                    m_rayContactResults[i].m_bodyBContactPointLocal = Pb;
                    m_rayContactResults[i].m_contactPointsValid = true;
                }

                // Calculate the friction due to sliding velocities
                // Get velocity of point
                btVector3 vel_a = T_wINa.getBasis() * vel_aINw;
                btVector3 omega_a = T_wINa.getBasis() * omega_aINw;
                btVector3 vel_cINa = vel_a + omega_a.cross(P_cINa);

                btVector3 vel_b = T_wINb.getBasis() * vel_bINw;
                btVector3 omega_b = T_wINb.getBasis() * omega_bINw;
                btVector3 vel_cINb = vel_b + omega_b.cross(P_cINb);

                btVector3 V_aINw = T_aINw.getBasis() * vel_cINa;
                btVector3 V_bINw = T_bINw.getBasis() * vel_cINb;

                btVector3 dV = V_aINw - V_bINw;

                // Check if the error is along the direction of sensor
                btVector3 orthogonalVelError = N_bINw.cross(dV);
                btVector3 velErrorDir = orthogonalVelError.cross(N_bINw);
                if (velErrorDir.length() > 0.0001){
                    velErrorDir.normalize();
                }
                dV = velErrorDir.dot(dV) * velErrorDir;

                F_d_w = m_dynamicFriction * coeffScale * dV;
                //                cerr << staticForce << endl;

                btVector3 Fw = F_s_w + F_d_w + F_n_w;

                // Lets find the added torque at the point where the force is applied

                btVector3 Fa = T_wINa.getBasis() * (-Fw);
                btVector3 Tau_a = P_cINa.cross(Fa * getParentBody()->m_bulletRigidBody->getLinearFactor());
                btVector3 Tau_aINw = T_aINw.getBasis() * Tau_a;

                btVector3 Fb = T_wINb.getBasis() * Fw;
                btVector3 Tau_b = P_cINb.cross(Fb * getSensedBTRigidBody(i)->getLinearFactor());
                btVector3 Tau_bINw = T_bINw.getBasis() * Tau_b;

                // Nows lets add the action and reaction friction forces to both the bodies
                getParentBody()->m_bulletRigidBody->applyCentralForce(-Fw);
                getParentBody()->m_bulletRigidBody->applyTorque(Tau_aINw);

                getSensedBTRigidBody(i)->applyCentralForce(Fw);
                getSensedBTRigidBody(i)->applyTorque(Tau_bINw);
            }

            else if (getSensedBodyType(i) == afBodyType::SOFT_BODY){

            }
        }
        else{
            m_rayContactResults[i].m_contactPointsValid = false;
            m_rayContactResults[i].m_firstTrigger = true;

            if(getVisibleFlag()){
                m_rayTracerResults[i].m_hitNormalMesh->setShowEnabled(false);
            }
        }
    }
}


///
/// \brief afJoint::~afJoint
///
afJoint::~afJoint(){
    if (m_btConstraint != nullptr){
        delete m_btConstraint;
    }

    if (m_feedback != nullptr){
        delete m_feedback;
    }
}


///
/// \brief afObjectManager::checkIfExists
/// \param a_obj
/// \param a_objectsVec
/// \return
///
bool afObjectManager::checkIfExists(afBaseObject *a_obj, vector<afBaseObject*> *a_objectsVec)
{
    vector<afBaseObject*>::iterator it;
    for (it = a_objectsVec->begin() ; it != a_objectsVec->end() ; ++it){
        if ((*it) == a_obj){
            return true;
        }
    }

    return false;
}


///
/// \brief afObjectManager::addObjectMissingParent
/// \param a_obj
///
void afObjectManager::addObjectMissingParent(afBaseObjectPtr a_obj)
{
    if (!checkIfExists(a_obj, &m_afObjectsMissingParents)){
        m_afObjectsMissingParents.push_back(a_obj);
    }
}


///
/// \brief afObjectManager::resolveObjectsMissingParents
/// \param a_newObject
///
void afObjectManager::resolveObjectsMissingParents(afBaseObjectPtr a_newObject)
{
    vector<afBaseObject*> stillMissingParents;
    for (vector<afBaseObject*>::iterator it = m_afObjectsMissingParents.begin() ; it != m_afObjectsMissingParents.end() ; ++it){
        afBaseObject *needParenting = *it;
        bool parentFound = false;
        if ( needParenting == a_newObject ){
            // If the newly added object is the object in question, then search all previously added objects
            parentFound = (*it)->resolveParent( (*it)->m_parentName, true);
        }

        if (parentFound == false){
            // Else, check if we get an exact name match
            if ( needParenting->m_parentName.compare( a_newObject->getQualifiedIdentifier() ) == 0 ){
                a_newObject->addChildObject((*it));
            }
            // Else, check if part of the name matches
            else if( a_newObject->getQualifiedIdentifier().find((*it)->m_parentName) != string::npos){
                if (needParenting == a_newObject ){
                    stillMissingParents.push_back((*it));
                }
                else{
                    if (needParenting->m_parentName.compare(a_newObject->getQualifiedIdentifier()) != 0){
                        cerr << "WARNING! Required parent name of "<< needParenting->getQualifiedName() << " set as " <<
                                needParenting->m_parentName << ". Parenting to closest match " << a_newObject->getQualifiedIdentifier() << endl;
                    }
                    a_newObject->addChildObject((*it));
                }
            }
            // Tried our best, keep this as an object missing parents, so that we can try again on the next object add.
            else{
                stillMissingParents.push_back((*it));
            }
        }
    }
    m_afObjectsMissingParents = stillMissingParents;
}


///
/// \brief afObjectManager::addBaseObject
/// \param a_obj
/// \param a_name
/// \return
///
bool afObjectManager::addBaseObject(afBaseObjectPtr a_obj, string a_name){
    if (a_obj->getType() == afType::INVALID){
        cerr << "ERROR! CANNOT ADD OBJECT TO WORLD WITH OBJECT_TYPE AS INVALID " << endl;
        return false;
    }
    (m_childrenObjectsMap[a_obj->getType()])[a_name] = a_obj;

    a_obj->calculateFrameSize();
    // Whenever a new object is added, try to resolve parenting of objects that require parenting.
    resolveObjectsMissingParents(a_obj);
    return true;
}


///
/// \brief afObjectManager::getBaseObject
/// \param a_name
/// \param objMap
/// \param suppress_warning
/// \return
///
afBaseObjectPtr afObjectManager::getBaseObject(string a_name, afBaseObjectMap* objMap, bool suppress_warning){
    if (objMap->find(a_name) != objMap->end()){
        return ((*objMap)[a_name]);
    }
    // We didn't find the object using the full name, try checking if the name is a substring of the fully qualified name
    int matching_obj_count = 0;
    vector<string> matching_obj_names;
    afBaseObjectPtr objHandle;
    afBaseObjectMap::iterator oIt = objMap->begin();
    for (; oIt != objMap->end() ; ++oIt){
        if (oIt->first.find(a_name) != string::npos){
            matching_obj_count++;
            matching_obj_names.push_back(oIt->first);
            objHandle = oIt->second;
        }
    }

    if (matching_obj_count == 1){
        // If only one object is found, return that object
        return objHandle;
    }
    else if(matching_obj_count > 1 && !suppress_warning){
        cerr << "WARNING! MULTIPLE OBJECTS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < matching_obj_names.size() ; i++){
            cerr << "\t" << i << ") " << matching_obj_names[i] << endl;
        }
        return nullptr;
    }
    else{
        if (!suppress_warning){
            cerr << "WARNING! CAN'T FIND ANY OBJECTS NAMED: \"" << a_name << "\" IN GLOBAL MAP \n";

            cerr <<"Existing OBJECTS in Map: " << objMap->size() << endl;
            afBaseObjectMap::iterator oIt = objMap->begin();
            for (; oIt != objMap->end() ; ++oIt){
                cerr << oIt->first << endl;
            }
        }
        return nullptr;
    }
}


template <class T>
///
/// \brief afObjectManager::getBaseObjects
/// \param objMap
/// \return
///
vector<T*> afObjectManager::getBaseObjects(afBaseObjectMap* objMap){
    vector<T*> objects;
    afBaseObjectMap::iterator oIt;

    for (oIt = objMap->begin() ; oIt != objMap->end() ; ++oIt){
        objects.push_back((T*)oIt->second);
    }

    return objects;
}


///
/// \brief afObjectManager::getLight
/// \param a_name
/// \param suppress_warning
/// \return
///
afLightPtr afObjectManager::getLight(string a_name, bool suppress_warning){
    return (afLightPtr)getBaseObject(a_name, getLightMap(), suppress_warning);
}


///
/// \brief afObjectManager::getCamera
/// \param a_name
/// \param suppress_warning
/// \return
///
afCameraPtr afObjectManager::getCamera(string a_name, bool suppress_warning){
    return (afCameraPtr)getBaseObject(a_name, getCameraMap(), suppress_warning);
}


///
/// \brief afObjectManager::getRigidBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afRigidBodyPtr afObjectManager::getRigidBody(string a_name, bool suppress_warning){
    return (afRigidBodyPtr)getBaseObject(a_name, getRigidBodyMap(), suppress_warning);
}


///
/// \brief afObjectManager::getRigidBody
/// \param a_body
/// \param suppress_warning
/// \return
///
afRigidBodyPtr afObjectManager::getRigidBody(btRigidBody* a_body, bool suppress_warning){
    afRigidBodyPtr rBody = nullptr;
    if (a_body->getUserPointer() == nullptr){
        if (!suppress_warning){
            cerr << "WARNING! CAN'T FIND ANY AF RIGID BODY BOUND TO BULLET RIGID BODY: \"" << a_body << "\"\n";
            cerr <<"Existing Bodies in Map: " << getRigidBodyMap()->size() << endl;
            afBaseObjectMap::iterator rbIt = getRigidBodyMap()->begin();
            for (; rbIt != getRigidBodyMap()->end() ; ++rbIt){
                cerr << rbIt->first << endl;
            }
        }
    }
    else{
        rBody = (afRigidBodyPtr) a_body->getUserPointer();
    }
    return rBody;
}


///
/// \brief afObjectManager::getRootRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afObjectManager::getRootRigidBody(afRigidBodyPtr a_bodyPtr){
    /// Find Root Body
    afRigidBodyPtr rootParentBody = nullptr;
    vector<int> bodyParentsCount;
    size_t rootParents = 0;
    if (a_bodyPtr){
        if (a_bodyPtr->m_parentBodies.size() == 0){
            rootParentBody = a_bodyPtr;
            rootParents++;
        }
        else{
            bodyParentsCount.resize(a_bodyPtr->m_parentBodies.size());
            vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
            for (int parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
                if ((*rIt)->m_parentBodies.size() == 0){
                    rootParentBody = (*rIt);
                    rootParents++;
                }
                bodyParentsCount[parentNum] = (*rIt)->m_parentBodies.size();
            }
        }
    }
    else{
        bodyParentsCount.resize(getRigidBodyMap()->size());
        afBaseObjectMap::const_iterator mIt = getRigidBodyMap()->begin();
        for(int bodyNum=0; mIt != getRigidBodyMap()->end() ; bodyNum++, ++mIt){
            afRigidBodyPtr rb = (afRigidBodyPtr)((*mIt).second);
            if (rb->m_parentBodies.size() == 0){
                rootParentBody = rb;
                ++rootParents;
            }
            bodyParentsCount[bodyNum] = rb->m_parentBodies.size();
        }

    }

    if (rootParents > 1){
        cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";
    }

    return rootParentBody;
}

///
/// \brief afObjectManager::getJoint
/// \param a_name
/// \param suppress_warning
/// \return
///
afJointPtr afObjectManager::getJoint(string a_name, bool suppress_warning){
    return (afJointPtr)getBaseObject(a_name, getJointMap(), suppress_warning);
}


///
/// \brief afObjectManager::getActuator
/// \param a_name
/// \param suppress_warning
/// \return
///
afActuatorPtr afObjectManager::getActuator(string a_name, bool suppress_warning){
    return (afActuatorPtr)getBaseObject(a_name, getActuatorMap(), suppress_warning);
}

///
/// \brief afObjectManager::getSensor
/// \param a_name
/// \param suppress_warning
/// \return
///
afSensorPtr afObjectManager::getSensor(string a_name, bool suppress_warning){
    return (afSensorPtr)getBaseObject(a_name, getSensorMap(), suppress_warning);

}


///
/// \brief afObjectManager::getSoftBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afSoftBodyPtr afObjectManager::getSoftBody(string a_name, bool suppress_warning){
    return (afSoftBodyPtr)getBaseObject(a_name, getSoftBodyMap(), suppress_warning);
}


///
/// \brief afObjectManager::getSoftBody
/// \param a_body
/// \param suppress_warning
/// \return
///
afSoftBodyPtr afObjectManager::getSoftBody(btSoftBody* a_body, bool suppress_warning){
    afSoftBodyPtr sBody = nullptr;
    if (a_body->getUserPointer() == nullptr){
        if (!suppress_warning){
            cerr << "WARNING! CAN'T FIND ANY AF SOFT BODY BOUND TO BULLET SOFT BODY: \"" << a_body << "\"\n";
            cerr << "Existing Bodies in Map: " << getSoftBodyMap()->size() << endl;
            afBaseObjectMap::iterator rbIt = getSoftBodyMap()->begin();
            for (; rbIt != getSoftBodyMap()->end() ; ++rbIt){
                cerr << rbIt->first << endl;
            }
        }
    }
    else{
        sBody = (afSoftBodyPtr) a_body->getUserPointer();
    }
    return sBody;
}


///
/// \brief afObjectManager::getGhostObject
/// \param a_name
/// \param suppress_warning
/// \return
///
afGhostObjectPtr afObjectManager::getGhostObject(string a_name, bool suppress_warning)
{
    return (afGhostObjectPtr)getBaseObject(a_name, getGhostObjectMap(), suppress_warning);
}


///
/// \brief afObjectManager::getGhostObject
/// \param a_body
/// \param suppress_warning
/// \return
///
afGhostObjectPtr afObjectManager::getGhostObject(btGhostObject *a_body, bool suppress_warning){
    afGhostObjectPtr ghostObj = nullptr;
    if (a_body->getUserPointer() == nullptr){
        if (!suppress_warning){
            cerr << "WARNING! CAN'T FIND ANY AF GHOST OBJECT BOUND TO BULLET GHOST OBJECT: \"" << a_body << "\"\n";
            cerr << "Existing Bodies in Map: " << getGhostObjectMap()->size() << endl;
            afBaseObjectMap::iterator rbIt = getGhostObjectMap()->begin();
            for (; rbIt != getGhostObjectMap()->end() ; ++rbIt){
                cerr << rbIt->first << endl;
            }
        }
    }
    else{
        ghostObj = (afGhostObjectPtr) a_body->getUserPointer();
    }
    return ghostObj;
}

afBaseObjectPtr afObjectManager::getBaseObject(string a_name, bool suppress_warning)
{
    vector<afBaseObjectPtr> foundObjs;
    afChildrenMap::iterator cmIt;
    for (cmIt = m_childrenObjectsMap.begin() ; cmIt != m_childrenObjectsMap.end() ; ++cmIt){
        afBaseObjectMap typedObjMap = cmIt->second;
        afBaseObjectPtr obj = getBaseObject(a_name, &typedObjMap, suppress_warning);
        if (obj){
            foundObjs.push_back(obj);
        }
    }

    if (foundObjs.size() == 1){
        return foundObjs[0];
    }

//    if (!suppress_warning){
        cerr << "WARNING! MULTIPLE OBJECTS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < foundObjs.size() ; i++){
            cerr << "\t" << i << ") " << foundObjs[i]->getQualifiedIdentifier() << ", Object Type " << foundObjs[i]->getTypeAsStr() << endl;
        }
        return nullptr;
//    }
}


///
/// \brief afObjectManager::getVolume
/// \param a_name
/// \param suppress_warning
/// \return
///
afVolumePtr afObjectManager::getVolume(string a_name, bool suppress_warning)
{
    return (afVolumePtr)getBaseObject(a_name, getVolumeMap(), suppress_warning);
}

///
/// \brief afObjectManager::getVehicle
/// \param a_name
/// \param suppress_warning
/// \return
///
afVehiclePtr afObjectManager::getVehicle(string a_name, bool suppress_warning){
    return (afVehiclePtr)getBaseObject(a_name, getVehicleMap(), suppress_warning);
}


///
/// \brief afObjectManager::addLight
/// \param a_obj
/// \return
///
afObjectManager::afObjectManager(){
}

string afObjectManager::addLight(afLightPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getLightMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addCamera
/// \param a_obj
/// \return
///
string afObjectManager::addCamera(afCameraPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getCameraMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addRigidBody
/// \param a_obj
/// \return
///
string afObjectManager::addRigidBody(afRigidBodyPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getRigidBodyMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addSoftBody
/// \param a_obj
/// \return
///
string afObjectManager::addSoftBody(afSoftBodyPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getSoftBodyMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}


///
/// \brief afObjectManager::addGhostObject
/// \param a_obj
/// \return
///
string afObjectManager::addGhostObject(afGhostObjectPtr a_obj)
{
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getGhostObjectMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addJoint
/// \param a_obj
/// \return
///
string afObjectManager::addJoint(afJointPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getJointMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addActuator
/// \param a_obj
/// \return
///
string afObjectManager::addActuator(afActuatorPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getActuatorMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addSensor
/// \param a_obj
/// \return
///
string afObjectManager::addSensor(afSensorPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getSensorMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}


///
/// \brief afObjectManager::addVehicle
/// \param a_obj
/// \return
///
string afObjectManager::addVehicle(afVehiclePtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getVehicleMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

///
/// \brief afObjectManager::addVolume
/// \param a_volume
/// \return
///
string afObjectManager::addVolume(afVolumePtr a_obj)
{
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getVehicleMap());
    string remaped_identifier = qualified_identifier + remap_str;
    addBaseObject(a_obj, qualified_identifier + remap_str);
    return remaped_identifier;
}

string afObjectManager::addBaseObject(afBaseObjectPtr a_obj)
{
    string remaped_name = "";
    switch (a_obj->getType()) {
    case afType::RIGID_BODY:
        remaped_name = addRigidBody((afRigidBodyPtr)a_obj);
        break;
    case afType::JOINT:
        remaped_name = addJoint((afJointPtr)a_obj);
        break;
    case afType::SOFT_BODY:
        remaped_name = addSoftBody((afSoftBodyPtr)a_obj);
        break;
    case afType::VEHICLE:
        remaped_name = addVehicle((afVehiclePtr)a_obj);
        break;
    case afType::VOLUME:
        remaped_name = addVolume((afVolumePtr)a_obj);
        break;
    case afType::GHOST_OBJECT:
        remaped_name = addGhostObject((afGhostObjectPtr)a_obj);
        break;
    case afType::SENSOR:
        remaped_name = addSensor((afSensorPtr)a_obj);
        break;
    case afType::ACTUATOR:
        remaped_name = addActuator((afActuatorPtr)a_obj);
        break;
    case afType::CAMERA:
        remaped_name = addCamera((afCameraPtr)a_obj);
        break;
    case afType::LIGHT:
        remaped_name = addLight((afLightPtr)a_obj);
        break;
    default:
        cerr << "ERROR! OBJECT " << a_obj->getQualifiedIdentifier() << "'s TYPE HAS NOT BEEN IMPLEMENTED IN OBJECT MANAGER YET!" << endl;
        break;
    }

    return remaped_name;
}


///
/// \brief afObjectManager::getLights
/// \return
///
afLightVec afObjectManager::getLights(){
    return getBaseObjects<afLight>(getLightMap());
}


///
/// \brief afObjectManager::getCameras
/// \return
///
afCameraVec afObjectManager::getCameras(){
    return getBaseObjects<afCamera>(getCameraMap());
}


///
/// \brief afObjectManager::getRigidBodies
/// \return
///
afRigidBodyVec afObjectManager::getRigidBodies(){
    return getBaseObjects<afRigidBody>(getRigidBodyMap());
}


///
/// \brief afObjectManager::getSoftBodies
/// \return
///
afSoftBodyVec afObjectManager::getSoftBodies(){
    return getBaseObjects<afSoftBody>(getSoftBodyMap());
}


///
/// \brief afObjectManager::getGhostObjects
/// \return
///
afGhostObjectVec afObjectManager::getGhostObjects()
{
    return getBaseObjects<afGhostObject>(getGhostObjectMap());
}


///
/// \brief afObjectManager::getJoints
/// \return
///
afJointVec afObjectManager::getJoints(){
    return getBaseObjects<afJoint>(getJointMap());
}


///
/// \brief afObjectManager::getSensors
/// \return
///
afSensorVec afObjectManager::getSensors(){
    return getBaseObjects<afSensor>(getSensorMap());
}


///
/// \brief afObjectManager::getVehicles
/// \return
///
afVehicleVec afObjectManager::getVehicles(){
    return getBaseObjects<afVehicle>(getVehicleMap());
}


///
/// \brief afObjectManager::getVolumes
/// \return
///
afVolumeVec afObjectManager::getVolumes()
{
    return getBaseObjects<afVolume>(getVolumeMap());
}


///
/// \brief afModelManager::getModel
/// \param a_name
/// \param suppress_warning
/// \return
///
afModelPtr afModelManager::getModel(string a_name, bool suppress_warning){
    afModelMap* modelsMap = getModelMap();
    if (modelsMap->find(a_name) != modelsMap->end()){
        return ((*modelsMap)[a_name]);
    }
    // We didn't find the object using the full name, try checking if the name is a substring of the fully qualified name
    int matching_obj_count = 0;
    vector<string> matching_models_names;
    afModelPtr objHandle;
    afModelMap::iterator oIt = modelsMap->begin();
    for (; oIt != modelsMap->end() ; ++oIt){
        if (oIt->first.find(a_name) != string::npos){
            matching_obj_count++;
            matching_models_names.push_back(oIt->first);
            objHandle = oIt->second;
        }
    }

    if (matching_obj_count == 1){
        // If only one object is found, return that object
        return objHandle;
    }
    else if(matching_obj_count > 1){
        cerr << "WARNING! MULTIPLE MODELS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < matching_models_names.size() ; i++){
            cerr << "\t" << i << ") " << matching_models_names[i] << endl;
        }
        return nullptr;
    }
    else{
        if (!suppress_warning){
            cerr << "WARNING! CAN'T FIND ANY MODELS NAMED: \"" << a_name << "\" IN GLOBAL MAP \n";

            cerr <<"Existing MODELS in Map: " << modelsMap->size() << endl;
            afModelMap::iterator oIt = modelsMap->begin();
            for (; oIt != modelsMap->end() ; ++oIt){
                cerr << oIt->first << endl;
            }
        }
        return nullptr;
    }
}


///
/// \brief afModelManager::addModel
/// \param a_obj
/// \return
///
afModelManager::afModelManager(afWorldPtr a_afWorld)
{
    m_afWorld = a_afWorld;
}

string afModelManager::addModel(afModelPtr a_model){
    string qualified_identifier = a_model->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, getModelMap());
    string remaped_identifier = qualified_identifier + remap_str;
    m_modelsMap[remaped_identifier] = a_model;

    // Add models objects to world
    addModelsChildrenToWorld(a_model);

    return remaped_identifier;
}


///
/// \brief afModelManager::getModels
/// \return
///
afModelVec afModelManager::getModels(){
    vector<afModelPtr> models;
    afModelMap::iterator oIt;

    for (oIt = m_modelsMap.begin() ; oIt != m_modelsMap.end() ; ++oIt){
        models.push_back(oIt->second);
    }

    return models;
}

void afModelManager::addModelsChildrenToWorld(afModelPtr a_model)
{
    afChildrenMap::iterator cIt;
    afChildrenMap* childrenMap = a_model->getChildrenMap();

    for(cIt = childrenMap->begin(); cIt != childrenMap->end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            m_afWorld->addBaseObject(childObj);
            m_afWorld->addChildsSceneObjectsToWorld(childObj);
        }
    }
}

void afModelManager::addChildsSceneObjectsToWorld(afBaseObjectPtr a_object)
{
    for(int i = 0 ; i < a_object->m_childrenSceneObjects.size() ; i++){
        m_afWorld->addSceneObjectToWorld(a_object->m_childrenSceneObjects[i]->getChaiObject());
    }
}



///
/// \brief afWorld::afWorld
/// \param a_global_namespace
///
afWorld::afWorld(): afIdentification(afType::WORLD), afModelManager(this){
    m_maxIterations = 10;

    // reset simulation time
    m_simulationTime = 0.0;

    // integration time step
    m_integrationTimeStep = 0.001;

    // maximum number of iterations
    m_integrationMaxIterations = 5;

    // Set the last simulation time to 0
    m_lastSimulationTime = 0.0;

    // setup broad phase collision detection
    m_bulletBroadphase = new btDbvtBroadphase();

    // setup the collision configuration
    m_bulletCollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    // setup the collision dispatcher
    m_bulletCollisionDispatcher = new btCollisionDispatcher(m_bulletCollisionConfiguration);

    // register GIMPACT collision detector for GIMPACT objects
    btGImpactCollisionAlgorithm::registerAlgorithm(m_bulletCollisionDispatcher);

    // setup the actual physics solver
    m_bulletSolver = new btSequentialImpulseConstraintSolver;

    // setup the dynamic world
    m_bulletWorld = new btSoftRigidDynamicsWorld(m_bulletCollisionDispatcher, m_bulletBroadphase,
                                                 m_bulletSolver, m_bulletCollisionConfiguration);

    //    m_bulletWorld = new bt(m_bulletCollisionDispatcher, m_bulletBroadphase, m_bulletSolver, m_bulletCollisionConfiguration);

    // assign gravity constant
    m_bulletWorld->setGravity(btVector3( 0.0, 0.0,-9.81));

    // Set SoftBody World Info
    m_bulletSoftBodyWorldInfo = new btSoftBodyWorldInfo();
    m_bulletSoftBodyWorldInfo->m_broadphase = m_bulletBroadphase;
    m_bulletSoftBodyWorldInfo->m_dispatcher = m_bulletCollisionDispatcher;
    m_bulletSoftBodyWorldInfo->air_density		=	(btScalar)0.0;
    m_bulletSoftBodyWorldInfo->water_density	=	0;
    m_bulletSoftBodyWorldInfo->water_offset		=	0;
    m_bulletSoftBodyWorldInfo->water_normal		=	btVector3(0,0,0);
    m_bulletSoftBodyWorldInfo->m_gravity.setValue(0,0,-9.81);

    //    m_bulletWorld->getDispatchInfo().m_enableSPU = true;
    m_bulletSoftBodyWorldInfo->m_sparsesdf.Initialize();

    m_chaiWorld = new cWorld();

    m_enclosureL = 4.0;
    m_enclosureW = 4.0;
    m_enclosureH = 3.0;

    m_pickMultiPoint = new cMultiPoint();
    m_pickMultiPoint->newPoint(cVector3d(0,0,0));
    m_pickMultiPoint->setPointSize(15);
    cColorf pickColor; pickColor.setGreenYellow();
    m_pickMultiPoint->setPointColor(pickColor);
    m_pickMultiPoint->setShowEnabled(false);
    addSceneObjectToWorld(m_pickMultiPoint);

    m_pickColor.setOrangeTomato();
    m_pickColor.setTransparencyLevel(0.3);
    m_namespace = "";
}

afWorld::~afWorld()
{
    m_pluginManager.close();

    if(m_bulletWorld){
        delete m_bulletWorld;
    }

    if(m_bulletCollisionConfiguration){
        delete m_bulletCollisionConfiguration;
    }

    if(m_bulletCollisionDispatcher){
        delete m_bulletCollisionDispatcher;
    }

    if(m_bulletBroadphase){
        delete m_bulletBroadphase;
    }

    if(m_bulletSoftBodyWorldInfo){
        delete m_bulletSoftBodyWorldInfo;
    }

    if(m_bulletSoftBodySolver){
        delete m_bulletSoftBodySolver;
    }

    if(m_bulletSolver){
        delete m_bulletSolver;
    }

    if (m_pickedConstraint != nullptr){
        delete m_pickedConstraint;
    }

    for(afChildrenMap::iterator it = m_childrenObjectsMap.begin() ; it != m_childrenObjectsMap.end() ; ++it){
        map<string, afBaseObject*> objMap = it->second;
        for(afBaseObjectMap::iterator oIt = objMap.begin() ; oIt != objMap.end() ; ++oIt){
            afBaseObject* obj = oIt->second;
            delete obj;
        }
    }

    for (map<string, afPointCloudPtr>::iterator it = m_pcMap.begin() ; it != m_pcMap.end() ; ++it){
        delete it->second;
    }

    if(m_chaiWorld){
        delete m_chaiWorld;
    }
}


///
/// \brief afWorld::get_enclosure_length
/// \return
///
double afWorld::getEnclosureLength(){
    return m_enclosureL;
}


///
/// \brief afWorld::get_enclosure_width
/// \return
///
double afWorld::getEnclosureWidth(){
    return m_enclosureW;
}


///
/// \brief afWorld::get_enclosure_height
/// \return
///
double afWorld::getEnclosureHeight(){
    return m_enclosureH;
}


///
/// \brief afWorld::get_enclosure_extents
/// \param length
/// \param width
/// \param height
///
void afWorld::getEnclosureExtents(double &length, double &width, double &height){
    length = m_enclosureL;
    width = m_enclosureW;
    height = m_enclosureH;
}

bool afWorld::loadCommunicationPlugin(afWorldPtr a_worldPtr, afWorldAttribsPtr a_attribs)
{
    bool result = false;
    if (isPassive() == false){
        afWorldCommunicationPlugin* commPlugin = new afWorldCommunicationPlugin();
        result = m_pluginManager.loadPlugin(a_worldPtr, a_attribs, commPlugin);
    }

    return result;
}


///
/// \brief afWorld::resetCameras
///
void afWorld::resetCameras(){
    cerr << "INFO! RESETTING ALL CAMERAS IN THE WORLD " << endl;
    afBaseObjectMap::iterator camIt;
    for (camIt = getCameraMap()->begin() ; camIt != getCameraMap()->end() ; ++camIt){
        camIt->second->reset();
    }

}


///
/// \brief afWorld::resetWorld
/// \param reset_time
///
void afWorld::resetDynamicBodies(){
    cerr << "INFO! RESETTING ALL BODIES IN THE WORLD " << endl;
    afBaseObjectMap::iterator rbIt;
    for (rbIt = getRigidBodyMap()->begin() ; rbIt != getRigidBodyMap()->end() ; ++rbIt){
        rbIt->second->reset();
    }
    clearResetBodiesFlag();
}


///
/// \brief afWorld::reset
///
void afWorld::reset(){
    cerr << "INFO! RESETTING WORLD" << endl;
    pausePhysics(true);
    for (afModelMap::iterator mIt = m_modelsMap.begin() ; mIt != m_modelsMap.end() ; ++mIt){
        (mIt->second)->reset();
    }

    // Call the reset for all plugins
    pluginsReset();
    clearResetFlag();
    pausePhysics(false);
}


///
/// \brief afWorld::setGravity
/// \param x
/// \param y
/// \param z
///
void afWorld::setGravity(afVector3d &vec)
{
    m_bulletWorld->setGravity(to_btVector(vec));
    m_bulletSoftBodyWorldInfo->m_gravity << vec;
}


///
/// \brief afWorld::getSimulationDeltaTime
/// \return
///
double afWorld::getSimulationDeltaTime()
{
    double dt = m_simulationTime - m_lastSimulationTime;
    return dt;
}


///
/// \brief afWorld::updateDynamics
/// \param a_interval
/// \param a_wallClock
/// \param a_loopFreq
/// \param a_numDevices
///
void afWorld::updateDynamics(double a_interval, int a_numDevices)
{
    // sanity check
    if (a_interval <= 0) { return; }

    if (m_resetFlag){
        reset();
        return;
    }

    if (m_resetBodiesFlag){
        resetDynamicBodies();
        return;
    }

    if (m_pausePhx){
        if (m_manualStepPhx > 0){
            m_manualStepPhx--;
        }
        else{
            return;
        }
    }

    m_freqCounterPhysics.signal(1);
    m_numDevices = a_numDevices;

    double dt = getSimulationDeltaTime();

    // integrate simulation during an certain interval
    m_bulletWorld->stepSimulation(a_interval, m_maxIterations, m_integrationTimeStep);

    // add time to overall simulation
    m_lastSimulationTime = m_simulationTime;
    m_simulationTime = m_simulationTime + a_interval;

    setTimeStamp(getSystemTime());
    estimateBodyWrenches();

    for (afModelMap::iterator mIt = m_modelsMap.begin() ; mIt != m_modelsMap.end() ; ++mIt){
        (mIt->second)->update(dt);
    }

    for (map<string, afPointCloudPtr>::iterator pcIt = m_pcMap.begin() ; pcIt != m_pcMap.end() ; ++pcIt){
        (pcIt->second)->setTimeStamp(getCurrentTimeStamp());
        (pcIt->second)->update(dt);
    }

    // Update all plugins, world, models and then objects
    pluginsPhysicsUpdate(dt);
}


///
/// \brief afWorld::estimateBodyWrenches
///
void afWorld::estimateBodyWrenches(){

    // First clear out the wrench estimation from last iteration
    afBaseObjectMap::iterator rbIt = getRigidBodyMap()->begin();
    for (; rbIt != getRigidBodyMap()->end() ; ++rbIt){
        afRigidBodyPtr rb = (afRigidBodyPtr)rbIt->second;
        rb->m_estimatedForce.setZero();
        rb->m_estimatedTorque.setZero();
    }


    // Now estimate the wrenches based on joints that have feedback enabled
    afBaseObjectMap::iterator jIt = getJointMap()->begin();
    for (; jIt != getJointMap()->end() ; ++ jIt){
        afJointPtr jnt = (afJointPtr)jIt->second;
        if (jnt->isFeedBackEnabled()){
            const btJointFeedback* fb = jnt->m_btConstraint->getJointFeedback();
            btMatrix3x3 R_wINp = jnt->m_afParentBody->m_bulletRigidBody->getWorldTransform().getBasis().transpose();
            btMatrix3x3 R_wINc = jnt->m_afChildBody->m_bulletRigidBody->getWorldTransform().getBasis().transpose();

            btVector3 F_jINp = R_wINp * fb->m_appliedForceBodyA;
            btVector3 F_jINc = R_wINc * fb->m_appliedForceBodyB;

            jnt->m_afParentBody->m_estimatedForce += F_jINp;
            jnt->m_afChildBody->m_estimatedForce += F_jINc;

            btVector3 T_jINp = R_wINp * fb->m_appliedTorqueBodyA;
            btVector3 T_jINc = R_wINc * fb->m_appliedTorqueBodyB;

            jnt->m_afParentBody->m_estimatedTorque += T_jINp;
            jnt->m_afChildBody->m_estimatedTorque += T_jINc;

            // We can also estimate the joint effort using the parent axes.
            if (jnt->m_jointType == afJointType::REVOLUTE || jnt->m_jointType == afJointType::TORSION_SPRING){
                jnt->m_estimatedEffort = btDot(jnt->m_axisA, T_jINp);
            }
            else if (jnt->m_jointType == afJointType::PRISMATIC || jnt->m_jointType == afJointType::LINEAR_SPRING){
                jnt->m_estimatedEffort = btDot(jnt->m_axisA, F_jINp);
            }
            else{
                // If its a multiDOF joint, we don't compute the joint effort
            }

        }
    }

}

///
/// \brief afWorld::updatePositionFromDynamics
///
void afWorld::updateSceneObjects()
{
    // Update all models
    for(afModelMap::iterator mIt = m_modelsMap.begin(); mIt != m_modelsMap.end(); ++mIt)
    {
        (mIt->second)->updateGlobalPose();
    }

    // Update all models
    for(afModelMap::iterator mIt = m_modelsMap.begin(); mIt != m_modelsMap.end(); ++mIt)
    {
        (mIt->second)->updateSceneObjects();
    }
}


///
/// \brief afWorld::pluginsGraphicsUpdate
///
void afWorld::pluginsGraphicsUpdate()
{
    m_pluginManager.graphicsUpdate();
    // Update all models
    for(afModelMap::iterator mIt = m_modelsMap.begin(); mIt != m_modelsMap.end(); ++mIt)
    {
        (mIt->second)->pluginsGraphicsUpdate();
    }

    for (map<string, afPointCloudPtr>::iterator pcIt = m_pcMap.begin() ; pcIt != m_pcMap.end() ; ++pcIt){
        (pcIt->second)->pluginsGraphicsUpdate();
    }

}


///
/// \brief afWorld::pluginsPhysicsUpdate
/// \param dt
///
void afWorld::pluginsPhysicsUpdate(double dt)
{
    m_pluginManager.physicsUpdate(dt);
    // Update all models
    for(afModelMap::iterator mIt = m_modelsMap.begin(); mIt != m_modelsMap.end(); ++mIt)
    {
        (mIt->second)->pluginsPhysicsUpdate(dt);
    }

    for (map<string, afPointCloudPtr>::iterator pcIt = m_pcMap.begin() ; pcIt != m_pcMap.end() ; ++pcIt){
        (pcIt->second)->pluginsPhysicsUpdate(dt);
    }
}

void afWorld::pluginsReset()
{
    m_pluginManager.reset();
    // Update all models
    for(afModelMap::iterator mIt = m_modelsMap.begin(); mIt != m_modelsMap.end(); ++mIt)
    {
        (mIt->second)->pluginsReset();
    }
}

///
/// \brief afWorld::addSceneObject
/// \param a_cObject
///
void afWorld::addSceneObjectToWorld(cGenericObject *a_cObject)
{
    m_chaiWorld->addChild(a_cObject);
}


///
/// \brief afWorld::removeChild
/// \param a_cObject
///
void afWorld::removeSceneObjectFromWorld(cGenericObject *a_cObject)
{
    m_chaiWorld->removeChild(a_cObject);
}


///
/// \brief afWorld::compute_step_size
/// \param adjust_int_steps
/// \return
///
double afWorld::computeStepSize(bool adjust_intetration_steps){
    double step_size = m_wallClock.getCurrentTimeSeconds() - getSimulationTime();
    if (adjust_intetration_steps){
        int min_iterations = 2;
        if (step_size >= getIntegrationTimeStep() * min_iterations){
            int int_steps_max =  step_size / getIntegrationTimeStep();
            if (int_steps_max > getMaxIterations()){
                int_steps_max = getMaxIterations();
            }
            setIntegrationMaxIterations(int_steps_max + min_iterations);        }
    }
    return step_size;
}


///
/// \brief afWorld::createDefaultWorld
/// \return
///
bool afWorld::createDefaultWorld(){
    // TRANSPARENT WALLS
    double box_l, box_w, box_h;
    box_l = getEnclosureLength();
    box_w = getEnclosureWidth();
    box_h = getEnclosureHeight();

    double thickness = 0.1;

    bool usePlanes = true;

    afRigidBody walls[5] = {afRigidBody(this, nullptr),
                            afRigidBody(this, nullptr),
                            afRigidBody(this, nullptr),
                            afRigidBody(this, nullptr),
                            afRigidBody(this, nullptr)};

    afRigidBodyAttributes rbAttribs[5];
    afPrimitiveShapeAttributes shapeAttribs[5];
    if (usePlanes){
        shapeAttribs[0].setPlaneData(-1, 0, 0, 0.0);
        shapeAttribs[1].setPlaneData(0, -1, 0, 0.0);
        shapeAttribs[2].setPlaneData(1, 0, 0, 0.0);
        shapeAttribs[3].setPlaneData(0, 1, 0, 0.0);
        shapeAttribs[4].setPlaneData(0, 0, 1, 0.0);

    }
    else{
        shapeAttribs[0].setBoxData(box_l, box_w, box_h);
        shapeAttribs[1].setBoxData(box_l, box_w, box_h);
        shapeAttribs[2].setBoxData(box_l, box_w, box_h);
        shapeAttribs[3].setBoxData(box_l, box_w, box_h);
        shapeAttribs[4].setBoxData(box_l, box_w, box_h);
    }

    for (int i = 0 ; i < 5 ; i++){
        rbAttribs[i].m_visualAttribs.m_primitiveShapes.push_back(shapeAttribs[i]);
        rbAttribs[i].m_visualAttribs.m_colorAttribs = afColorAttributes();
        rbAttribs[i].m_visualAttribs.m_geometryType = afGeometryType::SINGLE_SHAPE;
        // SET LOCATION
        //        rbAttribs[i].m_kinematicAttribs.m_location = LOCATION;
        walls[i].createFromAttribs(&rbAttribs[i]);
    }

    // add plane to world as we will want to make it visibe
    //    addChild(walls);
    return true;
}



bool afWorld::createFromAttribs(afWorldAttributes* a_attribs){

    a_attribs->resolveRelativeNamespace();
    a_attribs->resolveRelativePathAttribs();

    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_maxIterations = a_attribs->m_maxIterations;

    setGravity(a_attribs->m_gravity);

    afModelPtr envModel;

    if (a_attribs->m_environmentModel.m_use){
        envModel = new afModel(this);
        envModel->createFromAttribs(&a_attribs->m_environmentModel.m_modelAttribs);

    }
    else if (a_attribs->m_enclosure.m_use){
        m_enclosureL = a_attribs->m_enclosure.m_length;
        m_enclosureW = a_attribs->m_enclosure.m_width;
        m_enclosureH = a_attribs->m_enclosure.m_height;

        createDefaultWorld();
    }
    else{
        // THROW SOME ERROR THAT NO MODEL FOR WORLD CAN BE LOADED
    }


    m_skyBoxAttribs = a_attribs->m_skyBoxAttribs;

    for (size_t idx = 0 ; idx < a_attribs->m_lightAttribs.size(); idx++){
        afLightPtr lightPtr = new afLight(this, envModel);
        if (lightPtr->createFromAttribs(&a_attribs->m_lightAttribs[idx])){
            envModel->addLight(lightPtr);
        }
    }

    if (envModel->getLightMap()->size() == 0){
        // ADD A DEFAULT LIGHT???
        afLightAttributes lightAttribs;
        lightAttribs.m_kinematicAttribs.m_location.setPosition(afVector3d(2, 2, 5));
        lightAttribs.m_identificationAttribs.m_name = "default_light";
        afLightPtr lightPtr = new afLight(this, envModel);
        lightPtr->createFromAttribs(&lightAttribs);
        envModel->addLight(lightPtr);
    }

    if (a_attribs->m_showGUI){
        for (size_t idx = 0 ; idx < a_attribs->m_cameraAttribs.size(); idx++){
            afCameraPtr cameraPtr = new afCamera(this, envModel);
            if (cameraPtr->createFromAttribs(&a_attribs->m_cameraAttribs[idx])){
                envModel->addCamera(cameraPtr);
            }
        }

        if (envModel->getCameraMap()->size() == 0){
            // No valid cameras defined in the world config file
            // hence create a default camera
            afCameraPtr cameraPtr = new afCamera(this, envModel);
            afCameraAttributes camAttribs;
            camAttribs.m_lookAt.set(-1, 0, 0);
            camAttribs.m_identificationAttribs.m_name = "default_camera";
            if (cameraPtr->createFromAttribs(&camAttribs)){
                envModel->addCamera(cameraPtr);
            }

        }

        m_shaderAttribs = a_attribs->m_shaderAttribs;
        loadShaderProgram();
    }

    addModel(envModel);

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    return true;
}

bool afWorld::loadPlugins(afWorldPtr worldPtr, afWorldAttribsPtr attribs, vector<afPluginAttributes> *pluginAttribs)
{
    for (int i = 0 ; i < pluginAttribs->size(); i++){
        m_pluginManager.loadPlugin(worldPtr, attribs, (*pluginAttribs)[i].m_filename, (*pluginAttribs)[i].m_name, (*pluginAttribs)[i].m_path.c_str());
    }

    return true;
}

///
/// \brief afWorld::render
/// \param options
///
void afWorld::render(afRenderOptions &options)
{
    updateSceneObjects();

    // Update shadow maps once
    m_chaiWorld->updateShadowMaps(false, options.m_mirroredDisplay);

    afBaseObjectMap::iterator camIt;
    for (camIt = getCameraMap()->begin(); camIt != getCameraMap()->end(); ++ camIt){
        afCameraPtr cameraPtr = (afCameraPtr)camIt->second;
        if (!cameraPtr->overrideRendering()){
            cameraPtr->render(options);
        }
    }

    // Update all plugins, world, then models and then objects
    pluginsGraphicsUpdate();
}

cWorld *afWorld::getChaiWorld(){
    //    cerr << m_chaiWorld << endl;
    return m_chaiWorld;
}

afCameraPtr afWorld::getAssociatedCamera(GLFWwindow *a_window)
{
    afBaseObjectMap::iterator g_cameraIt;
    for (g_cameraIt = getCameraMap()->begin() ; g_cameraIt != getCameraMap()->end() ; ++g_cameraIt){
        afCameraPtr camPtr = (afCameraPtr)g_cameraIt->second;
        if (a_window == camPtr->m_window){
            return camPtr;
        }
    }
    return nullptr;
}


///
/// \brief afWorld::makeCameraWindowsFullScreen
/// \param a_fullscreen
///
void afWorld::makeCameraWindowsFullScreen(bool a_fullscreen)
{
    afBaseObjectMap::iterator cIt;

    for (cIt = getCameraMap()->begin() ; cIt != getCameraMap()->end() ; cIt++){
        afCameraPtr cameraPtr = (afCameraPtr)cIt->second;
        cameraPtr->makeWindowFullScreen(a_fullscreen);
    }
}


///
/// \brief afWorld::makeCameraWindowsMirrorVertical
/// \param a_mirrorVertical
///
void afWorld::makeCameraWindowsMirrorVertical(bool a_mirrorVertical)
{
    afBaseObjectMap::iterator cIt;

    for (cIt = getCameraMap()->begin() ; cIt != getCameraMap()->end() ; cIt++){
        afCameraPtr cameraPtr = (afCameraPtr)cIt->second;
        cameraPtr->setWindowMirrorVertical(a_mirrorVertical);
    }
}


///
/// \brief afWorld::destroyCameraWindows
///
void afWorld::destroyCameraWindows()
{
    afBaseObjectMap::iterator cIt;

    for (cIt = getCameraMap()->begin() ; cIt != getCameraMap()->end() ; cIt++){
        afCameraPtr cameraPtr = (afCameraPtr)cIt->second;
        cameraPtr->destroyWindow();
    }
}


///
/// \brief afWorld::createSkyBox
///
void afWorld::loadSkyBox(){
    if (m_skyBoxAttribs.m_use){

        m_skyBoxMesh = new cMesh();
        float cube[] = {
            // positions
            -1.0f,  1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,

            -1.0f, -1.0f,  1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f,  1.0f,
            -1.0f, -1.0f,  1.0f,

            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,

            -1.0f, -1.0f,  1.0f,
            -1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f, -1.0f,  1.0f,
            -1.0f, -1.0f,  1.0f,

            -1.0f,  1.0f, -1.0f,
            1.0f,  1.0f, -1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            -1.0f,  1.0f,  1.0f,
            -1.0f,  1.0f, -1.0f,

            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f,  1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f,  1.0f,
            1.0f, -1.0f,  1.0f
        };

        for (int vI = 0 ; vI < 12 ; vI++){
            int off = vI * 9;
            m_skyBoxMesh->newTriangle(
                        cVector3d(cube[off + 0], cube[off + 1], cube[off + 2]),
                    cVector3d(cube[off + 3], cube[off + 4], cube[off + 5]),
                    cVector3d(cube[off + 6], cube[off + 7], cube[off + 8]));
        }

        m_skyBoxMesh->computeAllNormals();

        cTextureCubeMapPtr newTexture = cTextureCubeMap::create();

        for (int iI = 0 ; iI < 6 ; iI++){
            newTexture->m_images[iI] = cImage::create();
        }

        bool res[6];

        res[0] = newTexture->m_images[0]->loadFromFile(m_skyBoxAttribs.m_rightImageFilepath.c_str());
        res[1] = newTexture->m_images[1]->loadFromFile(m_skyBoxAttribs.m_leftImageFilepath.c_str());
        res[2] = newTexture->m_images[2]->loadFromFile(m_skyBoxAttribs.m_bottomImageFilepath.c_str());
        res[3] = newTexture->m_images[3]->loadFromFile(m_skyBoxAttribs.m_topImageFilepath.c_str());
        res[4] = newTexture->m_images[4]->loadFromFile(m_skyBoxAttribs.m_frontImageFilepath.c_str());
        res[5] = newTexture->m_images[5]->loadFromFile(m_skyBoxAttribs.m_backImageFilepath.c_str());

        if (res[0] && res[1] && res[2] && res[3] && res[4] && res[5] && res[5]){
            // All images were loaded succesfully

            m_skyBoxMesh->setTexture(newTexture);
            m_skyBoxMesh->setUseTexture(true);

            cShaderProgramPtr shaderProgram;
            if (m_skyBoxAttribs.m_shaderAttribs.m_shaderDefined){
                shaderProgram = afShaderUtils::createFromAttribs(&m_skyBoxAttribs.m_shaderAttribs, getQualifiedName(), "SKYBOX_SHADERS");
            }
            else{
                cerr << "INFO! USING INTERNALLY DEFINED SKYBOX SHADERS" << endl;
                shaderProgram = cShaderProgram::create(AF_SKYBOX_VTX, AF_SKYBOX_FRAG);
            }
            if (shaderProgram->linkProgram()){
                // Just empty Pts to let us use the shader
                cGenericObject* go;
                cRenderOptions ro;
                shaderProgram->use(go, ro);
                m_skyBoxMesh->setShaderProgram(shaderProgram);
                addSceneObjectToWorld(m_skyBoxMesh);

            }
            else{
                cerr << "ERROR! FOR SKYBOX FAILED TO LOAD SHADERS" << endl;
                delete m_skyBoxMesh;
            }
        }
        else{
            cerr << "CAN'T LOAD SKY BOX IMAGES, IGNORING\n";
        }
    }
}

void afWorld::runHeadless(bool value)
{
    m_headless = value;
}

bool afWorld::isHeadless()
{
    return m_headless;
}


///
/// \brief afWorld::loadShaderProgram
///
void afWorld::loadShaderProgram(){
    if (m_shaderAttribs.m_shaderDefined){
        m_shaderProgram = afShaderUtils::createFromAttribs(&m_shaderAttribs, getQualifiedName(), "GLOBAL_SHADERS");
    }
}


///
/// \brief afWorld::buildCollisionGroups
///
void afWorld::buildCollisionGroups(){
    if (m_collisionGroups.size() > 0){
        vector<int> groupNumbers;

        map<uint, vector<afInertialObjectPtr> >::iterator cgIt;
        for(cgIt = m_collisionGroups.begin() ; cgIt != m_collisionGroups.end() ; ++cgIt){
            groupNumbers.push_back(cgIt->first);
        }

        for (uint i = 0 ; i < groupNumbers.size() - 1 ; i++){
            int aIdx = groupNumbers[i];
            vector<afInertialObjectPtr> grpA = m_collisionGroups[aIdx];
            for (uint j = i + 1 ; j < groupNumbers.size() ; j ++){
                int bIdx = groupNumbers[j];
                vector<afInertialObjectPtr> grpB = m_collisionGroups[bIdx];

                for(uint aBodyIdx = 0 ; aBodyIdx < grpA.size() ; aBodyIdx++){
                    afInertialObjectPtr bodyA = grpA[aBodyIdx];
                    for(uint bBodyIdx = 0 ; bBodyIdx < grpB.size() ; bBodyIdx++){
                        afInertialObjectPtr bodyB = grpB[bBodyIdx];
                        if (bodyA != bodyB && !bodyB->isCommonCollisionGroupIdx(bodyA->m_collisionGroups))
                            bodyA->m_bulletRigidBody->setIgnoreCollisionCheck(bodyB->m_bulletRigidBody, true);
                    }
                }
            }
        }
    }
}




// The following function has been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief afWorld::pickBody
/// \param rayFromWorld
/// \param rayToWorld
/// \return
///
bool afWorld::pickBody(const cVector3d &rayFromWorld, const cVector3d &rayToWorld){
    btDynamicsWorld* m_dynamicsWorld = m_bulletWorld;
    if (m_dynamicsWorld == nullptr)
        return false;

    btCollisionWorld::ClosestRayResultCallback rayCallback(to_btVector(rayFromWorld), to_btVector(rayToWorld));

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
    m_dynamicsWorld->rayTest(to_btVector(rayFromWorld), to_btVector(rayToWorld), rayCallback);
    if (rayCallback.hasHit())
    {
        cVector3d pickPos;
        pickPos << rayCallback.m_hitPointWorld;
        m_pickMultiPoint->setLocalPos(pickPos);
        m_pickMultiPoint->setShowEnabled(true);
        const btCollisionObject* colObject = rayCallback.m_collisionObject;
        if (colObject->getInternalType() == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
            btRigidBody* body = (btRigidBody*)btRigidBody::upcast(colObject);
            if (body){
                m_pickedRigidBody = getRigidBody(body, true);
                if (m_pickedRigidBody){
                    cerr << "User picked AF rigid body: " << m_pickedRigidBody->m_name << endl;
                    m_pickedBulletRigidBody = body;
                    m_pickedRigidBody->m_visualMesh->backupMaterialColors(true);
                    m_pickedRigidBody->m_visualMesh->setMaterial(m_pickColor);
                    m_savedState = m_pickedBulletRigidBody->getActivationState();
                    m_pickedBulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
                }

                //other exclusions?
                if (!(body->isStaticObject() || body->isKinematicObject()))
                {
                    //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                    btVector3 localPivot = body->getCenterOfMassTransform().inverse() * to_btVector(pickPos);
                    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                    m_dynamicsWorld->addConstraint(p2p, true);
                    m_pickedConstraint = p2p;
                    btScalar mousePickClamping = 1/body->getInvMass();
                    p2p->m_setting.m_impulseClamp = mousePickClamping;
                    //very weak constraint for picking
                    p2p->m_setting.m_tau = 1/body->getInvMass();
                }
                else{
                    cVector3d com;
                    com << body->getCenterOfMassPosition();
                    m_pickedOffset = com - pickPos;
                }
            }
        }
        else if((colObject->getInternalType() == btCollisionObject::CollisionObjectTypes::CO_SOFT_BODY)){
            btSoftBody* sBody = (btSoftBody*)btSoftBody::upcast(colObject);
            // Now find the closest node in the soft body so we can do
            // something about it.
            btVector3 hitPoint = rayCallback.m_hitPointWorld;

            // Max distance between the hit point softbody nodes to be considered
            double maxDistance = 0.1;

            // Index of closest Node. Initialize to -1 so it can be used
            // as boolean as well if a Node was Found
            int closestNodeIdx = -1;

            for (int nodeIdx = 0 ; nodeIdx < sBody->m_nodes.size() ; nodeIdx++){
                if ( (hitPoint - sBody->m_nodes[nodeIdx].m_x).length() < maxDistance ){
                    maxDistance = (hitPoint - sBody->m_nodes[nodeIdx].m_x).length();
                    closestNodeIdx = nodeIdx;
                }
            }

            if(closestNodeIdx >=0 ){
                m_pickedNode = &sBody->m_nodes[closestNodeIdx];
                m_pickedNode->m_v.setZero();
                m_pickedSoftBody = sBody;
                m_pickedNodeIdx = closestNodeIdx;
                m_pickedNodeGoal << hitPoint;
            }
        }


        m_oldPickingPos = rayToWorld;
        m_hitPos = pickPos;
        m_oldPickingDist = (pickPos - rayFromWorld).length();
    }
    return false;

}

// The following function has been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief afModel::movePickedBody
/// \param rayFromWorld
/// \param rayToWorld
/// \return
///
bool afWorld::movePickedBody(const cVector3d &rayFromWorld, const cVector3d &rayToWorld){
    if (m_pickedBulletRigidBody)
    {
        //keep it at the same picking distance
        cVector3d newLocation;

        cVector3d dir = rayToWorld - rayFromWorld;
        dir.normalize();
        dir *= m_oldPickingDist;

        newLocation = rayFromWorld + dir;
        // Set the position of grab sphere
        m_pickMultiPoint->setLocalPos(newLocation);

        if (m_pickedConstraint){
            btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
            if (pickCon)
            {
                pickCon->setPivotB(to_btVector(newLocation));
                return true;
            }
        }
        else{
            // In this case the rigidBody is a static or kinematic body
            btTransform curTrans = m_pickedBulletRigidBody->getWorldTransform();
            curTrans.setOrigin(to_btVector(newLocation + m_pickedOffset));
            m_pickedBulletRigidBody->getMotionState()->setWorldTransform(curTrans);
            m_pickedBulletRigidBody->setWorldTransform(curTrans);
            return true;
        }
    }

    if (m_pickedSoftBody){
        //keep it at the same picking distance

        cVector3d newPivotB;

        cVector3d dir = rayToWorld - rayFromWorld;
        dir.normalize();
        dir *= m_oldPickingDist;

        newPivotB = rayFromWorld + dir;
        m_pickMultiPoint->setLocalPos(newPivotB);
        m_pickedNodeGoal = newPivotB;
        return true;
    }
    return false;
}


// The following function has been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief afModel::removePickingConstraint
///
void afWorld::removePickingConstraint(){
    btDynamicsWorld* m_dynamicsWorld = m_bulletWorld;
    if (m_pickedConstraint)
    {
        m_pickedBulletRigidBody->forceActivationState(m_savedState);
        m_pickedBulletRigidBody->activate();
        m_dynamicsWorld->removeConstraint(m_pickedConstraint);
        delete m_pickedConstraint;
        m_pickedConstraint = nullptr;
    }

    if (m_pickedBulletRigidBody){
        m_pickMultiPoint->setShowEnabled(false);
        m_pickedBulletRigidBody = nullptr;
    }

    if (m_pickedRigidBody){
        m_pickedRigidBody->m_visualMesh->restoreMaterialColors(true);
    }

    if (m_pickedSoftBody){
        m_pickMultiPoint->setShowEnabled(false);
        m_pickedSoftBody = nullptr;
        m_pickedNodeIdx = -1;
        m_pickedNodeMass = 0;
    }
}


///
/// \brief afCamera::afCamera
///
afCamera::afCamera(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::CAMERA, a_afWorld, a_modelPtr){
    m_camera = nullptr;
    setOverrideRendering(false);
    m_monitors = glfwGetMonitors(&m_numMonitors);
    m_targetVisualMarker = new cMesh();
    cCreateSphere(m_targetVisualMarker, 0.03);
    m_targetVisualMarker->m_material->setBlack();
    m_targetVisualMarker->setShowFrame(false);
    m_targetVisualMarker->setTransparencyLevel(0.7);
    m_targetVisualMarker->setUseDisplayList(true);
    m_targetVisualMarker->markForUpdate(false);
    m_targetVisualMarker->setShowEnabled(false);
    addChildSceneObject(m_targetVisualMarker, cTransform());
}


///
/// \brief afCamera::setView
/// \param a_localPosition
/// \param a_localLookAt
/// \param a_localUp
/// \return
///
bool afCamera::setView(const cVector3d &a_localPosition, const cVector3d &a_localLookAt, const cVector3d &a_localUp){
    // copy new values to temp variables
    cVector3d pos = a_localPosition;
    cVector3d lookAt = a_localLookAt;
    cVector3d up = a_localUp;
    cVector3d Cy;

    // check validity of vectors
    if (pos.distancesq(lookAt) < C_SMALL) { return (false); }
    if (up.lengthsq() < C_SMALL) { return (false); }

    // compute new rotation matrix
    pos.sub(lookAt);
    pos.normalize();
    up.normalize();
    up.crossr(pos, Cy);
    if (Cy.lengthsq() < C_SMALL) { return (false); }
    Cy.normalize();
    pos.crossr(Cy,up);

    // update frame with new values
    setLocalPos(a_localPosition);
    cMatrix3d localRot;
    localRot.setCol(pos, Cy, up);
    setLocalRot(localRot);

    // World in this body frame
    cTransform T_wINb = getLocalTransform();
    T_wINb.invert();
    m_targetPos = T_wINb * a_localLookAt;
    m_targetVisualMarker->setLocalPos(m_targetPos);

    return true;
}

double afCamera::getFieldViewAngle() const { return m_camera->getFieldViewAngleRad(); }


///
/// \brief afCamera::setImagePublishInterval
/// \param a_interval
///
void afCamera::setImagePublishInterval(uint a_interval){
    uint minInterval = 1;
    m_imagePublishInterval = cMax(a_interval, minInterval);
}


///
/// \brief afCamera::setDepthPublishInterval
/// \param a_interval
///
void afCamera::setDepthPublishInterval(uint a_interval){
    uint minInterval = 1;
    m_depthPublishInterval = cMax(a_interval, minInterval);
}


///
/// \brief afCamera::setWindowMirrorVertical
/// \param a_enabled
///
void afCamera::setWindowMirrorVertical(bool a_enabled){
    m_camera->setMirrorVertical(a_enabled);
}


///
/// \brief afCamera::getGlobalPos
/// \return
///
cVector3d afCamera::getGlobalPos(){
    if (m_camera->getParent()){
        return m_camera->getParent()->getLocalTransform() * getLocalPos();
    }
    else{
        return m_camera->getLocalPos();
    }
}


///
/// \brief afCamera::setTargetPosLocal
/// \param a_pos
///
void afCamera::setTargetPos(cVector3d a_pos){
    setView(getLocalPos(), a_pos, getUpVector());
}

///
/// \brief afCamera::showTargetPos
/// \param a_show
///
void afCamera::showTargetPos(bool a_show){
    m_targetVisualMarker->setShowEnabled(a_show);
    m_targetVisualMarker->setShowFrame(a_show);
}


///
/// \brief afCamera::getRenderTime
/// \return
///
double afCamera::getRenderTimeStamp()
{
    return m_renderTimeStamp;
}


///
/// \brief afCamera::createFrameBuffers
/// \param imageAttribs
///
void afCamera::createFrameBuffers(afImageResolutionAttribs* imageAttribs){
    if (m_frameBuffersCreated){
        return;
    }

    m_publishImageResolution = *imageAttribs;
    m_frameBuffer = new cFrameBuffer();
    m_bufferColorImage = cImage::create();
    m_bufferDepthImage = cImage::create();
    m_frameBuffer->setup(m_camera, imageAttribs->m_width, imageAttribs->m_height, true, true);

    m_frameBuffersCreated = true;
}


///
/// \brief afCamera::createPreProcessingShaders
/// \param preprocessingShaderAttribs
///
void afCamera::createPreProcessingShaders(afShaderAttributes* preprocessingShaderAttribs){
    m_preprocessingShaderAttribs = *preprocessingShaderAttribs;
    m_preprocessingShaderProgram = afShaderUtils::createFromAttribs(&m_preprocessingShaderAttribs, getQualifiedName(), "FRAMEBUFFER_PREPROCESSING");
}


///
/// \brief afCamera::getTargetPosGlobal
/// \return
///
cVector3d afCamera::getTargetPosLocal(){
    return m_localTransform * m_targetPos;
}

///
/// \brief afCamera::getTargetPosGlobal
/// \return
///
cVector3d afCamera::getTargetPosGlobal(){
    return m_globalTransform * m_targetPos;
}

bool afCamera::createFromAttribs(afCameraAttributes *a_attribs)
{
    if (m_afWorld->isHeadless()){
        // Asked to run headless, don't load cameras
        return false;
    }
    storeAttributes(a_attribs);

    // Set some default values
    m_stereoMode = C_STEREO_DISABLED;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    m_camPos << a_attribs->m_kinematicAttribs.m_location.getPosition();
    m_camLookAt << a_attribs->m_lookAt;
    m_camUp << a_attribs->m_up;

    setOrthographic(a_attribs->m_orthographic);

    if (a_attribs->m_stereo){
        m_stereoMode = cStereoMode::C_STEREO_PASSIVE_LEFT_RIGHT;
    }

    m_controllingDevNames = a_attribs->m_controllingDeviceNames;

    m_publishImage = a_attribs->m_publishImage;
    m_imagePublishInterval = a_attribs->m_publishImageInterval;
    m_publishDepth = a_attribs->m_publishDepth;
    m_depthPublishInterval = a_attribs->m_publishDepthInterval;

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    m_camera = new cCamera(m_afWorld->getChaiWorld());

    addChildSceneObject(m_camera, cTransform());

    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;

    if (m_parentName.empty() == false){
        m_afWorld->addObjectMissingParent(this);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // position and orient the camera
    setView(m_camPos, m_camLookAt, m_camUp);
    m_initialTransform = getLocalTransform();
    // set the near and far clipping planes of the camera
    m_camera->setClippingPlanes(a_attribs->m_nearPlane, a_attribs->m_farPlane);

    // set stereo mode
    m_camera->setStereoMode(m_stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    m_camera->setStereoEyeSeparation(a_attribs->m_stereoEyeSeparation);
    m_camera->setStereoFocalLength(a_attribs->m_stereFocalLength);

    // set vertical mirrored display mode
    m_camera->setMirrorVertical(false);

    if (isOrthographic()){
        m_camera->setOrthographicView(a_attribs->m_orthoViewWidth);
    }
    else{
        m_camera->setFieldViewAngleRad(a_attribs->m_fieldViewAngle);
    }

    m_camera->setUseMultipassTransparency(a_attribs->m_multiPass);

    m_monitorNumber = a_attribs->m_monitorNumber;

    setVisibleFlag(a_attribs->m_visible);

    m_mouseControlScales = a_attribs->m_mouseControlScales;

    //    a_attribs->m_visible;
    createWindow();

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    m_graphicsDynamicsFreqLabel = new cLabel(font);
    m_wallSimTimeLabel = new cLabel(font);
    m_devicesModesLabel = new cLabel(font);
    m_deviceButtonLabel = new cLabel(font);
    m_controllingDeviceLabel = new cLabel(font);

    m_graphicsDynamicsFreqLabel->m_fontColor.setBlack();
    m_wallSimTimeLabel->m_fontColor.setBlack();
    m_devicesModesLabel->m_fontColor.setBlack();
    m_deviceButtonLabel->m_fontColor.setBlack();
    m_controllingDeviceLabel->m_fontColor.setBlack();
    m_controllingDeviceLabel->setFontScale(0.8);

    m_camera->m_frontLayer->addChild(m_graphicsDynamicsFreqLabel);
    m_camera->m_frontLayer->addChild(m_wallSimTimeLabel);
    m_camera->m_frontLayer->addChild(m_devicesModesLabel);
    m_camera->m_frontLayer->addChild(m_deviceButtonLabel);
    m_camera->m_frontLayer->addChild(m_controllingDeviceLabel);

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getCameraMap());
    setGlobalRemapIdx(remap_idx);


    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    if (m_publishImage || m_publishDepth){

        createPreProcessingShaders(&a_attribs->m_preProcessShaderAttribs);

        if(m_publishImage){
            enableImagePublishing(&a_attribs->m_publishImageResolution);
            afCameraVideoStreamerPlugin* videoPlugin = new afCameraVideoStreamerPlugin();
            m_pluginManager.loadPlugin(this, a_attribs, videoPlugin);
        }

        if (m_publishDepth){
            enableDepthPublishing(&a_attribs->m_publishImageResolution, &a_attribs->m_depthNoiseAttribs, &a_attribs->m_depthComputeShaderAttribs);
            afCameraDepthStreamerPlugin* depthPlugin = new afCameraDepthStreamerPlugin();
            m_pluginManager.loadPlugin(this, a_attribs, depthPlugin);
        }
    }

    loadCommunicationPlugin(this, a_attribs);

    return true;
}


///
/// \brief afCamera::createWindow
/// \return
///
bool afCamera::createWindow()
{
    string window_name = "AMBF Simulator Window " + to_string(s_cameraIdx + 1);
    if (m_controllingDevNames.size() > 0){
        for (int i = 0 ; i < m_controllingDevNames.size() ; i++){
            window_name += (" - " + m_controllingDevNames[i]);
        }

    }

    if (m_monitorNumber < 0 || m_monitorNumber >= m_numMonitors){
        cerr << "INFO! CAMERA \"" << m_name << "\" MONITOR NUMBER \"" << m_monitorNumber
             << "\" IS NOT IN RANGE OF AVAILABLE MONITORS \""<< m_numMonitors <<"\", USING DEFAULT" << endl;
        if (s_cameraIdx < m_numMonitors){
            m_monitorNumber = s_cameraIdx;
        }
        else{
            m_monitorNumber = 0;
        }
    }

    m_monitor = m_monitors[m_monitorNumber];

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(m_monitor);
    int w = 0.8 * mode->width;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    m_width = w;
    m_height = h;

    if (getVisibleFlag() == false){
        cerr << "INFO! CAMERA \"" << m_name << "\" SET TO INVISIBLE. THEREFORE IT IS ONLY VIEWABLE"
                                               " VIA ITS IMAGE / DEPTH TOPICS IF THOSE ARE SET TO TRUE" << endl;
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }
    else{
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    }

    m_window = glfwCreateWindow(w, h, window_name.c_str(), nullptr, s_mainWindow);
    if (s_windowIdx == 0){
        s_mainWindow = m_window;
    }

    if (!m_window)
    {
        cerr << "ERROR! FAILED TO CREATE OPENGL WINDOW" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 0;
    }

    assignWindowCallbacks(&m_afWorld->m_cameraWindowCallbacks);

    // set the current context
    glfwMakeContextCurrent(m_window);

    glfwSwapInterval(0);

    // get width and height of window
    glfwGetWindowSize(m_window, &m_width, &m_height);


    int xpos, ypos;
    glfwGetMonitorPos(m_monitor, &xpos, &ypos);
    x += xpos; y += ypos;

    // set position of window
    glfwSetWindowPos(m_window, x, y);

//    glfwSetWindowMonitor(m_window, m_monitor, m_win_x, m_win_y, m_width, m_height, mode->refreshRate);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cerr << "ERROR! FAILED TO INITIALIZE GLEW LIBRARY" << endl;
        glfwTerminate();
        return 0;
    }
#endif
    s_windowIdx++;
    s_cameraIdx++;

    return 1;
}

bool afCamera::assignWindowCallbacks(afCameraWindowCallBacks *a_callbacks)
{
    if (!m_window)
    {
        cout << "ERROR! FAILED TO CREATE OPENGL WINDOW" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 0;
    }

    if (a_callbacks->keyCallback){
        glfwSetKeyCallback(m_window, a_callbacks->keyCallback);
    }

    if (a_callbacks->mouseBtnsCallback){
        // set mouse buttons callback
        glfwSetMouseButtonCallback(m_window, a_callbacks->mouseBtnsCallback);
    }

    if (a_callbacks->mousePosCallback){
        //set mouse buttons callback
        glfwSetCursorPosCallback(m_window, a_callbacks->mousePosCallback);
    }

    if (a_callbacks->mouseScrollCallback){
        //set mouse scroll callback
        glfwSetScrollCallback(m_window, a_callbacks->mouseScrollCallback);
    }

    if (a_callbacks->windowSizeCallback){
        // set resize callback
        glfwSetWindowSizeCallback(m_window, a_callbacks->windowSizeCallback);

        // Initialize the window size
        a_callbacks->windowSizeCallback(m_window, m_width, m_height);
    }

    if (a_callbacks->dragDropCallback){
        // set drag and drop callback
        glfwSetDropCallback(m_window, a_callbacks->dragDropCallback);
    }

    return true;
}

///
/// \brief afCamera::publishDepthCPUBased
///
void afCamera::computeDepthOnCPU()
{

    m_frameBuffer->copyDepthBuffer(m_bufferDepthImage);

    cTransform projMatInv = m_camera->m_projectionMatrix;
    projMatInv.m_flagTransform = false;
    projMatInv.invert();

    int width = m_bufferDepthImage->getWidth();
    int height = m_bufferDepthImage->getHeight();
    int bbp = m_bufferDepthImage->getBytesPerPixel();

    double varScale = pow(2, sizeof(uint) * 8);

    for (int y_span = 0 ; y_span < height ; y_span++){
        double yImage = double(y_span) / (height - 1);
        for (int x_span = 0 ; x_span < width ; x_span++){
            double noise;
            if (m_depthNoise.isEnabled()){
                noise = m_depthNoise.generate();
            }
            else{
                noise = 0.0;
            }
            double xImage = double(x_span) / (width - 1);
            int idx = y_span * width + x_span;
            unsigned char b0 = m_bufferDepthImage->getData()[idx * bbp + 0];
            unsigned char b1 = m_bufferDepthImage->getData()[idx * bbp + 1];
            unsigned char b2 = m_bufferDepthImage->getData()[idx * bbp + 2];
            unsigned char b3 = m_bufferDepthImage->getData()[idx * bbp + 3];

            uint depth = uint (b3 << 24 | b2 << 16 | b1 << 8 | b0);
            double zImage = double (depth) / varScale;

            double xNDC = xImage * 2.0 - 1.0;
            double yNDC = yImage * 2.0 - 1.0;
            double zNDC = zImage * 2.0 - 1.0;
            double wNDC = 1.0;
            cVector3d pNDC = cVector3d(xNDC, yNDC, zNDC);
            cVector3d pClip = projMatInv * pNDC;
            double wClip = projMatInv(3, 0) * xNDC + projMatInv(3, 1) * yNDC + projMatInv(3, 2) * zNDC + projMatInv(3, 3) * wNDC;
            cVector3d pCam = cDiv(wClip, pClip);

            // Convert from OpenGL to AMBF coordinate frame.
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 0] = pCam.z() + noise;
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 1] = pCam.x();
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 2] = pCam.y();

        }
    }
}


///
/// \brief afCamera::renderDepthBuffer
///
void afCamera::computeDepthOnGPU()
{
    // Change the parent world for the camera so it only render's the depth quad (Mesh)
    m_camera->setParentWorld(m_dephtWorld);

    m_depthMesh->getShaderProgram()->setUniformi("diffuseMap", 3); // TextureUnit of frambuffer depth attachment

    cTransform invProj = m_camera->m_projectionMatrix;
    invProj.m_flagTransform = false;
    invProj.invert();

    m_depthMesh->getShaderProgram()->setUniform("invProjection", invProj, false);

    m_depthBuffer->renderView();

    m_camera->setParentWorld(m_afWorld->getChaiWorld());

    // bind texture
    glBindTexture(GL_TEXTURE_2D, m_depthBuffer->m_imageBuffer->getTextureId());

    // settings
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // copy data to depthPC
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, (GLvoid*)(m_depthPC.getData()));
}

///
/// \brief afCamera::getFrontLayer
/// \return
///
cWorld *afCamera::getFrontLayer(){
    return m_camera->m_frontLayer;
}


///
/// \brief afCamera::getBackLayer
/// \return
///
cWorld *afCamera::getBackLayer(){
    return m_camera->m_backLayer;
}


///
/// \brief afCamera::updatePositionFromDynamics
///
void afCamera::update(double dt)
{
}


///
/// \brief afCamera::updateLabels
/// \param options
///
void afCamera::updateLabels(afRenderOptions &options)
{
    // Not all labels change at every frame buffer.
    // We should prioritize the update of freqeunt labels

    // update haptic and graphic rate data
    string wallTimeStr = "Wall Time: " + cStr(m_afWorld->m_wallClock.getCurrentTimeSeconds(), 2) + " s";
    string simTimeStr = "Sim Time: " + cStr(m_afWorld->getSimulationTime(), 2) + " s";

    string graphicsFreqStr = "Gfx (" + cStr(m_afWorld->m_freqCounterGraphics.getFrequency(), 0) + " Hz)";
    string hapticFreqStr = "Phx (" + cStr(m_afWorld->m_freqCounterPhysics.getFrequency(), 0) + " Hz)";

    string timeLabelStr = wallTimeStr + " / " + simTimeStr;
    string dynHapticFreqLabelStr = graphicsFreqStr + " / " + hapticFreqStr;
    string modeLabelStr = "MODE: " + options.m_IIDModeStr;
    string btnLabelStr = " : " + options.m_IIDBtnActionStr;

    m_wallSimTimeLabel->setText(timeLabelStr);
    m_graphicsDynamicsFreqLabel->setText(dynHapticFreqLabelStr);
    m_devicesModesLabel->setText(modeLabelStr);
    m_deviceButtonLabel->setText(btnLabelStr);

    string controlling_dev_names;
    for (int devIdx = 0 ; devIdx < m_devHapticFreqLabels.size() ; devIdx++){
        m_devHapticFreqLabels[devIdx]->setLocalPos(10, (int)( m_height - ( devIdx + 1 ) * 20 ) );
        controlling_dev_names += m_controllingDevNames[devIdx] + " <> ";
    }

    m_controllingDeviceLabel->setText("Controlling Devices: [ " + controlling_dev_names + " ]");

    // update position of label
    m_wallSimTimeLabel->setLocalPos((int)(0.5 * (m_width - m_wallSimTimeLabel->getWidth() ) ), 30);
    m_graphicsDynamicsFreqLabel->setLocalPos((int)(0.5 * (m_width - m_graphicsDynamicsFreqLabel->getWidth() ) ), 10);
    m_devicesModesLabel->setLocalPos((int)(0.5 * (m_width - m_devicesModesLabel->getWidth())), 50);
    m_deviceButtonLabel->setLocalPos((int)(0.5 * (m_width - m_devicesModesLabel->getWidth()) + m_devicesModesLabel->getWidth()), 50);
    m_controllingDeviceLabel->setLocalPos((int)(0.5 * (m_width - m_controllingDeviceLabel->getWidth())), (int)(m_height - 20));

}

cCamera *afCamera::getInternalCamera(){
    return m_camera;
}

///
/// \brief afCamera::~afCamera
///
afCamera::~afCamera(){
    if (m_frameBuffer != nullptr){
        delete m_frameBuffer;
    }

    if (m_dephtWorld != nullptr){
        delete m_dephtWorld;
    }
}


///
/// \brief afCamera::render
/// \param options
///
void afCamera::render(afRenderOptions &options)
{
    if (getVisibleFlag()){

        // set current display context
        glfwMakeContextCurrent(m_window);

        // get width and height of window
        glfwGetFramebufferSize(m_window, &m_width, &m_height);

        // Update the Labels in a separate sub-routine
        if (options.m_updateLabels && !m_publishDepth && !m_publishImage){
            updateLabels(options);
        }

        renderSkyBox();

        // render world
        m_camera->renderView(m_width, m_height);

        // swap buffers
        glfwSwapBuffers(m_window);

        //    cerr << "Time Stamp Error: " << m_renderTimeStamp - getTimeStamp() << endl;

        // Only set the window_closed if the condition is met
        // otherwise a non-closed window will set the variable back
        // to false
        if (glfwWindowShouldClose(m_window)){
            options.m_windowClosed = true;
        }
    }

    renderFrameBuffer();

    m_sceneUpdateCounter++;
}


///
/// \brief afCamera::updateGlobalPose
/// \param a_forceUpdate
/// \param a_parentTransform
///
void afCamera::updateGlobalPose(bool a_forceUpdate, cTransform a_parentTransform)
{
    m_renderTimeStamp = getCurrentTimeStamp();
    afBaseObject::updateGlobalPose(a_forceUpdate, a_parentTransform);
}


///
/// \brief afCamera::renderSkyBox
///
void afCamera::renderSkyBox(){
    if (m_afWorld->m_skyBoxMesh != nullptr){
        if (m_afWorld->m_skyBoxMesh->getShaderProgram() != nullptr){
            cGenericObject* go;
            cRenderOptions ro;
            m_afWorld->m_skyBoxMesh->getShaderProgram()->use(go, ro);
            cMatrix3d rotOffsetPre(90, -90, 0, C_EULER_ORDER_XYZ, false, true);
            cMatrix3d rotOffsetPost(90, 90, 0, C_EULER_ORDER_ZYX, false, true);
            cTransform viewMat = rotOffsetPre * getLocalTransform() * rotOffsetPost;
            m_afWorld->m_skyBoxMesh->getShaderProgram()->setUniform("viewMat", viewMat, 1);
            m_afWorld->m_skyBoxMesh->getShaderProgram()->setUniform("projectionMat", m_camera->m_projectionMatrix, 0);
            m_afWorld->m_skyBoxMesh->getShaderProgram()->disable();
        }
    }
}


///
/// \brief afCamera::renderFrameBuffer
///
void afCamera::renderFrameBuffer(){
    if (m_publishImage || m_publishDepth){

        activatePreProcessingShaders();

        m_frameBuffer->renderView();
        m_frameBuffer->copyImageBuffer(m_bufferColorImage);

        deactivatePreProcessingShaders();
    }

    if (m_publishDepth){
        if (m_sceneUpdateCounter % m_depthPublishInterval == 0){
            if (m_useGPUForDepthComputation){
                computeDepthOnGPU();
            }
            else{
                computeDepthOnCPU();
            }
        }
    }
}


///
/// \brief afCamera::activatePreProcessingShaders
///
void afCamera::activatePreProcessingShaders()
{
    if (m_preprocessingShaderAttribs.m_shaderDefined){
        if (m_preprocessingShaderProgram.get()){
            preProcessingShadersUpdate();
            afBaseObjectMap::iterator rbIt;
            afBaseObjectMap* visualObjMap = m_afWorld->getRigidBodyMap();
            for (rbIt = visualObjMap->begin(); rbIt != visualObjMap->end() ; rbIt++){
                afRigidBodyPtr objPtr = (afRigidBodyPtr)rbIt->second;
                if (objPtr->m_visualMesh){
                    // Store the current shader Pgm
                    objPtr->backupShaderProgram();
                    objPtr->setShaderProgram(m_preprocessingShaderProgram);
                }
            }
            visualObjMap = m_afWorld->getGhostObjectMap();
            for (rbIt = visualObjMap->begin(); rbIt != visualObjMap->end() ; rbIt++){
                afGhostObjectPtr objPtr = (afGhostObjectPtr)rbIt->second;
                if (objPtr->m_visualMesh){
                    // Store the current shader Pgm
                    objPtr->backupShaderProgram();
                    objPtr->setShaderProgram(m_preprocessingShaderProgram);
                }
            }
            visualObjMap = m_afWorld->getSoftBodyMap();
            for (rbIt = visualObjMap->begin(); rbIt != visualObjMap->end() ; rbIt++){
                afSoftBodyPtr objPtr = (afSoftBodyPtr)rbIt->second;
                if (objPtr->m_visualMesh){
                    // Store the current shader Pgm
                    objPtr->backupShaderProgram();
                    objPtr->setShaderProgram(m_preprocessingShaderProgram);
                }
            }

            // Temporary. Set the voxel rendering mode that ignores lighting and
            // just renders the color of each voxel
            afBaseObjectMap::iterator vIt;
            afBaseObjectMap* vMap = m_afWorld->getVolumeMap();
            for (vIt = vMap->begin(); vIt != vMap->end() ; vIt++){
                afVolumePtr vPtr = (afVolumePtr)vIt->second;
                if (vPtr->getInternalVolume()){
                    // Store the current shader Pgm
                    vPtr->backupShaderProgram();
                    vPtr->getInternalVolume()->setRenderingModeVoxelColors();
                }
            }
        }
    }
}


///
/// \brief afCamera::deactivatePreProcessingShaders
///
void afCamera::deactivatePreProcessingShaders()
{
    if (m_preprocessingShaderAttribs.m_shaderDefined){
        if (m_preprocessingShaderProgram.get()){
            afBaseObjectMap::iterator objIt;
            afBaseObjectMap* visualObjMap = m_afWorld->getRigidBodyMap();
            for (objIt = visualObjMap->begin(); objIt != visualObjMap->end() ; objIt++){
                afRigidBodyPtr objPtr = (afRigidBody*)objIt->second;
                if (objPtr->m_visualMesh){
                    // Reassign the backedup shaderpgm for the next rendering pass
                    objPtr->restoreShaderProgram();
                }
            }
            visualObjMap = m_afWorld->getGhostObjectMap();
            for (objIt = visualObjMap->begin(); objIt != visualObjMap->end() ; objIt++){
                afGhostObjectPtr objPtr = (afGhostObjectPtr)objIt->second;
                if (objPtr->m_visualMesh){
                    // Reassign the backedup shaderpgm for the next rendering pass
                    objPtr->restoreShaderProgram();
                }
            }
            visualObjMap = m_afWorld->getSoftBodyMap();
            for (objIt = visualObjMap->begin(); objIt != visualObjMap->end() ; objIt++){
                afSoftBodyPtr objPtr = (afSoftBodyPtr)objIt->second;
                if (objPtr->m_visualMesh){
                    // Reassign the backedup shaderpgm for the next rendering pass
                    objPtr->restoreShaderProgram();
                }
            }
            visualObjMap = m_afWorld->getVolumeMap();
            for (objIt = visualObjMap->begin(); objIt != visualObjMap->end() ; objIt++){
                afVolumePtr volPtr = (afVolumePtr)objIt->second;
                if (volPtr->getInternalVolume()){
                    // Reassign the backedup shaderpgm for the next rendering pass
                    volPtr->restoreShaderProgram();
                }
            }
        }
    }
}

void afCamera::preProcessingShadersUpdate()
{
    cGenericObject* go;
    cRenderOptions po;
    m_preprocessingShaderProgram->use(go, po);

    //     Assign any shader attribs here.
    //     Example:
    //    m_preprocessingShaderProgram->setUniform("var_name", var);

    //     Also loop through visual objects to assign any specific object IDs etc.
    //     Example:
    //    afRigidBodyMap* rbMap = m_afWorld->getAFRigidBodyMap();
    //    for (rbIt = rbMap->begin(); rbIt != rbMap->end() ; rbIt++){
    //        afRigidBodyPtr rb = rbIt->second;
    //        if (rb->m_visualMesh){
    //            // Reassign the backedup shaderpgm for the next rendering pass
    //            rb->m_visualMesh->setShaderProgram(m_shaderProgramBackup[rb]);
    //        }
    //    }
}


///
/// \brief afCamera::enableImagePublishing
/// \param imageAttribs
///
void afCamera::enableImagePublishing(afImageResolutionAttribs* imageAttribs)
{
    createFrameBuffers(imageAttribs);
    m_publishImage = true;
}


///
/// \brief afCamera::enableDepthPublishing
/// \param imageAttribs
/// \param depthComputeShaderAttribs
/// \param noiseAtt
///
void afCamera::enableDepthPublishing(afImageResolutionAttribs* imageAttribs, afNoiseModelAttribs* noiseAtt, afShaderAttributes* depthComputeShaderAttribs)
{
    createFrameBuffers(imageAttribs);
    m_depthNoise.createFromAttribs(noiseAtt);

    // Set up the world
    m_dephtWorld = new cWorld();

    // Set up the frame buffer
    m_depthBuffer = new cFrameBuffer();
    
#ifndef GL_RGBA32F
#define GL_RGBA32F GL_RGBA32F_ARB
#endif
    
    m_depthBuffer->setup(m_camera, imageAttribs->m_width, imageAttribs->m_height, true, false, GL_RGBA32F);

    m_depthPC.setup(imageAttribs->m_width, imageAttribs->m_height, 3);

    // Set up the quad
    m_depthMesh = new cMesh();
    float quad[] = {
        // positions
        -1.0f,  1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f,  -1.0f, 0.0f,
        1.0f,  1.0f, 0.0f,
    };
    for (int vI = 0 ; vI < 2 ; vI++){
        int off = vI * 9;
        m_depthMesh->newTriangle(cVector3d(quad[off + 0], quad[off + 1], quad[off + 2]),
                cVector3d(quad[off + 3], quad[off + 4], quad[off + 5]),
                cVector3d(quad[off + 6], quad[off + 7], quad[off + 8]));
    }
    m_depthMesh->m_vertices->setTexCoord(0, 0.0, 1.0, 1.0);
    m_depthMesh->m_vertices->setTexCoord(1, 0.0, 0.0, 1.0);
    m_depthMesh->m_vertices->setTexCoord(2, 1.0, 0.0, 1.0);
    m_depthMesh->m_vertices->setTexCoord(3, 0.0, 1.0, 1.0);
    m_depthMesh->m_vertices->setTexCoord(4, 1.0, 0.0, 1.0);
    m_depthMesh->m_vertices->setTexCoord(5, 1.0, 1.0, 1.0);

    m_depthMesh->computeAllNormals();
    m_depthMesh->m_texture = m_frameBuffer->m_depthBuffer;
    m_depthMesh->setUseTexture(true);

    m_dephtWorld->addChild(m_depthMesh);
    m_dephtWorld->addChild(m_camera);

    m_depthBufferColorImage = cImage::create();
    m_depthBufferColorImage->allocate(m_publishImageResolution.m_width, m_publishImageResolution.m_height, GL_RGBA, GL_UNSIGNED_INT);

    cShaderProgramPtr shaderProgram;
    if (depthComputeShaderAttribs->m_shaderDefined){
        shaderProgram = afShaderUtils::createFromAttribs(depthComputeShaderAttribs, getQualifiedName(), "DEPTH_COMPUTE");
    }
    else{
        cerr << "INFO! USING INTERNALLY DEFINED DEPTH_COMPUTE SHADERS" << endl;
        shaderProgram = cShaderProgram::create(AF_DEPTH_COMPUTE_VTX, AF_DEPTH_COMPUTE_FRAG);
    }

    if (shaderProgram->linkProgram()){
        cGenericObject* go;
        cRenderOptions ro;
        shaderProgram->use(go, ro);
        m_depthMesh->setShaderProgram(shaderProgram);
        shaderProgram->disable();
        m_publishDepth = true;
    }
    else{
        cerr << "ERROR! FOR DEPTH_TO_PC2 FAILED TO COMPILE/LINK SHADER FILES: " << endl;
        m_publishDepth = false;
    }
}


///
/// \brief afCamera::makeWindowFullScreen
/// \param a_fullscreen
///
void afCamera::makeWindowFullScreen(bool a_fullscreen)
{
    if (getVisibleFlag()){
        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(m_monitor);

        // set fullscreen or window mode
        if (a_fullscreen)
        {
            glfwSetWindowMonitor(m_window, m_monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(0);
        }
        else
        {
            int w = 0.8 * mode->width;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(m_window, NULL, x, y, w, h, mode->refreshRate);

            int xpos, ypos;
            glfwGetMonitorPos(m_monitor, &xpos, &ypos);
            x += xpos; y += ypos;
            glfwSetWindowPos(m_window, x, y);
            glfwSwapInterval(0);
        }
    }
}


///
/// \brief afCamera::destroyWindow
///
void afCamera::destroyWindow()
{
    glfwDestroyWindow(m_window);
}



///
/// \brief afLight::afLight
///
afLight::afLight(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::LIGHT, a_afWorld, a_modelPtr){
}

bool afLight::createFromAttribs(afLightAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afLightAttributes &attribs = *a_attribs;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    bool valid = true;

    cTransform trans = to_cTransform(a_attribs->m_kinematicAttribs.m_location);
    setLocalTransform(trans);

    cVector3d dir = to_cVector3d(a_attribs->m_direction);
    setDir(dir);

    m_spotLight = new cSpotLight(m_afWorld->getChaiWorld());

    addChildSceneObject(m_spotLight, cTransform());

    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;

    if (m_parentName.empty() == false){
        m_afWorld->addObjectMissingParent(this);
    }

    m_initialTransform = getLocalTransform();

    m_spotLight->setSpotExponent(a_attribs->m_spotExponent);
    m_spotLight->setCutOffAngleDeg(a_attribs->m_cuttoffAngle * (180/3.14));
    m_spotLight->setShadowMapEnabled(true);

    switch (a_attribs->m_shadowQuality) {
    case afShadowQualityType::NO_SHADOW:
        m_spotLight->setShadowMapEnabled(false);
        break;
    case afShadowQualityType::VERY_LOW:
        m_spotLight->m_shadowMap->setQualityVeryLow();
        break;
    case afShadowQualityType::LOW:
        m_spotLight->m_shadowMap->setQualityLow();
        break;
    case afShadowQualityType::MEDIUM:
        m_spotLight->m_shadowMap->setQualityMedium();
        break;
    case afShadowQualityType::HIGH:
        m_spotLight->m_shadowMap->setQualityHigh();
        break;
    case afShadowQualityType::VERY_HIGH:
        m_spotLight->m_shadowMap->setQualityVeryHigh();
        break;
    }
    m_spotLight->setEnabled(true);

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getLightMap());
    setGlobalRemapIdx(remap_idx);

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    loadCommunicationPlugin(this, a_attribs);

    return valid;
}



///
/// \brief afLight::setDir COPIED FROM CDirectionalLight.cpp
/// \param a_direction
///
void afLight::setDir(const cVector3d &a_direction){
    // We arbitrarily point lights along the x axis of the stored
    // rotation matrix.
    cVector3d v0, v1, v2, z, y;
    a_direction.copyto(v0);

    // check vector
    if (v0.lengthsq() < 0.0001) { return; }

    // normalize direction vector
    v0.normalize();

    // compute 2 vector perpendicular to a_direction
    z.set(0.0, 0.0, 1.0);
    y.set(0.0, 1.0, 0.0);
    double a0 = cAngle(v0, z);
    double a1 = cAngle(v0, y);

    if (sin(a0) > sin(a1))
    {
        v0.crossr(z, v1);
        v0.crossr(v1, v2);
    }
    else
    {
        v0.crossr(y, v1);
        v0.crossr(v1, v2);
    }

    v1.normalize();
    v2.normalize();

    // update rotation matrix
    setLocalRot(cMatrix3d(v0,v1,v2));
}

void afLight::setCutOffAngle(double rad)
{
    m_spotLight->setCutOffAngleDeg(cRadToDeg(rad));
}


///
/// \brief afLight::getInternalLight
/// \return
///
cGenericLight *afLight::getInternalLight()
{
    return m_spotLight;
}


///
/// \brief afLight::updatePositionFromDynamics
///
void afLight::update(double dt)
{
}


afModel::afModel(afWorldPtr a_afWorld): afIdentification(afType::MODEL){
    //    m_pickDragVector = new cMesh();
    //    cCreateArrow(m_pickDragVector);
    //    m_pickDragVector->m_material->setPurpleAmethyst();
    //    m_pickDragVector->setShowEnabled(false);
    //    m_pickDragVector->setUseDisplayList(true);
    //    m_pickDragVector->markForUpdate(false);
    //    m_chaiWorld->addVisualMesh(m_pickDragVector);
    m_afWorld = a_afWorld;
}


/// Help from: https://stackoverflow.com/questions/1489830/efficient-way-to-determine-number-of-digits-in-an-integer
/// and https://stackoverflow.com/questions/11151548/get-the-number-of-digits-in-an-int/11151594
///
///
/// \brief afmodel::remapName
/// \param name
/// \param remap_idx_str
///
void afModel::remapName(string &name, string remap_idx_str){
    if (remap_idx_str.length() == 0){
        return;
    }
    int cur_idx = stoi(remap_idx_str);
    if (cur_idx == 1){
        name += remap_idx_str;
        return;
    }
    else{
        int n_digits = 1;
        while(cur_idx/=10){
            n_digits++;
        }
        name.erase(name.end() - n_digits, name.end());
        name += remap_idx_str;
    }
}

///
/// \brief afModel::createFromAttribs
/// \param a_attribs
/// \return
///
bool afModel::createFromAttribs(afModelAttributes *a_attribs)
{
    a_attribs->resolveRelativeNamespace();
    a_attribs->resolveRelativePathAttribs();

    setNamespace(a_attribs->m_identificationAttribs.m_namespace);
    setName(a_attribs->m_identificationAttribs.m_name);
    setIdentifier(a_attribs->m_identifier);

    bool enable_comm = a_attribs->m_enableComm;

    m_shaderAttribs = a_attribs->m_shaderAttribs;
    loadShaderProgram();

    // Loading Rigid Bodies
    for (size_t i = 0; i < a_attribs->m_rigidBodyAttribs.size(); ++i) {
        afRigidBodyPtr rBodyPtr = new afRigidBody(m_afWorld, this);
        if (rBodyPtr->createFromAttribs(&a_attribs->m_rigidBodyAttribs[i])){
            addRigidBody(rBodyPtr);
        }
    }

    // Loading Soft Bodies
    for (size_t i = 0; i < a_attribs->m_softBodyAttribs.size(); ++i) {
        afSoftBodyPtr sBodyPtr = new afSoftBody(m_afWorld, this);
        if (sBodyPtr->createFromAttribs(&a_attribs->m_softBodyAttribs[i])){
            addSoftBody(sBodyPtr);
        }
    }

    // Loading Ghost Objects
    for (size_t i = 0; i < a_attribs->m_ghostObjectAttribs.size(); ++i) {
        afGhostObjectPtr gObjPtr = new afGhostObject(m_afWorld, this);
        if (gObjPtr->createFromAttribs(&a_attribs->m_ghostObjectAttribs[i])){
            addGhostObject(gObjPtr);
        }
    }

    /// Loading Sensors
    for (size_t i = 0; i < a_attribs->m_sensorAttribs.size(); ++i) {
        afSensorPtr sensorPtr = nullptr;
        string type_str;
        bool valid = false;
        // Check which type of sensor is this so we can cast appropriately beforehand
        switch (a_attribs->m_sensorAttribs[i]->m_sensorType) {
        case afSensorType::RAYTRACER:
        {
            sensorPtr = new afProximitySensor(m_afWorld, this);
            type_str = "PROXIMITY";
            afRayTracerSensorAttributes* senAttribs = (afRayTracerSensorAttributes*) a_attribs->m_sensorAttribs[i];
            valid = ((afRayTracerSensor*)sensorPtr)->createFromAttribs(senAttribs);
            break;
        }
        case afSensorType::RESISTANCE:
        {
            sensorPtr = new afResistanceSensor(m_afWorld, this);
            type_str = "RESISTANCE";
            afResistanceSensorAttributes* senAttribs = (afResistanceSensorAttributes*) a_attribs->m_sensorAttribs[i];
            valid = ((afResistanceSensor*)sensorPtr)->createFromAttribs(senAttribs);
            break;
        }
        default:
            continue;
        }

        if (valid){
            addSensor(sensorPtr);
        }
    }

    // Loading Actuators
    for (size_t i = 0; i < a_attribs->m_actuatorAttribs.size(); ++i) {
        afActuatorPtr actuatorPtr = nullptr;
        string type_str;
        bool valid = false;
        switch (a_attribs->m_actuatorAttribs[i]->m_actuatorType) {
        case afActuatorType::CONSTRAINT:{
            actuatorPtr = new afConstraintActuator(m_afWorld, this);
            type_str = "CONSTRAINT";
            afConstraintActuatorAttributes* actAttribs = (afConstraintActuatorAttributes*)a_attribs->m_actuatorAttribs[i];
            valid = ((afConstraintActuator*)actuatorPtr)->createFromAttribs(actAttribs);
            break;
        }
        default:
            continue;
        }

        if (valid){
            addActuator(actuatorPtr);
        }
    }

    // Load Joints
    for (size_t i = 0; i < a_attribs->m_jointAttribs.size(); ++i) {
        afJointPtr jntPtr = new afJoint(m_afWorld, this);
        if (jntPtr->createFromAttribs(&a_attribs->m_jointAttribs[i])){
            addJoint(jntPtr);
        }
    }

    // Load Vehicles
    for (size_t i = 0; i < a_attribs->m_vehicleAttribs.size(); ++i) {
        afVehiclePtr vehiclePtr = new afVehicle(m_afWorld, this);
        if (vehiclePtr->createFromAttribs(&a_attribs->m_vehicleAttribs[i])){
            addVehicle(vehiclePtr);
        }
    }

    // Load Cameras
    for (size_t i = 0; i < a_attribs->m_cameraAttribs.size(); ++i) {
        afCameraPtr cameraPtr = new afCamera(m_afWorld, this);
        if (cameraPtr->createFromAttribs(&a_attribs->m_cameraAttribs[i])){
            addCamera(cameraPtr);
        }
    }

    // Load Lights
    for (size_t i = 0; i < a_attribs->m_lightAttribs.size(); ++i) {
        afLightPtr lightPtr = new afLight(m_afWorld, this);
        if (lightPtr->createFromAttribs(&a_attribs->m_lightAttribs[i])){
            addLight(lightPtr);
        }
    }

    // Load Volumes
    for (size_t i = 0; i < a_attribs->m_volumeAttribs.size(); ++i) {
        afVolumePtr volumePtr = new afVolume(m_afWorld, this);
        if (volumePtr->createFromAttribs(&a_attribs->m_volumeAttribs[i])){
            addVolume(volumePtr);
        }
    }

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    // This flag would ignore collision for all the multibodies in the scene

    if (a_attribs->m_ignoreInterCollision){
        ignoreCollisionChecking();
    }

    m_afWorld->buildCollisionGroups();

    return true;
}


///
/// \brief afModel::
/// \param pluginAttribs
/// \return
///
bool afModel::loadPlugins(afModelPtr modelPtr, afModelAttribsPtr attribs, vector<afPluginAttributes> *pluginAttribs)
{
    for (int i = 0 ; i < pluginAttribs->size(); i++){
        m_pluginManager.loadPlugin(modelPtr, attribs, (*pluginAttribs)[i].m_filename, (*pluginAttribs)[i].m_name, (*pluginAttribs)[i].m_path.c_str());
    }

    return true;
}


///
/// \brief afModel::update
/// \param dt
///
void afModel::update(double dt)
{
//    setTimeStamp(m_afWorld->getSystemTime());
    afChildrenMap::iterator cIt;
    for(cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->setTimeStamp(getWorldPtr()->getCurrentTimeStamp());
            childObj->update(dt);
        }
    }
}


///
/// \brief afModel::reset
///
void afModel::reset()
{
    afChildrenMap::iterator cIt;
    for(cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            oIt->second->reset();
        }
    }
}


///
/// \brief afModel::updateGlobalPose
///
void afModel::updateGlobalPose()
{
    afChildrenMap::iterator cIt;

    // Update global poses of all objects first
    for(cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->updateGlobalPose(false);
        }
    }

}


///
/// \brief afModel::updateSceneObjects
///
void afModel::updateSceneObjects()
{
    // Then update all scene objects
    for(afChildrenMap::iterator cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->updateSceneObjects();
        }
    }
}


///
/// \brief afModel::loadShaderProgram
///
void afModel::loadShaderProgram()
{
    if (m_shaderAttribs.m_shaderDefined){
        m_shaderProgram = afShaderUtils::createFromAttribs(&m_shaderAttribs, getQualifiedName(), "GLOBAL_SHADERS");
    }
}

afWorldPtr afModel::getWorldPtr()
{
    return m_afWorld;
}

///
/// \brief afModel::pluginsGraphicsUpdate
///
void afModel::pluginsGraphicsUpdate()
{
    m_pluginManager.graphicsUpdate();
    for(afChildrenMap::iterator cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->pluginsGraphicsUpdate();
        }
    }
}

///
/// \brief afModel::pluginsPhysicsUpdate
/// \param dt
///
void afModel::pluginsPhysicsUpdate(double dt)
{
    m_pluginManager.physicsUpdate(dt);
    for(afChildrenMap::iterator cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->pluginsPhysicsUpdate(dt);
        }
    }
}


///
/// \brief afModel::pluginsReset
///
void afModel::pluginsReset()
{
    m_pluginManager.reset();
    for(afChildrenMap::iterator cIt = m_childrenObjectsMap.begin(); cIt != m_childrenObjectsMap.end(); ++cIt)
    {
        for (afBaseObjectMap::iterator oIt = cIt->second.begin() ; oIt != cIt->second.end() ; ++oIt){
            afBaseObject* childObj = oIt->second;
            childObj->pluginsReset();
        }
    }
}


///
/// \brief afModel::removeCollisionChecking
///
void afModel::ignoreCollisionChecking(){

    /// Only ignore collision checking between the bodies
    /// defined in the specific model config file
    /// and not all the bodies in the world
    afBaseObjectMap::iterator rBodyItA = getRigidBodyMap()->begin();
    vector<btRigidBody*> rBodiesVec;
    rBodiesVec.resize(getRigidBodyMap()->size());
    uint i=0;
    for ( ; rBodyItA != getRigidBodyMap()->end() ; ++rBodyItA){
        rBodiesVec[i] = ((afRigidBodyPtr)rBodyItA->second)->m_bulletRigidBody;
        i++;
    }
    if (rBodiesVec.size() >0){
        for (uint i = 0 ; i < rBodiesVec.size() - 1 ; i++){
            for (uint j = i+1 ; j < rBodiesVec.size() ; j++){
                rBodiesVec[i]->setIgnoreCollisionCheck(rBodiesVec[j], true);
            }
        }
    }
}


///
/// \brief afModel::removeOverlappingCollisionChecking
///
void afModel::removeOverlappingCollisionChecking(){
    // This function checks all the constraints of each rigid body
    // if there are more than 1, it means that multiple bodies share each other
    // In this case, iteratively go over all the shared bodies and ignore their
    // collision if their common body has the same pivot
    afBaseObjectMap* rbMap = m_afWorld->getRigidBodyMap();
    afBaseObjectMap::iterator rBodyIt = rbMap->begin();
    vector<btRigidBody*> bodyFamily;
    pair<btVector3, btRigidBody*> pvtAandConnectedBody;
    vector< pair<btVector3, btRigidBody*> > pvtAandConnectedBodyVec;
    for ( ; rBodyIt != rbMap->end() ; ++rBodyIt){
        afRigidBodyPtr afBody = (afRigidBody*)rBodyIt->second;
        btRigidBody* rBody = afBody->m_bulletRigidBody;
        bodyFamily.clear();
        for(int cIdx = 0 ; cIdx < rBody->getNumConstraintRefs() ; cIdx++){
            if (rBody->getConstraintRef(cIdx)->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE){
                btHingeConstraint* joint = (btHingeConstraint*) rBody->getConstraintRef(cIdx);
                if (&joint->getRigidBodyA() == rBody){
                    pvtAandConnectedBody.first = joint->getAFrame().getOrigin();
                    pvtAandConnectedBody.second = &joint->getRigidBodyB();
                    pvtAandConnectedBodyVec.push_back(pvtAandConnectedBody);
                }
                else if (&joint->getRigidBodyB() == rBody){
                    pvtAandConnectedBody.first = joint->getBFrame().getOrigin();
                    pvtAandConnectedBody.second = &joint->getRigidBodyA();
                    pvtAandConnectedBodyVec.push_back(pvtAandConnectedBody);
                }
            }
        }
        if (pvtAandConnectedBodyVec.size() > 1){
            for (uint pvtIdx1 = 0 ; pvtIdx1 < pvtAandConnectedBodyVec.size() - 1 ; pvtIdx1++ ){
                btVector3 pvtA1 = pvtAandConnectedBodyVec[pvtIdx1].first;
                btRigidBody* connectedBodyA1 = pvtAandConnectedBodyVec[pvtIdx1].second;
                for (uint pvtIdx2 = pvtIdx1 + 1 ; pvtIdx2 < pvtAandConnectedBodyVec.size() ; pvtIdx2++ ){
                    btVector3 pvtA2 = pvtAandConnectedBodyVec[pvtIdx2].first;
                    btRigidBody* connectedBodyA2 = pvtAandConnectedBodyVec[pvtIdx2].second;
                    btVector3 diff = pvtA1 - pvtA2;
                    if (diff.length() < 0.1){
                        connectedBodyA1->setIgnoreCollisionCheck(connectedBodyA2, true);
                    }
                }

            }
        }
    }
}


///
/// \brief afModel::~afModel
///
afModel::~afModel(){
    //    afJointMap::const_iterator jIt = m_afJointMap.begin();
    //    for (; jIt != m_afJointMap.end() ; ++jIt){
    //        delete jIt->second;
    //    }
    //    afRigidBodyMap::iterator rIt = m_afRigidBodyMap.begin();
    //    for ( ; rIt != m_afRigidBodyMap.end() ; ++rIt){
    //        if (rIt->second)
    //            delete rIt->second;
    //    }
    //    afSoftBodyMap::const_iterator sIt = m_afSoftBodyMap.begin();
    //    for ( ; sIt != m_afSoftBodyMap.end() ; ++sIt){
    //        delete sIt->second;
    //    }
    m_pluginManager.close();
}

afVehicle::afVehicle(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(afType::VEHICLE, a_afWorld, a_modelPtr){
}

afVehicle::~afVehicle()
{
    if (m_vehicleRayCaster != nullptr){
        delete m_vehicleRayCaster;
    }

    if (m_vehicle != nullptr){
        delete m_vehicle;
    }

}


bool afVehicle::createFromAttribs(afVehicleAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afVehicleAttributes &attribs = *a_attribs;

    bool result = true;

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);

    setMinPublishFrequency(a_attribs->m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(a_attribs->m_communicationAttribs.m_maxPublishFreq);
    setPassive(a_attribs->m_communicationAttribs.m_passive);

    m_chassis = m_afWorld->getRigidBody(a_attribs->m_chassisBodyName);

    if (m_chassis == nullptr){
        result = false;
        return result;
    }

    // Get the inertial offset transform, so the wheels are offset properly.
    btTransform T_oInc = m_chassis->getInertialOffsetTransform();
    m_mass = m_chassis->getMass();
    m_inertia = m_chassis->getInertia();

    afPath high_res_filepath;

    m_numWheels = a_attribs->m_wheelAttribs.size();
    m_wheels.resize(m_numWheels);
    m_wheelAttribs = a_attribs->m_wheelAttribs;

    for (uint i = 0 ; i < m_numWheels ; i++){
        switch (m_wheelAttribs[i].m_representationType) {
        case afWheelRepresentationType::MESH:{
            m_wheels[i].m_mesh = new cMultiMesh();
            string meshFilepath = m_wheelAttribs[i].m_visualAttribs.m_meshFilepath.c_str();
            if (m_wheels[i].m_mesh->loadFromFile(meshFilepath)){
                m_afWorld->addSceneObjectToWorld(m_wheels[i].m_mesh);
                m_wheels[i].m_wheelRepresentationType = afWheelRepresentationType::MESH;
            }
            else{
                m_wheels[i].m_wheelRepresentationType = afWheelRepresentationType::INVALID;
                cerr << "ERROR! UNABLE TO FIND WHEEL IDX " << i << " MESH NAMED \"" << meshFilepath << "\" FOR VEHICLE \""
                     << m_name << "\", SKIPPING WHEEL!" << endl;
                continue;
            }
            break;
        }
        case afWheelRepresentationType::RIGID_BODY:{
            string rbName = m_wheelAttribs[i].m_wheelBodyName;
            m_wheels[i].m_wheelBody = m_afWorld->getRigidBody(rbName);
            if (m_wheels[i].m_wheelBody){
                // Since the wheel in the RayCast car in implicit. Disable the dynamic
                // properties of this wheel.
                m_wheels[i].m_wheelBody->setMass(0.0);
                btVector3 inertia(0, 0, 0);
                m_wheels[i].m_wheelBody->m_bulletRigidBody->setMassProps(0.0, inertia);
                // Print some info here to inform the user that we are setting the mass
                // and inertia to zero to make the wheel static.
                m_wheels[i].m_wheelRepresentationType = afWheelRepresentationType::RIGID_BODY;
            }
            else{
                m_wheels[i].m_wheelRepresentationType = afWheelRepresentationType::INVALID;
                cerr << "ERROR! UNABLE TO FIND WHEEL IDX " << i << " BODY NAMED \"" << rbName << "\" FOR VEHICLE \""
                     << m_name << "\", SKIPPING WHEEL!" << endl;
                continue;
            }
            break;
        }


        default:{
            m_wheels[i].m_wheelRepresentationType = afWheelRepresentationType::INVALID;
            cerr << "ERROR! UNABLE TO FIND \"MESH\" OR \"BODY\" FIELD FOR WHEEL OF VEHICLE \""
                 << m_name << "\", SKIPPING WHEEL!" << endl;
            continue;
        }
        };
    }

    m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_afWorld->m_bulletWorld);
    m_vehicle = new btRaycastVehicle(m_tuning, m_chassis->m_bulletRigidBody, m_vehicleRayCaster);

    m_chassis->m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_afWorld->m_bulletWorld->addVehicle(m_vehicle);

    m_vehicle->setCoordinateSystem(1, 2, 0);

    for (uint i = 0 ; i < m_numWheels ; i++){
        btVector3 off;
        off << m_wheelAttribs[i].m_offset;
        btVector3 dir;
        dir << m_wheelAttribs[i].m_downDirection;
        btVector3 axel_dir;
        axel_dir << m_wheelAttribs[i].m_axelDirection;

        off = T_oInc.inverse() * off;
        dir = T_oInc.getBasis().inverse() * dir;

        m_vehicle->addWheel(off, dir, axel_dir, m_wheelAttribs[i].m_suspensionAttribs.m_restLength, m_wheelAttribs[i].m_radius, m_tuning, m_wheelAttribs[i].m_isFront);
    }

    for (uint i = 0 ; i < m_numWheels ; i++){
        btWheelInfo& wheelInfo = m_vehicle->getWheelInfo(i);
        wheelInfo.m_suspensionStiffness = m_wheelAttribs[i].m_suspensionAttribs.m_stiffness;
        wheelInfo.m_wheelsDampingRelaxation = m_wheelAttribs[i].m_suspensionAttribs.m_damping;
        wheelInfo.m_wheelsDampingCompression = m_wheelAttribs[i].m_suspensionAttribs.m_compression;
        wheelInfo.m_frictionSlip = m_wheelAttribs[i].m_friction;
        wheelInfo.m_rollInfluence = m_wheelAttribs[i].m_rollInfluence;
    }

    string remap_idx = afUtils::getNonCollidingIdx(getQualifiedIdentifier(), m_afWorld->getVehicleMap());
    setGlobalRemapIdx(remap_idx);

    loadCommunicationPlugin(this, a_attribs);

    return result;
}

void afVehicle::engageBrake(){
    for (int i = 0 ; i < getWheelCount() ; i++){
        setWheelPower(i, 0.0);
        setWheelBrake(i, m_wheelAttribs[i].m_brakePowerMax);
    }
}

void afVehicle::releaseBrake()
{
    for (int i = 0 ; i < getWheelCount() ; i++){
        setWheelBrake(i, 0.0);
    }
}

void afVehicle::setWheelBrake(int i, double p){
    p = cClamp(p, 0.0, m_wheelAttribs[i].m_brakePowerMax);
    m_vehicle->setBrake(p, i);
}

void afVehicle::setWheelPower(int i, double p){
    p = cClamp(p, -m_wheelAttribs[i].m_enginePowerMax, m_wheelAttribs[i].m_enginePowerMax);
    m_vehicle->applyEngineForce(p, i);
}

void afVehicle::setWheelSteering(int i, double s){
    s = cClamp(s, m_wheelAttribs[i].m_steeringLimitMin, m_wheelAttribs[i].m_steeringLimitMax);
    m_vehicle->setSteeringValue(s, i);
}

void afVehicle::setChassisForce(btVector3 force){
    m_chassis->m_bulletRigidBody->applyCentralForce(force);
//    applyForce(force);
}

void afVehicle::setChassisTorque(btVector3 torque){
    m_chassis->m_bulletRigidBody->applyTorque(torque);
//    applyTorque(torque);
}


///
/// \brief afVehicle::updatePositionFromDynamics
///
void afVehicle::update(double dt){
    for (uint i = 0; i < m_numWheels ; i++){
        m_vehicle->updateWheelTransform(i, true);
        btTransform btTrans = m_vehicle->getWheelInfo(i).m_worldTransform;
        cTransform cTrans;
        cTrans << btTrans;
        if (m_wheels[i].m_wheelRepresentationType == afWheelRepresentationType::MESH){
            m_wheels[i].m_mesh->setLocalTransform(cTrans);
        }
        else if (m_wheels[i].m_wheelRepresentationType == afWheelRepresentationType::RIGID_BODY){
            //            m_wheels[i].m_wheelBody->m_bulletRigidBody->setWorldTransform(btTrans);
            m_wheels[i].m_wheelBody->m_bulletRigidBody->getMotionState()->setWorldTransform(btTrans);
        }
        else{
            // We have an invalid wheel. Skip.
        }

    }
}


///
/// \brief afDepthPointCloud::setup
/// \param a_width
/// \param a_height
/// \param a_numFields
/// \return
///
bool afDepthPointCloud::setup(uint a_width, uint a_height, uint a_numFields)
{
    m_width = a_width;
    m_height = a_height;
    m_numFields = a_numFields;
    m_data = (float*) malloc(m_width * m_height * m_numFields * sizeof(float));

    return true;
}


///
/// \brief afDepthPointCloud::~afDepthPointCloud
///
afDepthPointCloud::~afDepthPointCloud()
{
    if (m_data != nullptr){
        free(m_data);
    }
}

//------------------------------------------------------------------------------

afPointCloud::afPointCloud(afWorldPtr a_afWorld): afBaseObject(afType::POINT_CLOUD, a_afWorld)
{
    m_mpPtr = new cMultiPoint();
    m_afWorld->addSceneObjectToWorld(m_mpPtr);
}

afPointCloud::~afPointCloud()
{
    m_mpPtr->removeFromGraph();
    delete m_mpPtr;
}


void afPointCloud::update(double dt){
}

afGhostObject::afGhostObject(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(afType::GHOST_OBJECT, a_afWorld, a_modelPtr)
{
    m_bulletGhostObject = nullptr;
}

afGhostObject::~afGhostObject()
{
    if(m_bulletGhostObject){
        delete m_bulletGhostObject;
    }

    if(m_bulletGhostPairCallback){
        delete m_bulletGhostPairCallback;
        m_bulletGhostPairCallback = nullptr;
    }
}

void afGhostObject::update(double dt)
{
    //    cTransform trans;
    //    trans << m_bulletGhostObject->getWorldTransform();
    //    setLocalTransform(trans);
    m_bulletGhostObject->setWorldTransform(to_btTransform(m_globalTransform));
    vector<btRigidBody*> localSensedBodies;

    btManifoldArray* manifoldArray = new btManifoldArray();
    btBroadphasePairArray pairArray = m_bulletGhostObject->getOverlappingPairCache()->getOverlappingPairArray();

    for (int i = 0; i < pairArray.size(); i++)
    {
        manifoldArray->clear();

        btBroadphasePair pair = pairArray[i];

        //unless we manually perform collision detection on this pair, the contacts are in the dynamics world paircache:
        btBroadphasePair* collisionPair = m_afWorld->m_bulletWorld->getPairCache()->findPair(pair.m_pProxy0, pair.m_pProxy1);
        if (collisionPair == nullptr)
            continue;

        if (collisionPair->m_algorithm != nullptr){
            collisionPair->m_algorithm->getAllContactManifolds(*manifoldArray);
        }

        for (int j = 0; j < manifoldArray->size(); j++)
        {
            btPersistentManifold* manifold = manifoldArray->at(j);
            btCollisionObject* co;
            if (manifold->getBody0() == m_bulletGhostObject){
                co = const_cast<btCollisionObject*>(manifold->getBody1());
            }
            else{
                co = const_cast<btCollisionObject*>(manifold->getBody0());
            }

            for (int p = 0; p < manifold->getNumContacts(); p++)
            {
                btManifoldPoint pt = manifold->getContactPoint(p);
                if (pt.getDistance() < 0.0f)
                {
                    btRigidBody* pBody = dynamic_cast<btRigidBody*>(co);
                    if (pBody){
                        localSensedBodies.push_back(pBody);
                    }
                }
            }
        }
    }

    delete manifoldArray;

    for (int i = 0 ; i < m_sensedBodies.size() ; i++){
        m_sensedBodies[i]->setGravity(m_afWorld->m_bulletWorld->getGravity());
        m_sensedBodies[i]->applyCentralForce(btVector3(0, 0, 0));
        m_sensedBodies[i]->applyTorque(btVector3(0, 0, 0));
    }

    m_sensedBodies.clear();
    m_sensedBodies = localSensedBodies;


    for (int i = 0 ; i < m_sensedBodies.size() ; i++){
        if (m_sensedBodies[i]){
            m_sensedBodies[i]->setGravity(btVector3(0, 0, 0));
            double damping_factor = 1.0 - 0.1;
            btVector3 va(0, 0, 0);
            if (getParentObject()){
                afRigidBodyPtr parentBody = dynamic_cast<afRigidBodyPtr>(getParentObject());
                va = parentBody->m_bulletRigidBody->getLinearVelocity();
            }

            btVector3 vb = m_sensedBodies[i]->getLinearVelocity();
            double mag_va = va.length();
            btVector3 proj_vb_va(0, 0, 0);

            if (mag_va > 0.00001){
                proj_vb_va = va.normalized() * (vb.dot(va) / mag_va);
            }
            btVector3 orth_vb_va = vb - proj_vb_va;

            btVector3 wb =  m_sensedBodies[i]->getAngularVelocity();

            m_sensedBodies[i]->setLinearVelocity(damping_factor * orth_vb_va + va);
            m_sensedBodies[i]->setAngularVelocity(damping_factor * wb);
        }
    }
}

bool afGhostObject::createFromAttribs(afGhostObjectAttributes *a_attribs)
{
    bool valid = false;
    storeAttributes(a_attribs);

    setIdentifier(a_attribs->m_identifier);
    setName(a_attribs->m_identificationAttribs.m_name);
    setNamespace(a_attribs->m_identificationAttribs.m_namespace);
    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;

    if (m_parentName.empty() == false){
        m_afWorld->addObjectMissingParent(this);
    }

    createInertialObject();

    m_visualMesh = new cMultiMesh();
    m_collisionMesh = new cMultiMesh();
    m_collisionMesh->setShowEnabled(false);

    m_scale = a_attribs->m_kinematicAttribs.m_scale;

    if (afVisualUtils::createFromAttribs(&a_attribs->m_visualAttribs, m_visualMesh, m_name)){
        m_visualMesh->scale(m_scale);
    }
    else{
        // FAILED TO LOAD MESH. RETURN FALSE
        return 0;
    }

    if (a_attribs->m_collisionAttribs.m_geometryType == afGeometryType::MESH){
        if (m_collisionMesh->loadFromFile(a_attribs->m_collisionAttribs.m_meshFilepath.c_str()) ){
            m_collisionMesh->scale(m_scale);
            m_collisionMesh->removeDuplicateVertices();
            m_bulletCollisionShape = afShapeUtils::createCollisionShape(m_collisionMesh,
                                                                        a_attribs->m_collisionAttribs.m_margin,
                                                                        afTransform(),
                                                                        a_attribs->m_collisionAttribs.m_meshShapeType);
        }
        else{
            cerr << "WARNING! Body "
                 << m_name
                 << "'s mesh \"" << m_collisionMeshFilePath.c_str() << "\" not found\n";
            return false;
        }
    }
    else if(a_attribs->m_collisionAttribs.m_geometryType == afGeometryType::SINGLE_SHAPE ||
            a_attribs->m_collisionAttribs.m_geometryType == afGeometryType::COMPOUND_SHAPE){

        // A bug in Bullet where a plane shape appended to a compound shape doesn't collide with soft bodies.
        // Thus instead of using a compound, use the single collision shape.
        if (a_attribs->m_collisionAttribs.m_primitiveShapes.size() == 0){
            // ERROR! NO PRIMITIVE SHAPES HAVE BEEN DEFINED.
            return false;
        }
        else{
            btCompoundShape* compoundCollisionShape = new btCompoundShape();
            for (unsigned long sI = 0 ; sI < a_attribs->m_collisionAttribs.m_primitiveShapes.size() ; sI++){
                afPrimitiveShapeAttributes pS = a_attribs->m_collisionAttribs.m_primitiveShapes[sI];
                btCollisionShape* collShape = afShapeUtils::createCollisionShape(&pS, a_attribs->m_collisionAttribs.m_margin);

                // Here again, we consider both the inertial offset transform and the
                // shape offset transfrom. This will change the legacy behavior but
                // luckily only a few ADFs (i.e. -l 16,17 etc) use the compound collision
                // shape. So they shall be updated.
                btTransform shapeOffset = to_btTransform(pS.getOffset());
                compoundCollisionShape->addChildShape(shapeOffset, collShape);
            }
            m_bulletCollisionShape = compoundCollisionShape;
        }
    }

    if (m_bulletCollisionShape){
        m_bulletCollisionShape->setMargin(a_attribs->m_collisionAttribs.m_margin);
        m_bulletGhostObject->setCollisionShape(m_bulletCollisionShape);
        m_bulletGhostObject->setCollisionFlags(m_bulletGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        m_afWorld->m_bulletWorld->addCollisionObject(m_bulletGhostObject, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);

        if (m_bulletGhostPairCallback == nullptr){
            m_bulletGhostPairCallback = new btGhostPairCallback();
            m_afWorld->m_bulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_bulletGhostPairCallback);
        }

        cTransform trans = to_cTransform(a_attribs->m_kinematicAttribs.m_location);
        setInitialTransform(trans);
        setLocalTransform(trans);

        addChildSceneObject(m_visualMesh, cTransform());
        addChildSceneObject(m_visualMesh, cTransform());

        for (uint gI = 0 ; gI < a_attribs->m_collisionAttribs.m_groups.size() ; gI++){
            uint group =  a_attribs->m_collisionAttribs.m_groups[gI];
            // Sanity check for the group number
            if (group >= 0 && group <= 999){
                m_afWorld->m_collisionGroups[group].push_back(this);
                m_collisionGroups.push_back(group);
            }
            else{
                cerr << "WARNING! Ghost's "
                     << m_name
                     << "'s group number is \"" << group << "\" which should be between [0 - 999], ignoring\n";
            }
        }
        valid = true;
    }

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    return valid;
}

void afGhostObject::createInertialObject()
{
    m_bulletGhostObject = new btPairCachingGhostObject();
    m_bulletGhostObject->setUserPointer(this);
}

void afGhostObject::setLocalTransform(const cTransform &trans)
{
    m_bulletGhostObject->setWorldTransform(to_btTransform(trans));
    afBaseObject::setLocalTransform(trans);
}

cShaderProgramPtr afShaderUtils::createFromAttribs(afShaderAttributes *attribs, string objName, string type)
{
    cShaderProgramPtr shaderProgram;
    if (attribs->m_shaderDefined){
        string vtxShader = afUtils::loadFileContents(attribs->m_vtxFilepath.c_str());
        string fragShader = afUtils::loadFileContents(attribs->m_fragFilepath.c_str());
        shaderProgram = cShaderProgram::create(vtxShader, fragShader);
        if (shaderProgram->linkProgram()){
            cerr << "INFO! FOR OBJECT: "<< objName << ", LOADING SHADER TYPE " << type << " FROM FILES: " <<
                    "\n \t VERTEX: " << attribs->m_vtxFilepath.c_str() <<
                    "\n \t FRAGMENT: " << attribs->m_fragFilepath.c_str() << endl;
        }
        else{
            cerr << "ERROR! FOR OBJECT: "<< objName << ", FAILED TO LOAD SHADER TYPE " << type << " FROM FILES: " <<
                    "\n \t VERTEX: " << attribs->m_vtxFilepath.c_str() <<
                    "\n \t FRAGMENT: " << attribs->m_fragFilepath.c_str() << endl;
        }
    }
    return shaderProgram;
}

void afNoiseModel::createFromAttribs(afNoiseModelAttribs *a_attribs)
{
    m_attribs = *a_attribs;

    // Init Generator and Distribution

    m_randomNumberGenerator = default_random_engine(time(0));
    m_randomDistribution = new normal_distribution<double>(m_attribs.m_mean, m_attribs.m_std_dev);
}

///
/// \brief afVolume::afVolume
/// \param a_afWorld
/// \param a_modelPtr
///
afVolume::afVolume(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(afType::VOLUME, a_afWorld, a_modelPtr)
{
    clearResetFlag();
}


///
/// \brief afVolume::~afVolume
///
afVolume::~afVolume()
{

}

///
/// \brief afVolume::createFromAttribs
/// \param a_attribs
/// \return
///
bool afVolume::createFromAttribs(afVolumeAttributes *a_attribs)
{
    storeAttributes(a_attribs);
    afVolumeAttributes &attribs = *a_attribs;

    setNamespace(a_attribs->m_identificationAttribs.m_namespace);
    setName(a_attribs->m_identificationAttribs.m_name);
    setIdentifier(a_attribs->m_identifier);
    m_parentName = a_attribs->m_hierarchyAttribs.m_parentName;

    m_initialTransform << a_attribs->m_kinematicAttribs.m_location;
    setLocalTransform(m_initialTransform);
    m_scale = a_attribs->m_kinematicAttribs.m_scale;

    if (a_attribs->m_specificationType == afVolumeSpecificationType::MULTI_IMAGE){
        m_multiImage = cMultiImage::create();
        string path_and_prefix = a_attribs->m_multiImageAttribs.m_path.c_str() + "/" + a_attribs->m_multiImageAttribs.m_prefix;
        if (m_multiImage->loadFromFiles(path_and_prefix, a_attribs->m_multiImageAttribs.m_format, a_attribs->m_multiImageAttribs.m_count)){

            m_voxelObject = new cVoxelObject();
            // Setting transparency before setting the texture ensures that the rendering does not show empty spaces as black
            // and the depth point cloud is able to see the volume
            m_voxelObject->setTransparencyLevel(1.0);

            cTexture3dPtr texture = cTexture3d::create();
            texture->setImage(m_multiImage);
            m_voxelObject->setTexture(texture);
            // Copy the texture for reset
            m_originalTextureCopy = copy3DTexture(texture);

            // set the dimensions by assigning the position of the min and max corners
            m_minCornerInitial << ( a_attribs->m_dimensions / -2.0) * m_scale;
            m_maxCornerInitial << ( a_attribs->m_dimensions / 2.0) * m_scale;

            m_voxelObject->m_minCorner = m_minCornerInitial;
            m_voxelObject->m_maxCorner = m_maxCornerInitial;

            // set the texture coordinate at each corner.
            m_voxelObject->m_minTextureCoord.set(0.0, 0.0, 0.0);
            m_voxelObject->m_maxTextureCoord.set(1.0, 1.0, 1.0);

//            // set haptic properties
//            m_voxelObject->m_material->setStiffness(0.2 * 1);
//            m_voxelObject->m_material->setStaticFriction(0.0);
//            m_voxelObject->m_material->setDynamicFriction(0.0);

//            // enable materials
//            m_voxelObject->setUseMaterial(true);

//            // set material
//            m_voxelObject->m_material->setWhite();

            // set quality of graphic rendering
            m_voxelObject->setQuality(a_attribs->m_quality);

            m_voxelObject->setIsosurfaceValue(a_attribs->m_isosurfaceValue);
            m_voxelObject->setOpticalDensity(a_attribs->m_opticalDensity);

            addChildSceneObject(m_voxelObject, cTransform());

            cShaderProgramPtr shaderPgm = afShaderUtils::createFromAttribs(&a_attribs->m_shaderAttribs, m_name, "VOLUME");
            if (shaderPgm){
                m_voxelObject->setCustomShaderProgram(shaderPgm);
            }
            else{
                m_voxelObject->setRenderingModeIsosurfaceColors();
//                m_voxelObject->setRenderingModeVoxelColors();
            }
        }
        else{
            cerr << "ERROR! FAILED TO LOAD VOLUME FROM MULTI_IMAGES PATH: " << a_attribs->m_multiImageAttribs.m_path.c_str() << "/" << a_attribs->m_multiImageAttribs.m_prefix << endl;
            return false;
        }
    }

    if (m_parentName.empty() == false){
        m_afWorld->addObjectMissingParent(this);
    }
    if (a_attribs->m_colorAttribs.m_useMaterial){
        cMaterial mat = afMaterialUtils::createFromAttribs(&a_attribs->m_colorAttribs);
        m_voxelObject->setMaterial(mat);
    }

    loadPlugins(this, a_attribs, &a_attribs->m_pluginAttribs);

    return true;
}

///
/// \brief afVolume::update
/// \param dt
///
void afVolume::update(double dt)
{
}

void afVolume::updateSceneObjects()
{
    if (m_resetFlag){
        resetTextures();
    }
    afBaseObject::updateSceneObjects();
}

///
/// \brief afVolume::reset
///
void afVolume::reset()
{
    setResetFlag();
}

void afVolume::resetTextures()
{
    cTexture3dPtr tex = copy3DTexture(m_originalTextureCopy);
    m_voxelObject->setTexture(tex);
    tex->markForUpdate();
    afBaseObject::reset();
    clearResetFlag();
}

///
/// \brief afVolume::getShaderProgram
/// \return
///
cShaderProgramPtr afVolume::getShaderProgram()
{
    return m_voxelObject->getShaderProgram();
}

///
/// \brief afVolume::setShaderProgram
/// \param a_program
///
void afVolume::setShaderProgram(cShaderProgramPtr a_program)
{
    if (m_voxelObject){
        m_voxelObject->setShaderProgram(a_program);
    }
    else{
        cerr << "ERROR! VOLUME FOR THIS VOLUME OBJECT HAS NOT BEEN INITIALIZED YET. CAN'T SET SHADER PROGRAM" << endl;
    }
}


///
/// \brief afVolume::backupShaderProgram
///
void afVolume::backupShaderProgram()
{
   m_previousRenderingMode = m_voxelObject->getRenderingMode();
   m_prevLinearInterpolationFlag = m_voxelObject->getUseLinearInterpolation();
}


///
/// \brief afVolume::restoreShaderProgram
///
void afVolume::restoreShaderProgram()
{
    if (m_voxelObject){
        m_voxelObject->setRenderingMode(m_previousRenderingMode);
        m_voxelObject->setUseLinearInterpolation(m_prevLinearInterpolationFlag);
    }
}


///
/// \brief afVolume::getInternalVolume
/// \return
///
cVoxelObject* afVolume::getInternalVolume(){
    return m_voxelObject;
}


///
/// \brief afVolume::getDimensions
/// \return
///
cVector3d afVolume::getDimensions()
{
    cVector3d dim = m_maxCornerInitial - m_minCornerInitial;
    return dim;
}


///
/// \brief afVolume::getVoxelCount
/// \return
///
cVector3d afVolume::getVoxelCount()
{
    cVector3d count;
    count(0) = m_voxelObject->m_texture->m_image->getWidth();
    count(1) = m_voxelObject->m_texture->m_image->getHeight();
    count(2) = m_voxelObject->m_texture->m_image->getImageCount();
    return count;
}

///
/// \brief afVolume::getResolution
/// \return
///
cVector3d afVolume::getResolution()
{
    cVector3d count = getVoxelCount();
    cVector3d dims = getDimensions();
    cVector3d res;
    res(0) = dims(0) / count(0);
    res(1) = dims(1) / count(1);
    res(2) = dims(2) / count(2);
    return res;
}


///
/// \brief afVolume::localPosToVoxelIndex
/// \param pos
/// \param idx
/// \return
///
bool afVolume::localPosToVoxelIndex(cVector3d &pos, cVector3d& idx)
{

    idx = cVector3d(-1, -1, -1);
    if (pos.x() > getDimensions().x() || pos.y() > getDimensions().y() || pos.z() > getDimensions().z()){
        // Point is outside the voxel dimensions
        return false;
    }

    for (int i = 0 ; i < 3 ; i++){
        idx(i) = (pos(i) / getDimensions()(i) + 0.5) * getVoxelCount()(i);
    }


    // Sanity check.
    if (idx.x() > getVoxelCount().x() || idx.y() > getVoxelCount().y() || idx.z() > getVoxelCount().z()){
        // ERROR
        idx.set(-1, -1, -1);
        return false;
    }
    else{
        return true;
    }
}


///
/// \brief afVolume::voxelIndexToLocalPos
/// \param idx
/// \param pos
/// \return
///
bool afVolume::voxelIndexToLocalPos(cVector3d &idx, cVector3d &pos)
{
    // Init to invalid
    pos.set(1./0., 1./0., 1./0.);

    if (idx.x() > getVoxelCount().x() || idx.y() > getVoxelCount().y() || idx.z() > getVoxelCount().z()){
        // Index greater than voxel count
        return false;
    }

    for (int i = 0 ; i < 3 ; i++){
        pos(i) = idx(i) * getDimensions()(i) / getVoxelCount()(i);
    }

    // Sanity check.
    if (pos.x() > getDimensions().x() || pos.y() > getDimensions().y() || pos.z() > getDimensions().z()){
        // ERROR
        pos.set(1./0., 1./0., 1./0.);
        return false;
    }
    else{
        // Adjust pos
        pos -= getDimensions() / 2.0;
        return true;
    }
}


///
/// \brief afVolume::getVoxelValue
/// \param pos
/// \param color
/// \return
///
bool afVolume::getVoxelValue(cVector3d &pos, cColorb& color)
{
    cVector3d idx;
    if (localPosToVoxelIndex(pos, idx)){
        return m_voxelObject->m_texture->m_image->getVoxelColor(idx.x(), idx.y(), idx.z(), color);
    }
    else{
        // IDX not valid
        float i = -1.;
        color.set(i, i, i, i);
        return false;
    }
}

///
/// \brief afVolume::setVoxelValue
/// \param pos
/// \param color
/// \return
///
bool afVolume::setVoxelValue(cVector3d &pos, cColorb &color)
{
    cVector3d idx;
    if (localPosToVoxelIndex(pos, idx)){
        m_voxelObject->m_texture->m_image->setVoxelColor(idx.x(), idx.y(), idx.z(), color);
        return true;
    }
    else{
        // IDX not valid
        return false;
    }
}


///
/// \brief afVolume::backupTexture
///
void afVolume::backupTexture()
{
    m_backupTexture = copy3DTexture(m_voxelObject->m_texture);
}


///
/// \brief afVolume::restoreTexture
///
void afVolume::restoreTexture()
{
    m_voxelObject->setTexture(m_backupTexture);
    m_backupTexture->markForUpdate();
}


///
/// \brief afVolume::copy3DTexture
/// \param tex3D
/// \return
///
cTexture3dPtr afVolume::copy3DTexture(cTexture1dPtr tex3D)
{
    cTexture3dPtr copyTex = cTexture3d::create();
    copyTex = static_pointer_cast<cTexture3d>(tex3D)->copy();
    copyTex->m_image = static_pointer_cast<cMultiImage>(tex3D->m_image)->copy();
    return copyTex;
}
