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
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "afFramework.h"
#include "afConversions.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
//------------------------------------------------------------------------------

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
GLFWmonitor** afCamera::s_monitors;
int afCamera::s_numMonitors = 0;
int afCamera::s_numWindows = 0;
int afCamera::s_cameraIdx = 0;
int afCamera::s_windowIdx = 0;

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
ros::NodeHandle* afCamera::s_rosNode = nullptr;
image_transport::ImageTransport* afCamera::s_imageTransport = nullptr;
bool afCamera::s_imageTransportInitialized = false;
#endif
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
    return collisionShape;
}

btCollisionShape *afShapeUtils::createCollisionShape(const cMesh *a_collisionMesh,
                                                     double a_margin,
                                                     afMeshShapeType a_meshType)
{
    // create the collision shape
    btCollisionShape* collisionShape;

    switch (a_meshType) {
    case afMeshShapeType::CONCAVE_MESH:{
        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = a_collisionMesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int i=0; i<numTriangles; i++)
        {
            unsigned int vertexIndex0 = a_collisionMesh->m_triangles->getVertexIndex0(i);
            unsigned int vertexIndex1 = a_collisionMesh->m_triangles->getVertexIndex1(i);
            unsigned int vertexIndex2 = a_collisionMesh->m_triangles->getVertexIndex2(i);

            cVector3d vertex0 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex0);
            cVector3d vertex1 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex1);
            cVector3d vertex2 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex2);

            bulletMesh->addTriangle(btVector3(vertex0(0), vertex0(1), vertex0(2)),
                                    btVector3(vertex1(0), vertex1(1), vertex1(2)),
                                    btVector3(vertex2(0), vertex2(1), vertex2(2)));
        }

        // create mesh collision model
        collisionShape = new btGImpactMeshShape(bulletMesh);
        break;
    }
    case afMeshShapeType::CONVEX_MESH:{

        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = a_collisionMesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int i=0; i<numTriangles; i++)
        {
            unsigned int vertexIndex0 = a_collisionMesh->m_triangles->getVertexIndex0(i);
            unsigned int vertexIndex1 = a_collisionMesh->m_triangles->getVertexIndex1(i);
            unsigned int vertexIndex2 = a_collisionMesh->m_triangles->getVertexIndex2(i);

            cVector3d vertex0 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex0);
            cVector3d vertex1 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex1);
            cVector3d vertex2 = a_collisionMesh->m_vertices->getLocalPos(vertexIndex2);

            bulletMesh->addTriangle(btVector3(vertex0(0), vertex0(1), vertex0(2)),
                                    btVector3(vertex1(0), vertex1(1), vertex1(2)),
                                    btVector3(vertex2(0), vertex2(1), vertex2(2)));
        }

        // create mesh collision model
        collisionShape = new btConvexTriangleMeshShape(bulletMesh);
        break;
    }
    case afMeshShapeType::CONVEX_HULL:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        collisionShape = new btConvexHullShape((double*)(&a_collisionMesh->m_vertices->m_localPos[0]),
                a_collisionMesh->m_vertices->getNumElements(), sizeof(cVector3d));
        break;
    }
    default:
        break;
    }
    collisionShape->setMargin(a_margin);
    return collisionShape;
}


btCompoundShape *afShapeUtils::createCollisionShape(const cMultiMesh *a_collisionMultiMesh,
                                                    double a_margin,
                                                    afTransform m_inertialOffset,
                                                    afMeshShapeType a_meshType){
    // create the collision shape
    btCollisionShape* collisionShape;
    btCompoundShape* compoundCollisionShape = new btCompoundShape();
    btTransform inverseInertialOffsetTransform;
    inverseInertialOffsetTransform << m_inertialOffset.getInverse();

    switch (a_meshType) {
    case afMeshShapeType::CONCAVE_MESH:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
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
            collisionShape = new btGImpactMeshShape(bulletMesh);
            collisionShape->setMargin(a_margin);
            ((btGImpactMeshShape*) collisionShape)->updateBound();
            compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
        }
        break;
    }
    case afMeshShapeType::CONVEX_MESH:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
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
            collisionShape = new btConvexTriangleMeshShape(bulletMesh);
            compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
        }
        break;
    }
    case afMeshShapeType::CONVEX_HULL:{
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        for (it = a_collisionMultiMesh->m_meshes->begin(); it != a_collisionMultiMesh->m_meshes->end(); ++it)
        {
            cMesh* mesh = (*it);
            collisionShape = new btConvexHullShape((double*)(&mesh->m_vertices->m_localPos[0]), mesh->m_vertices->getNumElements(), sizeof(cVector3d));
            compoundCollisionShape->addChildShape(inverseInertialOffsetTransform, collisionShape);
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
cMaterial afMaterialUtils::createMaterialFromColor(afColorAttributes *a_color)
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

///
/// \brief afComm::afCreateCommInstance
/// \param type
/// \param a_name
/// \param a_namespace
/// \param a_min_freq
/// \param a_max_freq
/// \param time_out
///
void afComm::afCreateCommInstance(afObjectType type, string a_name, string a_namespace, int a_min_freq, int a_max_freq, double time_out){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    switch (type) {
    case afObjectType::ACTUATOR:
        m_afActuatorCommPtr.reset(new ambf_comm::Actuator(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::CAMERA:
        m_afCameraCommPtr.reset(new ambf_comm::Camera(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::LIGHT:
        m_afLightCommPtr.reset(new ambf_comm::Light(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::OBJECT:
        m_afObjectCommPtr.reset(new ambf_comm::Object(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::RIGID_BODY:
        m_afRigidBodyCommPtr.reset(new ambf_comm::RigidBody(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::SENSOR:
        m_afSensorCommPtr.reset(new ambf_comm::Sensor(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::VEHICLE:
        m_afVehicleCommPtr.reset(new ambf_comm::Vehicle(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afObjectType::WORLD:
        m_afWorldCommPtr.reset(new ambf_comm::World(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    default:
        cerr << "ERROR! COMMUNICATION TYPE FOR OBJECT NAMED " << a_name << " NOT IMPLEMENTED YET" << endl;
        break;
    }
    m_commType = type;
#endif
}


///
/// \brief afComm::afObjectSetTime
/// \param a_wall_time
/// \param a_sim_time
///
void afComm::afUpdateTimes(const double a_wall_time, const double a_sim_time){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afObjectCommPtr.get() != nullptr){
        m_afObjectCommPtr->set_wall_time(a_wall_time);
        m_afObjectCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afCameraCommPtr.get() != nullptr){
        m_afCameraCommPtr->set_wall_time(a_wall_time);
        m_afCameraCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afLightCommPtr.get() != nullptr){
        m_afLightCommPtr->set_wall_time(a_wall_time);
        m_afLightCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afSensorCommPtr.get() != nullptr){
        m_afSensorCommPtr->set_wall_time(a_wall_time);
        m_afSensorCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afActuatorCommPtr.get() != nullptr){
        m_afActuatorCommPtr->set_wall_time(a_wall_time);
        m_afActuatorCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afVehicleCommPtr.get() != nullptr){
        m_afVehicleCommPtr->set_wall_time(a_wall_time);
        m_afVehicleCommPtr->set_sim_time(a_sim_time);
    }
    if (m_afWorldCommPtr.get() != nullptr){
        m_afWorldCommPtr->set_wall_time(a_wall_time);
        m_afWorldCommPtr->set_sim_time(a_sim_time);
    }
#endif
}


///
/// \brief afComm::afObjectCommandExecute
/// \param dt
///
void afComm::fetchCommands(double dt){
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
    afCartesianControllerAttributes & attribs = *a_attribs;
    P_lin = attribs.P_lin;
    I_lin = attribs.I_lin;
    D_lin = attribs.D_lin;
    P_ang = attribs.P_ang;
    I_ang = attribs.I_ang;
    D_ang = attribs.D_ang;

    m_positionOutputType = attribs.m_positionOutputType;
    m_orientationOutputType = attribs.m_orientationOutputType;
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
}


///
/// \brief afObject::afObject
/// \param a_afWorld
///
afBaseObject::afBaseObject(afWorldPtr a_afWorld, afModelPtr a_afModel){
    m_afWorld = a_afWorld;
    m_modelPtr = a_afModel;
    m_parentObject = nullptr;
    m_visualMesh = nullptr;
}



///
/// \brief afObject::~afObject
///
afBaseObject::~afBaseObject(){

}

bool afBaseObject::createFromAttribs(afBaseObjectAttributes *a_attribs)
{

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

afBaseObjectPtr afBaseObject::getParentObject()
{
    return m_parentObject;
}


///
/// \brief afBaseObject::setLocalPos
/// \param pos
///
void afBaseObject::setLocalPos(const cVector3d &pos)
{
    return m_localTransform.setLocalPos(pos);
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

///
/// \brief afBaseObject::setLocalTransform
/// \param trans
///
void afBaseObject::setLocalTransform(const afTransform &trans)
{
    m_localTransform << trans;
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

void afBaseObject::removeAllChildSceneObjects(bool removeFromGraph){
    std::vector<afSceneObject*>::iterator it;

    for (it = m_childrenSceneObjects.begin() ; it < m_childrenSceneObjects.end() ; ++it){
        removeChildSceneObject((*it), removeFromGraph);
    }
}

void afBaseObject::updateSceneObjects(){
    // Assuming that the global pose was computed prior to this call.
    vector<afSceneObject*>::iterator it;
    for (it = m_childrenSceneObjects.begin() ; it != m_childrenSceneObjects.end() ; ++it){
        cTransform globalTrans = m_globalTransform * (*it)->getOffsetTransform();
        (*it)->getChaiObject()->setLocalTransform(globalTrans);
    }
}

void afBaseObject::updateGlobalPose(){
    cTransform a_globalTransform = m_localTransform;

    // Traverse up the parents to resolve the global pose
    afBaseObjectPtr a_parentObject = getParentObject();
    if (a_parentObject != nullptr){
        do{
            a_globalTransform = a_parentObject->getLocalTransform() * a_globalTransform;
            a_parentObject = a_parentObject->getParentObject();
        }
        while(a_parentObject != nullptr);
    }
    m_globalTransform = a_globalTransform;
}

void afBaseObject::showVisualFrame()
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

bool afBaseObject::resolveParenting(string a_parentName){
    // If the parent name is not empty, override the objects parent name
    if(a_parentName.empty() == false){
        m_parentName = a_parentName;
    }

    // If the parent is empty, indiciate error and return
    if (m_parentName.empty() == true){
        return false;
    }

    // Should generalize this to not be just a rigid body that we may parent to.
    afRigidBodyPtr pBody = m_afWorld->getAFRigidBody(m_parentName);
    if (pBody){
        pBody->addChildObject(this);
        return true;
    }
    else{
        cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
             << m_parentName << "\"" <<endl;
        return false;
    }
}


///
/// \brief afActuator::afActuator
/// \param a_afWorld
///
afActuator::afActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(a_afWorld, a_modelPtr){
}


///
/// \brief afConstraintActuator::afConstraintActuator
/// \param a_afWorld
///
afConstraintActuator::afConstraintActuator(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afActuator(a_afWorld, a_modelPtr){

}

bool afConstraintActuator::createFromAttribs(afConstraintActuatorAttributes *a_attribs)
{
    afConstraintActuatorAttributes& attribs = *a_attribs;

    bool result = true;

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    m_initialTransform = to_cTransform(attribs.m_kinematicAttribs.m_location);
    setLocalTransform(m_initialTransform);

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    m_showActuator = attribs.m_visible;

    // First search in the local space.
    m_parentBody = m_modelPtr->getAFRigidBodyLocal(m_parentName);

    if(!m_parentBody){
        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_modelPtr->getActuatorMap());
        m_parentBody = m_afWorld->getAFRigidBody(m_parentName + remap_idx);

        if (m_parentBody == nullptr){
            cerr << "ERROR: ACTUATOR'S "<< m_parentName + remap_idx << " NOT FOUND, IGNORING ACTUATOR\n";
            return 0;
        }
    }

    m_parentBody->addAFActuator(this);

    m_parentBody->addChildObject(this);

    m_maxImpulse = attribs.m_maxImpulse;
    m_tau = attribs.m_tau;

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFActuatorMap());

        afCreateCommInstance(afObjectType::ACTUATOR,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_afActuatorCommPtr->set_type("CONSTRAINT");
#endif
    }

    return result;
}


void afConstraintActuator::actuate(string a_rigid_body_name){

    afRigidBodyPtr body = m_afWorld->getAFRigidBody(a_rigid_body_name);
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

        actuate(a_rigidBody, P_aINc);
    }

}

void afConstraintActuator::actuate(string a_rigid_body_name, cVector3d a_bodyOffset){
    afRigidBodyPtr body = m_afWorld->getAFRigidBody(a_rigid_body_name);
    actuate(body, a_bodyOffset);
}

void afConstraintActuator::actuate(afRigidBodyPtr a_rigidBody, cVector3d a_bodyOffset){
    // Check if a constraint is already active
    if (m_constraint){
        // Check if the new requested actuation is the same as what is already
        // actuated. In this case simply ignore the request

        if (a_rigidBody == m_childBody){
            // We already have the same constraint. We can ignore the new request

            cerr << "INFO! ACTUATOR \"" << m_name << "\" IS ACTIVATED WITH THE SAME BODY AND OFFSET. THEREBY "
                                                          "IGNORING REQUEST \n";
            return;
        }
        else{
            // Deactuate the constraint first
            deactuate();
        }
    }

    if (a_rigidBody){
        btVector3 pvtA = to_btVector(getLocalPos());
        btVector3 pvtB = to_btVector(a_bodyOffset);
        m_childBody = a_rigidBody;
        m_constraint = new btPoint2PointConstraint(*m_parentBody->m_bulletRigidBody, *m_childBody->m_bulletRigidBody, pvtA, pvtB);
        m_constraint->m_setting.m_impulseClamp = m_maxImpulse;
        m_constraint->m_setting.m_tau = m_tau;
        m_afWorld->m_bulletWorld->addConstraint(m_constraint);
        m_active = true;
        return;
    }
    else{
        // We can warn that the requested body is in valid
    }
}

void afConstraintActuator::actuate(string a_softbody_name, int a_face_index){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, int a_face_index){

}

void afConstraintActuator::actuate(string a_softbody_name, int a_face_index, cVector3d a_bodyOffset){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, int a_face_index, cVector3d a_bodyOffset){

}

void afConstraintActuator::deactuate(){
    if (m_constraint){
        m_afWorld->m_bulletWorld->removeConstraint(m_constraint);
        delete m_constraint;

        m_constraint = nullptr;
        m_childBody = nullptr;
        m_childSotBody = nullptr;
        m_softBodyFaceIdx = -1;
    }
    m_active = false;
}


///
/// \brief afCartesianController::afExecuteCommand
/// \param dt
///
void afConstraintActuator::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afActuatorCommPtr.get() != nullptr){
        ambf_msgs::ActuatorCmd cmd = m_afActuatorCommPtr->get_command();

        if (cmd.actuate){
            if (m_active){
                // Constraint is active. Ignore request
                return;
            }
             string body_name = cmd.body_name.data;
            if (cmd.use_offset){
                cVector3d body_offset(cmd.body_offset.position.x,
                                      cmd.body_offset.position.y,
                                      cmd.body_offset.position.z);
                actuate(body_name, body_offset);
            }
            else{
                actuate(body_name);
            }
        }
        else{
            deactuate();
        }
    }
#endif
}


///
/// \brief afConstraintActuator::updatePositionFromDynamics
///
void afConstraintActuator::update(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afActuatorCommPtr.get() != nullptr){
        m_afActuatorCommPtr->set_name(m_name);
        m_afActuatorCommPtr->set_parent_name(m_parentName);
    }
#endif

}


///
/// \brief afInertialObject::afInertialObject
/// \param a_afWorld
///
afInertialObject::afInertialObject(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(a_afWorld, a_modelPtr)
{
    m_T_iINb.setIdentity();
    m_T_bINi.setIdentity();

    m_bulletRigidBody = nullptr;
    m_bulletSoftBody = nullptr;
    m_bulletCollisionShape = nullptr;
    m_bulletMotionState = nullptr;
}


///
/// \brief afInertialObject::~afInertialObject
///
afInertialObject::~afInertialObject()
{

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
afRigidBody::afRigidBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(a_afWorld, a_modelPtr){
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
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afRigidBodyCommPtr){
//        m_afRigidBodyPtr->cleanUp();
//        m_afRigidBodyPtr.reset();
    }
#endif

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
    afRigidBodyAttributes & attribs = *a_attribs;
    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);
    m_visualGeometryType = attribs.m_visualAttribs.m_geometryType;
    m_visualMeshFilePath = attribs.m_visualAttribs.m_meshFilepath;
    m_collisionGeometryType = attribs.m_collisionAttribs.m_geometryType;
    m_collisionMeshFilePath = attribs.m_collisionAttribs.m_meshFilepath;

    m_scale = attribs.m_kinematicAttribs.m_scale;

    m_visualMesh = new cMultiMesh();
    m_collisionMesh = new cMultiMesh();

    if (m_visualGeometryType == afGeometryType::MESH){
        if (m_visualMesh->loadFromFile(m_visualMeshFilePath.c_str()) ){
            m_visualMesh->scale(m_scale);
            m_visualMesh->setUseDisplayList(true);
//            m_visualMesh->markForUpdate(false);
        }
        else{
            cerr << "WARNING: Body "
                      << m_name
                      << "'s mesh \"" << m_visualMeshFilePath.c_str() << "\" not found\n";
            return 0;
        }
    }
    else if(m_visualGeometryType == afGeometryType::SINGLE_SHAPE ||
            m_visualGeometryType == afGeometryType::COMPOUND_SHAPE){

        for(unsigned long sI = 0 ; sI < attribs.m_visualAttribs.m_primitiveShapes.size() ; sI++){
            afPrimitiveShapeAttributes pS = attribs.m_visualAttribs.m_primitiveShapes[sI];
            cMesh* tempMesh = afShapeUtils::createVisualShape(&pS);;
            m_visualMesh->m_meshes->push_back(tempMesh);
        }
    }

    cMaterial mat = afMaterialUtils::createMaterialFromColor(&attribs.m_visualAttribs.m_colorAttribs);
    m_visualMesh->setMaterial(mat);
    // Important to set the transparency after setting the material, otherwise the alpha
    // channel ruins the Z-buffer depth testing in some way.
    m_visualMesh->setTransparencyLevel(attribs.m_visualAttribs.m_colorAttribs.m_alpha);

    m_shaderProgramDefined = attribs.m_shaderAttribs.m_shaderDefined;
    if (m_shaderProgramDefined){
        m_vtxShaderFilePath = attribs.m_shaderAttribs.m_vtxFilepath;
        m_fragShaderFilePath = attribs.m_shaderAttribs.m_fragFilepath;
        enableShaderProgram();
    }

    btTransform iOff;
    if (attribs.m_inertialAttribs.m_estimateInertialOffset){
        iOff.setOrigin(computeInertialOffset(m_collisionMesh));
    }

    // Set this now, but if we require the inertial offset to be estimated AND a collision
    // shape is a MESH, then estimate it to override this.
    btTransform inertialOffset = to_btTransform(attribs.m_inertialAttribs.m_inertialOffset);
    setInertialOffsetTransform(inertialOffset);

    if (m_collisionGeometryType == afGeometryType::MESH){
        if (m_collisionMesh->loadFromFile(m_collisionMeshFilePath.c_str()) ){
            m_collisionMesh->scale(m_scale);
            // Override the inertial offset if it is required by attribs
            if (attribs.m_inertialAttribs.m_estimateInertialOffset){
                btTransform inertialOffset;
                inertialOffset.setOrigin(computeInertialOffset(m_collisionMesh));
                setInertialOffsetTransform(inertialOffset);
            }

            m_bulletCollisionShape = afShapeUtils::createCollisionShape(m_collisionMesh,
                                                                        attribs.m_collisionAttribs.m_margin,
                                                                        attribs.m_inertialAttribs.m_inertialOffset,
                                                                        afMeshShapeType::CONCAVE_MESH);
        }
        else{
            cerr << "WARNING: Body "
                      << m_name
                      << "'s mesh \"" << m_collisionMeshFilePath.c_str() << "\" not found\n";
            return false;
        }
    }
    else if(m_collisionGeometryType == afGeometryType::SINGLE_SHAPE ||
            m_collisionGeometryType == afGeometryType::COMPOUND_SHAPE){

        // A bug in Bullet where a plane shape appended to a compound shape doesn't collide with soft bodies.
        // Thus instead of using a compound, use the single collision shape.
        if (attribs.m_collisionAttribs.m_primitiveShapes.size() == 0){
            // ERROR! NO PRIMITIVE SHAPES HAVE BEEN DEFINED.
            return false;
        }
        else if (attribs.m_collisionAttribs.m_primitiveShapes.size() == 1 && attribs.m_collisionAttribs.m_primitiveShapes[0].getShapeType() == afPrimitiveShapeType::PLANE){
            afPrimitiveShapeAttributes pS = attribs.m_collisionAttribs.m_primitiveShapes[0];
            m_bulletCollisionShape =  afShapeUtils::createCollisionShape(&pS, attribs.m_collisionAttribs.m_margin);
        }
        else{
            btCompoundShape* compoundCollisionShape = new btCompoundShape();
            for (unsigned long sI = 0 ; sI < attribs.m_collisionAttribs.m_primitiveShapes.size() ; sI++){
                afPrimitiveShapeAttributes pS = attribs.m_collisionAttribs.m_primitiveShapes[sI];
                btCollisionShape* collShape = afShapeUtils::createCollisionShape(&pS, attribs.m_collisionAttribs.m_margin);

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

    // The collision groups are sorted by integer indices. A group is an array of
    // ridig bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // the a body can be a part of multiple groups

    for (uint gI = 0 ; gI < attribs.m_collisionAttribs.m_groups.size() ; gI++){
        uint group =  attribs.m_collisionAttribs.m_groups[gI];
        // Sanity check for the group number
        if (group >= 0 && group <= 999){
            m_afWorld->m_collisionGroups[group].push_back(this);
            m_collisionGroups.push_back(group);
        }
        else{
            cerr << "WARNING: Body "
                      << m_name
                      << "'s group number is \"" << group << "\" which should be between [0 - 999], ignoring\n";
        }
    }

    m_controller.createFromAttribs(&attribs.m_controllerAttribs);

    setMass(attribs.m_inertialAttribs.m_mass);
    if(attribs.m_inertialAttribs.m_estimateInertia){
        estimateInertia();
    }
    else{
        setInertia(attribs.m_inertialAttribs.m_inertia);
    }

    // inertial origin in world
    cTransform T_iINw = to_cTransform(attribs.m_kinematicAttribs.m_location);
    cTransform T_mINw = T_iINw * to_cTransform(getInertialOffsetTransform());

    createInertialObject();

    setInitialTransform(T_mINw);
    setLocalTransform(T_mINw);

    setSurfaceProperties(attribs.m_surfaceAttribs);

    m_publish_children_names = attribs.m_publishChildrenNames;
    m_publish_joint_names = attribs.m_publishJointNames;
    m_publish_joint_positions = attribs.m_publishJointPositions;

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    addChildSceneObject(m_visualMesh, cTransform());
    m_afWorld->m_chaiWorld->addChild(m_visualMesh);
    m_afWorld->m_bulletWorld->addRigidBody(m_bulletRigidBody);

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFRigidBodyMap());

        afCreateCommInstance(afObjectType::RIGID_BODY,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }

    // Where to add the visual, collision and this object?
    return true;
}

void afRigidBody::createInertialObject()
{
    // create rigid body
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(m_mass, m_bulletMotionState, m_bulletCollisionShape, m_inertia);
    m_bulletRigidBody = new btRigidBody(rigidBodyCI);

    // by default deactivate sleeping mode
    m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_bulletRigidBody->setSleepingThresholds(0, 0);
}


///
/// \brief afRigidBody::enableShaderProgram
///
void afRigidBody::enableShaderProgram(){

    if (m_shaderProgramDefined){

        ifstream vsFile;
        ifstream fsFile;
        vsFile.open(m_vtxShaderFilePath.c_str());
        fsFile.open(m_fragShaderFilePath.c_str());
        // create a string stream
        stringstream vsBuffer, fsBuffer;
        // dump the contents of the file into it
        vsBuffer << vsFile.rdbuf();
        fsBuffer << fsFile.rdbuf();
        // close the files
        vsFile.close();
        fsFile.close();

        cShaderProgramPtr shaderProgram = cShaderProgram::create(vsBuffer.str(), fsBuffer.str());
        if (shaderProgram->linkProgram()){
            // Just empty Pts to let us use the shader
            cGenericObject* go=nullptr;
            cRenderOptions ro;
            shaderProgram->use(go, ro);
            // Set the ID for shadow and normal maps.
            shaderProgram->setUniformi("shadowMap", C_TU_SHADOWMAP);
            shaderProgram->setUniformi("normalMap", C_TU_NORMALMAP);
            shaderProgram->setUniformi("vEnableNormalMapping", 1);

            cerr << "INFO! FOR BODY: "<< m_name << ", USING SHADER FILES: " <<
                         "\n \t VERTEX: " << m_vtxShaderFilePath.c_str() <<
                         "\n \t FRAGMENT: " << m_fragShaderFilePath.c_str() << endl;

            m_visualMesh->setShaderProgram(shaderProgram);
        }
        else{
            cerr << "ERROR! FOR BODY: "<< m_name << ", FAILED TO LOAD SHADER FILES: " <<
                         "\n \t VERTEX: " << m_vtxShaderFilePath.c_str() <<
                         "\n \t FRAGMENT: " << m_fragShaderFilePath.c_str() << endl;

            m_shaderProgramDefined = false;
        }
    }
    // Check if the shader has been assigned by afWorld
    else if (m_visualMesh->getShaderProgram() != nullptr){
        m_shaderProgramDefined = true;
    }
    else{
        m_shaderProgramDefined = false;
    }
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
void afRigidBody::update()
{
    if (m_bulletRigidBody)
    {
        // Inertial and Mesh Transform
        btTransform T_iINw, T_mINw;
        m_bulletRigidBody->getMotionState()->getWorldTransform(T_iINw);
        T_mINw = T_iINw * getInverseInertialOffsetTransform();

        m_localTransform << T_mINw;
    }

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afRigidBodyCommPtr.get() != nullptr){
        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        cQuaternion q;
        q.fromRotMat(m_visualMesh->getLocalRot());

        // Update the Pose
        cVector3d localPos = getLocalPos();
        m_afRigidBodyCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
        m_afRigidBodyCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

        // Update the Wrench
        m_afRigidBodyCommPtr->cur_force(m_estimatedForce.x(), m_estimatedForce.y(), m_estimatedForce.z());
        m_afRigidBodyCommPtr->cur_torque(m_estimatedTorque.x(), m_estimatedTorque.y(), m_estimatedTorque.z());

        btVector3 v = m_bulletRigidBody->getLinearVelocity();
        btVector3 a = m_bulletRigidBody->getAngularVelocity();

        // Updated the Twist
        m_afRigidBodyCommPtr->cur_linear_velocity(v.x(), v.y(), v.z());
        m_afRigidBodyCommPtr->cur_angular_velocity(a.x(), a.y(), a.z());

        // Since the mass and inertia aren't going to change that often, write them
        // out intermittently
        if (m_write_count % 2000 == 0){
            m_afRigidBodyCommPtr->set_mass(getMass());
            m_afRigidBodyCommPtr->set_principal_inertia(getInertia().x(), getInertia().y(), getInertia().z());
        }

        ambf_msgs::RigidBodyCmd afCommand = m_afRigidBodyCommPtr->get_command();
        // We can set this body to publish it's children joint names in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_joint_names == true || afCommand.publish_joint_names == true){
            if (m_publish_joint_names == false){
                m_publish_joint_names = true;
                afObjectStateSetJointNames();
            }
            // Since joint names aren't going to change that often
            // change the field less so often
            if (m_write_count % 2000 == 0){
                afObjectStateSetJointNames();
            }
        }

        // We can set this body to publish joint positions in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_joint_positions == true || afCommand.publish_joint_positions == true){
            afObjectSetJointPositions();
            afObjectSetJointVelocities();
            afObjectSetJointEfforts();
        }

        // We can set this body to publish it's children names in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_children_names == true || afCommand.publish_children_names == true){
            if (m_publish_children_names == false){
                m_publish_children_names = true;
                afObjectStateSetChildrenNames();
            }
            // Since children names aren't going to change that often
            // change the field less so often
            if (m_write_count % 2000 == 0){

                afObjectStateSetChildrenNames();
                m_write_count = 0;
            }
        }


        m_write_count++;
    }
#endif
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
                m_afSensors[idx]->update();
            }

            m_threadUpdateFlags[threadIdx] = false;
        }
        usleep(1000);
    }
    return true;
}


///
/// \brief afRigidBody::afCommandExecute
/// \param dt
///
void afRigidBody::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afRigidBodyCommPtr.get() != nullptr){
        btVector3 force, torque;
        btVector3 lin_vel, ang_vel;
        ambf_msgs::RigidBodyCmd afCommand = m_afRigidBodyCommPtr->get_command();

        // IF THE COMMAND IS OF TYPE FORCE
        if (afCommand.cartesian_cmd_type == ambf_msgs::RigidBodyCmd::TYPE_FORCE){
            m_activeControllerType =  afControlType::FORCE;
            if (m_bulletRigidBody){
                force.setValue(afCommand.wrench.force.x,
                               afCommand.wrench.force.y,
                               afCommand.wrench.force.z);

                torque.setValue(afCommand.wrench.torque.x,
                                afCommand.wrench.torque.y,
                                afCommand.wrench.torque.z);

                m_bulletRigidBody->applyCentralForce(force);
                m_bulletRigidBody->applyTorque(torque);
            }
        }
        // IF THE COMMAND IS OF TYPE POSITION
        else if (afCommand.cartesian_cmd_type == ambf_msgs::RigidBodyCmd::TYPE_POSITION){
            m_activeControllerType = afControlType::POSITION;
            // If the body is kinematic, we just want to control the position
            if (m_bulletRigidBody->isStaticOrKinematicObject()){
                btTransform Tcommand;
                Tcommand.setOrigin(btVector3(afCommand.pose.position.x,
                                        afCommand.pose.position.y,
                                        afCommand.pose.position.z));

                Tcommand.setRotation(btQuaternion(afCommand.pose.orientation.x,
                                             afCommand.pose.orientation.y,
                                             afCommand.pose.orientation.z,
                                             afCommand.pose.orientation.w));

//                If the current pose is the same as before, ignore. Otherwise, update pose and collision AABB.
                if ((m_bulletRigidBody->getWorldTransform().getOrigin() - Tcommand.getOrigin()).norm() > 0.00001 ||
                        m_bulletRigidBody->getWorldTransform().getRotation().angleShortestPath(Tcommand.getRotation()) > 0.0001){
                    // Compensate for the inertial offset
                    Tcommand = Tcommand * getInertialOffsetTransform();
//                    cerr << "Updating Static Object Pose \n";
                    m_bulletRigidBody->getMotionState()->setWorldTransform(Tcommand);
                    m_bulletRigidBody->setWorldTransform(Tcommand);
                }

            }
            else{
                btVector3 cur_pos, cmd_pos;
                btQuaternion cmd_rot_quat = btQuaternion(afCommand.pose.orientation.x,
                                                         afCommand.pose.orientation.y,
                                                         afCommand.pose.orientation.z,
                                                         afCommand.pose.orientation.w);

                btMatrix3x3 cur_rot, cmd_rot;
                btTransform b_trans;
                m_bulletRigidBody->getMotionState()->getWorldTransform(b_trans);

                cur_pos = b_trans.getOrigin();
                cur_rot.setRotation(b_trans.getRotation());
                cmd_pos.setValue(afCommand.pose.position.x,
                                 afCommand.pose.position.y,
                                 afCommand.pose.position.z);
                if( cmd_rot_quat.length() < 0.9 || cmd_rot_quat.length() > 1.1 ){
                    cerr << "WARNING: BODY \"" << m_name << "'s\" rotation quaternion command"
                                                                 " not normalized" << endl;
                    if (cmd_rot_quat.length() < 0.1){
                        cmd_rot_quat.setW(1.0); // Invalid Quaternion
                    }
                }
                cmd_rot.setRotation(cmd_rot_quat);

                btVector3 pCommand, rCommand;
                // Use the internal Cartesian Position Controller to Compute Output
                pCommand = m_controller.computeOutput<btVector3>(cur_pos, cmd_pos, dt);
                // Use the internal Cartesian Rotation Controller to Compute Output
                rCommand = m_controller.computeOutput<btVector3>(cur_rot, cmd_rot, dt);

                if (m_controller.m_positionOutputType == afControlType::FORCE){
                    // IF PID GAINS WERE DEFINED, USE THE PID CONTROLLER
                    // Use the internal Cartesian Position Controller
                    m_bulletRigidBody->applyCentralForce(pCommand);
                    m_bulletRigidBody->applyTorque(rCommand);
                }
                else{
                    // ELSE USE THE VELOCITY INTERFACE
                    m_bulletRigidBody->setLinearVelocity(pCommand);
                    m_bulletRigidBody->setAngularVelocity(rCommand);

                }


            }
        }
        // IF THE COMMAND IS OF TYPE VELOCITY
        else if (afCommand.cartesian_cmd_type == ambf_msgs::RigidBodyCmd::TYPE_VELOCITY){
            m_activeControllerType = afControlType::VELOCITY;
            if (m_bulletRigidBody){
                lin_vel.setValue(afCommand.twist.linear.x,
                                 afCommand.twist.linear.y,
                                 afCommand.twist.linear.z);

                ang_vel.setValue(afCommand.twist.angular.x,
                                 afCommand.twist.angular.y,
                                 afCommand.twist.angular.z);

                m_bulletRigidBody->setLinearVelocity(lin_vel);
                m_bulletRigidBody->setAngularVelocity(ang_vel);
            }
        }

        size_t jntCmdSize = afCommand.joint_cmds.size();
        if (jntCmdSize > 0){
            size_t jntCmdCnt = m_CJ_PairsActive.size() < jntCmdSize ? m_CJ_PairsActive.size() : jntCmdSize;
            for (size_t jntIdx = 0 ; jntIdx < jntCmdCnt ; jntIdx++){
                // A joint can be controller in three different modes, Effort, Positon or Velocity.
                afJointPtr joint = m_CJ_PairsActive[jntIdx].m_childJoint;
                double jnt_cmd = afCommand.joint_cmds[jntIdx];
                if (afCommand.joint_cmds_types[jntIdx] == ambf_msgs::RigidBodyCmd::TYPE_FORCE){
                    joint->commandEffort(jnt_cmd);
                }
                else if (afCommand.joint_cmds_types[jntIdx] == ambf_msgs::RigidBodyCmd::TYPE_POSITION){
                    joint->commandPosition(jnt_cmd);
                }
                else if (afCommand.joint_cmds_types[jntIdx] == ambf_msgs::RigidBodyCmd::TYPE_VELOCITY){
                    joint->commandVelocity(jnt_cmd);
                }
                else{
                    cerr << "WARNING! FOR JOINT \"" <<
                                 m_CJ_PairsActive[jntIdx].m_childJoint->getName() <<
                                 " \" COMMAND TYPE NOT UNDERSTOOD, SUPPORTED TYPES ARE 0 -> FORCE, 1 -> POSITION, 2 -> VELOCITY " <<
                                 endl;
                }

            }
        }
    }
#endif
}


///
/// \brief afRigidBody::afObjectSetChildrenNames
///
void afRigidBody::afObjectStateSetChildrenNames(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_children = m_CJ_PairsActive.size();
    if (num_children > 0 && m_afRigidBodyCommPtr != NULL){
        vector<string> children_names;

        children_names.resize(num_children);
        for (size_t i = 0 ; i < num_children ; i++){
            children_names[i] = m_CJ_PairsActive[i].m_childBody->m_name;
        }
        m_afRigidBodyCommPtr->set_children_names(children_names);
    }
#endif
}


///
/// \brief afRigidBody::afObjectStateSetJointNames
///
void afRigidBody::afObjectStateSetJointNames(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_joints = m_CJ_PairsActive.size();
    if (num_joints > 0 && m_afRigidBodyCommPtr != NULL){
        vector<string> joint_names;
        joint_names.resize(num_joints);
        for (size_t i = 0 ; i < num_joints ; i++){
            joint_names[i] = m_CJ_PairsActive[i].m_childJoint->m_name;
        }
        m_afRigidBodyCommPtr->set_joint_names(joint_names);
    }
#endif
}


///
/// \brief afRigidBody::afObjectSetJointPositions
///
void afRigidBody::afObjectSetJointPositions(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_jnts = m_CJ_PairsActive.size();
    if (num_jnts > 0 && m_afRigidBodyCommPtr != NULL){
        if(m_joint_positions.size() != num_jnts){
            m_joint_positions.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_joint_positions[i] = m_CJ_PairsActive[i].m_childJoint->getPosition();
        }
        m_afRigidBodyCommPtr->set_joint_positions(m_joint_positions);
    }
#endif
}


///
/// \brief afRigidBody::afObjectSetJointVelocities
///
void afRigidBody::afObjectSetJointVelocities(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_jnts = m_CJ_PairsActive.size();
    if (num_jnts > 0 && m_afRigidBodyCommPtr != NULL){
        if(m_joint_velocities.size() != num_jnts){
            m_joint_velocities.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_joint_velocities[i] = m_CJ_PairsActive[i].m_childJoint->getVelocity();
        }
        m_afRigidBodyCommPtr->set_joint_velocities(m_joint_velocities);
    }
#endif
}


///
/// \brief afRigidBody::afObjectSetJointVelocities
///
void afRigidBody::afObjectSetJointEfforts(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_jnts = m_CJ_PairsActive.size();
    if (num_jnts > 0 && m_afRigidBodyCommPtr != NULL){
        if(m_joint_efforts.size() != num_jnts){
            m_joint_efforts.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_joint_efforts[i] = m_CJ_PairsActive[i].m_childJoint->getEffort();
        }
        m_afRigidBodyCommPtr->set_joint_efforts(m_joint_efforts);
    }
#endif
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
/// \brief afRigidBody::checkCollisionGroupIdx
/// \param a_idx
/// \return
///
bool afRigidBody::checkCollisionGroupIdx(uint a_idx){
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
/// \brief afRigidBody::isCommonCollisionGroupIdx
/// \param a_idx
/// \return
///
bool afRigidBody::isCommonCollisionGroupIdx(vector<uint> a_idx){
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

void afRigidBody::setLocalTransform(cTransform &trans)
{
    m_bulletMotionState->setWorldTransform(to_btTransform(trans));
    m_bulletRigidBody->setCenterOfMassTransform(to_btTransform(trans));
    afBaseObject::setLocalTransform(trans);
}

///
/// \brief afSoftBody::afSoftBody
/// \param a_afWorld
///
afSoftBody::afSoftBody(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(a_afWorld, a_modelPtr){
}


bool afSoftBody::createFromAttribs(afSoftBodyAttributes *a_attribs)
{
    afSoftBodyAttributes & attribs = *a_attribs;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    m_scale = attribs.m_kinematicAttribs.m_scale;

    btTransform iOff = to_btTransform(attribs.m_inertialAttribs.m_inertialOffset);
    setInertialOffsetTransform(iOff);

    m_visualMesh = new cMultiMesh();
    m_collisionMesh = new cMultiMesh();

    if (m_visualMesh->loadFromFile(attribs.m_visualAttribs.m_meshFilepath.c_str())){
        m_visualMesh->scale(m_scale);
        m_meshReductionSuccessful = false;
    }
    else
    {
        // If we can't find the visual mesh, we can proceed with
        // printing just an error and returning
        cerr << "ERROR! Soft Body " << m_name
             << "'s mesh " << attribs.m_visualAttribs.m_meshFilepath.c_str() << " not found\n";
        return 0;
    }

    if (m_collisionMesh->loadFromFile(attribs.m_collisionAttribs.m_meshFilepath.c_str())){
        m_collisionMesh->scale(m_scale);
        // Use the visual mesh for generating the softbody
        generateFromMesh(m_collisionMesh, attribs.m_collisionAttribs.m_margin);
//        cleanupMesh(m_visualMesh, m_vertexTree, m_trianglesPtr);
    }
    else
    {
        cerr << "ERROR! Soft Body " << m_name
             << "'s mesh " << attribs.m_collisionAttribs.m_meshFilepath.c_str() << " not found\n";
        return 0;
    }

    setMass(attribs.m_inertialAttribs.m_mass);
    m_bulletSoftBody->setTotalMass(m_mass, false);
    m_bulletSoftBody->getCollisionShape()->setUserPointer(m_bulletSoftBody);

    createInertialObject();

    cTransform pose = to_cTransform(attribs.m_kinematicAttribs.m_location);
    setLocalTransform(pose);

    cMaterial mat = afMaterialUtils::createMaterialFromColor(&attribs.m_visualAttribs.m_colorAttribs);
    m_visualMesh->setMaterial(mat);
    // Important to set the transparency after setting the material, otherwise the alpha
    // channel ruins the Z-buffer depth testing in some way.
    m_visualMesh->setTransparencyLevel(attribs.m_visualAttribs.m_colorAttribs.m_alpha);

    btSoftBody* softBody = m_bulletSoftBody;

    if (attribs.m_useMaterial){
        btSoftBody::Material *pm = softBody->appendMaterial();
//        pm->m_kLST = attribs.m_kLST;
//        pm->m_kAST = attribs.m_kAST;
//        pm->m_kVST = attribs.m_kVST;

        softBody->m_materials[0]->m_kLST = attribs.m_kLST;
        softBody->m_materials[0]->m_kAST = attribs.m_kAST;
        softBody->m_materials[0]->m_kVST = attribs.m_kVST;
    }

    if (attribs.m_usePoseMatching){
        softBody->m_cfg.kMT = attribs.m_kMT;
        softBody->setPose(false, true);
    }

    softBody->m_cfg.kVCF = attribs.m_kVCF;

    softBody->m_cfg.kDP = attribs.m_kDP;
    softBody->m_cfg.kDG = attribs.m_kDG;
    softBody->m_cfg.kLF = attribs.m_kLF;
    softBody->m_cfg.kPR = attribs.m_kPR;
    softBody->m_cfg.kVC = attribs.m_kVC;
    softBody->m_cfg.kDF = attribs.m_kDF;

    softBody->m_cfg.kCHR = attribs.m_kCHR;
    softBody->m_cfg.kKHR = attribs.m_kKHR;
    softBody->m_cfg.kSHR = attribs.m_kSHR;
    softBody->m_cfg.kAHR = attribs.m_kAHR;

    softBody->m_cfg.kSRHR_CL = attribs.m_kSRHR_CL;
    softBody->m_cfg.kSKHR_CL = attribs.m_kSKHR_CL;
    softBody->m_cfg.kSSHR_CL = attribs.m_kSSHR_CL;

    softBody->m_cfg.kSR_SPLT_CL = attribs.m_kSR_SPLT_CL;
    softBody->m_cfg.kSK_SPLT_CL = attribs.m_kSK_SPLT_CL;
    softBody->m_cfg.kSS_SPLT_CL = attribs.m_kSS_SPLT_CL;

    softBody->m_cfg.maxvolume = attribs.m_maxVolume;
    softBody->m_cfg.timescale = attribs.m_timeScale;

    softBody->m_cfg.viterations = attribs.m_vIterations;
    softBody->m_cfg.piterations = attribs.m_pIterations;
    softBody->m_cfg.diterations = attribs.m_dIterations;
    softBody->m_cfg.citerations = attribs.m_cIterations;

    softBody->m_cfg.collisions = attribs.m_flags;

    if (attribs.m_useBendingConstraints){
        softBody->generateBendingConstraints(attribs.m_bendingConstraint);
    }

    for (uint i = 0 ; i < attribs.m_fixedNodes.size() ; i++){
        uint nodeIdx = attribs.m_fixedNodes[i];
        if ( nodeIdx < softBody->m_nodes.size()){
            softBody->setMass(nodeIdx, 0);
        }
    }

    if(attribs.m_useClusters){
        softBody->generateClusters(attribs.m_clusters);
    }

    if (attribs.m_useConstraintRandomization){
        softBody->randomizeConstraints();
    }

    addChildSceneObject(m_visualMesh, cTransform());
    m_afWorld->m_chaiWorld->addChild(m_visualMesh);
    ((btSoftRigidDynamicsWorld*)m_afWorld->m_bulletWorld)->addSoftBody(m_bulletSoftBody);
    m_afWorld->m_bulletSoftBodyWorldInfo->m_sparsesdf.Reset();

    setPassive(true);

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFSoftBodyMap());

        afCreateCommInstance(afObjectType::SOFT_BODY,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }

    return true;
}

void afSoftBody::createInertialObject()
{
}

void afSoftBody::setLocalTransform(cTransform &trans)
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

    if (m_meshReductionSuccessful){
        for (int i = 0 ; i < m_bulletSoftBody->m_nodes.size() ; i++){
            btVector3 nodePos = m_bulletSoftBody->m_nodes[i].m_x;
            mesh->m_vertices->setLocalPos(i, nodePos[0], nodePos[1], nodePos[2]);
        }
    }
    else{
        for (int i = 0 ; i < m_vertexTree.size() ; i++){
            btVector3 nodePos = m_bulletSoftBody->m_nodes[i].m_x;
            for (int j = 0 ; j < m_vertexTree[i].vertexIdx.size() ; j++){
                int idx = m_vertexTree[i].vertexIdx[j];
                mesh->m_vertices->setLocalPos(idx, nodePos[0], nodePos[1], nodePos[2]);
            }
        }
    }
    afBaseObject::updateSceneObjects();
}

bool afSoftBody::cleanupMesh(cMultiMesh *multiMesh, std::vector<afSoftBody::VertexTree> &a_vertexTree, std::vector<unsigned int> &a_triangles)
{

    bool valid = true;
    //Store the elements of the mesh first.
    cMesh* reducedMesh = new cMesh();
    cMesh* tempMesh;
    reducedMesh->m_vertices->allocateData(a_vertexTree.size(), true, true, true, true, true, false);
    for (int i=0; i<a_vertexTree.size();i++){
        unsigned int resolved_idx;
        if (multiMesh->getVertex(a_vertexTree[i].vertexIdx[0], tempMesh, resolved_idx)){
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
        reducedMesh->setShowEdges(false);
        multiMesh->m_meshes->clear();
        multiMesh->addMesh(reducedMesh);
        m_meshReductionSuccessful = true;
    }

    return valid;
}

bool afSoftBody::generateFromMesh(cMultiMesh *multiMesh, const double a_margin)
{
    // create compound shape
    m_bulletCollisionShape = new btCompoundShape();;

    std::vector<cMesh*> *v_meshes;
    v_meshes = multiMesh->m_meshes;

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        cMesh* mesh = (*it);
        // read number of triangles of the object
        int numTriangles = mesh->m_triangles->getNumElements();
        std::vector<std::vector <int> > polyLines = mesh->m_lines;
//        computeUniqueVerticesandTriangles(mesh, &m_verticesPtr, &m_trianglesPtr, &_lines, true);
        computeUniqueVerticesandTrianglesSequential(mesh, &m_verticesPtr, &m_trianglesPtr, &polyLines, false);
        if (m_trianglesPtr.size() > 0){
            m_bulletSoftBody = createFromMesh(*m_afWorld->m_bulletSoftBodyWorldInfo,
                                              m_verticesPtr.data(), m_verticesPtr.size() / 3, m_trianglesPtr.data(), numTriangles);
            createLinksFromLines(m_bulletSoftBody, &polyLines, mesh);
        }
        else{
            m_bulletSoftBody = new btSoftBody(m_afWorld->m_bulletSoftBodyWorldInfo);
            /* Default material	*/
            btSoftBody::Material* pm = m_bulletSoftBody->appendMaterial();
            pm->m_kLST = 1;
            pm->m_kAST = 1;
            pm->m_kVST = 1;
            pm->m_flags = btSoftBody::fMaterial::Default;
            if (m_bulletSoftBody){
                m_bulletSoftBody->m_nodes.resize(mesh->m_vertices->getNumElements());
                for(int nIdx = 0 ; nIdx < m_bulletSoftBody->m_nodes.size() ; nIdx++){
                    btVector3 vPos = to_btVector(mesh->m_vertices->getLocalPos(nIdx));
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
    }
}

void afSoftBody::computeUniqueVerticesandTrianglesSequential(cMesh *mesh, std::vector<btScalar> *outputVertices, std::vector<unsigned int> *outputTriangles, std::vector<std::vector<int> > *outputLines, bool print_debug_info)
{
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
}


btSoftBody* afSoftBody::createFromMesh(btSoftBodyWorldInfo& worldInfo, const btScalar* vertices, int nNodes,
                                                 const unsigned int* triangles, int ntriangles, bool randomizeConstraints)
{
    unsigned int maxidx = 0;
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
        const unsigned int idx[] = {triangles[i], triangles[i + 1], triangles[i + 2]};
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
    afJointControllerAttributes& attribs = *a_attribs;

    m_P = attribs.m_P;
    m_I = attribs.m_I;
    m_D = attribs.m_D;
    m_maxImpulse = attribs.m_maxImpulse;
    m_outputType = attribs.m_outputType;

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
afJoint::afJoint(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(a_afWorld, a_modelPtr){
    m_posArray.resize(m_jpSize);
    m_dtArray.resize(m_jpSize);
}


bool afJoint::createFromAttribs(afJointAttributes *a_attribs)
{
    afJointAttributes &attribs = *a_attribs;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;
    m_childName = attribs.m_hierarchyAttribs.m_childName;
    m_enableActuator = attribs.m_enableMotor;
    m_controller.m_maxImpulse = attribs.m_maxMotorImpulse; // max rate of change of effort on Position Controllers
    m_offset = attribs.m_offset;
    m_lowerLimit = attribs.m_lowerLimit;
    m_upperLimit = attribs.m_upperLimit;
    //Default joint type is revolute if not type is specified
    m_jointType = attribs.m_jointType;
    m_damping = attribs.m_damping; // Initialize damping to 0

    // First we should search in the local Model space and if we don't find the body.
    // On then we find the world space

    string body1Name = m_namespace + m_parentName;
    string body2Name = m_namespace + m_childName;

    m_afParentBody = m_modelPtr->getAFRigidBodyLocal(body1Name, true);
    m_afChildBody = m_modelPtr->getAFRigidBodyLocal(body2Name, true);


    // If either body not found
    if (m_afParentBody == nullptr || m_afChildBody == nullptr){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_modelPtr->getJointMap());

        if (m_afParentBody == nullptr){
            m_afParentBody = m_afWorld->getAFRigidBody(body1Name + remap_idx, true);
        }
        if (m_afChildBody == nullptr){
            m_afChildBody = m_afWorld->getAFRigidBody(body2Name + remap_idx, true);
        }
    }

    // If we couldn't find the body with name_remapping, it might have been
    // Defined in another ambf file. Search without name_remapping string
    if(m_afParentBody == nullptr){
        m_afParentBody = m_afWorld->getAFRigidBody(m_parentName, true);
        // If a body is still not found, print error and ignore joint
        if (m_afParentBody == nullptr){
            cerr <<"ERROR: JOINT: \"" << m_name <<
                   "\'s\" PARENT BODY \"" << m_parentName <<
                   "\" NOT FOUND" << endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if (!(strcmp(m_afParentBody->m_name.c_str(), "world") == 0)
                && !(strcmp(m_afParentBody->m_name.c_str(), "World") == 0)
                && !(strcmp(m_afParentBody->m_name.c_str(), "WORLD") == 0)){
//            cerr <<"INFO: JOINT: \"" << m_name <<
//                   "\'s\" PARENT BODY \"" << m_parentName <<
//                   "\" FOUND IN ANOTHER AMBF CONFIG," << endl;
        }
    }
    if(m_afChildBody == nullptr){
        m_afChildBody = m_afWorld->getAFRigidBody(m_childName, true);
        // If any body is still not found, print error and ignore joint
        if (m_afChildBody == nullptr){
            cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" NOT FOUND" << endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ( !(strcmp(m_afChildBody->m_name.c_str(), "world") == 0)
                && !(strcmp(m_afChildBody->m_name.c_str(), "World") == 0)
                && !(strcmp(m_afChildBody->m_name.c_str(), "WORLD") == 0)){
            cerr <<"INFO: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" FOUND IN ANOTHER AMBF CONFIG," << endl;
        }
    }

    m_controller.createFromAttribs(&attribs.m_controllerAttribs);

    m_pvtA = to_btVector(attribs.m_parentPivot * m_afParentBody->m_scale);
    m_axisA = to_btVector(attribs.m_parentAxis);
    m_axisA.normalize();
    m_pvtA = m_afParentBody->getInverseInertialOffsetTransform() * m_pvtA;
    m_axisA = m_afParentBody->getInverseInertialOffsetTransform().getBasis() * m_axisA;

    m_pvtB = to_btVector(attribs.m_childPivot);
    m_axisB = to_btVector(attribs.m_childAxis);
    m_axisB.normalize();
    m_pvtB = m_afChildBody->getInverseInertialOffsetTransform() * m_pvtB;
    m_axisB = m_afChildBody->getInverseInertialOffsetTransform().getBasis() * m_axisB;

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    m_enableFeedback = attribs.m_enableFeedback;

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
    frameA.setRotation(quat_jINp);
    frameA.setOrigin(m_pvtA);

    // Rotation of child axis in parent axis as Quaternion
    btQuaternion quat_cINp;
    quat_cINp = afUtils::getRotBetweenVectors<btQuaternion, btVector3>(m_axisB, m_axisA);

    // Offset rotation along the parent axis
    btQuaternion quat_offINp;
    quat_offINp.setRotation(m_axisA, m_offset);
    // We need to post-multiply frameA's rot to cancel out the shift in axis, then
    // the offset along joint axis and finally frameB's axis alignment in frameA.
    frameB.setRotation( quat_cINp.inverse() * quat_offINp.inverse() * quat_jINp);
    frameB.setOrigin(m_pvtB);

    switch (m_jointType) {
    case afJointType::REVOLUTE:{
//        m_hinge = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
        m_hinge = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_hinge->setParam(BT_CONSTRAINT_ERP, attribs.m_erp);
        m_hinge->setParam(BT_CONSTRAINT_CFM, attribs.m_cfm);
        m_hinge->enableAngularMotor(false, 0.0, attribs.m_maxMotorImpulse);

        if(attribs.m_enableLimits){
            m_hinge->setLimit(m_lowerLimit, m_upperLimit);
        }
        m_btConstraint = m_hinge;
    }
        break;
    case afJointType::PRISMATIC:{
        m_slider = new btSliderConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_slider->setParam(BT_CONSTRAINT_ERP, attribs.m_erp);
        m_slider->setParam(BT_CONSTRAINT_CFM, attribs.m_cfm);

        if(attribs.m_enableLimits){
            m_slider->setLowerLinLimit(m_lowerLimit);
            m_slider->setUpperLinLimit(m_upperLimit);
        }
        // Ugly hack, divide by (default) fixed timestep to max linear motor force
        // since m_slider does have a max impulse setting method.
        m_slider->setMaxLinMotorForce(attribs.m_maxMotorImpulse / 0.001);
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
        m_spring = new btGeneric6DofSpringConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);

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

        if (attribs.m_enableLimits){
            // Somehow bullets springs limits for rotational joints are inverted.
            // So handle them internally rather than breaking AMBF description specificaiton
            if (m_jointType == afJointType::TORSION_SPRING){
                m_spring->setLimit(axisIdx, -m_upperLimit, -m_lowerLimit);
                m_spring->setEquilibriumPoint(-attribs.m_equilibriumPoint);
            }
            else{
                m_spring->setLimit(axisIdx, m_lowerLimit, m_upperLimit);
                m_spring->setEquilibriumPoint(attribs.m_equilibriumPoint);
            }

            m_spring->enableSpring(axisIdx, true);
        }

        m_spring->setStiffness(axisIdx, attribs.m_stiffness);
        m_spring->setDamping(axisIdx, m_damping);
        m_spring->setParam(BT_CONSTRAINT_STOP_ERP, attribs.m_erp, axisIdx);
        m_spring->setParam(BT_CONSTRAINT_CFM, attribs.m_cfm, axisIdx);
        m_btConstraint = m_spring;
    }
        break;
    case afJointType::P2P:{
        // p2p joint doesnt concern itself with rotations, its set using just the pivot information
        m_p2p = new btPoint2PointConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB);
        m_p2p->setParam(BT_CONSTRAINT_ERP, attribs.m_erp);
        m_p2p->setParam(BT_CONSTRAINT_CFM, attribs.m_cfm);
        m_btConstraint = m_p2p;

    }
        break;
    default:
        break;
    }

    if (m_btConstraint != nullptr){
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, attribs.m_ignoreInterCollision);
        m_afParentBody->addChildBodyJointPair(m_afChildBody, this);

        if (m_enableFeedback){
            m_btConstraint->enableFeedback(m_enableFeedback);
            m_feedback = new btJointFeedback();
            m_btConstraint->setJointFeedback(m_feedback);
        }
    }

    // Forcefully set passive for now.
    setPassive(true);
    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFJointMap());

        afCreateCommInstance(afObjectType::JOINT,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }

    return true;
}

btVector3 afJoint::getDefaultJointAxisInParent(afJointType a_type)
{
    btVector3 jINp;
    switch (a_type) {
    case afJointType::REVOLUTE:
    case afJointType::FIXED:
    case afJointType::LINEAR_SPRING:
    case afJointType::TORSION_SPRING:
    case afJointType::P2P:
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


void afJoint::remove(){
    if (m_btConstraint){
        m_afWorld->m_bulletWorld->removeConstraint(m_btConstraint);
    }
}


///
/// \brief afJoint::applyDamping
///
void afJoint::applyDamping(const double &dt){
    // First lets configure what type of joint is this.
    for (uint i = 0 ; i < m_jpSize-1 ; i++){
        m_posArray[i] = m_posArray[i+1];
        m_dtArray[i] = m_dtArray[i+1];
    }
    m_posArray[m_jpSize-1] = getPosition();
    m_dtArray[m_jpSize-1] = dt;
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
            btClamp(position_cmd, m_lowerLimit, m_upperLimit);
            double position_cur = getPosition();
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
    if (m_jointType == afJointType::REVOLUTE)
        return m_hinge->getHingeAngle();
    else if (m_jointType == afJointType::PRISMATIC)
        return m_slider->getLinearPos();
    else if (m_jointType == afJointType::FIXED)
        return 0;
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
        return 1.0 * angle; // Using the -1.0 since we always use bodyA as reference frame
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
        return -1.0 * angle; // Using the -1.0 since we always use bodyA as reference frame
    }
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
    double vel = (p_a - p_b) / dt_n;
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
afSensor::afSensor(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afBaseObject(a_afWorld, a_modelPtr){
}


///
/// \brief afSensor::afExecuteCommand
/// \param dt
///
void afSensor::fetchCommands(double dt){

}


///
/// \brief afSensor::updatePositionFromDynamics
///
void afSensor::update(){

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
        mesh->setShowEnabled(true);
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
        mesh->setShowEnabled(true);
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
    afRayTracerSensorAttributes &attribs = *a_attribs;

    bool result = true;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;
    m_localTransform << attribs.m_kinematicAttribs.m_location;

    m_range = attribs.m_range;

    if (m_range < 0.0){
        cerr << "ERROR! SENSOR RANGE CANNOT BE NEGATIVE" << endl;
        return 0;
    }

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    m_showSensor = attribs.m_visible;
    m_visibilitySphereRadius = attribs.m_visibleSize;

    // First search in the local space.
    m_parentBody = m_modelPtr->getAFRigidBodyLocal(m_parentName);

    if(m_parentBody == nullptr){
        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_modelPtr->getSensorMap());
        m_parentBody = m_afWorld->getAFRigidBody(m_parentName + remap_idx);
        if (m_parentBody == nullptr){
            cerr << "ERROR: SENSOR'S "<< m_parentName + remap_idx << " NOT FOUND, IGNORING SENSOR\n";
            return 0;
        }
    }

    m_parentBody->addAFSensor(this);
    m_parentBody->addChildObject(this);

    switch (attribs.m_specificationType) {
    case afSensactorSpecificationType::ARRAY:
    case afSensactorSpecificationType::PARAMETRIC:{
        m_count = attribs.m_raysAttribs.size();
        m_raysAttribs = attribs.m_raysAttribs;
        m_rayTracerResults.resize(m_count);
        break;
    }
    case afSensactorSpecificationType::MESH:{
        cMultiMesh* contourMesh = new cMultiMesh();
        if (contourMesh->loadFromFile(attribs.m_contourMeshFilepath.c_str())){
            m_raysAttribs = afShapeUtils::createRayAttribs(contourMesh, m_range);
            delete contourMesh;
            result = true;
        }
        else{
            cerr << "ERROR! BODY \"" << m_name << "\'s\" RESISTIVE MESH " <<
                    attribs.m_contourMeshFilepath.c_str() << " NOT FOUND. IGNORING\n";
            result = false;
        }
        break;
    }
    default:
        break;
    }

    if (m_showSensor){
        enableVisualization();
    }

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFSensorMap());

        afCreateCommInstance(afObjectType::SENSOR,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_afSensorCommPtr->set_type("PROXIMITY");
#endif
    }

    return result;
}


///
/// \brief afRayTracerSensor::visualize
///
void afRayTracerSensor::enableVisualization(){
    for (uint i = 0 ; i < m_count ; i++){
        m_rayTracerResults[i].enableVisualization(this, &m_raysAttribs[i], m_visibilitySphereRadius);
    }
}

///
/// \brief afRayTracerSensor::updatePositionFromDynamics
///
void afRayTracerSensor::update(){

    if (m_parentBody == nullptr){
        return;
    }
    cTransform T_bINw = getGlobalTransform();
    for (uint i = 0 ; i < m_count ; i++){
        btVector3 rayFromWorld, rayToWorld;
        rayFromWorld << T_bINw * to_cVector3d(m_raysAttribs[i].m_rayFromLocal);
        rayToWorld << T_bINw * to_cVector3d(m_raysAttribs[i].m_rayToLocal);

        btCollisionWorld::ClosestRayResultCallback rayCallBack(rayFromWorld, rayToWorld);
        m_afWorld->m_bulletWorld->rayTest(rayFromWorld, rayToWorld, rayCallBack);
        if (rayCallBack.hasHit()){
            if (m_showSensor){
                cVector3d Ph;
                Ph << rayCallBack.m_hitPointWorld;
                m_rayTracerResults[i].m_hitSphereMesh->setLocalPos(Ph);
                m_rayTracerResults[i].m_hitSphereMesh->setShowEnabled(true);
            }
            m_rayTracerResults[i].m_triggered = true;
            if (rayCallBack.m_collisionObject->getInternalType()
                    == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
                m_rayTracerResults[i].m_sensedBTRigidBody = (btRigidBody*)btRigidBody::upcast(rayCallBack.m_collisionObject);
                m_rayTracerResults[i].m_sensedAFRigidBody = m_afWorld->getAFRigidBody(m_rayTracerResults[i].m_sensedBTRigidBody);
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
                    m_rayTracerResults[i].m_sensedAFSoftBody = m_afWorld->getAFSoftBody(m_rayTracerResults[i].m_sensedBTSoftBody);
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
            if(m_showSensor){
                m_rayTracerResults[i].m_hitSphereMesh->setShowEnabled(false);
            }
            m_rayTracerResults[i].m_triggered = false;
            m_rayTracerResults[i].m_depthFraction = 0;
        }
    }

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    m_afSensorCommPtr->set_count(m_count);
    m_afSensorCommPtr->set_name(m_name);
    m_afSensorCommPtr->set_parent_name(m_parentName);
    m_afSensorCommPtr->set_range(m_range);
    cVector3d pos = getLocalPos();
    cMatrix3d rot = getLocalRot();
    cQuaternion quat;
    quat.fromRotMat(rot);
    m_afSensorCommPtr->cur_position(pos.x(), pos.y(), pos.z());
    m_afSensorCommPtr->cur_orientation(quat.x, quat.y, quat.z, quat.w);

    vector<bool> triggers;
    triggers.resize(m_count);

    vector<string> sensed_obj_names;
    sensed_obj_names.resize(m_count);

    vector<double> measurements;
    measurements.resize(m_count);

    for (int i = 0 ; i < m_count ; i++){
        triggers[i] = m_rayTracerResults[i].m_triggered;
        measurements[i] = m_rayTracerResults[i].m_depthFraction;
        if (m_rayTracerResults[i].m_triggered){
            if (m_rayTracerResults[i].m_sensedAFRigidBody){
                sensed_obj_names[i] = m_rayTracerResults[i].m_sensedAFRigidBody->m_name;
            }
            if (m_rayTracerResults[i].m_sensedAFSoftBody){
                sensed_obj_names[i] = m_rayTracerResults[i].m_sensedAFSoftBody->m_name;
            }
        }
        else{
            sensed_obj_names[i] = "";
        }
    }

    m_afSensorCommPtr->set_range(m_range);
    m_afSensorCommPtr->set_triggers(triggers);
    m_afSensorCommPtr->set_measurements(measurements);
    m_afSensorCommPtr->set_sensed_objects(sensed_obj_names);

#endif
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
/// \brief afRayTracerSensor::afExecuteCommand
/// \param dt
///
void afRayTracerSensor::fetchCommands(double dt){

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
    afResistanceSensorAttributes &attribs = *a_attribs;

    bool result = false;
    // Stop the communication instance from loading.
    bool temp_passive_store = attribs.m_communicationAttribs.m_passive;
    attribs.m_communicationAttribs.m_passive = true;
    result = afRayTracerSensor::createFromAttribs(&attribs);

    if (result){

        attribs.m_communicationAttribs.m_passive = temp_passive_store;

        setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
        setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
        setPassive(attribs.m_communicationAttribs.m_passive);

        m_staticContactFriction = attribs.m_staticContactFriction;

        m_staticContactDamping = attribs.m_staticContactDamping;

        m_dynamicFriction = attribs.m_dynamicFriction;

        m_useVariableCoeff = attribs.m_useVariableCoeff;

        m_contactArea = attribs.m_contactArea;
        m_contactNormalStiffness = attribs.m_contactNormalStiffness;

        m_contactNormalDamping = attribs.m_contactNormalDamping;

        m_rayContactResults.resize(m_count);

        for (uint i = 0 ; i < m_count ; i++){
            m_rayContactResults[i].m_bodyAContactPointLocal.set(0,0,0);
            m_rayContactResults[i].m_bodyBContactPointLocal.set(0,0,0);

            m_rayContactResults[i].m_tangentialError.set(0,0,0);
            m_rayContactResults[i].m_tangentialErrorLast.set(0,0,0);

            m_rayContactResults[i].m_contactPointsValid = false;

        }

        if (isPassive() == false){

            string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFSensorMap());

            afCreateCommInstance(afObjectType::SENSOR,
                                 getQualifiedName() + remap_idx,
                                 m_afWorld->getGlobalNamespace(),
                                 getMinPublishFrequency(),
                                 getMaxPublishFrequency());
    #ifdef C_ENABLE_AMBF_COMM_SUPPORT
            m_afSensorCommPtr->set_type("RESISTANCE");
    #endif
        }
    }

    return result;
}


///
/// \brief afResistanceSensor::updatePositionFromDynamics
///
void afResistanceSensor::update(){
    // Let's update the RayTracer Sensor First
    afRayTracerSensor::update();

    if (m_parentBody == nullptr){
        return;
    }

    for (uint i = 0 ; i < m_count ; i++){

        if (isTriggered(i)){
            if (m_showSensor){
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

            if(m_showSensor){
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
/// \brief afWorld::afWorld
/// \param a_global_namespace
///
bool afWorld::checkIfExists(afBaseObject *a_obj)
{
    vector<afBaseObject*>::iterator it;
    for (it = m_childrenAFObjects.begin() ; it != m_childrenAFObjects.end() ; ++it){
        if ((*it) == a_obj){
            return true;
        }
    }

    return false;
}

afWorld::afWorld(string a_global_namespace){
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

    m_pickSphere = new cMesh();
    cCreateSphere(m_pickSphere, 0.02);
    m_pickSphere->m_material->setPinkHot();
    m_pickSphere->setUseDisplayList(true);
    m_pickSphere->markForUpdate(false);
    m_pickSphere->setLocalPos(0,0,0);
    m_pickSphere->setShowEnabled(false);
    addSceneObjectToWorld(m_pickSphere);
    m_pickColor.setOrangeTomato();
    m_pickColor.setTransparencyLevel(0.3);
    m_namespace = "";
    setGlobalNamespace(a_global_namespace);

}

afWorld::~afWorld()
{
    if (m_pickedConstraint != nullptr){
        delete m_pickedConstraint;
    }

    for (afSensorMap::iterator sIt = m_afSensorMap.begin() ; sIt != m_afSensorMap.end() ; ++sIt){
        delete sIt->second;
    }

    for (afActuatorMap::iterator aIt = m_afActuatorMap.begin() ; aIt != m_afActuatorMap.end() ; ++aIt){
        delete aIt->second;
    }

    for (afJointMap::iterator jIt = m_afJointMap.begin() ; jIt != m_afJointMap.end() ; ++jIt){
        delete jIt->second;
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


///
/// \brief afWorld::getFullyQualifiedName
/// \param a_name
/// \return
///
string afWorld::resolveGlobalNamespace(string a_name){
    string fully_qualified_name = getGlobalNamespace() + a_name;
    fully_qualified_name = afUtils::removeAdjacentBackSlashes(fully_qualified_name);
    return fully_qualified_name;
}


///
/// \brief afWorld::setGlobalNamespace
/// \param a_global_namespace
///
void afWorld::setGlobalNamespace(string a_global_namespace){
    m_global_namespace = a_global_namespace;
    if (!m_global_namespace.empty()){
        cerr << " INFO! FORCE PREPENDING GLOBAL NAMESPACE \"" << m_global_namespace << "\" \n" ;
    }
}


///
/// \brief afWorld::resetCameras
///
void afWorld::resetCameras(){
    afCameraMap::iterator camIt;
    for (camIt = m_afCameraMap.begin() ; camIt != m_afCameraMap.end() ; ++camIt){
        afCameraPtr afCam = (camIt->second);
        afCam->setLocalTransform(afCam->getInitialTransform());
    }

}

///
/// \brief afWorld::resetWorld
/// \param reset_time
///
void afWorld::resetDynamicBodies(bool reset_time){
    pausePhysics(true);

    afRigidBodyMap::iterator rbIt;

    for (rbIt = m_afRigidBodyMap.begin() ; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
        afRigidBodyPtr afRB = (rbIt->second);
        btRigidBody* rB = afRB->m_bulletRigidBody;
        btVector3 zero(0, 0, 0);
        rB->clearForces();
        rB->setLinearVelocity(zero);
        rB->setAngularVelocity(zero);
        btTransform bt_T = to_btTransform(afRB->getInitialTransform());
        rB->getMotionState()->setWorldTransform(bt_T);
        rB->setWorldTransform(bt_T);
    }

    if (reset_time){
//        s_bulletWorld->setSimulationTime(0.0);
    }

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
    m_bulletWorld->setGravity(btVector3(vec(0), vec(1), vec(2)));
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
/// \brief afWorld::afExecuteCommand
/// \param dt
///
void afWorld::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT

    // If throttling in enabled, wait here until the step clock is toggled before
    // progressing towards next step
    if(m_afWorldCommPtr.get() != nullptr){
        while (!m_afWorldCommPtr->step_sim()){
            usleep(1);
        }
    }

    m_read_count++;
    if(m_read_count % 2000 == 0){
        m_afWorldCommPtr->update_params_from_server();
        if (m_afWorldCommPtr->m_paramsChanged){
            // Do the stuff

            vector<string> def_topics = m_afWorldCommPtr->get_defunct_topic_names();
            vector<string> new_topics = m_afWorldCommPtr->get_new_topic_names();

            for (int i = 0 ; i < def_topics.size() ; i++){
                string topic_name = def_topics[i];
                if (m_pcMap.find(topic_name) != m_pcMap.end()){
                    // Cleanup
                    afPointCloudPtr afPC = m_pcMap.find(topic_name)->second;
                    cMultiPointPtr mpPtr = afPC->m_mpPtr;
                    mpPtr->removeFromGraph();
                    m_pcMap.erase(topic_name);
                    delete mpPtr;
                }
            }

            for (int i = 0 ; i < new_topics.size() ; i++){
                string topic_name = new_topics[i];
                ambf_comm::PointCloudHandlerPtr pchPtr = m_afWorldCommPtr->get_point_clound_handler(topic_name);
                if (pchPtr){
                    cMultiPointPtr mpPtr = new cMultiPoint();
                    afPointCloudPtr afPC = new afPointCloud(this);
                    afPC->m_mpPtr = mpPtr;
                    afPC->m_pcCommPtr = pchPtr;
                    m_pcMap[topic_name] = afPC;
                    // Add as child, the header in PC message can override the parent later
                    addSceneObjectToWorld(mpPtr);
                }

            }
        }
        m_read_count = 0;
    }

#endif
}


///
/// \brief afWorld::updateDynamics
/// \param a_interval
/// \param a_wallClock
/// \param a_loopFreq
/// \param a_numDevices
///
void afWorld::updateDynamics(double a_interval, double a_wallClock, double a_loopFreq, int a_numDevices)
{
    // sanity check
    if (a_interval <= 0) { return; }

    if (m_pausePhx){
        if (m_manualStepPhx > 0){
            m_manualStepPhx--;
        }
        else{
            return;
        }
    }

    fetchCommands(a_interval);

    m_wallClock = a_wallClock;

    double dt = getSimulationDeltaTime();

    // Read the AF_COMM commands and apply to all different types of objects
    vector<afBaseObjectPtr>::iterator i;

    for(i = m_childrenAFObjects.begin(); i != m_childrenAFObjects.end(); ++i)
    {
        afBaseObject* childObj = *i;
        childObj->fetchCommands(dt);
    }

    // integrate simulation during an certain interval
    m_bulletWorld->stepSimulation(a_interval, m_maxIterations, m_integrationTimeStep);

    // add time to overall simulation
    m_lastSimulationTime = m_simulationTime;
    m_simulationTime = m_simulationTime + a_interval;

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afWorldCommPtr.get() != nullptr){
        m_afWorldCommPtr->set_sim_time(m_simulationTime);
        m_afWorldCommPtr->set_wall_time(m_wallClock);
        m_afWorldCommPtr->set_loop_freq(a_loopFreq);
        m_afWorldCommPtr->set_num_devices(a_numDevices);
    }
#endif

    afUpdateTimes(getWallTime(), getSimulationTime());

    estimateBodyWrenches();

    for(i = m_childrenAFObjects.begin(); i != m_childrenAFObjects.end(); ++i)
    {
        afBaseObject* childObj = *i;
        childObj->update();
    }
}


///
/// \brief afWorld::estimateBodyWrenches
///
void afWorld::estimateBodyWrenches(){

    // First clear out the wrench estimation from last iteration
    afRigidBodyMap::iterator rbIt = m_afRigidBodyMap.begin();
    for (; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
        rbIt->second->m_estimatedForce.setZero();
        rbIt->second->m_estimatedTorque.setZero();
    }


    // Now estimate the wrenches based on joints that have feedback enabled
    afJointMap::iterator jIt = m_afJointMap.begin();
    for (; jIt != m_afJointMap.end() ; ++ jIt){
        if (jIt->second->isFeedBackEnabled()){
            afJointPtr jnt = jIt->second;
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

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_paramsSet == false){
        // Create a default point cloud to listen to
        m_afWorldCommPtr->append_point_cloud_topic(getQualifiedName() + "/" + "point_cloud");
        m_afWorldCommPtr->set_params_on_server();
        m_paramsSet = true;
    }
#endif

    vector<afBaseObjectPtr>::iterator i;

    for(i = m_childrenAFObjects.begin(); i != m_childrenAFObjects.end(); ++i)
    {
        afBaseObject* childObj = *i;
        childObj->updateGlobalPose();
        childObj->updateSceneObjects();
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
    double step_size = g_wallClock.getCurrentTimeSeconds() - getSimulationTime();
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
}



bool afWorld::createFromAttribs(afWorldAttributes* a_attribs){

    afWorldAttributes & attribs = *a_attribs;

    attribs.resolveRelativeNamespace();
    attribs.resolveRelativePathAttribs();

    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    afCreateCommInstance(afObjectType::WORLD,
                         getQualifiedName(),
                         getGlobalNamespace(),
                         50,
                         2000,
                         10.0);

    m_maxIterations = attribs.m_maxIterations;

    setGravity(attribs.m_gravity);

    if (attribs.m_environmentModel.m_use){
        afModelPtr envModel = new afModel(this);
        if (envModel->createFromAttribs(&attribs.m_environmentModel.m_modelAttribs)){
            // ADD THE MODEL TO WORLD SOMEHOW
        }

    }
    else if (attribs.m_enclosure.m_use){
        m_enclosureL = attribs.m_enclosure.m_length;
        m_enclosureW = attribs.m_enclosure.m_width;
        m_enclosureH = attribs.m_enclosure.m_height;

        createDefaultWorld();
    }
    else{
        // THROW SOME ERROR THAT NO MODEL FOR WORLD CAN BE LOADED
    }


    m_skyBoxDefined = attribs.m_skyBoxAttribs.m_use;

    if (m_skyBoxDefined){
        m_skyBoxRight = attribs.m_skyBoxAttribs.m_rightImageFilepath;
        m_skyBoxLeft = attribs.m_skyBoxAttribs.m_leftImageFilepath;
        m_skyBoxTop = attribs.m_skyBoxAttribs.m_topImageFilepath;
        m_skyBoxBottom = attribs.m_skyBoxAttribs.m_bottomImageFilepath;
        m_skyBoxFront = attribs.m_skyBoxAttribs.m_frontImageFilepath;
        m_skyBoxBack = attribs.m_skyBoxAttribs.m_backImageFilepath;

        m_skyBox_shaderProgramDefined = attribs.m_skyBoxAttribs.m_shaderAttribs.m_shaderDefined;
        if (m_skyBox_shaderProgramDefined){
            m_skyBox_vsFilePath = attribs.m_skyBoxAttribs.m_shaderAttribs.m_vtxFilepath;
            m_skyBox_fsFilePath = attribs.m_skyBoxAttribs.m_shaderAttribs.m_fragFilepath;
        }
    }


    for (size_t idx = 0 ; idx < attribs.m_lightAttribs.size(); idx++){
        afLightPtr lightPtr = new afLight(this);
        if (lightPtr->createFromAttribs(&attribs.m_lightAttribs[idx])){
            string remaped_name = addAFLight(lightPtr);
        }
    }

    if (m_afLightMap.size() == 0){
        // ADD A DEFAULT LIGHT???
        afLightAttributes lightAttribs;
        lightAttribs.m_kinematicAttribs.m_location.setPosition(afVector3d(2, 2, 5));
        lightAttribs.m_identificationAttribs.m_name = "default_light";
        afLightPtr lightPtr = new afLight(this);
        lightPtr->createFromAttribs(&lightAttribs);
        string remaped_name = addAFLight(lightPtr);
    }

    if (attribs.m_showGUI){
        for (size_t idx = 0 ; idx < attribs.m_cameraAttribs.size(); idx++){
            afCameraPtr cameraPtr = new afCamera(this);
            if (cameraPtr->createFromAttribs(&attribs.m_cameraAttribs[idx])){
                string remaped_name = addAFCamera(cameraPtr);
            }
        }

        if (m_afCameraMap.size() == 0){
            // No valid cameras defined in the world config file
            // hence create a default camera
            afCameraPtr cameraPtr = new afCamera(this);
            afCameraAttributes camAttribs;
            camAttribs.m_lookAt.set(-1, 0, 0);
            camAttribs.m_identificationAttribs.m_name = "default_camera";
            if (cameraPtr->createFromAttribs(&camAttribs)){
                string remaped_name = addAFCamera(cameraPtr);
            }

        }

        m_shaderProgramDefined = attribs.m_shaderAttribs.m_shaderDefined;

        if (m_shaderProgramDefined){
            m_vsFilePath = attribs.m_shaderAttribs.m_vtxFilepath;
            m_fsFilePath = attribs.m_shaderAttribs.m_fragFilepath;
        }
    }

    return true;

}


///
/// \brief afWorld::render
/// \param options
///
void afWorld::render(afRenderOptions &options)
{
    // Update shadow maps once
    m_chaiWorld->updateShadowMaps(false, options.m_mirroredDisplay);

    updateSceneObjects();

    afCameraMap::iterator camIt;
    for (camIt = m_afCameraMap.begin(); camIt != m_afCameraMap.end(); ++ camIt){
        afCameraPtr cameraPtr = (camIt->second);
        cameraPtr->render(options);

    }

}


///
/// \brief afWorld::createSkyBox
///
void afWorld::loadSkyBox(){
    if (m_skyBoxDefined && m_skyBox_shaderProgramDefined){

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
        res[0] = newTexture->m_images[0]->loadFromFile(m_skyBoxRight.c_str());
        res[1] = newTexture->m_images[1]->loadFromFile(m_skyBoxLeft.c_str());
        res[2] = newTexture->m_images[3]->loadFromFile(m_skyBoxTop.c_str());
        res[3] = newTexture->m_images[2]->loadFromFile(m_skyBoxBottom.c_str());
        res[4] = newTexture->m_images[4]->loadFromFile(m_skyBoxFront.c_str());
        res[5] = newTexture->m_images[5]->loadFromFile(m_skyBoxBack.c_str());

//        res[0] = newTexture->m_images[0]->loadFromFile(m_skyBoxFront.c_str());
//        res[1] = newTexture->m_images[1]->loadFromFile(m_skyBoxBack.c_str());
//        res[2] = newTexture->m_images[2]->loadFromFile(m_skyBoxRight.c_str());
//        res[3] = newTexture->m_images[3]->loadFromFile(m_skyBoxLeft.c_str());
//        res[4] = newTexture->m_images[4]->loadFromFile(m_skyBoxTop.c_str());
//        res[5] = newTexture->m_images[5]->loadFromFile(m_skyBoxBottom.c_str());

        if (res[0] && res[1] && res[2] && res[3] && res[4] && res[5] && res[5]){
            // All images were loaded succesfully

            m_skyBoxMesh->setTexture(newTexture);
            m_skyBoxMesh->setUseTexture(true);

            addSceneObjectToWorld(m_skyBoxMesh);

            if (m_skyBox_shaderProgramDefined){
                ifstream vsFile;
                ifstream fsFile;
                vsFile.open(m_skyBox_vsFilePath.c_str());
                fsFile.open(m_skyBox_fsFilePath.c_str());
                // create a string stream
                stringstream vsBuffer, fsBuffer;
                // dump the contents of the file into it
                vsBuffer << vsFile.rdbuf();
                fsBuffer << fsFile.rdbuf();
                // close the files
                vsFile.close();
                fsFile.close();

                cShaderProgramPtr shaderProgram = cShaderProgram::create(vsBuffer.str(), fsBuffer.str());
                if (shaderProgram->linkProgram()){
                    // Just empty Pts to let us use the shader
                    cGenericObject* go;
                    cRenderOptions ro;
                    shaderProgram->use(go, ro);

                    cerr << "USING SKYBOX SHADER FILES: " <<
                                 "\n \t VERTEX: " << m_skyBox_vsFilePath.c_str() <<
                                 "\n \t FRAGMENT: " << m_skyBox_fsFilePath.c_str() << endl;
                    m_skyBoxMesh->setShaderProgram(shaderProgram);

                }
                else{
                    cerr << "ERROR! FOR SKYBOX FAILED TO LOAD SHADER FILES: " <<
                                 "\n \t VERTEX: " << m_skyBox_vsFilePath.c_str() <<
                                 "\n \t FRAGMENT: " << m_skyBox_fsFilePath.c_str() << endl;

                    m_skyBox_shaderProgramDefined = false;
                    removeSceneObjectFromWorld(m_skyBoxMesh);
                    delete m_skyBoxMesh;
                }
            }
        }
        else{
            cerr << "CAN'T LOAD SKY BOX IMAGES, IGNORING\n";
        }
    }
}


///
/// \brief afWorld::enableShaderProgram
///
void afWorld::enableShaderProgram(){
    if (m_shaderProgramDefined){
        afRigidBodyMap::iterator it;
        for (it = m_afRigidBodyMap.begin() ; it != m_afRigidBodyMap.end() ; ++it){
            afRigidBodyPtr rBody = it->second;
            rBody->m_vtxShaderFilePath = m_vsFilePath;
            rBody->m_fragShaderFilePath = m_fsFilePath;
            rBody->m_shaderProgramDefined = true;
        }
    }
}



string afWorld::addAFLight(afLightPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afLightMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afLightPtr, afLightMap>(a_obj, remaped_identifier, &m_afLightMap);
    return remaped_identifier;
}


string afWorld::addAFCamera(afCameraPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afCameraMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afCameraPtr, afCameraMap>(a_obj, qualified_identifier + remap_str, &m_afCameraMap);
    return remaped_identifier;
}


string afWorld::addAFRigidBody(afRigidBodyPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afRigidBodyMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afRigidBodyPtr, afRigidBodyMap>(a_obj, qualified_identifier + remap_str, &m_afRigidBodyMap);
    return remaped_identifier;
}


string afWorld::addAFSoftBody(afSoftBodyPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afSoftBodyMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afSoftBodyPtr, afSoftBodyMap>(a_obj, qualified_identifier + remap_str, &m_afSoftBodyMap);
    return remaped_identifier;
}


string afWorld::addAFJoint(afJointPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afJointMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afJointPtr, afJointMap>(a_obj, qualified_identifier + remap_str, &m_afJointMap);
    return remaped_identifier;
}


string afWorld::addAFActuator(afActuatorPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afActuatorMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afActuatorPtr, afActuatorMap>(a_obj, qualified_identifier + remap_str, &m_afActuatorMap);
    return remaped_identifier;
}


string afWorld::addAFSensor(afSensorPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afSensorMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afSensorPtr, afSensorMap>(a_obj, qualified_identifier + remap_str, &m_afSensorMap);
    return remaped_identifier;
}


string afWorld::addAFModel(afModelPtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afModelMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afModelPtr, afModelMap>(a_obj, qualified_identifier + remap_str, &m_afModelMap);
    return remaped_identifier;
}



string afWorld::addAFVehicle(afVehiclePtr a_obj){
    string qualified_identifier = a_obj->getQualifiedIdentifier();
    string remap_str = afUtils::getNonCollidingIdx(qualified_identifier, &m_afVehicleMap);
    string remaped_identifier = qualified_identifier + remap_str;
    addAFObject<afVehiclePtr, afVehicleMap>(a_obj, qualified_identifier + remap_str, &m_afVehicleMap);
    return remaped_identifier;
}


///
/// \brief afWorld::buildCollisionGroups
///
void afWorld::buildCollisionGroups(){
    if (m_collisionGroups.size() > 0){
        vector<int> groupNumbers;

        map<uint, vector<afRigidBodyPtr> >::iterator cgIt;
        for(cgIt = m_collisionGroups.begin() ; cgIt != m_collisionGroups.end() ; ++cgIt){
            groupNumbers.push_back(cgIt->first);
        }

        for (uint i = 0 ; i < groupNumbers.size() - 1 ; i++){
            int aIdx = groupNumbers[i];
            vector<afRigidBodyPtr> grpA = m_collisionGroups[aIdx];
            for (uint j = i + 1 ; j < groupNumbers.size() ; j ++){
                int bIdx = groupNumbers[j];
                vector<afRigidBodyPtr> grpB = m_collisionGroups[bIdx];

                for(uint aBodyIdx = 0 ; aBodyIdx < grpA.size() ; aBodyIdx++){
                    afRigidBodyPtr bodyA = grpA[aBodyIdx];
                    for(uint bBodyIdx = 0 ; bBodyIdx < grpB.size() ; bBodyIdx++){
                        afRigidBodyPtr bodyB = grpB[bBodyIdx];
                        if (bodyA != bodyB && !bodyB->isCommonCollisionGroupIdx(bodyA->m_collisionGroups))
                            bodyA->m_bulletRigidBody->setIgnoreCollisionCheck(bodyB->m_bulletRigidBody, true);
                    }
                }
            }
        }
    }
}


///
/// \brief afWorld::getAFLighs
/// \return
///
afLightVec  afWorld::getAFLighs(){
    return getAFObjects<afLightVec, afLightMap>(&m_afLightMap);
}


///
/// \brief afWorld::getAFCameras
/// \return
///
afCameraVec afWorld::getAFCameras(){
    return getAFObjects<afCameraVec, afCameraMap>(&m_afCameraMap);
}


///
/// \brief afWorld::getAFRigidBodies
/// \return
///
afRigidBodyVec afWorld::getAFRigidBodies(){
    return getAFObjects<afRigidBodyVec, afRigidBodyMap>(&m_afRigidBodyMap);
}


///
/// \brief afWorld::getAFSoftBodies
/// \return
///
afSoftBodyVec afWorld::getAFSoftBodies(){
    return getAFObjects<afSoftBodyVec, afSoftBodyMap>(&m_afSoftBodyMap);
}


///
/// \brief afWorld::getJoints
/// \return
///
afJointVec afWorld::getAFJoints(){
    return getAFObjects<afJointVec, afJointMap>(&m_afJointMap);
}


///
/// \brief afWorld::getSensors
/// \return
///
afSensorVec afWorld::getAFSensors(){
    return getAFObjects<afSensorVec, afSensorMap>(&m_afSensorMap);
}


///
/// \brief afWorld::getAFMultiBodies
/// \return
///
afModelVec afWorld::getAFMultiBodies(){
    return getAFObjects<afModelVec, afModelMap>(&m_afModelMap);
}


///
/// \brief afWorld::getAFVehicles
/// \return
///
afVehicleVec afWorld::getAFVehicles(){
    return getAFObjects<afVehicleVec, afVehicleMap>(&m_afVehicleMap);
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
        m_pickSphere->setLocalPos(pickPos);
        m_pickSphere->setShowEnabled(true);
        const btCollisionObject* colObject = rayCallback.m_collisionObject;
        if (colObject->getInternalType() == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
            btRigidBody* body = (btRigidBody*)btRigidBody::upcast(colObject);
            if (body){
                m_pickedAFRigidBody = getAFRigidBody(body, true);
                if (m_pickedAFRigidBody){
                    cerr << "User picked AF rigid body: " << m_pickedAFRigidBody->m_name << endl;
                    m_pickedBulletRigidBody = body;
                    m_pickedAFRigidBodyColor = m_pickedAFRigidBody->m_visualMesh->m_material->copy();
                    m_pickedAFRigidBody->m_visualMesh->setMaterial(m_pickColor);
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
        m_pickSphere->setLocalPos(newLocation);

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
        m_pickSphere->setLocalPos(newPivotB);
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
        m_pickSphere->setShowEnabled(false);
        m_pickedBulletRigidBody = nullptr;
    }

    if (m_pickedAFRigidBody){
        m_pickedAFRigidBody->m_visualMesh->setMaterial(m_pickedAFRigidBodyColor);
    }

    if (m_pickedSoftBody){
        m_pickSphere->setShowEnabled(false);
        m_pickedSoftBody = nullptr;
        m_pickedNodeIdx = -1;
        m_pickedNodeMass = 0;
    }
}


///
/// \brief afCamera::afCamera
///
afCamera::afCamera(afWorldPtr a_afWorld): afBaseObject(a_afWorld){

    s_monitors = glfwGetMonitors(&s_numMonitors);
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

///
/// \brief afCamera::createDefaultCamera
/// \return
///
bool afCamera::createDefaultCamera(){
    cerr << "INFO: USING DEFAULT CAMERA" << endl;

    m_camera = new cCamera(m_afWorld->m_chaiWorld);
    addChildSceneObject(m_camera, cTransform());

    m_namespace = m_afWorld->getNamespace();

    // Set a default name
    m_name = "default_camera";

    // position and orient the camera
    setView(cVector3d(4.0, 0.0, 2.0),  // camera position (eye)
            cVector3d(0.0, 0.0,-0.5),       // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));      // direction of the "up" vector

    m_initialTransform = getLocalTransform();

    // set the near and far clipping planes of the camera
    m_camera->setClippingPlanes(0.01, 10.0);

    // Set the Field of View
    m_camera->setFieldViewAngleRad(0.7);

    // set stereo mode
    m_camera->setStereoMode(cStereoMode::C_STEREO_DISABLED);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    m_camera->setStereoEyeSeparation(0.02);
    m_camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    setMirrorVertical(false);

    // create display context
    // compute desired size of window
    m_monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(m_monitor);
    int w = 0.5 * mode->width;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);
    m_window = glfwCreateWindow(w, h, "AMBF Simulator", nullptr, nullptr);
    s_mainWindow = m_window;

    m_win_x = x;
    m_win_y = y;
    m_width = w;
    m_height = h;

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

    s_windowIdx++;
    s_cameraIdx++;

    // Assign the Window Camera Handles
    addChildSceneObject(m_camera, cTransform());

    return true;
}

bool afCamera::createFromAttribs(afCameraAttributes *a_attribs)
{
    afCameraAttributes & attribs = *a_attribs;

    int monitorToLoad = attribs.m_monitorNumber;

    // Set some default values
    m_stereMode = C_STEREO_DISABLED;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;

    m_camPos << attribs.m_kinematicAttribs.m_location.getPosition();
    m_camLookAt << attribs.m_lookAt;
    m_camUp << attribs.m_up;

    m_orthographic = attribs.m_orthographic;

    if (monitorToLoad < 0 || monitorToLoad >= s_numMonitors){
        cerr << "INFO: CAMERA \"" << attribs.m_identificationAttribs.m_name << "\" MONITOR NUMBER \"" << monitorToLoad
             << "\" IS NOT IN RANGE OF AVAILABLE MONITORS \""<< s_numMonitors <<"\", USING DEFAULT" << endl;
        monitorToLoad = -1;
    }

    if (attribs.m_stereo){
        m_stereMode = cStereoMode::C_STEREO_PASSIVE_LEFT_RIGHT;
    }

    m_controllingDevNames = attribs.m_controllingDeviceNames;

    m_publishImage = attribs.m_publishImage;
    m_imagePublishInterval = attribs.m_publishImageInterval;
    m_publishDepth = attribs.m_publishDepth;
    m_depthPublishInterval = attribs.m_publishDepthInterval;

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    m_camera = new cCamera(m_afWorld->m_chaiWorld);

    addChildSceneObject(m_camera, cTransform());

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;

    //////////////////////////////////////////////////////////////////////////////////////
    // position and orient the camera
    setView(m_camPos, m_camLookAt, m_camUp);
    m_initialTransform = getLocalTransform();
    // set the near and far clipping planes of the camera
    m_camera->setClippingPlanes(attribs.m_nearPlane, attribs.m_farPlane);

    // set stereo mode
    m_camera->setStereoMode(m_stereMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    m_camera->setStereoEyeSeparation(attribs.m_stereoEyeSeparation);
    m_camera->setStereoFocalLength(attribs.m_stereFocalLength);

    // set vertical mirrored display mode
    m_camera->setMirrorVertical(false);

    if (m_orthographic){
        m_camera->setOrthographicView(attribs.m_orthoViewWidth);
    }
    else{
        m_camera->setFieldViewAngleRad(attribs.m_fieldViewAngle);
    }

    m_camera->setUseMultipassTransparency(attribs.m_multiPass);

    string window_name = "AMBF Simulator Window " + to_string(s_cameraIdx + 1);
    if (m_controllingDevNames.size() > 0){
        for (int i = 0 ; i < m_controllingDevNames.size() ; i++){
            window_name += (" - " + m_controllingDevNames[i]);
        }

    }

    // create display context
    if (monitorToLoad == -1){
        if (s_cameraIdx < s_numMonitors){
            monitorToLoad = s_cameraIdx;
        }
        else{
            monitorToLoad = 0;
        }
    }
    m_monitor = s_monitors[monitorToLoad];

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(m_monitor);
    int w = 0.5 * mode->width;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    m_win_x = x;
    m_win_y = y;
    m_width = w;
    m_height = h;

    m_window = glfwCreateWindow(w, h, window_name.c_str(), nullptr, s_mainWindow);
    if (s_windowIdx == 0){
        s_mainWindow = m_window;
    }

    if (!m_window)
    {
        cerr << "ERROR! FAILED TO CREATE OPENGL WINDOW" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(m_window, &m_width, &m_height);

    // set position of window
    glfwSetWindowPos(m_window, m_win_x, m_win_y);

    // set the current context
    glfwMakeContextCurrent(m_window);

    glfwSwapInterval(0);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cerr << "ERROR! FAILED TO INITIALIZE GLEW LIBRARY" << endl;
        glfwTerminate();
        return 1;
    }
#endif

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

    s_windowIdx++;
    s_cameraIdx++;

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFCameraMap());

        afCreateCommInstance(afObjectType::CAMERA,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }


    if (m_publishImage || m_publishDepth){
        m_frameBuffer = new cFrameBuffer();
        m_bufferColorImage = cImage::create();
        m_bufferDepthImage = cImage::create();
        m_frameBuffer->setup(m_camera, m_width, m_height, true, true);

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
        if (s_imageTransportInitialized == false){
            s_imageTransportInitialized = true;
            int argc = 0;
            char **argv = 0;
            ros::init(argc, argv, "ambf_image_transport_node");
            s_rosNode = new ros::NodeHandle();
            s_imageTransport = new image_transport::ImageTransport(*s_rosNode);
        }
        m_imagePublisher = s_imageTransport->advertise(getQualifiedName() + "/ImageData", 1);
#endif

        if (m_publishDepth){
            // Set up the world
            m_dephtWorld = new cWorld();

            // Set up the frame buffer
            m_depthBuffer = new cFrameBuffer();
            m_depthBuffer->setup(m_camera, m_width, m_height, true, false, GL_RGBA16);

            m_depthPC.setup(m_width, m_height, 3);

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
                m_depthMesh->newTriangle(
                            cVector3d(quad[off + 0], quad[off + 1], quad[off + 2]),
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
            m_depthMesh->m_texture = cTexture2d::create();
            m_depthMesh->m_texture->m_image->allocate(m_width, m_height, GL_RGBA, GL_UNSIGNED_BYTE);
            m_depthMesh->setUseTexture(true);

            m_dephtWorld->addChild(m_depthMesh);
            m_dephtWorld->addChild(m_camera);

            m_depthBufferColorImage = cImage::create();
            m_depthBufferColorImage->allocate(m_width, m_height, GL_RGBA, GL_UNSIGNED_INT);

            //                // DEBUGGING USING EXTERNALLY DEFINED SHADERS
            //                ifstream vsFile;
            //                ifstream fsFile;
            //                vsFile.open("/home/adnan/ambf/ambf_shaders/depth/shader.vs");
            //                fsFile.open("/home/adnan/ambf/ambf_shaders/depth/shader.fs");
            //                // create a string stream
            //                stringstream vsBuffer, fsBuffer;
            //                // dump the contents of the file into it
            //                vsBuffer << vsFile.rdbuf();
            //                fsBuffer << fsFile.rdbuf();
            //                // close the files
            //                vsFile.close();
            //                fsFile.close();
            //                cShaderProgramPtr shaderPgm = cShaderProgram::create(vsBuffer.str(), fsBuffer.str());

            cShaderProgramPtr shaderPgm = cShaderProgram::create(AF_DEPTH_COMPUTE_VTX, AF_DEPTH_COMPUTE_FRAG);
            if (shaderPgm->linkProgram()){
                cGenericObject* go;
                cRenderOptions ro;
                shaderPgm->use(go, ro);
                m_depthMesh->setShaderProgram(shaderPgm);
                shaderPgm->disable();
            }
            else{
                cerr << "ERROR! FOR DEPTH_TO_PC2 FAILED TO LOAD SHADER FILES: " << endl;
            }

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
            m_depthPointCloudMsg.reset(new sensor_msgs::PointCloud2());
            m_depthPointCloudModifier = new sensor_msgs::PointCloud2Modifier(*m_depthPointCloudMsg);
            m_depthPointCloudModifier->setPointCloud2FieldsByString(2, "xyz", "rgb");
            m_depthPointCloudModifier->resize(m_width*m_height);
            if (s_imageTransportInitialized == false){
                s_imageTransportInitialized = true;
                int argc = 0;
                char **argv = 0;
                ros::init(argc, argv, "ambf_image_transport_node");
                s_rosNode = new ros::NodeHandle();
            }
            m_depthPointCloudPub = s_rosNode->advertise<sensor_msgs::PointCloud2>(getQualifiedName() + "/DepthData", 1);
#endif

        }
    }

    return true;
}


///
/// \brief afCamera::renderFrameBuffer
///
void afCamera::renderFrameBuffer()
{
    m_frameBuffer->renderView();
    m_frameBuffer->copyImageBuffer(m_bufferColorImage);
    m_frameBuffer->copyDepthBuffer(m_bufferDepthImage);
    m_bufferColorImage->flipHorizontal();
    m_bufferDepthImage->flipHorizontal();
}


///
/// \brief afCamera::publishDepthCPUBased
///
void afCamera::computeDepthOnCPU()
{
    cTransform projMatInv = m_camera->m_projectionMatrix;
    projMatInv.m_flagTransform = false;
    projMatInv.invert();

    int width = m_bufferDepthImage->getWidth();
    int height = m_bufferDepthImage->getHeight();
    int bbp = m_bufferDepthImage->getBytesPerPixel();

    double varScale = pow(2, sizeof(uint) * 8);

    // Update the dimensions scale information.
    float n = -m_camera->getNearClippingPlane();
    float f = -m_camera->getFarClippingPlane();
    double fva = m_camera->getFieldViewAngleRad();
    double ar = m_camera->getAspectRatio();

    double delta_x;
    if (isOrthographic()){
        delta_x = m_camera->getOrthographicViewWidth();
    }
    else{
        delta_x = 2.0 * cAbs(f) * cTanRad(fva/2.0);
    }
    double delta_y = delta_x / ar;
    double delta_z = f-n;

    cVector3d maxWorldDimensions(delta_x, delta_y, delta_z);

    for (int y_span = 0 ; y_span < height ; y_span++){
        double yImage = double(y_span) / (height - 1);
        for (int x_span = 0 ; x_span < width ; x_span++){
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

            m_depthPC.m_data[idx * m_depthPC.m_numFields + 0] = pCam.x();
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 1] = pCam.y();
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 2] = pCam.z();

        }
    }

}


///
/// \brief afCamera::renderDepthBuffer
///
void afCamera::computeDepthOnGPU()
{

    m_bufferDepthImage->copyTo(m_depthMesh->m_texture->m_image);
    m_depthMesh->m_texture->markForUpdate();

    // Change the parent world for the camera so it only render's the depht quad (Mesh)
    m_camera->setParentWorld(m_dephtWorld);

    // Update the dimensions scale information.
    float n = -m_camera->getNearClippingPlane();
    float f = -m_camera->getFarClippingPlane();
    double fva = m_camera->getFieldViewAngleRad();
    double ar = m_camera->getAspectRatio();

    double maxX;
    if (isOrthographic()){
        maxX = m_camera->getOrthographicViewWidth();
    }
    else{
        maxX = 2.0 * cAbs(f) * cTanRad(fva/2.0);
    }
    double maxY = maxX / ar;
    double maxZ = f-n;

    cVector3d maxWorldDimensions(maxX, maxY, maxZ);

    m_depthMesh->getShaderProgram()->setUniform("maxWorldDimensions", maxWorldDimensions);
    m_depthMesh->getShaderProgram()->setUniformf("nearPlane", n);
    m_depthMesh->getShaderProgram()->setUniformf("farPlane", f);

    cTransform invProj = m_camera->m_projectionMatrix;
    invProj.m_flagTransform = false;
    invProj.invert();

    m_depthMesh->getShaderProgram()->setUniform("invProjection", invProj, false);

    m_depthBuffer->renderView();

    m_camera->setParentWorld(m_afWorld->m_chaiWorld);

    m_depthBuffer->copyImageBuffer(m_depthBufferColorImage, GL_UNSIGNED_INT);

//    // bind texture
//    glBindTexture(GL_TEXTURE_2D, m_depthBuffer->m_imageBuffer->getTextureId());

//    // settings
//    glPixelStorei(GL_PACK_ALIGNMENT, 1);

//    // copy pixel data if required
//    glGetTexImage(GL_TEXTURE_2D,
//                  0,
//                  GL_RGBA,
//                  GL_FLOAT,
//                  (GLvoid*)(m_depthBufferColorImage2)
//                  );

    uint width = m_depthBufferColorImage->getWidth();
    uint height = m_depthBufferColorImage->getHeight();
    uint bbp = m_depthBufferColorImage->getBytesPerPixel();

    double varScale = pow(2, sizeof(uint) * 8);

    for (uint y_span = 0 ; y_span < height ; y_span++){
        for (uint x_span = 0 ; x_span < width ; x_span++){

            uint idx = (y_span * width + x_span);
            unsigned char xByte0 = m_depthBufferColorImage->getData()[idx * bbp + 0];
            unsigned char xByte1 = m_depthBufferColorImage->getData()[idx * bbp + 1];
            unsigned char xByte2 = m_depthBufferColorImage->getData()[idx * bbp + 2];
            unsigned char xByte3 = m_depthBufferColorImage->getData()[idx * bbp + 3];

            unsigned char yByte0 = m_depthBufferColorImage->getData()[idx * bbp + 4];
            unsigned char yByte1 = m_depthBufferColorImage->getData()[idx * bbp + 5];
            unsigned char yByte2 = m_depthBufferColorImage->getData()[idx * bbp + 6];
            unsigned char yByte3 = m_depthBufferColorImage->getData()[idx * bbp + 7];

            unsigned char zByte0 = m_depthBufferColorImage->getData()[idx * bbp + 8];
            unsigned char zByte1 = m_depthBufferColorImage->getData()[idx * bbp + 9];
            unsigned char zByte2 = m_depthBufferColorImage->getData()[idx * bbp + 10];
            unsigned char zByte3 = m_depthBufferColorImage->getData()[idx * bbp + 11];

            uint ix = uint(xByte3 << 24 | xByte2 << 16 | xByte1 << 8 | xByte0);
            uint iy = uint(yByte3 << 24 | yByte2 << 16 | yByte1 << 8 | yByte0);
            uint iz = uint(zByte3 << 24 | zByte2 << 16 | zByte1 << 8 | zByte0);

            double px = double(ix) / varScale;
            double py = double(iy) / varScale;
            double pz = double(iz) / varScale;
            // Reconstruct from scales applied in the Frag Shader
            px = (px * maxX - (maxX / 2.0));
            py = (py  * maxY - (maxY / 2.0));
            pz = (pz * maxZ  + n);

            m_depthPC.m_data[idx * m_depthPC.m_numFields + 0] = px;
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 1] = py;
            m_depthPC.m_data[idx * m_depthPC.m_numFields + 2] = pz;
        }
    }
}


///
/// \brief afCamera::publishImage
///
void afCamera::publishImage(){
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    m_imageMatrix = cv::Mat(m_bufferColorImage->getHeight(), m_bufferColorImage->getWidth(), CV_8UC4, m_bufferColorImage->getData());
    cv::cvtColor(m_imageMatrix, m_imageMatrix, cv::COLOR_RGBA2RGB);
    sensor_msgs::ImagePtr rosMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_imageMatrix).toImageMsg();
    m_imagePublisher.publish(rosMsg);
#endif
}


///
/// \brief afCamera::publishDepthPointCloud
///
void afCamera::publishDepthPointCloud()
{
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    sensor_msgs::PointCloud2Iterator<float> pcMsg_x(*m_depthPointCloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> pcMsg_y(*m_depthPointCloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> pcMsg_z(*m_depthPointCloudMsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_r(*m_depthPointCloudMsg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_g(*m_depthPointCloudMsg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_b(*m_depthPointCloudMsg, "b");

    int width = m_depthBufferColorImage->getWidth();
    int height = m_depthBufferColorImage->getHeight();

    for (int idx = 0 ; idx < width * height ; idx++, ++pcMsg_x, ++pcMsg_y, ++pcMsg_z, ++pcMsg_r, ++pcMsg_g, ++pcMsg_b){
        *pcMsg_x = m_depthPC.m_data[idx * m_depthPC.m_numFields + 0];
        *pcMsg_y = m_depthPC.m_data[idx * m_depthPC.m_numFields + 1];
        *pcMsg_z = m_depthPC.m_data[idx * m_depthPC.m_numFields + 2];

        *pcMsg_r = m_bufferColorImage->getData()[idx * 4 + 0];
        *pcMsg_g = m_bufferColorImage->getData()[idx * 4 + 1];
        *pcMsg_b = m_bufferColorImage->getData()[idx * 4 + 2];
    }

    m_depthPointCloudMsg->header.frame_id = m_name;
    m_depthPointCloudMsg->header.stamp = ros::Time::now();
    m_depthPointCloudPub.publish(m_depthPointCloudMsg);
#endif
}

///
/// \brief afCamera::afObjectCommandExecute
/// \param dt
///
void afCamera::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afCameraCommPtr.get() != nullptr){
        ambf_msgs::CameraCmd m_afCommand = m_afCameraCommPtr->get_command();

        if (m_afCommand.enable_position_controller){
            cVector3d pos(m_afCommand.pose.position.x,
                          m_afCommand.pose.position.y,
                          m_afCommand.pose.position.z);

            cQuaternion rot_quat(m_afCommand.pose.orientation.w,
                                 m_afCommand.pose.orientation.x,
                                 m_afCommand.pose.orientation.y,
                                 m_afCommand.pose.orientation.z);

            cMatrix3d rot_mat;
            rot_quat.toRotMat(rot_mat);
            setLocalPos(pos);
            setLocalRot(rot_mat);
        }
        m_read_count++;
        if(m_read_count % 2000 == 0){
            // We may update the params intermittently
            m_afCameraCommPtr->update_params_from_server();
            if (m_afCameraCommPtr->m_paramsChanged){
                // Clear the flag so it can be used for testing again
                m_afCameraCommPtr->m_paramsChanged = false;

                double near_plane = m_afCameraCommPtr->get_near_plane();
                double far_plane = m_afCameraCommPtr->get_far_plane();
                double field_view_angle = m_afCameraCommPtr->get_field_view_angle();
                double orthographic_view_width = m_afCameraCommPtr->get_orthographic_view_width();
                double stereo_eye_separation = m_afCameraCommPtr->get_steteo_eye_separation();
                double stereo_focal_length = m_afCameraCommPtr->get_steteo_focal_length();

                string parent_name = m_afCameraCommPtr->get_parent_name();

                m_camera->setClippingPlanes(near_plane, far_plane);

                if (m_parentName.compare(parent_name) != 0){
                    resolveParenting(parent_name);
                }

                switch (m_afCameraCommPtr->get_projection_type()) {
                case ambf_comm::ProjectionType::PERSPECTIVE:
                    if (field_view_angle == 0){
                        field_view_angle = 0.7;
                        m_paramsSet = false;
                    }
                    m_camera->setFieldViewAngleRad(field_view_angle);
                    m_orthographic = false;
                    break;
                case ambf_comm::ProjectionType::ORTHOGRAPHIC:
                    if (orthographic_view_width == 0){
                        orthographic_view_width = 10.0;
                        m_paramsSet = false;
                    }
                    m_camera->setOrthographicView(orthographic_view_width);
                    m_orthographic = true;
                    break;
                default:
                    break;
                }

                switch (m_afCameraCommPtr->get_view_mode()) {
                case ambf_comm::ViewMode::MONO:
                    m_camera->setStereoMode(cStereoMode::C_STEREO_DISABLED);
                    break;
                case ambf_comm::ViewMode::STEREO:
                    m_camera->setStereoMode(cStereoMode::C_STEREO_PASSIVE_LEFT_RIGHT);
                    m_camera->setStereoEyeSeparation(stereo_eye_separation);
                    m_camera->setStereoFocalLength(stereo_focal_length);
                    break;
                default:
                    break;
                }
            }

            m_read_count = 0;
        }
    }
#endif
}


///
/// \brief afCamera::updatePositionFromDynamics
///
void afCamera::update()
{

    // update Transform data for m_ObjectPtr
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afCameraCommPtr.get() != nullptr){

        if (m_paramsSet == false){
            m_afCameraCommPtr->set_near_plane(m_camera->getNearClippingPlane());
            m_afCameraCommPtr->set_far_plane(m_camera->getFarClippingPlane());
            m_afCameraCommPtr->set_field_view_angle(m_camera->getFieldViewAngleRad());
            m_afCameraCommPtr->set_orthographic_view_width(m_camera->getOrthographicViewWidth());
            m_afCameraCommPtr->set_steteo_eye_separation(m_camera->getStereoEyeSeparation());
            m_afCameraCommPtr->set_steteo_focal_length(m_camera->getStereoFocalLength());
            m_afCameraCommPtr->set_parent_name(m_parentName);

            if (m_camera->isViewModePerspective()){
                m_afCameraCommPtr->set_projection_type(ambf_comm::ProjectionType::PERSPECTIVE);
            }
            else{
                m_afCameraCommPtr->set_projection_type(ambf_comm::ProjectionType::ORTHOGRAPHIC);
            }

            if (m_stereMode == C_STEREO_DISABLED){
                m_afCameraCommPtr->set_view_mode(ambf_comm::ViewMode::MONO);
            }
            else{
                m_afCameraCommPtr->set_view_mode(ambf_comm::ViewMode::STEREO);;
            }

            m_afCameraCommPtr->set_params_on_server();
            m_paramsSet = true;
        }

        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        cVector3d localPos = getLocalPos();
        m_afCameraCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
        cQuaternion q;
        q.fromRotMat(getLocalRot());
        m_afCameraCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

        m_write_count++;

        if (m_write_count % 2000 == 0){
            m_afCameraCommPtr->set_parent_name(m_parentName);
            m_write_count = 0;
        }
    }
#endif
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
    string wallTimeStr = "Wall Time: " + cStr(m_afWorld->g_wallClock.getCurrentTimeSeconds(), 2) + " s";
    string simTimeStr = "Sim Time: " + cStr(m_afWorld->getSimulationTime(), 2) + " s";

    string graphicsFreqStr = "Gfx (" + cStr(m_afWorld->m_freqCounterGraphics.getFrequency(), 0) + " Hz)";
    string hapticFreqStr = "Phx (" + cStr(m_afWorld->m_freqCounterHaptics.getFrequency(), 0) + " Hz)";

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


///
/// \brief afCamera::~afCamera
///
afCamera::~afCamera(){
    if (m_frameBuffer != nullptr){
        delete m_frameBuffer;
    }

    if (m_depthBuffer != nullptr){
        delete m_depthBuffer;
    }

    if (m_dephtWorld != nullptr){
        delete m_dephtWorld;
    }
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    if (s_imageTransport != nullptr){
        delete s_imageTransport;
    }

    s_imageTransportInitialized = false;
#endif

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (s_rosNode != nullptr){
        delete s_rosNode;
    }

    if (m_depthPointCloudModifier != nullptr){
        delete m_depthPointCloudModifier;
    }
#endif

}


///
/// \brief afCamera::render
/// \param options
///
void afCamera::render(afRenderOptions &options)
{
    // set current display context
    glfwMakeContextCurrent(m_window);

    // get width and height of window
    glfwGetFramebufferSize(m_window, &m_width, &m_height);

    // Update the Labels in a separate sub-routine
    if (options.m_updateLabels && !m_publishDepth){
        updateLabels(options);
    }

    if (m_afWorld->m_skyBox_shaderProgramDefined && m_afWorld->m_skyBoxMesh->getShaderProgram() != nullptr){
        cGenericObject* go;
        cRenderOptions ro;
        m_afWorld->m_skyBoxMesh->getShaderProgram()->use(go, ro);

        cMatrix3d rotOffsetPre(0, 0, 90, C_EULER_ORDER_ZYX, false, true);
        cMatrix3d rotOffsetPost(90, 90, 0, C_EULER_ORDER_ZYX, false, true);
        cTransform viewMat = rotOffsetPre * getLocalTransform() * rotOffsetPost;

        m_afWorld->m_skyBoxMesh->getShaderProgram()->setUniform("viewMat", viewMat, 1);

        m_afWorld->m_skyBoxMesh->getShaderProgram()->disable();
    }

    // render world
    renderView(m_width,m_height);

    // swap buffers
    glfwSwapBuffers(m_window);

    // Only set the window_closed if the condition is met
    // otherwise a non-closed window will set the variable back
    // to false
    if (glfwWindowShouldClose(m_window)){
        options.m_windowClosed = true;
    }

    m_sceneUpdateCounter++;

    if (m_publishImage || m_publishDepth){
        renderFrameBuffer();
    }

    if (m_publishImage){
        if (m_sceneUpdateCounter % m_imagePublishInterval == 0){
            publishImage();
        }
    }

    if (m_publishDepth){
        if (m_sceneUpdateCounter % m_depthPublishInterval == 0){
            if (m_useGPUForDepthComputation){
                computeDepthOnGPU();
            }
            else{
                computeDepthOnCPU();
            }
            publishDepthPointCloud();
        }
    }
}



///
/// \brief afLight::afLight
///
afLight::afLight(afWorldPtr a_afWorld): afBaseObject(a_afWorld){
}

///
/// \brief afLight::createDefaultLight
/// \return
///
bool afLight::createDefaultLight(){
    cerr << "INFO: NO LIGHT SPECIFIED, USING DEFAULT LIGHTING" << endl;
    m_spotLight = new cSpotLight(m_afWorld->m_chaiWorld);
    m_namespace = m_afWorld->getNamespace();
    m_name = "default_light";
    addChildSceneObject(m_spotLight, cTransform());
    m_spotLight->setLocalPos(cVector3d(0.0, 0.5, 2.5));
    m_spotLight->setDir(0, 0, -1);
    m_spotLight->setSpotExponent(0.3);
    m_spotLight->setCutOffAngleDeg(60);
    m_spotLight->setShadowMapEnabled(true);
    m_spotLight->m_shadowMap->setQualityVeryHigh();
    m_spotLight->setEnabled(true);
    m_afWorld->addSceneObjectToWorld(m_spotLight);

    return true;
}


bool afLight::createFromAttribs(afLightAttributes *a_attribs)
{
    afLightAttributes &attribs = *a_attribs;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    bool valid = true;

    cTransform trans = to_cTransform(attribs.m_kinematicAttribs.m_location);
    setLocalTransform(trans);

    cVector3d dir = to_cVector3d(attribs.m_direction);
    setDir(dir);

    m_spotLight = new cSpotLight(m_afWorld->m_chaiWorld);

    addChildSceneObject(m_spotLight, cTransform());

    m_afWorld->addSceneObjectToWorld(m_spotLight);

    m_parentName = attribs.m_hierarchyAttribs.m_parentName;

    m_initialTransform = getLocalTransform();

    m_spotLight->setSpotExponent(attribs.m_spotExponent);
    m_spotLight->setCutOffAngleDeg(attribs.m_cuttoffAngle * (180/3.14));
    m_spotLight->setShadowMapEnabled(true);

    switch (attribs.m_shadowQuality) {
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

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFLightMap());

        afCreateCommInstance(afObjectType::LIGHT,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }

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


void afLight::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afLightCommPtr.get() != nullptr){
        ambf_msgs::LightCmd m_afCommand = m_afLightCommPtr->get_command();

        if (m_afCommand.enable_position_controller){
            cVector3d pos(m_afCommand.pose.position.x,
                          m_afCommand.pose.position.y,
                          m_afCommand.pose.position.z);

            cQuaternion rot_quat(m_afCommand.pose.orientation.w,
                                 m_afCommand.pose.orientation.x,
                                 m_afCommand.pose.orientation.y,
                                 m_afCommand.pose.orientation.z);

            cMatrix3d rot_mat;
            rot_quat.toRotMat(rot_mat);
            setLocalPos(pos);
            setLocalRot(rot_mat);
        }
        m_read_count++;
        if(m_read_count % 2000 == 0){
            // We may update the params intermittently
            m_afLightCommPtr->update_params_from_server();
            if (m_afLightCommPtr->m_paramsChanged){
                // Clear the flag so it can be used for testing again
                m_afLightCommPtr->m_paramsChanged = false;

                double cutoff_angle = m_afLightCommPtr->get_cuttoff_angle();
                string parent_name = m_afLightCommPtr->get_parent_name();

                m_spotLight->setCutOffAngleDeg(cRadToDeg(cutoff_angle));

                if (m_parentName.compare(parent_name) != 0){
                    resolveParenting(parent_name);
                }
            }

            m_read_count = 0;
        }
    }
#endif
}



///
/// \brief afLight::updatePositionFromDynamics
///
void afLight::update()
{

    // update Transform data for m_ObjectPtr
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afLightCommPtr.get() != nullptr){

        if (m_paramsSet == false){
            m_afLightCommPtr->set_cuttoff_angle(cDegToRad(m_spotLight->getCutOffAngleDeg()));
            m_afLightCommPtr->set_type(ambf_comm::LightType::SPOT);
            m_afLightCommPtr->set_parent_name(m_parentName);

            m_afLightCommPtr->set_params_on_server();
            m_paramsSet = true;
        }

        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        cVector3d localPos = getLocalPos();
        m_afLightCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
        cQuaternion q;
        q.fromRotMat(getLocalRot());
        m_afLightCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

        m_write_count++;

        if (m_write_count % 2000 == 0){
            m_afLightCommPtr->set_parent_name(m_parentName);
            m_write_count = 0;
        }
    }
#endif
}


afModel::afModel(afWorldPtr a_afWorld): afBaseObject(a_afWorld){
    //    m_pickDragVector = new cMesh();
    //    cCreateArrow(m_pickDragVector);
    //    m_pickDragVector->m_material->setPurpleAmethyst();
    //    m_pickDragVector->setShowEnabled(false);
    //    m_pickDragVector->setUseDisplayList(true);
    //    m_pickDragVector->markForUpdate(false);
    //    m_chaiWorld->addVisualMesh(m_pickDragVector);
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


bool afModel::createFromAttribs(afModelAttributes *a_attribs)
{
    afModelAttributes& attribs = *a_attribs;
    attribs.resolveRelativeNamespace();
    attribs.resolveRelativePathAttribs();

    bool enable_comm = a_attribs->m_enableComm;

    // Loading Rigid Bodies
    for (size_t i = 0; i < attribs.m_rigidBodyAttribs.size(); ++i) {
        afRigidBodyPtr rBodyPtr = new afRigidBody(m_afWorld, this);
        if (rBodyPtr->createFromAttribs(&attribs.m_rigidBodyAttribs[i])){
            string remaped_name = m_afWorld->addAFRigidBody(rBodyPtr);
            m_afRigidBodyMapLocal[rBodyPtr->getQualifiedIdentifier()] = rBodyPtr;
        }
    }

    // Loading Soft Bodies
    for (size_t i = 0; i < attribs.m_softBodyAttribs.size(); ++i) {
        afSoftBodyPtr sBodyPtr = new afSoftBody(m_afWorld, this);
        if (sBodyPtr->createFromAttribs(&attribs.m_softBodyAttribs[i])){
            string remaped_name = m_afWorld->addAFSoftBody(sBodyPtr);
            m_afSoftBodyMapLocal[sBodyPtr->getQualifiedIdentifier()] = sBodyPtr;
        }
    }

    /// Loading Sensors
    for (size_t i = 0; i < attribs.m_sensorAttribs.size(); ++i) {
        afSensorPtr sensorPtr = nullptr;
        string type_str;
        bool valid = false;
        // Check which type of sensor is this so we can cast appropriately beforehand
        switch (attribs.m_sensorAttribs[i]->m_sensorType) {
        case afSensorType::RAYTRACER:
        {
            sensorPtr = new afProximitySensor(m_afWorld, this);
            type_str = "PROXIMITY";
            afRayTracerSensorAttributes* senAttribs = (afRayTracerSensorAttributes*) attribs.m_sensorAttribs[i];
            valid = ((afRayTracerSensor*)sensorPtr)->createFromAttribs(senAttribs);
            break;
        }
        case afSensorType::RESISTANCE:
        {
            sensorPtr = new afResistanceSensor(m_afWorld, this);
            type_str = "RESISTANCE";
            afResistanceSensorAttributes* senAttribs = (afResistanceSensorAttributes*) attribs.m_sensorAttribs[i];
            valid = ((afResistanceSensor*)sensorPtr)->createFromAttribs(senAttribs);
            break;
        }
        default:
            continue;
        }

        if (valid){
            string remaped_name = m_afWorld->addAFSensor(sensorPtr);
        }
    }

    // Loading Actuators
    for (size_t i = 0; i < attribs.m_actuatorAttribs.size(); ++i) {
        afActuatorPtr actuatorPtr = nullptr;
        string type_str;
        bool valid = false;
        switch (attribs.m_actuatorAttribs[i]->m_actuatorType) {
        case afActuatorType::CONSTRAINT:{
            actuatorPtr = new afConstraintActuator(m_afWorld, this);
            type_str = "CONSTRAINT";
            afConstraintActuatorAttributes* actAttribs = (afConstraintActuatorAttributes*)attribs.m_actuatorAttribs[i];
            valid = ((afConstraintActuator*)actuatorPtr)->createFromAttribs(actAttribs);
            break;
        }
        default:
            continue;
        }

        if (valid){
            string remaped_name = m_afWorld->addAFActuator(actuatorPtr);
        }
    }

    // Loading Joints
    for (size_t i = 0; i < attribs.m_jointAttribs.size(); ++i) {
        afJointPtr jntPtr = new afJoint(m_afWorld, this);
        if (jntPtr->createFromAttribs(&attribs.m_jointAttribs[i])){
            string remaped_name = m_afWorld->addAFJoint(jntPtr);
            m_afJointMapLocal[jntPtr->getQualifiedIdentifier()] = jntPtr;
        }
    }


    afVehiclePtr vehiclePtr;
    for (size_t i = 0; i < attribs.m_vehicleAttribs.size(); ++i) {
        vehiclePtr = new afVehicle(m_afWorld, this);
        if (vehiclePtr->createFromAttribs(&attribs.m_vehicleAttribs[i])){
            string remap_name = m_afWorld->addAFVehicle(vehiclePtr);
            m_afVehicleMapLocal[vehiclePtr->getQualifiedIdentifier()] = vehiclePtr;
        }
    }

    // This flag would ignore collision for all the multibodies in the scene

    if (attribs.m_ignoreInterCollision){
        ignoreCollisionChecking();
    }

    m_afWorld->buildCollisionGroups();

    return true;
}


///
/// \brief afModel::getRigidBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afRigidBodyPtr afModel::getAFRigidBodyLocal(string a_name, bool suppress_warning){
    if (m_afRigidBodyMapLocal.find(a_name) != m_afRigidBodyMapLocal.end()){
        return m_afRigidBodyMapLocal[a_name];
    }
    else{
        if (!suppress_warning){
            cerr << "WARNING: CAN'T FIND ANY BODY NAMED: " << a_name << " IN LOCAL MAP" << endl;

            cerr <<"Existing Bodies in Map: " << m_afRigidBodyMapLocal.size() << endl;
            afRigidBodyMap::iterator rbIt = m_afRigidBodyMapLocal.begin();
            for (; rbIt != m_afRigidBodyMapLocal.end() ; ++rbIt){
                cerr << rbIt->first << endl;
            }
        }
        return nullptr;
    }
}

///
/// \brief afModel::removeCollisionChecking
///
void afModel::ignoreCollisionChecking(){

    /// Only ignore collision checking between the bodies
    /// defined in the specific model config file
    /// and not all the bodies in the world
    afRigidBodyMap::iterator rBodyItA = m_afRigidBodyMapLocal.begin();
    vector<btRigidBody*> rBodiesVec;
    rBodiesVec.resize(m_afRigidBodyMapLocal.size());
    uint i=0;
    for ( ; rBodyItA != m_afRigidBodyMapLocal.end() ; ++rBodyItA){
        rBodiesVec[i] = rBodyItA->second->m_bulletRigidBody;
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
    afRigidBodyMap* rbMap = m_afWorld->getAFRigidBodyMap();
    afRigidBodyMap::iterator rBodyIt = rbMap->begin();
    vector<btRigidBody*> bodyFamily;
    pair<btVector3, btRigidBody*> pvtAandConnectedBody;
    vector< pair<btVector3, btRigidBody*> > pvtAandConnectedBodyVec;
    for ( ; rBodyIt != rbMap->end() ; ++rBodyIt){
        afRigidBodyPtr afBody = rBodyIt->second;
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



template <typename T, typename TMap>
///
/// \brief afWorld::addObject
/// \param a_obj
/// \param a_name
/// \param a_map
/// \return
///
bool afWorld::addAFObject(T a_obj, string a_name, TMap* a_map){
    (*a_map)[a_name] = a_obj;
    if (checkIfExists(a_obj) == false){
        m_childrenAFObjects.push_back(a_obj);
        a_obj->showVisualFrame();
    }
    return true;
}


template <typename T, typename TMap>
///
/// \brief afWorld::getObject
/// \param a_name
/// \param map
/// \param suppress_warning
/// \return
///
T afWorld::getAFObject(string a_name, TMap* a_map, bool suppress_warning){
    if (a_map->find(a_name) != a_map->end()){
        return ((*a_map)[a_name]);
    }
    // We didn't find the object using the full name, try checking if the name is a substring of the fully qualified name
    int matching_obj_count = 0;
    vector<string> matching_obj_names;
    T objHandle;
    typename TMap::iterator oIt = a_map->begin();
    for (; oIt != a_map->end() ; ++oIt){
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
    else if(matching_obj_count > 1){
        cerr << "WARNING: MULTIPLE OBJECTS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < matching_obj_names.size() ; i++){
            cerr << "\t" << i << ") " << matching_obj_names[i] << endl;
        }
        return nullptr;
    }
    else{
        if (!suppress_warning){
            cerr << "WARNING: CAN'T FIND ANY OBJECTS NAMED: \"" << a_name << "\" IN GLOBAL MAP \n";

            cerr <<"Existing OBJECTS in Map: " << a_map->size() << endl;
            typename TMap::iterator oIt = a_map->begin();
            for (; oIt != a_map->end() ; ++oIt){
                cerr << oIt->first << endl;
            }
        }
        return nullptr;
    }
}


template <typename TVec, typename TMap>
///
/// \brief afWorld::getObjects
/// \param a_map
/// \return
///
TVec afWorld::getAFObjects(TMap* a_map){
    TVec objects;
    typename TMap::iterator oIt;

    for (oIt = a_map->begin() ; oIt != a_map->end() ; ++oIt){
        objects.push_back(oIt->second);
    }

    return objects;
}


///
/// \brief afWorld::getAFLight
/// \param a_name
/// \param suppress_warning
/// \return
///
afLightPtr afWorld::getAFLight(string a_name, bool suppress_warning){
    return getAFObject<afLightPtr, afLightMap>(a_name, &m_afLightMap, suppress_warning);
}


///
/// \brief afWorld::getAFCamera
/// \param a_name
/// \param suppress_warning
/// \return
///
afCameraPtr afWorld::getAFCamera(string a_name, bool suppress_warning){
    return getAFObject<afCameraPtr, afCameraMap>(a_name, &m_afCameraMap, suppress_warning);
}


///
/// \brief afModel::getRidigBody
/// \param a_name
/// \return
///
afRigidBodyPtr afWorld::getAFRigidBody(string a_name, bool suppress_warning){
    return getAFObject<afRigidBodyPtr, afRigidBodyMap>(a_name, &m_afRigidBodyMap, suppress_warning);
}


///
/// \brief afWorld::getAFRigidBody
/// \param a_body
/// \param suppress_warning
/// \return
///
afRigidBodyPtr afWorld::getAFRigidBody(btRigidBody* a_body, bool suppress_warning){
    afRigidBodyMap::iterator afIt;
    for (afIt = m_afRigidBodyMap.begin() ; afIt != m_afRigidBodyMap.end() ; ++ afIt){
        afRigidBodyPtr afBody = afIt->second;
        if (a_body == afBody->m_bulletRigidBody){
            return afBody;
        }
    }
    if (!suppress_warning){
        cerr << "WARNING: CAN'T FIND ANY BODY BOUND TO BULLET RIGID BODY: \"" << a_body << "\"\n";

        cerr <<"Existing Bodies in Map: " << m_afRigidBodyMap.size() << endl;
        afRigidBodyMap::iterator rbIt = m_afRigidBodyMap.begin();
        for (; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
            cerr << rbIt->first << endl;
        }
    }
    return nullptr;
}


///
/// \brief afWorld::getAFSoftBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afSoftBodyPtr afWorld::getAFSoftBody(string a_name, bool suppress_warning){
    return getAFObject<afSoftBodyPtr, afSoftBodyMap>(a_name, &m_afSoftBodyMap, suppress_warning);
}


///
/// \brief afWorld::getAFSoftBody
/// \param a_body
/// \param suppress_warning
/// \return
///
afSoftBodyPtr afWorld::getAFSoftBody(btSoftBody* a_body, bool suppress_warning){
    afSoftBodyMap::iterator afIt;
    for (afIt = m_afSoftBodyMap.begin() ; afIt != m_afSoftBodyMap.end() ; ++ afIt){
        afSoftBodyPtr afBody = afIt->second;
        if (a_body == afBody->m_bulletSoftBody){
            return afBody;
        }
    }
    if (!suppress_warning){
        cerr << "WARNING: CAN'T FIND ANY BODY BOUND TO BULLET RIGID BODY: \"" << a_body << "\"\n";

        cerr <<"Existing Bodies in Map: " << m_afSoftBodyMap.size() << endl;
        afSoftBodyMap::iterator sbIt = m_afSoftBodyMap.begin();
        for (; sbIt != m_afSoftBodyMap.end() ; ++sbIt){
            cerr << sbIt->first << endl;
        }
    }
    return nullptr;
}


///
/// \brief afWorld::getAFModel
/// \param a_name
/// \param suppress_warning
/// \return
///
afModelPtr afWorld::getAFModel(string a_name, bool suppress_warning){
    return getAFObject<afModelPtr, afModelMap>(a_name, &m_afModelMap, suppress_warning);
}


///
/// \brief afWorld::getAFVehicle
/// \param a_name
/// \param suppress_warning
/// \return
///
afVehiclePtr afWorld::getAFVehicle(string a_name, bool suppress_warning){
    return getAFObject<afVehiclePtr, afVehicleMap>(a_name, &m_afVehicleMap, suppress_warning);
}


///
/// \brief afModel::getRootRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afWorld::getRootAFRigidBody(afRigidBodyPtr a_bodyPtr){
    if (!a_bodyPtr){
        cerr << "ERROR, BODY PTR IS NULL, CAN\'T LOOK UP ROOT BODIES" << endl;
        return nullptr;
    }

    /// Find Root Body
    afRigidBodyPtr rootParentBody;
    vector<int> bodyParentsCount;
    size_t rootParents = 0;
    if (a_bodyPtr->m_parentBodies.size() == 0){
        rootParentBody = a_bodyPtr;
        rootParents++;
    }
    else{
        bodyParentsCount.resize(a_bodyPtr->m_parentBodies.size());
        vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
        for (uint parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
            if ((*rIt)->m_parentBodies.size() == 0){
                rootParentBody = (*rIt);
                rootParents++;
            }
            bodyParentsCount[parentNum] = (*rIt)->m_parentBodies.size();
        }
    }

    // In case no root parent is found, it is understood that
    // the model chain is cyclical, perhaps return
    // the body with least number of parents
    if (rootParents == 0){
        auto minLineage = min_element(bodyParentsCount.begin(), bodyParentsCount.end());
        int idx = distance(bodyParentsCount.begin(), minLineage);
        rootParentBody = a_bodyPtr->m_parentBodies[idx];
        rootParents++;
        cerr << "WARNING! CYCLICAL CHAIN OF BODIES FOUND WITH NO UNIQUE PARENT, RETURING THE BODY WITH LEAST PARENTS";
    }

    if (rootParents > 1)
        cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

    return rootParentBody;
}


///
/// \brief afModel::getRootAFRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afModel::getRootAFRigidBodyLocal(afRigidBodyPtr a_bodyPtr){
    /// Find Root Body
    afRigidBodyPtr rootParentBody;
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
        bodyParentsCount.resize(m_afRigidBodyMapLocal.size());
        afRigidBodyMap::const_iterator mIt = m_afRigidBodyMapLocal.begin();
        for(int bodyNum=0; mIt != m_afRigidBodyMapLocal.end() ; bodyNum++, ++mIt){
            if ((*mIt).second->m_parentBodies.size() == 0){
                rootParentBody = (*mIt).second;
                ++rootParents;
            }
            bodyParentsCount[bodyNum] = (*mIt).second->m_parentBodies.size();
        }

    }

    if (rootParents > 1)
        cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

    return rootParentBody;
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
}

afVehicle::afVehicle(afWorldPtr a_afWorld, afModelPtr a_modelPtr): afInertialObject(a_afWorld, a_modelPtr){

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
    afVehicleAttributes &attribs = *a_attribs;

    bool result = true;

    setIdentifier(attribs.m_identifier);
    setName(attribs.m_identificationAttribs.m_name);
    setNamespace(attribs.m_identificationAttribs.m_namespace);

    setMinPublishFrequency(attribs.m_communicationAttribs.m_minPublishFreq);
    setMaxPublishFrequency(attribs.m_communicationAttribs.m_maxPublishFreq);
    setPassive(attribs.m_communicationAttribs.m_passive);

    m_chassis = m_afWorld->getAFRigidBody(attribs.m_chassisBodyName);

    if (m_chassis == nullptr){
        result = false;
        return result;
    }

    // Get the inertial offset transform, so the wheels are offset properly.
    btTransform T_oInc = m_chassis->getInertialOffsetTransform();
    m_mass = m_chassis->getMass();
    m_inertia = m_chassis->getInertia();

    afPath high_res_filepath;

    m_numWheels = attribs.m_wheelAttribs.size();
    m_wheels.resize(m_numWheels);
    m_wheelAttribs = attribs.m_wheelAttribs;

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
            m_wheels[i].m_wheelBody = m_afWorld->getAFRigidBody(rbName);
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

    if (isPassive() == false){

        string remap_idx = afUtils::getNonCollidingIdx(getQualifiedName(), m_afWorld->getAFVehicleMap());

        afCreateCommInstance(afObjectType::VEHICLE,
                             getQualifiedName() + remap_idx,
                             m_afWorld->getGlobalNamespace(),
                             getMinPublishFrequency(),
                             getMaxPublishFrequency());
    }

    return result;
}


///
/// \brief afVehicle::afExecuteCommand
/// \param dt
///
void afVehicle::fetchCommands(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_msgs::VehicleCmd af_cmd = m_afVehicleCommPtr->get_command();

    int maxWheelCount;

    if (af_cmd.brake == true){
        for (int i = 0 ; i < m_numWheels ; i++){
            m_vehicle->applyEngineForce(0.0, i);
            m_vehicle->setBrake(m_wheelAttribs[i].m_brakePowerMax, i);
        }
    }
    else{
        for (int i = 0 ; i < m_numWheels ; i++){
            m_vehicle->setBrake(0.0, i);
        }

        maxWheelCount = af_cmd.wheel_power.size() <= m_numWheels ? af_cmd.wheel_power.size() : m_numWheels;

        for (int i = 0 ; i < maxWheelCount ; i++){
            double val = af_cmd.wheel_power[i];
            val = cClamp(val, -m_wheelAttribs[i].m_enginePowerMax, m_wheelAttribs[i].m_enginePowerMax);
            m_vehicle->applyEngineForce(val, i);
        }

        maxWheelCount = af_cmd.wheel_brake.size() <= m_numWheels ? af_cmd.wheel_brake.size() : m_numWheels;

        for (int i = 0 ; i < maxWheelCount ; i++){
            double val = af_cmd.wheel_brake[i];
            val = cClamp(val, 0.0, m_wheelAttribs[i].m_brakePowerMax);
            m_vehicle->setBrake(val, i);
        }
    }

    maxWheelCount = af_cmd.wheel_steering.size() <= m_numWheels ? af_cmd.wheel_steering.size() : m_numWheels;

    for (int i = 0 ; i < maxWheelCount ; i++){
        double val = af_cmd.wheel_steering[i];
        val = cClamp(val, m_wheelAttribs[i].m_steeringLimitMin, m_wheelAttribs[i].m_steeringLimitMax);
        m_vehicle->setSteeringValue(val, i);
    }



    // Apply forces and torques on the chassis
    btVector3 force(af_cmd.chassis_wrench.force.x,
                    af_cmd.chassis_wrench.force.y,
                    af_cmd.chassis_wrench.force.z);
    btVector3 torque(af_cmd.chassis_wrench.torque.x,
                     af_cmd.chassis_wrench.torque.y,
                     af_cmd.chassis_wrench.torque.z);

    if (force.length() > 0.0){
        m_chassis->m_bulletRigidBody->applyCentralForce(force);
    }
    if (torque.length() > 0.0){
        m_chassis->m_bulletRigidBody->applyTorque(torque);
    }

#endif
}


///
/// \brief afVehicle::updatePositionFromDynamics
///
void afVehicle::update(){
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

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afVehicleCommPtr.get() != nullptr){

        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        cVector3d localPos = getLocalPos();
        m_afVehicleCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
        cQuaternion q;
        q.fromRotMat(getLocalRot());
        m_afVehicleCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

        // Since the mass and inertia aren't going to change that often, write them
        // out intermittently
        if (m_write_count % 2000 == 0){
            m_afVehicleCommPtr->set_wheel_count(m_numWheels);
            m_afVehicleCommPtr->set_mass(m_mass);
            m_afVehicleCommPtr->set_principal_inertia(getInertia().x(), getInertia().y(), getInertia().z());
        }
    }
#endif
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

afPointCloud::afPointCloud(afWorldPtr a_afWorld): afBaseObject(a_afWorld)
{

}


void afPointCloud::update()
{
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int mp_size = m_mpPtr->getNumPoints();
    sensor_msgs::PointCloudPtr pcPtr = m_pcCommPtr->get_point_cloud();
    if(pcPtr){
        double radius = m_pcCommPtr->get_radius();
        m_mpPtr->setPointSize(radius);
        int pc_size = pcPtr->points.size();
        int diff = pc_size - mp_size;
        string frame_id = pcPtr->header.frame_id;

        if (m_parentName.compare(frame_id) != 0 ){
            // First remove any existing parent
            if (m_mpPtr->getParent() != nullptr){
                m_mpPtr->getParent()->removeChild(m_mpPtr);
            }

            afRigidBodyPtr pBody = m_afWorld->getAFRigidBody(frame_id);
            if(pBody){
                pBody->addChildObject(this);
            }
            else{
                // Parent not found.
                cerr << "WARNING! FOR POINT CLOUD \""<< m_topicName <<
                        "\" PARENT BODY \"" << frame_id <<
                        "\" NOT FOUND" << endl;
            }
        }

        m_parentName = frame_id;

        if (diff >= 0){
            // PC array has either increased in size or the same size as MP array
            for (int pIdx = 0 ; pIdx < mp_size ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                m_mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
            }

            // Now add the new PC points to MP
            for (int pIdx = mp_size ; pIdx < mp_size + pc_size ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                m_mpPtr->newPoint(pcPos);
            }
        }
        else{
            // PC array has decreased in size as compared to MP array
            for (int pIdx = 0 ; pIdx < pc_size ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                m_mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
            }

            for (int pIdx = mp_size ; pIdx > pc_size ; pIdx--){
                m_mpPtr->removePoint(pIdx-1);
            }
        }

    }

#endif
}
