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

#include "afUtils.h"
#include "yaml-cpp/yaml.h"
#include "afAttributes.h"

using namespace ambf;

template<typename T>
///
/// \brief afUtils::getNonCollidingIdx
/// \param a_body_name
/// \param tMap
/// \return
///
std::string afUtils::getNonCollidingIdx(std::string a_body_name, const T* tMap){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (tMap->find(a_body_name) == tMap->end()){
        return remap_string;
    }
    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(tMap->find(a_body_name + remap_string) != tMap->end() && occurances < 100);
    return remap_string;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<cQuaternion, cVector3d>
/// \param v1
/// \param v2
/// \return
///
cQuaternion afUtils::getRotBetweenVectors<>(const cVector3d &v1, const cVector3d &v2){
    cQuaternion quat;
    double rot_angle = cAngle(v1, v2);
    if ( cAbs(rot_angle) < 0.1){
        quat.fromAxisAngle(cVector3d(0, 0, 1), rot_angle);
    }
    else if ( cAbs(rot_angle) > 3.13 ){
        cVector3d nx(1, 0, 0);
        double temp_ang = cAngle(v1, nx);
        if ( cAbs(temp_ang) > 0.1 && cAbs(temp_ang) < 3.13 ){
            cVector3d rot_axis = cCross(v1, nx);
            quat.fromAxisAngle(rot_axis, rot_angle);
        }
        else{
            cVector3d ny(0, 1, 0);
            cVector3d rot_axis = cCross(v2, ny);
            quat.fromAxisAngle(rot_axis, rot_angle);
        }
    }
    else{
        cVector3d rot_axis = cCross(v1, v2);
        quat.fromAxisAngle(rot_axis, rot_angle);
    }

    return quat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<cMatrix3d, cVector3d>
/// \param v1
/// \param v2
/// \return
///
cMatrix3d afUtils::getRotBetweenVectors<cMatrix3d, cVector3d>(const cVector3d &v1, const cVector3d &v2){
    cMatrix3d rot_mat;
    cQuaternion quat = getRotBetweenVectors<cQuaternion, cVector3d>(v1, v2);
    quat.toRotMat(rot_mat);
    return rot_mat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<btQuaternion, btVector3>
/// \param v1
/// \param v2
/// \return
///
btQuaternion afUtils::getRotBetweenVectors<btQuaternion, btVector3>(const btVector3 &v1, const btVector3 &v2){
    btQuaternion quat;
    double rot_angle = v1.angle(v2);
    if ( cAbs(rot_angle) < 0.1){
        quat.setEulerZYX(0,0,0);
    }
    else if ( cAbs(rot_angle) > 3.13 ){
        btVector3 nx(1, 0, 0);
        double temp_ang = v1.angle(nx);
        if ( cAbs(temp_ang) > 0.1 && cAbs(temp_ang) < 3.13 ){
            btVector3 rot_axis = v1.cross(nx);
            quat.setRotation(rot_axis, rot_angle);
        }
        else{
            btVector3 ny(0, 1, 0);
            btVector3 rot_axis = v2.cross(ny);
            quat.setRotation(rot_axis, rot_angle);
        }
    }
    else{
        btVector3 rot_axis = v1.cross(v2);
        quat.setRotation(rot_axis, rot_angle);
    }

    return quat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<btMatrix3x3, btVector3>
/// \param v1
/// \param v2
/// \return
///
btMatrix3x3 afUtils::getRotBetweenVectors<btMatrix3x3, btVector3>(const btVector3 &v1, const btVector3 &v2){
    btMatrix3x3 rot_mat;
    btQuaternion quat = getRotBetweenVectors<btQuaternion, btVector3>(v1, v2);
    rot_mat.setRotation(quat);
    return rot_mat;
}

template<>
///
/// \brief afUtils::convertDataTypes<cVector3d, btVector3>
/// \param p
/// \return
///
cVector3d afUtils::convertDataType<cVector3d, btVector3>(const btVector3 &p){
    cVector3d cPos(p.x(), p.y(), p.z());
    return cPos;
}

template<>
///
/// \brief afUtils::convertDataTypes<btVector3, cVector3d>
/// \param p
/// \return
///
btVector3 afUtils::convertDataType<btVector3, cVector3d>(const cVector3d &p){
    btVector3 btPos(p.x(), p.y(), p.z());
    return btPos;
}


template<>
///
/// \brief afUtils::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
cQuaternion afUtils::convertDataType<cQuaternion, btQuaternion>(const btQuaternion &q){
    cQuaternion cQuat(q.w(), q.x(), q.y(), q.z());
    return cQuat;
}


template<>
///
/// \brief afUtils::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
btQuaternion afUtils::convertDataType<btQuaternion, cQuaternion>(const cQuaternion &q){
    btQuaternion btQuat(q.x, q.y, q.z, q.w);
    return btQuat;
}


template<>
///
/// \brief afUtils::convertDataTypes<cMatrix3d, btMatrix3x3>
/// \param r
/// \return
///
cMatrix3d afUtils::convertDataType<cMatrix3d, btMatrix3x3>(const btMatrix3x3 &r){
    btQuaternion btQuat;
    r.getRotation(btQuat);

    cQuaternion cQuat(btQuat.w(), btQuat.x(), btQuat.y(), btQuat.z());

    cMatrix3d cMat;
    cQuat.toRotMat(cMat);

    return cMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
btMatrix3x3 afUtils::convertDataType<btMatrix3x3, cMatrix3d>(const cMatrix3d &r){
    cQuaternion cQuat;
    cQuat.fromRotMat(r);

    btQuaternion btQuat(cQuat.x, cQuat.y, cQuat.z, cQuat.w);
    btMatrix3x3 btMat;
    btMat.setRotation(btQuat);

    return btMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<cTransform, btTransform>
/// \param t
/// \return
///
cTransform afUtils::convertDataType<cTransform, btTransform>(const btTransform &t){
    cMatrix3d cRot = afUtils::convertDataType<cMatrix3d, btMatrix3x3>(t.getBasis());
    cVector3d cPos = afUtils::convertDataType<cVector3d, btVector3>(t.getOrigin());
    cTransform cMat(cPos, cRot);
    return cMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btTransform, cTransform>
/// \param t
/// \return
///
btTransform afUtils::convertDataType<btTransform, cTransform>(const cTransform &t){
    btMatrix3x3 btRot = afUtils::convertDataType<btMatrix3x3, cMatrix3d>(t.getLocalRot());
    btVector3 btPos = afUtils::convertDataType<btVector3, cVector3d>(t.getLocalPos());
    btTransform btMat(btRot, btPos);
    return btMat;
}


///
/// \brief afUtils::removeDoubleBackSlashes
/// \param a_name
/// \return
///
std::string afUtils::removeAdjacentBackSlashes(std::string a_name){
    std::string cleaned_name;
    int last_back_slash_idx = -2;
    for (int i = 0; i < a_name.length() ; i++){
        if (a_name[i] == '/'){
            if (i - last_back_slash_idx > 1){
                cleaned_name.push_back(a_name[i]);
            }
            last_back_slash_idx = i;
        }
        else{
            cleaned_name.push_back(a_name[i]);
        }
    }
    return cleaned_name;
}


///
/// \brief afUtils::mergeNamespace
/// \param a_namespace1
/// \return
///
std::string afUtils::mergeNamespace(std::string a_namespace1, std::string a_namespace2){
    a_namespace1 = removeAdjacentBackSlashes(a_namespace1);
    a_namespace2 = removeAdjacentBackSlashes(a_namespace2);

    if(a_namespace2.find('/') == 0){
        return a_namespace2;
    }
    else{
        return a_namespace1 + a_namespace2;
    }
}


///
/// \brief afUtils::getShapeTypeFromString
/// \param a_shape_str
/// \return
///
afPrimitiveShapeType afUtils::getShapeTypeFromString(const std::string &a_shape_str){
    afPrimitiveShapeType shapeType;
    if (a_shape_str.compare("Box") == 0 || a_shape_str.compare("box") == 0 || a_shape_str.compare("BOX") == 0){
        shapeType = afPrimitiveShapeType::BOX;
    }
    else if (a_shape_str.compare("Sphere") == 0 || a_shape_str.compare("sphere") == 0 || a_shape_str.compare("SPHERE") == 0){
        shapeType = afPrimitiveShapeType::SPHERE;
    }
    else if (a_shape_str.compare("Cylinder") == 0 || a_shape_str.compare("cylinder") == 0 || a_shape_str.compare("CYLINDER") == 0){
        shapeType = afPrimitiveShapeType::CYLINDER;
    }
    else if (a_shape_str.compare("Capsule") == 0 || a_shape_str.compare("capsule") == 0 || a_shape_str.compare("CAPSULE") == 0){
        shapeType = afPrimitiveShapeType::CAPSULE;
    }
    else if (a_shape_str.compare("Cone") == 0 || a_shape_str.compare("cone") == 0 || a_shape_str.compare("CONE") == 0){
        shapeType = afPrimitiveShapeType::CONE;
    }
    else if (a_shape_str.compare("Plane") == 0 || a_shape_str.compare("plane") == 0 || a_shape_str.compare("PLANE") == 0){
        shapeType = afPrimitiveShapeType::PLANE;

    }
    else{
        shapeType = afPrimitiveShapeType::INVALID;
    }

    return shapeType;
}


///
/// \brief afUtils::getMatrialFromNode
/// \param a_materialNode
/// \return
///
cMaterial afUtils::getMatrialAttribsFromNode(YAML::Node *a_node)
{
    YAML::Node& adfNode = *a_node;

    YAML::Node colorNameNode = adfNode["color"];
    YAML::Node colorRGBANode = adfNode["color rgba"];
    YAML::Node colorComponentsNode = adfNode["color components"];

    cMaterial mat;
    mat.setShininess(64);
    float r=0.5, g=0.5, b=0.5, a = 1.0;
    if(colorRGBANode.IsDefined()){
        r = colorRGBANode["r"].as<float>();
        g = colorRGBANode["g"].as<float>();
        b = colorRGBANode["b"].as<float>();
        a = colorRGBANode["a"].as<float>();
        mat.setColorf(r, g, b, a);
    }
    else if(colorComponentsNode.IsDefined()){
        if (colorComponentsNode["diffuse"].IsDefined()){
            r = colorComponentsNode["diffuse"]["r"].as<float>();
            g = colorComponentsNode["diffuse"]["g"].as<float>();
            b = colorComponentsNode["diffuse"]["b"].as<float>();
            mat.m_diffuse.set(r, g, b);
        }
        if (colorComponentsNode["ambient"].IsDefined()){
            float level = colorComponentsNode["ambient"]["level"].as<float>();
            r *= level;
            g *= level;
            b *= level;
            mat.m_ambient.set(r, g, b);
        }
        if (colorComponentsNode["specular"].IsDefined()){
            r = colorComponentsNode["specular"]["r"].as<float>();
            g = colorComponentsNode["specular"]["g"].as<float>();
            b = colorComponentsNode["specular"]["b"].as<float>();
            mat.m_specular.set(r, g, b);
        }
        if (colorComponentsNode["emission"].IsDefined()){
            r = colorComponentsNode["emission"]["r"].as<float>();
            g = colorComponentsNode["emission"]["g"].as<float>();
            b = colorComponentsNode["emission"]["b"].as<float>();
            mat.m_emission.set(r, g, b);
        }
        if (colorComponentsNode["shininess"].IsDefined()){
            unsigned int shininess;
            shininess = colorComponentsNode["shininess"].as<uint>();
            mat.setShininess(shininess);
        }
        a = colorComponentsNode["transparency"].as<float>();
    }
    else if(colorNameNode.IsDefined()){
        std::vector<double> rgba = afConfigHandler::getColorRGBA(colorNameNode.as<std::string>());
        mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
    }

    return mat;
}


///
/// \brief afUtils::getCartControllerFromNode
/// \param a_node
/// \return
///
afCartesianControllerAttributes afUtils::getCartControllerAttribsFromNode(YAML::Node *a_node)
{
    YAML::Node& controllerNode = *a_node;

    afCartesianControllerAttributes attribs;
    if(controllerNode.IsDefined()){
        // Check if the linear controller is defined
        if (controllerNode["linear"].IsDefined()){
            double P, I, D;
            P = controllerNode["linear"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (controllerNode["linear"]["I"].IsDefined()){
                I = controllerNode["linear"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = controllerNode["linear"]["D"].as<double>();
            attribs.P_lin = P;
            attribs.I_lin = I;
            attribs.D_lin = D;
            attribs.m_positionOutputType = afControlType::FORCE;
        }
        else{
            // Use preset values for the controller since we are going to be using its output for the
            // internal velocity controller
            attribs.P_lin = 10;
            attribs.I_lin = 0;
            attribs.D_lin = 0;
            attribs.m_positionOutputType = afControlType::VELOCITY;
        }

        // Check if the angular controller is defined
        if(controllerNode["angular"].IsDefined()){
            double P, I, D;
            P = controllerNode["angular"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (controllerNode["angular"]["I"].IsDefined()){
                I = controllerNode["angular"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = controllerNode["angular"]["D"].as<double>();
            attribs.P_ang = P;
            attribs.I_ang = I;
            attribs.D_ang = D;
            attribs.m_orientationOutputType = afControlType::FORCE;
        }
        else{
            // Use preset values for the controller since we are going to be using its output for the
            // internal velocity controller
            attribs.P_ang = 10;
            attribs.I_ang = 0;
            attribs.D_ang = 0;
            attribs.m_orientationOutputType = afControlType::VELOCITY;
        }
    }

    return attribs;
}


///
/// \brief afUtils::createVisualShape
/// \param a_primitiveShape
/// \return
///
cMesh* afUtils::createVisualShape(const afPrimitiveShapeAttributes& a_attribs){

    uint xs = 32;
    uint ys = 32;
    uint zs = 5;

    cVector3d dims = a_attribs.getDimensions();
    double dx = dims.x();
    double dy = dims.y();
    double dz = dims.z();

    cVector3d pNormal = a_attribs.getPlaneNormal();
    double nx = pNormal.x();
    double ny = pNormal.y();
    double nz = pNormal.z();

    double offset = a_attribs.getPlaneConstant();

    double radius = a_attribs.getRadius();
    double height = a_attribs.getHeight();

    cVector3d posOffset = a_attribs.getPosOffset();
    cMatrix3d rotOffset = a_attribs.getRotOffset();

    cMesh* tempMesh = new cMesh();

    switch (a_attribs.getShapeType()) {
    case afPrimitiveShapeType::BOX:{
        cCreateBox(tempMesh, dx, dy, dz, posOffset, rotOffset);
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        cCreateSphere(tempMesh, radius, xs, ys, posOffset, rotOffset);
        break;
    }
    case afPrimitiveShapeType::CYLINDER:{
        posOffset.set(posOffset.x(), posOffset.y(), posOffset.z() - 0.5 * height);
        cCreateCylinder(tempMesh, height, radius, xs, ys, zs, true, true, posOffset, rotOffset);
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        posOffset.set(posOffset.x(), posOffset.y(), posOffset.z() - 0.5 * height);
        cCreateEllipsoid(tempMesh, radius, radius, height, xs, ys, posOffset, rotOffset);
        break;
    }
    case afPrimitiveShapeType::CONE:{
        posOffset.set(posOffset.x(), posOffset.y(), posOffset.z() - 0.5 * height);
        cCreateCone(tempMesh, height, radius, 0, xs, ys, zs, true, true, posOffset, rotOffset);
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        cVector3d pos;
        cVector3d normal(nx, ny, nz);
        normal.normalize();
        pos = normal * offset;
        cQuaternion rot_quat = afUtils::getRotBetweenVectors<cQuaternion, cVector3d>(cVector3d(0, 0, 1), normal);
        cMatrix3d rot_mat;
        rot_quat.toRotMat(rot_mat);
        cCreatePlane(tempMesh, 100, 100, pos, rot_mat);
        break;
    }
    default:{
        delete tempMesh;
        tempMesh = nullptr;
        // Throw some warning or error as shape not understood
        break;
    }
    }
    return tempMesh;
}


///
/// \brief afUtils::createCollisionShape
/// \param a_primitiveShape
/// \return
///
btCollisionShape *afUtils::createCollisionShape(const afPrimitiveShapeAttributes *a_attribs)
{
    cVector3d dims = a_attribs->getDimensions();
    double dx = dims.x();
    double dy = dims.y();
    double dz = dims.z();

    cVector3d pNormal = a_attribs.getPlaneNormal();
    double nx = pNormal.x();
    double ny = pNormal.y();
    double nz = pNormal.z();

    double offset = a_attribs->getPlaneConstant();

    double radius = a_attribs->getRadius();
    double height = a_attribs->getHeight();

    btCollisionShape* tempCollisionShape;

    switch (a_attribs->getShapeType()) {
    case afPrimitiveShapeType::BOX:{
        btVector3 halfExtents(dx/2, dy/2, dz/2);
        tempCollisionShape = new btBoxShape(halfExtents);
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        tempCollisionShape = new btSphereShape(radius);
        break;
    }
    case afPrimitiveShapeType::CYLINDER:{
        height = height / 2;
        switch (a_attribs->getAxisType()) {
        case afAxisType::X:{
            btVector3 halfExtents(height, radius, radius);
            tempCollisionShape = new btCylinderShapeX(halfExtents);
            break;
        }
        case afAxisType::Y:{
            btVector3 halfExtents(radius, height, radius);
            tempCollisionShape = new btCylinderShape(halfExtents);
            break;
        }
        case afAxisType::Z:{
            btVector3 halfExtents(radius, radius, height);
            tempCollisionShape = new btCylinderShapeZ(halfExtents);
            break;
        }
        }
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        height = height - 2*radius;
        switch (a_attribs->getAxisType()) {
        case afAxisType::X:{
            tempCollisionShape = new btCapsuleShapeX(radius, height);
            break;
        }
        case afAxisType::Y:{
            tempCollisionShape = new btCapsuleShape(radius, height);
            break;
        }
        case afAxisType::Z:{
            tempCollisionShape = new btCapsuleShapeZ(radius, height);
            break;
        }
        }
        break;
    }
    case afPrimitiveShapeType::CONE:{
        switch (a_attribs->getAxisType()) {
        case afAxisType::X:{
            tempCollisionShape = new btConeShapeX(radius, height);
            break;
        }
        case afAxisType::Y:{
            tempCollisionShape = new btConeShape(radius, height);
            break;
        }
        case afAxisType::Z:{
            tempCollisionShape = new btConeShapeZ(radius, height);
            break;
        }
        }
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        tempCollisionShape = new btStaticPlaneShape(btVector3(nx, ny, nz), offset);
        break;
    }
    default:{
        tempCollisionShape = nullptr;
        // Throw some warning or error as shape not understood
        break;
    }
    }

    return tempCollisionShape;

}


///
/// \brief afUtils::createCollisionShapeFromMesh
/// \param a_collisionMesh
/// \param T_offset
/// \param a_margin
/// \return
///
btCompoundShape* afUtils::createCollisionShapeFromMesh(cMultiMesh* a_collisionMesh, btTransform T_offset, double a_margin){

    if (a_collisionMesh == nullptr){
        // ERROR THE MESH IS A NULLPTR
        return nullptr;
    }

    if (a_collisionMesh->getNumMeshes() == 0){
        // NO MESHES IN THIS MULTIMESH
        return nullptr;
    }

    btCompoundShape* compoundShape = new btCompoundShape();

    for (uint mI = 0 ; mI < a_collisionMesh->getNumMeshes() ; mI++){

        cMesh* mesh = a_collisionMesh->getMesh(mI);

        if (mesh == nullptr){
            continue;
        }

        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = mesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int tI = 0; tI < numTriangles; tI++)
        {
            unsigned int vertexIndex0 = mesh->m_triangles->getVertexIndex0(tI);
            unsigned int vertexIndex1 = mesh->m_triangles->getVertexIndex1(tI);
            unsigned int vertexIndex2 = mesh->m_triangles->getVertexIndex2(tI);

            btVector3 vtx0 = afUtils::convertDataType<btVector3, cVector3d>(mesh->m_vertices->getLocalPos(vertexIndex0));
            btVector3 vtx1 = afUtils::convertDataType<btVector3, cVector3d>(mesh->m_vertices->getLocalPos(vertexIndex1));
            btVector3 vtx2 = afUtils::convertDataType<btVector3, cVector3d>(mesh->m_vertices->getLocalPos(vertexIndex2));

            bulletMesh->addTriangle(vtx0, vtx1, vtx2);
        }

        // create mesh collision model
        btGImpactMeshShape* singleShape = new btGImpactMeshShape(bulletMesh);

        // assign settings
        singleShape->setMargin(a_margin);
        singleShape->updateBound();

        compoundShape->addChildShape(T_offset, singleShape);
    }

    return compoundShape;
}


///////////////////////////////////////////////

