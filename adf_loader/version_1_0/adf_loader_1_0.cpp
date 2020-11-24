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

#include "adf_loader_1_0.h"
#include "afUtils.h"

using namespace ambf;

template <>
///
/// \brief toXYZ<btVector3>
/// \param node
/// \return
///
btVector3 ADFUtils::toXYZ<btVector3>(YAML::Node* a_node){
    btVector3 v;
    YAML::Node & node = *a_node;
    v.setX(node["x"].as<double>());
    v.setY(node["y"].as<double>());
    v.setZ(node["z"].as<double>());
    return v;
}

template <>
///
/// \brief toXYZ<cVector3d>
/// \param node
/// \return
///
cVector3d ADFUtils::toXYZ<cVector3d>(YAML::Node* a_node){
    cVector3d v;
    YAML::Node & node = *a_node;
    v.x(node["x"].as<double>());
    v.y(node["y"].as<double>());
    v.z(node["z"].as<double>());
    return v;
}

template<>
///
/// \brief toRPY<btVector3>
/// \param node
/// \return
///
btVector3 ADFUtils::toRPY<btVector3>(YAML::Node* a_node){
    btVector3 v;
    YAML::Node & node = *a_node;
    v.setX(node["r"].as<double>());
    v.setY(node["p"].as<double>());
    v.setZ(node["y"].as<double>());
    return v;
}

template<>
///
/// \brief toRPY<cVector3>
/// \param node
/// \return
///
cVector3d ADFUtils::toRPY<cVector3d>(YAML::Node *node){
    cVector3d v;
    v.x((*node)["r"].as<double>());
    v.y((*node)["p"].as<double>());
    v.z((*node)["y"].as<double>());
    return v;
}


///
/// \brief ADFUtils::getMatrialAttribsFromNode
/// \param a_node
/// \param mat
/// \return
///
bool ADFUtils::getMatrialFromNode(YAML::Node *mat_node, cMaterial* m)
{
    YAML::Node& matNode = *mat_node;

    YAML::Node colorNameNode = matNode["color"];
    YAML::Node colorRGBANode = matNode["color rgba"];
    YAML::Node colorComponentsNode = matNode["color components"];

    cMaterial& mat = *m;
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
//    else if(colorNameNode.IsDefined()){
//        std::vector<double> rgba = afConfigHandler::getColorRGBA(colorNameNode.as<std::string>());
//        mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
//    }

    return true;
}


///
/// \brief afUtils::getCartControllerFromNode
/// \param a_node
/// \return
///
bool ADFUtils::getCartControllerAttribsFromNode(YAML::Node *a_node, afCartesianControllerAttributes* attribs)
{
    YAML::Node& controllerNode = *a_node;
    if(controllerNode.IsDefined()){
        double P, I, D;
        // Check if the linear controller is defined
        if (controllerNode["linear"].IsDefined()){
            P = controllerNode["linear"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (controllerNode["linear"]["I"].IsDefined()){
                I = controllerNode["linear"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = controllerNode["linear"]["D"].as<double>();
            attribs->m_positionOutputType = afControlType::FORCE;
        }
        else{
            // Use preset values for the controller since we are going to be using its output for the
            // internal velocity controller
            P = 10;
            I = 0;
            D = 0;
            attribs->m_positionOutputType = afControlType::VELOCITY;
        }
        attribs->P_lin = P;
        attribs->I_lin = I;
        attribs->D_lin = D;

        // Check if the angular controller is defined
        if(controllerNode["angular"].IsDefined()){
            P = controllerNode["angular"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (controllerNode["angular"]["I"].IsDefined()){
                I = controllerNode["angular"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = controllerNode["angular"]["D"].as<double>();
            attribs->m_orientationOutputType = afControlType::FORCE;
        }
        else{
            // Use preset values for the controller since we are going to be using its output for the
            // internal velocity controller
            P = 10;
            I = 0;
            D = 0;
            attribs->m_orientationOutputType = afControlType::VELOCITY;
        }
        attribs->P_ang = P;
        attribs->I_ang = I;
        attribs->D_ang = D;
    }
    else{
        return false;
    }

    return true;
}


///
/// \brief ADFUtils::getShapeTypeFromString
/// \param a_shape_str
/// \return
///
afPrimitiveShapeType ADFUtils::getShapeTypeFromString(const std::string &a_shape_str){
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
/// \brief ADFUtils::copyShapeOffsetData
/// \param offset_node
/// \param attribs
/// \return
///
bool ADFUtils::copyShapeOffsetData(YAML::Node *offset_node, afPrimitiveShapeAttributes* attribs)
{
    bool valid = true;

    YAML::Node offsetNode = *offset_node;

    if (offsetNode.IsDefined()){

        if (offsetNode["position"].IsDefined()){
            double px = offsetNode["position"]["x"].as<double>();
            double py = offsetNode["position"]["y"].as<double>();
            double pz = offsetNode["position"]["z"].as<double>();
            attribs->m_posOffset.set(px, py, pz);
        }
        else{
            attribs->m_posOffset.set(0, 0, 0);
        }

        if (offsetNode["orientation"].IsDefined()){
            double roll =  offsetNode["orientation"]["r"].as<double>();
            double pitch = offsetNode["orientation"]["p"].as<double>();
            double yaw =   offsetNode["orientation"]["y"].as<double>();
            attribs->m_rotOffset.setExtrinsicEulerRotationRad(roll,pitch,yaw,cEulerOrder::C_EULER_ORDER_XYZ);
        }
        else{
            attribs->m_rotOffset.identity();
        }
    }
    else{
        valid = false;
    }

    return valid;
}


///
/// \brief ADFUtils::copyPrimitiveShapeData
/// \param shape_node
/// \param attribs
/// \return
///
bool ADFUtils::copyPrimitiveShapeData(YAML::Node *shape_node, afPrimitiveShapeAttributes* attribs)
{
    bool valid = true;

    YAML::Node shapeNode = *shape_node;

    if(shapeNode["axis"].IsDefined()){
        std::string axis = shapeNode["axis"].as<std::string>();

        if (axis.compare("x") == 0 || axis.compare("X") == 0){
            attribs->m_axisType = afAxisType::X;
        }
        else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
            attribs->m_axisType = afAxisType::Y;
        }
        else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
            attribs->m_axisType = afAxisType::Z;
        }
        else{
            std::cerr << "WARNING: Axis string \"" << axis << "\" not understood!\n";
            attribs->m_axisType = afAxisType::Z;
        }
    }

    switch (attribs->m_shapeType) {
    case afPrimitiveShapeType::BOX:{
        double dx, dy, dz;
        dx = shapeNode["x"].as<double>();
        dy = shapeNode["y"].as<double>();
        dz = shapeNode["z"].as<double>();
        attribs->m_dimensions.set(dx, dy, dz);
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        attribs->m_radius = shapeNode["radius"].as<double>();
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        attribs->m_radius = shapeNode["radius"].as<double>();
        attribs->m_height = shapeNode["height"].as<double>();
        break;
    }
    case afPrimitiveShapeType::CONE:{
        attribs->m_radius = shapeNode["radius"].as<double>();
        attribs->m_height = shapeNode["height"].as<double>();
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        double nx, ny, nz;
        nx = shapeNode["normal"]["x"].as<double>();
        ny = shapeNode["normal"]["y"].as<double>();
        nz = shapeNode["normal"]["z"].as<double>();
        attribs->m_planeNormal.set(nx, ny, nz);
        attribs->m_planeConstant = shapeNode["offset"].as<double>();
        break;
    }
    default:{
        valid = false;
        break;
    }
    }

    return valid;
}


bool ADFLoader_1_0::loadRigidBody(std::string rb_config_file, std::string node_name, ambf::afRigidBodyAttributes *a_attribsRB)
{
    YAML::Node rootNode;
    try{
        rootNode = YAML::LoadFile(rb_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << rb_config_file << std::endl;
        return 0;
    }
    YAML::Node rb_node = rootNode[node_name];
    return loadRigidBody(&rb_node, a_attribsRB);

}

bool ADFLoader_1_0::loadRigidBody(YAML::Node *rb_node, ambf::afRigidBodyAttributes *a_attribsRB)
{
    YAML::Node& rbNode = *rb_node;
    if (rbNode.IsNull()){
        std::cerr << "ERROR: RIGID BODY " << rbNode << " NODE IS NULL\n";
        return false;
    }

    if (a_attribsRB == nullptr){
        std::cerr << "ERROR: RIGID BODY ATTRIBUTES IS A NULLPTR\n";
        return false;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = rbNode["name"];
    YAML::Node namespaceNode = rbNode["namespace"];
    YAML::Node meshPathHRNode = rbNode["high resolution path"];
    YAML::Node meshPathLRNode = rbNode["low resolution path"];
    YAML::Node posNode = rbNode["location"]["position"];
    YAML::Node rotNode = rbNode["location"]["orientation"];
    YAML::Node scaleNode = rbNode["scale"];
    YAML::Node meshNode = rbNode["mesh"];
    YAML::Node shapeNode = rbNode["shape"];
    YAML::Node compoundShapeNode = rbNode["compound shape"];
    YAML::Node geometryNode = rbNode["geometry"];
    YAML::Node collisionMarginNode = rbNode["collision margin"];
    YAML::Node collisionMeshNode = rbNode["collision mesh"];
    YAML::Node collisionShapeNode = rbNode["collision shape"];
    YAML::Node collisionOffsetNode = rbNode["collision offset"];
    YAML::Node collisionGeometryNode = rbNode["collision geometry"];
    YAML::Node compoundCollisionShapeNode = rbNode["compound collision shape"];
    YAML::Node massNode = rbNode["mass"];
    YAML::Node inertiaNode = rbNode["inertia"];
    YAML::Node inertialOffsetPosNode = rbNode["inertial offset"]["position"];
    YAML::Node inertialOffsetRotNode = rbNode["inertial offset"]["orientation"];
    YAML::Node controllerNode = rbNode["controller"];
    YAML::Node linDampingNode = rbNode["damping"]["linear"];
    YAML::Node angDampingNode = rbNode["damping"]["angular"];
    YAML::Node staticFrictionNode = rbNode["friction"]["static"];
    YAML::Node rollingFrictionNode = rbNode["friction"]["rolling"];
    YAML::Node restitutionNode = rbNode["restitution"];
    YAML::Node publishChildrenNamesNode = rbNode["publish children names"];
    YAML::Node publishJointNamesNode = rbNode["publish joint names"];
    YAML::Node publishJointPositionsNode = rbNode["publish joint positions"];
    YAML::Node publishFrequencyNode = rbNode["publish frequency"];
    YAML::Node collisionGroupsNode = rbNode["collision groups"];
    YAML::Node passiveNode = rbNode["passive"];
    YAML::Node shadersNode = rbNode["shaders"];


    if(nameNode.IsDefined()){
        std::string name = nameNode.as<std::string>();
        name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
        a_attribsRB->m_name = name;
    }

    a_attribsRB->m_visualGeometryType = afGeometryType::INVALID;
    a_attribsRB->m_collisionGeometryType = afGeometryType::INVALID;

    std::string visual_shape_str;
    std::string collision_shape_str;

    if (collisionShapeNode.IsDefined()){
        a_attribsRB->m_collisionGeometryType = afGeometryType::SINGLE_SHAPE;
        collision_shape_str = collisionShapeNode.as<std::string>();
    }
    else if (compoundCollisionShapeNode.IsDefined()){
        a_attribsRB->m_collisionGeometryType = afGeometryType::COMPOUND_SHAPE;
    }

    if (shapeNode.IsDefined()){
        a_attribsRB->m_visualGeometryType = afGeometryType::SINGLE_SHAPE;
        visual_shape_str = shapeNode.as<std::string>();
        if (a_attribsRB->m_collisionGeometryType == afGeometryType::INVALID){
            collision_shape_str = visual_shape_str;
            a_attribsRB->m_collisionGeometryType = afGeometryType::SINGLE_SHAPE;
            collisionGeometryNode = geometryNode;
            collisionShapeNode = shapeNode;
        }
    }
    else if (compoundShapeNode.IsDefined()){
        a_attribsRB->m_visualGeometryType = afGeometryType::COMPOUND_SHAPE;
        if (a_attribsRB->m_collisionGeometryType == afGeometryType::INVALID){
            a_attribsRB->m_collisionGeometryType = afGeometryType::COMPOUND_SHAPE;
            compoundCollisionShapeNode = compoundShapeNode;
        }
    }
    else if(meshNode.IsDefined()){
        a_attribsRB->m_visualMeshName= meshNode.as<std::string>();
        if (!a_attribsRB->m_visualMeshName.empty()){
            // Each ridig body can have a seperate path for its low and high res meshes
            // Incase they are defined, we use those paths and if they are not, we use
            // the paths for the whole file
            if (meshPathHRNode.IsDefined()){
                a_attribsRB->m_visualMeshFilePath = meshPathHRNode.as<std::string>() + a_attribsRB->m_visualMeshName;
            }
            a_attribsRB->m_visualGeometryType = afGeometryType::MESH;
        }

        // Only check for collision mesh definition if visual mesh is defined
        if (a_attribsRB->m_collisionGeometryType == afGeometryType::INVALID){
            if(collisionMeshNode.IsDefined()){
                a_attribsRB->m_collisionMeshName = collisionMeshNode.as<std::string>();
            }
            else{
                a_attribsRB->m_collisionMeshName = a_attribsRB->m_visualMeshName;
            }

            if (!a_attribsRB->m_collisionMeshName.empty()){
                if (meshPathLRNode.IsDefined()){
                    a_attribsRB->m_collisionMeshFilePath = meshPathLRNode.as<std::string>() + a_attribsRB->m_collisionMeshName;
                }
                else{
                    // If low res path is not defined, use the high res path to load the high-res mesh for collision
                    a_attribsRB->m_collisionMeshFilePath = a_attribsRB->m_visualMeshFilePath / a_attribsRB->m_collisionMeshName;
                }
                a_attribsRB->m_collisionGeometryType = afGeometryType::MESH;
            }
        }
    }

    if(!massNode.IsDefined()){
        std::cerr << "WARNING: Body "
                  << a_attribsRB->m_name
                  << "'s mass is not defined, hence ignoring\n";
        return false;
    }
    else if(massNode.as<double>() < 0.0){
        std::cerr << "WARNING: Body "
                  << a_attribsRB->m_name
                  << "'s mass is \"" << a_attribsRB->m_mass << "\". Mass cannot be negative, ignoring\n";
        return false;

    }
    else if (a_attribsRB->m_visualGeometryType == afGeometryType::INVALID && massNode.as<double>() > 0.0 && !inertiaNode.IsDefined()){
        std::cerr << "WARNING: Body "
                  << a_attribsRB->m_name
                  << "'s geometry is empty, mass > 0 and no intertia defined, hence ignoring\n";
        return false;
    }
    else if (a_attribsRB->m_visualGeometryType == afGeometryType::INVALID  && massNode.as<double>() > 0.0 && inertiaNode.IsDefined()){
        std::cerr << "INFO: Body "
                  << a_attribsRB->m_name
                  << "'s mesh field is empty but mass and interia defined\n";
    }

    if(scaleNode.IsDefined()){
        a_attribsRB->m_scale = scaleNode.as<double>();
    }

    if (a_attribsRB->m_visualGeometryType == afGeometryType::SINGLE_SHAPE){
        afPrimitiveShapeAttributes shapeAttribs;
        shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(visual_shape_str));
        ADFUtils::copyPrimitiveShapeData(&geometryNode, &shapeAttribs);
        shapeAttribs.setScale(a_attribsRB->m_scale);
        a_attribsRB->m_visualPrimitiveShapes.push_back(shapeAttribs);
    }

    else if (a_attribsRB->m_visualGeometryType == afGeometryType::COMPOUND_SHAPE){
        // First of all, set the inertial offset to 0.
        // Is this still necessary?
        inertialOffsetPosNode = rbNode["inertial offset undef"];
        for(uint shapeIdx = 0 ; shapeIdx < compoundShapeNode.size() ; shapeIdx++){
            visual_shape_str = compoundShapeNode[shapeIdx]["shape"].as<std::string>();
            geometryNode = compoundShapeNode[shapeIdx]["geometry"];
            YAML::Node shapeOffset = compoundShapeNode[shapeIdx]["offset"];

            afPrimitiveShapeAttributes shapeAttribs;
            shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(visual_shape_str));
            ADFUtils::copyPrimitiveShapeData(&geometryNode, &shapeAttribs);
            ADFUtils::copyShapeOffsetData(&shapeOffset, &shapeAttribs);
            shapeAttribs.setScale(a_attribsRB->m_scale);
            a_attribsRB->m_visualPrimitiveShapes.push_back(shapeAttribs);
        }
    }

    ADFUtils::getMatrialFromNode(&rbNode, &a_attribsRB->m_material);

    // Load any shader that have been defined
    if (shadersNode.IsDefined()){
        boost::filesystem::path shader_path = shadersNode["path"].as<std::string>();

        a_attribsRB->m_vtxShaderFilePath = shader_path / shadersNode["vertex"].as<std::string>();
        a_attribsRB->m_fragShaderFilePath = shader_path / shadersNode["fragment"].as<std::string>();

        a_attribsRB->m_shaderDefined = true;
    }

    // Load the inertial offset. If the body is a compound shape, the "inertial offset"
    // will be ignored, as per collision shape "offset" will be used.

    btTransform inertialOffsetTrans;
    btVector3 inertialOffsetPos;
    btQuaternion inertialOffsetRot;

    inertialOffsetPos.setValue(0,0,0);
    inertialOffsetRot.setEuler(0,0,0);

    if(inertialOffsetPosNode.IsDefined()){
        inertialOffsetPos = ADFUtils::toXYZ<btVector3>(&inertialOffsetPosNode);
        inertialOffsetPos = a_attribsRB->m_scale * inertialOffsetPos;
        if(inertialOffsetRotNode.IsDefined()){
            double r = inertialOffsetRotNode["r"].as<double>();
            double p = inertialOffsetRotNode["p"].as<double>();
            double y = inertialOffsetRotNode["y"].as<double>();
            inertialOffsetRot.setEulerZYX(y, p, r);
        }
    }

    inertialOffsetTrans.setOrigin(inertialOffsetPos);
    inertialOffsetTrans.setRotation(inertialOffsetRot);
    a_attribsRB->m_inertialOffset = inertialOffsetTrans;

    // Load Collision Margins
    if (collisionMarginNode.IsDefined()){
        a_attribsRB->m_collisionMargin = collisionMarginNode.as<double>();
    }

    if (namespaceNode.IsDefined()){
        a_attribsRB->m_namespace = namespaceNode.as<std::string>();
    }

    a_attribsRB->m_mass = massNode.as<double>();

    afCartesianControllerAttributes controllerAttribs;
    ADFUtils::getCartControllerAttribsFromNode(&controllerNode, &controllerAttribs);

    a_attribsRB->P_lin = controllerAttribs.P_lin;
    a_attribsRB->I_lin = controllerAttribs.I_lin;
    a_attribsRB->D_lin = controllerAttribs.D_lin;
    a_attribsRB->P_ang = controllerAttribs.P_ang;
    a_attribsRB->I_ang = controllerAttribs.I_ang;
    a_attribsRB->D_ang = controllerAttribs.D_ang;
    a_attribsRB->m_positionOutputType = controllerAttribs.m_positionOutputType;
    a_attribsRB->m_orientationOutputType = controllerAttribs.m_orientationOutputType;


   // Inertial origin in world
    cTransform T_iINw;
    T_iINw.identity();

    if(posNode.IsDefined()){
        T_iINw.setLocalPos(ADFUtils::toXYZ<cVector3d>(&posNode));
    }

    if(rotNode.IsDefined()){
        double r = rotNode["r"].as<double>();
        double p = rotNode["p"].as<double>();
        double y = rotNode["y"].as<double>();
        cMatrix3d rot;
        rot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        T_iINw.setLocalRot(rot);
    }

    // Mesh Origin in World
    cTransform T_mINw = T_iINw * afUtils::convertDataType<cTransform, btTransform>(a_attribsRB->m_inertialOffset);

    a_attribsRB->m_location = T_mINw;

    afSurfaceAttributes surfaceAttribs;

    if (linDampingNode.IsDefined()){
        surfaceAttribs.m_linear_damping = linDampingNode.as<double>();
    }
    if (angDampingNode.IsDefined()){
        surfaceAttribs.m_angular_damping = angDampingNode.as<double>();
    }
    if (staticFrictionNode.IsDefined()){
        surfaceAttribs.m_static_friction = staticFrictionNode.as<double>();
    }
    if (rollingFrictionNode.IsDefined()){
        surfaceAttribs.m_rolling_friction = rollingFrictionNode.as<double>();
    }
    if (restitutionNode.IsDefined()){
        surfaceAttribs.m_restitution = restitutionNode.as<double>();
    }

    a_attribsRB->m_surfaceAttribs = surfaceAttribs;

    if (publishChildrenNamesNode.IsDefined()){
        a_attribsRB->m_publishChildrenNames = publishChildrenNamesNode.as<bool>();
    }

    if (publishJointNamesNode.IsDefined()){
        a_attribsRB->m_publishJointNames = publishJointNamesNode.as<bool>();
    }

    if (publishJointPositionsNode.IsDefined()){
        a_attribsRB->m_publishJointPositions = publishJointPositionsNode.as<bool>();
    }

    if (publishFrequencyNode.IsDefined()){
        a_attribsRB->m_minPublishFreq = publishFrequencyNode["low"].as<uint>();
        a_attribsRB->m_maxPublishFreq = publishFrequencyNode["high"].as<uint>();
    }

    // The collision groups are sorted by integer indices. A group consists of a set of
    // afRigidBodies that collide with each other. The bodies in one group
    // should not collide with bodies from another group. Moreover,
    // a body can be a part of multiple groups to allow advanced collision behavior.

    if (collisionGroupsNode.IsDefined()){
        for (unsigned long gIdx = 0 ; gIdx < collisionGroupsNode.size() ; gIdx++){
            int gNum = collisionGroupsNode[gIdx].as<int>();
            // Sanity check for the group number
            if (gNum >= 0 && gNum <= 999){
                a_attribsRB->m_collisionGroups.push_back(gNum);
            }
            else{
                std::cerr << "WARNING: Body "
                          << a_attribsRB->m_name
                          << "'s group number is \"" << gNum << "\" which should be between [0 - 999], ignoring\n";
            }
        }
    }

    if (passiveNode.IsDefined()){
        a_attribsRB->m_passive = passiveNode.as<bool>();
    }


    return true;
}

bool ADFLoader_1_0::loadSoftBody(std::string sb_config_file, std::string node_name, afSoftBodyAttributes *attribs)
{
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(sb_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << sb_config_file << std::endl;
        return 0;
    }

    YAML::Node softBodyNode = baseNode[node_name];

    return loadSoftBody(&softBodyNode, attribs);
}

bool ADFLoader_1_0::loadSoftBody(YAML::Node *sb_node, afSoftBodyAttributes *attribs)
{
    YAML::Node& sbNode = *sb_node;
    if (sbNode.IsNull()){
        std::cerr << "ERROR: SOFT BODY'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = sbNode["name"];
    YAML::Node meshNode = sbNode["mesh"];
    YAML::Node collisionMarginNode = sbNode["collision margin"];
    YAML::Node scaleNode = sbNode["scale"];
    YAML::Node inertialOffsetPosNode = sbNode["inertial offset"]["position"];
    YAML::Node inertialOffsetRotNode = sbNode["inertial offset"]["orientation"];
    YAML::Node meshPathHRNode = sbNode["high resolution path"];
    YAML::Node meshPathLRNode = sbNode["low resolution path"];
    YAML::Node mameSpaceNode = sbNode["namespace"];
    YAML::Node massNode = sbNode["mass"];
    YAML::Node linGainNode = sbNode["linear gain"];
    YAML::Node angGainNode = sbNode["angular gain"];
    YAML::Node posNode = sbNode["location"]["position"];
    YAML::Node rotNode = sbNode["location"]["orientation"];
    YAML::Node colorNode = sbNode["color"];
    YAML::Node colorRGBANode = sbNode["color rgba"];
    YAML::Node colorComponentsNode = sbNode["color components"];
    YAML::Node configDataNode = sbNode["config"];
    YAML::Node randomizeConstraintsNode = sbNode["randomize constraints"];

    YAML::Node cfg_kLSTNode = configDataNode["kLST"];
    YAML::Node cfg_kASTNode = configDataNode["kAST"];
    YAML::Node cfg_kVSTNode = configDataNode["kVST"];
    YAML::Node cfg_kVCFNode = configDataNode["kVCF"];
    YAML::Node cfg_kDPNode = configDataNode["kDP"];
    YAML::Node cfg_kDGNode = configDataNode["kDG"];
    YAML::Node cfg_kLFNode = configDataNode["kLF"];
    YAML::Node cfg_kPRNode = configDataNode["kPR"];
    YAML::Node cfg_kVCNode = configDataNode["kVC"];
    YAML::Node cfg_kDFNode = configDataNode["kDF"];
    YAML::Node cfg_kMTNode = configDataNode["kMT"];
    YAML::Node cfg_kCHRNode = configDataNode["kCHR"];
    YAML::Node cfg_kKHRNode = configDataNode["kKHR"];
    YAML::Node cfg_kSHRNode = configDataNode["kSHR"];
    YAML::Node cfg_kAHRNode = configDataNode["kAHR"];
    YAML::Node cfg_kSRHR_CLNode = configDataNode["kSRHR_CL"];
    YAML::Node cfg_kSKHR_CLNode = configDataNode["kSKHR_CL"];
    YAML::Node cfg_kSSHR_CLNode = configDataNode["kSSHR_CL"];
    YAML::Node cfg_kSR_SPLT_CLNode = configDataNode["kSR_SPLT_CL"];
    YAML::Node cfg_kSK_SPLT_CLNode = configDataNode["kSK_SPLT_CL"];
    YAML::Node cfg_kSS_SPLT_CLNode = configDataNode["kSS_SPLT_CL"];
    YAML::Node cfg_maxvolumeNode = configDataNode["maxvolume"];
    YAML::Node cfg_timescaleNode = configDataNode["timescale"];
    YAML::Node cfg_viterationsNode = configDataNode["viterations"];
    YAML::Node cfg_piterationsNode = configDataNode["piterations"];
    YAML::Node cfg_diterationsNode = configDataNode["diterations"];
    YAML::Node cfg_citerationsNode = configDataNode["citerations"];
    YAML::Node cfg_flagsNode = configDataNode["flags"];
    YAML::Node cfg_bendingConstraintNode = configDataNode["bending constraint"];
    YAML::Node cfg_cuttingNode = configDataNode["cutting"];
    YAML::Node cfg_clustersNode = configDataNode["clusters"];
    YAML::Node cfg_fixed_nodesNode = configDataNode["fixed nodes"];

    if(nameNode.IsDefined()){
        m_name = nameNode.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    if(meshNode.IsDefined())
        m_mesh_name = meshNode.as<std::string>();

    if(scaleNode.IsDefined())
        m_scale = scaleNode.as<double>();

    if(inertialOffsetPosNode.IsDefined()){
        btTransform trans;
        btQuaternion quat;
        btVector3 pos;
        quat.setEulerZYX(0,0,0);
        pos.setValue(0,0,0);
        if(inertialOffsetRotNode.IsDefined()){
            double r = inertialOffsetRotNode["r"].as<double>();
            double p = inertialOffsetRotNode["p"].as<double>();
            double y = inertialOffsetRotNode["y"].as<double>();
            quat.setEulerZYX(y, p, r);
        }
        pos = toXYZ<btVector3>(&inertialOffsetPosNode);
        trans.setRotation(quat);
        trans.setOrigin(pos);
        setInertialOffsetTransform(trans);
    }

    boost::filesystem::path high_res_filepath;
    boost::filesystem::path low_res_filepath;
    if (meshPathHRNode.IsDefined()){
        high_res_filepath = meshPathHRNode.as<std::string>() + m_mesh_name;
        if (high_res_filepath.is_relative()){
            high_res_filepath =  mB->getMultiBodyPath() + '/' + high_res_filepath.c_str();
        }
    }
    else{
        high_res_filepath = mB->getHighResMeshesPath() + m_mesh_name;
    }
    if (meshPathLRNode.IsDefined()){
        low_res_filepath = meshPathLRNode.as<std::string>() + m_mesh_name;
        if (low_res_filepath.is_relative()){
            low_res_filepath = mB->getMultiBodyPath() + '/' + low_res_filepath.c_str();
        }
    }
    else{
        low_res_filepath = mB->getLowResMeshesPath() + m_mesh_name;
    }
    double _collision_margin = 0.1;
    if(collisionMarginNode.IsDefined()){
        _collision_margin = collisionMarginNode.as<double>();
    }

    if (loadFromFile(high_res_filepath.c_str())){
        scale(m_scale);
    }
    else{
        // If we can't find the visual mesh, we can proceed with
        // printing just a warning
        std::cerr << "WARNING: Soft Body " << m_name
                  << "'s mesh " << high_res_filepath << " not found\n";
    }

    if(m_lowResMesh.loadFromFile(low_res_filepath.c_str())){
        buildContactTriangles(_collision_margin, &m_lowResMesh);
        m_lowResMesh.scale(m_scale);
    }
    else{
        // If we can't find the collision mesh, then we have a problem,
        // stop loading this softbody and return with 0
        std::cerr << "WARNING: Soft Body " << m_name
                  << "'s mesh " << low_res_filepath << " not found\n";
        return 0;
    }

    if(mameSpaceNode.IsDefined()){
        m_namespace = afUtils::removeAdjacentBackSlashes(mameSpaceNode.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

    if(massNode.IsDefined()){
        m_mass = massNode.as<double>();
        if(linGainNode.IsDefined()){
            K_lin = linGainNode["P"].as<double>();
            D_lin = linGainNode["D"].as<double>();
            _lin_gains_computed = true;
        }
        if(angGainNode.IsDefined()){
            K_ang = angGainNode["P"].as<double>();
            D_ang = angGainNode["D"].as<double>();
            _ang_gains_computed = true;
        }
    }

    buildDynamicModel();

    if(posNode.IsDefined()){
        pos = toXYZ<cVector3d>(&posNode);
        setLocalPos(pos);
    }

    if(rotNode.IsDefined()){
        double r = rotNode["r"].as<double>();
        double p = rotNode["p"].as<double>();
        double y = rotNode["y"].as<double>();
        rot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(rot);
    }

    cMaterial _mat;
    float _r, _g, _b, _a;
    if(colorRGBANode.IsDefined()){
        _r = colorRGBANode["r"].as<float>();
        _g = colorRGBANode["g"].as<float>();
        _b = colorRGBANode["b"].as<float>();
        _a = colorRGBANode["a"].as<float>();
        _mat.setColorf(_r, _g, _b, _a);
        m_gelMesh.setMaterial(_mat);
        m_gelMesh.setTransparencyLevel(colorRGBANode["a"].as<float>());
    }
    else if(colorComponentsNode.IsDefined()){
        if (colorComponentsNode["diffuse"].IsDefined()){
            _r = colorComponentsNode["diffuse"]["r"].as<float>();
            _g = colorComponentsNode["diffuse"]["g"].as<float>();
            _b = colorComponentsNode["diffuse"]["b"].as<float>();
            _mat.m_diffuse.set(_r, _g, _b);
        }
        if (colorComponentsNode["ambient"].IsDefined()){
            float _level = colorComponentsNode["ambient"]["level"].as<float>();
            _r *= _level;
            _g *= _level;
            _b *= _level;
            _mat.m_ambient.set(_r, _g, _b);
        }
        if (colorComponentsNode["specular"].IsDefined()){
            _r = colorComponentsNode["specular"]["r"].as<float>();
            _g = colorComponentsNode["specular"]["g"].as<float>();
            _b = colorComponentsNode["specular"]["b"].as<float>();
            _mat.m_specular.set(_r, _g, _b);
        }
        if (colorComponentsNode["emission"].IsDefined()){
            _r = colorComponentsNode["emission"]["r"].as<float>();
            _g = colorComponentsNode["emission"]["g"].as<float>();
            _b = colorComponentsNode["emission"]["b"].as<float>();
            _mat.m_emission.set(_r, _g, _b);
        }

        _a = colorComponentsNode["transparency"].as<float>();
        _mat.setTransparencyLevel(_a);
        m_gelMesh.setMaterial(_mat);
        //        m_gelMesh.setTransparencyLevel(_a);
    }
    else if(colorNode.IsDefined()){
        std::vector<double> rgba = m_afWorld->getColorRGBA(colorNode.as<std::string>());
        _mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
        m_gelMesh.setMaterial(_mat);
        m_gelMesh.setTransparencyLevel(rgba[3]);
    }

    if (configDataNode.IsNull()){
        printf("Warning, no soft body config properties defined");
    }
    else{
        if (cfg_kLSTNode.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kLST = cfg_kLSTNode.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kLST = cfg_kLSTNode.as<double>();
        }
        if (cfg_kASTNode.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kAST = cfg_kASTNode.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kAST = cfg_kASTNode.as<double>();
        }
        if (cfg_kVSTNode.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kVST = cfg_kVSTNode.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kVST = cfg_kVSTNode.as<double>();
        }
        if (cfg_kVCFNode.IsDefined()) m_bulletSoftBody->m_cfg.kVCF = cfg_kVCFNode.as<double>();
        if (cfg_kDPNode.IsDefined()) m_bulletSoftBody->m_cfg.kDP = cfg_kDPNode.as<double>();
        if (cfg_kDGNode.IsDefined()) m_bulletSoftBody->m_cfg.kDG = cfg_kDGNode.as<double>();
        if (cfg_kLFNode.IsDefined()) m_bulletSoftBody->m_cfg.kLF = cfg_kLFNode.as<double>();
        if (cfg_kPRNode.IsDefined()) m_bulletSoftBody->m_cfg.kPR = cfg_kPRNode.as<double>();
        if (cfg_kVCNode.IsDefined()) m_bulletSoftBody->m_cfg.kVC = cfg_kVCNode.as<double>();
        if (cfg_kDFNode.IsDefined()) m_bulletSoftBody->m_cfg.kDF = cfg_kDFNode.as<double>();
        if (cfg_kMTNode.IsDefined()){
            m_bulletSoftBody->m_cfg.kMT = cfg_kMTNode.as<double>();
            m_bulletSoftBody->setPose(false, true);
        }
        if (cfg_kCHRNode.IsDefined()) m_bulletSoftBody->m_cfg.kCHR = cfg_kCHRNode.as<double>();
        if (cfg_kKHRNode.IsDefined()) m_bulletSoftBody->m_cfg.kKHR = cfg_kKHRNode.as<double>();
        if (cfg_kSHRNode.IsDefined()) m_bulletSoftBody->m_cfg.kSHR = cfg_kSHRNode.as<double>();
        if (cfg_kAHRNode.IsDefined()) m_bulletSoftBody->m_cfg.kAHR = cfg_kAHRNode.as<double>();
        if (cfg_kSRHR_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSRHR_CL = cfg_kSRHR_CLNode.as<double>();
        if (cfg_kSKHR_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSKHR_CL = cfg_kSKHR_CLNode.as<double>();
        if (cfg_kSSHR_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSSHR_CL = cfg_kSSHR_CLNode.as<double>();
        if (cfg_kSR_SPLT_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSR_SPLT_CL = cfg_kSR_SPLT_CLNode.as<double>();
        if (cfg_kSK_SPLT_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSK_SPLT_CL = cfg_kSK_SPLT_CLNode.as<double>();
        if (cfg_kSS_SPLT_CLNode.IsDefined()) m_bulletSoftBody->m_cfg.kSS_SPLT_CL = cfg_kSS_SPLT_CLNode.as<double>();
        if (cfg_maxvolumeNode.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_maxvolumeNode.as<double>();
        if (cfg_timescaleNode.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_timescaleNode.as<double>();
        if (cfg_viterationsNode.IsDefined()) m_bulletSoftBody->m_cfg.viterations = cfg_viterationsNode.as<int>();
        if (cfg_piterationsNode.IsDefined()) m_bulletSoftBody->m_cfg.piterations = cfg_piterationsNode.as<int>();
        if (cfg_diterationsNode.IsDefined()) m_bulletSoftBody->m_cfg.diterations = cfg_diterationsNode.as<int>();
        if (cfg_citerationsNode.IsDefined()) m_bulletSoftBody->m_cfg.citerations = cfg_citerationsNode.as<int>();
        if (cfg_flagsNode.IsDefined()){
            m_bulletSoftBody->m_cfg.collisions = cfg_flagsNode.as<int>();
        }
        if (cfg_bendingConstraintNode.IsDefined()){
            int _bending = cfg_bendingConstraintNode.as<int>();
            m_bulletSoftBody->generateBendingConstraints(_bending);
        }
        if (cfg_fixed_nodesNode.IsDefined()){
            for (uint i = 0 ; i < cfg_fixed_nodesNode.size() ; i++){
                int nodeIdx = cfg_fixed_nodesNode[i].as<int>();
                if (nodeIdx < m_bulletSoftBody->m_nodes.size()){
                    m_bulletSoftBody->setMass(nodeIdx, 0);
                }
            }
        }
        if(cfg_clustersNode.IsDefined()){
            int num_clusters = cfg_clustersNode.as<int>();
            m_bulletSoftBody->generateClusters(num_clusters);
        }
    }

    if (randomizeConstraintsNode.IsDefined())
        if (randomizeConstraintsNode.as<bool>() == true)
            m_bulletSoftBody->randomizeConstraints();

    m_afWorld->addChild(this);
    return true;
}

bool ADFLoader_1_0::loadJoint(std::string jnt_config_file, std::string node_name, ambf::afJointAttributes *attribs)
{

}

bool ADFLoader_1_0::loadJoint(YAML::Node *jnt_node, ambf::afJointAttributes *attribs)
{

}

bool ADFLoader_1_0::loadSensor(std::string sen_config_file, std::string node_name, ambf::afSensorAttributes *attribs)
{

}

bool ADFLoader_1_0::loadSensor(YAML::Node *sen_node, ambf::afSensorAttributes *attribs)
{

}

bool ADFLoader_1_0::loadActutator(std::string act_config_file, std::string node_name, ambf::afActuatorAttributes *attribs)
{

}

bool ADFLoader_1_0::loadActutator(YAML::Node *act_node, ambf::afActuatorAttributes *attribs)
{

}

bool ADFLoader_1_0::loadVehicle(std::string vh_config_file, std::string node_name, ambf::afVehicleAttributes *attribs)
{

}

bool ADFLoader_1_0::loadVehicle(YAML::Node *vh_node, ambf::afVehicleAttributes *attribs)
{

}

bool ADFLoader_1_0::loadMultiBody(std::string mb_config_file, ambf::afMultiBodyAttributes *attribs)
{

}

bool ADFLoader_1_0::loadWorld(std::string wd_config_file, ambf::afWorldAttributes *attribs)
{

}
