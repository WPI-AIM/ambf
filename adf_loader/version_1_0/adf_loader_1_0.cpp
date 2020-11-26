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
using namespace adf_loader_1_0;

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
bool ADFUtils::getMatrialFromNode(YAML::Node *a_node, cMaterial* m)
{
    YAML::Node& matNode = *a_node;

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


bool ADFUtils::getShaderAttribsFromNode(YAML::Node *a_node, afShaderAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node shadersNode = node["shaders"];

    bool valid = true;

    if (shadersNode.IsDefined()){
        boost::filesystem::path shader_path = shadersNode["path"].as<std::string>();

        attribs->m_vtxShaderFilePath = shader_path / shadersNode["vertex"].as<std::string>();
        attribs->m_fragShaderFilePath = shader_path / shadersNode["fragment"].as<std::string>();

        attribs->m_shaderDefined = true;
    }
    else{
        attribs->m_shaderDefined = false;
        valid = false;
    }

    return valid;
}

bool ADFUtils::getVisualAttribsFromNode(YAML::Node *a_node, afVisualAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node meshNode = node["mesh"];
    YAML::Node shapeNode = node["shape"];
    YAML::Node compoundShapeNode = node["compound shape"];
    YAML::Node geometryNode = node["geometry"];
    YAML::Node meshPathHRNode = node["high resolution path"];

    bool valid = false;

    std::string shape_str;

    if (shapeNode.IsDefined()){
        attribs->m_geometryType = afGeometryType::SINGLE_SHAPE;
        shape_str = shapeNode.as<std::string>();
        afPrimitiveShapeAttributes shapeAttribs;
        shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(shape_str));
        ADFUtils::copyPrimitiveShapeData(&geometryNode, &shapeAttribs);
        attribs->m_primitiveShapes.push_back(shapeAttribs);
    }
    else if (compoundShapeNode.IsDefined()){
        attribs->m_geometryType = afGeometryType::COMPOUND_SHAPE;
        for(uint shapeIdx = 0 ; shapeIdx < compoundShapeNode.size() ; shapeIdx++){
            shape_str = compoundShapeNode[shapeIdx]["shape"].as<std::string>();
            geometryNode = compoundShapeNode[shapeIdx]["geometry"];
            YAML::Node shapeOffset = compoundShapeNode[shapeIdx]["offset"];

            afPrimitiveShapeAttributes shapeAttribs;
            shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(shape_str));
            ADFUtils::copyPrimitiveShapeData(&geometryNode, &shapeAttribs);
            ADFUtils::copyShapeOffsetData(&shapeOffset, &shapeAttribs);
            attribs->m_primitiveShapes.push_back(shapeAttribs);
        }
    }
    else if(meshNode.IsDefined()){
        attribs->m_meshName = meshNode.as<std::string>();
        if (!attribs->m_meshName.empty()){
            // Each rigid body can have a seperate path for its low and high res meshes
            // Incase they are defined, we use these paths and if they are not, we use
            // the paths for the whole file
            if (meshPathHRNode.IsDefined()){
                boost::filesystem::path visualPath = meshPathHRNode.as<std::string>();
                attribs->m_meshFilePath = visualPath / attribs->m_meshName;
            }
            attribs->m_geometryType = afGeometryType::MESH;
        }
        else{
            valid = false;
        }
    }
    else{
        valid = false;
    }

    return valid;
}

bool ADFUtils::getSurfaceAttribsFromNode(YAML::Node *a_node, afSurfaceAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node linDampingNode = node["damping"]["linear"];
    YAML::Node angDampingNode = node["damping"]["angular"];
    YAML::Node staticFrictionNode = node["friction"]["static"];
    YAML::Node rollingFrictionNode = node["friction"]["rolling"];
    YAML::Node restitutionNode = node["restitution"];

    bool valid = true;

    if (linDampingNode.IsDefined()){
        attribs->m_linearDamping = linDampingNode.as<double>();
    }
    if (angDampingNode.IsDefined()){
        attribs->m_angularDamping = angDampingNode.as<double>();
    }
    if (staticFrictionNode.IsDefined()){
        attribs->m_staticFriction = staticFrictionNode.as<double>();
    }
    if (rollingFrictionNode.IsDefined()){
        attribs->m_rollingFriction = rollingFrictionNode.as<double>();
    }
    if (restitutionNode.IsDefined()){
        attribs->m_restitution = restitutionNode.as<double>();
    }

    return valid;
}


///
/// \brief afUtils::getCartControllerFromNode
/// \param a_node
/// \return
///
bool ADFUtils::getCartControllerAttribsFromNode(YAML::Node *a_node, afCartesianControllerAttributes* attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node controllerNode = node["controller"];

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

bool ADFUtils::getCollisionAttribsFromNode(YAML::Node *a_node, afCollisionAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node collisionMarginNode = node["collision margin"];
    YAML::Node collisionGroupsNode = node["collision groups"];

    YAML::Node collisionMeshNode = node["collision mesh"];
    YAML::Node collisionShapeNode = node["collision shape"];
    YAML::Node collisionOffsetNode = node["collision offset"];
    YAML::Node collisionGeometryNode = node["collision geometry"];
    YAML::Node compoundCollisionShapeNode = node["compound collision shape"];

    YAML::Node meshPathHRNode = node["high resolution path"];
    YAML::Node meshPathLRNode = node["low resolution path"];


    bool valid = true;
    std::string shape_str;

    if (collisionMarginNode.IsDefined()){
        attribs->m_margin = collisionMarginNode.as<double>();
    }

    if (collisionGroupsNode.IsDefined()){
        for (uint gIdx = 0 ; gIdx < collisionGroupsNode.size() ; gIdx++){
            int gNum = collisionGroupsNode[gIdx].as<int>();
            // Sanity check for the group number
            if (gNum >= 0 && gNum <= 999){
                attribs->m_groups.push_back(gNum);
            }
            else{
                std::cerr << "WARNING: Body Collision group number is \""
                          << gNum
                          << "\" but should be between [0 - 999], ignoring\n";
            }
        }
    }

    if(collisionShapeNode.IsDefined()){
        attribs->m_geometryType = afGeometryType::SINGLE_SHAPE;
        shape_str = collisionShapeNode.as<std::string>();
        afPrimitiveShapeAttributes shapeAttribs;
        shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(shape_str));
        ADFUtils::copyPrimitiveShapeData(&collisionGeometryNode, &shapeAttribs);
        ADFUtils::copyShapeOffsetData(&collisionOffsetNode, &shapeAttribs);
        attribs->m_primitiveShapes.push_back(shapeAttribs);
    }
    else if(compoundCollisionShapeNode.IsDefined()){
        attribs->m_geometryType = afGeometryType::COMPOUND_SHAPE;
        for (uint shapeIdx = 0 ; shapeIdx < compoundCollisionShapeNode.size() ; shapeIdx++){
            shape_str = compoundCollisionShapeNode[shapeIdx]["shape"].as<std::string>();
            YAML::Node _collisionGeometryNode = compoundCollisionShapeNode[shapeIdx]["geometry"];
            YAML::Node _shapeOffsetNode = compoundCollisionShapeNode[shapeIdx]["offset"];

            afPrimitiveShapeAttributes shapeAttribs;
            shapeAttribs.setShapeType(ADFUtils::getShapeTypeFromString(shape_str));
            ADFUtils::copyPrimitiveShapeData(&_collisionGeometryNode, &shapeAttribs);
            ADFUtils::copyShapeOffsetData(&_shapeOffsetNode, &shapeAttribs);
            attribs->m_primitiveShapes.push_back(shapeAttribs);
        }
    }
    else if (collisionMeshNode.IsDefined()){
        attribs->m_meshName = collisionMeshNode.as<std::string>();
        if (!attribs->m_meshName.empty()){
            if (meshPathLRNode.IsDefined()){
                boost::filesystem::path collPath = meshPathLRNode.as<std::string>();
                attribs->m_meshFilePath = collPath / attribs->m_meshName;
            }
            else if(meshPathHRNode.IsDefined()){
                boost::filesystem::path collPath = meshPathHRNode.as<std::string>();
                attribs->m_meshFilePath = collPath / attribs->m_meshName;
            }
            else{
                attribs->m_meshFilePath = "";
            }
            attribs->m_geometryType = afGeometryType::MESH;
        }
        else{
            valid = false;
        }
    }
    else{
        valid = false;
    }

    return valid;
}

bool ADFUtils::getCommunicationAttribsFromNode(YAML::Node *a_node, afCommunicationAttributes *attribs)
{
    YAML::Node& node = *a_node;
    YAML::Node publishFrequencyNode = node["publish frequency"];
    YAML::Node passiveNode = node["passive"];

    bool valid = true;

    if (publishFrequencyNode.IsDefined()){
        attribs->m_minPublishFreq = publishFrequencyNode["low"].as<uint>();
        attribs->m_maxPublishFreq = publishFrequencyNode["high"].as<uint>();
    }

    if (passiveNode.IsDefined()){
        attribs->m_passive = passiveNode.as<double>();
    }

    return valid;
}

bool ADFUtils::getHierarchyAttribsFromNode(YAML::Node *a_node, afHierarchyAttributes *attribs)
{
    YAML::Node &node = &a_node;

    YAML::Node parentNameNode = node["parent"];
    YAML::Node childNameNode = node["child"];

    bool valid = true;

    if (childNameNode.IsDefined()){
        attribs->m_childName = childNameNode.as<std::string>();
        valid = true;
    }
    else{
        valid = false;
    }

    if (!parentNameNode.IsDefined()){
        attribs->m_parentName = parentNameNode.as<std::string>();
        valid = true;
    }
    else{
        valid = false;
    }

    return valid;
}

bool ADFUtils::getIdentificationAttribsFromNode(YAML::Node *a_node, afIdentificationAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];

    bool valid;

    if(nameNode.IsDefined()){
        std::string name = nameNode.as<std::string>();
        name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
        attribs->m_name = name;
        valid = true;
    }

    if (namespaceNode.IsDefined()){
        attribs->m_namespace = namespaceNode.as<std::string>();
        valid = true;
    }

    return valid;
}

bool ADFUtils::getInertialAttrisFromNode(YAML::Node *a_node, afInertialAttributes *attribs)
{
    YAML::Node &node = *a_node;

    YAML::Node massNode = node["mass"];
    YAML::Node inertiaNode = node["inertia"];
    YAML::Node inertialOffset = node["inertial offset"];

    bool valid = true;

    if(!massNode.IsDefined()){
        std::cerr << "WARNING: Body's mass is not defined, ignoring!\n";
        return false;
    }
    else{
        attribs->m_mass = massNode.as<double>();
    }

    if(inertiaNode.IsDefined()){
        double ix = inertiaNode["ix"].as<double>();
        double iy = inertiaNode["ix"].as<double>();
        double iz = inertiaNode["ix"].as<double>();

        attribs->m_inertia.setValue(ix, iy, iz);
        attribs->m_estimateInertia = false;
    }
    else{
        attribs->m_estimateInertia = true;
    }

    if(inertialOffset.IsDefined()){
        YAML::Node _inertialOffsetPos = inertialOffset["position"];
        YAML::Node _inertialOffsetRot = inertialOffset["orientation"];

        if (_inertialOffsetPos.IsDefined()){
            btVector3 iP = ADFUtils::toXYZ<btVector3>(&_inertialOffsetPos);
            attribs->m_inertialOffset.setOrigin(iP);
        }

        if (_inertialOffsetRot.IsDefined()){
            double r = _inertialOffsetRot["r"].as<double>();
            double p = _inertialOffsetRot["p"].as<double>();
            double y = _inertialOffsetRot["y"].as<double>();
            btMatrix3x3 iR;
            iR.setEulerZYX(y, p, r);
            attribs->m_inertialOffset.setBasis(iR);
        }
    }

    return valid;
}

bool ADFUtils::getJointControllerAttribsFromNode(YAML::Node *a_node, afJointControllerAttributes *attribs)
{

}

bool ADFUtils::getKinematicAttribsFromNode(YAML::Node *a_node, afKinematicAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node scaleNode = node["scale"];

    bool valid = true;

    if(scaleNode.IsDefined()){
        attribs->m_scale = scaleNode.as<double>();
    }

    if(posNode.IsDefined()){
        cVector3d pos = ADFUtils::toXYZ<cVector3d>(&posNode);
        attribs->m_location.setLocalPos(pos);
    }
    else{
        valid = false;
    }

    if(rotNode.IsDefined()){
        double r = rotNode["r"].as<double>();
        double p = rotNode["p"].as<double>();
        double y = rotNode["y"].as<double>();
        cMatrix3d rot;
        rot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        attribs->m_location.setLocalRot(rot);
    }
    else{
        valid = false;
    }

    return valid;
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

bool ADFLoader_1_0::loadRigidBody(YAML::Node *a_node, ambf::afRigidBodyAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: RIGID BODY " << node << " NODE IS NULL\n";
        return false;
    }

    if (attribs == nullptr){
        std::cerr << "ERROR: RIGID BODY ATTRIBUTES IS A NULLPTR\n";
        return false;
    }
    // Declare all the yaml parameters that we want to look for
    // IDENTIFICATION
    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    // KINEMATICS
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node scaleNode = node["scale"];
    // INERTIAL
    YAML::Node massNode = node["mass"];
    YAML::Node inertiaNode = node["inertia"];
    YAML::Node inertialOffsetPosNode = node["inertial offset"]["position"];
    YAML::Node inertialOffsetRotNode = node["inertial offset"]["orientation"];
    // VISUAL
    YAML::Node meshPathHRNode = node["high resolution path"];
    YAML::Node meshPathLRNode = node["low resolution path"];
    YAML::Node meshNode = node["mesh"];
    YAML::Node shapeNode = node["shape"];
    YAML::Node compoundShapeNode = node["compound shape"];
    YAML::Node geometryNode = node["geometry"];
    YAML::Node shadersNode = node["shaders"];
    // COLLISION
    YAML::Node collisionMarginNode = node["collision margin"];
    YAML::Node collisionMeshNode = node["collision mesh"];
    YAML::Node collisionShapeNode = node["collision shape"];
    YAML::Node collisionOffsetNode = node["collision offset"];
    YAML::Node collisionGeometryNode = node["collision geometry"];
    YAML::Node compoundCollisionShapeNode = node["compound collision shape"];
    YAML::Node collisionGroupsNode = node["collision groups"];
    // CONTROLLERS
    YAML::Node controllerNode = node["controller"];
    // SURFACE PROPS
    YAML::Node linDampingNode = node["damping"]["linear"];
    YAML::Node angDampingNode = node["damping"]["angular"];
    YAML::Node staticFrictionNode = node["friction"]["static"];
    YAML::Node rollingFrictionNode = node["friction"]["rolling"];
    YAML::Node restitutionNode = node["restitution"];
    // COMMUNICATION
    YAML::Node publishFrequencyNode = node["publish frequency"];
    YAML::Node passiveNode = node["passive"];
    // Rigid Body Specific
    YAML::Node publishChildrenNamesNode = node["publish children names"];
    YAML::Node publishJointNamesNode = node["publish joint names"];
    YAML::Node publishJointPositionsNode = node["publish joint positions"];

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getVisualAttribsFromNode(&node, &attribs->m_visualAttribs);
    adfUtils.getCollisionAttribsFromNode(&node, &attribs->m_collisionAttribs);
    adfUtils.getInertialAttrisFromNode(&node, &attribs->m_inertialAttribs);
    adfUtils.getCartControllerAttribsFromNode(&node, &attribs->m_controllerAttribs);
    adfUtils.getSurfaceAttribsFromNode(&node, &attribs->m_surfaceAttribs);
    adfUtils.getCommunicationAttribsFromNode(&node, &attribs->m_communicationAttribs);

    if (publishChildrenNamesNode.IsDefined()){
        attribs->m_publishChildrenNames = publishChildrenNamesNode.as<bool>();
    }

    if (publishJointNamesNode.IsDefined()){
        attribs->m_publishJointNames = publishJointNamesNode.as<bool>();
    }

    if (publishJointPositionsNode.IsDefined()){
        attribs->m_publishJointPositions = publishJointPositionsNode.as<bool>();
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

bool ADFLoader_1_0::loadSoftBody(YAML::Node *a_node, afSoftBodyAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: SOFT BODY'S YAML NODE IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = node["name"];
    YAML::Node meshNode = node["mesh"];
    YAML::Node collisionMarginNode = node["collision margin"];
    YAML::Node scaleNode = node["scale"];
    YAML::Node inertialOffsetPosNode = node["inertial offset"]["position"];
    YAML::Node inertialOffsetRotNode = node["inertial offset"]["orientation"];
    YAML::Node meshPathHRNode = node["high resolution path"];
    YAML::Node meshPathLRNode = node["low resolution path"];
    YAML::Node mameSpaceNode = node["namespace"];
    YAML::Node massNode = node["mass"];
    YAML::Node linGainNode = node["linear gain"];
    YAML::Node angGainNode = node["angular gain"];
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node colorNode = node["color"];
    YAML::Node colorRGBANode = node["color rgba"];
    YAML::Node colorComponentsNode = node["color components"];

    YAML::Node configDataNode = node["config"];

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

    YAML::Node randomizeConstraintsNode = node["randomize constraints"];

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getVisualAttribsFromNode(&node, &attribs->m_visualAttribs);
    adfUtils.getCollisionAttribsFromNode(&node, &attribs->m_collisionAttribs);
    adfUtils.getInertialAttrisFromNode(&node, &attribs->m_inertialAttribs);
    adfUtils.getCartControllerAttribsFromNode(&node, &attribs->m_controllerAttribs);
    adfUtils.getCommunicationAttribsFromNode(&node, &attribs->m_communicationAttribs);


    if (configDataNode.IsNull()){
        printf("Warning, no soft body config properties defined");
    }
    else{
        if (cfg_kLSTNode.IsDefined()){
            attribs->m_kLST = cfg_kLSTNode.as<float>();
        }
        if (cfg_kASTNode.IsDefined()){
            attribs->m_kAST = cfg_kASTNode.as<float>();
        }
        if (cfg_kVSTNode.IsDefined()){
            attribs->m_kVST = cfg_kVSTNode.as<float>();
        }
        if (cfg_kVCFNode.IsDefined()){
            attribs->m_kVCF = cfg_kVCFNode.as<float>();
        }
        if (cfg_kDPNode.IsDefined()){
            attribs->m_kDP = cfg_kDPNode.as<float>();
        }
        if (cfg_kDGNode.IsDefined()){
            attribs->m_kDG = cfg_kDGNode.as<float>();
        }
        if (cfg_kLFNode.IsDefined()){
            attribs->m_kLF = cfg_kLFNode.as<float>();
        }
        if (cfg_kPRNode.IsDefined()) {
            attribs->m_kPR = cfg_kPRNode.as<float>();
        }
        if (cfg_kVCNode.IsDefined()) {
            attribs->m_kVC = cfg_kVCNode.as<float>();
        }
        if (cfg_kDFNode.IsDefined()) {
            attribs->m_kDF = cfg_kDFNode.as<float>();
        }
        if (cfg_kMTNode.IsDefined()){
           attribs->m_kMT = cfg_kMTNode.as<float>();
        }
        if (cfg_kCHRNode.IsDefined()) {
            attribs->m_kCHR = cfg_kCHRNode.as<float>();
        }
        if (cfg_kKHRNode.IsDefined()) {
           attribs->m_kKHR = cfg_kKHRNode.as<float>();
        }
        if (cfg_kSHRNode.IsDefined()) {
           attribs->m_kSHR = cfg_kSHRNode.as<float>();
        }
        if (cfg_kAHRNode.IsDefined()) {
           attribs->m_kAHR = cfg_kAHRNode.as<float>();
        }
        if (cfg_kSRHR_CLNode.IsDefined()) {
            attribs->m_kSRHR_CL = cfg_kSRHR_CLNode.as<float>();
        }
        if (cfg_kSKHR_CLNode.IsDefined()) {
            attribs->m_kSKHR_CL = cfg_kSKHR_CLNode.as<float>();
        }
        if (cfg_kSSHR_CLNode.IsDefined()) {
            attribs->m_kSSHR_CL = cfg_kSSHR_CLNode.as<float>();
        }
        if (cfg_kSR_SPLT_CLNode.IsDefined()) {
           attribs->m_kSR_SPLT_CL = cfg_kSR_SPLT_CLNode.as<float>();
        }
        if (cfg_kSK_SPLT_CLNode.IsDefined()) {
            attribs->m_kSK_SPLT_CL = cfg_kSK_SPLT_CLNode.as<float>();
        }
        if (cfg_kSS_SPLT_CLNode.IsDefined()) {
            attribs->m_kSS_SPLT_CL = cfg_kSS_SPLT_CLNode.as<float>();
        }
        if (cfg_maxvolumeNode.IsDefined()) {
            attribs->m_maxVolume = cfg_maxvolumeNode.as<float>();
        }
        if (cfg_timescaleNode.IsDefined()) {
            attribs->m_timeScale = cfg_timescaleNode.as<float>();
        }
        if (cfg_viterationsNode.IsDefined()) {
           attribs->m_vIterations = cfg_viterationsNode.as<int>();
        }
        if (cfg_piterationsNode.IsDefined()) {
           attribs->m_pIterations = cfg_piterationsNode.as<int>();
        }
        if (cfg_diterationsNode.IsDefined()) {
           attribs->m_dIterations = cfg_diterationsNode.as<int>();
        }
        if (cfg_citerationsNode.IsDefined()) {
            attribs->m_cIterations = cfg_citerationsNode.as<int>();
        }
        if (cfg_flagsNode.IsDefined()){
            attribs->m_flags = cfg_flagsNode.as<int>();
        }
        if (cfg_bendingConstraintNode.IsDefined()){
            attribs->m_bendingConstraint = cfg_bendingConstraintNode.as<int>();
        }
        if (cfg_fixed_nodesNode.IsDefined()){
            for (uint i = 0 ; i < cfg_fixed_nodesNode.size() ; i++){
                attribs->m_fixedNodes.push_back(i);
            }
        }
        if(cfg_clustersNode.IsDefined()){
            attribs->m_clusters = cfg_clustersNode.as<int>();
        }
    }

    if (randomizeConstraintsNode.IsDefined()){
        attribs->m_randomizeConstraints = randomizeConstraintsNode.as<bool>();
    }

    return true;
}

bool ADFLoader_1_0::loadJoint(std::string jnt_config_file, std::string node_name, ambf::afJointAttributes *attribs)
{
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(jnt_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO JOINT CONFIG: " << jnt_config_file << std::endl;
        return 0;
    }
    if (baseNode.IsNull()) return false;

    YAML::Node baseJointNode = baseNode[node_name];
    return loadJoint(&baseJointNode, attribs);

}

bool ADFLoader_1_0::loadJoint(YAML::Node *a_node, ambf::afJointAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: JOINT'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node parentNameNode = node["parent"];
    YAML::Node childNameNode = node["child"];
    YAML::Node nameNode = node["name"];
    YAML::Node parentPivotNode = node["parent pivot"];
    YAML::Node childPivotNode = node["child pivot"];
    YAML::Node parentAxisNode = node["parent axis"];
    YAML::Node childAxisNode = node["child axis"];
    YAML::Node originNode = node["origin"];
    YAML::Node axisNode = node["axis"];
    YAML::Node enableMotorNode = node["enable motor"];
    YAML::Node enableFeedbackNode = node["enable feedback"];
    YAML::Node maxMotorImpulseNode = node["max motor impulse"];
    YAML::Node limitsNode = node["joint limits"];
    YAML::Node erpNode = node["joint erp"];
    YAML::Node cfmNode = node["joint cfm"];
    YAML::Node offsetNode = node["offset"];
    YAML::Node dampingNode = node["damping"];
    YAML::Node stiffnessNode = node["stiffness"];
    YAML::Node typeNode = node["type"];
    YAML::Node controllerNode = node["controller"];
    YAML::Node ignoreInterCollisionNode = node["ignore inter-collision"];
    YAML::Node equilibriumPointNode = node["equilibrium point"];
    YAML::Node passiveNode = node["passive"];

    ADFUtils adfUtils;

    adfUtils.getHierarchyAttribsFromNode(&node, &attribs->m_hierarchyAttribs);
    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);

    attribs->m_parentPivot = ADFUtils::toXYZ<cVector3d>(&parentPivotNode);
    attribs->m_childPivot = ADFUtils::toXYZ<cVector3d>(&childPivotNode);
    attribs->m_parentAxis = ADFUtils::toXYZ<cVector3d>(&parentAxisNode);
    attribs->m_childAxis = ADFUtils::toXYZ<cVector3d>(&childAxisNode);
    attribs->m_parentPivot = ADFUtils::toXYZ<cVector3d>(&parentPivotNode);

    // Joint Transform in Parent
    btTransform T_j_p;
    // Joint Axis
    btVector3 joint_axis(0,0,1);
    m_enableActuator = true;
    m_controller.max_impulse = 10; // max rate of change of effort on Position Controllers
    m_jointOffset = 0.0;
    m_lowerLimit = -100;
    m_upperLimit = 100;
    //Default joint type is revolute if not type is specified
    m_jointType = afJointType::REVOLUTE;
    m_jointDamping = 0.0; // Initialize damping to 0

    bool _ignore_inter_collision = true;

    m_pvtA = toXYZ<btVector3>( &jointParentPivot);
    m_axisA = toXYZ<btVector3>( &jointParentAxis);
    m_pvtB = toXYZ<btVector3>( &jointChildPivot);
    m_axisB = toXYZ<btVector3>( &jointChildAxis);

    // Scale the pivot before transforming as the default scale methods don't move this pivot
    m_pvtA *= m_afParentBody->m_scale;
    m_pvtA = m_afParentBody->getInertialOffsetTransform().inverse() * m_pvtA;
    m_pvtB = m_afChildBody->getInertialOffsetTransform().inverse() * m_pvtB;
    m_axisA = m_afParentBody->getInertialOffsetTransform().getBasis().inverse() * m_axisA;
    m_axisB = m_afChildBody->getInertialOffsetTransform().getBasis().inverse() * m_axisB;

    if(jointOffset.IsDefined()){
        m_jointOffset = jointOffset.as<double>();
    }

    if (jointDamping.IsDefined()){
        m_jointDamping = jointDamping.as<double>();
    }

    if(jointLimits.IsDefined()){
        if (jointLimits["low"].IsDefined())
            m_lowerLimit = jointLimits["low"].as<double>();
        if (jointLimits["high"].IsDefined())
            m_upperLimit = jointLimits["high"].as<double>();
    }

    if (jointController.IsDefined()){
        if( (jointController["P"]).IsDefined())
            m_controller.P = jointController["P"].as<double>();
        if( (jointController["I"]).IsDefined())
            m_controller.I = jointController["I"].as<double>();
        if( (jointController["D"]).IsDefined())
            m_controller.D = jointController["D"].as<double>();

        // If the PID controller in defined, the gains will be used to command the joint force (effort)
        m_controller.m_outputType = afControlType::FORCE;
    }
    else{
        // If the controller gains are not defined, a velocity based control will be used.
        // The tracking velocity can be controller by setting "max motor impulse" field
        // for the joint data-block in the ADF file.
        m_controller.P = 10;
        m_controller.I = 0;
        m_controller.D = 0;
        m_controller.m_outputType = afControlType::VELOCITY;
    }

    // Bullet takes the x axis as the default for prismatic joints
    btVector3 ax_cINp;

    if (jointType.IsDefined()){
        if ((strcmp(jointType.as<std::string>().c_str(), "hinge") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "revolute") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "continuous") == 0)){
            m_jointType = afJointType::REVOLUTE;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "slider") == 0)
                 || (strcmp(jointType.as<std::string>().c_str(), "prismatic") == 0)){
            m_jointType = afJointType::PRISMATIC;
            // For this case constraint axis is the world x axis
            ax_cINp.setValue(1, 0, 0);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "fixed") == 0)){
            m_jointType = afJointType::FIXED;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "spring") == 0)){
            m_jointType = afJointType::LINEAR_SPRING;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "linear spring") == 0)){
            m_jointType = afJointType::LINEAR_SPRING;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "torsion spring") == 0)){
            m_jointType = afJointType::TORSION_SPRING;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "torsional spring") == 0)){
            m_jointType = afJointType::TORSION_SPRING;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "angular spring") == 0)){
            m_jointType = afJointType::TORSION_SPRING;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "p2p") == 0)){
            m_jointType = afJointType::P2P;
            // For this case the constraint axis doesnt matter
            ax_cINp.setValue(0, 0, 1);
        }

    }

    double _jointERP, _jointCFM;

    if(jointERP.IsDefined()){
        _jointERP = jointERP.as<double>();
    }
    else{
        _jointERP = mB->m_jointERP;
    }

    if(jointCFM.IsDefined()){
        _jointCFM = jointCFM.as<double>();
    }
    else{
        _jointCFM = mB->m_jointCFM;
    }

    if (jointIgnoreInterCollision.IsDefined()){
        _ignore_inter_collision = jointIgnoreInterCollision.as<bool>();
    }

    if (jointPassive.IsDefined()){
        m_passive = jointPassive.as<bool>();
    }

    // Compute frameA and frameB from constraint axis data. This step is common
    // for all joints, the only thing that changes in the constraint axis which can be
    // set the appropriate joint type

    btTransform frameA, frameB;
    frameA.setIdentity();
    frameB.setIdentity();

    // Rotation of constraint in parent axis as quaternion
    btQuaternion Q_conINp;
    Q_conINp = afUtils::getRotBetweenVectors<btQuaternion, btVector3>(ax_cINp, m_axisA);
    frameA.setRotation(Q_conINp);
    frameA.setOrigin(m_pvtA);

    // Rotation of child axis in parent axis as Quaternion
    btQuaternion Q_cINp;
    Q_cINp = afUtils::getRotBetweenVectors<btQuaternion, btVector3>(m_axisB, m_axisA);

    // Offset rotation along the parent axis
    btQuaternion Q_offINp;
    Q_offINp.setRotation(m_axisA, m_jointOffset);
    // We need to post-multiply frameA's rot to cancel out the shift in axis, then
    // the offset along joint axis and finally frameB's axis alignment in frameA.
    frameB.setRotation( Q_cINp.inverse() * Q_offINp.inverse() * Q_conINp);
    frameB.setOrigin(m_pvtB);

    // If the joint is revolute, hinge or continous
    if (m_jointType == afJointType::REVOLUTE){
#ifdef USE_PIVOT_AXIS_METHOD
        m_btConstraint = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
#else
        m_hinge = new btHingeConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_hinge->setParam(BT_CONSTRAINT_ERP, _jointERP);
        m_hinge->setParam(BT_CONSTRAINT_CFM, _jointCFM);
#endif
        // Don't enable motor yet, only enable when set position is called
        // this keeps the joint behave freely when it's launched
        if(jointMaxMotorImpulse.IsDefined()){
            double max_impulse = jointMaxMotorImpulse.as<double>();
            m_hinge->enableAngularMotor(false, 0.0, max_impulse);
        }
        else{
            m_hinge->enableAngularMotor(false, 0.0, 0.1);
        }

        if(jointLimits.IsDefined()){
            m_hinge->setLimit(m_lowerLimit, m_upperLimit);
        }

        m_btConstraint = m_hinge;
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        m_afParentBody->addChildJointPair(m_afChildBody, this);
    }
    // If the joint is slider, prismatic or linear
    else if (m_jointType == afJointType::PRISMATIC){
        m_slider = new btSliderConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);
        m_slider->setParam(BT_CONSTRAINT_ERP, _jointERP);
        m_slider->setParam(BT_CONSTRAINT_CFM, _jointCFM);

        if (jointEnableMotor.IsDefined()){
            m_enableActuator = jointEnableMotor.as<int>();
            // Don't enable motor yet, only enable when set position is called
            if(jointMaxMotorImpulse.IsDefined()){
                m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            }
        }

        if(jointLimits.IsDefined()){
            m_slider->setLowerLinLimit(m_lowerLimit);
            m_slider->setUpperLinLimit(m_upperLimit);
        }

        if(jointMaxMotorImpulse.IsDefined()){
            m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            // Ugly hack, divide by (default) fixed timestep to max linear motor force
            // since m_slider does have a max impulse setting method.
            m_slider->setMaxLinMotorForce(m_controller.max_impulse / 0.001);
        }
        else{
            // Default to 1000.0
            m_slider->setMaxLinMotorForce(1000);
            m_slider->setPoweredLinMotor(false);
        }

        m_btConstraint = m_slider;
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        m_afParentBody->addChildJointPair(m_afChildBody, this);
    }

    // If the joint is a spring
    else if (m_jointType == afJointType::LINEAR_SPRING || m_jointType == afJointType::TORSION_SPRING){
        m_spring = new btGeneric6DofSpringConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB, true);

        // Initialize all the 6 axes to 0 stiffness and damping
        // and limits also set to 0-0
        for (int axIdx = 0 ; axIdx < 6 ; axIdx++){
            m_spring->setLimit(axIdx, 0.0, 0.0);
            m_spring->setStiffness(axIdx, 0.0);
            m_spring->setDamping(axIdx, 0.0);
            m_spring->enableSpring(axIdx, false);
        }

        // We treat springs along the z axes of constraint, thus chosed
        // the appropriate axis number based on if the spring is linear
        // or torsional [0-2] -> linear, [3-5] -> rotational
        int _axisNumber = -1;

        if (m_jointType == afJointType::LINEAR_SPRING){
            _axisNumber = 2;
        }
        else if (m_jointType == afJointType::TORSION_SPRING){
            _axisNumber = 5;
        }

        double _low, _high;
        if (jointLimits.IsDefined()){

            _high =  jointLimits["high"].as<double>();
            _low = jointLimits["low"].as<double>();

            // Somehow bullets springs limits for rotational joints are inverted.
            // So handle them internally rather than breaking AMBF description specificaiton
            if (m_jointType == afJointType::TORSION_SPRING){
                double _temp = _low;
                _low = - _high;
                _high = - _temp;

            }

            btVector3 _limLow, _limHigh;
            _limLow.setValue(0, 0, 0);
            _limHigh.setValue(0, 0, 0);
            _limLow.setZ(_low);
            _limHigh.setZ(_low);

            m_spring->setLimit(_axisNumber, _low, _high);
            m_spring->enableSpring(_axisNumber, true);
        }

        if (node["equiblirium point"].IsDefined()){
            double _equiblirium = node["equiblirium point"].as<double>();
            // The equiblirium offset if also inverted for torsional springs
            // Fix it internally rather than breaking AMBF description specificaiton
            if (m_jointType == afJointType::TORSION_SPRING){
                _equiblirium = - _equiblirium;
            }
            m_spring->setEquilibriumPoint(_axisNumber, _equiblirium);
        }
        else{
            m_spring->setEquilibriumPoint(_axisNumber, _low + ((_high - _low) / 2));
        }

        // Calculcated a stiffness value based on the masses of connected bodies.
        double _stiffness = 10 * m_afParentBody->getMass() + m_afChildBody->getMass();
        // If stiffness defined, override the above value
        if (jointStiffness.IsDefined()){
            _stiffness = jointStiffness.as<double>();
        }
        m_spring->setStiffness(_axisNumber, _stiffness);

        m_spring->setDamping(_axisNumber, m_jointDamping);

        m_spring->setParam(BT_CONSTRAINT_STOP_ERP, _jointERP, _axisNumber);
        m_spring->setParam(BT_CONSTRAINT_CFM, _jointCFM, _axisNumber);

        m_btConstraint = m_spring;
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);

        m_afParentBody->addChildJointPair(m_afChildBody, this);
    }
    else if (m_jointType == afJointType::P2P){
        // p2p joint doesnt concern itself with rotations, its set using just the pivot information
        m_p2p = new btPoint2PointConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, m_pvtA, m_pvtB);
        m_p2p->setParam(BT_CONSTRAINT_ERP, _jointERP);
        m_p2p->setParam(BT_CONSTRAINT_CFM, _jointCFM);

        if (jointEnableMotor.IsDefined()){
            m_enableActuator = jointEnableMotor.as<int>();
            // Don't enable motor yet, only enable when set position is called
            if(jointMaxMotorImpulse.IsDefined()){
                m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            }
        }

        m_btConstraint = m_p2p;
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        m_afParentBody->addChildJointPair(m_afChildBody, this);
    }
    else if (m_jointType == afJointType::FIXED){
        m_btConstraint = new btFixedConstraint(*m_afParentBody->m_bulletRigidBody, *m_afChildBody->m_bulletRigidBody, frameA, frameB);
        //        ((btFixedConstraint *) m_btConstraint)->setParam(BT_CONSTRAINT_ERP, _jointERP);
        //        ((btFixedConstraint *) m_btConstraint)->setParam(BT_CONSTRAINT_CFM, _jointCFM);
        m_afWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        m_afParentBody->addChildJointPair(m_afChildBody, this);
    }

    if (jointEnableFeedback.IsDefined()){
        if (m_btConstraint != nullptr){
            m_feedbackEnabled = jointEnableFeedback.as<bool>();
            if (m_feedbackEnabled){
                m_btConstraint->enableFeedback(m_feedbackEnabled);
                m_feedback = new btJointFeedback();
                m_btConstraint->setJointFeedback(m_feedback);
            }
        }
    }
    return true;

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
