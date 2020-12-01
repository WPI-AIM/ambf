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
    attribs->m_geometryType = afGeometryType::INVALID;

    // Each rigid body can have a seperate path for its low and high res meshes
    // Incase they are defined, we use these paths and if they are not, we use
    // the paths for the whole file
    if (meshPathHRNode.IsDefined()){
        attribs->m_meshPath = meshPathHRNode.as<std::string>();
    }

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

bool ADFUtils::getWheelAttribsFromNode(YAML::Node *a_node, afWheelAttributes *attribs)
{
    YAML::Node& node = *a_node;

    YAML::Node rigidBodyNameNode = node["body"];
    YAML::Node meshNameNode = node["mesh"];
    YAML::Node widthNode = node["width"];
    YAML::Node radiusNode = node["radius"];
    YAML::Node frictionNode = node["friction"];
    YAML::Node suspensionNode = node["suspension"];
    YAML::Node rollInfluenceNode = node["roll influence"];
    YAML::Node downDirNode = node["down direction"];
    YAML::Node axelDirNode = node["axel direction"];
    YAML::Node offsetNode = node["offset"];
    YAML::Node frontNode = node["front"];
    YAML::Node steeringLimitsNode = node["steering limits"];
    YAML::Node maxEnginePowerNode = node["max engine power"];
    YAML::Node maxBrakePowerNode = node["max brake power"];

    bool valid = true;

    attribs->m_representationType = afWheelRepresentationType::INVALID;

    if (rigidBodyNameNode.IsDefined()){
        attribs->m_wheelBodyName = rigidBodyNameNode.as<std::string>();
        attribs->m_representationType = afWheelRepresentationType::RIGID_BODY;
    }
    else if (meshNameNode.IsDefined()){
        attribs->m_visualAttribs.m_meshName = meshNameNode.as<std::string>();
        attribs->m_representationType = afWheelRepresentationType::MESH;
    }
    else{
        std::cerr << "ERROR! UNABLE TO FIND \"MESH\" OR \"BODY\" FIELD FOR WHEEL OF VEHICLE. SKIPPING WHEEL!" << std::endl;
        return false;
    }


    if (widthNode.IsDefined()){
        attribs->m_width = widthNode.as<double>();
    }

    if (radiusNode.IsDefined()){
        attribs->m_radius = radiusNode.as<double>();
    }

    if (frictionNode.IsDefined()){
        attribs->m_friction = frictionNode.as<double>();
    }

    if (suspensionNode.IsDefined()){
        attribs->m_suspensionAttribs.m_stiffness = suspensionNode["stiffness"].as<double>();
        attribs->m_suspensionAttribs.m_damping = suspensionNode["damping"].as<double>();
        attribs->m_suspensionAttribs.m_compression = suspensionNode["compression"].as<double>();
        attribs->m_suspensionAttribs.m_restLength = suspensionNode["rest length"].as<double>();
    }

    if (rollInfluenceNode.IsDefined()){
        attribs->m_rollInfluence = rollInfluenceNode.as<double>();
    }

    if (downDirNode.IsDefined()){
        attribs->m_downDirection = toXYZ<cVector3d>(&downDirNode);
    }

    if (axelDirNode.IsDefined()){
       attribs->m_axelDirection = toXYZ<cVector3d>(&axelDirNode);
    }

    if (offsetNode.IsDefined()){
        attribs->m_offset = toXYZ<cVector3d>(&offsetNode);
    }

    if (frontNode.IsDefined()){
        attribs->m_isFront = frontNode.as<bool>();
    }

    if (steeringLimitsNode.IsDefined()){
        attribs->m_steeringLimitMax = steeringLimitsNode["high"].as<double>();
        attribs->m_steeringLimitMin = steeringLimitsNode["low"].as<double>();
    }

    if (maxEnginePowerNode.IsDefined()){
        attribs->m_enginePowerMax = maxEnginePowerNode.as<double>();
    }

    if (maxBrakePowerNode.IsDefined()){
        attribs->m_brakePowerMax = maxBrakePowerNode.as<double>();
    }

    return valid;
}

afJointType ADFUtils::getJointTypeFromString(const std::string &a_joint_str)
{
    afJointType jointType;

    if ((strcmp(a_joint_str.c_str(), "hinge") == 0)
            || (strcmp(a_joint_str.c_str(), "revolute") == 0)
            || (strcmp(a_joint_str.c_str(), "continuous") == 0)){
        jointType = afJointType::REVOLUTE;
    }
    else if ((strcmp(a_joint_str.c_str(), "slider") == 0)
             || (strcmp(a_joint_str.c_str(), "prismatic") == 0)){
        jointType = afJointType::PRISMATIC;
    }
    else if ((strcmp(a_joint_str.c_str(), "fixed") == 0)){
        jointType = afJointType::FIXED;
    }
    else if ((strcmp(a_joint_str.c_str(), "spring") == 0)){
        jointType = afJointType::LINEAR_SPRING;
    }
    else if ((strcmp(a_joint_str.c_str(), "linear spring") == 0)){
        jointType = afJointType::LINEAR_SPRING;
    }
    else if ((strcmp(a_joint_str.c_str(), "torsion spring") == 0)
             || (strcmp(a_joint_str.c_str(), "torsional spring") == 0)
             || (strcmp(a_joint_str.c_str(), "angular spring") == 0)){
        jointType = afJointType::TORSION_SPRING;
    }
    else if ((strcmp(a_joint_str.c_str(), "p2p") == 0)){
        jointType = afJointType::P2P;
    }
    else{
        std::cerr << "ERROR! JOINT TYPE NOT UNDERSTOOD \n";
        jointType = afJointType::INVALID;
    }

    return jointType;
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

    attribs->m_geometryType = afGeometryType::INVALID;

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


    if (meshPathLRNode.IsDefined()){
        attribs->m_meshPath = meshPathLRNode.as<std::string>();
    }
    else if(meshPathHRNode.IsDefined()){
        attribs->m_meshPath = meshPathHRNode.as<std::string>();
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
    YAML::Node &node = *a_node;

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
    YAML::Node & node = *a_node;
    YAML::Node controllerNode = node["controller"];

    if (controllerNode.IsDefined()){
        if( (controllerNode["P"]).IsDefined())
            attribs->P = controllerNode["P"].as<double>();
        if( (controllerNode["I"]).IsDefined())
            attribs->I = controllerNode["I"].as<double>();
        if( (controllerNode["D"]).IsDefined())
            attribs->D = controllerNode["D"].as<double>();

        // If the PID controller in defined, the gains will be used to command the joint force (effort)
        attribs->m_outputType = afControlType::FORCE;
    }
    else{
        // If the controller gains are not defined, a velocity based control will be used.
        // The tracking velocity can be controller by setting "max motor impulse" field
        // for the joint data-block in the ADF file.
        attribs->P = 10;
        attribs->I = 0;
        attribs->D = 0;
        attribs->m_outputType = afControlType::VELOCITY;
    }

    return true;
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


bool ADFLoader_1_0::loadObjectAttribs(std::string root_config_file, std::string a_objName, afObjectType a_objType, afBaseObjectAttributes *attribs)
{
    YAML::Node rootNode;
    try{
        rootNode = YAML::LoadFile(root_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << root_config_file << std::endl;
        return 0;
    }
    YAML::Node node = rootNode[a_objName];
    switch (a_objType) {
    case afObjectType::CONSTRAINT_ACTUATOR:
        return loadConstraintActuatorAttribs(&node, (afConstraintActuatorAttributes*)attribs);
    case afObjectType::RAYTRACER_SENSOR:
        return loadRayTracerSensorAttribs(&node, (afRayTracerSensorAttributes*)attribs);
    case afObjectType::RESISTANCE_SENSOR:
        return loadResistanceSensorAttribs(&node, (afResistanceSensorAttributes*)attribs);
    case afObjectType::RIGID_BODY:
        return loadRigidBodyAttribs(&node, (afRigidBodyAttributes*)attribs);
    case afObjectType::SOFT_BODY:
        return loadSoftBodyAttribs(&node, (afSoftBodyAttributes*)attribs);
    case afObjectType::VEHICLE:
        return loadVehicleAttribs(&node, (afVehicleAttributes*)attribs);
    case afObjectType::LIGHT:
        return loadLightAttribs(&node, (afLightAttributes*)attribs);
    case afObjectType::CAMERA:
        return loadCameraAttribs(&node, (afCameraAttributes*)attribs);
    default:
        return false;
    }
}

bool ADFLoader_1_0::loadLightAttribs(YAML::Node *a_node, afLightAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: LIGHT'S " << node << " NODE IS NULL\n";
        return false;
    }

    if (attribs == nullptr){
        std::cerr << "ERROR: LIGHT'S ATTRIBUTES IS A NULLPTR\n";
        return false;
    }

    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node parentNode = node["parent"];
    YAML::Node locationNode = node["location"];
    YAML::Node directionNode = node["direction"];
    YAML::Node spotExponentNode = node["spot exponent"];
    YAML::Node shadowQualityNode = node["shadow quality"];
    YAML::Node cuttOffAngleNode = node["cutoff angle"];

    bool valid = true;

    cVector3d location, direction;
    double spot_exponent, cuttoff_angle;
    int shadow_quality;

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getHierarchyAttribsFromNode(&node, &attribs->m_hierarchyAttribs);
    adfUtils.getKinematicAttribsFromNode(&node, &attribs->m_kinematicAttribs);

    if (directionNode.IsDefined()){
        attribs->m_direction = ADFUtils::toXYZ<cVector3d>(&directionNode);
    }

    if (spotExponentNode.IsDefined()){
        attribs->m_spotExponent = spotExponentNode.as<double>();
    }

    if (shadowQualityNode.IsDefined()){
        attribs->m_shadowQuality = shadowQualityNode.as<uint>();
    }

    if (cuttOffAngleNode.IsDefined()){
        attribs->m_cuttoffAngle = cuttOffAngleNode.as<double>();
    }

    return valid;
}

bool ADFLoader_1_0::loadCameraAttribs(YAML::Node *a_node, afCameraAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: CAMERA'S " << node << " NODE IS NULL\n";
        return false;
    }

    if (attribs == nullptr){
        std::cerr << "ERROR: CAMERA'S ATTRIBUTES IS A NULLPTR\n";
        return false;
    }

    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node parentNode = node["parent"];
    YAML::Node locationNode = node["location"];
    YAML::Node lookAtNode = node["look at"];
    YAML::Node upNode = node["up"];
    YAML::Node clippingPlaneNode = node["clipping plane"];
    YAML::Node fieldViewAngleNode = node["field view angle"];
    YAML::Node orthoWidthNode = node["orthographic view width"];
    YAML::Node stereoNode = node["stereo"];
    YAML::Node controllingDevicesDataNode = node["controlling devices"];
    YAML::Node monitorNode = node["monitor"];
    YAML::Node publishImageNode = node["publish image"];
    YAML::Node publishImageIntervalNode = node["publish image interval"];
    YAML::Node publishDepthNode = node["publish depth"];
    YAML::Node publishDepthIntervalNode = node["publish depth interval"];
    YAML::Node multiPassNode = node["multipass"];

    bool valid = true;

    ADFUtils adfUtils;
    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getKinematicAttribsFromNode(&node, &attribs->m_kinematicAttribs);
    adfUtils.getHierarchyAttribsFromNode(&node, &attribs->m_hierarchyAttribs);

    if (lookAtNode.IsDefined()){
        attribs->m_lookAt = ADFUtils::toXYZ<cVector3d>(&lookAtNode);
    }

    if (upNode.IsDefined()){
        attribs->m_up = ADFUtils::toXYZ<cVector3d>(&upNode);
    }

    if (clippingPlaneNode.IsDefined()){
        attribs->m_nearPlane = clippingPlaneNode["near"].as<double>();
        attribs->m_farPlane = clippingPlaneNode["far"].as<double>();
    }

    if (fieldViewAngleNode.IsDefined()){
        attribs->m_fieldViewAngle = fieldViewAngleNode.as<double>();
    }

    if (orthoWidthNode.IsDefined()){
        attribs->m_orthographic = true;
        attribs->m_orthoViewWidth = orthoWidthNode.as<double>();
    }

    if (stereoNode.IsDefined()){
        std::string mode_str = stereoNode["mode"].as<std::string>();
        if (mode_str.compare("PASSIVE") || mode_str.compare("passive") || mode_str.compare("Passive")){
            attribs->m_stereo = true;
        }
        attribs->m_stereoEyeSeparation = stereoNode["eye separation"].as<double>();
        attribs->m_stereFocalLength = stereoNode["focal length"].as<double>();
    }

    if (monitorNode.IsDefined()){
        attribs->m_monitorNumber = monitorNode.as<uint>();
    }

    if (controllingDevicesDataNode.IsDefined()){
        for(uint idx = 0 ; idx < controllingDevicesDataNode.size() ; idx++){
            attribs->m_controllingDeviceNames.push_back( controllingDevicesDataNode[idx].as<std::string>());
        }
    }

    if (publishImageNode.IsDefined()){
        attribs->m_publishImage = publishImageNode.as<bool>();
    }

    if (publishImageIntervalNode.IsDefined()){
        attribs->m_publishImageInterval = publishImageIntervalNode.as<uint>();
    }

    if (publishDepthNode.IsDefined()){
        attribs->m_publishDepth = publishDepthNode.as<bool>();
    }

    if (publishDepthIntervalNode.IsDefined()){
        attribs->m_publishDepthInterval = publishDepthIntervalNode.as<uint>();
    }

    if (multiPassNode.IsDefined()){
        attribs->m_multiPass = multiPassNode.as<bool>();
    }

    return valid;
}

bool ADFLoader_1_0::loadRigidBodyAttribs(YAML::Node *a_node, afRigidBodyAttributes *attribs)
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

bool ADFLoader_1_0::loadSoftBodyAttribs(YAML::Node *a_node, afSoftBodyAttributes *attribs)
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

bool ADFLoader_1_0::loadJointAttribs(YAML::Node *a_node, afJointAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: JOINT'S YAML CONFIG DATA IS NULL\n";
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
    adfUtils.getCommunicationAttribsFromNode(&node, &attribs->m_communicationAttribs);
    adfUtils.getJointControllerAttribsFromNode(&node, &attribs->m_controllerAttribs);

    attribs->m_parentPivot = ADFUtils::toXYZ<cVector3d>(&parentPivotNode);
    attribs->m_childPivot = ADFUtils::toXYZ<cVector3d>(&childPivotNode);
    attribs->m_parentAxis = ADFUtils::toXYZ<cVector3d>(&parentAxisNode);
    attribs->m_childAxis = ADFUtils::toXYZ<cVector3d>(&childAxisNode);
    attribs->m_parentPivot = ADFUtils::toXYZ<cVector3d>(&parentPivotNode);
    attribs->m_jointType = ADFUtils::getJointTypeFromString(typeNode.as<std::string>());


    if(offsetNode.IsDefined()){
        attribs->m_offset= offsetNode.as<double>();
    }

    if (dampingNode.IsDefined()){
        attribs->m_damping = dampingNode.as<double>();
    }

    if(limitsNode.IsDefined()){
        attribs->m_lowerLimit = limitsNode["low"].as<double>();
        attribs->m_upperLimit = limitsNode["high"].as<double>();
        attribs->m_enableLimits = true;
    }

    if(enableMotorNode.IsDefined()){
        attribs->m_enableMotor = enableFeedbackNode.as<bool>();
    }

    if(maxMotorImpulseNode.IsDefined()){
        attribs->m_maxMotorImpulse = maxMotorImpulseNode.as<double>();
    }

    if(stiffnessNode.IsDefined()){
        attribs->m_stiffness = stiffnessNode.as<double>();
    }

    if(equilibriumPointNode.IsDefined()){
        attribs->m_equilibriumPoint = equilibriumPointNode.as<double>();
    }

    if(erpNode.IsDefined()){
        attribs->m_erp = erpNode.as<double>();
    }

    if(cfmNode.IsDefined()){
        attribs->m_cfm = cfmNode.as<double>();
    }

    if (ignoreInterCollisionNode.IsDefined()){
       attribs->m_ignoreInterCollision = ignoreInterCollisionNode.as<bool>();
    }

    if (enableFeedbackNode.IsDefined()){
        attribs->m_enableFeedback = enableFeedbackNode.as<bool>();
    }

    return true;
}

bool ADFLoader_1_0::loadSensorAttribs(YAML::Node *a_node, afSensorAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: SENSOR'S YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node parentNameNode = node["parent"];
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node publishFrequencyNode = node["publish frequency"];

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getHierarchyAttribsFromNode(&node, &attribs->m_hierarchyAttribs);
    adfUtils.getKinematicAttribsFromNode(&node, &attribs->m_kinematicAttribs);
    adfUtils.getCommunicationAttribsFromNode(&node, &attribs->m_communicationAttribs);

    return result;
}

bool ADFLoader_1_0::loadRayTracerSensorAttribs(YAML::Node *a_node, afRayTracerSensorAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: SENSOR'S YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node parentNameNode = node["parent"];
    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node publishFrequencyNode = node["publish frequency"];
    YAML::Node rangeNode = node["range"];
    YAML::Node visibleNode = node["visible"];
    YAML::Node visibleSizeNode = node["visible size"];
    YAML::Node arrayNode = node["array"];
    YAML::Node meshNode = node["mesh"];
    YAML::Node parametricNode = node["parametric"];

    loadSensorAttribs(&node, attribs);

    attribs->m_specificationType = afSensactorSpecificationType::INVALID;

    if(rangeNode.IsDefined()){
        attribs->m_range = rangeNode.as<double>();
    }

    if (visibleNode.IsDefined()){
        attribs->m_visible = visibleNode.as<bool>();
    }

    if (visibleSizeNode.IsDefined()){
        attribs->m_visibleSize = visibleSizeNode.as<double>();
    }

    if (arrayNode.IsDefined()){
        uint count = arrayNode.size();
        attribs->m_raysAttribs.resize(count);
        cTransform T_sINp = attribs->m_kinematicAttribs.m_location;
        cMatrix3d R_sINp = T_sINp.getLocalRot();
        for (uint i = 0 ; i < count ; i++){
            YAML::Node offsetNode = arrayNode[i]["offset"];
            YAML::Node directionNode = arrayNode[i]["direction"];

            cVector3d offset, start, dir, end;

            offset = ADFUtils::toXYZ<cVector3d>(&offsetNode);
            dir = ADFUtils::toXYZ<cVector3d>(&directionNode);
            start = T_sINp * offset;
            dir = R_sINp * dir;
            dir.normalize();
            end = start + dir * attribs->m_range;

            attribs->m_raysAttribs[i].m_range = attribs->m_range;
            attribs->m_raysAttribs[i].m_rayFromLocal = start;
            attribs->m_raysAttribs[i].m_direction = dir;
            attribs->m_raysAttribs[i].m_rayToLocal = end;
        }
        attribs->m_specificationType = afSensactorSpecificationType::ARRAY;
        result = true;
    }

    else if (meshNode.IsDefined()){
        // We are not going to load the mesh in the loader and this would restrict us in using a
        // specific mesh processing library, thus let the ambf_framework load the mesh.
        attribs->m_contourMesh = meshNode.as<std::string>();
        attribs->m_specificationType = afSensactorSpecificationType::MESH;
        result = true;
    }
    else if (parametricNode.IsDefined()){
        YAML::Node resolutionNode = parametricNode["resolution"];
        YAML::Node horSpanNode = parametricNode["horizontal angle"];
        YAML::Node verSpanNode = parametricNode["vertical angle"];
        YAML::Node startOffsetNode = parametricNode["start offset"];

        uint resolution = resolutionNode.as<uint>();
        double horizontal_span = horSpanNode.as<double>();
        double vertical_span = verSpanNode.as<double>();
        double start_offset = startOffsetNode.as<double>();

        if (resolution < 2){
            std::cerr << "ERROR! FOR SENSOR \"" << attribs->m_identificationAttribs.m_name << "\" RESOLUTION MUST BE GREATER THAN EQUAL TO 2. IGNORING! \n";
            return false;
        }

        double h_start = -horizontal_span / 2.0;
        double v_start = -vertical_span / 2.0;
        double h_step = horizontal_span / (resolution - 1);
        double v_step = vertical_span / (resolution - 1);
        attribs->m_raysAttribs.resize(resolution * resolution);

        // Choose an initial point facing the +ve x direction
        cVector3d nx(1, 0, 0);
        cTransform T_sINp = attribs->m_kinematicAttribs.m_location;
        cMatrix3d R_sINp = T_sINp.getLocalRot();
        for (uint i = 0 ; i < resolution ; i++){
            double h_angle = h_start + i * h_step;
            for (uint j = 0 ; j < resolution ; j++){
                double v_angle = v_start + j * v_step;

                cMatrix3d deltaR;
                cVector3d start, dir, end;

                deltaR.setExtrinsicEulerRotationRad(0, v_angle, h_angle, cEulerOrder::C_EULER_ORDER_XYZ);
                start = T_sINp * (deltaR * (nx * start_offset));
                dir = R_sINp * deltaR * nx;
                dir.normalize();
                end = start + dir * attribs->m_range;

                uint sIdx = resolution * i + j;
                attribs->m_raysAttribs[sIdx].m_range = attribs->m_range;
                attribs->m_raysAttribs[sIdx].m_rayFromLocal = start;
                attribs->m_raysAttribs[sIdx].m_direction = dir;
                attribs->m_raysAttribs[sIdx].m_rayToLocal = end;
            }

        }
        result = true;
        attribs->m_specificationType = afSensactorSpecificationType::PARAMETRIC;
    }
    else{
        result = false;
    }

    return result;

}

bool ADFLoader_1_0::loadActuatorAttribs(YAML::Node *a_node, afActuatorAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: ACTUATOR'S YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node parentNameNode = node["parent"];
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node publishFrequencyNode = node["publish frequency"];

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);
    adfUtils.getHierarchyAttribsFromNode(&node, &attribs->m_hierarchyAttribs);
    adfUtils.getKinematicAttribsFromNode(&node, &attribs->m_kinematicAttribs);
    adfUtils.getCommunicationAttribsFromNode(&node, &attribs->m_communicationAttribs);

    return result;
}

bool ADFLoader_1_0::loadConstraintActuatorAttribs(YAML::Node *a_node, afConstraintActuatorAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: ACTUATOR'S YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = node["name"];
    YAML::Node namespaceNode = node["namespace"];
    YAML::Node parentNameNode = node["parent"];
    YAML::Node posNode = node["location"]["position"];
    YAML::Node rotNode = node["location"]["orientation"];
    YAML::Node publishFrequencyNode = node["publish frequency"];
    YAML::Node visibleNode = node["visible"];
    YAML::Node visibleSizeNode = node["visible size"];
    YAML::Node maxImpulseNode = node["max impulse"];
    YAML::Node tauNode = node["tau"];

    loadActuatorAttribs(&node, attribs);

    if (visibleNode.IsDefined()){
        attribs->m_visible = visibleNode.as<bool>();
    }

    if (visibleSizeNode.IsDefined()){
        attribs->m_visibleSize = visibleSizeNode.as<bool>();
    }

    if (maxImpulseNode.IsDefined()){
        attribs->m_maxImpulse = maxImpulseNode.as<double>();
    }

    if (tauNode.IsDefined()){
        attribs->m_tau = tauNode.as<double>();
    }

    return result;
}

bool ADFLoader_1_0::loadVehicleAttribs(YAML::Node* a_node, afVehicleAttributes *attribs)
{
    YAML::Node& node = *a_node;
    if (node.IsNull()){
        std::cerr << "ERROR: VEHICLE'S YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node nameNode = node["name"];
    YAML::Node nameSpaceNode = node["namespace"];
    YAML::Node chassisNode = node["chassis"];
    YAML::Node meshPathHRNode = node["high resolution path"];
    YAML::Node wheelsNode = node["wheels"];

    ADFUtils adfUtils;

    adfUtils.getIdentificationAttribsFromNode(&node, &attribs->m_identificationAttribs);

    if (chassisNode.IsDefined()){
        attribs->m_chassisBodyName = chassisNode.as<std::string>();
    }

    boost::filesystem::path high_res_filepath;

    if (meshPathHRNode.IsDefined()){
        attribs->m_wheelsVisualPath = meshPathHRNode.as<std::string>();
    }

    for (uint i = 0 ; i < wheelsNode.size() ; i++){
        afWheelAttributes _wheelAttribs;
        YAML::Node _wheelNode = wheelsNode[i];
        if (adfUtils.getWheelAttribsFromNode(&_wheelNode, &_wheelAttribs)){
            attribs->m_wheelAttribs.push_back(_wheelAttribs);
        }
    }

    return result;
}

bool ADFLoader_1_0::loadMultiBodyAttribs(YAML::Node* a_node, afMultiBodyAttributes *attribs)
{

}

bool ADFLoader_1_0::loadWorldAttribs(YAML::Node* a_node, afWorldAttributes *attribs)
{

}
