
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

    \author    <http://www.aimlab.wpi.edu>
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \courtesy: Dejaime Antônio de Oliveira Neto at https://www.gamedev.net/profile/187867-dejaime/ for initial direction
    \motivation: https://www.gamedev.net/articles/programming/engines-and-middleware/yaml-basics-and-parsing-with-yaml-cpp-r3508/
    \version   $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "afMultiBody.h"
#include "chai3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#define PI 3.14159
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Declare Static Variables
cMaterial afRigidBody::m_mat;
afRigidBodySurfaceProperties afRigidBody::m_surfaceProps;

cMaterial afSoftBody::m_mat;

boost::filesystem::path afConfigHandler::s_boostBaseDir;
std::string afConfigHandler::s_color_config;
std::vector<std::string> afConfigHandler::s_multiBody_configs;
std::string afConfigHandler::s_world_config;
YAML::Node afConfigHandler::s_colorsNode;
std::map<std::string, std::string> afConfigHandler::s_gripperConfigFiles;

cBulletWorld* afWorld::m_chaiWorld;
double afWorld::m_encl_length;
double afWorld::m_encl_width;
double afWorld::m_encl_height;
//------------------------------------------------------------------------------

/// End declare static variables

/// Utility Functions

///
/// \brief assignXYZ
/// \param v
///
void assignXYZ(YAML::Node* node, btVector3 *v){
    v->setX((*node)["x"].as<double>());
    v->setY((*node)["y"].as<double>());
    v->setZ((*node)["z"].as<double>());
}

///
/// \brief assignXYZ
/// \param node
/// \param v
///
void assignXYZ(YAML::Node* node, cVector3d *v){
    v->x((*node)["x"].as<double>());
    v->y((*node)["y"].as<double>());
    v->z((*node)["z"].as<double>());
}


///
/// \brief assignRPY
/// \param v
/// \param node
///
void assignRPY(YAML::Node* node, btVector3 *v){
    v->setX((*node)["r"].as<double>());
    v->setY((*node)["p"].as<double>());
    v->setZ((*node)["y"].as<double>());
}


///////////////////////////////////////////////

///
/// \brief afConfigHandler::afConfigHandler
///
afConfigHandler::afConfigHandler(){

}

///
/// \brief afConfigHandler::load_yaml
/// \param a_config_file
/// \return
///
bool afConfigHandler::loadBaseConfig(std::string a_config_file){
    try{
        configNode = YAML::LoadFile(a_config_file);
    } catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << a_config_file << std::endl;
        return 0;
    }


    //Declare all the YAML Params that we want to look for
    YAML::Node cfgWorldFiles = configNode["world config"];
    YAML::Node cfgColorFile = configNode["color config"];
    YAML::Node cfgMultiBodyFiles = configNode["multibody configs"];
    YAML::Node cfgGripperConfigs = configNode["gripper configs"];


    s_boostBaseDir = boost::filesystem::path(a_config_file).parent_path();

    if(cfgWorldFiles.IsDefined()){
        boost::filesystem::path world_cfg_filename = cfgWorldFiles.as<std::string>();
        if (world_cfg_filename.is_relative()){
            world_cfg_filename = s_boostBaseDir / world_cfg_filename;
        }
        s_world_config = world_cfg_filename.c_str();
    }
    else{
        std::cerr << "ERROR! WORLD CONFIG NOT DEFINED \n";
        return 0;
    }

    if(cfgColorFile.IsDefined()){
        boost::filesystem::path color_cfg_filename = cfgColorFile.as<std::string>();
        if (color_cfg_filename.is_relative()){
            color_cfg_filename = s_boostBaseDir / color_cfg_filename;
        }
        s_color_config = color_cfg_filename.c_str();
        s_colorsNode = YAML::LoadFile(s_color_config.c_str());
        if (!s_colorsNode){
            std::cerr << "ERROR! COLOR CONFIG NOT FOUND \n";
        }
    }
    else{
        return 0;
    }

    if (cfgMultiBodyFiles.IsDefined()){
        for (size_t i = 0 ; i < cfgMultiBodyFiles.size() ; i++){
            boost::filesystem::path mb_cfg_filename =  cfgMultiBodyFiles[i].as<std::string>();
            if (mb_cfg_filename.is_relative()){
                mb_cfg_filename = s_boostBaseDir / mb_cfg_filename;
            }
            s_multiBody_configs.push_back(std::string(mb_cfg_filename.c_str()));
        }
    }
    else{
        std::cerr << "PATH AND MULTIBODY CONFIG NOT DEFINED \n";
        return 0;
    }

    if(cfgGripperConfigs.IsDefined()){
        for(size_t i = 0 ; i < cfgGripperConfigs.size(); ++i){
            std::string gripper_name = cfgGripperConfigs[i].as<std::string>();
            boost::filesystem::path gripper_cfg_filename =  configNode[gripper_name].as<std::string>();
            if (gripper_cfg_filename.is_relative()){
                gripper_cfg_filename = s_boostBaseDir / gripper_cfg_filename;
            }
            s_gripperConfigFiles[gripper_name] = std::string(gripper_cfg_filename.c_str());
        }
    }
    else{
        std::cerr << "ERROR! GRIPPER CONFIGS NOT DEFINED \n";
        return 0;
    }

    return 1;
}

///
/// \brief afConfigHandler::get_world_config
/// \return
///
std::string afConfigHandler::getWorldConfig(){
    return s_world_config;
}

///
/// \brief afConfigHandler::get_puzzle_config
/// \return
///
std::string afConfigHandler::getMultiBodyConfig(int i){
    if (i <= numMultiBodyConfig()){
        return s_multiBody_configs[i];
    }
    else{
        printf("i = %d, Whereas only %d multi bodies specified", i, s_multiBody_configs.size());
        return "";
    }
}

///
/// \brief afConfigHandler::get_color_config
/// \return
///
std::string afConfigHandler::getColorConfig(){
    return s_color_config;
}

///
/// \brief afConfigHandler::get_gripper_config
/// \param a_gripper_name
/// \return
///
std::string afConfigHandler::getGripperConfig(std::string a_gripper_name){
    if(s_gripperConfigFiles.find(a_gripper_name) != s_gripperConfigFiles.end()){
        return s_gripperConfigFiles[a_gripper_name];
    }
    else{
        std::cerr << "WARNING! GRIPPER CONFIG FOR \"" << a_gripper_name
                  << "\" NOT FOUND, RETURNING DEFAULT \n";
        return s_gripperConfigFiles["Default"];
    }
}

///
/// \brief afConfigHandler::get_color_rgba
/// \param a_color_name
/// \return
///
std::vector<double> afConfigHandler::getColorRGBA(std::string a_color_name){
    std::vector<double> color_rgba = {0.5, 0.5, 0.5, 0.5};
    // Help from https://stackoverflow.com/questions/15425442/retrieve-random-key-element-for-stdmap-in-c
    if(strcmp(a_color_name.c_str(), "random") == 0 || strcmp(a_color_name.c_str(), "RANDOM") == 0){
        YAML::const_iterator it = s_colorsNode.begin();
        std::advance(it, rand() % s_colorsNode.size());
        color_rgba[0] = it->second["r"].as<int>() / 255.0;
        color_rgba[1] = it->second["g"].as<int>() / 255.0;
        color_rgba[2] = it->second["b"].as<int>() / 255.0;
        color_rgba[3] = it->second["a"].as<int>() / 255.0;
    }
    else if(s_colorsNode[a_color_name].IsDefined()){
        color_rgba[0] = s_colorsNode[a_color_name]["r"].as<int>() / 255.0;
        color_rgba[1] = s_colorsNode[a_color_name]["g"].as<int>() / 255.0;
        color_rgba[2] = s_colorsNode[a_color_name]["b"].as<int>() / 255.0;
        color_rgba[3] = s_colorsNode[a_color_name]["a"].as<int>() / 255.0;
    }
    else{
        std::cerr << "WARNING! COLOR NOT FOUND, RETURNING BALANCED COLOR\n";
    }
    return color_rgba;
}

///
/// \brief afBody::afBody
/// \param a_world
///
afRigidBody::afRigidBody(cBulletWorld* a_world): cBulletMultiMesh(a_world){
    setFrameSize(0.5);
    m_mesh_name.clear();
    m_collision_mesh_name.clear();
    m_scale = 1.0;
}

///
/// \brief afRigidBody::upwardTreePopulation
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::upwardTreePopulation(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //1a. We add the child body and all of it's children to this body
    std::vector<afRigidBodyPtr> cBodies;
    cBodies = a_childBody->m_childrenBodies;
    cBodies.push_back(a_childBody);

    std::vector<afRigidBodyPtr>::iterator cBodyIt;
    for (cBodyIt = cBodies.begin() ; cBodyIt != cBodies.end(); ++cBodyIt){
        bool _cExists = false;
        for (size_t cIdx = 0; cIdx < m_childrenBodies.size() ; cIdx++){
            if (*cBodyIt == m_childrenBodies[cIdx]){
                _cExists = true;
                break;
            }
        }

        if (!_cExists){
            m_childrenBodies.push_back(*cBodyIt);
        }
        else{
//            std::cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A CHILD BODY NAMED \""
//                      << (*cBodyIt)->m_name << "\" PARALLEL LINKAGE FOUND" << std::endl;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    //1b. We add the child joint and the children joint of child body to this body
    std::vector<afJointPtr> cJoints;
    cJoints = a_childBody->m_joints;
    cJoints.push_back(a_jnt);

    std::vector<afJointPtr>::iterator cJointIt;
    for (cJointIt = cJoints.begin() ; cJointIt != cJoints.end(); ++cJointIt){
        bool _cJointExists = false;
        for (size_t jIdx = 0; jIdx < m_joints.size() ; jIdx++){
            if (*cJointIt == m_joints[jIdx]){
                _cJointExists = true;
                break;
            }
        }
        if(!_cJointExists){
            m_joints.push_back(*cJointIt);
        }
        else{
//            std::cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A JOINT NAMED \""
//                      << (*cJointIt)->m_name << "\ PARALLEL LINKAGE FOUND" << std::endl;
        }
    }
}

///
/// \brief afRigidBody::downwardTreePopulation
/// \param a_parentBody
///
void afRigidBody::downwardTreePopulation(afRigidBodyPtr a_parentBody){
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //2a. We add the child body and all of it's children to this body
    std::vector<afRigidBodyPtr> pBodies;
    pBodies = a_parentBody->m_parentBodies;
    pBodies.push_back(a_parentBody);

    std::vector<afRigidBodyPtr>::iterator pBobyIt;
    for (pBobyIt = pBodies.begin() ; pBobyIt != pBodies.end(); ++pBobyIt){
        bool _pExists = false;
        for (size_t pIdx = 0; pIdx < m_parentBodies.size() ; pIdx++){
            if (*pBobyIt == m_parentBodies[pIdx]){
                _pExists = true;
                break;
            }
        }

        if (!_pExists){
            m_parentBodies.push_back(*pBobyIt);
        }
        else{
//            std::cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A CHILD BODY NAMED \""
//                      << (*pBobyIt)->m_name << "\" PARALLEL LINKAGE FOUND" << std::endl;
        }
    }
}

///
/// \brief afBody::add_child_body
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::addChildBody(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
    //TODO: TEST THIS LOGIC RIGOROUSLY
    if (this == a_childBody){
        std::cerr << "INFO, BODY \"" << this->m_name << "\": CANNOT HAVE ITSELF AS ITS CHILD" << std::endl;
    }
    else{
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //1.. We go higher up the tree and add all the children and joints beneath to the parents
        std::vector<afRigidBodyPtr> pBodies;
        pBodies = this->m_parentBodies;
        pBodies.push_back(this);

        std::vector<afRigidBodyPtr>::iterator pIt;

        for (pIt = pBodies.begin() ; pIt != pBodies.end() ; ++pIt){
            (*pIt)->upwardTreePopulation(a_childBody, a_jnt);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //2. Now we add this body as the parent of all the children of the child body
        std::vector<afRigidBodyPtr> cBodies;
        cBodies = a_childBody->m_childrenBodies;
        cBodies.push_back(a_childBody);

        std::vector<afRigidBodyPtr>::iterator cIt;
        for (cIt = cBodies.begin() ; cIt != cBodies.end() ; ++cIt){
            (*cIt)->downwardTreePopulation(this);
        }
    }
}

///
/// \brief afBody::load
/// \param file
/// \param name
/// \param mB
/// \param name_remapping
/// \return
///
bool afRigidBody::loadRidigBody(std::string rb_config_file, std::string node_name, afMultiBodyPtr mB) {
    YAML::Node baseNode;
    m_mBPtr = mB;
    try{
        baseNode = YAML::LoadFile(rb_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << rb_config_file << std::endl;
        return 0;
    }
    YAML::Node bodyNode = baseNode[node_name];
    return loadRidigBody(&bodyNode, node_name, mB);
}

///
/// \brief afRigidBody::loadRidigBody
/// \param rb_config_data
/// \param name
/// \param mB
/// \return
///
bool afRigidBody::loadRidigBody(YAML::Node* rb_node, std::string node_name, afMultiBodyPtr mB){
    YAML::Node bodyNode = *rb_node;
    m_mBPtr = mB;
    if (bodyNode.IsNull()){
        std::cerr << "ERROR: RIGID BODY'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node bodyName = bodyNode["name"];
    YAML::Node bodyMesh = bodyNode["mesh"];
    YAML::Node bodyShape = bodyNode["shape"];
    YAML::Node bodyGeometry = bodyNode["geometry"];
    YAML::Node bodyCollisionMesh = bodyNode["collision mesh"];
    YAML::Node bodyCollisionShape = bodyNode["collision shape"];
    YAML::Node bodyCollisionGeometry = bodyNode["collision geometry"];
    YAML::Node bodyScale = bodyNode["scale"];
    YAML::Node bodyInertialOffsetPos = bodyNode["inertial offset"]["position"];
    YAML::Node bodyInertialOffsetRot = bodyNode["inertial offset"]["orientation"];
    YAML::Node bodyMeshPathHR = bodyNode["high resolution path"];
    YAML::Node bodyMeshPathLR = bodyNode["low resolution path"];
    YAML::Node bodyNameSpace = bodyNode["namespace"];
    YAML::Node bodyMass = bodyNode["mass"];
    YAML::Node bodyLinGain = bodyNode["linear gain"];
    YAML::Node bodyAngGain = bodyNode["angular gain"];
    YAML::Node bodyInertia = bodyNode["inertia"];
    YAML::Node bodyPos = bodyNode["location"]["position"];
    YAML::Node bodyRot = bodyNode["location"]["orientation"];
    YAML::Node bodyColorRGBA = bodyNode["color rgba"];
    YAML::Node bodyColor = bodyNode["color"];
    YAML::Node bodyLinDamping = bodyNode["damping"]["linear"];
    YAML::Node bodyAngDamping = bodyNode["damping"]["angular"];
    YAML::Node bodyStaticFriction = bodyNode["friction"]["static"];
    YAML::Node bodyRollingFriction = bodyNode["friction"]["rolling"];
    YAML::Node bodyRestitution = bodyNode["restitution"];
    YAML::Node bodyPublishChildrenNames = bodyNode["publish children names"];
    YAML::Node bodyPublishJointPositions = bodyNode["publish joint positions"];
    YAML::Node bodyPublishFrequency = bodyNode["publish frequency"];
    YAML::Node bodyCollisionGroups = bodyNode["collision groups"];

    if(bodyName.IsDefined()){
        m_name = bodyName.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    // Check if the name is world and if it has already been defined.
    if (strcmp(m_name.c_str(), "world") == 0 || strcmp(m_name.c_str(), "World") == 0 || strcmp(m_name.c_str(), "WORLD") == 0){
        if(mB->getRidigBody(node_name, true)){
            // Since we already found a world body/frame, we can skip adding another
            return 0;
        }

    }

    bool _visual_geometry_valid = false;
    bool _collision_geometry_valid = false;
    m_visualGeometryType = GeometryType::invalid;
    m_collisionGeometryType = GeometryType::invalid;
    boost::filesystem::path high_res_filepath;
    boost::filesystem::path low_res_filepath;
    std::string _visual_shape_str;
    std::string _collision_shape_str;

    if (bodyCollisionShape.IsDefined()){
        _collision_geometry_valid = true;
        m_collisionGeometryType = GeometryType::shape;
        _collision_shape_str = bodyCollisionShape.as<std::string>();
    }

    if (bodyShape.IsDefined()){
        _visual_geometry_valid = true;
        m_visualGeometryType = GeometryType::shape;
        _visual_shape_str = bodyShape.as<std::string>();
        if (!_collision_geometry_valid){
            _collision_shape_str = _visual_shape_str;
            _collision_geometry_valid = true;
            m_collisionGeometryType = GeometryType::shape;
            bodyCollisionGeometry = bodyGeometry;
            bodyCollisionShape = bodyShape;
        }
    }
    else if(bodyMesh.IsDefined()){
        m_mesh_name = bodyMesh.as<std::string>();
        if (!m_mesh_name.empty()){
            // Each ridig body can have a seperate path for its low and high res meshes
            // Incase they are defined, we use those paths and if they are not, we use
            // the paths for the whole file
            if (bodyMeshPathHR.IsDefined()){
                high_res_filepath = bodyMeshPathHR.as<std::string>() + m_mesh_name;
                if (high_res_filepath.is_relative()){
                    high_res_filepath = mB->getMultiBodyPath() + '/' + high_res_filepath.c_str();
                }
            }
            else{
                high_res_filepath = mB->getHighResMeshesPath() + m_mesh_name;
            }
            _visual_geometry_valid = true;
            m_visualGeometryType = GeometryType::mesh;
        }

        // Only check for collision mesh definition if visual mesh is defined
        if (!_collision_geometry_valid){
            if(bodyCollisionMesh.IsDefined()){
                m_collision_mesh_name = bodyCollisionMesh.as<std::string>();
                if (!m_collision_mesh_name.empty()){
                    if (bodyMeshPathLR.IsDefined()){
                        low_res_filepath = bodyMeshPathLR.as<std::string>() + m_collision_mesh_name;
                        if (low_res_filepath.is_relative()){
                            low_res_filepath = mB->getMultiBodyPath() + '/' + low_res_filepath.c_str();
                        }
                    }
                    else{
                        // If low res path is not defined, use the high res path to load the high-res mesh for collision
                        low_res_filepath = mB->getLowResMeshesPath() + m_collision_mesh_name;
                    }
                    _collision_geometry_valid = true;
                    m_collisionGeometryType = GeometryType::mesh;
                }
            }
            else{
                m_collision_mesh_name = m_mesh_name;
                if (bodyMeshPathLR.IsDefined()){
                    low_res_filepath = bodyMeshPathLR.as<std::string>() + m_collision_mesh_name;
                    if (low_res_filepath.is_relative()){
                        low_res_filepath = mB->getMultiBodyPath() + '/' + low_res_filepath.c_str();
                    }
                }
                else{
                    // If low res path is not defined, use the high res path to load the high-res mesh for collision
                    low_res_filepath = mB->getLowResMeshesPath() + m_collision_mesh_name;
                }
                _collision_geometry_valid = true;
                m_collisionGeometryType = GeometryType::mesh;
            }
        }
    }

    if(!bodyMass.IsDefined()){
        std::cerr << "WARNING: Body "
                  << m_name
                  << "'s mass is not defined, hence ignoring\n";
        return 0;
    }
    else if(bodyMass.as<double>() < 0.0){
        std::cerr << "WARNING: Body "
                  << m_name
                  << "'s mass is \"" << m_mass << "\". Mass cannot be negative, ignoring\n";
        return 0;

    }
    else if (!_visual_geometry_valid && bodyMass.as<double>() > 0.0 && !bodyInertia.IsDefined()){
        std::cerr << "WARNING: Body "
                  << m_name
                  << "'s geometry is empty, mass > 0 and no intertia defined, hence ignoring\n";
        return 0;
    }
    else if (!_visual_geometry_valid && bodyMass.as<double>() > 0.0 && bodyInertia.IsDefined()){
        std::cerr << "INFO: Body "
                  << m_name
                  << "'s mesh field is empty but mass and interia defined\n";
    }

    if(bodyScale.IsDefined()){
        m_scale = bodyScale.as<double>();
    }

    if (m_visualGeometryType == GeometryType::mesh){
        if ( loadFromFile(high_res_filepath.c_str()) ){
            if(m_scale != 1.0){
                this->scale(m_scale);
            }
        }
        else{
            std::cerr << "WARNING: Body "
                      << m_name
                      << "'s mesh \"" << high_res_filepath << "\" not found\n";
        }
    }

    else if (m_visualGeometryType == GeometryType::shape){
        cMesh* tempMesh = new cMesh();
            if (_visual_shape_str.compare("Box") == 0 || _visual_shape_str.compare("box") == 0 || _visual_shape_str.compare("BOX") == 0){
                double x = bodyGeometry["x"].as<double>();
                double y = bodyGeometry["y"].as<double>();
                double z = bodyGeometry["z"].as<double>();
                cCreateBox(tempMesh, x, y, z);
            }
            else if (_visual_shape_str.compare("Sphere") == 0 || _visual_shape_str.compare("sphere") == 0 || _visual_shape_str.compare("SPHERE") == 0){
                int dx = bodyGeometry["dx"].as<int>();
                int dy = bodyGeometry["dy"].as<int>();
                double radius = bodyGeometry["radius"].as<double>();
                cCreateSphere(tempMesh, radius, dx, dy);
            }
            else if (_visual_shape_str.compare("Cylinder") == 0 || _visual_shape_str.compare("cylinder") == 0 || _visual_shape_str.compare("CYLINDER") == 0){
                int x_count = bodyGeometry["dx"].as<int>();
                int y_count = bodyGeometry["dy"].as<int>();
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                cCreateCylinder(tempMesh, height, radius, x_count, y_count, 1, true, true, cVector3d(0.0, 0.0,-0.5 * height));
            }
            else if (_visual_shape_str.compare("Capsule") == 0 || _visual_shape_str.compare("capsule") == 0 || _visual_shape_str.compare("CAPSULE") == 0){
                int dx = bodyGeometry["dx"].as<int>();
                int dy = bodyGeometry["dy"].as<int>();
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                cCreateEllipsoid(tempMesh, radius, radius, height, dx, dy);
            }
            else if (_visual_shape_str.compare("Cone") == 0 || _visual_shape_str.compare("cone") == 0 || _visual_shape_str.compare("Cone") == 0){
                int dx = bodyGeometry["dx"].as<int>();
                int dy = bodyGeometry["dy"].as<int>();
                int dz = bodyGeometry["dz"].as<int>();
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                cCreateCone(tempMesh, height, radius, 0, dx, dy, dz, true, true, cVector3d(0.0, 0.0, -0.5 * height));
            }
            m_meshes->push_back(tempMesh);
        }

    if(bodyColorRGBA.IsDefined()){
        m_mat.setColorf(bodyColorRGBA["r"].as<float>(),
                bodyColorRGBA["g"].as<float>(),
                bodyColorRGBA["b"].as<float>(),
                bodyColorRGBA["a"].as<float>());
    }
    else if(bodyColor.IsDefined()){
        std::vector<double> rgba = mB->getColorRGBA(bodyColor.as<std::string>());
        m_mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
    }

    setMaterial(m_mat);

    if(m_collisionGeometryType == GeometryType::mesh){

        if( m_lowResMesh.loadFromFile(low_res_filepath.c_str()) ){
            if(m_scale != 1.0){
                m_lowResMesh.scale(m_scale);
            }
        }
        else{
            std::cerr << "WARNING: Body "
                      << m_name
                      << "'s mesh \"" << low_res_filepath << "\" not found\n";
        }

    }
    else if (m_collisionGeometryType == GeometryType::shape){
        std::string _shape_str = bodyCollisionShape.as<std::string>();
        if (_shape_str.compare("Box") == 0 || _shape_str.compare("box") == 0 ||_shape_str.compare("BOX") == 0){
            double x = bodyCollisionGeometry["x"].as<double>();
            double y = bodyCollisionGeometry["y"].as<double>();
            double z = bodyCollisionGeometry["z"].as<double>();
            btVector3 halfExtents(x/2, y/2, z/2);
            m_bulletCollisionShape = new btBoxShape(halfExtents);
        }
        else if (_shape_str.compare("Sphere") == 0 || _shape_str.compare("sphere") == 0 ||_shape_str.compare("SPHERE") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            m_bulletCollisionShape = new btSphereShape(radius);
        }
        else if (_shape_str.compare("Cylinder") == 0 || _shape_str.compare("cylinder") == 0 ||_shape_str.compare("CYLINDER") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            double height = bodyCollisionGeometry["height"].as<double>();
            std::string axis = "z";
            if(bodyCollisionGeometry["axis"].IsDefined()){
                axis = bodyCollisionGeometry["axis"].as<std::string>();
            }
            if (axis.compare("x") == 0 || axis.compare("X") == 0){
                btVector3 halfExtents(height/2, radius, radius);
                m_bulletCollisionShape = new btCylinderShapeX(halfExtents);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                btVector3 halfExtents(radius, height/2, radius);
                m_bulletCollisionShape = new btCylinderShape(halfExtents);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                btVector3 halfExtents(radius, radius, height/2);
                m_bulletCollisionShape = new btCylinderShapeZ(halfExtents);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                btVector3 halfExtents(radius, radius, height/2);
                m_bulletCollisionShape = new btCylinderShapeZ(halfExtents);
            }
        }
        else if (_shape_str.compare("Capsule") == 0 || _shape_str.compare("capsule") == 0 ||_shape_str.compare("CAPSULE") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            double height = bodyCollisionGeometry["height"].as<double>();
            // Adjust for height as bullet treats the height as the distance
            // between the two spheres forming the capsule's ends.
            height = height - 2*radius;
            std::string axis = "z";
            if(bodyCollisionGeometry["axis"].IsDefined()){
                axis = bodyCollisionGeometry["axis"].as<std::string>();
            }
            if (axis.compare("x") == 0 || axis.compare("X") == 0){
                m_bulletCollisionShape = new btCapsuleShapeX(radius, height);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                m_bulletCollisionShape = new btCapsuleShape(radius, height);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                m_bulletCollisionShape = new btCapsuleShapeZ(radius, height);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                m_bulletCollisionShape = new btCapsuleShapeZ(radius, height);
            }
        }
        else if (_shape_str.compare("Cone") == 0 || _shape_str.compare("cone") == 0 ||_shape_str.compare("CONE") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            double height = bodyCollisionGeometry["height"].as<double>();
            std::string axis = "z";
            if(bodyCollisionGeometry["axis"].IsDefined()){
                axis = bodyCollisionGeometry["axis"].as<std::string>();
            }
            if (axis.compare("x") == 0 || axis.compare("X") == 0){
                m_bulletCollisionShape = new btConeShapeX(radius, height);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                m_bulletCollisionShape = new btConeShape(radius, height);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                m_bulletCollisionShape = new btConeShapeZ(radius, height);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                m_bulletCollisionShape = new btConeShapeZ(radius, height);
            }
        }
    }

    if (bodyNameSpace.IsDefined()){
        m_body_namespace = bodyNameSpace.as<std::string>();
    }
    else{
        m_body_namespace = mB->getNameSpace();
    }

    btTransform iOffTrans;
    btQuaternion iOffQuat;
    btVector3 iOffPos;
    iOffQuat.setEuler(0,0,0);
    iOffPos.setValue(0,0,0);

    if(bodyInertialOffsetPos.IsDefined()){
        assignXYZ( &bodyInertialOffsetPos, &iOffPos);
        if(bodyInertialOffsetRot.IsDefined()){
            double r = bodyInertialOffsetRot["r"].as<double>();
            double p = bodyInertialOffsetRot["p"].as<double>();
            double y = bodyInertialOffsetRot["y"].as<double>();
            iOffQuat.setEulerZYX(y, p, r);
        }
    }
    else if (m_collisionGeometryType == GeometryType::mesh){
        // Call the compute inertial offset before the build contact triangle method
        // Sanity check, see if a mesh is defined or not
        if (m_lowResMesh.m_meshes->size() > 0){
            iOffPos = computeInertialOffset(m_lowResMesh.m_meshes[0][0]);
        }
    }

    iOffTrans.setOrigin(iOffPos);
    iOffTrans.setRotation(iOffQuat);
    setInertialOffsetTransform(iOffTrans);

    if (m_collisionGeometryType == GeometryType::mesh){
        // Build contact triangles
        buildContactTriangles(0.001, &m_lowResMesh);
    }

    m_mass = bodyMass.as<double>();
    if(bodyLinGain.IsDefined()){
        K_lin = bodyLinGain["P"].as<double>();
        D_lin = bodyLinGain["D"].as<double>();
        _lin_gains_computed = true;
    }
    if(bodyAngGain.IsDefined()){
        K_ang = bodyAngGain["P"].as<double>();
        D_ang = bodyAngGain["D"].as<double>();
        _ang_gains_computed = true;
    }

    if(m_mass == 0.0){
        setInertia(cVector3d(0, 0, 0));
    }
    else{
        if(bodyInertia.IsDefined()){
            setInertia(cVector3d(bodyInertia["ix"].as<double>(), bodyInertia["iy"].as<double>(), bodyInertia["iz"].as<double>()));
            if (m_inertia.x() < 0.0 || m_inertia.y() < 0.0 || m_inertia.z() < 0.0 ){
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s intertia is \"" << &m_inertia << "\". Inertia cannot be negative, ignoring\n";
                return 0;
            }
        }
        else if (m_lowResMesh.m_meshes->size() > 0 || m_collisionGeometryType == GeometryType::shape){
            estimateInertia();
        }
    }

    buildDynamicModel();

    if(bodyPos.IsDefined()){
        assignXYZ( &bodyPos, &m_initialPos);
        setLocalPos(m_initialPos);
    }

    if(bodyRot.IsDefined()){
        double r = bodyRot["r"].as<double>();
        double p = bodyRot["p"].as<double>();
        double y = bodyRot["y"].as<double>();
        m_initialRot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(m_initialRot);
    }

    if (bodyLinDamping.IsDefined())
        m_surfaceProps.m_linear_damping = bodyLinDamping.as<double>();
    if (bodyAngDamping.IsDefined())
        m_surfaceProps.m_angular_damping = bodyAngDamping.as<double>();
    if (bodyStaticFriction.IsDefined())
        m_surfaceProps.m_static_friction = bodyStaticFriction.as<double>();
    if (bodyRollingFriction.IsDefined())
        m_surfaceProps.m_rolling_friction = bodyRollingFriction.as<double>();
    if (bodyRestitution.IsDefined())
        m_surfaceProps.m_restitution = bodyRestitution.as<double>();

    if (bodyPublishChildrenNames.IsDefined()){
        _publish_children_names = bodyPublishChildrenNames.as<bool>();
    }

    if (bodyPublishJointPositions.IsDefined()){
        _publish_joint_positions = bodyPublishJointPositions.as<bool>();
    }

    if (bodyPublishFrequency.IsDefined()){
        _min_publish_frequency = bodyPublishFrequency["low"].as<int>();
        _max_publish_frequency = bodyPublishFrequency["high"].as<int>();
    }
    else{
        // Set min to 50 Hz and max to 2000 Hz
        _min_publish_frequency = 50;
        _max_publish_frequency = 2000;
    }

    // The collision groups are sorted by integer indices. A group is an array of
    // ridig bodies that collide with each other. The bodies in one group
    // are not meant to collide with bodies from another group. Lastly
    // the a body can be a part of multiple groups

    if (bodyCollisionGroups.IsDefined()){
        for (int gIdx = 0 ; gIdx < bodyCollisionGroups.size() ; gIdx++){
            int gNum = bodyCollisionGroups[gIdx].as<int>();
            // Sanity check for the group number
            if (gNum >= 0 && gNum <= 999){
                mB->m_collisionGroups[gNum].push_back(this);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s group number is \"" << gNum << "\" which should be between [0 - 999], ignoring\n";
            }
        }
    }

    setConfigProperties(this, &m_surfaceProps);
    mB->m_chaiWorld->addChild(this);
    return true;
}

///
/// \brief afRigidBody::computeInertialOffset
/// \param mesh
/// \return
///
btVector3 afRigidBody::computeInertialOffset(cMesh* mesh){
    cVector3d intertialOffset(0, 0, 0);
    cVector3d vPos;
    // Sanity Check
    if (mesh){
        int nvertices = mesh->getNumVertices();
        int i;
        double idx;
        for (i = 0, idx = 0 ; i < nvertices ; i++, idx++){
            vPos = mesh->m_vertices->getLocalPos(i);
            intertialOffset = ((( idx ) / ( idx + 1.0 )) * intertialOffset) + (( 1.0 / ( idx + 1.0 )) * vPos);
        }
    }
    return btVector3(intertialOffset.x(), intertialOffset.y(), intertialOffset.z());
}

///
/// \brief afBody::compute_gains
///
void afRigidBody::computeControllerGains(){
    if (_lin_gains_computed && _ang_gains_computed){
        return;
    }

    double lumped_mass = m_mass;
    cVector3d lumped_intertia = m_inertia;
    for(m_bodyIt = m_childrenBodies.begin() ; m_bodyIt != m_childrenBodies.end() ; ++m_bodyIt){
        lumped_mass += (*m_bodyIt)->getMass();
        lumped_intertia += (*m_bodyIt)->getInertia();
    }
    if (!_lin_gains_computed){
        K_lin = lumped_mass * 20;
        D_lin = K_lin / 10;
        _lin_gains_computed = true;
    }
    if (!_ang_gains_computed){
        K_ang = lumped_mass * 10;
        D_ang = K_ang / 2;
        _ang_gains_computed = true;
    }
}

///
/// \brief afBody::set_surface_properties
/// \param a_body
/// \param a_props
///
void afRigidBody::setConfigProperties(const afRigidBodyPtr a_body, const afRigidBodySurfacePropertiesPtr a_props){
    a_body->m_bulletRigidBody->setFriction(a_props->m_static_friction);
    a_body->m_bulletRigidBody->setDamping(a_props->m_linear_damping, a_props->m_angular_damping);
    a_body->m_bulletRigidBody->setRollingFriction(a_props->m_rolling_friction);
    a_body->m_bulletRigidBody->setRestitution(a_props->m_restitution);
}

///
/// \brief afRigidBody::updatePositionFromDynamics
///
void afRigidBody::updatePositionFromDynamics()
{
    if (m_bulletRigidBody)
    {
        // get transformation matrix of object
        btTransform trans;
        m_bulletRigidBody->getMotionState()->getWorldTransform(trans);
        trans *= m_inertialOffsetTransform.inverse();

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

    // update Transform data for m_ObjectPtr
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afObjectPtr.get() != nullptr){
        m_afObjectPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
        m_afObjectPtr->cur_orientation(q.x, q.y, q.z, q.w);
        if (_publish_joint_positions){
            afObjectSetJointPositions();
        }
        if (_publish_children_names){
            // Since children names aren't going to change that often
            // change the field less so often
            if (m_write_count % 2000 == 0){
                afObjectStateSetChildrenNames();
                m_write_count = 0;
            }
        }
        if (_publish_joint_names){
            // Since joint names aren't going to change that often
            // change the field less so often
            if (m_write_count % 2000 == 0){
                afObjectStateSetJointNames();
            }
        }
        m_write_count++;
    }
#endif
}

///
/// \brief afRigidBody::afCommandExecute
/// \param dt
///
void afRigidBody::afObjectCommandExecute(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afObjectPtr.get() != nullptr){
        m_afObjectPtr->update_af_cmd();
        cVector3d force, torque;
        ObjectCommand m_afCommand = m_afObjectPtr->m_objectCommand;
        m_af_enable_position_controller = m_afCommand.enable_position_controller;
        if (m_afCommand.enable_position_controller){
            computeControllerGains();
            cVector3d cur_pos, cmd_pos, rot_axis, rot_axix_w_gain;
            cQuaternion cur_rot, cmd_rot;
            cMatrix3d cur_rot_mat, cmd_rot_mat;
            btTransform b_trans;
            double rot_angle;
            m_bulletRigidBody->getMotionState()->getWorldTransform(b_trans);
            cur_pos.set(b_trans.getOrigin().getX(),
                        b_trans.getOrigin().getY(),
                        b_trans.getOrigin().getZ());

            cur_rot.x = b_trans.getRotation().getX();
            cur_rot.y = b_trans.getRotation().getY();
            cur_rot.z = b_trans.getRotation().getZ();
            cur_rot.w = b_trans.getRotation().getW();
            cur_rot.toRotMat(cur_rot_mat);

            cmd_pos.set(m_afCommand.px, m_afCommand.py, m_afCommand.pz);

            cmd_rot.x = m_afCommand.qx;
            cmd_rot.y = m_afCommand.qy;
            cmd_rot.z = m_afCommand.qz;
            cmd_rot.w = m_afCommand.qw;
            cmd_rot.toRotMat(cmd_rot_mat);

            m_dpos_prev = m_dpos;
            m_dpos = cmd_pos - cur_pos;
            m_ddpos = (m_dpos - m_dpos_prev)/dt;
            m_drot_prev = m_drot;
            m_drot = cMul(cTranspose(cur_rot_mat), cmd_rot_mat);
            m_drot.toAxisAngle(rot_axis, rot_angle);

            force = K_lin * m_dpos + D_lin * m_ddpos;
            torque = cMul(K_ang * rot_angle, rot_axis);
            cur_rot_mat.mul(torque);
        }
        else{
            force.set(m_afCommand.Fx, m_afCommand.Fy, m_afCommand.Fz);
            torque.set(m_afCommand.Nx, m_afCommand.Ny, m_afCommand.Nz);
        }
        addExternalForce(force);
        addExternalTorque(torque);
        size_t jntCmdSize = m_afCommand.size_J_cmd;
        if (jntCmdSize > 0){
            size_t jntCmdCnt = m_joints.size() < jntCmdSize ? m_joints.size() : jntCmdSize;
            // If the enable position controllers flag is set, run
            // position control on all joints
            if (m_afCommand.enable_position_controller){
                for (size_t jnt = 0 ; jnt < jntCmdCnt ; jnt++){
                    m_joints[jnt]->commandPosition(m_afCommand.J_cmd[jnt]);
                }
            }
            // Otherwise, read the pos controller mask and set the joints with 1
            // in the mask to pos ctrl and the others to effort control
            else{
                size_t jntMaskSize = m_afCommand.position_controller_mask.size();
                for (size_t jnt = 0 ; jnt < jntCmdCnt ; jnt++){
                    // The size of pos ctrl mask can be less than the num of joint commands
                    // keep this in check and still read the mask to apply it. Run
                    // effort control on the masks not specified
                    if (jnt < jntMaskSize){
                        if (m_afCommand.position_controller_mask[jnt] == true){
                            m_joints[jnt]->commandPosition(m_afCommand.J_cmd[jnt]);
                        }
                        else{
                            m_joints[jnt]->commandEffort(m_afCommand.J_cmd[jnt]);
                        }
                    }
                    else{
                        m_joints[jnt]->commandEffort(m_afCommand.J_cmd[jnt]);
                    }
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
    int num_children = m_childrenBodies.size();
    if (num_children > 0 && m_afObjectPtr != NULL){
        std::vector<std::string> children_names;

        children_names.resize(num_children);
        for (size_t i = 0 ; i < num_children ; i++){
            children_names[i] = m_childrenBodies[i]->m_name;
        }
        m_afObjectPtr->set_children_names(children_names);
    }
#endif
}

///
/// \brief afRigidBody::afObjectStateSetJointNames
///
void afRigidBody::afObjectStateSetJointNames(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_joints = m_joints.size();
    if (num_joints > 0 && m_afObjectPtr != NULL){
        std::vector<std::string> joint_names;
        joint_names.resize(num_joints);
        for (size_t i = 0 ; i < num_joints ; i++){
            joint_names[i] = m_joints[i]->m_name;
        }
        m_afObjectPtr->set_joint_names(joint_names);
    }
#endif
}

///
/// \brief afRigidBody::afObjectSetJointPositions
///
void afRigidBody::afObjectSetJointPositions(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    int num_jnts = m_joints.size();
    if (num_jnts > 0 && m_afObjectPtr != NULL){
        if(m_joint_positions.size() != num_jnts){
            m_joint_positions.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_joint_positions[i] = m_joints[i]->getPosition();
        }
        m_afObjectPtr->set_joint_positions(m_joint_positions);
    }
#endif
}

///
/// \brief afRigidBody::setAngle
/// \param angle
///
void afRigidBody::setAngle(double &angle){
    if (m_parentBodies.size() == 0){
        for (size_t jnt = 0 ; jnt < m_joints.size() ; jnt++){
            m_joints[jnt]->commandPosition(angle);
        }

    }
}

///
/// \brief afRigidBody::setAngle
/// \param angles
///
void afRigidBody::setAngle(std::vector<double> &angles){
    if (m_parentBodies.size() == 0){
        double jntCmdSize = m_joints.size() < angles.size() ? m_joints.size() : angles.size();
        for (size_t jnt = 0 ; jnt < jntCmdSize ; jnt++){
            m_joints[jnt]->commandPosition(angles[jnt]);
        }

    }
}

afRigidBody::~afRigidBody(){
    int numConstraints = m_bulletRigidBody->getNumConstraintRefs();
    for (int i = numConstraints - 1 ; i >=0 ; i--){
        btTypedConstraint * tConstraint = m_bulletRigidBody->getConstraintRef(i);
        m_bulletRigidBody->removeConstraintRef(tConstraint);
    }
}

///
/// \brief afSoftBody::afSoftBody
/// \param a_chaiWorld
///
afSoftBody::afSoftBody(cBulletWorld *a_chaiWorld): afSoftMultiMesh(a_chaiWorld){

}

///
/// \brief afSoftBody::load
/// \param file
/// \param name
/// \param mB
/// \return
///
bool afSoftBody::loadSoftBody(std::string sb_config_file, std::string node_name, afMultiBodyPtr mB) {
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(sb_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << sb_config_file << std::endl;
        return 0;
    }

    YAML::Node softBodyNode = baseNode[node_name];
    return loadSoftBody(&softBodyNode, node_name, mB);
}

///
/// \brief afSoftBody::loadSofyBody
/// \param sb_config_data
/// \param name
/// \param mB
/// \return
///
bool afSoftBody::loadSoftBody(YAML::Node* sb_node, std::string node_name, afMultiBodyPtr mB) {
    YAML::Node softBodyNode = *sb_node;
    if (softBodyNode.IsNull()){
        std::cerr << "ERROR: SOFT BODY'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node softBodyName = softBodyNode["name"];
    YAML::Node softBodyMesh = softBodyNode["mesh"];
    YAML::Node softBodyScale = softBodyNode["scale"];
    YAML::Node softBodyInertialOffsetPos = softBodyNode["inertial offset"]["position"];
    YAML::Node softBodyInertialOffsetRot = softBodyNode["inertial offset"]["orientation"];
    YAML::Node softBodyMeshPathHR = softBodyNode["high resolution path"];
    YAML::Node softBodyMeshPathLR = softBodyNode["low resolution path"];
    YAML::Node softBodyNameSpace = softBodyNode["namespace"];
    YAML::Node softBodyMass = softBodyNode["mass"];
    YAML::Node softBodyLinGain = softBodyNode["linear gain"];
    YAML::Node softBodyAngGain = softBodyNode["angular gain"];
    YAML::Node softBodyPos = softBodyNode["location"]["position"];
    YAML::Node softBodyRot = softBodyNode["location"]["orientation"];
    YAML::Node softBodyColorRaw = softBodyNode["color raw"];
    YAML::Node softBodyColor = softBodyNode["color"];
    YAML::Node softBodyConfigData = softBodyNode["config"];
    YAML::Node softBodyRandomizeConstraints = softBodyNode["randomize constraints"];

    YAML::Node cfg_kLST = softBodyConfigData["kLST"];
    YAML::Node cfg_kVCF = softBodyConfigData["kVCF"];
    YAML::Node cfg_kDP = softBodyConfigData["kDP"];
    YAML::Node cfg_kDG = softBodyConfigData["kDG"];
    YAML::Node cfg_kLF = softBodyConfigData["kLF"];
    YAML::Node cfg_kPR = softBodyConfigData["kPR"];
    YAML::Node cfg_kVC = softBodyConfigData["kVC"];
    YAML::Node cfg_kDF = softBodyConfigData["kDF"];
    YAML::Node cfg_kMT = softBodyConfigData["kMT"];
    YAML::Node cfg_kCHR = softBodyConfigData["kCHR"];
    YAML::Node cfg_kKHR = softBodyConfigData["kKHR"];
    YAML::Node cfg_kSHR = softBodyConfigData["kSHR"];
    YAML::Node cfg_kAHR = softBodyConfigData["kAHR"];
    YAML::Node cfg_kSRHR_CL = softBodyConfigData["kSRHR_CL"];
    YAML::Node cfg_kSKHR_CL = softBodyConfigData["kSKHR_CL"];
    YAML::Node cfg_kSSHR_CL = softBodyConfigData["kSSHR_CL"];
    YAML::Node cfg_kSR_SPLT_CL = softBodyConfigData["kSR_SPLT_CL"];
    YAML::Node cfg_kSK_SPLT_CL = softBodyConfigData["kSK_SPLT_CL"];
    YAML::Node cfg_kSS_SPLT_CL = softBodyConfigData["kSS_SPLT_CL"];
    YAML::Node cfg_maxvolume = softBodyConfigData["maxvolume"];
    YAML::Node cfg_timescale = softBodyConfigData["timescale"];
    YAML::Node cfg_viterations = softBodyConfigData["viterations"];
    YAML::Node cfg_piterations = softBodyConfigData["piterations"];
    YAML::Node cfg_diterations = softBodyConfigData["diterations"];
    YAML::Node cfg_citerations = softBodyConfigData["citerations"];
    YAML::Node cfg_collisions = softBodyConfigData["collisions"];

    if(softBodyName.IsDefined()){
        m_name = softBodyName.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    if(softBodyMesh.IsDefined())
        m_mesh_name = softBodyMesh.as<std::string>();

    if(softBodyScale.IsDefined())
        m_scale = softBodyScale.as<double>();

    if(softBodyInertialOffsetPos.IsDefined()){
        btTransform trans;
        btQuaternion quat;
        btVector3 pos;
        quat.setEulerZYX(0,0,0);
        pos.setValue(0,0,0);
        if(softBodyInertialOffsetRot.IsDefined()){
            double r = softBodyInertialOffsetRot["r"].as<double>();
            double p = softBodyInertialOffsetRot["p"].as<double>();
            double y = softBodyInertialOffsetRot["y"].as<double>();
            quat.setEulerZYX(y, p, r);
        }
        assignXYZ( &softBodyInertialOffsetPos, &pos);
        trans.setRotation(quat);
        trans.setOrigin(pos);
        setInertialOffsetTransform(trans);
    }

    boost::filesystem::path high_res_filepath;
    boost::filesystem::path low_res_filepath;
    if (softBodyMeshPathHR.IsDefined()){
        high_res_filepath = softBodyMeshPathHR.as<std::string>() + m_mesh_name;
        if (high_res_filepath.is_relative()){
            high_res_filepath =  mB->getMultiBodyPath() + '/' + high_res_filepath.c_str();
        }
    }
    else{
        high_res_filepath = mB->getHighResMeshesPath() + m_mesh_name;
    }
    if (softBodyMeshPathLR.IsDefined()){
        low_res_filepath = softBodyMeshPathLR.as<std::string>() + m_mesh_name;
        if (low_res_filepath.is_relative()){
            low_res_filepath = mB->getMultiBodyPath() + '/' + low_res_filepath.c_str();
        }
    }
    else{
        low_res_filepath = mB->getLowResMeshesPath() + m_mesh_name;
    }

    loadFromFile(high_res_filepath.c_str());
    m_lowResMesh.loadFromFile(low_res_filepath.c_str());
    scale(m_scale);
    m_lowResMesh.scale(m_scale);
    buildContactTriangles(0.001, &m_lowResMesh);

    if(softBodyMass.IsDefined()){
        m_mass = softBodyMass.as<double>();
        if(softBodyLinGain.IsDefined()){
            K_lin = softBodyLinGain["P"].as<double>();
            D_lin = softBodyLinGain["D"].as<double>();
            _lin_gains_computed = true;
        }
        if(softBodyAngGain.IsDefined()){
            K_ang = softBodyAngGain["P"].as<double>();
            D_ang = softBodyAngGain["D"].as<double>();
            _ang_gains_computed = true;
        }
    }

    buildDynamicModel();

    if(softBodyPos.IsDefined()){
        assignXYZ( &softBodyPos, &pos);
        setLocalPos(pos);
    }

    if(softBodyRot.IsDefined()){
        double r = softBodyRot["r"].as<double>();
        double p = softBodyRot["p"].as<double>();
        double y = softBodyRot["y"].as<double>();
        rot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(rot);
    }

    if(softBodyColorRaw.IsDefined()){
        m_mat.setColorf(softBodyColorRaw["r"].as<float>(),
                softBodyColorRaw["g"].as<float>(),
                softBodyColorRaw["b"].as<float>(),
                softBodyColorRaw["a"].as<float>());
    }
    else if(softBodyColor.IsDefined()){
        std::vector<double> rgba = mB->getColorRGBA(softBodyColor.as<std::string>());
        m_mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);

    }

    if (softBodyConfigData.IsNull()){
        printf("Warning, no soft body config properties defined");
    }
    else{
        if (cfg_kLST.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kLST = cfg_kLST.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kLST = cfg_kLST.as<double>();
        }
        if (cfg_kVCF.IsDefined()) m_bulletSoftBody->m_cfg.kVCF = cfg_kVCF.as<double>();
        if (cfg_kDP.IsDefined()) m_bulletSoftBody->m_cfg.kDP = cfg_kDP.as<double>();
        if (cfg_kDG.IsDefined()) m_bulletSoftBody->m_cfg.kDG = cfg_kDG.as<double>();
        if (cfg_kLF.IsDefined()) m_bulletSoftBody->m_cfg.kLF = cfg_kLF.as<double>();
        if (cfg_kPR.IsDefined()) m_bulletSoftBody->m_cfg.kPR = cfg_kPR.as<double>();
        if (cfg_kVC.IsDefined()) m_bulletSoftBody->m_cfg.kVC = cfg_kVC.as<double>();
        if (cfg_kDF.IsDefined()) m_bulletSoftBody->m_cfg.kDF = cfg_kDF.as<double>();
        if (cfg_kMT.IsDefined()) m_bulletSoftBody->m_cfg.kMT = cfg_kMT.as<double>();
        if (cfg_kCHR.IsDefined()) m_bulletSoftBody->m_cfg.kCHR = cfg_kCHR.as<double>();
        if (cfg_kKHR.IsDefined()) m_bulletSoftBody->m_cfg.kKHR = cfg_kKHR.as<double>();
        if (cfg_kSHR.IsDefined()) m_bulletSoftBody->m_cfg.kSHR = cfg_kSHR.as<double>();
        if (cfg_kAHR.IsDefined()) m_bulletSoftBody->m_cfg.kAHR = cfg_kAHR.as<double>();
        if (cfg_kSRHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSRHR_CL = cfg_kSRHR_CL.as<double>();
        if (cfg_kSKHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSKHR_CL = cfg_kSKHR_CL.as<double>();
        if (cfg_kSSHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSSHR_CL = cfg_kSSHR_CL.as<double>();
        if (cfg_kSR_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSR_SPLT_CL = cfg_kSR_SPLT_CL.as<double>();
        if (cfg_kSK_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSK_SPLT_CL = cfg_kSK_SPLT_CL.as<double>();
        if (cfg_kSS_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSS_SPLT_CL = cfg_kSS_SPLT_CL.as<double>();
        if (cfg_maxvolume.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_maxvolume.as<double>();
        if (cfg_timescale.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_timescale.as<double>();
        if (cfg_viterations.IsDefined()) m_bulletSoftBody->m_cfg.viterations = cfg_viterations.as<double>();
        if (cfg_piterations.IsDefined()) m_bulletSoftBody->m_cfg.piterations = cfg_piterations.as<double>();
        if (cfg_diterations.IsDefined()) m_bulletSoftBody->m_cfg.diterations = cfg_diterations.as<double>();
        if (cfg_citerations.IsDefined()) m_bulletSoftBody->m_cfg.citerations = cfg_citerations.as<double>();
        if (cfg_collisions.IsDefined()) m_bulletSoftBody->m_cfg.collisions = cfg_collisions.as<double>();
    }

    if (softBodyRandomizeConstraints.IsDefined())
        if (softBodyRandomizeConstraints.as<bool>() == true)
            m_bulletSoftBody->randomizeConstraints();


    setMaterial(m_mat);
    //    setConfigProperties(this, &m_bulletSoftBody->m_cfg);
    mB->m_chaiWorld->addChild(this);
    return true;
}


///
/// \brief afSoftBody::setConfigProperties
/// \param a_body
/// \param a_configProps
///
void afSoftBody::setConfigProperties(const afSoftBodyPtr a_body, const afSoftBodyConfigPropertiesPtr a_configProps){

}


///
/// \brief afController::computeOutput
/// \param process_val
/// \param set_point
/// \param current_time
/// \return
///
double afController::computeOutput(double process_val, double set_point, double current_time){
        for (size_t i = n-1 ; i >= 1 ; i--){
            t[i] = t[i-1];
            e[i] = e[i-1];
            de[i] = de[i-1];
        }
        t[0] = current_time;
        e[0] = set_point - process_val;
        double dt = t[0] - t[1];
        if (!dt > 0.0001 || !dt > 0.0){
            dt = 0.0001;
        }
        de[0] = de[0] + ( (de[0] - de[1]) / dt );
        dde[0] = (e[0] - e[1]) / dt;
        output = (P * e[0]) + (I * de[0]) + (D * dde[0]);
//        boundImpulse(output);
//        std::cerr << "Output " << output << std::endl ;
        return output;
}


///
/// \brief afController::boundImpulse
/// \param effort_cmd
/// \param effort_time
///
void afController::boundImpulse(double &effort_cmd){
    double impulse = ( effort_cmd - m_last_cmd ) / (t[0]- t[1]);
//    std::cerr << "Before " << effort_cmd ;
    int sign = 1;
    if (impulse > max_impulse){
        if (impulse < 0){
            sign = -1;
        }
        effort_cmd = m_last_cmd + (sign * max_impulse * (t[0]- t[1]));
    }
//    std::cerr << " - After " << effort_cmd << " Impulse: " << max_impulse << std::endl ;
    m_last_cmd = effort_cmd;
}


///
/// \brief afJoint::afJoint
///
afJoint::afJoint(){
}

///
/// \brief afJoint::print_vec
/// \param name
/// \param v
///
void afJoint::printVec(std::string name, btVector3* v){
    printf("\t -%s: \n "
           "\t\t px = %f \n "
           "\t\t py = %f \n "
           "\t\t pz = %f \n",
           name.c_str(), v->x(), v->y(), v->z());
}


///
/// \brief afJoint::load
/// \param file
/// \param name
/// \param mB
/// \param name_remapping
/// \return
///
bool afJoint::loadJoint(std::string jnt_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
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
    return loadJoint(&baseJointNode, node_name, mB, name_remapping);
}

///
/// \brief afJoint::loadJoint
/// \param node
/// \param node_name
/// \param mB
/// \param name_remapping
/// \return
///
bool afJoint::loadJoint(YAML::Node* jnt_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node jointNode = *jnt_node;
    if (jointNode.IsNull()){
        std::cerr << "ERROR: JOINT'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }
    // Declare all the yaml parameters that we want to look for
    YAML::Node jointParentName = jointNode["parent"];
    YAML::Node jointChildName = jointNode["child"];
    YAML::Node jointName = jointNode["name"];
    YAML::Node jointParentPivot = jointNode["parent pivot"];
    YAML::Node jointChildPivot = jointNode["child pivot"];
    YAML::Node jointParentAxis = jointNode["parent axis"];
    YAML::Node jointChildAxis = jointNode["child axis"];
    YAML::Node jointOrigin = jointNode["origin"];
    YAML::Node jointAxis = jointNode["axis"];
    YAML::Node jointEnableMotor = jointNode["enable motor"];
    YAML::Node jointMaxMotorImpulse = jointNode["max motor impulse"];
    YAML::Node jointLimits = jointNode["joint limits"];
    YAML::Node jointOffset = jointNode["offset"];
    YAML::Node jointDamping = jointNode["joint damping"];
    YAML::Node jointType = jointNode["type"];
    YAML::Node jointController = jointNode["controller"];

    if (!jointParentName.IsDefined() || !jointChildName.IsDefined()){
        std::cerr << "ERROR: PARENT/CHILD FOR: " << node_name << " NOT DEFINED \n";
        return false;
    }
    m_name = jointName.as<std::string>();
    m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    m_parent_name = jointParentName.as<std::string>();
    m_child_name = jointChildName.as<std::string>();
    // Joint Transform in Parent
    btTransform T_j_p;
    // Joint Axis
    btVector3 joint_axis(0,0,1);
    m_enable_actuator = true;
    m_controller.max_impulse = 10; // max rate of change of effort on Position Controllers
    m_joint_offset = 0.0;
    m_lower_limit = -100;
    m_higher_limit = 100;
    //Default joint type is revolute if not type is specified
    m_jointType = JointType::revolute;

    afRigidBodyPtr afBodyA, afBodyB;

    m_mB = mB;

    afBodyA = mB->getRidigBody(mB->getNameSpace() + m_parent_name + name_remapping, true);
    afBodyB = mB->getRidigBody(mB->getNameSpace() + m_child_name + name_remapping, true);

    // If we couldn't find the body with name_remapping, it might have been
    // Defined in another ambf file. Search without name_remapping string
    if(afBodyA == NULL){
        afBodyA = mB->getRidigBody(m_parent_name, true);
        // If any body is still not found, print error and ignore joint
        if (afBodyA == NULL){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" PARENT BODY \"" << m_parent_name <<
                        "\" NOT FOUND" << std::endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ((!strcmp(afBodyA->m_name.c_str(), "world") == 0)
                &&(!strcmp(afBodyA->m_name.c_str(), "World") == 0)
                &&(!strcmp(afBodyA->m_name.c_str(), "WORLD") == 0)){
//            std::cerr <<"INFO: JOINT: \"" << m_name <<
//                        "\'s\" PARENT BODY \"" << m_parent_name <<
//                        "\" FOUND IN ANOTHER AMBF CONFIG," << std::endl;
        }
    }
    if(afBodyB == NULL){
        afBodyB = mB->getRidigBody(m_child_name, true);
        // If any body is still not found, print error and ignore joint
        if (afBodyB == NULL){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_child_name <<
                        "\" NOT FOUND" << std::endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ((!strcmp(afBodyB->m_name.c_str(), "world") == 0)
                &&(!strcmp(afBodyB->m_name.c_str(), "World") == 0)
                &&(!strcmp(afBodyB->m_name.c_str(), "WORLD") == 0)){
            std::cerr <<"INFO: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_child_name <<
                        "\" FOUND IN ANOTHER AMBF CONFIG," << std::endl;
        }
    }

    else{
        m_rbodyA = afBodyA->m_bulletRigidBody;
        m_rbodyB = afBodyB->m_bulletRigidBody;
    }

    if (jointParentPivot.IsDefined() & jointParentAxis.IsDefined() & jointChildPivot.IsDefined() & jointChildAxis.IsDefined()){
        assignXYZ( &jointParentPivot, &m_pvtA);
        assignXYZ( &jointParentAxis, &m_axisA);
        assignXYZ( &jointChildPivot, &m_pvtB);
        assignXYZ( &jointChildAxis, &m_axisB);

        if (m_axisA.length() < 0.9 || m_axisA.length() > 1.1 ){
            std::cerr << "WARNING: Joint " << m_name << "'s parent axis is not normalized\n";
            // If length is > 1, we can normalize it, no big deal
            if (m_axisA.length() > 1.1){
                m_axisA.normalize();
            }
            // However if the length is <0, there is something wrong, just ignore
            else{
                std::cerr << "Ignoring \n";
                return 0;
            }
        }
        if (m_axisB.length() < 0.9 || m_axisB.length() > 1.1 ){
            std::cerr << "WARNING: Joint " << m_name << "'s child axis is not normalized\n";
            // If length is > 1, we can normalize it, no big deal
            if (m_axisB.length() > 1.1){
                m_axisB.normalize();
            }
            // However if the length is <0, there is something wrong, just ignore
            else{
                std::cerr << "Ignoring \n";
                return 0;
            }
        }

        // Scale the pivot before transforming as the default scale methods don't move this pivot
        m_pvtA *= afBodyA->m_scale;
        m_pvtA = afBodyA->getInertialOffsetTransform().inverse() * m_pvtA;
        m_pvtB = afBodyB->getInertialOffsetTransform().inverse() * m_pvtB;
        m_axisA = afBodyA->getInertialOffsetTransform().getBasis().inverse() * m_axisA;
        m_axisB = afBodyB->getInertialOffsetTransform().getBasis().inverse() * m_axisB;
    }
    else if(jointOrigin.IsDefined()){
        btQuaternion quat;
        btVector3 pos;
        YAML::Node jointXYZ = jointOrigin['position'];
        YAML::Node jointRPY = jointOrigin['orientation'];
        if (jointXYZ.IsDefined()){
            assignXYZ(&jointXYZ, &pos);
            T_j_p.setOrigin(pos);
        }
        if (jointRPY.IsDefined()){
            quat.setEulerZYX(jointRPY['y'].as<double>(),
                    jointRPY['p'].as<double>(),
                    jointRPY['r'].as<double>());
            T_j_p.setRotation(quat);
        }

        if (jointAxis.IsDefined()){
            assignXYZ(&jointAxis, &joint_axis);
        }
    }
    else{
        std::cerr << "ERROR: JOINT CONFIGURATION FOR: " << node_name << " NOT DEFINED \n";
        return false;
    }

    if(jointOffset.IsDefined()){
        m_joint_offset = jointOffset.as<double>();
    }

    if(jointLimits.IsDefined()){
        if (jointLimits["low"].IsDefined())
            m_lower_limit = jointLimits["low"].as<double>();
        if (jointLimits["high"].IsDefined())
            m_higher_limit = jointLimits["high"].as<double>();
    }

    if (jointController.IsDefined()){
        if( (jointController["P"]).IsDefined())
            m_controller.P = jointController["P"].as<double>();
        if( (jointController["I"]).IsDefined())
            m_controller.I = jointController["I"].as<double>();
        if( (jointController["D"]).IsDefined())
            m_controller.D = jointController["D"].as<double>();
    }

    if (jointType.IsDefined()){
        if ((strcmp(jointType.as<std::string>().c_str(), "hinge") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "revolute") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "continuous") == 0)){
            m_jointType = JointType::revolute;
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "slider") == 0)
                 || (strcmp(jointType.as<std::string>().c_str(), "prismatic") == 0)){
            m_jointType = JointType::prismatic;
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "fixed") == 0)){
            m_jointType = JointType::fixed;
        }

    }
    if (m_jointType == JointType::revolute){
#ifdef USE_PIVOT_AXIS_METHOD
        m_btConstraint = new btHingeConstraint(*m_rbodyA, *m_rbodyB, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
#else
        btTransform frameA, frameB;
        frameA.setIdentity();
        frameB.setIdentity();

        // Bullet takes the x axis as the default for prismatic joints
        btVector3 nz(0,0,1);

        btQuaternion quat_nz_p;
        quat_nz_p = getRotationBetweenVectors(nz, m_axisA);
        frameA.setRotation(quat_nz_p);
        frameA.setOrigin(m_pvtA);

        btQuaternion quat_c_p;
        quat_c_p = getRotationBetweenVectors(m_axisB, m_axisA);
        btQuaternion offset_quat;
        offset_quat.setRotation(m_axisA, m_joint_offset);
        // We need to post-multiply frameA's rot to cancel out the shift in axis, then
        // the offset along joint axis and finally frameB's axis alignment in frameA.
        frameB.setRotation( quat_c_p.inverse() * offset_quat.inverse() * quat_nz_p);
        frameB.setOrigin(m_pvtB);

        m_hinge = new btHingeConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB, true);
        m_btConstraint = m_hinge;
#endif
        // Don't enable motor yet, only enable when set position is called
        // this keeps the joint behave freely when it's launched

        if(jointMaxMotorImpulse.IsDefined()){
            m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            ((btHingeConstraint*)m_btConstraint)->setMaxMotorImpulse(m_controller.max_impulse);
        }

        if(jointLimits.IsDefined()){
            ((btHingeConstraint*)m_btConstraint)->setLimit(m_lower_limit, m_higher_limit);
        }

        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
        afBodyA->addChildBody(afBodyB, this);
    }
    else if (m_jointType == JointType::prismatic){
        btTransform frameA, frameB;
        frameA.setIdentity();
        frameB.setIdentity();

        // Bullet takes the x axis as the default for prismatic joints
        btVector3 nx(1,0,0);

        btQuaternion quat_nx_p;
        quat_nx_p = getRotationBetweenVectors(nx, m_axisA);
        frameA.setRotation(quat_nx_p);
        frameA.setOrigin(m_pvtA);

        btQuaternion quat_c_p;
        quat_c_p = getRotationBetweenVectors(m_axisB, m_axisA);
        btQuaternion offset_quat;
        offset_quat.setRotation(m_axisA, m_joint_offset);
        // We need to post-multiply frameA's rot to cancel out the shift in axis, then
        // the offset along joint axis and finally frameB's axis alignment in frameA.
        frameB.setRotation( quat_c_p.inverse() * offset_quat.inverse() * quat_nx_p);
        frameB.setOrigin(m_pvtB);

        m_slider = new btSliderConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB, true);
        m_btConstraint = m_slider;

        if (jointEnableMotor.IsDefined()){
            m_enable_actuator = jointEnableMotor.as<int>();
            // Don't enable motor yet, only enable when set position is called
            if(jointMaxMotorImpulse.IsDefined()){
                m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            }
        }

        if(jointLimits.IsDefined()){
            ((btSliderConstraint*) m_btConstraint)->setLowerLinLimit(m_lower_limit);
            ((btSliderConstraint*) m_btConstraint)->setUpperLinLimit(m_higher_limit);
        }

        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
        afBodyA->addChildBody(afBodyB, this);
    }
    else if (m_jointType == JointType::fixed){
        btTransform frameA, frameB;
        frameA.setIdentity();
        frameB.setIdentity();

        frameA.setOrigin(m_pvtA);

        btQuaternion quat_c_p;
        quat_c_p = getRotationBetweenVectors(m_axisB, m_axisA);
        btQuaternion offset_quat;
        offset_quat.setRotation(m_axisA, m_joint_offset);
        // We need to post-multiply frameA's rot to cancel out the shift in axis, then
        // the offset along joint axis and finally frameB's axis alignment in frameA.
        frameB.setRotation( quat_c_p.inverse() * offset_quat.inverse());
        frameB.setOrigin(m_pvtB);
        m_btConstraint = new btFixedConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB);
        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
        afBodyA->addChildBody(afBodyB, this);
    }
    return true;
}


///
/// \brief afJoint::getRotationBetweenVectors
/// \param v1
/// \param v2
/// \return
///
btQuaternion afJoint::getRotationBetweenVectors(btVector3 &v1, btVector3 &v2){
    btQuaternion quat;
    double rot_angle = v1.angle(v2);
    if ( abs(rot_angle) < 0.1){
        quat.setEulerZYX(0,0,0);
    }
    else if ( abs(rot_angle) > 3.13 ){
        btVector3 nx(1, 0, 0);
        double temp_ang = v1.angle(nx);
        if ( abs(temp_ang) > 0.1 && abs(temp_ang) < 3.13 ){
            btVector3 rot_axis = v1.cross(nx);
            quat.setRotation(rot_axis, rot_angle);
        }
        else{
            btVector3 ny(0, 1, 0);
            btVector3 rot_axis = m_axisA.cross(ny);
            quat.setRotation(rot_axis, rot_angle);
        }
    }
    else{
        btVector3 rot_axis = v1.cross(v2);
        quat.setRotation(rot_axis, rot_angle);
    }

    return quat;
}

///
/// \brief afJoint::command_position
/// \param cmd
///
void afJoint::commandPosition(double &position_cmd){
    // The torque commands disable the motor, so double check and re-enable the motor
    // if it was set to be enabled in the first place
    if (m_enable_actuator){
        if (m_jointType == JointType::revolute){
            double effort_command = m_controller.computeOutput(m_hinge->getHingeAngle(), position_cmd, m_mB->m_wallClock.getCurrentTimeSeconds());
            btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
            btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
            m_btConstraint->getRigidBodyA().applyTorque(-hingeAxisInWorld * effort_command);
            m_btConstraint->getRigidBodyB().applyTorque(hingeAxisInWorld * effort_command);
        }
        else if(m_jointType == JointType::prismatic){
            double effort_command = m_controller.computeOutput(m_slider->getLinearPos(), position_cmd,  m_mB->m_wallClock.getCurrentTimeSeconds());
            btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
            const btVector3 sliderAxisInWorld = trA.getBasis()*m_axisA;
            const btVector3 relPos(0,0,0);
            m_rbodyA->applyForce(-sliderAxisInWorld * effort_command, relPos);
            m_rbodyB->applyForce(sliderAxisInWorld * effort_command, relPos);
        }
    }
    else{
        std::cerr << "WARNING, MOTOR NOT ENABLED FOR JOINT: " << m_name << std::endl;
    }
}

///
/// \brief afJoint::command_torque
/// \param cmd
///
void afJoint::commandEffort(double &cmd){
    if (m_jointType == JointType::revolute){
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
        m_btConstraint->getRigidBodyA().applyTorque(-hingeAxisInWorld * cmd);
        m_btConstraint->getRigidBodyB().applyTorque(hingeAxisInWorld * cmd);
    }
    else if (m_jointType == JointType::prismatic){
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        const btVector3 sliderAxisInWorld = trA.getBasis()*m_axisA;
        const btVector3 relPos(0,0,0);
        m_rbodyA->applyForce(-sliderAxisInWorld * cmd, relPos);
        m_rbodyB->applyForce(sliderAxisInWorld * cmd, relPos);
    }
}

double afJoint::getPosition(){
    if (m_jointType == JointType::revolute)
        return m_hinge->getHingeAngle();
    else if (m_jointType == JointType::prismatic)
        return m_slider->getLinearPos();
    else if (m_jointType == JointType::fixed)
        return 0;
}

///
/// \brief afJoint::~afJoint
///
afJoint::~afJoint(){
    delete m_btConstraint;
}

///
/// \brief afWorld::afWorld
/// \param a_chaiWorld
///
afWorld::afWorld(cBulletWorld* a_chaiWorld){
    m_chaiWorld = a_chaiWorld;
    m_encl_length = 4.0;
    m_encl_width = 4.0;
    m_encl_height = 3.0;
}

///
/// \brief afWorld::get_enclosure_length
/// \return
///
double afWorld::getEnclosureLength(){
    return m_encl_length;
}

///
/// \brief afWorld::get_enclosure_width
/// \return
///
double afWorld::getEnclosureWidth(){
    return m_encl_width;
}

///
/// \brief afWorld::get_enclosure_height
/// \return
///
double afWorld::getEnclosureHeight(){
    return m_encl_height;
}

///
/// \brief afWorld::get_enclosure_extents
/// \param length
/// \param width
/// \param height
///
void afWorld::getEnclosureExtents(double &length, double &width, double &height){
    length = m_encl_length;
    width = m_encl_width;
    height = m_encl_height;
}


///
/// \brief afWorld::load_world
/// \param a_world_config
/// \return
///
bool afWorld::loadWorld(std::string a_world_config){
    if (a_world_config.empty()){
        a_world_config = getWorldConfig();
    }
    YAML::Node worldNode;
    try{
        worldNode = YAML::LoadFile(a_world_config);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << a_world_config << std::endl;
        return 0;
    }

    YAML::Node worldEnclosureData = worldNode["enclosure"];
    YAML::Node worldLightsData = worldNode["lights"];
    YAML::Node worldCamerasData = worldNode["cameras"];

    if (worldEnclosureData.IsDefined()){
        m_encl_length = worldEnclosureData["length"].as<double>();
        m_encl_width =  worldEnclosureData["width"].as<double>();
        m_encl_height = worldEnclosureData["height"].as<double>();
    }

    if (worldLightsData.IsDefined()){
        size_t n_lights = worldLightsData.size();
        for (size_t idx = 0 ; idx < n_lights; idx++){
            std::string light_name = worldLightsData[idx].as<std::string>();
            afLightPtr lightPtr = new afLight();
            YAML::Node lightNode = worldNode[light_name];
            if (lightPtr->loadLight(&lightNode, light_name)){
                m_lights.push_back(lightPtr);
            }
        }
    }

    if (worldCamerasData.IsDefined()){
        size_t n_cameras = worldCamerasData.size();
        for (size_t idx = 0 ; idx < n_cameras; idx++){
            std::string camera_name = worldCamerasData[idx].as<std::string>();
            afCameraPtr cameraPtr = new afCamera();
            YAML::Node cameraNode = worldNode[camera_name];
            if (cameraPtr->loadCamera(&cameraNode, camera_name)){
                m_cameras.push_back(cameraPtr);
            }
        }
    }

    return true;

}

///
/// \brief afCamera::afCamera
///
afCamera::afCamera(){
}

///
/// \brief afCamera::loadCamera
/// \param camera_node
/// \param camera_name
/// \return
///
bool afCamera::loadCamera(YAML::Node* a_camera_node, std::string a_camera_name){
    YAML::Node cameraNode = *a_camera_node;
    m_name = a_camera_name;
    YAML::Node cameraLocationData = cameraNode["location"];
    YAML::Node cameraLookAtData = cameraNode["look at"];
    YAML::Node cameraUpData = cameraNode["up"];
    YAML::Node cameraClippingPlaneData = cameraNode["clipping plane"];
    YAML::Node cameraFieldViewAngleData = cameraNode["field view angle"];
    YAML::Node cameraOrthoWidthData = cameraNode["orthographic view width"];
    YAML::Node cameraControllingDevicesData = cameraNode["controlling devices"];

    bool _is_valid = true;

    if (cameraLocationData.IsDefined()){
        assignXYZ(&cameraLocationData, &m_location);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOCATION NOT DEFINED, IGNORING " << std::endl;
         _is_valid = false;
    }
    if (cameraLookAtData.IsDefined()){
        assignXYZ(&cameraLookAtData, &m_look_at);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOOK AT NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraUpData.IsDefined()){
        assignXYZ(&cameraUpData, &m_up);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA UP NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraClippingPlaneData.IsDefined()){
        m_clipping_plane_limits[0] = cameraClippingPlaneData["near"].as<double>();
        m_clipping_plane_limits[1] = cameraClippingPlaneData["far"].as<double>();
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA CLIPPING PLANE NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraFieldViewAngleData.IsDefined()){
        m_field_view_angle = cameraFieldViewAngleData.as<double>();
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA FIELD VIEW DATA NOT DEFINED, IGNORING " << std::endl;
        m_field_view_angle = 0.8;
    }
    if (cameraOrthoWidthData.IsDefined()){
        m_enable_ortho_view = true;
        m_ortho_view_width = cameraOrthoWidthData.as<double>();
    }
    else{
         m_enable_ortho_view = false;
    }
    if (cameraControllingDevicesData.IsDefined()){
        for(int idx = 0 ; idx < cameraControllingDevicesData.size() ; idx++){
            m_controlling_devices.push_back( cameraControllingDevicesData[idx].as<std::string>());
        }
    }

    return _is_valid;
}



///
/// \brief afLight::afLight
///
afLight::afLight(){
}

///
/// \brief afLight::loadLight
/// \param light_node
/// \return
///
bool afLight::loadLight(YAML::Node* a_light_node, std::string a_light_name){
    YAML::Node lightNode = *a_light_node;
    m_name = a_light_name;
    YAML::Node lightLocationData = lightNode["location"];
    YAML::Node lightDirectionData = lightNode["direction"];
    YAML::Node lightSpotExponentData = lightNode["spot exponent"];
    YAML::Node lightShadowQualityData = lightNode["shadow quality"];
    YAML::Node lightCuttOffAngleData = lightNode["cutoff angle"];

    bool _is_valid = true;

    if (lightLocationData.IsDefined()){
        assignXYZ(&lightLocationData, &m_location);
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT LOCATION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (lightDirectionData.IsDefined()){
        assignXYZ(&lightDirectionData, &m_direction);
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT DIRECTION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (lightSpotExponentData.IsDefined()){
        m_spot_exponent = lightSpotExponentData.as<double>();
    }
    if (lightShadowQualityData.IsDefined()){
        int shadow_quality = lightShadowQualityData.as<int>();
        if (shadow_quality < 0){
            shadow_quality = 0;
            std::cerr << "INFO: LIGHT \"" << a_light_name << "\" SHADOW QUALITY SHOULD BE BETWEEN [0-5] " << std::endl;
        }
        else if (shadow_quality > 5){
            shadow_quality = 5;
            std::cerr << "INFO: LIGHT \"" << a_light_name << "\" SHADOW QUALITY SHOULD BE BETWEEN [0-5] " << std::endl;
        }
        m_shadow_quality = (ShadowQuality)shadow_quality;
    }
    if (lightCuttOffAngleData.IsDefined()){
        m_cuttoff_angle = lightCuttOffAngleData.as<double>();
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT CUTOFF NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    return _is_valid;
}

///
/// \brief afMultiBody::afMultiBody
///
afMultiBody::afMultiBody(){
    m_wallClock.start(true);
}

afMultiBody::afMultiBody(cBulletWorld *a_chaiWorld){
    m_wallClock.start(true);
    m_chaiWorld = a_chaiWorld;
}


/// Help from: https://stackoverflow.com/questions/1489830/efficient-way-to-determine-number-of-digits-in-an-integer
/// and https://stackoverflow.com/questions/11151548/get-the-number-of-digits-in-an-int/11151594
///
///
/// \brief afMultiBody::remapName
/// \param name
/// \param remap_idx_str
///
void afMultiBody::remapName(std::string &name, std::string remap_idx_str){
    if (remap_idx_str.length() == 0){
        return;
    }
    int cur_idx = std::stoi(remap_idx_str);
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

template<typename T>
///
/// \brief afMultiBody::remapBodyName
/// \param a_body_name
/// \param tMap
/// \return
///
std::string afMultiBody::remapBodyName(std::string a_body_name, const T* tMap){
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

///
/// \brief afMultiBody::remapJointName
/// \param a_joint_name
/// \return
///
std::string afMultiBody::remapJointName(std::string a_joint_name){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (m_afJointMap.find(a_joint_name) == m_afJointMap.end()){
        return remap_string;
    }

    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(m_afJointMap.find(a_joint_name + remap_string) != m_afJointMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief afMultiBody::loadMultiBody
/// \param i
/// \return
///
bool afMultiBody::loadMultiBody(int i){
    std::string multibody_config = getMultiBodyConfig(i);
    return loadMultiBody(multibody_config);
}

///
/// \brief afMultiBody::loadAllMultiBodies
///
void afMultiBody::loadAllMultiBodies(){
    for (int i = 0 ; i < numMultiBodyConfig(); i++){
        loadMultiBody(i);
    }
}

///
/// \brief afMultiBody::loadMultiBody
/// \param a_multibody_config
/// \return
///
bool afMultiBody::loadMultiBody(std::string a_multibody_config_file){
    if (a_multibody_config_file.empty()){
        a_multibody_config_file = getMultiBodyConfig();
    }
    YAML::Node multiBodyNode;
    try{
        multiBodyNode = YAML::LoadFile(a_multibody_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << a_multibody_config_file << std::endl;
        return 0;
    }

    // Declare all the yaml parameters that we want to look for
    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodySoftBodies = multiBodyNode["soft bodies"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];

    boost::filesystem::path mb_cfg_dir = boost::filesystem::path(a_multibody_config_file).parent_path();
    m_multibody_path = mb_cfg_dir.c_str();

    /// Loading Rigid Bodies
    afRigidBodyPtr tmpRigidBody;
    boost::filesystem::path high_res_filepath;
    boost::filesystem::path low_res_filepath;
    if(multiBodyMeshPathHR.IsDefined() && multiBodyMeshPathLR.IsDefined()){
        high_res_filepath = multiBodyMeshPathHR.as<std::string>();
        low_res_filepath = multiBodyMeshPathLR.as<std::string>();

        if (high_res_filepath.is_relative()){
            high_res_filepath = mb_cfg_dir / high_res_filepath;
        }
        if (low_res_filepath.is_relative()){
            low_res_filepath = mb_cfg_dir / low_res_filepath;
        }
        m_multibody_high_res_meshes_path = high_res_filepath.c_str();
        m_multibody_low_res_meshes_path = low_res_filepath.c_str();
    }
    else{
        m_multibody_high_res_meshes_path = "../resources/models/puzzle/high_res/";
        m_multibody_low_res_meshes_path = "../resources/models/puzzle/low_res/";
    }
    if (multiBodyNameSpace.IsDefined()){
        m_multibody_namespace = multiBodyNameSpace.as<std::string>();
    }
    else{
        m_multibody_namespace = "/ambf/env/";
    }

    size_t totalRigidBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        tmpRigidBody = new afRigidBody(m_chaiWorld);
        std::string rb_name = multiBodyRidigBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(rb_name, &m_afRigidBodyMap);
        //        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        YAML::Node rb_node = multiBodyNode[rb_name];
        if (tmpRigidBody->loadRidigBody(&rb_node, rb_name, this)){
            m_afRigidBodyMap[(m_multibody_namespace + rb_name + remap_str).c_str()] = tmpRigidBody;
            std::string af_name = tmpRigidBody->m_name;
            if ((strcmp(af_name.c_str(), "world") == 0) ||
                    (strcmp(af_name.c_str(), "World") == 0) ||
                    (strcmp(af_name.c_str(), "WORLD") == 0)){
                continue;
            }
            else{
                tmpRigidBody->afObjectCreate(tmpRigidBody->m_name + remap_str,
                                             tmpRigidBody->m_body_namespace,
                                             tmpRigidBody->getMinPublishFrequency(),
                                             tmpRigidBody->getMaxPublishFrequency());
            }
        }
    }


    /// Loading Soft Bodies
    afSoftBodyPtr tmpSoftBody;
    size_t totalSoftBodies = multiBodySoftBodies.size();
    for (size_t i = 0; i < totalSoftBodies; ++i) {
        tmpSoftBody = new afSoftBody(m_chaiWorld);
        std::string sb_name = multiBodySoftBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(sb_name, &m_afSoftBodyMap);
        //        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        YAML::Node sb_node = multiBodyNode[sb_name];
        if (tmpSoftBody->loadSoftBody(&sb_node, sb_name, this)){
            m_afSoftBodyMap[(m_multibody_namespace + sb_name + remap_str).c_str()] = tmpSoftBody;
            //            tmpSoftBody->createAFObject(tmpSoftBody->m_name + remap_str);
        }
    }

    /// Loading Joints
    afJointPtr tmpJoint;
    size_t totalJoints = multiBodyJoints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint();
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
        std::string remap_str = remapJointName(jnt_name);
        //        printf("Loading body: %s \n", (jnt_name + remap_str).c_str());
        YAML::Node jnt_node = multiBodyNode[jnt_name];
        if (tmpJoint->loadJoint(&jnt_node, jnt_name, this, remap_str)){
            m_afJointMap[m_multibody_namespace + jnt_name+remap_str] = tmpJoint;
        }
    }

    // This flag would ignore collision for all the multibodies in the scene
    bool _ignore_inter_collision = false;
    if (multiBodyNode["ignore inter-collision"].IsDefined()){
        if (multiBodyNode["ignore inter-collision"].as<bool>()){
            ignoreCollisionChecking();
            _ignore_inter_collision = multiBodyNode["ignore inter-collision"].as<bool>();
        }
    }
    // If the ignore_inter_collision flag is false, then ignore collision based on collision
    // groups
    if (!_ignore_inter_collision){
        buildCollisionGroups();
    }

//    removeOverlappingCollisionChecking();

    return true;
}

///
/// \brief afMultiBody::removeCollisionChecking
///
void afMultiBody::ignoreCollisionChecking(){
    afRigidBodyMap::iterator rBodyItA = m_afRigidBodyMap.begin();
    std::vector<btRigidBody*> rBodiesVec;
    rBodiesVec.resize(m_afRigidBodyMap.size());
    int i=0;
    for ( ; rBodyItA != m_afRigidBodyMap.end() ; ++rBodyItA){
        rBodiesVec[i] = rBodyItA->second->m_bulletRigidBody;
        i++;
    }

    for (int i = 0 ; i < rBodiesVec.size() - 1 ; i++){
        for (int j = i+1 ; j < rBodiesVec.size() ; j++){
            rBodiesVec[i]->setIgnoreCollisionCheck(rBodiesVec[j], true);
        }
    }
}

///
/// \brief afMultiBody::buildCollisionGroups
///
void afMultiBody::buildCollisionGroups(){
   std::vector<int> groupNumbers;
   groupNumbers.resize(m_collisionGroups.size());

//   for (int aIdx = 0 ; aIdx < groupNumbers.size() ; aIdx++){
//       std::cerr << "****" << std::endl;
//        std::cerr << "Group " << aIdx << " = [" ;
//       std::vector<afRigidBodyPtr> grpA = m_collisionGroups[aIdx];
//       for(int aBodyIdx = 0 ; aBodyIdx < grpA.size() ; aBodyIdx++){
//           std::cerr << " " << grpA[aBodyIdx]->m_name << ",";
//       }
//       std::cerr << " ]" << std::endl;
//   }

   for (int aIdx = 0 ; aIdx < groupNumbers.size() - 1 ; aIdx++){
       std::vector<afRigidBodyPtr> grpA = m_collisionGroups[aIdx];
       for (int bIdx = aIdx + 1 ; bIdx < groupNumbers.size() ; bIdx ++){
           std::vector<afRigidBodyPtr> grpB = m_collisionGroups[bIdx];

           for(int aBodyIdx = 0 ; aBodyIdx < grpA.size() ; aBodyIdx++){
               afRigidBodyPtr bodyA = grpA[aBodyIdx];
               for(int bBodyIdx = 0 ; bBodyIdx < grpB.size() ; bBodyIdx++){
                   afRigidBodyPtr bodyB = grpB[bBodyIdx];
                   bodyA->m_bulletRigidBody->setIgnoreCollisionCheck(bodyB->m_bulletRigidBody, true);
               }
           }
       }
   }
}

///
/// \brief afMultiBody::removeOverlappingCollisionChecking
///
void afMultiBody::removeOverlappingCollisionChecking(){
    // This function checks all the constraints of each rigid body
    // if there are more than 1, it means that multiple bodies share each other
    // In this case, iteratively go over all the shared bodies and ignore their
    // collision if their common body has the same pivot
    afRigidBodyMap::iterator rBodyIt = m_afRigidBodyMap.begin();
    std::vector<btRigidBody*> bodyFamily;
    std::pair<btVector3, btRigidBody*> pvtAandConnectedBody;
    std::vector< std::pair<btVector3, btRigidBody*> > pvtAandConnectedBodyVec;
    for ( ; rBodyIt != m_afRigidBodyMap.end() ; ++rBodyIt){
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
            for (int pvtIdx1 = 0 ; pvtIdx1 < pvtAandConnectedBodyVec.size() - 1 ; pvtIdx1++ ){
                btVector3 pvtA1 = pvtAandConnectedBodyVec[pvtIdx1].first;
                btRigidBody* connectedBodyA1 = pvtAandConnectedBodyVec[pvtIdx1].second;
                for (int pvtIdx2 = pvtIdx1 + 1 ; pvtIdx2 < pvtAandConnectedBodyVec.size() ; pvtIdx2++ ){
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

/////
///// \brief afMultiBody::removeOverlappingCollisionChecking
/////
//void afMultiBody::removeOverlappingCollisionChecking(){
//    // This function checks all the constraints of each ridig body
//    // if there are more than 1, it means that multiple bodies share each other
//    // In this case, iteratively go over all the shared bodies and ignore their
//    // collision

//    std::vector<btRigidBody*> rigidBodiesA, rigidBodiesB;
//    std::vector<afJointPtr> afJoints;
//    int num_joints = m_afJointMap.size();
//    rigidBodiesA.resize(num_joints);
//    rigidBodiesB.resize(num_joints);
//    afJoints.resize(num_joints);
//    std::vector<bool> aBodiesChecked(num_joints, false);
//    std::vector<bool> bBodiesChecked(num_joints, false);

//    afJointMap::iterator jIt = m_afJointMap.begin();
//    for (int jIdx = 0; jIt != m_afJointMap.end() ; ++ jIt, jIdx++){
//        afJointPtr afJoint = jIt->second;
//        rigidBodiesA[jIdx] = &afJoint->m_hinge->getRigidBodyA();
//        rigidBodiesB[jIdx] = &afJoint->m_hinge->getRigidBodyB();
//        afJoints[jIdx] = afJoint;
//    }
//    std::pair<btVector3, btRigidBody*> pvtConnectedBodyPair;
//    std::vector< std::pair<btVector3, btRigidBody*> > pvtConnectedBodyPairs;
//    std::vector < std::vector< std::pair<btVector3, btRigidBody*> > > bodyPvtConnectedBodyPairs;

//    for (int a1 = 0 ; a1 < rigidBodiesA.size() ; a1 ++){
//        if (aBodiesChecked[a1] == false){
//            btRigidBody* rBodyA = rigidBodiesA[a1];
//            for (int a2 = a1 ; a2 < rigidBodiesA.size() ; a2 ++){
//                btRigidBody* rBodyB = rigidBodiesA[a2];
//                if (rBodyA == rBodyB){
//                    pvtConnectedBodyPair.first = afJoints[a1]->m_pvtA;
//                    pvtConnectedBodyPair.second = rigidBodiesB[a2];
//                    pvtConnectedBodyPairs.push_back(pvtConnectedBodyPair);
//                    aBodiesChecked[a2] = true;
//                }
//            }
//            for (int b1 = 0 ; b1 < rigidBodiesB.size() ; b1 ++){
//                if (bBodiesChecked[b1] == false){
//                    btRigidBody* rBodyB = rigidBodiesB[b1];
//                    if (rBodyA == rBodyB){
//                        pvtConnectedBodyPair.first = afJoints[b1]->m_pvtB;
//                        pvtConnectedBodyPair.second = rigidBodiesA[b1];
//                        pvtConnectedBodyPairs.push_back(pvtConnectedBodyPair);
//                        bBodiesChecked[b1] = true;
//                    }
//                }
//            }
//            bodyPvtConnectedBodyPairs.push_back(pvtConnectedBodyPairs);
//        }
//    }

//}

///
/// \brief afMultiBody::getRidigBody
/// \param a_name
/// \return
///
afRigidBodyPtr afMultiBody::getRidigBody(std::string a_name, bool suppress_warning){
    if (m_afRigidBodyMap.find(a_name) != m_afRigidBodyMap.end()){
        return m_afRigidBodyMap[a_name];
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY BODY NAMED: " << a_name << std::endl;
        }
        return NULL;
    }
}

///
/// \brief afMultiBody::getRootRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afMultiBody::getRootRigidBody(afRigidBodyPtr a_bodyPtr){
    /// Find Root Body
    afRigidBodyPtr rootParentBody;
    std::vector<int> lineageSize;
    size_t rootParents = 0;
    if (a_bodyPtr){
        if (a_bodyPtr->m_parentBodies.size() == 0){
            rootParentBody = a_bodyPtr;
            rootParents++;
        }
        else{
            lineageSize.resize(a_bodyPtr->m_parentBodies.size());
            std::vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
            for (int parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
                if ((*rIt)->m_parentBodies.size() == 0){
                    rootParentBody = (*rIt);
                    rootParents++;
                }
                lineageSize[parentNum] = (*rIt)->m_parentBodies.size();
            }
        }
    }
    else{
        lineageSize.resize(m_afRigidBodyMap.size());
        afRigidBodyMap::const_iterator mIt = m_afRigidBodyMap.begin();
        for(int bodyNum=0; mIt != m_afRigidBodyMap.end() ; bodyNum++, ++mIt){
            if ((*mIt).second->m_parentBodies.size() == 0){
                rootParentBody = (*mIt).second;
                ++rootParents;
            }
            lineageSize[bodyNum] = (*mIt).second->m_parentBodies.size();
        }

    }
    // In case no root parent is found, it is understood that
    // the multibody chain is cyclical, perhaps return
    // the body with least number of parents
    if (rootParents == 0){
        auto minLineage = std::min_element(lineageSize.begin(), lineageSize.end());
        int idx = std::distance(lineageSize.begin(), minLineage);
        rootParentBody = a_bodyPtr->m_parentBodies[idx];
        rootParents++;
        std::cerr << "WARNING! CYCLICAL CHAIN OF BODIES FOUND WITH NO UNIQUE PARENT, RETURING THE BODY WITH LEAST PARENTS";
    }

    if (rootParents > 1)
        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

    return rootParentBody;
}

///
/// \brief afMultiBody::~afMultiBody
///
afMultiBody::~afMultiBody(){
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

}
//------------------------------------------------------------------------------
