
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

cBulletWorld* afWorld::s_bulletWorld;
double afWorld::m_encl_length;
double afWorld::m_encl_width;
double afWorld::m_encl_height;

GLFWwindow* afCamera::s_mainWindow = NULL;
GLFWmonitor** afCamera::s_monitors;
int afCamera::s_numMonitors = 0;
int afCamera::s_numWindows = 0;
int afCamera::s_cameraIdx = 0;
int afCamera::s_windowIdx = 0;
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

///
/// \brief cVec2btVec
/// \param cVec
/// \return
///
btVector3 cVec2btVec(const cVector3d &cVec){
    btVector3 bVec(cVec.x(), cVec.y(), cVec.z());
    return bVec;
}

///
/// \brief btVec2cVec
/// \param bVec
/// \return
///
cVector3d btVec2cVec(const btVector3 &bVec){
    cVector3d cVec(bVec.x(), bVec.y(), bVec.z());
    return cVec;
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
afRigidBody::afRigidBody(afWorldPtr a_afWorld): cBulletMultiMesh(a_afWorld->s_bulletWorld){
    m_afWorld = a_afWorld;
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
    YAML::Node bodyCollisionMargin = bodyNode["collision margin"];
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
        if(m_afWorld->getRidigBody(node_name, true)){
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
            setUseDisplayList(true);
            markForUpdate(false);
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
        std::vector<double> rgba = m_afWorld->getColorRGBA(bodyColor.as<std::string>());
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

    double _collision_margin = 0.001;

    if (bodyCollisionMargin.IsDefined()){
        _collision_margin = bodyCollisionMargin.as<double>();
    }

    if (m_collisionGeometryType == GeometryType::mesh){
        // Build contact triangles
        buildContactTriangles(_collision_margin, &m_lowResMesh);
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
                m_collisionGroupsIdx.push_back(gNum);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s group number is \"" << gNum << "\" which should be between [0 - 999], ignoring\n";
            }
        }
    }

    setConfigProperties(this, &m_surfaceProps);
    m_afWorld->s_bulletWorld->addChild(this);
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
            force.set(m_afCommand.fx, m_afCommand.fy, m_afCommand.fz);
            torque.set(m_afCommand.tx, m_afCommand.ty, m_afCommand.tz);
        }
        addExternalForce(force);
        addExternalTorque(torque);
        size_t jntCmdSize = m_afCommand.joint_commands_size;
        if (jntCmdSize > 0){
            size_t jntCmdCnt = m_joints.size() < jntCmdSize ? m_joints.size() : jntCmdSize;
            for (size_t jnt = 0 ; jnt < jntCmdCnt ; jnt++){
                // If the enable position controllers flag is set, run
                // position control on all joints
                // The size of pos ctrl mask can be less than the num of joint commands
                // keep this in check and still read the mask to apply it. Run
                // effort control on the masks not specified
                if (m_afCommand.position_controller_mask[jnt] == true || m_afCommand.enable_position_controller == true){
                    m_joints[jnt]->commandPosition(m_afCommand.joint_commands[jnt]);
                }
                else{
                    m_joints[jnt]->commandEffort(m_afCommand.joint_commands[jnt]);
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

///
/// \brief afRigidBody::checkCollisionGroupIdx
/// \param a_idx
/// \return
///
bool afRigidBody::checkCollisionGroupIdx(int a_idx){
    bool in_group = false;
    for (int i = 0 ; i < m_collisionGroupsIdx.size() ; i++){
        if (m_collisionGroupsIdx[i] == a_idx){
            in_group = true;
            break;
        }
    }

    return in_group;
}

bool afRigidBody::isCommonCollisionGroupIdx(std::vector<int> a_idx){
    bool in_group = false;
    for (int i = 0 ; i < a_idx.size() ; i ++){
        for (int j = 0 ; j < m_collisionGroupsIdx.size() ; j++){
            if (a_idx[i] == m_collisionGroupsIdx[j]){
                in_group = true;
                break;
            }
        }
    }

    return in_group;
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

///
/// \brief afSoftBody::afSoftBody
/// \param a_chaiWorld
///
afSoftBody::afSoftBody(afWorldPtr a_afWorld): afSoftMultiMesh(a_afWorld->s_bulletWorld){
    m_afWorld = a_afWorld;
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
    YAML::Node softBodyCollisionMargin = softBodyNode["collision margin"];
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
     double _collision_margin = 0.1;
     if(softBodyCollisionMargin.IsDefined()){
         _collision_margin = softBodyCollisionMargin.as<double>();
     }

    loadFromFile(high_res_filepath.c_str());
    m_lowResMesh.loadFromFile(low_res_filepath.c_str());
    scale(m_scale);
    m_lowResMesh.scale(m_scale);
    buildContactTriangles(_collision_margin, &m_lowResMesh);

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
        std::vector<double> rgba = m_afWorld->getColorRGBA(softBodyColor.as<std::string>());
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
    m_bulletSoftBody->getCollisionShape()->setMargin(_collision_margin);
    m_afWorld->s_bulletWorld->addChild(this);
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
afJoint::afJoint(afWorldPtr a_afWorld){
    m_afWorld = a_afWorld;
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

    afBodyA = m_afWorld->getRidigBody(mB->getNameSpace() + m_parent_name + name_remapping, true);
    afBodyB = m_afWorld->getRidigBody(mB->getNameSpace() + m_child_name + name_remapping, true);

    // If we couldn't find the body with name_remapping, it might have been
    // Defined in another ambf file. Search without name_remapping string
    if(afBodyA == NULL){
        afBodyA = m_afWorld->getRidigBody(m_parent_name, true);
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
        afBodyB = m_afWorld->getRidigBody(m_child_name, true);
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

        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
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

        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
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
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, true);
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
    s_bulletWorld = a_chaiWorld;
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

bool afWorld::createDefaultWorld(){
    // TRANSPARENT WALLS
    double _box_l, _box_w, _box_h;
    _box_l = getEnclosureLength();
    _box_w = getEnclosureWidth();
    _box_h = getEnclosureHeight();

    // bullet static walls and ground
    cBulletStaticPlane* _bulletGround;

    cBulletStaticPlane* _bulletBoxWall[4];

    _bulletBoxWall[0] = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, -1.0, 0.0), -0.5 * _box_w);
    _bulletBoxWall[1] = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, 1.0, 0.0), -0.5 * _box_w);
    _bulletBoxWall[2] = new cBulletStaticPlane(s_bulletWorld, cVector3d(-1.0, 0.0, 0.0), -0.5 * _box_l);
    _bulletBoxWall[3] = new cBulletStaticPlane(s_bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.5 * _box_l);

    cVector3d _nz(0.0, 0.0, 1.0);
    cMaterial _matPlane;
    _matPlane.setWhiteIvory();
    _matPlane.setShininess(1);
    cVector3d _planeNorm;
    cMatrix3d _planeRot;

    double _dim1, _dim2;

    for (int i = 0 ; i < 4 ; i++){
        cBulletStaticPlane* wall = _bulletBoxWall[i];
        _planeNorm = cCross(wall->getPlaneNormal(), _nz);
        _planeRot.setAxisAngleRotationDeg(_planeNorm, 90);
        if (i < 2){
            _dim1 = _box_l; _dim2 = _box_h;
        }
        else{
            _dim1 = _box_h; _dim2 = _box_w;
        }
        cCreatePlane(wall, _dim1, _dim2,
                     wall->getPlaneConstant() * wall->getPlaneNormal(), _planeRot);
        wall->setMaterial(_matPlane);
        if (i == 0) wall->setTransparencyLevel(0.3, true, true);
        else wall->setTransparencyLevel(0.5, true, true);

        s_bulletWorld->addChild(wall);
    }


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create ground plane
    _bulletGround = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, 0.0, 1.0), -0.5 * _box_h);

    // add plane to world as we will want to make it visibe
    s_bulletWorld->addChild(_bulletGround);

    // create a mesh plane where the static plane is located
    cCreatePlane(_bulletGround, _box_l + 0.4, _box_w + 0.8,
                 _bulletGround->getPlaneConstant() * _bulletGround->getPlaneNormal());
    _bulletGround->computeAllNormals();

    // define some material properties and apply to mesh
    _bulletGround->m_material->m_emission.setGrayLevel(0.3);
    _bulletGround->m_material->setGreenChartreuse();
    _bulletGround->m_bulletRigidBody->setFriction(0.5);
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

    createDefaultWorld();

    if (worldLightsData.IsDefined()){
        size_t n_lights = worldLightsData.size();
        for (size_t idx = 0 ; idx < n_lights; idx++){
            std::string light_name = worldLightsData[idx].as<std::string>();
            afLightPtr lightPtr = new afLight(this);
            YAML::Node lightNode = worldNode[light_name];
            if (lightPtr->loadLight(&lightNode, light_name)){
                m_lights.push_back(lightPtr);
            }
        }
    }

    if (m_lights.size() == 0){
        // No Valid Lights defined, so use the default light
        afLightPtr lightPtr = new afLight(this);
        if (lightPtr->createDefaultLight()){
            m_lights.push_back(lightPtr);
        }
    }

    if (worldCamerasData.IsDefined()){
        for (size_t idx = 0 ; idx < worldCamerasData.size(); idx++){
            std::string camera_name = worldCamerasData[idx].as<std::string>();
            afCameraPtr cameraPtr = new afCamera(this);
            YAML::Node cameraNode = worldNode[camera_name];
            if (cameraPtr->loadCamera(&cameraNode, camera_name)){
                m_cameras.push_back(cameraPtr);
            }
        }
    }

    if (m_cameras.size() == 0){
        // No valid cameras defined in the world config file
        // hence create a default camera
        afCameraPtr cameraPtr = new afCamera(this);
        if (cameraPtr->createDefaultCamera()){
            m_cameras.push_back(cameraPtr);
        }

    }

    return true;

}

///
/// \brief afWorld::addRigidBody
/// \param a_name
/// \param a_rb
/// \return
///
bool afWorld::addRigidBody(std::string a_name, afRigidBodyPtr a_rb){
    m_afRigidBodyMap[a_name] = a_rb;
//    s_bulletWorld->addChild(a_rb);
    return true;
}

///
/// \brief afWorld::addSoftBody
/// \param a_name
/// \param a_sb
/// \return
///
bool afWorld::addSoftBody(std::string a_name, afSoftBodyPtr a_sb){
    m_afSoftBodyMap[a_name] = a_sb;
//    s_bulletWorld->addChild(s_sb);
    return true;
}

///
/// \brief afWorld::addJoint
/// \param a_name
/// \param a_jnt
/// \return
///
bool afWorld::addJoint(std::string a_name, afJointPtr a_jnt){
    m_afJointMap[a_name] = a_jnt;
    return true;
}


///
/// \brief afCamera::afCamera
///
afCamera::afCamera(afWorldPtr a_afWorld): cCamera(a_afWorld->s_bulletWorld){

    s_monitors = glfwGetMonitors(&s_numMonitors);
    m_afWorld = a_afWorld;
}

///
/// \brief afCamera::createDefaultCamera
/// \return
///
bool afCamera::createDefaultCamera(){
    std::cerr << "INFO: USING DEFAULT CAMERA" << std::endl;

    // position and orient the camera
    setTargetPos(cVector3d(0.0, 0.0,-0.5));
    set(cVector3d(4.0, 0.0, 2.0),    // camera position (eye)
        cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    setClippingPlanes(0.01, 10.0);

    // set stereo mode
    setStereoMode(cStereoMode::C_STEREO_DISABLED);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    setStereoEyeSeparation(0.02);
    setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    setMirrorVertical(false);

    // create display context
    // compute desired size of window
    const GLFWvidmode* _mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.5 * _mode->width;
    int h = 0.5 * _mode->height;
    int x = 0.5 * (_mode->width - w);
    int y = 0.5 * (_mode->height - h);
    m_window = glfwCreateWindow(w, h, "AMBF Simulator", NULL, NULL);
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

    m_frontLayer->addChild(m_graphicsDynamicsFreqLabel);
    m_frontLayer->addChild(m_wallSimTimeLabel);
    m_frontLayer->addChild(m_devicesModesLabel);
    m_frontLayer->addChild(m_deviceButtonLabel);
    m_frontLayer->addChild(m_controllingDeviceLabel);

    s_windowIdx++;
    s_cameraIdx++;

    // Assign the Window Camera Handles
    m_afWorld->s_bulletWorld->addChild(this);
    return true;
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
    YAML::Node cameraParent = cameraNode["parent"];

    bool _is_valid = true;
    cVector3d _location, _up, _look_at;
    double _clipping_plane_limits[2], _field_view_angle;
    bool _enable_ortho_view = false;
    double _stereo_eye_seperation, _ortho_view_width;

    if (cameraLocationData.IsDefined()){
        assignXYZ(&cameraLocationData, &_location);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOCATION NOT DEFINED, IGNORING " << std::endl;
         _is_valid = false;
    }
    if (cameraLookAtData.IsDefined()){
        assignXYZ(&cameraLookAtData, &_look_at);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOOK AT NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraUpData.IsDefined()){
        assignXYZ(&cameraUpData, &_up);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA UP NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraClippingPlaneData.IsDefined()){
        _clipping_plane_limits[1] = cameraClippingPlaneData["far"].as<double>();
        _clipping_plane_limits[0] = cameraClippingPlaneData["near"].as<double>();
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA CLIPPING PLANE NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraFieldViewAngleData.IsDefined()){
        _field_view_angle = cameraFieldViewAngleData.as<double>();
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA FIELD VIEW DATA NOT DEFINED, IGNORING " << std::endl;
        _field_view_angle = 0.8;
    }
    if (cameraOrthoWidthData.IsDefined()){
        _enable_ortho_view = true;
        _ortho_view_width = cameraOrthoWidthData.as<double>();
    }
    else{
         _enable_ortho_view = false;
    }
    if (cameraControllingDevicesData.IsDefined()){
        for(int idx = 0 ; idx < cameraControllingDevicesData.size() ; idx++){
            m_controllingDevNames.push_back( cameraControllingDevicesData[idx].as<std::string>());
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // position and orient the camera
    setTargetPos(_look_at);
    set(_location,
        _look_at,
        _up);

    // set the near and far clipping planes of the camera
    setClippingPlanes(_clipping_plane_limits[0], _clipping_plane_limits[1]);

    // set stereo mode
    setStereoMode(cStereoMode::C_STEREO_DISABLED);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    setStereoEyeSeparation(0.02);
    setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    setMirrorVertical(false);

    setFieldViewAngleRad(_field_view_angle);

    // Check if ortho view is enabled
    if (_enable_ortho_view){
        setOrthographicView(_ortho_view_width);
    }

    std::string window_name = "AMBF Simulator Window " + std::to_string(s_cameraIdx + 1);
    if (m_controllingDevNames.size() > 0){
        for (int i = 0 ; i < m_controllingDevNames.size() ; i++){
            window_name += (" - " + m_controllingDevNames[i]);
        }

    }

    // create display context
    int _monitor_to_load = 0;
    if (s_cameraIdx < s_numMonitors){
        _monitor_to_load = s_cameraIdx;
    }
    m_monitor = s_monitors[_monitor_to_load];

    // compute desired size of window
    const GLFWvidmode* _mode = glfwGetVideoMode(m_monitor);
    int w = 0.5 * _mode->width;
    int h = 0.5 * _mode->height;
    int x = 0.5 * (_mode->width - w);
    int y = 0.5 * (_mode->height - h);

    m_win_x = x;
    m_win_y = y;
    m_width = w;
    m_height = h;

    m_window = glfwCreateWindow(w, h, window_name.c_str(), NULL, s_mainWindow);
    if (s_windowIdx == 0){
        s_mainWindow = m_window;
    }

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

    m_frontLayer->addChild(m_graphicsDynamicsFreqLabel);
    m_frontLayer->addChild(m_wallSimTimeLabel);
    m_frontLayer->addChild(m_devicesModesLabel);
    m_frontLayer->addChild(m_deviceButtonLabel);
    m_frontLayer->addChild(m_controllingDeviceLabel);

    s_windowIdx++;
    s_cameraIdx++;

    bool _set_body_as_parent = false;

    if (cameraParent.IsDefined()){
        _set_body_as_parent = true;
        std::string parent_name = cameraParent.as<std::string>();
        afRigidBodyPtr pBody = m_afWorld->getRidigBody(parent_name);
        if (pBody){
            pBody->addChild(this);
        }
        else{
            std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                      << parent_name << "\"" <<std::endl;
        }
    }
    if (! _set_body_as_parent){
        m_afWorld->s_bulletWorld->addChild(this);
    }

    return _is_valid;
}


///
/// \brief afCamera::measuredPos
/// \return
///
cVector3d afCamera::measuredPos(){
    return getLocalPos();
}

///
/// \brief afCamera::measuredRot
/// \return
///
cMatrix3d afCamera::measuredRot(){
    return getLocalRot();
}



///
/// \brief afLight::afLight
///
afLight::afLight(afWorldPtr a_afWorld){
    m_afWorld = a_afWorld;
}

///
/// \brief afLight::createDefaultLight
/// \return
///
bool afLight::createDefaultLight(){
    std::cerr << "INFO: NO LIGHT SPECIFIED, USING DEFAULT LIGHTING" << std::endl;
    m_spotLight = new cSpotLight(m_afWorld->s_bulletWorld);
    m_spotLight->setLocalPos(cVector3d(0.0, 0.5, 2.5));
    m_spotLight->setDir(0, 0, -1);
    m_spotLight->setSpotExponent(0.3);
    m_spotLight->setCutOffAngleDeg(60);
    m_spotLight->setShadowMapEnabled(true);
    m_spotLight->m_shadowMap->setQualityVeryHigh();
    m_spotLight->setEnabled(true);
    m_afWorld->s_bulletWorld->addChild(m_spotLight);

    return true;
}


///
/// \brief afLight::loadLight
/// \param light_node
/// \return
///
bool afLight::loadLight(YAML::Node* a_light_node, std::string a_light_name){
    m_name = a_light_name;
    YAML::Node lightNode = *a_light_node;
    YAML::Node lightLocationData = lightNode["location"];
    YAML::Node lightDirectionData = lightNode["direction"];
    YAML::Node lightSpotExponentData = lightNode["spot exponent"];
    YAML::Node lightShadowQualityData = lightNode["shadow quality"];
    YAML::Node lightCuttOffAngleData = lightNode["cutoff angle"];
    YAML::Node lightParent = lightNode["parent"];

    bool _is_valid = true;
    cVector3d _location, _direction;
    double _spot_exponent, _cuttoff_angle;
    int _shadow_quality;



    if (lightLocationData.IsDefined()){
        assignXYZ(&lightLocationData, &_location);
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT LOCATION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (lightDirectionData.IsDefined()){
        assignXYZ(&lightDirectionData, &_direction);
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT DIRECTION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (lightSpotExponentData.IsDefined()){
        _spot_exponent = lightSpotExponentData.as<double>();
    }
    if (lightShadowQualityData.IsDefined()){
        _shadow_quality = lightShadowQualityData.as<int>();
        if (_shadow_quality < 0){
            _shadow_quality = 0;
            std::cerr << "INFO: LIGHT \"" << a_light_name << "\" SHADOW QUALITY SHOULD BE BETWEEN [0-5] " << std::endl;
        }
        else if (_shadow_quality > 5){
            _shadow_quality = 5;
            std::cerr << "INFO: LIGHT \"" << a_light_name << "\" SHADOW QUALITY SHOULD BE BETWEEN [0-5] " << std::endl;
        }
    }
    if (lightCuttOffAngleData.IsDefined()){
        _cuttoff_angle = lightCuttOffAngleData.as<double>();
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT CUTOFF NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }

    if (_is_valid){
        m_spotLight = new cSpotLight(m_afWorld->s_bulletWorld);
        m_spotLight->setLocalPos(_location);
        m_spotLight->setDir(_direction);
        m_spotLight->setSpotExponent(_spot_exponent);
        m_spotLight->setCutOffAngleDeg(_cuttoff_angle * (180/3.14));
        m_spotLight->setShadowMapEnabled(true);

        ShadowQuality sQ = (ShadowQuality) _shadow_quality;
        switch (sQ) {
        case ShadowQuality::no_shadow:
            m_spotLight->setShadowMapEnabled(false);
            break;
        case ShadowQuality::very_low:
            m_spotLight->m_shadowMap->setQualityVeryLow();
            break;
        case ShadowQuality::low:
            m_spotLight->m_shadowMap->setQualityLow();
            break;
        case ShadowQuality::medium:
            m_spotLight->m_shadowMap->setQualityMedium();
            break;
        case ShadowQuality::high:
            m_spotLight->m_shadowMap->setQualityHigh();
            break;
        case ShadowQuality::very_high:
            m_spotLight->m_shadowMap->setQualityVeryHigh();
            break;
        }
        m_spotLight->setEnabled(true);

        bool _set_body_as_parent = false;

        if (lightParent.IsDefined()){
            _set_body_as_parent = true;
            std::string parent_name = lightParent.as<std::string>();
            afRigidBodyPtr pBody = m_afWorld->getRidigBody(parent_name);
            if (pBody){
                pBody->addChild(m_spotLight);
            }
            else{
                std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                          << parent_name << "\"" <<std::endl;
            }
        }
        if (! _set_body_as_parent){
            m_afWorld->s_bulletWorld->addChild(m_spotLight);
        }
    }

    return _is_valid;
}

///
/// \brief afMultiBody::afMultiBody
///
afMultiBody::afMultiBody(){
    m_wallClock.start(true);
}

afMultiBody::afMultiBody(afWorldPtr a_afWorld){
    m_wallClock.start(true);
    m_pickSphere = new cMesh();
    m_afWorld = a_afWorld;
    cCreateSphere(m_pickSphere, 0.02);
    m_pickSphere->m_material->setPinkHot();
    m_pickSphere->setUseDisplayList(true);
    m_pickSphere->markForUpdate(false);
    m_pickSphere->setLocalPos(0,0,0);
    m_pickSphere->setShowEnabled(false);
    a_afWorld->s_bulletWorld->addChild(m_pickSphere);

//    m_pickDragVector = new cMesh();
//    cCreateArrow(m_pickDragVector);
//    m_pickDragVector->m_material->setPurpleAmethyst();
//    m_pickDragVector->setShowEnabled(false);
//    m_pickDragVector->setUseDisplayList(true);
//    m_pickDragVector->markForUpdate(false);
//    m_chaiWorld->addChild(m_pickDragVector);
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
    afJointMap _jntMap = *(m_afWorld->getJointMap());
    if (_jntMap.find(a_joint_name) == _jntMap.end()){
        return remap_string;
    }

    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(_jntMap.find(a_joint_name + remap_string) != _jntMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief afMultiBody::loadMultiBody
/// \param i
/// \return
///
bool afMultiBody::loadMultiBody(int i){
    std::string multibody_config = m_afWorld->getMultiBodyConfig(i);
    return loadMultiBody(multibody_config);
}

///
/// \brief afMultiBody::loadAllMultiBodies
///
void afMultiBody::loadAllMultiBodies(){
    for (int i = 0 ; i < m_afWorld->numMultiBodyConfig(); i++){
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
        a_multibody_config_file = m_afWorld->getMultiBodyConfig();
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
    afRigidBodyMap* _rbMap = m_afWorld->getRigidBodyMap();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        tmpRigidBody = new afRigidBody(m_afWorld);
        std::string rb_name = multiBodyRidigBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(rb_name, _rbMap);
        //        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        YAML::Node rb_node = multiBodyNode[rb_name];
        if (tmpRigidBody->loadRidigBody(&rb_node, rb_name, this)){
            (*_rbMap)[(m_multibody_namespace + rb_name + remap_str).c_str()] = tmpRigidBody;
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
    afSoftBodyMap* _sbMap = m_afWorld->getSoftBodyMap();
    for (size_t i = 0; i < totalSoftBodies; ++i) {
        tmpSoftBody = new afSoftBody(m_afWorld);
        std::string sb_name = multiBodySoftBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(sb_name, _sbMap);
        //        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        YAML::Node sb_node = multiBodyNode[sb_name];
        if (tmpSoftBody->loadSoftBody(&sb_node, sb_name, this)){
            (*_sbMap)[(m_multibody_namespace + sb_name + remap_str).c_str()] = tmpSoftBody;
            //            tmpSoftBody->createAFObject(tmpSoftBody->m_name + remap_str);
        }
    }

    /// Loading Joints
    afJointPtr tmpJoint;
    size_t totalJoints = multiBodyJoints.size();
    afJointMap* _jntMap = m_afWorld->getJointMap();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint(m_afWorld);
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
        std::string remap_str = remapJointName(jnt_name);
        //        printf("Loading body: %s \n", (jnt_name + remap_str).c_str());
        YAML::Node jnt_node = multiBodyNode[jnt_name];
        if (tmpJoint->loadJoint(&jnt_node, jnt_name, this, remap_str)){
            (*_jntMap)[m_multibody_namespace + jnt_name+remap_str] = tmpJoint;
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
    afRigidBodyMap* _rbMap = m_afWorld->getRigidBodyMap();
    afRigidBodyMap::iterator rBodyItA = _rbMap->begin();
    std::vector<btRigidBody*> rBodiesVec;
    rBodiesVec.resize(_rbMap->size());
    int i=0;
    for ( ; rBodyItA != _rbMap->end() ; ++rBodyItA){
        rBodiesVec[i] = rBodyItA->second->m_bulletRigidBody;
        i++;
    }
    if (rBodiesVec.size() >0){
        for (int i = 0 ; i < rBodiesVec.size() - 1 ; i++){
            for (int j = i+1 ; j < rBodiesVec.size() ; j++){
                rBodiesVec[i]->setIgnoreCollisionCheck(rBodiesVec[j], true);
            }
        }
    }
}

///
/// \brief afMultiBody::buildCollisionGroups
///
void afMultiBody::buildCollisionGroups(){
    if (m_collisionGroups.size() > 0){
        std::vector<int> groupNumbers;

        std::map<int, std::vector<afRigidBodyPtr> >::iterator cgIt;
        for(cgIt = m_collisionGroups.begin() ; cgIt != m_collisionGroups.end() ; ++cgIt){
            groupNumbers.push_back(cgIt->first);
        }

        for (int i = 0 ; i < groupNumbers.size() - 1 ; i++){
            int aIdx = groupNumbers[i];
            std::vector<afRigidBodyPtr> grpA = m_collisionGroups[aIdx];
            for (int j = i + 1 ; j < groupNumbers.size() ; j ++){
                int bIdx = groupNumbers[j];
                std::vector<afRigidBodyPtr> grpB = m_collisionGroups[bIdx];

                for(int aBodyIdx = 0 ; aBodyIdx < grpA.size() ; aBodyIdx++){
                    afRigidBodyPtr bodyA = grpA[aBodyIdx];
                    for(int bBodyIdx = 0 ; bBodyIdx < grpB.size() ; bBodyIdx++){
                        afRigidBodyPtr bodyB = grpB[bBodyIdx];
                        if (bodyA != bodyB && !bodyB->isCommonCollisionGroupIdx(bodyA->m_collisionGroupsIdx))
                            bodyA->m_bulletRigidBody->setIgnoreCollisionCheck(bodyB->m_bulletRigidBody, true);
                    }
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
    afRigidBodyMap* _rbMap = m_afWorld->getRigidBodyMap();
    afRigidBodyMap::iterator rBodyIt = _rbMap->begin();
    std::vector<btRigidBody*> bodyFamily;
    std::pair<btVector3, btRigidBody*> pvtAandConnectedBody;
    std::vector< std::pair<btVector3, btRigidBody*> > pvtAandConnectedBodyVec;
    for ( ; rBodyIt != _rbMap->end() ; ++rBodyIt){
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


///
/// \brief afMultiBody::getRidigBody
/// \param a_name
/// \return
///
afRigidBodyPtr afWorld::getRidigBody(std::string a_name, bool suppress_warning){
    if (m_afRigidBodyMap.find(a_name) != m_afRigidBodyMap.end()){
        return m_afRigidBodyMap[a_name];
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY BODY NAMED: " << a_name << std::endl;

            std::cerr <<"Existing Bodies in Map: " << m_afRigidBodyMap.size() << std::endl;
            afRigidBodyMap::iterator rbIt = m_afRigidBodyMap.begin();
            for (; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
                std::cerr << rbIt->first << std::endl;
            }
        }
        return NULL;
    }
}

///
/// \brief afMultiBody::getRootRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afWorld::getRootRigidBody(afRigidBodyPtr a_bodyPtr){
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

// The following functions have been copied from btRidigBodyBase by Erwin Coumans
// with slight modification
///
/// \brief afMultiBody::pickBody
/// \param rayFromWorld
/// \param rayToWorld
/// \return
///
bool afMultiBody::pickBody(const cVector3d &rayFromWorld, const cVector3d &rayToWorld){
    btDynamicsWorld* m_dynamicsWorld = m_afWorld->s_bulletWorld->m_bulletWorld;
    if (m_dynamicsWorld == 0)
        return false;

    btCollisionWorld::ClosestRayResultCallback rayCallback(cVec2btVec(rayFromWorld), cVec2btVec(rayToWorld));

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
    m_dynamicsWorld->rayTest(cVec2btVec(rayFromWorld), cVec2btVec(rayToWorld), rayCallback);
    if (rayCallback.hasHit())
    {
        cVector3d pickPos = btVec2cVec(rayCallback.m_hitPointWorld);
        m_pickSphere->setLocalPos(pickPos);
        m_pickSphere->setShowEnabled(true);
        btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body)
        {
            //other exclusions?
            if (!(body->isStaticObject() || body->isKinematicObject()))
            {
                m_pickedBody = body;
                m_savedState = m_pickedBody->getActivationState();
                m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
                //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                btVector3 localPivot = body->getCenterOfMassTransform().inverse() * cVec2btVec(pickPos);
                btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                m_dynamicsWorld->addConstraint(p2p, true);
                m_pickedConstraint = p2p;
                btScalar mousePickClamping = 30.f;
                p2p->m_setting.m_impulseClamp = mousePickClamping;
                //very weak constraint for picking
                p2p->m_setting.m_tau = 0.001f;
            }
        }

        m_oldPickingPos = rayToWorld;
        m_hitPos = pickPos;
        m_oldPickingDist = (pickPos - rayFromWorld).length();
    }
    return false;

}

///
/// \brief afMultiBody::movePickedBody
/// \param rayFromWorld
/// \param rayToWorld
/// \return
///
bool afMultiBody::movePickedBody(const cVector3d &rayFromWorld, const cVector3d &rayToWorld){
    if (m_pickedBody && m_pickedConstraint)
    {
        btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
        if (pickCon)
        {
            //keep it at the same picking distance

            cVector3d newPivotB;

            cVector3d dir = rayToWorld - rayFromWorld;
            dir.normalize();
            dir *= m_oldPickingDist;

            newPivotB = rayFromWorld + dir;
            // Set the position of grab sphere
            m_pickSphere->setLocalPos(newPivotB);
            // Set the vector properties of the grab velocity vector
//            cVector3d vPos = newPivotB - m_pickSphere->getLocalPos();
//            cVector3d vDir = vPos;
//            m_pickDragVector->setShowEnabled(true);
//            m_pickDragVector->clear();
//            double scale = 5.0;
//            cCreateArrow(m_pickDragVector, vPos.length() * scale, 0.01, 0.1, 0.03, 0, 32, vDir, cVector3d(0,0,0));

            pickCon->setPivotB(cVec2btVec(newPivotB));
            return true;
        }
    }
    return false;
}

///
/// \brief afMultiBody::removePickingConstraint
///
void afMultiBody::removePickingConstraint(){
    btDynamicsWorld* m_dynamicsWorld = m_afWorld->s_bulletWorld->m_bulletWorld;
    if (m_pickedConstraint)
    {
        m_pickSphere->setShowEnabled(false);
        m_pickedBody->forceActivationState(m_savedState);
        m_pickedBody->activate();
        m_dynamicsWorld->removeConstraint(m_pickedConstraint);
        delete m_pickedConstraint;
        m_pickedConstraint = 0;
        m_pickedBody = 0;
    }
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
