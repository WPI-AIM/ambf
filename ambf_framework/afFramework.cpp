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
#include "afFramework.h"
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
afRigidBodySurfaceProperties afRigidBody::m_surfaceProps;

boost::filesystem::path afConfigHandler::s_basePath;
std::string afConfigHandler::s_colorConfigFileName;
std::vector<std::string> afConfigHandler::s_multiBodyConfigFileNames;
std::string afConfigHandler::s_worldConfigFileName;
std::string afConfigHandler::s_inputDevicesConfigFileName;
YAML::Node afConfigHandler::s_colorsNode;

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

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
ros::NodeHandle* afCamera::s_imageTransportNode;
image_transport::ImageTransport* afCamera::s_imageTransport;
bool afCamera::s_imageTransportInitialized = false;
#endif
//------------------------------------------------------------------------------

/// End declare static variables

/// Utility Functions

template <>
///
/// \brief toXYZ<btVector3>
/// \param node
/// \return
///
btVector3 toXYZ<btVector3>(YAML::Node* node){
    btVector3 v;
    v.setX((*node)["x"].as<double>());
    v.setY((*node)["y"].as<double>());
    v.setZ((*node)["z"].as<double>());
    return v;
}

template <>
///
/// \brief toXYZ<cVector3d>
/// \param node
/// \return
///
cVector3d toXYZ<cVector3d>(YAML::Node* node){
    cVector3d v;
    v.x((*node)["x"].as<double>());
    v.y((*node)["y"].as<double>());
    v.z((*node)["z"].as<double>());
    return v;
}

template<>
///
/// \brief toRPY<btVector3>
/// \param node
/// \return
///
btVector3 toRPY<btVector3>(YAML::Node* node){
    btVector3 v;
    v.setX((*node)["r"].as<double>());
    v.setY((*node)["p"].as<double>());
    v.setZ((*node)["y"].as<double>());
    return v;
}

template<>
///
/// \brief toRPY<cVector3>
/// \param node
/// \return
///
cVector3d toRPY<cVector3d>(YAML::Node *node){
    cVector3d v;
    v.x((*node)["r"].as<double>());
    v.y((*node)["p"].as<double>());
    v.z((*node)["y"].as<double>());
    return v;
}


///
/// \brief cVec2btVec
/// \param cVec
/// \return
///
btVector3 toBTvec(const cVector3d &cVec){
    btVector3 bVec(cVec.x(), cVec.y(), cVec.z());
    return bVec;
}

///
/// \brief btVec2cVec
/// \param bVec
/// \return
///
cVector3d toCvec(const btVector3 &bVec){
    cVector3d cVec(bVec.x(), bVec.y(), bVec.z());
    return cVec;
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
        std::cerr << "PLEASE PROVIDE A VALID LAUNCH FILE. EXITING \n";
        return 0;
    }


    //Declare all the YAML Params that we want to look for
    YAML::Node cfgWorldFiles = configNode["world config"];
    YAML::Node cfgColorFile = configNode["color config"];
    YAML::Node cfgInputDevicesFile = configNode["input devices config"];
    YAML::Node cfgMultiBodyFiles = configNode["multibody configs"];


    s_basePath = boost::filesystem::path(a_config_file).parent_path();

    if(cfgWorldFiles.IsDefined()){
        boost::filesystem::path world_cfg_filename = cfgWorldFiles.as<std::string>();
        if (world_cfg_filename.is_relative()){
            world_cfg_filename = s_basePath / world_cfg_filename;
        }
        s_worldConfigFileName = world_cfg_filename.c_str();
    }
    else{
        std::cerr << "ERROR! WORLD CONFIG NOT DEFINED \n";
        return 0;
    }

    if(cfgInputDevicesFile.IsDefined()){
        boost::filesystem::path input_devices_cfg_filename = cfgInputDevicesFile.as<std::string>();
        if (input_devices_cfg_filename.is_relative()){
            input_devices_cfg_filename = s_basePath / input_devices_cfg_filename;
        }
        s_inputDevicesConfigFileName = input_devices_cfg_filename.c_str();
    }
    else{
        std::cerr << "ERROR! INPUT DEVICES CONFIG NOT DEFINED \n";
        return 0;
    }

    if(cfgColorFile.IsDefined()){
        boost::filesystem::path color_cfg_filename = cfgColorFile.as<std::string>();
        if (color_cfg_filename.is_relative()){
            color_cfg_filename = s_basePath / color_cfg_filename;
        }
        s_colorConfigFileName = color_cfg_filename.c_str();
        s_colorsNode = YAML::LoadFile(s_colorConfigFileName.c_str());
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
                mb_cfg_filename = s_basePath / mb_cfg_filename;
            }
            s_multiBodyConfigFileNames.push_back(std::string(mb_cfg_filename.c_str()));
        }
    }
    else{
        std::cerr << "PATH AND MULTIBODY CONFIG NOT DEFINED \n";
        return 0;
    }

    return 1;
}

///
/// \brief afConfigHandler::get_world_config
/// \return
///
std::string afConfigHandler::getWorldConfig(){
    return s_worldConfigFileName;
}

///
/// \brief afConfigHandler::getInputDevicesConfig
/// \return
///
std::string afConfigHandler::getInputDevicesConfig(){
    return s_inputDevicesConfigFileName;
}

///
/// \brief afConfigHandler::get_puzzle_config
/// \return
///
std::string afConfigHandler::getMultiBodyConfig(int i){
    if (i <= getNumMBConfigs()){
        return s_multiBodyConfigFileNames[i];
    }
    else{
        //printf("i = %d, Whereas only %d multi bodies specified", i, s_multiBodyConfigFileNames.size());
        printf("i = %d, Whereas only %lu multi bodies specified", i, (unsigned long)s_multiBodyConfigFileNames.size());
        return "";
    }
}

///
/// \brief afConfigHandler::get_color_config
/// \return
///
std::string afConfigHandler::getColorConfig(){
    return s_colorConfigFileName;
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
/// \brief afCartesianController::afCartesianController
///
afCartesianController::afCartesianController(){
    m_dPos.setValue(0, 0, 0);
    m_dPos_cvec.set(0, 0, 0);
    m_dRot.setIdentity();
    m_dRot_cvec.identity();
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
    btVector3 _dPos_prev, _ddPos, _output;

    _dPos_prev = m_dPos;
    m_dPos = set_point - process_val;
    _ddPos = (m_dPos - _dPos_prev) / dt;

    _output = P_lin * (m_dPos) * ts + D_lin * (_ddPos);
    return _output;
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
    btVector3 _error_cur, _error_prev;
    btMatrix3x3 _dRot_prev;
    btQuaternion _dRotQuat, _dRotQuat_prev;
    btVector3 _output;

    _dRot_prev = m_dRot;
    _dRot_prev.getRotation(_dRotQuat_prev);
    _error_prev = _dRotQuat_prev.getAxis() * _dRotQuat_prev.getAngle();

    m_dRot = process_val.transpose() * set_point;
    m_dRot.getRotation(_dRotQuat);
    _error_cur = _dRotQuat.getAxis() * _dRotQuat.getAngle();

    _output = (P_ang * _error_cur * ts) + (D_ang * (_error_cur - _error_prev) / dt);

    // Important to transform the torque in the world frame as its represented
    // in the body frame from the above computation
    _output = process_val * _output;
    return _output;
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
    cVector3d _dPos_prev, _ddPos, _output;

    _dPos_prev = m_dPos_cvec;
    m_dPos_cvec = set_point - process_val;
    _ddPos = (m_dPos_cvec - _dPos_prev) / dt;

    _output = P_lin * (m_dPos_cvec) * ts + D_lin * (_ddPos);
    return _output;
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
    cVector3d _error_cur, _error_prev;
    cMatrix3d _dRot_prev;
    cVector3d _e_axis, _e_axis_prev;
    double _e_angle, _e_angle_prev;
    cVector3d _output;

    _dRot_prev = m_dRot_cvec;
    _dRot_prev.toAxisAngle(_e_axis_prev, _e_angle_prev);
    _error_prev = _e_axis_prev * _e_angle_prev;

    m_dRot_cvec = cTranspose(process_val) * set_point;
    m_dRot_cvec.toAxisAngle(_e_axis, _e_angle);
    _error_cur = _e_axis * _e_angle;

    _output = (P_ang * _error_cur * ts) + (D_ang * (_error_cur - _error_prev) / dt);

    // Important to transform the torque in the world frame as its represented
    // in the body frame from the above computation
    _output = process_val * _output;
    return _output;
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

    m_dpos.setValue(0, 0, 0);
    m_torque.setValue(0, 0, 0);
}


///
/// \brief afRigidBody::updateUpwardHeirarchyForAddition
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::updateUpwardHeirarchyForAddition(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
    /////////////////////////////////////////////////////////////////////////////////////////////////
    //1a. We add the child body and all of it's children to this body
    std::vector<afChildJointPair> cjPairs;
    cjPairs = a_childBody->m_childAndJointPairs;
    for (int i = 0 ; i < cjPairs.size() ; i++){
        cjPairs[i].m_directConnection = false; // Make sure to mark that these are not directly connected to the body
    }
    cjPairs.push_back(afChildJointPair(a_childBody, a_jnt, true));

    std::vector<afChildJointPair>::iterator cjIt;
    for (cjIt = cjPairs.begin() ; cjIt != cjPairs.end(); ++cjIt){
        bool _cExists = false;
        for (size_t cjIdx = 0; cjIdx < m_childAndJointPairs.size() ; cjIdx++){
            if (cjIt->m_childBody == m_childAndJointPairs[cjIdx].m_childBody){
                _cExists = true;
                break;
            }
        }

        if (!_cExists){
            m_childAndJointPairs.push_back(*cjIt);
            if (cjIt->m_childBody->m_afSensors.size() > 0){
                m_afSensors.insert(m_afSensors.end(), cjIt->m_childBody->m_afSensors.begin(), cjIt->m_childBody->m_afSensors.end());
            }
        }
        else{
            //            std::cerr << "INFO, BODY \"" << this->m_name << "\": ALREADY HAS A CHILD BODY NAMED \""
            //                      << (*cBodyIt)->m_name << "\" PARALLEL LINKAGE FOUND" << std::endl;
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
/// \brief afRigidBody::remove
///
void afRigidBody::remove(){
    if (m_bulletRigidBody){
        m_bulletRigidBody->clearForces();
    }

    if (m_afObjectPtr){
        m_afObjectPtr->cleanUp();
        m_afObjectPtr.reset();
    }

    updateDownwardHeirarchyForRemoval();
    updateUpwardHeirarchyForRemoval();

    if (m_bulletRigidBody){
        m_afWorld->s_bulletWorld->m_bulletWorld->removeRigidBody(m_bulletRigidBody);
    }
}


///
/// \brief afRigidBody::updateUpwardHeirarchyForRemoval
///
void afRigidBody::updateUpwardHeirarchyForRemoval(){
    // We want to remove not only the current body from the parents list of all its children, but also all
    // the parents of this body from the parents list of its children

    std::vector<afRigidBodyPtr> childrensParents = m_parentBodies;
    childrensParents.push_back(this);
    std::vector<afRigidBodyPtr>::iterator cpIt;
    std::vector<afChildJointPair>::iterator cjIt;

    for (cpIt = childrensParents.begin() ; cpIt != childrensParents.end() ; ++cpIt){

        for (cjIt = m_childAndJointPairs.begin() ; cjIt != m_childAndJointPairs.end(); ++cjIt){
            afRigidBodyPtr childBody = cjIt->m_childBody;
            std::vector<afRigidBodyPtr>::iterator pIt;
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
    std::vector<afChildJointPair> parentsChildrenJointPairs = m_childAndJointPairs;
    parentsChildrenJointPairs.push_back( afChildJointPair(this, NULL));
    std::vector<afChildJointPair>::iterator pCJIt;
    std::vector<afRigidBodyPtr>::iterator pIt;

    for (pCJIt = parentsChildrenJointPairs.begin() ; pCJIt != parentsChildrenJointPairs.end() ; ++pCJIt){
        for (pIt = m_parentBodies.begin() ; pIt != m_parentBodies.end(); ++pIt){
            afRigidBodyPtr parentBody = *pIt;
            std::vector<afChildJointPair>::iterator cjIt;
            for (cjIt = parentBody->m_childAndJointPairs.begin() ; cjIt != parentBody->m_childAndJointPairs.end() ; ++cjIt){
                if (pCJIt->m_childBody == cjIt->m_childBody){
                    if (this == cjIt->m_childBody){
                        // This the special case where we provide null for the joint.
                        // We want to clear this joint
                        if (cjIt->m_childJoint){
                            cjIt->m_childJoint->remove();
                        }
                    }
                    parentBody->m_childAndJointPairs.erase(cjIt);
                    break;
                }
            }
        }
    }

    // Also make sure to remove all the directly connected joints to this body
    std::vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_childAndJointPairs.begin() ; cjIt != m_childAndJointPairs.end() ; ++cjIt){
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
void afRigidBody::addChildJointPair(afRigidBodyPtr a_childBody, afJointPtr a_jnt){
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
            (*pIt)->updateUpwardHeirarchyForAddition(a_childBody, a_jnt);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //2. Now we add this body as the parent of all the children of the child body
        std::vector<afChildJointPair> cjPairs;
        cjPairs = a_childBody->m_childAndJointPairs;
        for (int i = 0 ; i < cjPairs.size() ; i++){
            cjPairs[i].m_directConnection = false; // Make sure to mark that these are not directly connected to the body
        }
        cjPairs.push_back(afChildJointPair(a_childBody, a_jnt, true));

        std::vector<afChildJointPair>::iterator cjIt;
        for (cjIt = cjPairs.begin() ; cjIt != cjPairs.end() ; ++cjIt){
            cjIt->m_childBody->updateDownwardHeirarchyForAddition(this);
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
bool afRigidBody::loadRigidBody(std::string rb_config_file, std::string node_name, afMultiBodyPtr mB) {
    YAML::Node baseNode;
    m_mBPtr = mB;
    try{
        baseNode = YAML::LoadFile(rb_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << rb_config_file << std::endl;
        return 0;
    }
    YAML::Node bodyNode = baseNode[node_name];
    return loadRigidBody(&bodyNode, node_name, mB);
}

///
/// \brief afRigidBody::loadRidigBody
/// \param rb_config_data
/// \param name
/// \param mB
/// \return
///
bool afRigidBody::loadRigidBody(YAML::Node* rb_node, std::string node_name, afMultiBodyPtr mB){
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
    YAML::Node bodyCompoundShape = bodyNode["compound shape"];
    YAML::Node bodyGeometry = bodyNode["geometry"];
    YAML::Node bodyCollisionMesh = bodyNode["collision mesh"];
    YAML::Node bodyCollisionShape = bodyNode["collision shape"];
    YAML::Node bodyCompoundCollisionShape = bodyNode["compound collision shape"];
    YAML::Node bodyCollisionGeometry = bodyNode["collision geometry"];
    YAML::Node bodyCollisionMargin = bodyNode["collision margin"];
    YAML::Node bodyScale = bodyNode["scale"];
    YAML::Node bodyInertialOffsetPos = bodyNode["inertial offset"]["position"];
    YAML::Node bodyInertialOffsetRot = bodyNode["inertial offset"]["orientation"];
    YAML::Node bodyMeshPathHR = bodyNode["high resolution path"];
    YAML::Node bodyMeshPathLR = bodyNode["low resolution path"];
    YAML::Node bodyNamespace = bodyNode["namespace"];
    YAML::Node bodyMass = bodyNode["mass"];
    YAML::Node bodyController = bodyNode["controller"];
    YAML::Node bodyInertia = bodyNode["inertia"];
    YAML::Node bodyPos = bodyNode["location"]["position"];
    YAML::Node bodyRot = bodyNode["location"]["orientation"];
    YAML::Node bodyColor = bodyNode["color"];
    YAML::Node bodyColorRGBA = bodyNode["color rgba"];
    YAML::Node bodyColorComponents = bodyNode["color components"];
    YAML::Node bodyLinDamping = bodyNode["damping"]["linear"];
    YAML::Node bodyAngDamping = bodyNode["damping"]["angular"];
    YAML::Node bodyStaticFriction = bodyNode["friction"]["static"];
    YAML::Node bodyRollingFriction = bodyNode["friction"]["rolling"];
    YAML::Node bodyRestitution = bodyNode["restitution"];
    YAML::Node bodyPublishChildrenNames = bodyNode["publish children names"];
    YAML::Node bodyPublishJointNames = bodyNode["publish joint names"];
    YAML::Node bodyPublishJointPositions = bodyNode["publish joint positions"];
    YAML::Node bodyPublishFrequency = bodyNode["publish frequency"];
    YAML::Node bodyCollisionGroups = bodyNode["collision groups"];
    YAML::Node bodyResistiveSurface = bodyNode["resistive surface"];


    if(bodyName.IsDefined()){
        m_name = bodyName.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    // Check if the name is world and if it has already been defined.
    if (strcmp(m_name.c_str(), "world") == 0 || strcmp(m_name.c_str(), "World") == 0 || strcmp(m_name.c_str(), "WORLD") == 0){
        if(m_afWorld->getAFRigidBody(node_name, true)){
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

    std::string high_res_path;

    if (bodyCollisionShape.IsDefined()){
        _collision_geometry_valid = true;
        m_collisionGeometryType = GeometryType::shape;
        _collision_shape_str = bodyCollisionShape.as<std::string>();
    }
    else if (bodyCompoundCollisionShape.IsDefined()){
        _collision_geometry_valid = true;
        m_collisionGeometryType = GeometryType::compound_shape;
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
    else if (bodyCompoundShape.IsDefined()){
        _visual_geometry_valid = true;
        m_visualGeometryType = GeometryType::compound_shape;
        if (!_collision_geometry_valid){
            _collision_geometry_valid = true;
            m_collisionGeometryType = GeometryType::compound_shape;
            bodyCompoundCollisionShape = bodyCompoundShape;
        }
    }
    else if(bodyMesh.IsDefined()){
        m_mesh_name = bodyMesh.as<std::string>();
        if (!m_mesh_name.empty()){
            // Each ridig body can have a seperate path for its low and high res meshes
            // Incase they are defined, we use those paths and if they are not, we use
            // the paths for the whole file
            if (bodyMeshPathHR.IsDefined()){
                high_res_path = bodyMeshPathHR.as<std::string>();
                high_res_filepath = bodyMeshPathHR.as<std::string>() + m_mesh_name;
                if (high_res_filepath.is_relative()){
                    high_res_path = mB->getMultiBodyPath() + '/' + high_res_path;
                    high_res_filepath = mB->getMultiBodyPath() + '/' + high_res_filepath.c_str();
                }
            }
            else{
                high_res_path = mB->getHighResMeshesPath();
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
        int dx = 32; // Default x resolution for shape
        int dy = 32; // Default y resolution for shape
        int dz = 5; // Default z resolution for shape
        if (bodyGeometry["dx"].IsDefined()){
            dx = bodyGeometry["dx"].as<int>();
        }
        if (bodyGeometry["dy"].IsDefined()){
            dy = bodyGeometry["dy"].as<int>();
        }
        if (bodyGeometry["dz"].IsDefined()){
            dz = bodyGeometry["dZ"].as<int>();
        }
        cMesh* tempMesh = new cMesh();
        if (_visual_shape_str.compare("Box") == 0 || _visual_shape_str.compare("box") == 0 || _visual_shape_str.compare("BOX") == 0){
            double x = bodyGeometry["x"].as<double>();
            double y = bodyGeometry["y"].as<double>();
            double z = bodyGeometry["z"].as<double>();
            x *= m_scale;
            y *= m_scale;
            z *= m_scale;
            cCreateBox(tempMesh, x, y, z);
        }
        else if (_visual_shape_str.compare("Sphere") == 0 || _visual_shape_str.compare("sphere") == 0 || _visual_shape_str.compare("SPHERE") == 0){
            double radius = bodyGeometry["radius"].as<double>();
            radius *= m_scale;
            cCreateSphere(tempMesh, radius, dx, dy);
        }
        else if (_visual_shape_str.compare("Cylinder") == 0 || _visual_shape_str.compare("cylinder") == 0 || _visual_shape_str.compare("CYLINDER") == 0){
            double radius = bodyGeometry["radius"].as<double>();
            double height = bodyGeometry["height"].as<double>();
            radius *= m_scale;
            height *= m_scale;
            cCreateCylinder(tempMesh, height, radius, dx, dy, dz, true, true, cVector3d(0.0, 0.0,-0.5 * height));
        }
        else if (_visual_shape_str.compare("Capsule") == 0 || _visual_shape_str.compare("capsule") == 0 || _visual_shape_str.compare("CAPSULE") == 0){
            double radius = bodyGeometry["radius"].as<double>();
            double height = bodyGeometry["height"].as<double>();
            radius *= m_scale;
            height *= m_scale;
            cCreateEllipsoid(tempMesh, radius, radius, height, dx, dy);
        }
        else if (_visual_shape_str.compare("Cone") == 0 || _visual_shape_str.compare("cone") == 0 || _visual_shape_str.compare("CONE") == 0){
            double radius = bodyGeometry["radius"].as<double>();
            double height = bodyGeometry["height"].as<double>();
            radius *= m_scale;
            height *= m_scale;
            cCreateCone(tempMesh, height, radius, 0, dx, dy, dz, true, true, cVector3d(0.0, 0.0, -0.5 * height));
        }
        m_meshes->push_back(tempMesh);
    }

    else if (m_visualGeometryType == GeometryType::compound_shape){
        // First of all, set the inertial offset to 0.
        bodyInertialOffsetPos = bodyNode["inertial offset undef"];
        for(int shapeIdx = 0 ; shapeIdx < bodyCompoundShape.size() ; shapeIdx++){
            _visual_shape_str = bodyCompoundShape[shapeIdx]["shape"].as<std::string>();
            bodyGeometry = bodyCompoundShape[shapeIdx]["geometry"];
            YAML::Node shapeOffset = bodyCompoundShape[shapeIdx]["offset"];
            int dx = 32; // Default x resolution for shape
            int dy = 32; // Default y resolution for shape
            int dz = 5; // Default z resolution for shape
            double px = shapeOffset["position"]["x"].as<double>();
            double py = shapeOffset["position"]["y"].as<double>();
            double pz = shapeOffset["position"]["z"].as<double>();
            double roll =  shapeOffset["orientation"]["r"].as<double>();
            double pitch = shapeOffset["orientation"]["p"].as<double>();
            double yaw =   shapeOffset["orientation"]["y"].as<double>();
            cVector3d shapePos(px, py, pz);
            cMatrix3d shapeRot;
            shapeRot.setExtrinsicEulerRotationRad(roll,pitch,yaw,cEulerOrder::C_EULER_ORDER_XYZ);
            if (bodyGeometry["dx"].IsDefined()){
                dx = bodyGeometry["dx"].as<int>();
            }
            if (bodyGeometry["dy"].IsDefined()){
                dy = bodyGeometry["dy"].as<int>();
            }
            if (bodyGeometry["dz"].IsDefined()){
                dz = bodyGeometry["dz"].as<int>();
            }
            cMesh* tempMesh = new cMesh();
            if (_visual_shape_str.compare("Box") == 0 || _visual_shape_str.compare("box") == 0 || _visual_shape_str.compare("BOX") == 0){
                double x = bodyGeometry["x"].as<double>();
                double y = bodyGeometry["y"].as<double>();
                double z = bodyGeometry["z"].as<double>();
                x *= m_scale;
                y *= m_scale;
                z *= m_scale;
                //                std::cerr << "---------------------" << std::endl;
                //                std::cerr << shapeIdx << std::endl;
                //                std::cerr << "Geometry : " << x << " " << y << " " << z << std::endl;
                //                std::cerr << "Location : " << std::endl;
                //                std::cerr << "\tRotation: " << pitch << " " << roll << " " << yaw << std::endl;
                //                std::cerr << "\tPosition: " << px << " " << py << " " << pz << std::endl;
                //                std::cerr << "Scale    : "  << m_scale << std::endl;

                cCreateBox(tempMesh, x, y, z, shapePos, shapeRot);
            }
            else if (_visual_shape_str.compare("Sphere") == 0 || _visual_shape_str.compare("sphere") == 0 || _visual_shape_str.compare("SPHERE") == 0){
                double radius = bodyGeometry["radius"].as<double>();
                radius *= m_scale;
                cCreateSphere(tempMesh, radius, dx, dy, shapePos, shapeRot);
            }
            else if (_visual_shape_str.compare("Cylinder") == 0 || _visual_shape_str.compare("cylinder") == 0 || _visual_shape_str.compare("CYLINDER") == 0){
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                shapePos.set(shapePos.x(), shapePos.y(), shapePos.z() - 0.5 * height);
                cCreateCylinder(tempMesh, height, radius, dx, dy, dz, true, true, shapePos, shapeRot);
            }
            else if (_visual_shape_str.compare("Capsule") == 0 || _visual_shape_str.compare("capsule") == 0 || _visual_shape_str.compare("CAPSULE") == 0){
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                cCreateEllipsoid(tempMesh, radius, radius, height, dx, dy, shapePos, shapeRot);
            }
            else if (_visual_shape_str.compare("Cone") == 0 || _visual_shape_str.compare("cone") == 0 || _visual_shape_str.compare("CONE") == 0){
                double radius = bodyGeometry["radius"].as<double>();
                double height = bodyGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                shapePos.set(shapePos.x(), shapePos.y(), shapePos.z() - 0.5 * height);
                cCreateCone(tempMesh, height, radius, 0, dx, dy, dz, true, true, shapePos, shapeRot);
            }
            m_meshes->push_back(tempMesh);
        }
    }

    cMaterial _mat;
    double _r, _g, _b, _a;
    if(bodyColorRGBA.IsDefined()){
        _r = bodyColorRGBA["r"].as<float>();
        _g = bodyColorRGBA["g"].as<float>();
        _b = bodyColorRGBA["b"].as<float>();
        _a = bodyColorRGBA["a"].as<float>();
        _mat.setColorf(_r, _g, _b, _a);
        setMaterial(_mat);
        setTransparencyLevel(_a);
    }
    else if(bodyColorComponents.IsDefined()){

        if (bodyColorComponents["diffuse"].IsDefined()){
            _r = bodyColorComponents["diffuse"]["r"].as<float>();
            _g = bodyColorComponents["diffuse"]["g"].as<float>();
            _b = bodyColorComponents["diffuse"]["b"].as<float>();
            _mat.m_diffuse.set(_r, _g, _b);
        }
        if (bodyColorComponents["ambient"].IsDefined()){
            double _level = bodyColorComponents["ambient"]["level"].as<float>();
            _r *= _level;
            _g *= _level;
            _b *= _level;
            _mat.m_ambient.set(_r, _g, _b);
        }
        if (bodyColorComponents["specular"].IsDefined()){
            _r = bodyColorComponents["specular"]["r"].as<float>();
            _g = bodyColorComponents["specular"]["g"].as<float>();
            _b = bodyColorComponents["specular"]["b"].as<float>();
            _mat.m_specular.set(_r, _g, _b);
        }
        if (bodyColorComponents["emission"].IsDefined()){
            _r = bodyColorComponents["emission"]["r"].as<float>();
            _g = bodyColorComponents["emission"]["g"].as<float>();
            _b = bodyColorComponents["emission"]["b"].as<float>();
            _mat.m_emission.set(_r, _g, _b);
        }
        _a = bodyColorComponents["transparency"].as<float>();
        setMaterial(_mat);
        setTransparencyLevel(_a);
    }
    else if(bodyColor.IsDefined()){
        std::vector<double> rgba = m_afWorld->getColorRGBA(bodyColor.as<std::string>());
        _mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
        setMaterial(_mat);
        setTransparencyLevel(rgba[3]);
    }

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
            x *= m_scale;
            y *= m_scale;
            z *= m_scale;
            btVector3 halfExtents(x/2, y/2, z/2);
            m_bulletCollisionShape = new btBoxShape(halfExtents);
        }
        else if (_shape_str.compare("Sphere") == 0 || _shape_str.compare("sphere") == 0 ||_shape_str.compare("SPHERE") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            radius *= m_scale;
            m_bulletCollisionShape = new btSphereShape(radius);
        }
        else if (_shape_str.compare("Cylinder") == 0 || _shape_str.compare("cylinder") == 0 ||_shape_str.compare("CYLINDER") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            double height = bodyCollisionGeometry["height"].as<double>();
            radius *= m_scale;
            height *= m_scale;
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
            radius *= m_scale;
            height *= m_scale;
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
            radius *= m_scale;
            height *= m_scale;
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
    else if (m_collisionGeometryType == GeometryType::compound_shape){
        btCompoundShape* _compoundCollisionShape = new btCompoundShape();
        for (int shapeIdx = 0 ; shapeIdx < bodyCompoundCollisionShape.size() ; shapeIdx++){
            std::string _shape_str = bodyCompoundCollisionShape[shapeIdx]["shape"].as<std::string>();
            bodyCollisionGeometry = bodyCompoundCollisionShape[shapeIdx]["geometry"];
            YAML::Node shapeOffset = bodyCompoundCollisionShape[shapeIdx]["offset"];
            double px = shapeOffset["position"]["x"].as<double>();
            double py = shapeOffset["position"]["y"].as<double>();
            double pz = shapeOffset["position"]["z"].as<double>();
            double roll =  shapeOffset["orientation"]["r"].as<double>();
            double pitch = shapeOffset["orientation"]["p"].as<double>();
            double yaw =   shapeOffset["orientation"]["y"].as<double>();
            btVector3 shapePos(px, py, pz);
            btMatrix3x3 shapeRot;
            shapeRot.setEulerZYX(roll, pitch, yaw);
            btTransform shapeTrans(shapeRot, shapePos);
            if (_shape_str.compare("Box") == 0 || _shape_str.compare("box") == 0 ||_shape_str.compare("BOX") == 0){
                double x = bodyCollisionGeometry["x"].as<double>();
                double y = bodyCollisionGeometry["y"].as<double>();
                double z = bodyCollisionGeometry["z"].as<double>();
                x *= m_scale;
                y *= m_scale;
                z *= m_scale;
                btVector3 halfExtents(x/2, y/2, z/2);
                _compoundCollisionShape->addChildShape(shapeTrans, new btBoxShape(halfExtents));
            }
            else if (_shape_str.compare("Sphere") == 0 || _shape_str.compare("sphere") == 0 ||_shape_str.compare("SPHERE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                radius *= m_scale;
                _compoundCollisionShape->addChildShape(shapeTrans, new btSphereShape(radius));
            }
            else if (_shape_str.compare("Cylinder") == 0 || _shape_str.compare("cylinder") == 0 ||_shape_str.compare("CYLINDER") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    btVector3 halfExtents(height/2, radius, radius);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeX(halfExtents));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    btVector3 halfExtents(radius, height/2, radius);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShape(halfExtents));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    btVector3 halfExtents(radius, radius, height/2);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeZ(halfExtents));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    btVector3 halfExtents(radius, radius, height/2);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeZ(halfExtents));
                }
            }
            else if (_shape_str.compare("Capsule") == 0 || _shape_str.compare("capsule") == 0 ||_shape_str.compare("CAPSULE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                // Adjust for height as bullet treats the height as the distance
                // between the two spheres forming the capsule's ends.
                height = height - 2*radius;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeX(radius, height));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShape(radius, height));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeZ(radius, height));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeZ(radius, height));
                }
            }
            else if (_shape_str.compare("Cone") == 0 || _shape_str.compare("cone") == 0 ||_shape_str.compare("CONE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeX(radius, height));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShape(radius, height));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeZ(radius, height));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeZ(radius, height));
                }
            }
        }
        m_bulletCollisionShape = _compoundCollisionShape;
    }

    else if (m_collisionGeometryType == GeometryType::compound_shape){
        btCompoundShape* _compoundCollisionShape = new btCompoundShape();
        for (int shapeIdx = 0 ; shapeIdx < bodyCompoundCollisionShape.size() ; shapeIdx++){
            std::string _shape_str = bodyCompoundCollisionShape[shapeIdx]["shape"].as<std::string>();
            bodyCollisionGeometry = bodyCompoundCollisionShape[shapeIdx]["geometry"];
            YAML::Node shapeOffset = bodyCompoundCollisionShape[shapeIdx]["offset"];
            double px = shapeOffset["position"]["x"].as<double>();
            double py = shapeOffset["position"]["y"].as<double>();
            double pz = shapeOffset["position"]["z"].as<double>();
            double roll =  shapeOffset["orientation"]["r"].as<double>();
            double pitch = shapeOffset["orientation"]["p"].as<double>();
            double yaw =   shapeOffset["orientation"]["y"].as<double>();
            btVector3 shapePos(px, py, pz);
            btMatrix3x3 shapeRot;
            shapeRot.setEulerZYX(roll, pitch, yaw);
            btTransform shapeTrans(shapeRot, shapePos);
            if (_shape_str.compare("Box") == 0 || _shape_str.compare("box") == 0 ||_shape_str.compare("BOX") == 0){
                double x = bodyCollisionGeometry["x"].as<double>();
                double y = bodyCollisionGeometry["y"].as<double>();
                double z = bodyCollisionGeometry["z"].as<double>();
                x *= m_scale;
                y *= m_scale;
                z *= m_scale;
                //                std::cerr << "---------------------" << std::endl;
                //                std::cerr << shapeIdx << std::endl;
                //                std::cerr << "Geometry : " << x << " " << y << " " << z << std::endl;
                //                std::cerr << "Location : " << std::endl;
                //                std::cerr << "\tRotation: " << pitch << " " << roll << " " << yaw << std::endl;
                //                std::cerr << "\tPosition: " << px << " " << py << " " << pz << std::endl;
                //                std::cerr << "Scale    : "  << m_scale << std::endl;
                btVector3 halfExtents(x/2, y/2, z/2);
                _compoundCollisionShape->addChildShape(shapeTrans, new btBoxShape(halfExtents));
            }
            else if (_shape_str.compare("Sphere") == 0 || _shape_str.compare("sphere") == 0 ||_shape_str.compare("SPHERE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                radius *= m_scale;
                _compoundCollisionShape->addChildShape(shapeTrans, new btSphereShape(radius));
            }
            else if (_shape_str.compare("Cylinder") == 0 || _shape_str.compare("cylinder") == 0 ||_shape_str.compare("CYLINDER") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    btVector3 halfExtents(height/2, radius, radius);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeX(halfExtents));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    btVector3 halfExtents(radius, height/2, radius);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShape(halfExtents));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    btVector3 halfExtents(radius, radius, height/2);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeZ(halfExtents));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    btVector3 halfExtents(radius, radius, height/2);
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCylinderShapeZ(halfExtents));
                }
            }
            else if (_shape_str.compare("Capsule") == 0 || _shape_str.compare("capsule") == 0 ||_shape_str.compare("CAPSULE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                // Adjust for height as bullet treats the height as the distance
                // between the two spheres forming the capsule's ends.
                height = height - 2*radius;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeX(radius, height));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShape(radius, height));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeZ(radius, height));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    _compoundCollisionShape->addChildShape(shapeTrans, new btCapsuleShapeZ(radius, height));
                }
            }
            else if (_shape_str.compare("Cone") == 0 || _shape_str.compare("cone") == 0 ||_shape_str.compare("CONE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeX(radius, height));
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShape(radius, height));
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeZ(radius, height));
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    _compoundCollisionShape->addChildShape(shapeTrans, new btConeShapeZ(radius, height));
                }
            }
        }
        m_bulletCollisionShape = _compoundCollisionShape;
    }

    if (bodyNamespace.IsDefined()){
        m_namespace = bodyNamespace.as<std::string>();
    }
    else{
        m_namespace = mB->getNamespace();
    }

    btTransform iOffTrans;
    btQuaternion iOffQuat;
    btVector3 iOffPos;
    iOffQuat.setEuler(0,0,0);
    iOffPos.setValue(0,0,0);

    if(bodyInertialOffsetPos.IsDefined()){
        iOffPos = toXYZ<btVector3>(&bodyInertialOffsetPos);
        iOffPos = m_scale * iOffPos;
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
    if(bodyController.IsDefined()){
        // Check if the linear controller is defined
        if (bodyController["linear"].IsDefined()){
            double _P, _D;
            _P = bodyController["linear"]["P"].as<double>();
            _D = bodyController["linear"]["D"].as<double>();
            m_controller.setLinearGains(_P, 0, _D);
            _lin_gains_computed = true;
        }

        // Check if the angular controller is defined
        if(bodyController["angular"].IsDefined()){
            double _P, _D;
            _P = bodyController["angular"]["P"].as<double>();
            _D = bodyController["angular"]["D"].as<double>();
            m_controller.setAngularGains(_P, 0, _D);
            _ang_gains_computed = true;
        }
    }

    // If no controller gains are defined, compute based on lumped mass
    // and intertia
    if(!_lin_gains_computed || !_ang_gains_computed){
        computeControllerGains();
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
        else if (m_lowResMesh.m_meshes->size() > 0 || m_collisionGeometryType == GeometryType::shape || m_collisionGeometryType == GeometryType::compound_shape){
            estimateInertia();
        }
    }

    buildDynamicModel();

    if(bodyPos.IsDefined()){
        m_initialPos = toXYZ<cVector3d>(&bodyPos);
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

    // Handle resistive surface features

    if (bodyResistiveSurface.IsDefined()){
        m_resistiveSurface.enable = true;

        if (bodyResistiveSurface["face resolution"].IsDefined()){
            m_resistiveSurface.faceResolution = bodyResistiveSurface["face resolution"].as<double>();
        }
        if (bodyResistiveSurface["edge resolution"].IsDefined()){
            m_resistiveSurface.edgeResolution = bodyResistiveSurface["edge resolution"].as<double>();
        }
        if (bodyResistiveSurface["range"].IsDefined()){
            m_resistiveSurface.range = bodyResistiveSurface["range"].as<double>();
        }
        if (bodyResistiveSurface["depth"].IsDefined()){
            m_resistiveSurface.depth = bodyResistiveSurface["depth"].as<double>();
        }
        if (bodyResistiveSurface["contact area"].IsDefined()){
            m_resistiveSurface.contactArea = bodyResistiveSurface["contact area"].as<double>();
        }

        if (bodyResistiveSurface["friction"].IsDefined()){
            YAML::Node bodyResistiveFriction = bodyResistiveSurface["friction"];
            if (bodyResistiveFriction["static"].IsDefined()){
                m_resistiveSurface.staticContactFriction = bodyResistiveFriction["static"].as<double>();
            }
            if (bodyResistiveFriction["damping"].IsDefined()){
                m_resistiveSurface.staticContactDamping = bodyResistiveFriction["damping"].as<double>();
            }
            if (bodyResistiveFriction["dynamic"].IsDefined()){
                m_resistiveSurface.dynamicFriction = bodyResistiveFriction["dynamic"].as<double>();
            }
            if (bodyResistiveFriction["variable"].IsDefined()){
                m_resistiveSurface.useVariableCoeff = bodyResistiveFriction["variable"].as<bool>();
            }
        }

        if (bodyResistiveSurface["contact stiffness"].IsDefined()){
            m_resistiveSurface.contactNormalStiffness = bodyResistiveSurface["contact stiffness"].as<double>();
        }
        if (bodyResistiveSurface["contact damping"].IsDefined()){
            m_resistiveSurface.contactNormalDamping = bodyResistiveSurface["contact damping"].as<double>();
        }

        bool _showSensors = false;
        double _visiblitySize = 0;
        if (bodyResistiveSurface["visible"].IsDefined()){
            _showSensors = bodyResistiveSurface["visible"].as<bool>();
            if (bodyResistiveSurface["visible size"].IsDefined()){
                _visiblitySize = bodyResistiveSurface["visible size"].as<double>();
            }
        }

        cMesh* _sourceMesh = NULL;
        if (bodyResistiveSurface["mesh"].IsDefined()){
            std::string _resistiveMeshName = bodyResistiveSurface["mesh"].as<std::string>();
            _resistiveMeshName = high_res_path + _resistiveMeshName;
            cMultiMesh* resistiveMesh = new cBulletMultiMesh(m_afWorld->s_bulletWorld);
            if (resistiveMesh->loadFromFile(_resistiveMeshName)){
                _sourceMesh = (*resistiveMesh->m_meshes)[0];
            }
            else{
                std::cerr << "ERROR! BODY \"" << m_name <<
                             "\'s\" RESISTIVE MESH " <<
                             _resistiveMeshName << " NOT FOUND. IGNORING\n";
            }
        }
        else{
            _sourceMesh = (*m_meshes)[0];
        }

        // Lets assign a resistive sensor per each triangle face.
        if (_sourceMesh){
            for (int tIdx = 0 ; tIdx < _sourceMesh->m_triangles->getNumElements() ; tIdx++ ){
                int vIdx0 = _sourceMesh->m_triangles->getVertexIndex0(tIdx);
                int vIdx1 = _sourceMesh->m_triangles->getVertexIndex1(tIdx);
                int vIdx2 = _sourceMesh->m_triangles->getVertexIndex2(tIdx);

                cVector3d v0 = _sourceMesh->m_vertices->getLocalPos(vIdx0);
                cVector3d v1 = _sourceMesh->m_vertices->getLocalPos(vIdx1);
                cVector3d v2 = _sourceMesh->m_vertices->getLocalPos(vIdx2);

                cVector3d e1 = v1 - v0;
                cVector3d e2 = v2 - v1;

                cVector3d centroid = ( v0 + v1 + v2 ) / 3;

                cVector3d normal = cCross(e1, e2);

                normal.normalize();
                cVector3d rayFrom = centroid - (normal * m_resistiveSurface.depth);
                cVector3d rayTo = rayFrom + (normal * m_resistiveSurface.range);

                afResistanceSensor* _resistanceSensor = new afResistanceSensor(m_afWorld);
                _resistanceSensor->setRayFromInLocal(rayFrom);
                _resistanceSensor->setRayToInLocal(rayTo);
                _resistanceSensor->setDirection(normal);
                _resistanceSensor->setRange(m_resistiveSurface.range);
                _resistanceSensor->setContactArea(m_resistiveSurface.contactArea);
                _resistanceSensor->setStaticContactFriction(m_resistiveSurface.staticContactFriction);
                _resistanceSensor->setStaticContactDamping(m_resistiveSurface.staticContactDamping);
                _resistanceSensor->setDynamicFriction(m_resistiveSurface.dynamicFriction);
                _resistanceSensor->setContactNormalStiffness(m_resistiveSurface.contactNormalStiffness);
                _resistanceSensor->setContactNormalDamping(m_resistiveSurface.contactNormalDamping);
                _resistanceSensor->useVariableCoeff(m_resistiveSurface.useVariableCoeff);

                if (_showSensors){
                    if (_visiblitySize <= 0){
                        _visiblitySize = m_resistiveSurface.range / 5;
                    }
                    _resistanceSensor->setSensorVisibilityRadius(_visiblitySize);
                    _resistanceSensor->enableVisualization();
                }

                _resistanceSensor->m_sensorType = afSensorType::resistance;

                this->addAFSensor(_resistanceSensor);
                std::string _sensorName = m_name + "_sensor" + std::to_string(tIdx);
                m_afWorld->addAFSensor(_resistanceSensor, _sensorName);
                _resistanceSensor->m_parentBody = this;
            }
        }
# ifdef AMBF_ENABLE_PARALLEL_SENSOR_PROCESSING
        // Sub-divide the sensors into the threads
        int _nSensors = m_afSensors.size();
        int _nBlocks = _nSensors / m_sensorThreadBlockSize;

        for (int bIdx = 0 ; bIdx < _nBlocks ; bIdx++){
            m_threadUpdateFlags.push_back(false);
        }

        for (int bIdx = 0 ; bIdx < _nBlocks ; bIdx++){
            std::thread* th = new std::thread(&afRigidBody::updateBodySensors, this, bIdx);
            m_sensorThreads.push_back(th);
        }
#endif

    }

    if (bodyPublishChildrenNames.IsDefined()){
        m_publish_children_names = bodyPublishChildrenNames.as<bool>();
    }

    if (bodyPublishJointNames.IsDefined()){
        m_publish_joint_names = bodyPublishJointNames.as<bool>();
    }

    if (bodyPublishJointPositions.IsDefined()){
        m_publish_joint_positions = bodyPublishJointPositions.as<bool>();
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

    double P_lin, D_lin, P_ang, D_ang;
    double lumped_mass = m_mass;
    cVector3d lumped_intertia = m_inertia;
    std::vector<afChildJointPair>::iterator sjIt;
    for(sjIt = m_childAndJointPairs.begin() ; sjIt != m_childAndJointPairs.end() ; ++sjIt){
        lumped_mass += sjIt->m_childBody->getMass();
        lumped_intertia += sjIt->m_childBody->getInertia();
    }
    if (!_lin_gains_computed){
        P_lin = lumped_mass * 20;
        D_lin = P_lin / 100;
        m_controller.setLinearGains(P_lin, 0, D_lin);
        _lin_gains_computed = true;
    }
    // TODO
    // Need a better way of estimating angular gains
    if (!_ang_gains_computed){
        P_ang = lumped_mass * 10;
        D_ang = lumped_mass;
        m_controller.setAngularGains(P_ang, 0, D_ang);
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

    // Update the data for sensors
#ifdef AMBF_ENABLE_PARALLEL_SENSOR_PROCESSING
    for (int thIdx = 0 ; thIdx < m_sensorThreads.size() ; thIdx++){
        m_threadUpdateFlags[thIdx] = true;
    }
#else
    for (int i = 0 ; i < m_afSensors.size() ; i++){
        m_afSensors[i]->updateSensor();
    }
#endif

    // update Transform data for m_ObjectPtr
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afObjectPtr.get() != nullptr){
        m_afObjectPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
        m_afObjectPtr->cur_orientation(q.x, q.y, q.z, q.w);

        // Since the mass and inertia aren't going to change that often, write them
        // out intermittently
        if (m_write_count % 2000 == 0){
            m_afObjectPtr->set_mass(getMass());
            m_afObjectPtr->set_principal_intertia(getInertia().x(), getInertia().y(), getInertia().z());
        }

        // We can set this body to publish it's children joint names in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_joint_names == true || m_afObjectPtr->m_objectCommand.publish_joint_names == true){
            // Since joint names aren't going to change that often
            // change the field less so often
            if (m_write_count % 2000 == 0){
                afObjectStateSetJointNames();
            }
        }

        // We can set this body to publish joint positions in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_joint_positions == true || m_afObjectPtr->m_objectCommand.publish_joint_positions == true){
            afObjectSetJointPositions();
        }

        // We can set this body to publish it's children names in either its AMBF Description file or
        // via it's afCommand using ROS Message
        if (m_publish_children_names == true || m_afObjectPtr->m_objectCommand.publish_children_names == true){
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
bool afRigidBody::updateBodySensors(int threadIdx){
    int startIdx = threadIdx * m_sensorThreadBlockSize;
    int endIdx = startIdx + m_sensorThreadBlockSize;

    endIdx = endIdx > m_afSensors.size() ? m_afSensors.size() : endIdx;
    while (m_keepSensorThreadsAlive){
        if (m_threadUpdateFlags[threadIdx] == true){

            for (int idx = startIdx ; idx < endIdx ; idx++){
                m_afSensors[idx]->updateSensor();
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
void afRigidBody::afObjectCommandExecute(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afObjectPtr.get() != nullptr){
        m_afObjectPtr->update_af_cmd();
        btVector3 force, torque;
        ObjectCommand m_afCommand = m_afObjectPtr->m_objectCommand;
        m_af_enable_position_controller = m_afCommand.enable_position_controller;
        // If the body is kinematic, we just want to control the position
        if (m_bulletRigidBody->isStaticOrKinematicObject() && m_afCommand.enable_position_controller){
            btTransform _Td;
            _Td.setOrigin(btVector3(m_afCommand.px, m_afCommand.py, m_afCommand.pz));
            _Td.setRotation(btQuaternion(m_afCommand.qx, m_afCommand.qy, m_afCommand.qz, m_afCommand.qw));
            m_bulletRigidBody->getMotionState()->setWorldTransform(_Td);
        }
        else{
            if (m_afCommand.enable_position_controller){
                btVector3 _cur_pos, _cmd_pos;
                btQuaternion _cmd_rot_quat = btQuaternion(m_afCommand.qx, m_afCommand.qy, m_afCommand.qz, m_afCommand.qw);
                btMatrix3x3 _cur_rot, _cmd_rot;
                btTransform _b_trans;
                m_bulletRigidBody->getMotionState()->getWorldTransform(_b_trans);

                _cur_pos = _b_trans.getOrigin();
                _cur_rot.setRotation(_b_trans.getRotation());
                _cmd_pos.setValue(m_afCommand.px, m_afCommand.py, m_afCommand.pz);
                if( _cmd_rot_quat.length() < 0.9 || _cmd_rot_quat.length() > 1.1 ){
                    std::cerr << "WARNING: BODY \"" << m_name << "'s\" rotation quaternion command"
                                                                 " not normalized" << std::endl;
                    if (_cmd_rot_quat.length() < 0.1){
                        _cmd_rot_quat.setW(1.0); // Invalid Quaternion
                    }
                }
                _cmd_rot.setRotation(_cmd_rot_quat);

                // Use the internal Cartesian Position Controller
                force = m_controller.computeOutput<btVector3>(_cur_pos, _cmd_pos, dt);
                // Use the internal Cartesian Rotation Controller
                torque = m_controller.computeOutput<btVector3>(_cur_rot, _cmd_rot, dt);
            }
            else{
                force.setValue(m_afCommand.fx, m_afCommand.fy, m_afCommand.fz);
                torque.setValue(m_afCommand.tx, m_afCommand.ty, m_afCommand.tz);
            }

            if (m_bulletRigidBody){
                m_bulletRigidBody->applyCentralForce(force);
                m_bulletRigidBody->applyTorque(torque);
            }
        }
        size_t jntCmdSize = m_afCommand.joint_commands_size;
        if (jntCmdSize > 0){
            size_t jntCmdCnt = m_childAndJointPairs.size() < jntCmdSize ? m_childAndJointPairs.size() : jntCmdSize;
            for (size_t jntIdx = 0 ; jntIdx < jntCmdCnt ; jntIdx++){
                // If the enable position controllers flag is set, run
                // position control on all joints
                // The size of pos ctrl mask can be less than the num of joint commands
                // keep this in check and still read the mask to apply it. Run
                // effort control on the masks not specified
                afJointPtr joint = m_childAndJointPairs[jntIdx].m_childJoint;
                if (m_afCommand.position_controller_mask[jntIdx] == true ){
                    joint->commandPosition(m_afCommand.joint_commands[jntIdx]);
                }
                else{
                    joint->commandEffort(m_afCommand.joint_commands[jntIdx]);
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
    int num_children = m_childAndJointPairs.size();
    if (num_children > 0 && m_afObjectPtr != NULL){
        std::vector<std::string> children_names;

        children_names.resize(num_children);
        for (size_t i = 0 ; i < num_children ; i++){
            children_names[i] = m_childAndJointPairs[i].m_childBody->m_name;
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
    int num_joints = m_childAndJointPairs.size();
    if (num_joints > 0 && m_afObjectPtr != NULL){
        std::vector<std::string> joint_names;
        joint_names.resize(num_joints);
        for (size_t i = 0 ; i < num_joints ; i++){
            joint_names[i] = m_childAndJointPairs[i].m_childJoint->m_name;
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
    int num_jnts = m_childAndJointPairs.size();
    if (num_jnts > 0 && m_afObjectPtr != NULL){
        if(m_joint_positions.size() != num_jnts){
            m_joint_positions.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_joint_positions[i] = m_childAndJointPairs[i].m_childJoint->getPosition();
        }
        m_afObjectPtr->set_joint_positions(m_joint_positions);
    }
#endif
}


///
/// \brief afRigidBody::applyForceAtPoint
/// \param a_forceInWorldFrame
/// \param a_pointInLocalFrame
///
void afRigidBody::applyForceAtPointOnBody(const cVector3d &a_forceInWorld, const cVector3d &a_pointInWorld){
    if (m_bulletRigidBody){
        btTransform T_bodyInWorld = m_bulletRigidBody->getWorldTransform();
        btTransform T_worldInBody = T_bodyInWorld.inverse(); // Invert once here so we dont have to invert multiple times.
        btVector3 worldForce = toBTvec(a_forceInWorld);
        btVector3 worldPoint = toBTvec(a_pointInWorld);
        btVector3 localForce = T_worldInBody.getBasis() * worldForce;
        btVector3 localPoint = T_worldInBody * worldPoint;
        btVector3 localTorque = localPoint.cross(localForce);
        btVector3 worldTorque = T_worldInBody.getBasis() * localTorque;

        m_bulletRigidBody->applyCentralForce(worldForce);
        m_bulletRigidBody->applyTorque(worldTorque);
    }

}

///
/// \brief afRigidBody::setAngle
/// \param angle
///
void afRigidBody::setAngle(double &angle){
    if (m_parentBodies.size() == 0){
        for (size_t jnt = 0 ; jnt < m_childAndJointPairs.size() ; jnt++){
            m_childAndJointPairs[jnt].m_childJoint->commandPosition(angle);
        }

    }
}

///
/// \brief afRigidBody::setAngle
/// \param angles
///
void afRigidBody::setAngle(std::vector<double> &angles){
    if (m_parentBodies.size() == 0){
        double jntCmdSize = m_childAndJointPairs.size() < angles.size() ? m_childAndJointPairs.size() : angles.size();
        for (size_t jntIdx = 0 ; jntIdx < jntCmdSize ; jntIdx++){
            m_childAndJointPairs[jntIdx].m_childJoint->commandPosition(angles[jntIdx]);
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

///
/// \brief afRigidBody::isCommonCollisionGroupIdx
/// \param a_idx
/// \return
///
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
/// \brief afRigidBody::isChild
/// \param a_body
/// \return
///
bool afRigidBody::isChild(btRigidBody *a_body){
    bool isChild = false;
    std::vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_childAndJointPairs.begin() ; cjIt != m_childAndJointPairs.end() ; ++cjIt){
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
    std::vector<afChildJointPair>::iterator cjIt;
    for (cjIt = m_childAndJointPairs.begin() ; cjIt != m_childAndJointPairs.end() ; ++cjIt){
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
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << sb_config_file << std::endl;
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
    YAML::Node softBodyColor = softBodyNode["color"];
    YAML::Node softBodyColorRGBA = softBodyNode["color rgba"];
    YAML::Node softBodyColorComponents = softBodyNode["color components"];
    YAML::Node softBodyConfigData = softBodyNode["config"];
    YAML::Node softBodyRandomizeConstraints = softBodyNode["randomize constraints"];

    YAML::Node cfg_kLST = softBodyConfigData["kLST"];
    YAML::Node cfg_kAST = softBodyConfigData["kAST"];
    YAML::Node cfg_kVST = softBodyConfigData["kVST"];
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
    YAML::Node cfg_flags = softBodyConfigData["flags"];
    YAML::Node cfg_bendingConstraint = softBodyConfigData["bending constraint"];
    YAML::Node cfg_cutting = softBodyConfigData["cutting"];
    YAML::Node cfg_clusters = softBodyConfigData["clusters"];
    YAML::Node cfg_fixed_nodes = softBodyConfigData["fixed nodes"];

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
        pos = toXYZ<btVector3>(&softBodyInertialOffsetPos);
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

    if(softBodyNameSpace.IsDefined()){
        m_namespace = softBodyNameSpace.as<std::string>();
    }
    else{
        m_namespace = mB->getNamespace();
    }

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
        pos = toXYZ<cVector3d>(&softBodyPos);
        setLocalPos(pos);
    }

    if(softBodyRot.IsDefined()){
        double r = softBodyRot["r"].as<double>();
        double p = softBodyRot["p"].as<double>();
        double y = softBodyRot["y"].as<double>();
        rot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(rot);
    }

    cMaterial _mat;
    double _r, _g, _b, _a;
    if(softBodyColorRGBA.IsDefined()){
        _r = softBodyColorRGBA["r"].as<float>();
        _g = softBodyColorRGBA["g"].as<float>();
        _b = softBodyColorRGBA["b"].as<float>();
        _a = softBodyColorRGBA["a"].as<float>();
        _mat.setColorf(_r, _g, _b, _a);
        m_gelMesh.setMaterial(_mat);
        m_gelMesh.setTransparencyLevel(softBodyColorRGBA["a"].as<float>());
    }
    else if(softBodyColorComponents.IsDefined()){
        if (softBodyColorComponents["diffuse"].IsDefined()){
            _r = softBodyColorComponents["diffuse"]["r"].as<float>();
            _g = softBodyColorComponents["diffuse"]["g"].as<float>();
            _b = softBodyColorComponents["diffuse"]["b"].as<float>();
            _mat.m_diffuse.set(_r, _g, _b);
        }
        if (softBodyColorComponents["ambient"].IsDefined()){
            double _level = softBodyColorComponents["ambient"]["level"].as<float>();
            _r *= _level;
            _g *= _level;
            _b *= _level;
            _mat.m_ambient.set(_r, _g, _b);
        }
        if (softBodyColorComponents["specular"].IsDefined()){
            _r = softBodyColorComponents["specular"]["r"].as<float>();
            _g = softBodyColorComponents["specular"]["g"].as<float>();
            _b = softBodyColorComponents["specular"]["b"].as<float>();
            _mat.m_specular.set(_r, _g, _b);
        }
        if (softBodyColorComponents["emission"].IsDefined()){
            _r = softBodyColorComponents["emission"]["r"].as<float>();
            _g = softBodyColorComponents["emission"]["g"].as<float>();
            _b = softBodyColorComponents["emission"]["b"].as<float>();
            _mat.m_emission.set(_r, _g, _b);
        }

        _a = softBodyColorComponents["transparency"].as<float>();
        _mat.setTransparencyLevel(_a);
        m_gelMesh.setMaterial(_mat);
        //        m_gelMesh.setTransparencyLevel(_a);
    }
    else if(softBodyColor.IsDefined()){
        std::vector<double> rgba = m_afWorld->getColorRGBA(softBodyColor.as<std::string>());
        _mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
        m_gelMesh.setMaterial(_mat);
        m_gelMesh.setTransparencyLevel(rgba[3]);
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
        if (cfg_kAST.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kAST = cfg_kAST.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kAST = cfg_kAST.as<double>();
        }
        if (cfg_kVST.IsDefined()){
            btSoftBody::Material *pm = m_bulletSoftBody->appendMaterial();
            pm->m_kVST = cfg_kVST.as<double>();
            m_bulletSoftBody->m_materials[0]->m_kVST = cfg_kVST.as<double>();
        }
        if (cfg_kVCF.IsDefined()) m_bulletSoftBody->m_cfg.kVCF = cfg_kVCF.as<double>();
        if (cfg_kDP.IsDefined()) m_bulletSoftBody->m_cfg.kDP = cfg_kDP.as<double>();
        if (cfg_kDG.IsDefined()) m_bulletSoftBody->m_cfg.kDG = cfg_kDG.as<double>();
        if (cfg_kLF.IsDefined()) m_bulletSoftBody->m_cfg.kLF = cfg_kLF.as<double>();
        if (cfg_kPR.IsDefined()) m_bulletSoftBody->m_cfg.kPR = cfg_kPR.as<double>();
        if (cfg_kVC.IsDefined()) m_bulletSoftBody->m_cfg.kVC = cfg_kVC.as<double>();
        if (cfg_kDF.IsDefined()) m_bulletSoftBody->m_cfg.kDF = cfg_kDF.as<double>();
        if (cfg_kMT.IsDefined()){
            m_bulletSoftBody->m_cfg.kMT = cfg_kMT.as<double>();
            m_bulletSoftBody->setPose(false, true);
        }
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
        if (cfg_flags.IsDefined()){
            m_bulletSoftBody->m_cfg.collisions = cfg_flags.as<int>();
        }
        if (cfg_bendingConstraint.IsDefined()){
            int _bending = cfg_bendingConstraint.as<int>();
            m_bulletSoftBody->generateBendingConstraints(_bending);
        }
        if (cfg_fixed_nodes.IsDefined()){
            for (int i = 0 ; i < cfg_fixed_nodes.size() ; i++){
                int nodeIdx = cfg_fixed_nodes[i].as<int>();
                if (nodeIdx < m_bulletSoftBody->m_nodes.size()){
                    m_bulletSoftBody->setMass(nodeIdx, 0);
                }
            }
        }
        if(cfg_clusters.IsDefined()){
            int num_clusters = cfg_clusters.as<int>();
            m_bulletSoftBody->generateClusters(num_clusters);
        }
    }

    if (softBodyRandomizeConstraints.IsDefined())
        if (softBodyRandomizeConstraints.as<bool>() == true)
            m_bulletSoftBody->randomizeConstraints();

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
double afJointController::computeOutput(double process_val, double set_point, double current_time){
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
void afJointController::boundImpulse(double &effort_cmd){
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
    m_curPos = 0.0; // Initialize Cur Joint position to 0
    m_prevPos = 0.0; // Initialize Prev Joint position to 0
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
    YAML::Node jointERP = jointNode["joint erp"];
    YAML::Node jointCFM = jointNode["joint cfm"];
    YAML::Node jointOffset = jointNode["offset"];
    YAML::Node jointDamping = jointNode["damping"];
    YAML::Node jointStiffness = jointNode["stiffness"];
    YAML::Node jointType = jointNode["type"];
    YAML::Node jointController = jointNode["controller"];
    YAML::Node jointIgnoreInterCollision = jointNode["ignore inter-collision"];

    if (!jointParentName.IsDefined() || !jointChildName.IsDefined()){
        std::cerr << "ERROR: PARENT/CHILD FOR: " << node_name << " NOT DEFINED \n";
        return false;
    }
    m_name = jointName.as<std::string>();
    m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    m_parentName = jointParentName.as<std::string>();
    m_childName = jointChildName.as<std::string>();
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
    m_jointType = JointType::revolute;
    m_jointDamping = 0.0; // Initialize damping to 0

    afRigidBodyPtr afBodyA, afBodyB;

    m_mB = mB;

    // First we should search in the local MultiBody space and if we don't find the body.
    // On then we find the world space

    afBodyA = mB->getAFRigidBodyLocal(mB->getNamespace() + m_parentName, true);
    afBodyB = mB->getAFRigidBodyLocal(mB->getNamespace() + m_childName, true);

    if (!afBodyA){
        afBodyA = m_afWorld->getAFRigidBody(mB->getNamespace() + m_parentName + name_remapping, true);
    }
    if (!afBodyB){
        afBodyB = m_afWorld->getAFRigidBody(mB->getNamespace() + m_childName + name_remapping, true);
    }

    bool _ignore_inter_collision = true;

    // If we couldn't find the body with name_remapping, it might have been
    // Defined in another ambf file. Search without name_remapping string
    if(afBodyA == NULL){
        afBodyA = m_afWorld->getAFRigidBody(m_parentName, true);
        // If any body is still not found, print error and ignore joint
        if (afBodyA == NULL){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" PARENT BODY \"" << m_parentName <<
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
        afBodyB = m_afWorld->getAFRigidBody(m_childName, true);
        // If any body is still not found, print error and ignore joint
        if (afBodyB == NULL){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" NOT FOUND" << std::endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ((!strcmp(afBodyB->m_name.c_str(), "world") == 0)
                &&(!strcmp(afBodyB->m_name.c_str(), "World") == 0)
                &&(!strcmp(afBodyB->m_name.c_str(), "WORLD") == 0)){
            std::cerr <<"INFO: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" FOUND IN ANOTHER AMBF CONFIG," << std::endl;
        }
    }

    else{
        m_rbodyA = afBodyA->m_bulletRigidBody;
        m_rbodyB = afBodyB->m_bulletRigidBody;
    }

    if (jointParentPivot.IsDefined() & jointParentAxis.IsDefined() & jointChildPivot.IsDefined() & jointChildAxis.IsDefined()){
        m_pvtA = toXYZ<btVector3>( &jointParentPivot);
        m_axisA = toXYZ<btVector3>( &jointParentAxis);
        m_pvtB = toXYZ<btVector3>( &jointChildPivot);
        m_axisB = toXYZ<btVector3>( &jointChildAxis);

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
        YAML::Node jointXYZ = jointOrigin["position"];
        YAML::Node jointRPY = jointOrigin["orientation"];
        if (jointXYZ.IsDefined()){
            pos = toXYZ<btVector3>(&jointXYZ);
            T_j_p.setOrigin(pos);
        }
        if (jointRPY.IsDefined()){
            quat.setEulerZYX(jointRPY['y'].as<double>(),
                    jointRPY['p'].as<double>(),
                    jointRPY['r'].as<double>());
            T_j_p.setRotation(quat);
        }

        if (jointAxis.IsDefined()){
            joint_axis = toXYZ<btVector3>(&jointAxis);
        }
    }
    else{
        std::cerr << "ERROR: JOINT CONFIGURATION FOR: " << node_name << " NOT DEFINED \n";
        return false;
    }

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
    }

    // Bullet takes the x axis as the default for prismatic joints
    btVector3 ax_cINp;

    if (jointType.IsDefined()){
        if ((strcmp(jointType.as<std::string>().c_str(), "hinge") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "revolute") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "continuous") == 0)){
            m_jointType = JointType::revolute;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "slider") == 0)
                 || (strcmp(jointType.as<std::string>().c_str(), "prismatic") == 0)){
            m_jointType = JointType::prismatic;
            // For this case constraint axis is the world x axis
            ax_cINp.setValue(1, 0, 0);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "fixed") == 0)){
            m_jointType = JointType::fixed;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "spring") == 0)){
            m_jointType = JointType::linear_spring;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "linear spring") == 0)){
            m_jointType = JointType::linear_spring;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "torsion spring") == 0)){
            m_jointType = JointType::torsion_spring;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "torsional spring") == 0)){
            m_jointType = JointType::torsion_spring;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "angular spring") == 0)){
            m_jointType = JointType::torsion_spring;
            // For this case constraint axis is the world z axis
            ax_cINp.setValue(0, 0, 1);
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "p2p") == 0)){
            m_jointType = JointType::p2p;
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
    if (m_jointType == JointType::revolute){
#ifdef USE_PIVOT_AXIS_METHOD
        m_btConstraint = new btHingeConstraint(*m_rbodyA, *m_rbodyB, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
#else
        m_hinge = new btHingeConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB, true);
        m_hinge->setParam(BT_CONSTRAINT_ERP, _jointERP);
        m_hinge->setParam(BT_CONSTRAINT_CFM, _jointCFM);
#endif
        // Don't enable motor yet, only enable when set position is called
        // this keeps the joint behave freely when it's launched
        if(jointMaxMotorImpulse.IsDefined()){
            m_controller.max_impulse = jointMaxMotorImpulse.as<double>();
            m_hinge->setMaxMotorImpulse(m_controller.max_impulse);
        }

        if(jointLimits.IsDefined()){
            m_hinge->setLimit(m_lowerLimit, m_upperLimit);
        }

        m_btConstraint = m_hinge;
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        afBodyA->addChildJointPair(afBodyB, this);
    }
    // If the joint is slider, prismatic or linear
    else if (m_jointType == JointType::prismatic){
        m_slider = new btSliderConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB, true);
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

        m_btConstraint = m_slider;
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        afBodyA->addChildJointPair(afBodyB, this);
    }

    // If the joint is a spring
    else if (m_jointType == JointType::linear_spring || m_jointType == JointType::torsion_spring){
        m_spring = new btGeneric6DofSpringConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB, true);

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

        if (m_jointType == JointType::linear_spring){
            _axisNumber = 2;
        }
        else if (m_jointType == JointType::torsion_spring){
            _axisNumber = 5;
        }

        double _low, _high;
        if (jointLimits.IsDefined()){

            _high =  jointLimits["high"].as<double>();
            _low = jointLimits["low"].as<double>();

            // Somehow bullets springs limits for rotational joints are inverted.
            // So handle them internally rather than breaking AMBF description specificaiton
            if (m_jointType == JointType::torsion_spring){
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

        if (jointNode["equiblirium point"].IsDefined()){
            double _equiblirium = jointNode["equiblirium point"].as<double>();
            // The equiblirium offset if also inverted for torsional springs
            // Fix it internally rather than breaking AMBF description specificaiton
            if (m_jointType == JointType::torsion_spring){
                _equiblirium = - _equiblirium;
            }
            m_spring->setEquilibriumPoint(_axisNumber, _equiblirium);
        }
        else{
            m_spring->setEquilibriumPoint(_axisNumber, _low + ((_high - _low) / 2));
        }

        // Calculcated a stiffness value based on the masses of connected bodies.
        double _stiffness = 10 * afBodyA->getMass() + afBodyB->getMass();
        // If stiffness defined, override the above value
        if (jointStiffness.IsDefined()){
            _stiffness = jointStiffness.as<double>();
        }
        m_spring->setStiffness(_axisNumber, _stiffness);

        m_spring->setDamping(_axisNumber, m_jointDamping);

        m_spring->setParam(BT_CONSTRAINT_STOP_ERP, _jointERP, _axisNumber);
        m_spring->setParam(BT_CONSTRAINT_CFM, _jointCFM, _axisNumber);

        m_btConstraint = m_spring;
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);

        afBodyA->addChildJointPair(afBodyB, this);
    }
    else if (m_jointType == JointType::p2p){
        // p2p joint doesnt concern itself with rotations, its set using just the pivot information
        m_p2p = new btPoint2PointConstraint(*m_rbodyA, *m_rbodyB, m_pvtA, m_pvtB);
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
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        afBodyA->addChildJointPair(afBodyB, this);
    }
    else if (m_jointType == JointType::fixed){
        m_btConstraint = new btFixedConstraint(*m_rbodyA, *m_rbodyB, frameA, frameB);
        //        ((btFixedConstraint *) m_btConstraint)->setParam(BT_CONSTRAINT_ERP, _jointERP);
        //        ((btFixedConstraint *) m_btConstraint)->setParam(BT_CONSTRAINT_CFM, _jointCFM);
        m_afWorld->s_bulletWorld->m_bulletWorld->addConstraint(m_btConstraint, _ignore_inter_collision);
        afBodyA->addChildJointPair(afBodyB, this);
    }
    return true;
}


void afJoint::remove(){
    if (m_btConstraint){
        m_afWorld->s_bulletWorld->m_bulletWorld->removeConstraint(m_btConstraint);
    }
}


///
/// \brief afJoint::applyDamping
///
void afJoint::applyDamping(const double &dt){
    // First lets configure what type of joint is this.
    m_prevPos = m_curPos;
    m_curPos = getPosition();
    double effort = - m_jointDamping * (m_curPos - m_prevPos)/dt;
    commandEffort(effort);
}

///
/// \brief afJoint::command_position
/// \param cmd
///
void afJoint::commandPosition(double &position_cmd){
    // The torque commands disable the motor, so double check and re-enable the motor
    // if it was set to be enabled in the first place
    if (m_enableActuator){
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
    if (m_jointType == JointType::revolute || m_jointType == JointType::torsion_spring){
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
        m_btConstraint->getRigidBodyA().applyTorque(-hingeAxisInWorld * cmd);
        m_btConstraint->getRigidBodyB().applyTorque(hingeAxisInWorld * cmd);
    }
    else if (m_jointType == JointType::prismatic || m_jointType == JointType::linear_spring){
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        const btVector3 sliderAxisInWorld = trA.getBasis()*m_axisA;
        const btVector3 relPos(0,0,0);
        m_rbodyA->applyForce(-sliderAxisInWorld * cmd, relPos);
        m_rbodyB->applyForce(sliderAxisInWorld * cmd, relPos);
    }
}

///
/// \brief afJoint::getPosition
/// \return
///
double afJoint::getPosition(){
    if (m_jointType == JointType::revolute)
        return m_hinge->getHingeAngle();
    else if (m_jointType == JointType::prismatic)
        return m_slider->getLinearPos();
    else if (m_jointType == JointType::fixed)
        return 0;
    else if (m_jointType == JointType::linear_spring){
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
    else if (m_jointType == JointType::torsion_spring){
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
/// \brief afRayTracerSensor
/// \param a_afWorld
///
afRayTracerSensor::afRayTracerSensor(afWorldPtr a_afWorld): afSensor(a_afWorld){

}

///
/// \brief afProximitySensor::loadSensor
/// \param sensor_config_file
/// \param node_name
/// \param name_remapping_idx
/// \return
///
bool afRayTracerSensor::loadSensor(std::string sensor_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(sensor_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO SENSOR CONFIG: " << sensor_config_file << std::endl;
        return 0;
    }
    if (baseNode.IsNull()) return false;

    YAML::Node baseSensorNode = baseNode[node_name];
    return loadSensor(&baseSensorNode, node_name, mB, name_remapping);
}

///
/// \brief afProximitySensor::loadSensor
/// \param sensor_node
/// \param node_name
/// \param name_remapping_idx
/// \return
///
bool afRayTracerSensor::loadSensor(YAML::Node *sensor_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node sensorNode = *sensor_node;
    if (sensorNode.IsNull()){
        std::cerr << "ERROR: SENSOR'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool _is_valid = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node sensorParentName = sensorNode["parent"];
    YAML::Node sensorName = sensorNode["name"];
    YAML::Node sensorLocation = sensorNode["location"];
    YAML::Node sensorDirection = sensorNode["direction"];
    YAML::Node sensorRange = sensorNode["range"];
    YAML::Node sensorVisible = sensorNode["visible"];
    YAML::Node sensorVisibleSize = sensorNode["visible size"];

    std::string _parent_name;
    if (sensorParentName.IsDefined()){
        _parent_name = sensorParentName.as<std::string>();
    }
    else{
        _is_valid = false;
    }

    m_name = sensorName.as<std::string>();
    m_location = toXYZ<cVector3d>(&sensorLocation);
    m_direction = toXYZ<cVector3d>(&sensorDirection);
    m_range = sensorRange.as<double>();

    if (m_range < 0.0){
        std::cerr << "ERROR! SENSOR RANGE CANNOT BE NEGATIVE" << std::endl;
        return 0;
    }

    if (sensorVisible.IsDefined()){
        m_showSensor = sensorVisible.as<bool>();
    }
    else{
        m_showSensor = false;
    }

    // Chosed an random scale to divide the visual radius of the sensor markers
    m_visibilitySphereRadius = m_range / 8;
    if (sensorVisibleSize.IsDefined()){
        m_visibilitySphereRadius = sensorVisibleSize.as<double>();
    }

    if (m_showSensor){
        enableVisualization();
    }

    // First search in the local space.
    m_parentBody = mB->getAFRigidBodyLocal(_parent_name);

    if(!m_parentBody){
        m_parentBody = m_afWorld->getAFRigidBody(_parent_name + name_remapping);
    }

    if (m_parentBody == NULL){
        std::cerr << "ERROR: SENSOR'S "<< _parent_name + name_remapping << " NOT FOUND, IGNORING SENSOR\n";
        return 0;
    }
    else{
        m_parentBody->addAFSensor(this);
    }

    m_rayFromLocal = m_location;
    m_rayToLocal = m_rayFromLocal + (m_direction * m_range);

    return true;
}


///
/// \brief afSensor::processSensor
///
void afRayTracerSensor::updateSensor(){
    btVector3 _rayFromWorld, _rayToWorld;
    // Transform of World in Body
    cTransform T_bInw = m_parentBody->getLocalTransform();
    _rayFromWorld = toBTvec(T_bInw *  m_rayFromLocal);
    _rayToWorld = toBTvec(T_bInw *  m_rayToLocal);

    // Check for global flag for debug visibility of this sensor
    if (m_showSensor){
        m_fromSphereMesh->setLocalPos(toCvec(_rayFromWorld) );
        m_toSphereMesh->setLocalPos(toCvec(_rayToWorld) );
    }

    btCollisionWorld::ClosestRayResultCallback _rayCallBack(_rayFromWorld, _rayToWorld);
    m_afWorld->s_bulletWorld->m_bulletWorld->rayTest(_rayFromWorld, _rayToWorld, _rayCallBack);
    if (_rayCallBack.hasHit()){
        if (m_showSensor){
            m_hitSphereMesh->setLocalPos(toCvec(_rayCallBack.m_hitPointWorld));
            m_hitSphereMesh->setShowEnabled(true);
        }
        m_triggered = true;
        if (_rayCallBack.m_collisionObject->getInternalType()
                == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
            m_sensedRigidBody = (btRigidBody*)btRigidBody::upcast(_rayCallBack.m_collisionObject);
            m_sensedBodyType = RIGID_BODY;
        }
        else if (_rayCallBack.m_collisionObject->getInternalType()
                 == btCollisionObject::CollisionObjectTypes::CO_SOFT_BODY){
            btSoftBody* _sensedSoftBody = (btSoftBody*)btSoftBody::upcast(_rayCallBack.m_collisionObject);

            // Now get the node which is closest to the hit point;
            btVector3 _hitPoint = _rayCallBack.m_hitPointWorld;
            int _sensedSoftBodyNodeIdx = -1;
            int _sensedSoftBodyFaceIdx = -1;

            double _maxDistance = 0.1;
            for (int faceIdx = 0 ; faceIdx < _sensedSoftBody->m_faces.size() ; faceIdx++){
                btVector3 _faceCenter(0, 0, 0);
                // Iterate over all the three nodes of the face to find the this centroid to the hit
                // point in world to store this face as the closest face
                for (int nIdx = 0 ; nIdx < 3 ; nIdx++){
                    _faceCenter += _sensedSoftBody->m_faces[faceIdx].m_n[nIdx]->m_x;
                }
                _faceCenter /= 3;
                if ( (_hitPoint - _faceCenter).length() < _maxDistance ){
                    _sensedSoftBodyFaceIdx = faceIdx;
                    _maxDistance = (_hitPoint - _faceCenter).length();
                }

            }
            // If sensedBodyFaceIdx is not -1, we sensed some face. Lets capture it
            if (_sensedSoftBodyFaceIdx > -1){
                m_sensedSoftBodyFaceIdx = _sensedSoftBodyFaceIdx;
                m_sensedSoftBodyFace = &_sensedSoftBody->m_faces[_sensedSoftBodyFaceIdx];
                m_sensedSoftBody = _sensedSoftBody;
                m_sensedBodyType = SOFT_BODY;
            }
            // Reset the maxDistance for node checking
            _maxDistance = 0.1;
            // Iterate over all the softbody nodes to figure out which node is closest to the
            // hit point in world
            for (int nodeIdx = 0 ; nodeIdx < _sensedSoftBody->m_nodes.size() ; nodeIdx++){
                if ( (_hitPoint - _sensedSoftBody->m_nodes[nodeIdx].m_x).length() < _maxDistance ){
                    _sensedSoftBodyNodeIdx = nodeIdx;
                    _maxDistance = (_hitPoint - _sensedSoftBody->m_nodes[nodeIdx].m_x).length();
                }
            }
            // If sensedBodyNodeIdx is not -1, we sensed some node. Lets capture it
            if (_sensedSoftBodyNodeIdx > -1){
                m_sensedSoftBodyNodeIdx = _sensedSoftBodyNodeIdx;
                m_sensedSoftBodyNode = &_sensedSoftBody->m_nodes[_sensedSoftBodyNodeIdx];
                m_sensedSoftBody = _sensedSoftBody;
                m_sensedBodyType = SOFT_BODY;
            }
        }
        m_depthFraction = (1.0 - _rayCallBack.m_closestHitFraction);
        m_contactNormal = toCvec(_rayCallBack.m_hitNormalWorld);
        m_sensedLocationWorld = toCvec(_rayCallBack.m_hitPointWorld);
    }
    else{
        if(m_showSensor){
            m_hitSphereMesh->setShowEnabled(false);
        }
        m_triggered = false;
        m_depthFraction = 0;
    }
}

///
/// \brief afRayTracerSensor::visualize
///
void afRayTracerSensor::enableVisualization(){
    m_hitSphereMesh = new cMesh();
    m_fromSphereMesh = new cMesh();
    m_toSphereMesh = new cMesh();
    cCreateSphere(m_hitSphereMesh, m_visibilitySphereRadius);
    cCreateSphere(m_fromSphereMesh, m_visibilitySphereRadius);
    cCreateSphere(m_toSphereMesh, m_visibilitySphereRadius);
    m_afWorld->s_bulletWorld->addChild(m_hitSphereMesh);
    m_afWorld->s_bulletWorld->addChild(m_fromSphereMesh);
    m_afWorld->s_bulletWorld->addChild(m_toSphereMesh);
    m_hitSphereMesh->m_material->setPinkHot();
    m_fromSphereMesh->m_material->setRed();
    m_toSphereMesh->m_material->setGreen();
    m_hitSphereMesh->setShowEnabled(false);

    m_fromSphereMesh->setUseDisplayList(true);
    m_toSphereMesh->setUseDisplayList(true);
    m_hitSphereMesh->setUseDisplayList(true);

    m_fromSphereMesh->markForUpdate(false);
    m_toSphereMesh->markForUpdate(false);
    m_hitSphereMesh->markForUpdate(false);

    m_hitNormalMesh = new cMesh();
    cCreateArrow(m_hitNormalMesh, m_visibilitySphereRadius*10,
                 m_visibilitySphereRadius*0.5,
                 m_visibilitySphereRadius*1,
                 m_visibilitySphereRadius*0.8,
                 false);
    m_afWorld->s_bulletWorld->addChild(m_hitNormalMesh);
    m_hitNormalMesh->m_material->setGreenForest();
    m_hitNormalMesh->setShowEnabled(false);
    m_hitNormalMesh->setUseDisplayList(true);
    m_hitNormalMesh->markForUpdate(false);
}


///
/// \brief afProximitySensor::afProximitySensor
/// \param a_afWorld
///
afProximitySensor::afProximitySensor(afWorldPtr a_afWorld): afRayTracerSensor(a_afWorld){
    m_afWorld = a_afWorld;
    m_sensorType = afSensorType::proximity;
}


///
/// \brief afResistanceSensor::afResistanceSensor
/// \param a_afWorld
///
afResistanceSensor::afResistanceSensor(afWorld* a_afWorld): afRayTracerSensor(a_afWorld){
    m_lastContactPosInWorld.set(0,0,0);
    m_curContactPosInWorld.set(0,0,0);
    m_staticContactFriction = 0;
    m_dynamicFriction = 0;

    m_bodyAContactPointLocal.set(0,0,0);
    m_bodyBContactPointLocal.set(0,0,0);

    m_tangentialError.set(0,0,0);
    m_tangentialErrorLast.set(0,0,0);

    m_contactPointsValid = false;
    m_contactArea = 0.1;
    m_staticContactDamping = 0.1;

    m_contactNormalStiffness = 0;
    m_contactNormalDamping = 0;

    m_sensorType = afSensorType::resistance;
}


///
/// \brief afResistanceSensor::loadSensor
/// \param sensor_node
/// \param node_name
/// \param mB
/// \param name_remapping_idx
/// \return
///
bool afResistanceSensor::loadSensor(YAML::Node *sensor_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx){
    bool result = false;
    result = afRayTracerSensor::loadSensor(sensor_node, node_name, mB, name_remapping_idx);

    if (result){

        YAML::Node bodyResistiveFriction = (*sensor_node)["friction"];
        YAML::Node sensorContactArea = (*sensor_node)["contact area"];
        YAML::Node sensorContactStiffness = (*sensor_node)["contact stiffness"];
        YAML::Node sensorContactDamping = (*sensor_node)["contact damping"];

        if (bodyResistiveFriction["static"].IsDefined()){
            m_staticContactFriction = bodyResistiveFriction["static"].as<double>();
        }

        if (bodyResistiveFriction["damping"].IsDefined()){
            m_staticContactDamping = bodyResistiveFriction["damping"].as<double>();
        }

        if (bodyResistiveFriction["dynamic"].IsDefined()){
            m_dynamicFriction = bodyResistiveFriction["dynamic"].as<double>();
        }

        if (bodyResistiveFriction["variable"].IsDefined()){
            m_useVariableCoeff = bodyResistiveFriction["variable"].as<bool>();
        }

        if (sensorContactArea.IsDefined()){
            m_contactArea = sensorContactArea.as<double>();
        }

        if (sensorContactStiffness.IsDefined()){
            m_contactNormalStiffness = sensorContactStiffness.as<double>();
        }

        if (sensorContactDamping.IsDefined()){
            m_contactNormalDamping = sensorContactDamping.as<double>();
        }
    }
    return result;
}


///
/// \brief afResistanceSensor::updateSensor
///
void afResistanceSensor::updateSensor(){
    // Let's update the RayTracer Sensor First
    afRayTracerSensor::updateSensor();

    // Find the rigid or softbody that this sensor made a contact with
    if (isTriggered()){
        if (m_showSensor){
            m_hitNormalMesh->setLocalPos(getSensedPoint());
            m_hitNormalMesh->setLocalRot(afUtils::getRotBetweenVectors<cMatrix3d, cVector3d>(cVector3d(0,0,1), m_contactNormal));
            m_hitNormalMesh->setShowEnabled(true);
        }

        btVector3 F_s_w(0,0,0); // Due to "stick" friction
        btVector3 F_d_w(0,0,0); // Due to "sliding" friction
        btVector3 F_n_w(0,0,0); // Force normal to contact point.

        if (getSensedBodyType() == SensedBodyType::RIGID_BODY){

            // Get the fraction of contact point penetration from the range of the sensor
            // Subscript (a) represents parent body, which is the parent of this sensor
            // Subscript (b) represents the sensed body, which is in contact with the resistive sensor
            // Subscript (c) represents contact point
            // Subscript (w) represents world
            btTransform T_aINw = getParentBody()->m_bulletRigidBody->getWorldTransform();
            btTransform T_wINa = T_aINw.inverse(); // Invert once to save computation later
            btVector3 P_cINw = toBTvec(getSensedPoint());
            btVector3 P_cINa = T_wINa * P_cINw;
            btVector3 vel_aINw = getParentBody()->m_bulletRigidBody->getLinearVelocity();
            btVector3 omega_aINw = getParentBody()->m_bulletRigidBody->getAngularVelocity();
            btVector3 N_a = toBTvec(m_direction);
            btVector3 N_aINw = T_aINw.getBasis() * N_a;

            btTransform T_bINw = getSensedRigidBody()->getWorldTransform();
            btTransform T_wINb = T_bINw.inverse(); // Invert once to save computation later
            btVector3 P_cINb = T_wINb * P_cINw;
            btVector3 vel_bINw = getSensedRigidBody()->getLinearVelocity();
            btVector3 omega_bINw = getSensedRigidBody()->getAngularVelocity();
            btVector3 N_bINw = toBTvec(m_contactNormal);
            btVector3 N_b = T_wINb.getBasis() * N_bINw;

            double depthFractionLast = m_depthFraction;

            if (m_depthFraction < 0 || m_depthFraction > 1){
                std::cerr << "LOGIC ERROR! "<< m_name <<" Depth Fraction is " << m_depthFraction <<
                             ". It should be between [0-1]" << std::endl;
                std::cerr << "Ray Start: "<< m_rayFromLocal <<"\nRay End: " << m_rayToLocal <<
                             "\nSensed Point: " << toCvec(P_cINa) << std::endl;
                std::cerr << "----------\n";
                m_depthFraction = 0;
            }

            // First calculate the normal contact force
            btVector3 F_n_a = ((m_contactNormalStiffness * m_depthFraction)
                               + m_contactNormalDamping * (m_depthFraction - depthFractionLast)) * (N_a);
            F_n_w = T_aINw.getBasis() * F_n_a;

            double coeffScale = 1;
            if (m_useVariableCoeff){
                coeffScale = F_n_w.length();
            }

            if(m_contactPointsValid){
                btVector3 P_aINw = T_aINw * toBTvec(m_bodyAContactPointLocal);
                btVector3 P_bINw = T_bINw * toBTvec(m_bodyBContactPointLocal);
                btVector3 error;
                error = P_aINw - P_bINw;
                btVector3 orthogonalError = N_aINw.cross(error);
                btVector3 errorDir = orthogonalError.cross(N_aINw);
                if (errorDir.length() > 0.0001){
                    errorDir.normalize();
                }
                double errorMag = errorDir.dot(error);
                if (errorMag < 0.0){
                    std::cerr << errorMag << std::endl;
                }

                btVector3 tangentialError = errorMag * errorDir;
                btVector3 tangentialErrorLast = toBTvec(m_tangentialErrorLast);
                m_tangentialErrorLast = m_tangentialError;
                m_tangentialError = toCvec(tangentialError);

                if (tangentialError.length() > 0.0 && tangentialError.length() <= m_contactArea){
                    F_s_w = m_staticContactFriction * coeffScale * tangentialError +
                            m_staticContactDamping * (tangentialError - tangentialErrorLast);
                }
                else{
                    m_contactPointsValid = false;
                }

                //                std::cerr << "F Static: " << F_static << std::endl;
                //                std::cerr << "F Normal: " << F_normal << std::endl;
                //                std::cerr << "Depth Ra: " << m_depthFraction << std::endl;
                //                std::cerr << "------------\n";
            }
            else{
                m_bodyAContactPointLocal = toCvec(T_wINa * toBTvec(getSensedPoint()));
                m_bodyBContactPointLocal = toCvec(T_wINb * toBTvec(getSensedPoint()));
                m_contactPointsValid = true;
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
            //                std::cerr << staticForce << std::endl;

            btVector3 Fw = F_s_w + F_d_w + F_n_w;

            // Lets find the added torque at the point where the force is applied

            btVector3 Fa = T_wINa.getBasis() * (-Fw);
            btVector3 Tau_a = P_cINa.cross(Fa * getParentBody()->m_bulletRigidBody->getLinearFactor());
            btVector3 Tau_aINw = T_aINw.getBasis() * Tau_a;

            btVector3 Fb = T_wINb.getBasis() * Fw;
            btVector3 Tau_b = P_cINb.cross(Fb * getSensedRigidBody()->getLinearFactor());
            btVector3 Tau_bINw = T_bINw.getBasis() * Tau_b;

            // Nows lets add the action and reaction friction forces to both the bodies
            getParentBody()->m_bulletRigidBody->applyCentralForce(-Fw);
            getParentBody()->m_bulletRigidBody->applyTorque(Tau_aINw);

            getSensedRigidBody()->applyCentralForce(Fw);
            getSensedRigidBody()->applyTorque(Tau_bINw);
        }

        else if (getSensedBodyType() == SensedBodyType::SOFT_BODY){

        }
    }
    else{
        m_contactPointsValid = false;
        m_firstTrigger = true;

        if(m_showSensor){
            m_hitNormalMesh->setShowEnabled(false);
        }
    }
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

    m_pickSphere = new cMesh();
    cCreateSphere(m_pickSphere, 0.02);
    m_pickSphere->m_material->setPinkHot();
    m_pickSphere->setUseDisplayList(true);
    m_pickSphere->markForUpdate(false);
    m_pickSphere->setLocalPos(0,0,0);
    m_pickSphere->setShowEnabled(false);
    s_bulletWorld->addChild(m_pickSphere);
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
/// \brief afWorld::resetWorld
/// \param reset_time
///
void afWorld::resetWorld(bool reset_time){
    pausePhysics(true);

    afRigidBodyMap::iterator _rbIt;

    for (_rbIt = m_afRigidBodyMap.begin() ; _rbIt != m_afRigidBodyMap.end() ; _rbIt++){
        afRigidBodyPtr afRB = (_rbIt->second);
        btRigidBody* rB = afRB->m_bulletRigidBody;
        btVector3 zero(0, 0, 0);
        rB->clearForces();
        rB->setLinearVelocity(zero);
        rB->setAngularVelocity(zero);
        cTransform c_T(afRB->getInitialPosition(), afRB->getInitialRotation());
        btTransform bt_T = afUtils::convertDataType<btTransform, cTransform>(c_T);
        rB->setWorldTransform(bt_T);
    }

    if (reset_time){
//        s_bulletWorld->setSimulationTime(0.0);
    }

    pausePhysics(false);
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

    if (usePlanes){
        // bullet static walls and ground
        cBulletStaticPlane* bulletGround;

        cBulletStaticPlane* bulletBoxWall[4];

        bulletBoxWall[0] = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, -1.0, 0.0), -0.5 * box_w);
        bulletBoxWall[1] = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, 1.0, 0.0), -0.5 * box_w);
        bulletBoxWall[2] = new cBulletStaticPlane(s_bulletWorld, cVector3d(-1.0, 0.0, 0.0), -0.5 * box_l);
        bulletBoxWall[3] = new cBulletStaticPlane(s_bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.5 * box_l);

        cVector3d nz(0.0, 0.0, 1.0);
        cMaterial matPlane;
        matPlane.setWhiteIvory();
        matPlane.setShininess(1);
        cVector3d planeNorm;
        cMatrix3d planeRot;

        double dim1, dim2;

        for (int i = 0 ; i < 4 ; i++){
            cBulletStaticPlane* wall = bulletBoxWall[i];
            planeNorm = cCross(wall->getPlaneNormal(), nz);
            planeRot.setAxisAngleRotationDeg(planeNorm, 90);
            if (i < 2){
                dim1 = box_l; dim2 = box_h;
            }
            else{
                dim1 = box_h; dim2 = box_w;
            }
            cCreatePlane(wall, dim1, dim2,
                         wall->getPlaneConstant() * wall->getPlaneNormal(), planeRot);
            wall->setMaterial(matPlane);
            if (i == 0) wall->setTransparencyLevel(0.3, true, true);
            else wall->setTransparencyLevel(0.5, true, true);

            s_bulletWorld->addChild(wall);
        }


        //////////////////////////////////////////////////////////////////////////
        // GROUND
        //////////////////////////////////////////////////////////////////////////

        // create ground plane
        bulletGround = new cBulletStaticPlane(s_bulletWorld, cVector3d(0.0, 0.0, 1.0), -0.5 * box_h);

        // add plane to world as we will want to make it visibe
        s_bulletWorld->addChild(bulletGround);

        // create a mesh plane where the static plane is located
        cCreatePlane(bulletGround, box_l + 0.4, box_w + 0.8,
                     bulletGround->getPlaneConstant() * bulletGround->getPlaneNormal());
        bulletGround->computeAllNormals();

        // define some material properties and apply to mesh
        bulletGround->m_material->m_emission.setGrayLevel(0.3);
        bulletGround->m_material->setWhiteAzure();
        bulletGround->m_bulletRigidBody->setFriction(0.9);
        bulletGround->m_bulletRigidBody->setRollingFriction(0.05);
        bulletGround->m_bulletRigidBody->setDamping(0.5, 0.1);
    }
    else{

        cBulletBox* boundaryWalls[5];

        box_l = box_l + thickness;
        box_w = box_w + thickness;
        box_h = box_h + thickness;

        boundaryWalls[0] = new cBulletBox(s_bulletWorld, box_l, thickness, box_h); // Right Wall
        boundaryWalls[1] = new cBulletBox(s_bulletWorld, box_l, thickness, box_h); // Left Wall
        boundaryWalls[2] = new cBulletBox(s_bulletWorld, thickness, box_w, box_h); // Back Wall
        boundaryWalls[3] = new cBulletBox(s_bulletWorld, thickness, box_w, box_h); // Front Wall
        boundaryWalls[4] = new cBulletBox(s_bulletWorld, box_l + 0.5, box_w + 0.5, thickness); // Front Wall

        boundaryWalls[0]->setLocalPos(0, box_w/2, 0);
        boundaryWalls[1]->setLocalPos(0, -box_w/2, 0);
        boundaryWalls[2]->setLocalPos(-box_l/2, 0, 0);
        boundaryWalls[3]->setLocalPos(box_l/2, 0, 0);
        boundaryWalls[4]->setLocalPos(0, 0, -box_h/2);

        for(int i = 0 ; i < 5 ; i++){
            boundaryWalls[i]->m_material->setWhiteIvory();
            boundaryWalls[i]->setTransparencyLevel(0.5, true, true);
            boundaryWalls[i]->m_material->setShininess(1);
            s_bulletWorld->addChild(boundaryWalls[i]);
            boundaryWalls[i]->setMass(0.0);
            boundaryWalls[i]->estimateInertia();
            boundaryWalls[i]->buildDynamicModel();
        }

        // Make the front wall more transparent
        boundaryWalls[3]->setTransparencyLevel(0.3, true, true);


        // define some material properties and apply to mesh
        boundaryWalls[4]->m_material->m_emission.setGrayLevel(0.3);
        boundaryWalls[4]->m_material->setWhiteAzure();
        boundaryWalls[4]->setTransparencyLevel(0.5, true, true);
        boundaryWalls[4]->m_bulletRigidBody->setFriction(0.5);
        boundaryWalls[4]->m_bulletRigidBody->setRollingFriction(0.05);
        boundaryWalls[4]->m_bulletRigidBody->setDamping(0.5, 0.1);
    }
}


///
/// \brief afWorld::load_world
/// \param a_world_config
/// \return
///
bool afWorld::loadWorld(std::string a_world_config, bool showGUI){
    if (a_world_config.empty()){
        a_world_config = getWorldConfig();
    }
    YAML::Node worldNode;
    try{
        worldNode = YAML::LoadFile(a_world_config);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << a_world_config << std::endl;
        return 0;
    }

    YAML::Node worldEnclosureData = worldNode["enclosure"];
    YAML::Node worldLightsData = worldNode["lights"];
    YAML::Node worldCamerasData = worldNode["cameras"];
    YAML::Node worldNamespace = worldNode["namespace"];

    if (worldNamespace.IsDefined()){
        m_world_namespace = worldNamespace.as<std::string>();
    }
    else{
        // Use the default namespace
        m_world_namespace = "/ambf/env/";
    }

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
            if (lightPtr->loadLight(&lightNode, light_name, this)){
                addAFLight(lightPtr, light_name);
                lightPtr->afObjectCreate(lightPtr->m_name,
                                         lightPtr->getNamespace(),
                                         lightPtr->getMinPublishFrequency(),
                                         lightPtr->getMaxPublishFrequency());
            }
        }
    }

    if (m_afLightMap.size() == 0){
        // No Valid Lights defined, so use the default light
        afLightPtr lightPtr = new afLight(this);
        if (lightPtr->createDefaultLight()){
            addAFLight(lightPtr, "default_light");
            lightPtr->afObjectCreate(lightPtr->m_name,
                                     lightPtr->getNamespace(),
                                     lightPtr->getMinPublishFrequency(),
                                     lightPtr->getMaxPublishFrequency());
        }
    }

    if (showGUI){
        if (worldCamerasData.IsDefined()){
            for (size_t idx = 0 ; idx < worldCamerasData.size(); idx++){
                std::string camera_name = worldCamerasData[idx].as<std::string>();
                afCameraPtr cameraPtr = new afCamera(this);
                YAML::Node cameraNode = worldNode[camera_name];
                if (cameraPtr->loadCamera(&cameraNode, camera_name, this)){
                    addAFCamera(cameraPtr, camera_name);
                    cameraPtr->afObjectCreate(cameraPtr->m_name,
                                              cameraPtr->getNamespace(),
                                              cameraPtr->getMinPublishFrequency(),
                                              cameraPtr->getMaxPublishFrequency());
                }
            }
        }

        if (m_afCameraMap.size() == 0){
            // No valid cameras defined in the world config file
            // hence create a default camera
            afCameraPtr cameraPtr = new afCamera(this);
            if (cameraPtr->createDefaultCamera()){
                addAFCamera(cameraPtr, "default_camera");
                cameraPtr->afObjectCreate(cameraPtr->m_name,
                                          cameraPtr->getNamespace(),
                                          cameraPtr->getMinPublishFrequency(),
                                          cameraPtr->getMaxPublishFrequency());
            }

        }
    }

    return true;

}

///
/// \brief afWorld::addLight
/// \param a_name
/// \param a_light
/// \return
///
bool afWorld::addAFLight(afLightPtr a_light, std::string a_name){
    m_afLightMap[a_name] = a_light;
    return true;
}

///
/// \brief afWorld::addCamera
/// \param a_name
/// \param a_cam
/// \return
///
bool afWorld::addAFCamera(afCameraPtr a_cam, std::string a_name){
    m_afCameraMap[a_name] = a_cam;
    return true;
}

///
/// \brief afWorld::addRigidBody
/// \param a_name
/// \param a_rb
/// \return
///
bool afWorld::addAFRigidBody(afRigidBodyPtr a_rb, std::string a_name){
    m_afRigidBodyMap[a_name] = a_rb;
    return true;
}

///
/// \brief afWorld::addSoftBody
/// \param a_name
/// \param a_sb
/// \return
///
bool afWorld::addAFSoftBody(afSoftBodyPtr a_sb, std::string a_name){
    m_afSoftBodyMap[a_name] = a_sb;
    return true;
}

///
/// \brief afWorld::addJoint
/// \param a_name
/// \param a_jnt
/// \return
///
bool afWorld::addAFJoint(afJointPtr a_jnt, std::string a_name){
    m_afJointMap[a_name] = a_jnt;
    return true;
}

///
/// \brief afWorld::addSensor
/// \param a_sensor
/// \param a_name
/// \return
///
bool afWorld::addAFSensor(afSensorPtr a_sensor, std::string a_name){
    m_afSensorMap[a_name] = a_sensor;
    return true;
}

///
/// \brief afWorld::addAFMultiBody
/// \param a_multiBody
/// \param a_name
/// \return
///
bool afWorld::addAFMultiBody(afMultiBodyPtr a_multiBody, std::string a_name){
    m_afMultiBodyMap[a_name] = a_multiBody;
    return true;
}

///
/// \brief afWorld::getAFLighs
/// \return
///
afLightVec  afWorld::getAFLighs(){
    afLightVec _lights;
    afLightMap::iterator _lIt;

    for (_lIt = m_afLightMap.begin() ; _lIt != m_afLightMap.end() ; _lIt++){
        _lights.push_back(_lIt->second);
    }

    return _lights;
}

///
/// \brief afWorld::getAFCameras
/// \return
///
afCameraVec afWorld::getAFCameras(){
    afCameraVec _cams;
    afCameraMap::iterator _cIt;

    for (_cIt = m_afCameraMap.begin() ; _cIt != m_afCameraMap.end() ; _cIt++){
        _cams.push_back(_cIt->second);
    }

    return _cams;
}

///
/// \brief afWorld::getAFRigidBodies
/// \return
///
afRigidBodyVec afWorld::getAFRigidBodies(){
    afRigidBodyVec _rbs;
    afRigidBodyMap::iterator _rbIt;

    for (_rbIt = m_afRigidBodyMap.begin() ; _rbIt != m_afRigidBodyMap.end() ; _rbIt++){
        _rbs.push_back(_rbIt->second);
    }

    return _rbs;
}

///
/// \brief afWorld::getAFSoftBodies
/// \return
///
afSoftBodyVec afWorld::getAFSoftBodies(){
    afSoftBodyVec _sbs;
    afSoftBodyMap::iterator _sbIt;

    for (_sbIt = m_afSoftBodyMap.begin() ; _sbIt != m_afSoftBodyMap.end() ; _sbIt++){
        _sbs.push_back(_sbIt->second);
    }

    return _sbs;
}

///
/// \brief afWorld::getJoints
/// \return
///
afJointVec afWorld::getAFJoints(){
    afJointVec _jnts;
    afJointMap::iterator _jIt;

    for (_jIt = m_afJointMap.begin() ; _jIt != m_afJointMap.end() ; _jIt++){
        _jnts.push_back(_jIt->second);
    }

    return _jnts;
}

///
/// \brief afWorld::getSensors
/// \return
///
afSensorVec afWorld::getAFSensors(){
    afSensorVec _sensors;
    afSensorMap::iterator _sIt;

    for (_sIt = m_afSensorMap.begin() ; _sIt != m_afSensorMap.end() ; _sIt++){
        _sensors.push_back(_sIt->second);
    }

    return _sensors;
}


///
/// \brief afWorld::getAFMultiBodies
/// \return
///
afMultiBodyVec afWorld::getAFMultiBodies(){
    afMultiBodyVec _multiBodies;
    afMultiBodyMap::iterator _mIt;

    for (_mIt = m_afMultiBodyMap.begin() ; _mIt != m_afMultiBodyMap.end() ; _mIt++){
        _multiBodies.push_back(_mIt->second);
    }

    return _multiBodies;
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
    btDynamicsWorld* m_dynamicsWorld = s_bulletWorld->m_bulletWorld;
    if (m_dynamicsWorld == 0)
        return false;

    btCollisionWorld::ClosestRayResultCallback rayCallback(toBTvec(rayFromWorld), toBTvec(rayToWorld));

    rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
    m_dynamicsWorld->rayTest(toBTvec(rayFromWorld), toBTvec(rayToWorld), rayCallback);
    if (rayCallback.hasHit())
    {
        cVector3d pickPos = toCvec(rayCallback.m_hitPointWorld);
        m_pickSphere->setLocalPos(pickPos);
        m_pickSphere->setShowEnabled(true);
        const btCollisionObject* colObject = rayCallback.m_collisionObject;
        if (colObject->getInternalType() == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
            btRigidBody* body = (btRigidBody*)btRigidBody::upcast(colObject);
            if (body){
                m_lastPickedBody = getAFRigidBody(body, true);
                if (m_lastPickedBody){
                    std::cerr << "Picked AF Rigid Body: " << m_lastPickedBody->m_name << std::endl;
                }
                //other exclusions?
                if (!(body->isStaticObject() || body->isKinematicObject()))
                {
                    m_pickedBody = body;
                    m_pickedBodyColor = m_lastPickedBody->m_material->m_ambient;
                    m_lastPickedBody->m_material->setGrayLightSlate();
                    m_lastPickedBody->markForUpdate();
                    m_savedState = m_pickedBody->getActivationState();
                    m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
                    //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                    btVector3 localPivot = body->getCenterOfMassTransform().inverse() * toBTvec(pickPos);
                    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                    m_dynamicsWorld->addConstraint(p2p, true);
                    m_pickedConstraint = p2p;
                    btScalar mousePickClamping = 30.f;
                    p2p->m_setting.m_impulseClamp = mousePickClamping;
                    //very weak constraint for picking
                    p2p->m_setting.m_tau = 0.001f;
                }
            }
        }
        else if((colObject->getInternalType() == btCollisionObject::CollisionObjectTypes::CO_SOFT_BODY)){
            btSoftBody* sBody = (btSoftBody*)btSoftBody::upcast(colObject);
            // Now find the closest node in the soft body so we can do
            // something about it.
            btVector3 _hitPoint = rayCallback.m_hitPointWorld;

            // Max distance between the hit point softbody nodes to be considered
            double _maxDistance = 0.1;

            // Index of closest Node. Initialize to -1 so it can be used
            // as boolean as well if a Node was Found
            int _closestNodeIdx = -1;

            for (int nodeIdx = 0 ; nodeIdx < sBody->m_nodes.size() ; nodeIdx++){
                if ( (_hitPoint - sBody->m_nodes[nodeIdx].m_x).length() < _maxDistance ){
                    _maxDistance = (_hitPoint - sBody->m_nodes[nodeIdx].m_x).length();
                    _closestNodeIdx = nodeIdx;
                }
            }

            if(_closestNodeIdx >=0 ){
                m_pickedNode = &sBody->m_nodes[_closestNodeIdx];
                m_pickedNode->m_v.setZero();
                m_pickedSoftBody = sBody;
                m_pickedNodeIdx = _closestNodeIdx;
                m_pickedNodeGoal = toCvec(_hitPoint);
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
/// \brief afMultiBody::movePickedBody
/// \param rayFromWorld
/// \param rayToWorld
/// \return
///
bool afWorld::movePickedBody(const cVector3d &rayFromWorld, const cVector3d &rayToWorld){
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
            pickCon->setPivotB(toBTvec(newPivotB));
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
/// \brief afMultiBody::removePickingConstraint
///
void afWorld::removePickingConstraint(){
    btDynamicsWorld* m_dynamicsWorld = s_bulletWorld->m_bulletWorld;
    if (m_pickedConstraint)
    {
        m_pickSphere->setShowEnabled(false);
        m_pickedBody->forceActivationState(m_savedState);
        m_pickedBody->activate();
        m_dynamicsWorld->removeConstraint(m_pickedConstraint);
        delete m_pickedConstraint;
        m_pickedConstraint = 0;
        m_pickedBody = 0;
        if (m_lastPickedBody){
            m_lastPickedBody->m_material->m_ambient = m_pickedBodyColor;
            m_lastPickedBody->markForUpdate();
        }
    }

    if (m_pickedSoftBody){
        m_pickSphere->setShowEnabled(false);
        m_pickedSoftBody = 0;
        m_pickedNodeIdx = -1;
        m_pickedNodeMass = 0;
    }
}


///
/// \brief afCamera::afCamera
///
afCamera::afCamera(afWorldPtr a_afWorld): afRigidBody(a_afWorld){

    s_monitors = glfwGetMonitors(&s_numMonitors);
    m_afWorld = a_afWorld;

    m_targetVisualMarker = new cMesh();
    cCreateSphere(m_targetVisualMarker, 0.03);
    m_targetVisualMarker->m_material->setBlack();
    m_targetVisualMarker->setShowFrame(false);
    m_targetVisualMarker->setTransparencyLevel(0.7);
    m_targetVisualMarker->setUseDisplayList(true);
    m_targetVisualMarker->markForUpdate(false);
    m_targetVisualMarker->setShowEnabled(false);
    addChild(m_targetVisualMarker);
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
    cTransform _T_wINb = getLocalTransform();
    _T_wINb.invert();
    m_targetPos = _T_wINb * a_localLookAt;
    m_targetVisualMarker->setLocalPos(m_targetPos);

    return true;
}

cVector3d afCamera::getGlobalPos(){
    if (getParent()){
        return getParent()->getLocalTransform() * getLocalPos();
    }
    else{
        return getLocalPos();
    }
}


///
/// \brief afCamera::setTargetPosLocal
/// \param a_pos
///
void afCamera::setTargetPos(cVector3d a_pos){
    if(getParent()){
        cTransform T_inv = getParent()->getLocalTransform();
        T_inv.invert();
        //        a_pos = T_inv * a_pos;
    }
    setView(getLocalPos(), a_pos, m_camera->getUpVector());
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
cVector3d afCamera::getTargetPos(){
    cTransform _T_pInw;
    _T_pInw.identity();
    if (getParent()){
        //        _T_pInw = getParent()->getLocalTransform();
    }
    return _T_pInw * getLocalTransform() * m_targetPos;
}

///
/// \brief afCamera::createDefaultCamera
/// \return
///
bool afCamera::createDefaultCamera(){
    std::cerr << "INFO: USING DEFAULT CAMERA" << std::endl;

    m_camera = new cCamera(m_afWorld->s_bulletWorld);
    addChild(m_camera);

    // Set a default name
    m_name = "default_camera";

    // position and orient the camera
    setView(cVector3d(4.0, 0.0, 2.0),  // camera position (eye)
            cVector3d(0.0, 0.0,-0.5),       // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));      // direction of the "up" vector

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
    const GLFWvidmode* _mode = glfwGetVideoMode(m_monitor);
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

    m_camera->m_frontLayer->addChild(m_graphicsDynamicsFreqLabel);
    m_camera->m_frontLayer->addChild(m_wallSimTimeLabel);
    m_camera->m_frontLayer->addChild(m_devicesModesLabel);
    m_camera->m_frontLayer->addChild(m_deviceButtonLabel);
    m_camera->m_frontLayer->addChild(m_controllingDeviceLabel);

    s_windowIdx++;
    s_cameraIdx++;

    // Assign the Window Camera Handles
    m_afWorld->s_bulletWorld->addChild(this);

    // Make sure to set the mass to 0 as this is a kinematic body
    m_mass = 0.0;
    // Build the model which inturn adds this body to bullet world
    buildDynamicModel();
    return true;
}

///
/// \brief afCamera::loadCamera
/// \param camera_node
/// \param camera_name
/// \return
///
bool afCamera::loadCamera(YAML::Node* a_camera_node, std::string a_camera_name, afWorldPtr a_world){
    YAML::Node cameraNode = *a_camera_node;
    YAML::Node cameraName = cameraNode["name"];
    YAML::Node cameraNamespace = cameraNode["namespace"];
    YAML::Node cameraLocationData = cameraNode["location"];
    YAML::Node cameraLookAtData = cameraNode["look at"];
    YAML::Node cameraUpData = cameraNode["up"];
    YAML::Node cameraClippingPlaneData = cameraNode["clipping plane"];
    YAML::Node cameraFieldViewAngleData = cameraNode["field view angle"];
    YAML::Node cameraOrthoWidthData = cameraNode["orthographic view width"];
    YAML::Node cameraStereo = cameraNode["stereo"];
    YAML::Node cameraControllingDevicesData = cameraNode["controlling devices"];
    YAML::Node cameraParent = cameraNode["parent"];
    YAML::Node cameraMonitor = cameraNode["monitor"];
    YAML::Node cameraPublishImage = cameraNode["publish image"];
    YAML::Node cameraMultiPass = cameraNode["multipass"];

    bool _is_valid = true;
    cVector3d _location, _up, _look_at;
    double _clipping_plane_limits[2], _field_view_angle;
    bool _enable_ortho_view = false;
    double _stereoEyeSeperation, _stereoFocalLength, _orthoViewWidth;
    std::string _stereoModeStr;
    int _monitorToLoad = -1;
    bool _useMultiPassTransparency = false;

    // Set some default values
    m_stereMode = C_STEREO_DISABLED;
    _stereoFocalLength = 2.0;
    _stereoEyeSeperation = 0.02;

    if (cameraName.IsDefined()){
        m_name = cameraName.as<std::string>();
    }
    else{
        m_name = "camera_" + std::to_string(a_world->getAFCameras().size() + 1);
    }

    if (cameraNamespace.IsDefined()){
        m_namespace = cameraNamespace.as<std::string>();
    }
    else{
        m_namespace = a_world->getNamespace();
    }

    if (cameraLocationData.IsDefined()){
        _location = toXYZ<cVector3d>(&cameraLocationData);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOCATION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraLookAtData.IsDefined()){
        _look_at = toXYZ<cVector3d>(&cameraLookAtData);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOOK AT NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraUpData.IsDefined()){
        _up = toXYZ<cVector3d>(&cameraUpData);
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
        _orthoViewWidth = cameraOrthoWidthData.as<double>();
    }
    else{
        _enable_ortho_view = false;
    }
    if (cameraStereo.IsDefined()){
        _stereoModeStr = cameraStereo["mode"].as<std::string>();
        if (_stereoModeStr.compare("PASSIVE") || _stereoModeStr.compare("passive") || _stereoModeStr.compare("Passive")){
            m_stereMode = cStereoMode::C_STEREO_PASSIVE_LEFT_RIGHT;
        }
        _stereoEyeSeperation = cameraStereo["eye separation"].as<double>();
        _stereoFocalLength = cameraStereo["focal length"].as<double>();
    }
    if (cameraMonitor.IsDefined()){
        _monitorToLoad = cameraMonitor.as<int>();
        if (_monitorToLoad < 0 || _monitorToLoad >= s_numMonitors){
            std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" MONITOR NUMBER \"" << _monitorToLoad
                      << "\" IS NOT IN RANGE OF AVAILABLE MONITORS \""<< s_numMonitors <<"\", USING DEFAULT" << std::endl;
            _monitorToLoad = -1;
        }

    }
    if (cameraControllingDevicesData.IsDefined()){
        for(int idx = 0 ; idx < cameraControllingDevicesData.size() ; idx++){
            m_controllingDevNames.push_back( cameraControllingDevicesData[idx].as<std::string>());
        }
    }

    if (cameraPublishImage.IsDefined()){
        m_publishImage = cameraPublishImage.as<bool>();
    }

    if (cameraMultiPass.IsDefined()){
        _useMultiPassTransparency = cameraMultiPass.as<bool>();
    }

    if(_is_valid){
        m_camera = new cCamera(a_world->s_bulletWorld);
        addChild(m_camera);

        bool _overrideParent = false;

        if (cameraParent.IsDefined()){
            _overrideParent = true;
            std::string parent_name = cameraParent.as<std::string>();
            afRigidBodyPtr pBody = a_world->getAFRigidBody(parent_name);
            if (pBody){
                pBody->addChild(this);
            }
            else{
                std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                          << parent_name << "\"" <<std::endl;
            }
        }
        if (! _overrideParent){
            a_world->s_bulletWorld->addChild(this);
        }

        //////////////////////////////////////////////////////////////////////////////////////
        // position and orient the camera
        setView(_location, _look_at, _up);

        // set the near and far clipping planes of the camera
        m_camera->setClippingPlanes(_clipping_plane_limits[0], _clipping_plane_limits[1]);

        // set stereo mode
        m_camera->setStereoMode(m_stereMode);

        // set stereo eye separation and focal length (applies only if stereo is enabled)
        m_camera->setStereoEyeSeparation(_stereoEyeSeperation);
        m_camera->setStereoFocalLength(_stereoFocalLength);

        // set vertical mirrored display mode
        m_camera->setMirrorVertical(false);

        m_camera->setFieldViewAngleRad(_field_view_angle);

        // Check if ortho view is enabled
        if (_enable_ortho_view){
            m_camera->setOrthographicView(_orthoViewWidth);
        }

        m_camera->setUseMultipassTransparency(_useMultiPassTransparency);

        std::string window_name = "AMBF Simulator Window " + std::to_string(s_cameraIdx + 1);
        if (m_controllingDevNames.size() > 0){
            for (int i = 0 ; i < m_controllingDevNames.size() ; i++){
                window_name += (" - " + m_controllingDevNames[i]);
            }

        }

        // create display context
        if (_monitorToLoad == -1){
            if (s_cameraIdx < s_numMonitors){
                _monitorToLoad = s_cameraIdx;
            }
            else{
                _monitorToLoad = 0;
            }
        }
        m_monitor = s_monitors[_monitorToLoad];

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

        m_camera->m_frontLayer->addChild(m_graphicsDynamicsFreqLabel);
        m_camera->m_frontLayer->addChild(m_wallSimTimeLabel);
        m_camera->m_frontLayer->addChild(m_devicesModesLabel);
        m_camera->m_frontLayer->addChild(m_deviceButtonLabel);
        m_camera->m_frontLayer->addChild(m_controllingDeviceLabel);

        s_windowIdx++;
        s_cameraIdx++;

        // Make sure to set the mass to 0 as this is a kinematic body
        m_mass = 0.0;
        // Build the model which inturn adds this body to bullet world
        buildDynamicModel();


#ifdef AF_ENABLE_OPEN_CV_SUPPORT
        if (m_publishImage){
            m_frameBuffer = new cFrameBuffer();
            m_imageFromBuffer = cImage::create();

            if (s_imageTransportInitialized == false){
                s_imageTransportInitialized = true;
                int argc = 0;
                char **argv = 0;
                ros::init(argc, argv, "ambf_image_transport_node");
                s_imageTransportNode = new ros::NodeHandle();
                s_imageTransport = new image_transport::ImageTransport(*s_imageTransportNode);
            }

            m_frameBuffer->setup(m_camera, m_width, m_height, true, true);
            m_imagePublisher = s_imageTransport->advertise("/ambf/image_data/" + m_name, 1);
        }
#endif
    }

    return _is_valid;
}


///
/// \brief afCamera::publishImage
///
void afCamera::publishImage(){
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    if (m_publishImage){
        m_frameBuffer->renderView();
        m_frameBuffer->copyImageBuffer(m_imageFromBuffer);
        m_imageFromBuffer->flipHorizontal();
        m_imageMatrix = cv::Mat(m_imageFromBuffer->getHeight(), m_imageFromBuffer->getWidth(), CV_8UC4, m_imageFromBuffer->getData());
        cv::cvtColor(m_imageMatrix, m_imageMatrix, cv::COLOR_BGRA2RGB);
        sensor_msgs::ImagePtr rosMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_imageMatrix).toImageMsg();
        m_imagePublisher.publish(rosMsg);
    }
#endif
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
/// \brief afCamera::~afCamera
///
afCamera::~afCamera(){
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    if (m_publishImage){
        delete m_frameBuffer;;
        if (s_imageTransportInitialized == true){
            s_imageTransportInitialized = false;
            delete s_imageTransport;
            delete s_imageTransportNode;
        }
    }
#endif
}



///
/// \brief afLight::afLight
///
afLight::afLight(afWorldPtr a_afWorld): afRigidBody(a_afWorld){
    m_afWorld = a_afWorld;
}

///
/// \brief afLight::createDefaultLight
/// \return
///
bool afLight::createDefaultLight(){
    std::cerr << "INFO: NO LIGHT SPECIFIED, USING DEFAULT LIGHTING" << std::endl;
    m_spotLight = new cSpotLight(m_afWorld->s_bulletWorld);
    m_name = "default_light";
    addChild(m_spotLight);
    m_spotLight->setLocalPos(cVector3d(0.0, 0.5, 2.5));
    m_spotLight->setDir(0, 0, -1);
    m_spotLight->setSpotExponent(0.3);
    m_spotLight->setCutOffAngleDeg(60);
    m_spotLight->setShadowMapEnabled(true);
    m_spotLight->m_shadowMap->setQualityVeryHigh();
    m_spotLight->setEnabled(true);
    m_afWorld->s_bulletWorld->addChild(m_spotLight);

    m_mass = 0.0;
    buildDynamicModel();

    return true;
}


///
/// \brief afLight::loadLight
/// \param light_node
/// \return
///
bool afLight::loadLight(YAML::Node* a_light_node, std::string a_light_name, afWorldPtr a_world){
    m_name = a_light_name;
    YAML::Node lightNode = *a_light_node;
    YAML::Node lightName = lightNode["name"];
    YAML::Node lightNamespace = lightNode["namespace"];
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

    if(lightName.IsDefined()){
        m_name = lightName.as<std::string>();
    }
    else{
        m_name = "light_" + std::to_string(a_world->getAFLighs().size() + 1);
    }

    if (lightNamespace.IsDefined()){
        m_namespace = lightNamespace.as<std::string>();
    }
    else{
        m_namespace = a_world->getNamespace();
    }

    if (lightLocationData.IsDefined()){
        _location = toXYZ<cVector3d>(&lightLocationData);
    }
    else{
        std::cerr << "INFO: LIGHT \"" << a_light_name << "\" LIGHT LOCATION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (lightDirectionData.IsDefined()){
        _direction = toXYZ<cVector3d>(&lightDirectionData);
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
        m_spotLight = new cSpotLight(a_world->s_bulletWorld);
        addChild(m_spotLight);

        bool _overrideDefaultParenting = false;
        if (lightParent.IsDefined()){
            _overrideDefaultParenting = true;
            std::string parent_name = lightParent.as<std::string>();
            afRigidBodyPtr pBody = a_world->getAFRigidBody(parent_name);
            if (pBody){
                pBody->addChild(this);
            }
            else{
                std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                          << parent_name << "\"" <<std::endl;
            }
        }
        if (! _overrideDefaultParenting){
            a_world->s_bulletWorld->addChild(this);
        }

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

        m_mass = 0.0;
        buildDynamicModel();
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
    m_afWorld = a_afWorld;

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
    afJointMap _jntMap = *(m_afWorld->getAFJointMap());
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
/// \brief afMultiBody::remapSensorName
/// \param a_sensor_name
/// \return
///
std::string afMultiBody::remapSensorName(std::string a_sensor_name){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    afSensorMap _sensorMap = *(m_afWorld->getAFSensorMap());
    if (_sensorMap.find(a_sensor_name) == _sensorMap.end()){
        return remap_string;
    }

    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(_sensorMap.find(a_sensor_name + remap_string) != _sensorMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief afMultiBody::loadAllMultiBodies
///
void afMultiBody::loadAllMultiBodies(bool enable_comm){
    for (int i = 0 ; i < m_afWorld->getNumMBConfigs(); i++){
        loadMultiBody(i, enable_comm);
    }
}

///
/// \brief afMultiBody::loadMultiBody
/// \param i
/// \return
///
bool afMultiBody::loadMultiBody(int i, bool enable_comm){
    if (i >= m_afWorld->getNumMBConfigs()){
        std::cerr << "ERROR, REQUESTED MULTI-BODY IDX " << i << " HOWEVER, " <<
                     m_afWorld->getNumMBConfigs() - 1 << " INDEXED MULTI-BODIES DEFINED" << std::endl;
        return 0;
    }
    std::string multibody_config = m_afWorld->getMultiBodyConfig(i);
    return loadMultiBody(multibody_config, enable_comm);
}

///
/// \brief afMultiBody::loadMultiBody
/// \param a_multibody_config
/// \return
///
bool afMultiBody::loadMultiBody(std::string a_multibody_config_file, bool enable_comm){
    if (a_multibody_config_file.empty()){
        a_multibody_config_file = m_afWorld->getMultiBodyConfig();
    }
    YAML::Node multiBodyNode;
    try{
        multiBodyNode = YAML::LoadFile(a_multibody_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << a_multibody_config_file << std::endl;
        return 0;
    }

    // Declare all the yaml parameters that we want to look for
    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodySoftBodies = multiBodyNode["soft bodies"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];
    YAML::Node multiBodySensors = multiBodyNode["sensors"];
    YAML::Node multiBodyJointERP = multiBodyNode["joint erp"];
    YAML::Node multiBodyJointCFM = multiBodyNode["joint cfm"];
    YAML::Node multiBodyIgnoreInterCollision = multiBodyNode["ignore inter-collision"];

    boost::filesystem::path mb_cfg_dir = boost::filesystem::path(a_multibody_config_file).parent_path();
    m_multibody_path = mb_cfg_dir.c_str();

    /// Loading Rigid Bodies
    afRigidBodyPtr rBodyPtr;
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
        m_mb_namespace = multiBodyNameSpace.as<std::string>();
    }
    else{
        m_mb_namespace = "/ambf/env/";
    }

    size_t totalRigidBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        rBodyPtr = new afRigidBody(m_afWorld);
        std::string rb_name = multiBodyRidigBodies[i].as<std::string>();
        YAML::Node rb_node = multiBodyNode[rb_name];
        if (rBodyPtr->loadRigidBody(&rb_node, rb_name, this)){
            std::string remap_str = remapBodyName(rBodyPtr->getNamespace() + rb_name, m_afWorld->getAFRigidBodyMap());
            m_afWorld->addAFRigidBody(rBodyPtr, rBodyPtr->getNamespace() + rb_name + remap_str);
            m_afRigidBodyMapLocal[rBodyPtr->getNamespace() + rb_name] = rBodyPtr;
            if (enable_comm){
                std::string af_name = rBodyPtr->m_name;
                if ((strcmp(af_name.c_str(), "world") == 0) ||
                        (strcmp(af_name.c_str(), "World") == 0) ||
                        (strcmp(af_name.c_str(), "WORLD") == 0)){
                    continue;
                }
                else{
                    rBodyPtr->afObjectCreate(rBodyPtr->m_name + remap_str,
                                             rBodyPtr->getNamespace(),
                                             rBodyPtr->getMinPublishFrequency(),
                                             rBodyPtr->getMaxPublishFrequency());
                }
            }
        }
    }

    /// Loading Soft Bodies
    afSoftBodyPtr sBodyPtr;
    size_t totalSoftBodies = multiBodySoftBodies.size();
    for (size_t i = 0; i < totalSoftBodies; ++i) {
        sBodyPtr = new afSoftBody(m_afWorld);
        std::string sb_name = multiBodySoftBodies[i].as<std::string>();
        YAML::Node sb_node = multiBodyNode[sb_name];
        if (sBodyPtr->loadSoftBody(&sb_node, sb_name, this)){
            std::string remap_str = remapBodyName(sb_name, m_afWorld->getAFSoftBodyMap());
            m_afWorld->addAFSoftBody(sBodyPtr, sBodyPtr->getNamespace() + sb_name + remap_str);
            m_afSoftBodyMapLocal[sBodyPtr->getNamespace() + sb_name] = sBodyPtr;
            //            tmpSoftBody->createAFObject(tmpSoftBody->m_name + remap_str);
        }
    }

    /// Loading Sensors
    afSensorPtr sensorPtr = 0;
    size_t totalSensors = multiBodySensors.size();
    for (size_t i = 0; i < totalSensors; ++i) {
        std::string sensor_name = multiBodySensors[i].as<std::string>();
        std::string remap_str = remapSensorName(m_mb_namespace + sensor_name);
        YAML::Node sensor_node = multiBodyNode[sensor_name];
        // Check which type of sensor is this so we can cast appropriately beforehand
        if (sensor_node["type"].IsDefined()){
            std::string _sensor_type = sensor_node["type"].as<std::string>();
            // Check if this is a proximity sensor
            // More sensors to follow
            if (_sensor_type.compare("Proximity") == 0 || _sensor_type.compare("proximity") == 0 || _sensor_type.compare("PROXIMITY") == 0){
                sensorPtr = new afProximitySensor(m_afWorld);
            }
            if (_sensor_type.compare("Resistance") == 0 || _sensor_type.compare("resistance") == 0 || _sensor_type.compare("RESISTANCE") == 0){
                sensorPtr = new afResistanceSensor(m_afWorld);
            }

            // Finally load the sensor from afmb config data
            if (sensorPtr){
                if (sensorPtr->loadSensor(&sensor_node, sensor_name, this, remap_str)){
                    m_afWorld->addAFSensor(sensorPtr, m_mb_namespace + sensor_name + remap_str);
                }
            }
        }
        else{
            continue;
        }
    }

    if (multiBodyJointERP.IsDefined()){
        m_jointERP = multiBodyJointERP.as<double>();
    }
    if (multiBodyJointCFM.IsDefined()){
        m_jointCFM = multiBodyJointCFM.as<double>();
    }

    /// Loading Joints
    afJointPtr jntPtr;
    size_t totalJoints = multiBodyJoints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        jntPtr = new afJoint(m_afWorld);
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
        YAML::Node jnt_node = multiBodyNode[jnt_name];
        std::string remap_str = remapJointName(m_mb_namespace + jnt_name);
        if (jntPtr->loadJoint(&jnt_node, jnt_name, this, remap_str)){
            m_afWorld->addAFJoint(jntPtr, m_mb_namespace + jnt_name + remap_str);
            m_afJointMapLocal[m_mb_namespace + jnt_name] = jntPtr;
        }
    }

    // This flag would ignore collision for all the multibodies in the scene
    bool _ignoreInterCollision = false;
    if (multiBodyIgnoreInterCollision.IsDefined()){
        _ignoreInterCollision = multiBodyIgnoreInterCollision.as<bool>();
        if (_ignoreInterCollision){
            ignoreCollisionChecking();
        }
    }
    // If the ignore_inter_collision flag is false, then ignore collision based on collision
    // groups
    if (! _ignoreInterCollision){
        buildCollisionGroups();
    }

    //    removeOverlappingCollisionChecking();

    return true;
}


///
/// \brief afMultiBody::getRigidBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afRigidBodyPtr afMultiBody::getAFRigidBodyLocal(std::string a_name, bool suppress_warning){
    if (m_afRigidBodyMapLocal.find(a_name) != m_afRigidBodyMapLocal.end()){
        return m_afRigidBodyMapLocal[a_name];
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY BODY NAMED: " << a_name << std::endl;

            std::cerr <<"Existing Bodies in Map: " << m_afRigidBodyMapLocal.size() << std::endl;
            afRigidBodyMap::iterator rbIt = m_afRigidBodyMapLocal.begin();
            for (; rbIt != m_afRigidBodyMapLocal.end() ; ++rbIt){
                std::cerr << rbIt->first << std::endl;
            }
        }
        return NULL;
    }
}

///
/// \brief afMultiBody::removeCollisionChecking
///
void afMultiBody::ignoreCollisionChecking(){

    /// Only ignore collision checking between the bodies
    /// defined in the specific multibody config file
    /// and not all the bodies in the world
    afRigidBodyMap::iterator rBodyItA = m_afRigidBodyMapLocal.begin();
    std::vector<btRigidBody*> rBodiesVec;
    rBodiesVec.resize(m_afRigidBodyMapLocal.size());
    int i=0;
    for ( ; rBodyItA != m_afRigidBodyMapLocal.end() ; ++rBodyItA){
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
    afRigidBodyMap* _rbMap = m_afWorld->getAFRigidBodyMap();
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
afRigidBodyPtr afWorld::getAFRigidBody(std::string a_name, bool suppress_warning){
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
        std::cerr << "WARNING: CAN'T FIND ANY BODY BOUND TO BULLET RIGID BODY: " << a_body << std::endl;

        std::cerr <<"Existing Bodies in Map: " << m_afRigidBodyMap.size() << std::endl;
        afRigidBodyMap::iterator rbIt = m_afRigidBodyMap.begin();
        for (; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
            std::cerr << rbIt->first << std::endl;
        }
    }
    return NULL;
}



///
/// \brief afWorld::getAFMultiBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afMultiBodyPtr afWorld::getAFMultiBody(std::string a_name, bool suppress_warning){
    if (m_afMultiBodyMap.find(a_name) != m_afMultiBodyMap.end()){
        return m_afMultiBodyMap[a_name];
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY MULTI-BODY NAMED: " << a_name << std::endl;

            std::cerr <<"Existing Multi Bodies in Map: " << m_afMultiBodyMap.size() << std::endl;
            afMultiBodyMap::iterator mbIt = m_afMultiBodyMap.begin();
            for (; mbIt != m_afMultiBodyMap.end() ; ++mbIt){
                std::cerr << mbIt->first << std::endl;
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
afRigidBodyPtr afWorld::getRootAFRigidBody(afRigidBodyPtr a_bodyPtr){
    if (!a_bodyPtr){
        std::cerr << "ERROR, BODY PTR IS NULL, CAN\'T LOOK UP ROOT BODIES" << std::endl;
        return 0;
    }

    /// Find Root Body
    afRigidBodyPtr rootParentBody;
    std::vector<int> bodyParentsCount;
    size_t rootParents = 0;
    if (a_bodyPtr->m_parentBodies.size() == 0){
        rootParentBody = a_bodyPtr;
        rootParents++;
    }
    else{
        bodyParentsCount.resize(a_bodyPtr->m_parentBodies.size());
        std::vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
        for (int parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
            if ((*rIt)->m_parentBodies.size() == 0){
                rootParentBody = (*rIt);
                rootParents++;
            }
            bodyParentsCount[parentNum] = (*rIt)->m_parentBodies.size();
        }
    }

    // In case no root parent is found, it is understood that
    // the multibody chain is cyclical, perhaps return
    // the body with least number of parents
    if (rootParents == 0){
        auto minLineage = std::min_element(bodyParentsCount.begin(), bodyParentsCount.end());
        int idx = std::distance(bodyParentsCount.begin(), minLineage);
        rootParentBody = a_bodyPtr->m_parentBodies[idx];
        rootParents++;
        std::cerr << "WARNING! CYCLICAL CHAIN OF BODIES FOUND WITH NO UNIQUE PARENT, RETURING THE BODY WITH LEAST PARENTS";
    }

    if (rootParents > 1)
        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

    return rootParentBody;
}


///
/// \brief afMultiBody::getRootAFRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afMultiBody::getRootAFRigidBodyLocal(afRigidBodyPtr a_bodyPtr){
    /// Find Root Body
    afRigidBodyPtr rootParentBody;
    std::vector<int> bodyParentsCount;
    size_t rootParents = 0;
    if (a_bodyPtr){
        if (a_bodyPtr->m_parentBodies.size() == 0){
            rootParentBody = a_bodyPtr;
            rootParents++;
        }
        else{
            bodyParentsCount.resize(a_bodyPtr->m_parentBodies.size());
            std::vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
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
