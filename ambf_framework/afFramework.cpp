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

double afWorld::m_encl_length;
double afWorld::m_encl_width;
double afWorld::m_encl_height;
int afWorld::m_maxIterations;

GLFWwindow* afCamera::s_mainWindow = NULL;
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


///
/// \brief toBTtransfrom
/// \param cTrans
/// \return
///
btTransform toBTtransfrom(const cTransform &cTrans){
    btTransform btTrans;
    btVector3 btVec(cTrans.getLocalPos().x(), cTrans.getLocalPos().y(), cTrans.getLocalPos().z());
    btTrans.setOrigin(btVec);
    cQuaternion cQuat;
    cQuat.fromRotMat(cTrans.getLocalRot());

    btQuaternion btQuat(cQuat.x, cQuat.y, cQuat.z, cQuat.w);
    btTrans.setRotation(btQuat);

    return btTrans;
}


///
/// \brief toCtransform
/// \param btTrans
/// \return
///
cTransform toCtransform(const btTransform &btTrans){
    cTransform cTrans;
    cVector3d cVec(btTrans.getOrigin().x(), btTrans.getOrigin().y(), btTrans.getOrigin().z());
    cTrans.setLocalPos(cVec);

    cQuaternion cQuat(btTrans.getRotation().w(), btTrans.getRotation().x(), btTrans.getRotation().y(), btTrans.getRotation().z());
    cMatrix3d cRot;
    cQuat.toRotMat(cRot);
    cTrans.setLocalRot(cRot);
    return cTrans;
}


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
/// \brief afComm::afCreateCommInstance
/// \param type
/// \param a_name
/// \param a_namespace
/// \param a_min_freq
/// \param a_max_freq
/// \param time_out
///
void afComm::afCreateCommInstance(afCommType type, std::string a_name, std::string a_namespace, int a_min_freq, int a_max_freq, double time_out){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    switch (type) {
    case afCommType::ACTUATOR:
        m_afActuatorCommPtr.reset(new ambf_comm::Actuator(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::CAMERA:
        m_afCameraCommPtr.reset(new ambf_comm::Camera(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::LIGHT:
        m_afLightCommPtr.reset(new ambf_comm::Light(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::OBJECT:
        m_afObjectCommPtr.reset(new ambf_comm::Object(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::RIGID_BODY:
        m_afRigidBodyCommPtr.reset(new ambf_comm::RigidBody(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::SENSOR:
        m_afSensorCommPtr.reset(new ambf_comm::Sensor(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::VEHICLE:
        m_afVehicleCommPtr.reset(new ambf_comm::Vehicle(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    case afCommType::WORLD:
        m_afWorldCommPtr.reset(new ambf_comm::World(a_name, a_namespace, a_min_freq, a_max_freq, time_out));
        break;
    default:
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
void afComm::afExecuteCommand(double dt){
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
    enable(true);
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

    enable(true);
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
    btVector3 _output(0, 0, 0); // Initialize the output to zero
    if (isEnabled()){
        btVector3 _dPos_prev, _ddPos;
        _dPos_prev = m_dPos;
        m_dPos = set_point - process_val;
        _ddPos = (m_dPos - _dPos_prev) / dt;

        _output = P_lin * (m_dPos) * ts + D_lin * (_ddPos);
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
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
    btVector3 _output(0, 0, 0);

    if (isEnabled()){
    btVector3 _error_cur, _error_prev;
    btMatrix3x3 _dRot_prev;
    btQuaternion _dRotQuat, _dRotQuat_prev;
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
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }

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
    cVector3d _output(0, 0, 0);
    if (isEnabled()){
        cVector3d _dPos_prev, _ddPos;
        _dPos_prev = m_dPos_cvec;
        m_dPos_cvec = set_point - process_val;
        _ddPos = (m_dPos_cvec - _dPos_prev) / dt;

        _output = P_lin * (m_dPos_cvec) * ts + D_lin * (_ddPos);
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
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
    cVector3d _output(0, 0, 0);
    if (isEnabled()){
    cVector3d _error_cur, _error_prev;
    cMatrix3d _dRot_prev;
    cVector3d _e_axis, _e_axis_prev;
    double _e_angle, _e_angle_prev;
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
    }
    else{
        // Maybe throw a console warning to notify the user that this controller is disabled
    }
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
    // Not implemented yet
}


///
/// \brief afObject::afObject
/// \param a_afWorld
///
afBaseObject::afBaseObject(afWorldPtr a_afWorld): cBulletMultiMesh(a_afWorld){
    m_afWorld = a_afWorld;
}



///
/// \brief afObject::~afObject
///
afBaseObject::~afBaseObject(){

}


///
/// \brief afActuator::afActuator
/// \param a_afWorld
///
afActuator::afActuator(afWorldPtr a_afWorld): afBaseObject(a_afWorld){

}


///
/// \brief afConstraintActuator::afConstraintActuator
/// \param a_afWorld
///
afConstraintActuator::afConstraintActuator(afWorldPtr a_afWorld): afActuator(a_afWorld){

}


bool afConstraintActuator::loadActuator(std::string actuator_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(actuator_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO ACTUATOR CONFIG: " << actuator_config_file << std::endl;
        return 0;
    }
    if (baseNode.IsNull()) return false;

    YAML::Node baseActuatorrNode = baseNode[node_name];
    return loadActuator(&baseActuatorrNode, node_name, mB, name_remapping);
}


bool afConstraintActuator::loadActuator(YAML::Node *actuator_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node actuatorNode = *actuator_node;
    if (actuatorNode.IsNull()){
        std::cerr << "ERROR: ACTUATOR'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node actuatorParentName = actuatorNode["parent"];
    YAML::Node actuatorName = actuatorNode["name"];
    YAML::Node actuatorNamespace = actuatorNode["namespace"];
    YAML::Node actuatorPos = actuatorNode["location"]["position"];
    YAML::Node actuatorRot = actuatorNode["location"]["orientation"];
    YAML::Node actuatorPublishFrequency = actuatorNode["publish frequency"];
    YAML::Node actuatorVisible = actuatorNode["visible"];
    YAML::Node actuatorVisibleSize = actuatorNode["visible size"];
    YAML::Node actuatorMaxImpulse = actuatorNode["max impulse"];
    YAML::Node actuatorTau = actuatorNode["tau"];


    if (actuatorParentName.IsDefined()){
        m_parentName = actuatorParentName.as<std::string>();
    }
    else{
        result = false;
    }

    m_name = actuatorName.as<std::string>();

    if(actuatorPos.IsDefined()){
        m_initialPos = toXYZ<cVector3d>(&actuatorPos);
        setLocalPos(m_initialPos);
    }

    if(actuatorRot.IsDefined()){
        double r = actuatorRot["r"].as<double>();
        double p = actuatorRot["p"].as<double>();
        double y = actuatorRot["y"].as<double>();
        m_initialRot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(m_initialRot);
    }

    if(actuatorNamespace.IsDefined()){
        m_namespace = actuatorNamespace.as<std::string>();
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

    if (actuatorPublishFrequency.IsDefined()){
        m_min_publish_frequency = actuatorPublishFrequency["low"].as<int>();
        m_max_publish_frequency = actuatorPublishFrequency["high"].as<int>();
    }

    if (actuatorVisible.IsDefined()){
        m_showActuator = actuatorVisible.as<bool>();
    }
    else{
        m_showActuator = false;
    }

    // First search in the local space.
    m_parentBody = mB->getAFRigidBodyLocal(m_parentName);

    if(!m_parentBody){
        m_parentBody = m_afWorld->getAFRigidBody(m_parentName + name_remapping);
    }

    if (m_parentBody == NULL){
        std::cerr << "ERROR: ACTUATOR'S "<< m_parentName + name_remapping << " NOT FOUND, IGNORING ACTUATOR\n";
        return 0;
    }
    else{
        m_parentBody->addAFActuator(this);
    }

    if (actuatorMaxImpulse.IsDefined()){
        m_maxImpulse = actuatorMaxImpulse.as<double>();
    }

    if (actuatorTau.IsDefined()){
        m_tau = actuatorTau.as<double>();
    }

    return result;

}


void afConstraintActuator::actuate(std::string a_rigid_body_name){

    afRigidBodyPtr body = m_afWorld->getAFRigidBody(a_rigid_body_name);
    actuate(body);

}

void afConstraintActuator::actuate(afRigidBodyPtr a_rigidBody){
    if (a_rigidBody){
        // Since this method does not require an explicit offset, find the relative location
        // of the body and then use it as its offset for the constraint
        btTransform T_aINp = toBTtransfrom(getLocalTransform());
        btTransform T_pINw = m_parentBody->m_bulletRigidBody->getCenterOfMassTransform();
        btTransform T_wINc = a_rigidBody->m_bulletRigidBody->getCenterOfMassTransform().inverse();
        btTransform T_aINw = T_pINw * T_aINp;

        btTransform T_aINc = T_wINc * T_aINw;

        cVector3d P_aINc = toCvec(T_aINc.getOrigin());

        actuate(a_rigidBody, P_aINc);
    }

}

void afConstraintActuator::actuate(std::string a_rigid_body_name, cVector3d a_bodyOffset){
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

            std::cerr << "INFO! ACTUATOR \"" << m_name << "\" IS ACTIVATED WITH THE SAME BODY AND OFFSET. THEREBY "
                                                          "IGNORING REQUEST \n";
            return;
        }
        else{
            // Deactuate the constraint first
            deactuate();
        }
    }

    if (a_rigidBody){
        btVector3 pvtA = toBTvec(getLocalPos());
        btVector3 pvtB = toBTvec(a_bodyOffset);
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

void afConstraintActuator::actuate(std::string a_softbody_name, int a_face_index){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, int a_face_index){

}

void afConstraintActuator::actuate(std::string a_softbody_name, int a_face_index, cVector3d a_bodyOffset){

}

void afConstraintActuator::actuate(afSoftBodyPtr a_softBody, int a_face_index, cVector3d a_bodyOffset){

}

void afConstraintActuator::deactuate(){
    if (m_constraint){
        m_afWorld->m_bulletWorld->removeConstraint(m_constraint);
        delete m_constraint;

        m_constraint = 0;
        m_childBody = 0;
        m_childSotBody = 0;
        m_softBodyFaceIdx = -1;
    }
    m_active = false;
}


///
/// \brief afCartesianController::afExecuteCommand
/// \param dt
///
void afConstraintActuator::afExecuteCommand(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afActuatorCommPtr.get() != nullptr){
        ambf_msgs::ActuatorCmd cmd = m_afActuatorCommPtr->get_command();

        if (cmd.actuate){
            if (m_active){
                // Constraint is active. Ignore request
                return;
            }
             std::string body_name = cmd.body_name.data;
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
void afConstraintActuator::updatePositionFromDynamics(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afActuatorCommPtr.get() != nullptr){
        m_afActuatorCommPtr->set_name(m_name);
        m_afActuatorCommPtr->set_parent_name(m_parentName);
    }
#endif

}



///
/// \brief afBody::afBody
/// \param a_world
///
afRigidBody::afRigidBody(afWorldPtr a_afWorld): afBaseObject(a_afWorld){
    m_afWorld = a_afWorld;
    setFrameSize(0.5);
    m_mesh_name.clear();
    m_collision_mesh_name.clear();
    m_scale = 1.0;

    m_dpos.setValue(0, 0, 0);
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
    cjPairs = a_childBody->m_CJ_PairsAll;
    for (int i = 0 ; i < cjPairs.size() ; i++){
        cjPairs[i].m_directConnection = false; // Make sure to mark that these are not directly connected to the body
    }
    cjPairs.push_back(afChildJointPair(a_childBody, a_jnt, true));

    std::vector<afChildJointPair>::iterator cjIt;
    for (cjIt = cjPairs.begin() ; cjIt != cjPairs.end(); ++cjIt){
        bool _cExists = false;
        for (size_t cjIdx = 0; cjIdx < m_CJ_PairsAll.size() ; cjIdx++){
            if (cjIt->m_childBody == m_CJ_PairsAll[cjIdx].m_childBody){
                _cExists = true;
                break;
            }
        }

        if (!_cExists){
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
    m_meshes->clear();
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

        for (cjIt = m_CJ_PairsAll.begin() ; cjIt != m_CJ_PairsAll.end(); ++cjIt){
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

    // Also update the tree for active CJ pairs
    for (cpIt = childrensParents.begin() ; cpIt != childrensParents.end() ; ++cpIt){

        for (cjIt = m_CJ_PairsActive.begin() ; cjIt != m_CJ_PairsActive.end(); ++cjIt){
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
    std::vector<afChildJointPair> parentsChildrenJointPairs = m_CJ_PairsAll;
    parentsChildrenJointPairs.push_back( afChildJointPair(this, NULL));
    std::vector<afChildJointPair>::iterator pCJIt;
    std::vector<afRigidBodyPtr>::iterator pIt;

    for (pCJIt = parentsChildrenJointPairs.begin() ; pCJIt != parentsChildrenJointPairs.end() ; ++pCJIt){
        for (pIt = m_parentBodies.begin() ; pIt != m_parentBodies.end(); ++pIt){
            afRigidBodyPtr parentBody = *pIt;
            std::vector<afChildJointPair>::iterator cjIt;
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
    std::vector<afChildJointPair>::iterator cjIt;
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
    parentsChildrenJointPairs.push_back( afChildJointPair(this, NULL));

    for (pCJIt = parentsChildrenJointPairs.begin() ; pCJIt != parentsChildrenJointPairs.end() ; ++pCJIt){
        for (pIt = m_parentBodies.begin() ; pIt != m_parentBodies.end(); ++pIt){
            afRigidBodyPtr parentBody = *pIt;
            std::vector<afChildJointPair>::iterator cjIt;
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
        cjPairs = a_childBody->m_CJ_PairsAll;
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
    YAML::Node bodyCollisionMeshType = bodyNode["collision mesh type"];
    YAML::Node bodyCollisionShape = bodyNode["collision shape"];
    YAML::Node bodyCompoundCollisionShape = bodyNode["compound collision shape"];
    YAML::Node bodyCollisionOffset = bodyNode["collision offset"];
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
    YAML::Node bodyControllerOutputType = bodyNode["controller output type"];
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
    YAML::Node bodyPassive = bodyNode["passive"];
    YAML::Node bodyShaders = bodyNode["shaders"];


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
        else if (_visual_shape_str.compare("Plane") == 0 || _visual_shape_str.compare("plane") == 0 || _visual_shape_str.compare("PLANE") == 0){
            double offset = bodyGeometry["offset"].as<double>();

            double nx = bodyGeometry["normal"]["x"].as<double>();
            double ny = bodyGeometry["normal"]["y"].as<double>();
            double nz = bodyGeometry["normal"]["z"].as<double>();

            offset *= m_scale;
            cVector3d pos;
            cVector3d normal(nx, ny, nz);
            normal.normalize();
            pos = normal * offset;
            cQuaternion rot_quat = afUtils::getRotBetweenVectors<cQuaternion, cVector3d>(cVector3d(0, 0, 1), normal);
            cMatrix3d rot_mat;
            rot_quat.toRotMat(rot_mat);
            cCreatePlane(tempMesh, 100, 100, pos, rot_mat);
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

    cMaterial mat;
    mat.setShininess(64);
    double r, g, b, a;
    if(bodyColorRGBA.IsDefined()){
        r = bodyColorRGBA["r"].as<float>();
        g = bodyColorRGBA["g"].as<float>();
        b = bodyColorRGBA["b"].as<float>();
        a = bodyColorRGBA["a"].as<float>();
        mat.setColorf(r, g, b, a);
        setMaterial(mat);
        setTransparencyLevel(a);
    }
    else if(bodyColorComponents.IsDefined()){

        if (bodyColorComponents["diffuse"].IsDefined()){
            r = bodyColorComponents["diffuse"]["r"].as<float>();
            g = bodyColorComponents["diffuse"]["g"].as<float>();
            b = bodyColorComponents["diffuse"]["b"].as<float>();
            mat.m_diffuse.set(r, g, b);
        }
        if (bodyColorComponents["ambient"].IsDefined()){
            double _level = bodyColorComponents["ambient"]["level"].as<float>();
            r *= _level;
            g *= _level;
            b *= _level;
            mat.m_ambient.set(r, g, b);
        }
        if (bodyColorComponents["specular"].IsDefined()){
            r = bodyColorComponents["specular"]["r"].as<float>();
            g = bodyColorComponents["specular"]["g"].as<float>();
            b = bodyColorComponents["specular"]["b"].as<float>();
            mat.m_specular.set(r, g, b);
        }
        if (bodyColorComponents["emission"].IsDefined()){
            r = bodyColorComponents["emission"]["r"].as<float>();
            g = bodyColorComponents["emission"]["g"].as<float>();
            b = bodyColorComponents["emission"]["b"].as<float>();
            mat.m_emission.set(r, g, b);
        }
        if (bodyColorComponents["shininess"].IsDefined()){
            double shininess;
            shininess = bodyColorComponents["shininess"].as<int>();
            mat.setShininess(shininess);
        }
        a = bodyColorComponents["transparency"].as<float>();
        setMaterial(mat);
        setTransparencyLevel(a);
    }
    else if(bodyColor.IsDefined()){
        std::vector<double> rgba = m_afWorld->getColorRGBA(bodyColor.as<std::string>());
        mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
        setMaterial(mat);
        setTransparencyLevel(rgba[3]);
    }

    // Load any shader that have been defined
    if (bodyShaders.IsDefined()){
        boost::filesystem::path shader_path = bodyShaders["path"].as<std::string>();

        if (shader_path.is_relative()){
            shader_path = mB->getMultiBodyPath() / shader_path;
        }

        m_vsFilePath = shader_path / bodyShaders["vertex"].as<std::string>();
        m_fsFilePath = shader_path / bodyShaders["fragment"].as<std::string>();

        m_shaderProgramDefined = true;
    }

    // Load the inertial offset. If the body is a componnd shape, the "inertial offset"
    // will be ignored, as per collision shape "offset" will be used.

    btTransform inertial_offset_trans;
    btVector3 inertial_offset_pos;
    btQuaternion inertial_offset_rot;

    inertial_offset_pos.setValue(0,0,0);
    inertial_offset_rot.setEuler(0,0,0);

    if(bodyInertialOffsetPos.IsDefined()){
        inertial_offset_pos = toXYZ<btVector3>(&bodyInertialOffsetPos);
        inertial_offset_pos = m_scale * inertial_offset_pos;
        if(bodyInertialOffsetRot.IsDefined()){
            double r = bodyInertialOffsetRot["r"].as<double>();
            double p = bodyInertialOffsetRot["p"].as<double>();
            double y = bodyInertialOffsetRot["y"].as<double>();
            inertial_offset_rot.setEulerZYX(y, p, r);
        }
    }

    inertial_offset_trans.setOrigin(inertial_offset_pos);
    inertial_offset_trans.setRotation(inertial_offset_rot);
    setInertialOffsetTransform(inertial_offset_trans);

    // Load Collision Margins

    double collision_margin = 0.001;

    if (bodyCollisionMargin.IsDefined()){
        collision_margin = bodyCollisionMargin.as<double>();
    }

    // Begin loading the collision geometry
    if(m_collisionGeometryType == GeometryType::mesh){
        m_lowResMesh.removeAllMesh();
        if( m_lowResMesh.loadFromFile(low_res_filepath.c_str()) ){
            if(m_scale != 1.0){
                m_lowResMesh.scale(m_scale);
            }

            if (bodyInertialOffsetPos.IsDefined() == false){
                // Call the compute inertial offset before the build contact triangle method
                // Sanity check, see if a mesh is defined or not
                if (m_lowResMesh.m_meshes->size() > 0){
                    inertial_offset_pos = computeInertialOffset(m_lowResMesh.m_meshes[0][0]);
                    setInertialOffsetTransform(inertial_offset_trans);
                }
            }

            if (bodyCollisionMeshType.IsDefined()){
                std::cerr << "INFO! LOADING LOW RES MESH NAMES " << low_res_filepath.c_str() << std::endl;
                std::string mesh_type = bodyCollisionMeshType.as<std::string>();
                if (mesh_type.compare("CONVEX_HULL") == 0){
                    buildContactHull(collision_margin, &m_lowResMesh);
                    std::cerr << "INFO! FOR " << m_name << " CREATING CONVEX HULL FROM COLLISION MESH (CASE 1)" << std::endl;
                }
                else if( (mesh_type.compare("CONVEX_TRIANGLES") == 0) || (mesh_type.compare("CONVEX_MESH") == 0) ){
                    buildContactConvexTriangles(collision_margin, &m_lowResMesh);
                    std::cerr << "INFO! FOR " << m_name << " CREATING CONVEX TRIANGLES FROM COLLISION MESH (CASE 2)" << std::endl;
                }
                else if( (mesh_type.compare("TRIMESH") == 0) || (mesh_type.compare("CONCAVE_MESH") == 0) ){
                    buildContactTriangles(collision_margin, &m_lowResMesh);
                    std::cerr << "INFO! FOR " << m_name << " CREATING TRIANGLE MESH FROM COLLISION MESH (CASE 3)" << std::endl;
                }
                else{
                     std::cerr << "ERROR! FOR " << m_name << " COLLISION MESH TYPE NOT UNDERSTOOD" << std::endl;
                     // Use the mesh data to build the collision shape
                     buildContactTriangles(collision_margin, &m_lowResMesh);
                }
            }
            else{
                // Use the mesh data to build the collision shape
                buildContactTriangles(collision_margin, &m_lowResMesh);
            }
        }
        else{
            std::cerr << "WARNING: Body "
                      << m_name
                      << "'s mesh \"" << low_res_filepath << "\" not found\n";
        }

    }
    else if (m_collisionGeometryType == GeometryType::shape){
        btCompoundShape* compoundCollisionShape = new btCompoundShape();
        btCollisionShape* singleCollisionShape;
        std::string _shape_str = bodyCollisionShape.as<std::string>();
        btTransform shapeOffsetTrans;
        if (bodyCollisionOffset.IsDefined()){
            double px = bodyCollisionOffset["position"]["x"].as<double>();
            double py = bodyCollisionOffset["position"]["y"].as<double>();
            double pz = bodyCollisionOffset["position"]["z"].as<double>();
            double roll =  bodyCollisionOffset["orientation"]["r"].as<double>();
            double pitch = bodyCollisionOffset["orientation"]["p"].as<double>();
            double yaw =   bodyCollisionOffset["orientation"]["y"].as<double>();
            btVector3 shapePos(px, py, pz);
            btMatrix3x3 shapeRot;
            shapeRot.setEulerZYX(roll, pitch, yaw);
            shapeOffsetTrans.setBasis(shapeRot);
            shapeOffsetTrans.setOrigin(shapePos);
        }
        else{
            // If a shape offset is not defined, set the shape offset equal to the intertial offset transform
            // This is to take care of legacy ADF where a shape offset is not set.
            shapeOffsetTrans = getInertialOffsetTransform();
        }

        // A bug in Bullet where a compound plane shape doesn't collide with soft bodies.
        // Thus for a plane, instead of using a compound, use the single collision shape.
        bool is_plane = false;

        if (_shape_str.compare("Box") == 0 || _shape_str.compare("box") == 0 ||_shape_str.compare("BOX") == 0){
            double x = bodyCollisionGeometry["x"].as<double>();
            double y = bodyCollisionGeometry["y"].as<double>();
            double z = bodyCollisionGeometry["z"].as<double>();
            x *= m_scale;
            y *= m_scale;
            z *= m_scale;
            btVector3 halfExtents(x/2, y/2, z/2);
            singleCollisionShape = new btBoxShape(halfExtents);
        }
        else if (_shape_str.compare("Plane") == 0 || _shape_str.compare("plane") == 0 ||_shape_str.compare("PLANE") == 0){
            double offset = bodyCollisionGeometry["offset"].as<double>();

            double nx = bodyCollisionGeometry["normal"]["x"].as<double>();
            double ny = bodyCollisionGeometry["normal"]["y"].as<double>();
            double nz = bodyCollisionGeometry["normal"]["z"].as<double>();
            offset *= m_scale;
            // A bug in Bullet where a compound plane shape doesn't collide with soft bodies.
            // Thus for a plane, instead of using a compound, use the single collision shape.
            is_plane = true;
            m_bulletCollisionShape = new btStaticPlaneShape(btVector3(nx, ny, nz), offset);
        }
        else if (_shape_str.compare("Sphere") == 0 || _shape_str.compare("sphere") == 0 ||_shape_str.compare("SPHERE") == 0){
            double radius = bodyCollisionGeometry["radius"].as<double>();
            radius *= m_scale;
            singleCollisionShape = new btSphereShape(radius);
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
                singleCollisionShape = new btCylinderShapeX(halfExtents);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                btVector3 halfExtents(radius, height/2, radius);
                singleCollisionShape = new btCylinderShape(halfExtents);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                btVector3 halfExtents(radius, radius, height/2);
                singleCollisionShape = new btCylinderShapeZ(halfExtents);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                btVector3 halfExtents(radius, radius, height/2);
                singleCollisionShape = new btCylinderShapeZ(halfExtents);
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
                singleCollisionShape = new btCapsuleShapeX(radius, height);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                singleCollisionShape = new btCapsuleShape(radius, height);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                singleCollisionShape = new btCapsuleShapeZ(radius, height);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                singleCollisionShape = new btCapsuleShapeZ(radius, height);
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
                singleCollisionShape = new btConeShapeX(radius, height);
            }
            else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                singleCollisionShape = new btConeShape(radius, height);
            }
            else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                singleCollisionShape = new btConeShapeZ(radius, height);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s axis \"" << axis << "\" not understood?\n";
                singleCollisionShape = new btConeShapeZ(radius, height);
            }
        }

        // Now, a collision shape has to address both an inertial offset transform as well as
        // a shape offset.

        if (is_plane == false){
            compoundCollisionShape->addChildShape(getInverseInertialOffsetTransform() * shapeOffsetTrans, singleCollisionShape);
            m_bulletCollisionShape = compoundCollisionShape;
        }

    }
    else if (m_collisionGeometryType == GeometryType::compound_shape){
        btCollisionShape* singleCollisionShape;
        btCompoundShape* compoundCollisionShape = new btCompoundShape();
        for (int shapeIdx = 0 ; shapeIdx < bodyCompoundCollisionShape.size() ; shapeIdx++){
            std::string shape_str = bodyCompoundCollisionShape[shapeIdx]["shape"].as<std::string>();
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
            btTransform shapeOffsetTrans(shapeRot, shapePos);
            if (shape_str.compare("Box") == 0 || shape_str.compare("box") == 0 ||shape_str.compare("BOX") == 0){
                double x = bodyCollisionGeometry["x"].as<double>();
                double y = bodyCollisionGeometry["y"].as<double>();
                double z = bodyCollisionGeometry["z"].as<double>();
                x *= m_scale;
                y *= m_scale;
                z *= m_scale;
                btVector3 halfExtents(x/2, y/2, z/2);
               singleCollisionShape = new btBoxShape(halfExtents);
            }
            else if (shape_str.compare("Sphere") == 0 || shape_str.compare("sphere") == 0 ||shape_str.compare("SPHERE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                radius *= m_scale;
                singleCollisionShape = new btSphereShape(radius);
            }
            else if (shape_str.compare("Cylinder") == 0 || shape_str.compare("cylinder") == 0 ||shape_str.compare("CYLINDER") == 0){
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
                    singleCollisionShape = new btCylinderShapeX(halfExtents);
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    btVector3 halfExtents(radius, height/2, radius);
                    singleCollisionShape = new btCylinderShape(halfExtents);
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    btVector3 halfExtents(radius, radius, height/2);
                    singleCollisionShape = new btCylinderShapeZ(halfExtents);
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    btVector3 halfExtents(radius, radius, height/2);
                    singleCollisionShape = new btCylinderShapeZ(halfExtents);
                }
            }
            else if (shape_str.compare("Capsule") == 0 || shape_str.compare("capsule") == 0 ||shape_str.compare("CAPSULE") == 0){
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
                    singleCollisionShape = new btCapsuleShapeX(radius, height);
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    singleCollisionShape = new btCapsuleShape(radius, height);
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    singleCollisionShape = new btCapsuleShapeZ(radius, height);
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    singleCollisionShape = new btCapsuleShapeZ(radius, height);
                }
            }
            else if (shape_str.compare("Cone") == 0 || shape_str.compare("cone") == 0 ||shape_str.compare("CONE") == 0){
                double radius = bodyCollisionGeometry["radius"].as<double>();
                double height = bodyCollisionGeometry["height"].as<double>();
                radius *= m_scale;
                height *= m_scale;
                std::string axis = "z";
                if(bodyCollisionGeometry["axis"].IsDefined()){
                    axis = bodyCollisionGeometry["axis"].as<std::string>();
                }
                if (axis.compare("x") == 0 || axis.compare("X") == 0){
                    singleCollisionShape = new btConeShapeX(radius, height);
                }
                else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
                    singleCollisionShape = new btConeShape(radius, height);
                }
                else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
                    singleCollisionShape = new btConeShapeZ(radius, height);
                }
                else{
                    std::cerr << "WARNING: Body "
                              << m_name
                              << "'s axis \"" << axis << "\" not understood?\n";
                    singleCollisionShape = new btConeShapeZ(radius, height);
                }
            }
            // Here again, we consider both the inertial offset transform and the
            // shape offset transfrom. This will change the legacy behavior but
            // luckily only a few ADFs (i.e. -l 16,17 etc) use the compound collision
            // shape. So they shall be updated.
            compoundCollisionShape->addChildShape(getInverseInertialOffsetTransform() * shapeOffsetTrans, singleCollisionShape);
        }
        m_bulletCollisionShape = compoundCollisionShape;
    }

    if (bodyNamespace.IsDefined()){
        m_namespace = afUtils::removeAdjacentBackSlashes(bodyNamespace.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

    m_mass = bodyMass.as<double>();
    if(bodyController.IsDefined()){
        // Check if the linear controller is defined
        if (bodyController["linear"].IsDefined()){
            double P, I, D;
            P = bodyController["linear"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (bodyController["linear"]["I"].IsDefined()){
                I = bodyController["linear"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = bodyController["linear"]["D"].as<double>();
            m_controller.setLinearGains(P, I, D);
            m_controller.m_positionOutputType = afControlType::force;
            m_lin_gains_defined = true;
        }

        // Check if the angular controller is defined
        if(bodyController["angular"].IsDefined()){
            double P, I, D;
            P = bodyController["angular"]["P"].as<double>();
            // For legacy where we didn't define the I term
            if (bodyController["angular"]["I"].IsDefined()){
                I = bodyController["angular"]["I"].as<double>();
            }
            else{
                I = 0;
            }
            D = bodyController["angular"]["D"].as<double>();
            m_controller.setAngularGains(P, I, D);
            m_controller.m_orientationOutputType = afControlType::force;
            m_ang_gains_defined = true;
        }
    }

    if(!m_lin_gains_defined){
        // Use preset values for the controller since we are going to be using its output for the
        // internal velocity controller
        m_controller.setLinearGains(10, 0, 0);
        m_controller.m_positionOutputType = afControlType::velocity;
    }

    if(!m_ang_gains_defined){
        // Use preset values for the controller since we are going to be using its output for the
        // internal velocity controller
        m_controller.setAngularGains(10, 0, 0);
        m_controller.m_orientationOutputType = afControlType::velocity;
    }

    if (bodyControllerOutputType.IsDefined()){
        std::string outputType = bodyControllerOutputType.as<std::string>();
        if ( (outputType.compare("FORCE") == 0) || (outputType.compare("force") == 0)){
            m_controller.m_positionOutputType = afControlType::force;
            m_controller.m_orientationOutputType = afControlType::force;
        }
        else if ( (outputType.compare("VELOCITY") == 0) || (outputType.compare("velocity") == 0) ){
            m_controller.m_positionOutputType= afControlType::velocity;
            m_controller.m_orientationOutputType= afControlType::velocity;
        }
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

    cVector3d tempPos(0, 0, 0);
    cMatrix3d tempRot(0, 0, 0);
    if(bodyPos.IsDefined()){
        tempPos = toXYZ<cVector3d>(&bodyPos);
        setLocalPos(tempPos);
    }

    if(bodyRot.IsDefined()){
        double r = bodyRot["r"].as<double>();
        double p = bodyRot["p"].as<double>();
        double y = bodyRot["y"].as<double>();
        tempRot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(tempRot);
    }

    // Inertial origin in world
    cTransform T_iINw = getLocalTransform();

    // Mesh Origin in World
    cTransform T_mINw = T_iINw * afUtils::convertDataType<cTransform, btTransform>(getInertialOffsetTransform());

    m_initialPos = T_mINw.getLocalPos();
    m_initialRot = T_mINw.getLocalRot();
    setLocalPos(T_mINw.getLocalPos());
    setLocalRot(T_mINw.getLocalRot());

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
        m_publish_children_names = bodyPublishChildrenNames.as<bool>();
    }

    if (bodyPublishJointNames.IsDefined()){
        m_publish_joint_names = bodyPublishJointNames.as<bool>();
    }

    if (bodyPublishJointPositions.IsDefined()){
        m_publish_joint_positions = bodyPublishJointPositions.as<bool>();
    }

    if (bodyPublishFrequency.IsDefined()){
        m_min_publish_frequency = bodyPublishFrequency["low"].as<int>();
        m_max_publish_frequency = bodyPublishFrequency["high"].as<int>();
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
                mB->m_afWorld->m_collisionGroups[gNum].push_back(this);
                m_collisionGroupsIdx.push_back(gNum);
            }
            else{
                std::cerr << "WARNING: Body "
                          << m_name
                          << "'s group number is \"" << gNum << "\" which should be between [0 - 999], ignoring\n";
            }
        }
    }

    if (bodyPassive.IsDefined()){
        bool passive = bodyPassive.as<bool>();
        setPassive(passive);
    }

    setConfigProperties(this, &m_surfaceProps);
    m_afWorld->addChild(this);
    return true;
}


///
/// \brief afRigidBody::enableShaderProgram
///
void afRigidBody::enableShaderProgram(){

    if (m_shaderProgramDefined){

        std::ifstream vsFile;
        std::ifstream fsFile;
        vsFile.open(m_vsFilePath.c_str());
        fsFile.open(m_fsFilePath.c_str());
        // create a string stream
        std::stringstream vsBuffer, fsBuffer;
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
            // Set the ID for shadow and normal maps.
            shaderProgram->setUniformi("shadowMap", C_TU_SHADOWMAP);
            bool enable_normal_mapping = false;
            for (int i = 0 ; i < m_meshes->size() ; i++){
                cMesh* mesh = (*m_meshes)[i];
                if (mesh->m_normalMap.get() != nullptr){
                    if (mesh->m_normalMap->m_image.get() != nullptr){
                        enable_normal_mapping = true;
                    }
                }
            }
            if (enable_normal_mapping){
                shaderProgram->setUniformi("normalMap", C_TU_NORMALMAP);
                shaderProgram->setUniformi("vEnableNormalMapping", 1);
            }
            else{
                shaderProgram->setUniformi("vEnableNormalMapping", 0);
            }

            std::cerr << "INFO! FOR BODY: "<< m_name << ", USING SHADER FILES: " <<
                         "\n \t VERTEX: " << m_vsFilePath.c_str() <<
                         "\n \t FRAGMENT: " << m_fsFilePath.c_str() << std::endl;

            setShaderProgram(shaderProgram);
        }
        else{
            std::cerr << "ERROR! FOR BODY: "<< m_name << ", FAILED TO LOAD SHADER FILES: " <<
                         "\n \t VERTEX: " << m_vsFilePath.c_str() <<
                         "\n \t FRAGMENT: " << m_fsFilePath.c_str() << std::endl;

            m_shaderProgramDefined = false;
        }
    }
    // Check if the shader has been assigned by afWorld
    else if (getShaderProgram() != nullptr){
        m_shaderProgramDefined = true;
    }
    else{
        m_shaderProgramDefined = false;
    }
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
    if (m_lin_gains_defined && m_ang_gains_defined){
        return;
    }

    double P_lin, D_lin, P_ang, D_ang;
    double lumped_mass = m_mass;
    cVector3d lumped_intertia = m_inertia;
    std::vector<afChildJointPair>::iterator sjIt;
    for(sjIt = m_CJ_PairsActive.begin() ; sjIt != m_CJ_PairsActive.end() ; ++sjIt){
        lumped_mass += sjIt->m_childBody->getMass();
        lumped_intertia += sjIt->m_childBody->getInertia();
    }
    if (!m_lin_gains_defined){
        P_lin = lumped_mass * 20;
        D_lin = P_lin / 100;
        m_controller.setLinearGains(P_lin, 0, D_lin);
        m_lin_gains_defined = true;
    }
    // TODO
    // Need a better way of estimating angular gains
    if (!m_ang_gains_defined){
        P_ang = lumped_mass * 10;
        D_ang = lumped_mass;
        m_controller.setAngularGains(P_ang, 0, D_ang);
        m_ang_gains_defined = true;
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
        // Inertial and Mesh Transform
        btTransform T_iINw, T_mINw;
        m_bulletRigidBody->getMotionState()->getWorldTransform(T_iINw);
        T_mINw = T_iINw * getInverseInertialOffsetTransform();

        btVector3 pos = T_mINw.getOrigin();
        btQuaternion q = T_mINw.getRotation();

        // set new position
        m_localPos.set(pos[0],pos[1],pos[2]);

        // set new orientation
        cQuaternion quaternion(q.getW(), q.getX(), q.getY(), q.getZ());
        quaternion.toRotMat(m_localRot);

        // orthogonalize frame
        m_localRot.orthogonalize();
    }

    // Update the data for sensors
//#ifdef AMBF_ENABLE_PARALLEL_SENSOR_PROCESSING
//    for (int thIdx = 0 ; thIdx < m_sensorThreads.size() ; thIdx++){
//        m_threadUpdateFlags[thIdx] = true;
//    }
//#else
//    for (int i = 0 ; i < m_afSensors.size() ; i++){
//        m_afSensors[i]->updatePositionFromDynamics();
//    }
//#endif

    // update Transform data for m_ObjectPtr
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if(m_afRigidBodyCommPtr.get() != nullptr){
        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        cQuaternion q;
        q.fromRotMat(m_localRot);

        // Update the Pose
        m_afRigidBodyCommPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
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
bool afRigidBody::updateBodySensors(int threadIdx){
    int startIdx = threadIdx * m_sensorThreadBlockSize;
    int endIdx = startIdx + m_sensorThreadBlockSize;

    endIdx = endIdx > m_afSensors.size() ? m_afSensors.size() : endIdx;
    while (m_keepSensorThreadsAlive){
        if (m_threadUpdateFlags[threadIdx] == true){

            for (int idx = startIdx ; idx < endIdx ; idx++){
                m_afSensors[idx]->updatePositionFromDynamics();
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
void afRigidBody::afExecuteCommand(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afRigidBodyCommPtr.get() != nullptr){
        btVector3 force, torque;
        btVector3 lin_vel, ang_vel;
        ambf_msgs::RigidBodyCmd afCommand = m_afRigidBodyCommPtr->get_command();

        // IF THE COMMAND IS OF TYPE FORCE
        if (afCommand.cartesian_cmd_type == ambf_msgs::RigidBodyCmd::TYPE_FORCE){
            m_activeControllerType = afControlType::force;
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
            m_activeControllerType = afControlType::position;
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
//                    std::cerr << "Updating Static Object Pose \n";
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
                    std::cerr << "WARNING: BODY \"" << m_name << "'s\" rotation quaternion command"
                                                                 " not normalized" << std::endl;
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

                if (m_controller.m_positionOutputType == afControlType::force){
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
            m_activeControllerType = afControlType::velocity;
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
                    std::cerr << "WARNING! FOR JOINT \"" <<
                                 m_CJ_PairsActive[jntIdx].m_childJoint->getName() <<
                                 " \" COMMAND TYPE NOT UNDERSTOOD, SUPPORTED TYPES ARE 0 -> FORCE, 1 -> POSITION, 2 -> VELOCITY " <<
                                 std::endl;
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
        std::vector<std::string> children_names;

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
        std::vector<std::string> joint_names;
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
        for (size_t jnt = 0 ; jnt < m_CJ_PairsActive.size() ; jnt++){
            m_CJ_PairsActive[jnt].m_childJoint->commandPosition(angle);
        }

    }
}


///
/// \brief afRigidBody::setAngle
/// \param angles
///
void afRigidBody::setAngle(std::vector<double> &angles){
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
    std::vector<afChildJointPair>::iterator cjIt;
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

///
/// \brief afSoftBody::afSoftBody
/// \param a_chaiWorld
///
afSoftBody::afSoftBody(afWorldPtr a_afWorld): afSoftMultiMesh(a_afWorld){
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
        m_namespace = afUtils::removeAdjacentBackSlashes(softBodyNameSpace.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

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
        rot.setExtrinsicEulerRotationRad(r, p, y,cEulerOrder::C_EULER_ORDER_XYZ);
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

    m_afWorld->addChild(this);
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
    int n = queue_length - 1;
    for (size_t i = 0 ; i < n ; i++){
        t[i] = t[i+1];
        e[i] = e[i+1];
        de[i] = de[i+1];
        ie[i] = ie[i+1];
    }
    t[n] = current_time;
    e[n] = set_point - process_val;
    double dt = t[n] - t[n-1];
    // Clamp freq to 10 KHZ
    if (dt <= 0.00001){
        dt = 0.00001;
    }
    de[n] = (e[n] - e[n-1]) / dt;
    ie[n] = Ie_sum + ((e[n] + e[n-1]) / 2 * dt);
    Ie_sum = ie[n];
    output = (P * e[n]) + (I * ie[n]) + (D * de[n]);
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
    m_posArray.resize(m_jpSize);
    m_dtArray.resize(m_jpSize);
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
    YAML::Node jointEnableFeedback = jointNode["enable feedback"];
    YAML::Node jointMaxMotorImpulse = jointNode["max motor impulse"];
    YAML::Node jointLimits = jointNode["joint limits"];
    YAML::Node jointERP = jointNode["joint erp"];
    YAML::Node jointCFM = jointNode["joint cfm"];
    YAML::Node jointOffset = jointNode["offset"];
    YAML::Node jointDamping = jointNode["damping"];
    YAML::Node jointStiffness = jointNode["stiffness"];
    YAML::Node jointType = jointNode["type"];
    YAML::Node jointController = jointNode["controller"];
    YAML::Node jointControllerOutputType = jointNode["controller output type"];
    YAML::Node jointIgnoreInterCollision = jointNode["ignore inter-collision"];
    YAML::Node jointPassive = jointNode["passive"];

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

    m_mB = mB;

    // First we should search in the local MultiBody space and if we don't find the body.
    // On then we find the world space

    std::string qualified_name_a = mB->getNamespace() + m_parentName;
    std::string qualified_name_b = mB->getNamespace() + m_childName;

    m_afParentBody = mB->getAFRigidBodyLocal(qualified_name_a, true);
    m_afChildBody = mB->getAFRigidBodyLocal(qualified_name_b, true);

    if (m_afParentBody == nullptr){
        m_afParentBody = m_afWorld->getAFRigidBody(qualified_name_a + name_remapping, true);
    }
    if (m_afChildBody == nullptr){
        m_afChildBody = m_afWorld->getAFRigidBody(qualified_name_b + name_remapping, true);
    }

    bool _ignore_inter_collision = true;

    // If we couldn't find the body with name_remapping, it might have been
    // Defined in another ambf file. Search without name_remapping string
    if(m_afParentBody == nullptr){
        m_afParentBody = m_afWorld->getAFRigidBody(m_parentName, true);
        // If any body is still not found, print error and ignore joint
        if (m_afParentBody == nullptr){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" PARENT BODY \"" << m_parentName <<
                        "\" NOT FOUND" << std::endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ((!strcmp(m_afParentBody->m_name.c_str(), "world") == 0)
                &&(!strcmp(m_afParentBody->m_name.c_str(), "World") == 0)
                &&(!strcmp(m_afParentBody->m_name.c_str(), "WORLD") == 0)){
            //            std::cerr <<"INFO: JOINT: \"" << m_name <<
            //                        "\'s\" PARENT BODY \"" << m_parentName <<
            //                        "\" FOUND IN ANOTHER AMBF CONFIG," << std::endl;
        }
    }
    if(m_afChildBody == nullptr){
        m_afChildBody = m_afWorld->getAFRigidBody(m_childName, true);
        // If any body is still not found, print error and ignore joint
        if (m_afChildBody == nullptr){
            std::cerr <<"ERROR: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" NOT FOUND" << std::endl;
            return 0;
        }
        // If the body is not world, print what we just did
        if ((!strcmp(m_afChildBody->m_name.c_str(), "world") == 0)
                &&(!strcmp(m_afChildBody->m_name.c_str(), "World") == 0)
                &&(!strcmp(m_afChildBody->m_name.c_str(), "WORLD") == 0)){
            std::cerr <<"INFO: JOINT: \"" << m_name <<
                        "\'s\" CHILD BODY \"" << m_childName <<
                        "\" FOUND IN ANOTHER AMBF CONFIG," << std::endl;
        }
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
        m_pvtA *= m_afParentBody->m_scale;
        m_pvtA = m_afParentBody->getInertialOffsetTransform().inverse() * m_pvtA;
        m_pvtB = m_afChildBody->getInertialOffsetTransform().inverse() * m_pvtB;
        m_axisA = m_afParentBody->getInertialOffsetTransform().getBasis().inverse() * m_axisA;
        m_axisB = m_afChildBody->getInertialOffsetTransform().getBasis().inverse() * m_axisB;
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

        // If the PID controller in defined, the gains will be used to command the joint force (effort)
        m_controller.m_outputType = afControlType::force;
    }
    else{
        // If the controller gains are not defined, a velocity based control will be used.
        // The tracking velocity can be controller by setting "max motor impulse" field
        // for the joint data-block in the ADF file.
        m_controller.P = 10;
        m_controller.I = 0;
        m_controller.D = 0;
        m_controller.m_outputType = afControlType::velocity;
    }

    if (jointControllerOutputType.IsDefined()){
        std::string outputType = jointControllerOutputType.as<std::string>();
        if ( (outputType.compare("FORCE") == 0) || (outputType.compare("force") == 0)){
            m_controller.m_outputType= afControlType::force;
        }
        else if ( (outputType.compare("VELOCITY") == 0) || (outputType.compare("velocity") == 0) ){
            m_controller.m_outputType= afControlType::velocity;
        }
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
    if (m_jointType == JointType::revolute){
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
    else if (m_jointType == JointType::prismatic){
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
    else if (m_jointType == JointType::linear_spring || m_jointType == JointType::torsion_spring){
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

        if (jointNode["equilibrium point"].IsDefined()){
            double _equiblirium = jointNode["equilibrium point"].as<double>();
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
    else if (m_jointType == JointType::p2p){
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
    else if (m_jointType == JointType::fixed){
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
    for (int i = 0 ; i < m_jpSize-1 ; i++){
        m_posArray[i] = m_posArray[i+1];
        m_dtArray[i] = m_dtArray[i+1];
    }
    m_posArray[m_jpSize-1] = getPosition();
    m_dtArray[m_jpSize-1] = dt;
    double effort = - m_jointDamping * getVelocity();
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
        if (m_jointType == JointType::revolute || m_jointType == JointType::prismatic){
            // Sanity check
            btClamp(position_cmd, m_lowerLimit, m_upperLimit);
            double position_cur = getPosition();

            if (m_jointType == JointType::revolute){
                if ((m_upperLimit - m_lowerLimit) >= 2*PI ){
                    // The joint is continous. Need some optimization
                    position_cmd = getShortestAngle(position_cur, position_cmd);
                    position_cur = 0.0;
                }

            }

            double command = m_controller.computeOutput(position_cur, position_cmd, m_afWorld->getSimulationTime());
            if (m_controller.m_outputType == afControlType::force){
                commandEffort(command);
            }
            else{
                commandVelocity(command);
            }
        }
    }
    else{
        std::cerr << "WARNING, MOTOR NOT ENABLED FOR JOINT: " << m_name << std::endl;
    }
}

double afJoint::getShortestAngle(double current, double target)
{
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
    if (m_jointType == JointType::revolute || m_jointType == JointType::torsion_spring){
        if (m_jointType == JointType::revolute && ! skip_motor_check){
            m_hinge->enableMotor(false);
        }
        btTransform trA = m_btConstraint->getRigidBodyA().getWorldTransform();
        btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
        m_btConstraint->getRigidBodyA().applyTorque(-hingeAxisInWorld * cmd);
        m_btConstraint->getRigidBodyB().applyTorque(hingeAxisInWorld * cmd);
    }
    else if (m_jointType == JointType::prismatic || m_jointType == JointType::linear_spring){
        if (m_jointType == JointType::prismatic && ! skip_motor_check){
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
    if (m_jointType == JointType::revolute){
        m_hinge->enableMotor(true);
        m_hinge->setMotorTargetVelocity(velocity_cmd);

    }
    else if (m_jointType == JointType::prismatic){
        m_slider->setPoweredLinMotor(true);
        m_slider->setTargetLinMotorVelocity(velocity_cmd);
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
afSensor::afSensor(afWorldPtr a_afWorld): afBaseObject(a_afWorld){
}


///
/// \brief afSensor::afExecuteCommand
/// \param dt
///
void afSensor::afExecuteCommand(double dt){

}


///
/// \brief afSensor::updatePositionFromDynamics
///
void afSensor::updatePositionFromDynamics(){

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

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node sensorParentName = sensorNode["parent"];
    YAML::Node sensorName = sensorNode["name"];
    YAML::Node sensorNamespace = sensorNode["namespace"];
    YAML::Node sensorPos = sensorNode["location"]["position"];
    YAML::Node sensorRot = sensorNode["location"]["orientation"];
    YAML::Node sensorRange = sensorNode["range"];
    YAML::Node sensorPublishFrequency = sensorNode["publish frequency"];
    YAML::Node sensorVisible = sensorNode["visible"];
    YAML::Node sensorVisibleSize = sensorNode["visible size"];
    YAML::Node sensorArray = sensorNode["array"];
    YAML::Node sensorMesh = sensorNode["mesh"];
    YAML::Node sensorParametric = sensorNode["parametric"];

    if (sensorParentName.IsDefined()){
        m_parentName = sensorParentName.as<std::string>();
    }
    else{
        result = false;
    }

    m_name = sensorName.as<std::string>();

    if(sensorPos.IsDefined()){
        m_initialPos = toXYZ<cVector3d>(&sensorPos);
        setLocalPos(m_initialPos);
    }

    if(sensorRot.IsDefined()){
        double r = sensorRot["r"].as<double>();
        double p = sensorRot["p"].as<double>();
        double y = sensorRot["y"].as<double>();
        m_initialRot.setExtrinsicEulerRotationRad(r,p,y,cEulerOrder::C_EULER_ORDER_XYZ);
        setLocalRot(m_initialRot);
    }

    if(sensorNamespace.IsDefined()){
        m_namespace = sensorNamespace.as<std::string>();
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

    m_range = 0.0;
    if(sensorRange.IsDefined()){
        m_range = sensorRange.as<double>();
    }

    if (m_range < 0.0){
        std::cerr << "ERROR! SENSOR RANGE CANNOT BE NEGATIVE" << std::endl;
        return 0;
    }

    if (sensorPublishFrequency.IsDefined()){
        m_min_publish_frequency = sensorPublishFrequency["low"].as<int>();
        m_max_publish_frequency = sensorPublishFrequency["high"].as<int>();
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

    // First search in the local space.
    m_parentBody = mB->getAFRigidBodyLocal(m_parentName);

    if(!m_parentBody){
        m_parentBody = m_afWorld->getAFRigidBody(m_parentName + name_remapping);
    }

    if (m_parentBody == nullptr){
        std::cerr << "ERROR: SENSOR'S "<< m_parentName + name_remapping << " NOT FOUND, IGNORING SENSOR\n";
        return 0;
    }
    else{
        m_parentBody->addAFSensor(this);
    }

    if (sensorArray.IsDefined()){
        m_count = sensorArray.size();
        m_sensedResults.resize(m_count);
        for (int i = 0 ; i < m_count ; i++){
            YAML::Node offsetNode = sensorArray[i]["offset"];
            YAML::Node directionNode = sensorArray[i]["direction"];
            cVector3d offset = toXYZ<cVector3d>(&offsetNode);
            cVector3d dir = toXYZ<cVector3d>(&directionNode);
            m_sensedResults[i].m_range = m_range;
            m_sensedResults[i].m_rayFromLocal = getLocalTransform() * offset;
            m_sensedResults[i].m_direction = getLocalRot() * dir;
            m_sensedResults[i].m_direction.normalize();
            m_sensedResults[i].m_rayToLocal = m_sensedResults[i].m_rayFromLocal + m_sensedResults[i].m_direction * m_sensedResults[i].m_range;

        }
        result = true;
    }

    else if (sensorMesh.IsDefined()){
        std::string mesh_name = sensorMesh.as<std::string>();
        mesh_name = mB->getHighResMeshesPath() + mesh_name;
        cMultiMesh* multiMesh = new cMultiMesh();
        if (multiMesh->loadFromFile(mesh_name)){
            cMesh* sourceMesh = (*multiMesh->m_meshes)[0];
            if (sourceMesh){
                m_count = sourceMesh->m_triangles->getNumElements();
                m_sensedResults.resize(m_count);
                for (int i = 0 ; i < m_count ; i++ ){

                    m_sensedResults[i].m_range = m_range;

                    int vIdx0 = sourceMesh->m_triangles->getVertexIndex0(i);
                    int vIdx1 = sourceMesh->m_triangles->getVertexIndex1(i);
                    int vIdx2 = sourceMesh->m_triangles->getVertexIndex2(i);

                    cVector3d v0 = sourceMesh->m_vertices->getLocalPos(vIdx0);
                    cVector3d v1 = sourceMesh->m_vertices->getLocalPos(vIdx1);
                    cVector3d v2 = sourceMesh->m_vertices->getLocalPos(vIdx2);

                    cVector3d e1 = v1 - v0;
                    cVector3d e2 = v2 - v1;

                    cVector3d centroid = ( v0 + v1 + v2 ) / 3;

                    cVector3d dir = cCross(e1, e2);

                    dir.normalize();
                    m_sensedResults[i].m_rayFromLocal = getLocalTransform() * centroid ;
                    m_sensedResults[i].m_direction = getLocalRot() * dir;
                    m_sensedResults[i].m_direction.normalize();
                    m_sensedResults[i].m_rayToLocal = m_sensedResults[i].m_rayFromLocal + m_sensedResults[i].m_direction * m_sensedResults[i].m_range;
                }
            }
            delete multiMesh;
            result = true;
        }
        else{
            std::cerr << "ERROR! BODY \"" << m_name <<
                         "\'s\" RESISTIVE MESH " <<
                         mesh_name << " NOT FOUND. IGNORING\n";
            result = false;
        }
    }
    else if (sensorParametric.IsDefined()){
        YAML::Node resolutionNode = sensorParametric["resolution"];
        YAML::Node horSpanNode = sensorParametric["horizontal angle"];
        YAML::Node verSpanNode = sensorParametric["vertical angle"];
        YAML::Node startOffsetNode = sensorParametric["start offset"];

        int resolution = resolutionNode.as<int>();
        double horizontal_span = horSpanNode.as<double>();
        double vertical_span = verSpanNode.as<double>();
        double start_offset = startOffsetNode.as<double>();

        if (resolution < 2){
            std::cerr << "ERROR! FOR SENSOR \"" << m_name << "\" RESOLUTION MUST BE GREATER THAN EQUAL TO 2. IGNORING! \n";
            return false;
        }

        double h_start = -horizontal_span / 2.0;
        double v_start = -vertical_span / 2.0;
        double h_step = horizontal_span / (resolution - 1);
        double v_step = vertical_span / (resolution - 1);
        m_count = resolution * resolution;
        m_sensedResults.resize(m_count);

        // Choose an initial point facing the +ve x direction
        cVector3d point(1, 0, 0);
        for (int i = 0 ; i < resolution ; i++){
            double h_angle = h_start + i * h_step;
            for (int j = 0 ; j < resolution ; j++){
                double v_angle = v_start + j * v_step;

                cMatrix3d mat;
                mat.setExtrinsicEulerRotationRad(0, v_angle, h_angle, cEulerOrder::C_EULER_ORDER_XYZ);

                cVector3d start_point = point * start_offset;
                cVector3d ray_from = mat * start_point;
                cVector3d dir = mat * point;
                cVector3d ray_to = ray_from + dir * m_range;

                dir.normalize();
                int sIdx = resolution * i + j;
                m_sensedResults[sIdx].m_range = m_range;
                m_sensedResults[sIdx].m_rayFromLocal = getLocalTransform() * ray_from ;
                m_sensedResults[sIdx].m_direction = getLocalRot() * dir;
                m_sensedResults[sIdx].m_direction.normalize();
                m_sensedResults[sIdx].m_rayToLocal = m_sensedResults[sIdx].m_rayFromLocal +
                        m_sensedResults[sIdx].m_direction *
                        m_sensedResults[sIdx].m_range;
            }

        }

    }
    else{
        m_count = 0;
        result = false;
    }

    if (m_showSensor){
        enableVisualization();
    }


    return result;
}


///
/// \brief afRayTracerSensor::updatePositionFromDynamics
///
void afRayTracerSensor::updatePositionFromDynamics(){

    if (m_parentBody == nullptr){
        return;
    }
    cTransform T_bInw = m_parentBody->getLocalTransform();
    for (int i = 0 ; i < m_count ; i++){
        btVector3 rayFromWorld, rayToWorld;
        rayFromWorld = toBTvec(T_bInw *  m_sensedResults[i].m_rayFromLocal);
        rayToWorld = toBTvec(T_bInw *  m_sensedResults[i].m_rayToLocal);

        // Check for global flag for debug visibility of this sensor
        if (m_showSensor){
            m_sensedResults[i].m_fromSphereMesh->setLocalPos(toCvec(rayFromWorld) );
            m_sensedResults[i].m_toSphereMesh->setLocalPos(toCvec(rayToWorld) );
        }

        btCollisionWorld::ClosestRayResultCallback rayCallBack(rayFromWorld, rayToWorld);
        m_afWorld->m_bulletWorld->rayTest(rayFromWorld, rayToWorld, rayCallBack);
        if (rayCallBack.hasHit()){
            if (m_showSensor){
                m_sensedResults[i].m_hitSphereMesh->setLocalPos(toCvec(rayCallBack.m_hitPointWorld));
                m_sensedResults[i].m_hitSphereMesh->setShowEnabled(true);
            }
            m_sensedResults[i].m_triggered = true;
            if (rayCallBack.m_collisionObject->getInternalType()
                    == btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY){
                m_sensedResults[i].m_sensedBTRigidBody = (btRigidBody*)btRigidBody::upcast(rayCallBack.m_collisionObject);
                m_sensedResults[i].m_sensedAFRigidBody = m_afWorld->getAFRigidBody(m_sensedResults[i].m_sensedBTRigidBody);
                m_sensedResults[i].m_sensedBodyType = afBodyType::RIGID_BODY;
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
                    m_sensedResults[i].m_sensedSoftBodyFaceIdx = sensedSoftBodyFaceIdx;
                    m_sensedResults[i].m_sensedSoftBodyFace = &sensedSoftBody->m_faces[sensedSoftBodyFaceIdx];
                    m_sensedResults[i].m_sensedBTSoftBody = sensedSoftBody;
                    m_sensedResults[i].m_sensedAFSoftBody = m_afWorld->getAFSoftBody(m_sensedResults[i].m_sensedBTSoftBody);
                    m_sensedResults[i].m_sensedBodyType = afBodyType::SOFT_BODY;
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
                    m_sensedResults[i].m_sensedSoftBodyNodeIdx = sensedSoftBodyNodeIdx;
                    m_sensedResults[i].m_sensedSoftBodyNode = &sensedSoftBody->m_nodes[sensedSoftBodyNodeIdx];
                    m_sensedResults[i].m_sensedBTSoftBody = sensedSoftBody;
                    m_sensedResults[i].m_sensedBodyType = afBodyType::SOFT_BODY;
                }
            }
            m_sensedResults[i].m_depthFraction = (1.0 - rayCallBack.m_closestHitFraction);
            m_sensedResults[i].m_contactNormal = toCvec(rayCallBack.m_hitNormalWorld);
            m_sensedResults[i].m_sensedLocationWorld = toCvec(rayCallBack.m_hitPointWorld);
        }
        else{
            if(m_showSensor){
                m_sensedResults[i].m_hitSphereMesh->setShowEnabled(false);
            }
            m_sensedResults[i].m_triggered = false;
            m_sensedResults[i].m_depthFraction = 0;
        }
    }

#if C_ENABLE_AMBF_COMM_SUPPORT
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

    std::vector<bool> triggers;
    triggers.resize(m_count);

    std::vector<std::string> sensed_obj_names;
    sensed_obj_names.resize(m_count);

    std::vector<double> measurements;
    measurements.resize(m_count);

    for (int i = 0 ; i < m_count ; i++){
        triggers[i] = m_sensedResults[i].m_triggered;
        measurements[i] = m_sensedResults[i].m_depthFraction;
        if (m_sensedResults[i].m_triggered){
            if (m_sensedResults[i].m_sensedAFRigidBody){
                sensed_obj_names[i] = m_sensedResults[i].m_sensedAFRigidBody->m_name;
            }
            if (m_sensedResults[i].m_sensedAFSoftBody){
                sensed_obj_names[i] = m_sensedResults[i].m_sensedAFSoftBody->m_name;
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

///
/// \brief afRayTracerSensor::visualize
///
void afRayTracerSensor::enableVisualization(){
    for (int i = 0 ; i < m_count ; i++){
        if (m_sensedResults[i].m_hitSphereMesh == nullptr){
            cMesh* mesh = new cMesh();
            cCreateSphere(mesh, m_visibilitySphereRadius);
            m_afWorld->addChild(mesh);
            mesh->m_material->setPinkHot();
            mesh->setShowEnabled(false);
            mesh->setUseDisplayList(true);
            mesh->markForUpdate(false);
            m_sensedResults[i].m_hitSphereMesh = mesh;
        }

        if (m_sensedResults[i].m_fromSphereMesh == nullptr){
            cMesh* mesh = new cMesh();
            cCreateSphere(mesh, m_visibilitySphereRadius);
            m_afWorld->addChild(mesh);
            mesh->m_material->setRed();
            mesh->setShowEnabled(true);
            mesh->setUseDisplayList(true);
            mesh->markForUpdate(false);
            m_sensedResults[i].m_fromSphereMesh = mesh;
        }

        if (m_sensedResults[i].m_toSphereMesh == nullptr){
            cMesh* mesh = new cMesh();
            cCreateSphere(mesh, m_visibilitySphereRadius);
            m_afWorld->addChild(mesh);
            mesh->m_material->setGreen();
            mesh->setShowEnabled(true);
            mesh->setUseDisplayList(true);
            mesh->markForUpdate(false);
            m_sensedResults[i].m_toSphereMesh = mesh;
        }

        if (m_sensedResults[i].m_hitNormalMesh == nullptr){
            cMesh* mesh = new cMesh();
            cCreateArrow(mesh, m_visibilitySphereRadius*10,
                         m_visibilitySphereRadius*0.5,
                         m_visibilitySphereRadius*1,
                         m_visibilitySphereRadius*0.8,
                         false);
            m_afWorld->addChild(mesh);
            mesh->m_material->setGreenForest();
            mesh->setShowEnabled(false);
            mesh->setUseDisplayList(true);
            mesh->markForUpdate(false);
            m_sensedResults[i].m_hitNormalMesh = mesh;
        }
    }
}


///
/// \brief afRayTracerSensor::afExecuteCommand
/// \param dt
///
void afRayTracerSensor::afExecuteCommand(double dt){

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

        m_resistanceContacts.resize(m_count);

        for (int i = 0 ; i < m_count ; i++){
            m_resistanceContacts[i].m_bodyAContactPointLocal.set(0,0,0);
            m_resistanceContacts[i].m_bodyBContactPointLocal.set(0,0,0);

            m_resistanceContacts[i].m_tangentialError.set(0,0,0);
            m_resistanceContacts[i].m_tangentialErrorLast.set(0,0,0);

            m_resistanceContacts[i].m_contactPointsValid = false;

        }
    }
    return result;
}


///
/// \brief afResistanceSensor::updatePositionFromDynamics
///
void afResistanceSensor::updatePositionFromDynamics(){
    // Let's update the RayTracer Sensor First
    afRayTracerSensor::updatePositionFromDynamics();

    if (m_parentBody == nullptr){
        return;
    }

    for (int i = 0 ; i < m_count ; i++){

        if (isTriggered(i)){
            if (m_showSensor){
                m_sensedResults[i].m_hitNormalMesh->setLocalPos(getSensedPoint(i));
                m_sensedResults[i].m_hitNormalMesh->setLocalRot(afUtils::getRotBetweenVectors<cMatrix3d,
                                                                cVector3d>(cVector3d(0,0,1), m_sensedResults[i].m_contactNormal));
                m_sensedResults[i].m_hitNormalMesh->setShowEnabled(true);
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
                btVector3 P_cINw = toBTvec(getSensedPoint(i));
                btVector3 P_cINa = T_wINa * P_cINw;
                btVector3 vel_aINw = getParentBody()->m_bulletRigidBody->getLinearVelocity();
                btVector3 omega_aINw = getParentBody()->m_bulletRigidBody->getAngularVelocity();
                btVector3 N_a = toBTvec(m_sensedResults[i].m_direction);
                btVector3 N_aINw = T_aINw.getBasis() * N_a;

                btTransform T_bINw = getSensedBTRigidBody(i)->getWorldTransform();
                btTransform T_wINb = T_bINw.inverse(); // Invert once to save computation later
                btVector3 P_cINb = T_wINb * P_cINw;
                btVector3 vel_bINw = getSensedBTRigidBody(i)->getLinearVelocity();
                btVector3 omega_bINw = getSensedBTRigidBody(i)->getAngularVelocity();
                btVector3 N_bINw = toBTvec(m_sensedResults[i].m_contactNormal);
                btVector3 N_b = T_wINb.getBasis() * N_bINw;

                double depthFractionLast = m_sensedResults[i].m_depthFraction;

                if (m_sensedResults[i].m_depthFraction < 0 || m_sensedResults[i].m_depthFraction > 1){
                    std::cerr << "LOGIC ERROR! "<< m_name <<" Depth Fraction is " << m_sensedResults[i].m_depthFraction <<
                                 ". It should be between [0-1]" << std::endl;
                    std::cerr << "Ray Start: "<< m_sensedResults[i].m_rayFromLocal <<"\nRay End: " << m_sensedResults[i].m_rayToLocal <<
                                 "\nSensed Point: " << toCvec(P_cINa) << std::endl;
                    std::cerr << "----------\n";
                    m_sensedResults[i].m_depthFraction = 0;
                }

                // First calculate the normal contact force
                btVector3 F_n_a = ((m_contactNormalStiffness * m_sensedResults[i].m_depthFraction)
                                   + m_contactNormalDamping * (m_sensedResults[i].m_depthFraction - depthFractionLast)) * (N_a);
                F_n_w = T_aINw.getBasis() * F_n_a;

                double coeffScale = 1;
                if (m_useVariableCoeff){
                    coeffScale = F_n_w.length();
                }

                if(m_resistanceContacts[i].m_contactPointsValid){
                    btVector3 P_aINw = T_aINw * toBTvec(m_resistanceContacts[i].m_bodyAContactPointLocal);
                    btVector3 P_bINw = T_bINw * toBTvec(m_resistanceContacts[i].m_bodyBContactPointLocal);
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
                    btVector3 tangentialErrorLast = toBTvec(m_resistanceContacts[i].m_tangentialErrorLast);
                    m_resistanceContacts[i].m_tangentialErrorLast = m_resistanceContacts[i].m_tangentialError;
                    m_resistanceContacts[i].m_tangentialError = toCvec(tangentialError);

                    if (tangentialError.length() > 0.0 && tangentialError.length() <= m_contactArea){
                        F_s_w = m_staticContactFriction * coeffScale * tangentialError +
                                m_staticContactDamping * (tangentialError - tangentialErrorLast);
                    }
                    else{
                        m_resistanceContacts[i].m_contactPointsValid = false;
                    }

                    //                std::cerr << "F Static: " << F_static << std::endl;
                    //                std::cerr << "F Normal: " << F_normal << std::endl;
                    //                std::cerr << "Depth Ra: " << m_depthFraction << std::endl;
                    //                std::cerr << "------------\n";
                }
                else{
                    m_resistanceContacts[i].m_bodyAContactPointLocal = toCvec(T_wINa * toBTvec(getSensedPoint(i)));
                    m_resistanceContacts[i].m_bodyBContactPointLocal = toCvec(T_wINb * toBTvec(getSensedPoint(i)));
                    m_resistanceContacts[i].m_contactPointsValid = true;
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
            m_resistanceContacts[i].m_contactPointsValid = false;
            m_resistanceContacts[i].m_firstTrigger = true;

            if(m_showSensor){
                m_sensedResults[i].m_hitNormalMesh->setShowEnabled(false);
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
/// \brief afPointCloudsHandler::afPointCloudsHandler
/// \param a_afWorld
///
afPointCloudsHandler::afPointCloudsHandler(afWorldPtr a_afWorld): afBaseObject(a_afWorld){
}


///
/// \brief afPointCloudsHandler::updatePositionFromDynamics
///
void afPointCloudsHandler::updatePositionFromDynamics(){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT

    std::map<std::string, afMultiPointUnit>::iterator it;

    for (it = m_pcMap.begin() ; it != m_pcMap.end() ; ++it){
        std::string pc_topic_name = it->first;
        cMultiPointPtr mpPtr = it->second.m_mpPtr;
        ambf_comm::PointCloudHandlerPtr pchPtr = it->second.m_pchPtr;

        int mp_size = mpPtr->getNumPoints();
        sensor_msgs::PointCloudPtr pcPtr = pchPtr->get_point_cloud();
        if(pcPtr){
            double radius = pchPtr->get_radius();
            mpPtr->setPointSize(radius);
            int pc_size = pcPtr->points.size();
            int diff = pc_size - mp_size;
            std::string frame_id = pcPtr->header.frame_id;

            if (it->second.m_parentName.compare(frame_id) != 0 ){
                // First remove any existing parent
                if (mpPtr->getParent() != nullptr){
                    mpPtr->getParent()->removeChild(mpPtr);
                }

                afRigidBodyPtr pBody = m_afWorld->getAFRigidBody(frame_id);
                if(pBody){
                    pBody->addChild(mpPtr);
                }
                else{
                    // Parent not found.
                    std::cerr << "WARNING! FOR POINT CLOUD \""<< pc_topic_name <<
                                 "\" PARENT BODY \"" << frame_id <<
                                 "\" NOT FOUND, SETTING WORLD AS PARENT" <<
                                 std::endl;
                    addChild(mpPtr);
                }
            }

            it->second.m_parentName = frame_id;

            if (diff >= 0){
                // PC array has either increased in size or the same size as MP array
                for (int pIdx = 0 ; pIdx < mp_size ; pIdx++){
                    cVector3d pcPos(pcPtr->points[pIdx].x,
                                    pcPtr->points[pIdx].y,
                                    pcPtr->points[pIdx].z);
                    mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
                }

                // Now add the new PC points to MP
                for (int pIdx = mp_size ; pIdx < mp_size + pc_size ; pIdx++){
                    cVector3d pcPos(pcPtr->points[pIdx].x,
                                    pcPtr->points[pIdx].y,
                                    pcPtr->points[pIdx].z);
                    mpPtr->newPoint(pcPos);
                }
            }
            else{
                // PC array has decreased in size as compared to MP array
                for (int pIdx = 0 ; pIdx < pc_size ; pIdx++){
                    cVector3d pcPos(pcPtr->points[pIdx].x,
                                    pcPtr->points[pIdx].y,
                                    pcPtr->points[pIdx].z);
                    mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
                }

                for (int pIdx = mp_size ; pIdx > pc_size ; pIdx--){
                    mpPtr->removePoint(pIdx-1);
                }
            }

        }

    }
#endif

}


///
/// \brief afWorld::afWorld
/// \param a_chaiWorld
/// \param a_global_namespace
///
afWorld::afWorld(std::string a_global_namespace){
    m_maxIterations = 10;
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
    addChild(m_pickSphere);
    m_pickColor.setOrangeTomato();
    m_pickColor.setTransparencyLevel(0.3);
    m_namespace = "";
    setGlobalNamespace(a_global_namespace);

    m_pointCloudHandlerPtr = new afPointCloudsHandler(this);
    addChild(m_pointCloudHandlerPtr);
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
/// \brief afWorld::getFullyQualifiedName
/// \param a_name
/// \return
///
std::string afWorld::resolveGlobalNamespace(std::string a_name){
    std::string fully_qualified_name = getGlobalNamespace() + a_name;
    fully_qualified_name = afUtils::removeAdjacentBackSlashes(fully_qualified_name);
    return fully_qualified_name;
}


///
/// \brief afWorld::setGlobalNamespace
/// \param a_global_namespace
///
void afWorld::setGlobalNamespace(std::string a_global_namespace){
    m_global_namespace = a_global_namespace;
    if (!m_global_namespace.empty()){
        std::cerr << " INFO! FORCE PREPENDING GLOBAL NAMESPACE \"" << m_global_namespace << "\" \n" ;
    }
}


///
/// \brief afWorld::resetCameras
///
void afWorld::resetCameras(){
    afCameraMap::iterator camIt;
    for (camIt = m_afCameraMap.begin() ; camIt != m_afCameraMap.end() ; camIt++){
        afCameraPtr afCam = (camIt->second);
        cTransform c_T(afCam->getInitialPosition(), afCam->getInitialRotation());
        afCam->setLocalTransform(c_T);
    }

}

///
/// \brief afWorld::resetWorld
/// \param reset_time
///
void afWorld::resetDynamicBodies(bool reset_time){
    pausePhysics(true);

    afRigidBodyMap::iterator rbIt;

    for (rbIt = m_afRigidBodyMap.begin() ; rbIt != m_afRigidBodyMap.end() ; rbIt++){
        afRigidBodyPtr afRB = (rbIt->second);
        btRigidBody* rB = afRB->m_bulletRigidBody;
        btVector3 zero(0, 0, 0);
        rB->clearForces();
        rB->setLinearVelocity(zero);
        rB->setAngularVelocity(zero);
        cTransform c_T(afRB->getInitialPosition(), afRB->getInitialRotation());
        btTransform bt_T = afUtils::convertDataType<btTransform, cTransform>(c_T);
        rB->getMotionState()->setWorldTransform(bt_T);
        rB->setWorldTransform(bt_T);
    }

    if (reset_time){
//        s_bulletWorld->setSimulationTime(0.0);
    }

    pausePhysics(false);
}


///
/// \brief afWorld::afExecuteCommand
/// \param dt
///
void afWorld::afExecuteCommand(double dt){
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

            std::vector<std::string> def_topics = m_afWorldCommPtr->get_defunct_topic_names();
            std::vector<std::string> new_topics = m_afWorldCommPtr->get_new_topic_names();

            for (int i = 0 ; i < def_topics.size() ; i++){
                std::string topic_name = def_topics[i];
                if (m_pointCloudHandlerPtr->m_pcMap.find(topic_name) != m_pointCloudHandlerPtr->m_pcMap.end()){
                    // Cleanup
                    cMultiPointPtr mpPtr = m_pointCloudHandlerPtr->m_pcMap.find(topic_name)->second.m_mpPtr;
                    mpPtr->removeFromGraph();
                    m_pointCloudHandlerPtr->m_pcMap.erase(topic_name);
                    delete mpPtr;
                }
            }

            for (int i = 0 ; i < new_topics.size() ; i++){
                std::string topic_name = new_topics[i];
                ambf_comm::PointCloudHandlerPtr pchPtr = m_afWorldCommPtr->get_point_clound_handler(topic_name);
                if (pchPtr){
                    cMultiPointPtr mpPtr = new cMultiPoint();
                    afMultiPointUnit mpUnit;
                    mpUnit.m_mpPtr = mpPtr;
                    mpUnit.m_pchPtr = pchPtr;
                    m_pointCloudHandlerPtr->m_pcMap[topic_name] = mpUnit;
                    // Add as child, the header in PC message can override the parent later
                    m_pointCloudHandlerPtr->addChild(mpPtr);
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

    afExecuteCommand(a_interval);

    m_wallClock = a_wallClock;

    double dt = getSimulationDeltaTime();
    // Read the AF_COMM commands and apply to all different types of objects
    afRigidBodyMap::iterator rbIt;
    for(rbIt = m_afRigidBodyMap.begin() ; rbIt != m_afRigidBodyMap.end() ; rbIt++){
        (rbIt->second)->afExecuteCommand(dt);
    }

    afCameraMap::iterator camIt;
    for(camIt = m_afCameraMap.begin() ; camIt != m_afCameraMap.end() ; camIt++){
        (camIt->second)->afExecuteCommand(dt);
    }

    afLightMap::iterator lightIt;
    for(lightIt = m_afLightMap.begin() ; lightIt != m_afLightMap.end() ; lightIt++){
        (lightIt->second)->afExecuteCommand(dt);
    }

//    afSensorMap::iterator senIt;
//    for(senIt = m_afSensorMap.begin() ; senIt != m_afSensorMap.end() ; senIt++){
//        (senIt->second)->afExecuteCommand(dt);
//    }

    afActuatorMap::iterator actIt;
    for (actIt = m_afActuatorMap.begin() ; actIt != m_afActuatorMap.end() ; ++actIt){
        (actIt->second)->afExecuteCommand(dt);
    }

    afVehicleMap::iterator vIt;
    for (vIt = m_afVehicleMap.begin() ; vIt != m_afVehicleMap.end() ; ++vIt){
        (vIt->second)->afExecuteCommand(dt);
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

    updatePositionFromDynamics();
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
            if (jnt->m_jointType == revolute || jnt->m_jointType == torsion_spring){
                jnt->m_estimatedEffort = btDot(jnt->m_axisA, T_jINp);
            }
            else if (jnt->m_jointType == prismatic || jnt->m_jointType == linear_spring){
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
void afWorld::updatePositionFromDynamics()
{

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_paramsSet == false){
        // Create a default point cloud to listen to
        m_afWorldCommPtr->append_point_cloud_topic(m_namespace + m_name + "/" + "point_cloud");
        m_afWorldCommPtr->set_params_on_server();
        m_paramsSet = true;
    }
#endif

    afUpdateTimes(getWallTime(), getSimulationTime());
    std::list<cBulletGenericObject*>::iterator i;

    estimateBodyWrenches();

    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cBulletGenericObject* nextItem = *i;
        nextItem->updatePositionFromDynamics();
    }

//    afSensorMap::iterator senIt;
//    for(senIt = m_afSensorMap.begin() ; senIt != m_afSensorMap.end() ; senIt++){
//        (senIt->second)->updatePositionFromDynamics();
//    }
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

    if (usePlanes){
        // bullet static walls and ground
        cBulletStaticPlane* bulletGround;

        cBulletStaticPlane* bulletBoxWall[4];

        bulletBoxWall[0] = new cBulletStaticPlane(this, cVector3d(0.0, -1.0, 0.0), -0.5 * box_w);
        bulletBoxWall[1] = new cBulletStaticPlane(this, cVector3d(0.0, 1.0, 0.0), -0.5 * box_w);
        bulletBoxWall[2] = new cBulletStaticPlane(this, cVector3d(-1.0, 0.0, 0.0), -0.5 * box_l);
        bulletBoxWall[3] = new cBulletStaticPlane(this, cVector3d(1.0, 0.0, 0.0), -0.5 * box_l);

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

            addChild(wall);
        }


        //////////////////////////////////////////////////////////////////////////
        // GROUND
        //////////////////////////////////////////////////////////////////////////

        // create ground plane
        bulletGround = new cBulletStaticPlane(this, cVector3d(0.0, 0.0, 1.0), -0.5 * box_h);

        // add plane to world as we will want to make it visibe
        addChild(bulletGround);

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

        boundaryWalls[0] = new cBulletBox(this, box_l, thickness, box_h); // Right Wall
        boundaryWalls[1] = new cBulletBox(this, box_l, thickness, box_h); // Left Wall
        boundaryWalls[2] = new cBulletBox(this, thickness, box_w, box_h); // Back Wall
        boundaryWalls[3] = new cBulletBox(this, thickness, box_w, box_h); // Front Wall
        boundaryWalls[4] = new cBulletBox(this, box_l + 0.5, box_w + 0.5, thickness); // Front Wall

        boundaryWalls[0]->setLocalPos(0, box_w/2, 0);
        boundaryWalls[1]->setLocalPos(0, -box_w/2, 0);
        boundaryWalls[2]->setLocalPos(-box_l/2, 0, 0);
        boundaryWalls[3]->setLocalPos(box_l/2, 0, 0);
        boundaryWalls[4]->setLocalPos(0, 0, -box_h/2);

        for(int i = 0 ; i < 5 ; i++){
            boundaryWalls[i]->m_material->setWhiteIvory();
            boundaryWalls[i]->setTransparencyLevel(0.5, true, true);
            boundaryWalls[i]->m_material->setShininess(1);
            addChild(boundaryWalls[i]);
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

    m_world_config_path = boost::filesystem::path(a_world_config).parent_path();
    printf("INFO! WORLD CONFIG PATH: %s \n", m_world_config_path.c_str());

    m_name = "World";

    YAML::Node worldEnclosureData = worldNode["enclosure"];
    YAML::Node worldLightsData = worldNode["lights"];
    YAML::Node worldCamerasData = worldNode["cameras"];
    YAML::Node worldEnvironment = worldNode["environment"];
    YAML::Node worldSkyBox = worldNode["skybox"];
    YAML::Node worldNamespace = worldNode["namespace"];
    YAML::Node worldMaxIterations = worldNode["max iterations"];
    YAML::Node worldGravity = worldNode["gravity"];
    YAML::Node worldShaders = worldNode["shaders"];

    if (worldNamespace.IsDefined()){
        m_namespace = afUtils::removeAdjacentBackSlashes(worldNamespace.as<std::string>());
    }

    afCreateCommInstance(afCommType::WORLD,
                         m_name,
                         resolveGlobalNamespace(m_namespace),
                         50,
                         2000,
                         10.0);

    if(worldMaxIterations.IsDefined()){
        if (worldMaxIterations.as<int>() > 1){
            m_maxIterations = worldMaxIterations.as<int>();
            std::cerr << "INFO! SETTING SIMULATION MAX ITERATIONS TO : " << m_maxIterations << std::endl;
        }
        else{
            std::cerr << "ERROR! USER SPECIFIED MAX ITERATIONS < 2 : " << worldMaxIterations.as<int>()<< std::endl;
            std::cerr << "INFO! IGNORING AND USING MAX ITERATIONS : " << m_maxIterations << std::endl;
        }
    }

    cVector3d gravityVector(0, 0, -9.81);

    if (worldGravity.IsDefined()){
        double x, y, z;
        x = worldGravity["x"].as<double>();
        y = worldGravity["y"].as<double>();
        z = worldGravity["z"].as<double>();
        gravityVector.set(x, y, z);
    }

    setGravity(gravityVector);

    if (worldEnclosureData.IsDefined()){
        m_encl_length = worldEnclosureData["length"].as<double>();
        m_encl_width =  worldEnclosureData["width"].as<double>();
        m_encl_height = worldEnclosureData["height"].as<double>();
    }

    m_light = new cPositionalLight(this);
    m_light->setLocalPos(2, 2, 5);
    m_light->setShowEnabled(true);
    m_light->setEnabled(true);
    addChild(m_light);

    bool env_defined = false;
    if(worldEnvironment.IsDefined()){
        std::string world_adf = worldEnvironment.as<std::string>();
        boost::filesystem::path p(world_adf);
        if (p.is_relative()){
            p = m_world_config_path / p;
        }
        env_defined = loadADF(p.string(), false);
    }

    if (!env_defined){
        createDefaultWorld();
    }

    if (worldSkyBox.IsDefined()){
        boost::filesystem::path skybox_path = worldSkyBox["path"].as<std::string>();

        if (skybox_path.is_relative()){
            skybox_path = m_world_config_path / skybox_path;
        }

        if (worldSkyBox["right"].IsDefined() &&
                worldSkyBox["left"].IsDefined() &&
                worldSkyBox["top"].IsDefined() &&
                worldSkyBox["bottom"].IsDefined() &&
                worldSkyBox["front"].IsDefined() &&
                worldSkyBox["back"].IsDefined()
                )
        {
            m_skyBoxDefined = true;
        }
        else{
            m_skyBoxDefined = false;
        }

        if (m_skyBoxDefined){

            m_skyBoxRight = skybox_path / worldSkyBox["right"].as<std::string>();
            m_skyBoxLeft = skybox_path / worldSkyBox["left"].as<std::string>();
            m_skyBoxTop = skybox_path / worldSkyBox["top"].as<std::string>();
            m_skyBoxBottom = skybox_path / worldSkyBox["bottom"].as<std::string>();
            m_skyBoxFront = skybox_path / worldSkyBox["front"].as<std::string>();
            m_skyBoxBack = skybox_path / worldSkyBox["back"].as<std::string>();

            if (worldSkyBox["shaders"].IsDefined()){
                boost::filesystem::path shader_path = worldSkyBox["shaders"]["path"].as<std::string>();

                if (shader_path.is_relative()){
                    shader_path = m_world_config_path / shader_path;
                }

                m_skyBox_vsFilePath = shader_path / worldSkyBox["shaders"]["vertex"].as<std::string>();
                m_skyBox_fsFilePath = shader_path / worldSkyBox["shaders"]["fragment"].as<std::string>();

                m_skyBox_shaderProgramDefined = true;
            }
            else{
                m_skyBox_shaderProgramDefined = false;
            }
        }
    }

    if (worldLightsData.IsDefined()){
        size_t n_lights = worldLightsData.size();
        for (size_t idx = 0 ; idx < n_lights; idx++){
            std::string light_name = worldLightsData[idx].as<std::string>();
            afLightPtr lightPtr = new afLight(this);
            YAML::Node lightNode = worldNode[light_name];
            if (lightPtr->loadLight(&lightNode, light_name, this)){
                addAFLight(lightPtr, light_name);
                lightPtr->afCreateCommInstance(afCommType::LIGHT,
                                               lightPtr->m_name,
                                               resolveGlobalNamespace(lightPtr->getNamespace()),
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
            lightPtr->afCreateCommInstance(afCommType::LIGHT,
                                           lightPtr->m_name,
                                           resolveGlobalNamespace(lightPtr->getNamespace()),
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
                    cameraPtr->afCreateCommInstance(afCommType::CAMERA,
                                                    cameraPtr->m_name,
                                                    resolveGlobalNamespace(cameraPtr->getNamespace()),
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
                cameraPtr->afCreateCommInstance(afCommType::CAMERA,
                                                cameraPtr->m_name,
                                                resolveGlobalNamespace(cameraPtr->getNamespace()),
                                                cameraPtr->getMinPublishFrequency(),
                                                cameraPtr->getMaxPublishFrequency());
            }

        }

        if (worldShaders.IsDefined()){
            boost::filesystem::path shader_path = worldShaders["path"].as<std::string>();

            if (shader_path.is_relative()){
                shader_path = m_world_config_path / shader_path;
            }

            m_vsFilePath = shader_path / worldShaders["vertex"].as<std::string>();
            m_fsFilePath = shader_path / worldShaders["fragment"].as<std::string>();

            m_shaderProgramDefined = true;
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
    updateShadowMaps(false, options.m_mirroredDisplay);

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

            addChild(m_skyBoxMesh);

            if (m_skyBox_shaderProgramDefined){
                std::ifstream vsFile;
                std::ifstream fsFile;
                vsFile.open(m_skyBox_vsFilePath.c_str());
                fsFile.open(m_skyBox_fsFilePath.c_str());
                // create a string stream
                std::stringstream vsBuffer, fsBuffer;
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

                    std::cerr << "USING SKYBOX SHADER FILES: " <<
                                 "\n \t VERTEX: " << m_skyBox_vsFilePath.c_str() <<
                                 "\n \t FRAGMENT: " << m_skyBox_fsFilePath.c_str() << std::endl;
                    m_skyBoxMesh->setShaderProgram(shaderProgram);

                }
                else{
                    std::cerr << "ERROR! FOR SKYBOX FAILED TO LOAD SHADER FILES: " <<
                                 "\n \t VERTEX: " << m_skyBox_vsFilePath.c_str() <<
                                 "\n \t FRAGMENT: " << m_skyBox_fsFilePath.c_str() << std::endl;

                    m_skyBox_shaderProgramDefined = false;
                    removeChild(m_skyBoxMesh);
                    delete m_skyBoxMesh;
                }
            }
        }
        else{
            std::cerr << "CAN'T LOAD SKY BOX IMAGES, IGNORING\n";
        }
    }
}


///
/// \brief afWorld::enableShaderProgram
///
void afWorld::enableShaderProgram(){
    if (m_shaderProgramDefined){
        afRigidBodyVec rbVec = getAFRigidBodies();
        for (int i = 0 ; i < rbVec.size() ; i++){
            rbVec[i]->m_vsFilePath = m_vsFilePath;
            rbVec[i]->m_fsFilePath = m_fsFilePath;
            rbVec[i]->m_shaderProgramDefined = true;
        }
    }
}


///
/// \brief afWorld::loadAllADFs
/// \param enable_comm
///
void afWorld::loadAllADFs(bool enable_comm){
    for (int i = 0 ; i < getNumMBConfigs(); i++){
        loadADF(i, enable_comm);
    }
}


///
/// \brief afWorld::loadADF
/// \param i
/// \param enable_comm
/// \return
///
bool afWorld::loadADF(int i, bool enable_comm){
    if (i >= getNumMBConfigs()){
        std::cerr << "ERROR, REQUESTED MULTI-BODY IDX " << i << " HOWEVER, " <<
                     getNumMBConfigs() - 1 << " INDEXED MULTI-BODIES DEFINED" << std::endl;
        return 0;
    }

    std::string adf_filepath = getMultiBodyConfig(i);
    loadADF(adf_filepath, enable_comm);
    return true;
}


///
/// \brief afWorld::loadADF
/// \param a_multibody_config_file
/// \return
///
bool afWorld::loadADF(std::string a_adf_filepath, bool enable_comm){
    afMultiBodyPtr mB(new afMultiBody(this));
    bool success = mB->loadMultiBody(a_adf_filepath, enable_comm);
    if (success){
        boost::filesystem::path p(a_adf_filepath);
        addAFMultiBody(mB, p.stem().string() + afUtils::getNonCollidingIdx(p.stem().string(), &m_afMultiBodyMap));
        buildCollisionGroups();
    }
    return success;
}


///
/// \brief afWorld::addLight
/// \param a_name
/// \param a_light
/// \return
///
bool afWorld::addAFLight(afLightPtr a_light, std::string a_name){
    return addObject<afLightPtr, afLightMap>(a_light, a_name, &m_afLightMap);
}

///
/// \brief afWorld::addCamera
/// \param a_name
/// \param a_cam
/// \return
///
bool afWorld::addAFCamera(afCameraPtr a_cam, std::string a_name){
    return addObject<afCameraPtr, afCameraMap>(a_cam, a_name, &m_afCameraMap);
}

///
/// \brief afWorld::addRigidBody
/// \param a_name
/// \param a_rb
/// \return
///
bool afWorld::addAFRigidBody(afRigidBodyPtr a_rb, std::string a_name){
    return addObject<afRigidBodyPtr, afRigidBodyMap>(a_rb, a_name, &m_afRigidBodyMap);
}

///
/// \brief afWorld::addSoftBody
/// \param a_name
/// \param a_sb
/// \return
///
bool afWorld::addAFSoftBody(afSoftBodyPtr a_sb, std::string a_name){
    return addObject<afSoftBodyPtr, afSoftBodyMap>(a_sb, a_name, &m_afSoftBodyMap);
}

///
/// \brief afWorld::addJoint
/// \param a_name
/// \param a_jnt
/// \return
///
bool afWorld::addAFJoint(afJointPtr a_jnt, std::string a_name){
    return addObject<afJointPtr, afJointMap>(a_jnt, a_name, &m_afJointMap);
}

///
/// \brief afWorld::addAFActuator
/// \param a_sensor
/// \param a_name
/// \return
///
bool afWorld::addAFActuator(afActuatorPtr a_actuator, std::string a_name){
    return addObject<afActuatorPtr, afActuatorMap>(a_actuator, a_name, &m_afActuatorMap);
}

///
/// \brief afWorld::addSensor
/// \param a_sensor
/// \param a_name
/// \return
///
bool afWorld::addAFSensor(afSensorPtr a_sensor, std::string a_name){
    return addObject<afSensorPtr, afSensorMap>(a_sensor, a_name, &m_afSensorMap);
}

///
/// \brief afWorld::addAFMultiBody
/// \param a_multiBody
/// \param a_name
/// \return
///
bool afWorld::addAFMultiBody(afMultiBodyPtr a_multiBody, std::string a_name){
    return addObject<afMultiBodyPtr, afMultiBodyMap>(a_multiBody, a_name, &m_afMultiBodyMap);
}


///
/// \brief afWorld::addAFVehicle
/// \param a_vehicle
/// \param a_name
/// \return
///
bool afWorld::addAFVehicle(afVehiclePtr a_vehicle, std::string a_name){
    return addObject<afVehiclePtr, afVehicleMap>(a_vehicle, a_name, &m_afVehicleMap);
}


///
/// \brief afWorld::buildCollisionGroups
///
void afWorld::buildCollisionGroups(){
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
/// \brief afWorld::getAFLighs
/// \return
///
afLightVec  afWorld::getAFLighs(){
    return getObjects<afLightVec, afLightMap>(&m_afLightMap);
}


///
/// \brief afWorld::getAFCameras
/// \return
///
afCameraVec afWorld::getAFCameras(){
    return getObjects<afCameraVec, afCameraMap>(&m_afCameraMap);
}


///
/// \brief afWorld::getAFRigidBodies
/// \return
///
afRigidBodyVec afWorld::getAFRigidBodies(){
    return getObjects<afRigidBodyVec, afRigidBodyMap>(&m_afRigidBodyMap);
}


///
/// \brief afWorld::getAFSoftBodies
/// \return
///
afSoftBodyVec afWorld::getAFSoftBodies(){
    return getObjects<afSoftBodyVec, afSoftBodyMap>(&m_afSoftBodyMap);
}


///
/// \brief afWorld::getJoints
/// \return
///
afJointVec afWorld::getAFJoints(){
    return getObjects<afJointVec, afJointMap>(&m_afJointMap);
}


///
/// \brief afWorld::getSensors
/// \return
///
afSensorVec afWorld::getAFSensors(){
    return getObjects<afSensorVec, afSensorMap>(&m_afSensorMap);
}


///
/// \brief afWorld::getAFMultiBodies
/// \return
///
afMultiBodyVec afWorld::getAFMultiBodies(){
    return getObjects<afMultiBodyVec, afMultiBodyMap>(&m_afMultiBodyMap);
}


///
/// \brief afWorld::getAFVehicles
/// \return
///
afVehicleVec afWorld::getAFVehicles(){
    return getObjects<afVehicleVec, afVehicleMap>(&m_afVehicleMap);
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
                m_pickedAFRigidBody = getAFRigidBody(body, true);
                if (m_pickedAFRigidBody){
                    std::cerr << "User picked AF rigid body: " << m_pickedAFRigidBody->m_name << std::endl;
                    m_pickedBulletRigidBody = body;
                    m_pickedAFRigidBodyColor = m_pickedAFRigidBody->m_material->copy();
                    m_pickedAFRigidBody->setMaterial(m_pickColor);
                    m_savedState = m_pickedBulletRigidBody->getActivationState();
                    m_pickedBulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
                }

                //other exclusions?
                if (!(body->isStaticObject() || body->isKinematicObject()))
                {
                    //printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
                    btVector3 localPivot = body->getCenterOfMassTransform().inverse() * toBTvec(pickPos);
                    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
                    m_dynamicsWorld->addConstraint(p2p, true);
                    m_pickedConstraint = p2p;
                    btScalar mousePickClamping = 1/body->getInvMass();
                    p2p->m_setting.m_impulseClamp = mousePickClamping;
                    //very weak constraint for picking
                    p2p->m_setting.m_tau = 1/body->getInvMass();
                }
                else{
                    m_pickedOffset = toCvec(body->getCenterOfMassPosition()) - pickPos;
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
                pickCon->setPivotB(toBTvec(newLocation));
                return true;
            }
        }
        else{
            // In this case the rigidBody is a static or kinematic body
            btTransform curTrans = m_pickedBulletRigidBody->getWorldTransform();
            curTrans.setOrigin(toBTvec(newLocation + m_pickedOffset));
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
/// \brief afMultiBody::removePickingConstraint
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
        m_pickedBulletRigidBody = 0;
    }

    if (m_pickedAFRigidBody){
        m_pickedAFRigidBody->setMaterial(m_pickedAFRigidBodyColor);
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
afCamera::afCamera(afWorldPtr a_afWorld): afBaseObject(a_afWorld){

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


///
/// \brief afCamera::setImagePublishInterval
/// \param a_interval
///
void afCamera::setImagePublishInterval(uint a_interval){
    m_imagePublishInterval = cMax(a_interval, (uint)1 );
}


///
/// \brief afCamera::setDepthPublishInterval
/// \param a_interval
///
void afCamera::setDepthPublishInterval(uint a_interval){
    m_depthPublishInterval = cMax(a_interval, (uint)1 );
}


///
/// \brief afCamera::getGlobalPos
/// \return
///
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

    m_camera = new cCamera(m_afWorld);
    addChild(m_camera);

    m_namespace = m_afWorld->getNamespace();

    // Set a default name
    m_name = "default_camera";

    // position and orient the camera
    setView(cVector3d(4.0, 0.0, 2.0),  // camera position (eye)
            cVector3d(0.0, 0.0,-0.5),       // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));      // direction of the "up" vector

    m_initialPos = getLocalPos();
    m_initialRot = getLocalRot();

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
    m_afWorld->addChild(this);

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
    YAML::Node cameraPublishImageInterval = cameraNode["publish image interval"];
    YAML::Node cameraPublishDepth = cameraNode["publish depth"];
    YAML::Node cameraPublishDepthInterval = cameraNode["publish depth interval"];
    YAML::Node cameraMultiPass = cameraNode["multipass"];

    bool _is_valid = true;
    double _clipping_plane_limits[2], _field_view_angle;
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
        m_namespace = afUtils::removeAdjacentBackSlashes(cameraNamespace.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(m_afWorld->getNamespace(), m_namespace);

    if (cameraLocationData.IsDefined()){
        m_camPos = toXYZ<cVector3d>(&cameraLocationData);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOCATION NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraLookAtData.IsDefined()){
        m_camLookAt = toXYZ<cVector3d>(&cameraLookAtData);
    }
    else{
        std::cerr << "INFO: CAMERA \"" << a_camera_name << "\" CAMERA LOOK AT NOT DEFINED, IGNORING " << std::endl;
        _is_valid = false;
    }
    if (cameraUpData.IsDefined()){
        m_camUp = toXYZ<cVector3d>(&cameraUpData);
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
         m_orthographic = true;
        _orthoViewWidth = cameraOrthoWidthData.as<double>();
    }
    else{
         m_orthographic = false;
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

    if (cameraPublishImageInterval.IsDefined()){
        m_imagePublishInterval = cMax(cameraPublishImageInterval.as<uint>(), (uint)1);
    }

    if (cameraPublishDepth.IsDefined()){
        m_publishDepth = cameraPublishDepth.as<bool>();
    }

    if (cameraPublishDepthInterval.IsDefined()){
        m_depthPublishInterval = cMax(cameraPublishDepthInterval.as<uint>(), (uint)1);
    }

    if (cameraMultiPass.IsDefined()){
        _useMultiPassTransparency = cameraMultiPass.as<bool>();
    }

    if(_is_valid){
        m_camera = new cCamera(a_world);
        m_camera->setLocalPos(0, 0, 0);
        cMatrix3d I3;
        I3.identity();
        m_camera->setLocalRot(I3);
        addChild(m_camera);

        if (cameraParent.IsDefined()){
            m_parentName = cameraParent.as<std::string>();
        }
        else{
            m_parentName = "";
            a_world->addChild(this);
        }

        //////////////////////////////////////////////////////////////////////////////////////
        // position and orient the camera
        setView(m_camPos, m_camLookAt, m_camUp);
        m_initialPos = getLocalPos();
        m_initialRot = getLocalRot();
        // set the near and far clipping planes of the camera
        m_camera->setClippingPlanes(_clipping_plane_limits[0], _clipping_plane_limits[1]);

        // set stereo mode
        m_camera->setStereoMode(m_stereMode);

        // set stereo eye separation and focal length (applies only if stereo is enabled)
        m_camera->setStereoEyeSeparation(_stereoEyeSeperation);
        m_camera->setStereoFocalLength(_stereoFocalLength);

        // set vertical mirrored display mode
        m_camera->setMirrorVertical(false);

        if (m_orthographic){
            m_camera->setOrthographicView(_orthoViewWidth);
        }
        else{
            m_camera->setFieldViewAngleRad(_field_view_angle);
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

        if (!m_window)
        {
            std::cerr << "ERROR! FAILED TO CREATE OPENGL WINDOW" << std::endl;
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
            std::cerr << "ERROR! FAILED TO INITIALIZE GLEW LIBRARY" << std::endl;
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

        // Make sure to set the mass to 0 as this is a kinematic body
        m_mass = 0.0;
        // Build the model which inturn adds this body to bullet world
        buildDynamicModel();


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
            m_imagePublisher = s_imageTransport->advertise(m_namespace + m_name + "/ImageData", 1);
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
//                std::ifstream vsFile;
//                std::ifstream fsFile;
//                vsFile.open("/home/adnan/ambf/ambf_shaders/depth/shader.vs");
//                fsFile.open("/home/adnan/ambf/ambf_shaders/depth/shader.fs");
//                // create a string stream
//                std::stringstream vsBuffer, fsBuffer;
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
                    std::cerr << "ERROR! FOR DEPTH_TO_PC2 FAILED TO LOAD SHADER FILES: " << std::endl;
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
            m_depthPointCloudPub = s_rosNode->advertise<sensor_msgs::PointCloud2>(m_namespace + m_name + "/DepthData", 1);
#endif

            }
        }
    }

    return _is_valid;
}


///
/// \brief afCamera::renderFrameBuffer
///
void afCamera::renderFrameBuffer()
{
    m_frameBuffer->renderView();
    m_frameBuffer->copyImageBuffer(m_bufferColorImage);
    m_frameBuffer->copyDepthBuffer(m_bufferDepthImage);
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

    m_camera->setParentWorld(m_afWorld);

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

    int width = m_depthBufferColorImage->getWidth();
    int height = m_depthBufferColorImage->getHeight();
    int bbp = m_depthBufferColorImage->getBytesPerPixel();

    double varScale = pow(2, sizeof(uint) * 8);

    for (int y_span = 0 ; y_span < height ; y_span++){
        for (int x_span = 0 ; x_span < width ; x_span++){

            int idx = (y_span * width + x_span);
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
    // UGLY HACK TO FLIP ONCES BEFORE PUBLISHING AND THEN AGAIN AFTER TO HAVE CORRECT MAPPING
    // WITH THE COLORED DETPH POINT CLOUD
    m_bufferColorImage->flipHorizontal();
    m_imageMatrix = cv::Mat(m_bufferColorImage->getHeight(), m_bufferColorImage->getWidth(), CV_8UC4, m_bufferColorImage->getData());
    cv::cvtColor(m_imageMatrix, m_imageMatrix, cv::COLOR_RGBA2RGB);
    sensor_msgs::ImagePtr rosMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_imageMatrix).toImageMsg();
    m_imagePublisher.publish(rosMsg);
    m_bufferColorImage->flipHorizontal();
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
/// \brief afCamera::resolveParenting
/// \return
///
bool afCamera::resolveParenting(std::string a_parent_name){
    // If the parent name is not empty, override the objects parent name
    if(!a_parent_name.empty()){
        m_parentName = a_parent_name;
    }

    if (!m_parentName.empty()){
        afRigidBodyPtr pBody = m_afWorld->getAFRigidBody(m_parentName);
        if (pBody){
            pBody->addChild(this);
            // Now also update the postion and orientation, w.r.t the parent.
            setView(m_camPos, m_camLookAt, m_camUp);
            m_initialPos = getLocalPos();
            m_initialRot = getLocalRot();
            return true;
        }
        else{
            std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                      << m_parentName << "\"" <<std::endl;
            return false;
        }
    }
    else{
        // No parent assigned, so report success as we have nothing to find
        return true;
    }
}


///
/// \brief afCamera::afObjectCommandExecute
/// \param dt
///
void afCamera::afExecuteCommand(double dt){
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

                std::string parent_name = m_afCameraCommPtr->get_parent_name();

                m_camera->setClippingPlanes(near_plane, far_plane);

                if (m_parentName.compare(parent_name) != 0){
                    // Parent has changed. Find the appropriate parent
                    if (getParent() != nullptr){
                        getParent()->removeChild(this);
                    }

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
void afCamera::updatePositionFromDynamics()
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
        m_afCameraCommPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
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
    std::string wallTimeStr = "Wall Time: " + cStr(m_afWorld->g_wallClock.getCurrentTimeSeconds(), 2) + " s";
    std::string simTimeStr = "Sim Time: " + cStr(m_afWorld->getSimulationTime(), 2) + " s";

    std::string graphicsFreqStr = "Gfx (" + cStr(m_afWorld->m_freqCounterGraphics.getFrequency(), 0) + " Hz)";
    std::string hapticFreqStr = "Phx (" + cStr(m_afWorld->m_freqCounterHaptics.getFrequency(), 0) + " Hz)";

    std::string timeLabelStr = wallTimeStr + " / " + simTimeStr;
    std::string dynHapticFreqLabelStr = graphicsFreqStr + " / " + hapticFreqStr;
    std::string modeLabelStr = "MODE: " + options.m_IIDModeStr;
    std::string btnLabelStr = " : " + options.m_IIDBtnActionStr;

    m_wallSimTimeLabel->setText(timeLabelStr);
    m_graphicsDynamicsFreqLabel->setText(dynHapticFreqLabelStr);
    m_devicesModesLabel->setText(modeLabelStr);
    m_deviceButtonLabel->setText(btnLabelStr);

    std::string controlling_dev_names;
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

int dcntr = 0;
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

    // Only set the _window_closed if the condition is met
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
    m_afWorld = a_afWorld;
}

///
/// \brief afLight::createDefaultLight
/// \return
///
bool afLight::createDefaultLight(){
    std::cerr << "INFO: NO LIGHT SPECIFIED, USING DEFAULT LIGHTING" << std::endl;
    m_spotLight = new cSpotLight(m_afWorld);
    m_namespace = m_afWorld->getNamespace();
    m_name = "default_light";
    addChild(m_spotLight);
    m_spotLight->setLocalPos(cVector3d(0.0, 0.5, 2.5));
    m_spotLight->setDir(0, 0, -1);
    m_spotLight->setSpotExponent(0.3);
    m_spotLight->setCutOffAngleDeg(60);
    m_spotLight->setShadowMapEnabled(true);
    m_spotLight->m_shadowMap->setQualityVeryHigh();
    m_spotLight->setEnabled(true);
    m_afWorld->addChild(m_spotLight);

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
        m_namespace = afUtils::removeAdjacentBackSlashes(lightNamespace.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(m_afWorld->getNamespace(), m_namespace);

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
        m_spotLight = new cSpotLight(a_world);
        m_spotLight->setLocalPos(0, 0, 0);
        cMatrix3d I3;
        I3.identity();
        m_spotLight->setLocalRot(I3);
        addChild(m_spotLight);

        if (lightParent.IsDefined()){
            m_parentName = lightParent.as<std::string>();
        }
        else{
            m_parentName = "";
            a_world->addChild(this);
        }

        setLocalPos(_location);
        setDir(_direction);

        m_initialPos = getLocalPos();
        m_initialRot = getLocalRot();

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
    m_localRot.setCol(v0,v1,v2);
}


///
/// \brief afLight::resolveParenting
/// \return
///
bool afLight::resolveParenting(std::string a_parent_name){
    // If the parent name is not empty, override the objects parent name
    if(!a_parent_name.empty()){
        m_parentName = a_parent_name;
    }

    if (!m_parentName.empty()){
        afRigidBodyPtr pBody = m_afWorld->getAFRigidBody(m_parentName);
        if (pBody){
            pBody->addChild(this);
            // Now also update the postion and orientation, w.r.t the parent.
            setLocalPos(m_initialPos);
            setLocalRot(m_initialRot);
            return true;
        }
        else{
            std::cerr << "WARNING! " << m_name << ": COULDN'T FIND PARENT BODY NAMED\""
                      << m_parentName << "\"" <<std::endl;
            return false;
        }
    }
    else{
        // No parent assigned, so report success as we have nothing to find
        return true;
    }

}



void afLight::afExecuteCommand(double dt){
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
                std::string parent_name = m_afLightCommPtr->get_parent_name();

                m_spotLight->setCutOffAngleDeg(cRadToDeg(cutoff_angle));

                if (m_parentName.compare(parent_name) != 0){
                    // Parent has changed. Find the appropriate parent
                    if (getParent() != nullptr){
                        getParent()->removeChild(this);
                    }

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
void afLight::updatePositionFromDynamics()
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
        m_afLightCommPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
        m_afLightCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

        m_write_count++;

        if (m_write_count % 2000 == 0){
            m_afLightCommPtr->set_parent_name(m_parentName);
            m_write_count = 0;
        }
    }
#endif
}


///
/// \brief afMultiBody::afMultiBody
///
afMultiBody::afMultiBody(){
}

afMultiBody::afMultiBody(afWorldPtr a_afWorld){
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


///
/// \brief afMultiBody::loadMultiBody
/// \param a_multibody_config_file
/// \param enable_comm
/// \return
///
bool afMultiBody::loadMultiBody(std::string a_adf_filepath, bool enable_comm){
    YAML::Node multiBodyNode;
    try{
        multiBodyNode = YAML::LoadFile(a_adf_filepath);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD ADF FILE: " << a_adf_filepath << std::endl;
        return 0;
    }

    // Declare all the yaml parameters that we want to look for
    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodySoftBodies = multiBodyNode["soft bodies"];
    YAML::Node multiBodyVehicles = multiBodyNode["vehicles"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];
    YAML::Node multiBodySensors = multiBodyNode["sensors"];
    YAML::Node multiBodyActuators = multiBodyNode["actuators"];
    YAML::Node multiBodyJointERP = multiBodyNode["joint erp"];
    YAML::Node multiBodyJointCFM = multiBodyNode["joint cfm"];
    YAML::Node multiBodyIgnoreInterCollision = multiBodyNode["ignore inter-collision"];

    boost::filesystem::path mb_cfg_dir = boost::filesystem::path(a_adf_filepath).parent_path();
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
        m_namespace = afUtils::removeAdjacentBackSlashes(multiBodyNameSpace.as<std::string>());
    }
    m_namespace = afUtils::mergeNamespace(m_afWorld->getNamespace(), m_namespace);

    size_t totalRigidBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        rBodyPtr = new afRigidBody(m_afWorld);
        std::string rb_name = multiBodyRidigBodies[i].as<std::string>();
        YAML::Node rb_node = multiBodyNode[rb_name];
        if (rBodyPtr->loadRigidBody(&rb_node, rb_name, this)){
            std::string remap_str = afUtils::getNonCollidingIdx(rBodyPtr->getNamespace() + rb_name, m_afWorld->getAFRigidBodyMap());
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
                    // Only create a comm instance if the body is not passive
                    if (rBodyPtr->isPassive() == false){
                        rBodyPtr->afCreateCommInstance(afCommType::RIGID_BODY,
                                                       rBodyPtr->m_name + remap_str,
                                                       m_afWorld->resolveGlobalNamespace(rBodyPtr->getNamespace()),
                                                       rBodyPtr->getMinPublishFrequency(),
                                                       rBodyPtr->getMaxPublishFrequency());
                    }

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
            std::string remap_str = afUtils::getNonCollidingIdx(sBodyPtr->getNamespace() + sb_name, m_afWorld->getAFSoftBodyMap());
            m_afWorld->addAFSoftBody(sBodyPtr, sBodyPtr->getNamespace() + sb_name + remap_str);
            m_afSoftBodyMapLocal[sBodyPtr->getNamespace() + sb_name] = sBodyPtr;
        }
    }

    /// Loading Sensors
    afSensorPtr sensorPtr = 0;
    size_t totalSensors = multiBodySensors.size();
    for (size_t i = 0; i < totalSensors; ++i) {
        std::string sensor_name = multiBodySensors[i].as<std::string>();
        std::string remap_str = afUtils::getNonCollidingIdx(m_namespace + sensor_name, m_afWorld->getAFSensorMap());
        YAML::Node sensor_node = multiBodyNode[sensor_name];
        // Check which type of sensor is this so we can cast appropriately beforehand
        if (sensor_node["type"].IsDefined()){
            std::string sensor_type = sensor_node["type"].as<std::string>();
            // Check if this is a proximity sensor
            // More sensors to follow
            if (sensor_type.compare("Proximity") == 0 || sensor_type.compare("proximity") == 0 || sensor_type.compare("PROXIMITY") == 0){
                sensorPtr = new afProximitySensor(m_afWorld);
            }
            if (sensor_type.compare("Resistance") == 0 || sensor_type.compare("resistance") == 0 || sensor_type.compare("RESISTANCE") == 0){
                sensorPtr = new afResistanceSensor(m_afWorld);
            }

            // Finally load the sensor from ambf config data
            if (sensorPtr){
                if (sensorPtr->loadSensor(&sensor_node, sensor_name, this, remap_str)){
                    std::cerr << "LOADING SENSOR NUMBER " << i << "\n";
                    m_afWorld->addAFSensor(sensorPtr, m_namespace + sensor_name + remap_str);
//                    if (enable_comm){
                        std::cerr << "LOADING SENSOR COMM \n";
                        sensorPtr->afCreateCommInstance(afCommType::SENSOR,
                                                        sensorPtr->m_name + remap_str,
                                                        m_afWorld->resolveGlobalNamespace(sensorPtr->getNamespace()),
                                                        sensorPtr->getMinPublishFrequency(),
                                                        sensorPtr->getMaxPublishFrequency());
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
                        sensorPtr->m_afSensorCommPtr->set_type(sensor_type);
#endif
//                    }
                }
            }
        }
        else{
            continue;
        }
    }

    /// Loading Sensors
    afActuatorPtr actuatorPtr = 0;
    size_t totalActuators = multiBodyActuators.size();
    for (size_t i = 0; i < totalActuators; ++i) {
        std::string actuator_name = multiBodyActuators[i].as<std::string>();
        std::string remap_str = afUtils::getNonCollidingIdx(m_namespace + actuator_name, m_afWorld->getAFActuatorMap());
        YAML::Node actuator_node = multiBodyNode[actuator_name];
        // Check which type of sensor is this so we can cast appropriately beforehand
        if (actuator_node["type"].IsDefined()){
            std::string actuator_type = actuator_node["type"].as<std::string>();
            // Check if this is a constraint sensor
            // More actuators to follow
            if (actuator_type.compare("Constraint") == 0 || actuator_type.compare("constraint") == 0 || actuator_type.compare("CONSTRAINT") == 0){
                actuatorPtr = new afConstraintActuator(m_afWorld);
            }

            // Finally load the sensor from ambf config data
            if (actuatorPtr){
                if (actuatorPtr->loadActuator(&actuator_node, actuator_name, this, remap_str)){
                    std::cerr << "LOADING ACTUATOR NUMBER " << i << "\n";
                    m_afWorld->addAFActuator(actuatorPtr, m_namespace + actuator_name + remap_str);
//                    if (enable_comm){
                        std::cerr << "LOADING ACTUATOR COMM \n";
                        actuatorPtr->afCreateCommInstance(afCommType::ACTUATOR,
                                                        actuatorPtr->m_name + remap_str,
                                                        m_afWorld->resolveGlobalNamespace(actuatorPtr->getNamespace()),
                                                        actuatorPtr->getMinPublishFrequency(),
                                                        actuatorPtr->getMaxPublishFrequency());
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
                        actuatorPtr->m_afActuatorCommPtr->set_type(actuator_type);
#endif
//                    }
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
        std::string remap_str = afUtils::getNonCollidingIdx(m_namespace + jnt_name, m_afWorld->getAFJointMap());
        if (jntPtr->loadJoint(&jnt_node, jnt_name, this, remap_str)){
            m_afWorld->addAFJoint(jntPtr, m_namespace + jnt_name + remap_str);
            m_afJointMapLocal[m_namespace + jnt_name] = jntPtr;
        }
    }


    size_t totalVehicles = multiBodyVehicles.size();
    afVehiclePtr vehiclePtr;
    for (size_t i = 0; i < totalVehicles; ++i) {
        vehiclePtr = new afVehicle(m_afWorld);
        std::string veh_name = multiBodyVehicles[i].as<std::string>();
        YAML::Node veh_node = multiBodyNode[veh_name];
        if (vehiclePtr->loadVehicle(&veh_node, veh_name, this)){
            std::string remap_str = afUtils::getNonCollidingIdx(vehiclePtr->getNamespace() + veh_name, m_afWorld->getAFVehicleMap());
            m_afWorld->addAFVehicle(vehiclePtr, vehiclePtr->getNamespace() + veh_name + remap_str);
            m_afVehicleMapLocal[vehiclePtr->getNamespace() + veh_name] = vehiclePtr;
            if (enable_comm){

                vehiclePtr->afCreateCommInstance(afCommType::VEHICLE,
                                                 vehiclePtr->m_name + remap_str,
                                                 m_afWorld->resolveGlobalNamespace(vehiclePtr->getNamespace()),
                                                 vehiclePtr->getMinPublishFrequency(),
                                                 vehiclePtr->getMaxPublishFrequency());
            }
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
            std::cerr << "WARNING: CAN'T FIND ANY BODY NAMED: " << a_name << " IN LOCAL MAP" << std::endl;

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



template <typename T, typename TMap>
///
/// \brief afWorld::addObject
/// \param a_obj
/// \param a_name
/// \param a_map
/// \return
///
bool afWorld::addObject(T a_obj, std::string a_name, TMap* a_map){
    (*a_map)[a_name] = a_obj;

    // Set the size of the frame.
    cVector3d bounds = a_obj->getBoundaryMax();
    double frame_size;
    if (bounds.length() > 0.001){
        double max_axis = cMax3(bounds.x(), bounds.y(), bounds.z());
        frame_size = max_axis * 1.2;
    }
    else{
        frame_size = 0.5;
    }
    a_obj->setFrameSize(frame_size);
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
T afWorld::getObject(std::string a_name, TMap* a_map, bool suppress_warning){
    if (a_map->find(a_name) != a_map->end()){
        return ((*a_map)[a_name]);
    }
    // We didn't find the object using the full name, try checking if the name is a substring of the fully qualified name
    int matching_obj_count = 0;
    std::vector<std::string> matching_obj_names;
    T objHandle;
    typename TMap::iterator oIt = a_map->begin();
    for (; oIt != a_map->end() ; ++oIt){
        if (oIt->first.find(a_name) != std::string::npos){
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
        std::cerr << "WARNING: MULTIPLE OBJECTS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < matching_obj_names.size() ; i++){
            std::cerr << "\t" << i << ") " << matching_obj_names[i] << std::endl;
        }
        return NULL;
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY OBJECTS NAMED: \"" << a_name << "\" IN GLOBAL MAP \n";

            std::cerr <<"Existing OBJECTS in Map: " << a_map->size() << std::endl;
            typename TMap::iterator oIt = a_map->begin();
            for (; oIt != a_map->end() ; ++oIt){
                std::cerr << oIt->first << std::endl;
            }
        }
        return NULL;
    }
}


template <typename TVec, typename TMap>
///
/// \brief afWorld::getObjects
/// \param a_map
/// \return
///
TVec afWorld::getObjects(TMap* a_map){
    TVec _objects;
    typename TMap::iterator _oIt;

    for (_oIt = a_map->begin() ; _oIt != a_map->end() ; _oIt++){
        _objects.push_back(_oIt->second);
    }

    return _objects;
}


///
/// \brief afWorld::getAFLight
/// \param a_name
/// \param suppress_warning
/// \return
///
afLightPtr afWorld::getAFLight(std::string a_name, bool suppress_warning){
    return getObject<afLightPtr, afLightMap>(a_name, &m_afLightMap, suppress_warning);
}


///
/// \brief afWorld::getAFCamera
/// \param a_name
/// \param suppress_warning
/// \return
///
afCameraPtr afWorld::getAFCamera(std::string a_name, bool suppress_warning){
    return getObject<afCameraPtr, afCameraMap>(a_name, &m_afCameraMap, suppress_warning);
}


///
/// \brief afMultiBody::getRidigBody
/// \param a_name
/// \return
///
afRigidBodyPtr afWorld::getAFRigidBody(std::string a_name, bool suppress_warning){
    return getObject<afRigidBodyPtr, afRigidBodyMap>(a_name, &m_afRigidBodyMap, suppress_warning);
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
        std::cerr << "WARNING: CAN'T FIND ANY BODY BOUND TO BULLET RIGID BODY: \"" << a_body << "\"\n";

        std::cerr <<"Existing Bodies in Map: " << m_afRigidBodyMap.size() << std::endl;
        afRigidBodyMap::iterator rbIt = m_afRigidBodyMap.begin();
        for (; rbIt != m_afRigidBodyMap.end() ; ++rbIt){
            std::cerr << rbIt->first << std::endl;
        }
    }
    return NULL;
}


///
/// \brief afWorld::getAFSoftBody
/// \param a_name
/// \param suppress_warning
/// \return
///
afSoftBodyPtr afWorld::getAFSoftBody(std::string a_name, bool suppress_warning){
    return getObject<afSoftBodyPtr, afSoftBodyMap>(a_name, &m_afSoftBodyMap, suppress_warning);
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
        std::cerr << "WARNING: CAN'T FIND ANY BODY BOUND TO BULLET RIGID BODY: \"" << a_body << "\"\n";

        std::cerr <<"Existing Bodies in Map: " << m_afSoftBodyMap.size() << std::endl;
        afSoftBodyMap::iterator sbIt = m_afSoftBodyMap.begin();
        for (; sbIt != m_afSoftBodyMap.end() ; ++sbIt){
            std::cerr << sbIt->first << std::endl;
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
    return getObject<afMultiBodyPtr, afMultiBodyMap>(a_name, &m_afMultiBodyMap, suppress_warning);
}


///
/// \brief afWorld::getAFVehicle
/// \param a_name
/// \param suppress_warning
/// \return
///
afVehiclePtr afWorld::getAFVehicle(std::string a_name, bool suppress_warning){
    return getObject<afVehiclePtr, afVehicleMap>(a_name, &m_afVehicleMap, suppress_warning);
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

afVehicle::afVehicle(afWorldPtr a_afWorld): afBaseObject(a_afWorld){

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

bool afVehicle::loadVehicle(std::string vehicle_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx){
    YAML::Node vehicleNode;
    try{
        vehicleNode = YAML::LoadFile(vehicle_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO VEHICLE CONFIG: " << vehicle_config_file << std::endl;
        return 0;
    }
    if (vehicleNode.IsNull()) return false;

    YAML::Node baseSensorNode = vehicleNode[node_name];
    return loadVehicle(&baseSensorNode, node_name, mB, name_remapping_idx);
}

bool afVehicle::loadVehicle(YAML::Node *vehicle_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx){
    YAML::Node vehicleNode = *vehicle_node;
    if (vehicleNode.IsNull()){
        std::cerr << "ERROR: VEHICLE'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    bool result = true;
    // Declare all the yaml parameters that we want to look for
    YAML::Node vehicleMeshPathHR = vehicleNode["high resolution path"];
    YAML::Node vehicleName = vehicleNode["name"];
    YAML::Node vehicleNameSpace = vehicleNode["namespace"];
    YAML::Node vehicleChassis = vehicleNode["chassis"];
    YAML::Node vehicleWheels = vehicleNode["wheels"];

    m_name = vehicleName.as<std::string>();
    std::string chassis_name = vehicleChassis.as<std::string>();

    if (vehicleNameSpace.IsDefined()){
        m_namespace = vehicleNameSpace.as<std::string>();
    }
    m_namespace = afUtils::mergeNamespace(mB->getNamespace(), m_namespace);

    m_chassis = m_afWorld->getAFRigidBody(chassis_name);

    if (m_chassis == NULL){
        result = false;
        return result;
    }

    // Get the inertial offset transform, so the wheels are offset properly.
    btTransform T_oInc = m_chassis->getInertialOffsetTransform();
    m_mass = m_chassis->getMass();
    m_inertia = m_chassis->getInertia();

    std::string high_res_path;
    boost::filesystem::path high_res_filepath;

    m_numWheels = vehicleWheels.size();
    m_wheels.resize(m_numWheels);

    for (int i = 0 ; i < m_numWheels ; i++){
        YAML::Node rigidBodyName = vehicleWheels[i]["body"];
        YAML::Node meshName = vehicleWheels[i]["mesh"];
        YAML::Node widthNode = vehicleWheels[i]["width"];
        YAML::Node radiusNode = vehicleWheels[i]["radius"];
        YAML::Node frictionNode = vehicleWheels[i]["friction"];
        YAML::Node suspensionNode = vehicleWheels[i]["suspension"];
        YAML::Node rollInfluenceNode = vehicleWheels[i]["roll influence"];
        YAML::Node downDirNode = vehicleWheels[i]["down direction"];
        YAML::Node axelDirNode = vehicleWheels[i]["axel direction"];
        YAML::Node offsetNode = vehicleWheels[i]["offset"];
        YAML::Node frontNode = vehicleWheels[i]["front"];
        YAML::Node steeringLimitsNode = vehicleWheels[i]["steering limits"];
        YAML::Node maxEnginePowerNode = vehicleWheels[i]["max engine power"];
        YAML::Node maxBrakePowerNode = vehicleWheels[i]["max brake power"];


        if (rigidBodyName.IsDefined()){
            std::string rb_name = rigidBodyName.as<std::string>();
            m_wheels[i].m_wheelBody = m_afWorld->getAFRigidBody(rb_name);
            if (m_wheels[i].m_wheelBody){
                // Since the wheel in the RayCast car in implicit. Disable the dynamic
                // properties of this wheel.
                m_wheels[i].m_wheelBody->setMass(0.0);
                btVector3 inertia(0, 0, 0);
                m_wheels[i].m_wheelBody->m_bulletRigidBody->setMassProps(0.0, inertia);
                // Print some info here to inform the user that we are setting the mass
                // and inertia to zero to make the wheel static.
                m_wheels[i].m_wheelBodyType = afWheel::WheelBodyType::RIGID_BODY;
            }
            else{
                m_wheels[i].m_wheelBodyType = afWheel::WheelBodyType::INVALID;
                std::cerr << "ERROR! UNABLE TO FIND WHEEL IDX " << i << " BODY NAMED \"" << rb_name << "\" FOR VEHICLE \""
                          << m_name << "\", SKIPPING WHEEL!" << std::endl;
                continue;
            }

        }
        else if (meshName.IsDefined()){
            std::string mesh_name = meshName.as<std::string>();
            if (vehicleMeshPathHR.IsDefined()){
                high_res_path = vehicleMeshPathHR.as<std::string>();
                high_res_filepath = vehicleMeshPathHR.as<std::string>() + mesh_name;
                if (high_res_filepath.is_relative()){
                    high_res_path = mB->getMultiBodyPath() + '/' + high_res_path;
                    high_res_filepath = mB->getMultiBodyPath() + '/' + high_res_filepath.c_str();
                }
            }
            else{
                high_res_path = mB->getHighResMeshesPath();
                high_res_filepath = mB->getHighResMeshesPath() + mesh_name;
            }

            m_wheels[i].m_mesh = new cMultiMesh();
            if (m_wheels[i].m_mesh->loadFromFile(high_res_filepath.c_str())){
                m_afWorld->addChild(m_wheels[i].m_mesh);
                m_wheels[i].m_wheelBodyType = afWheel::WheelBodyType::MESH;
            }
            else{
                m_wheels[i].m_wheelBodyType = afWheel::WheelBodyType::INVALID;
                std::cerr << "ERROR! UNABLE TO FIND WHEEL IDX " << i << " MESH NAMED \"" << mesh_name << "\" FOR VEHICLE \""
                          << m_name << "\", SKIPPING WHEEL!" << std::endl;
                continue;
            }


        }
        else{
            m_wheels[i].m_wheelBodyType = afWheel::WheelBodyType::INVALID;
            std::cerr << "ERROR! UNABLE TO FIND \"MESH\" OR \"BODY\" FIELD FOR WHEEL OF VEHICLE \""
                      << m_name << "\", SKIPPING WHEEL!" << std::endl;
            continue;
        }


        if (widthNode.IsDefined()){
            m_wheels[i].m_width = widthNode.as<double>();
        }

        if (radiusNode.IsDefined()){
            m_wheels[i].m_radius = radiusNode.as<double>();
        }

        if (frictionNode.IsDefined()){
            m_wheels[i].m_friction = frictionNode.as<double>();
        }

        if (suspensionNode.IsDefined()){
            m_wheels[i].m_suspensionStiffness = suspensionNode["stiffness"].as<double>();
            m_wheels[i].m_suspensionDamping = suspensionNode["damping"].as<double>();
            m_wheels[i].m_suspensionCompression = suspensionNode["compression"].as<double>();
            m_wheels[i].m_suspensionRestLength = suspensionNode["rest length"].as<double>();
        }

        if (rollInfluenceNode.IsDefined()){
            m_wheels[i].m_rollInfluence = rollInfluenceNode.as<double>();
        }

        if (downDirNode.IsDefined()){
            m_wheels[i].m_downDirection = toXYZ<cVector3d>(&downDirNode);
        }

        if (axelDirNode.IsDefined()){
            m_wheels[i].m_axelDirection = toXYZ<cVector3d>(&axelDirNode);
        }

        if (offsetNode.IsDefined()){
            m_wheels[i].m_offset = toXYZ<cVector3d>(&offsetNode);
        }

        if (frontNode.IsDefined()){
            m_wheels[i].m_isFront = frontNode.as<bool>();
        }

        if (steeringLimitsNode.IsDefined()){
            m_wheels[i].m_high_steering_lim = steeringLimitsNode["high"].as<double>();
            m_wheels[i].m_low_steering_lim = steeringLimitsNode["low"].as<double>();
        }

        if (maxEnginePowerNode.IsDefined()){
            m_wheels[i].m_max_engine_power = maxEnginePowerNode.as<double>();
        }

        if (maxBrakePowerNode.IsDefined()){
            m_wheels[i].m_max_brake_power = maxBrakePowerNode.as<double>();
        }

    }

    m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_afWorld->m_bulletWorld);
    m_vehicle = new btRaycastVehicle(m_tuning, m_chassis->m_bulletRigidBody, m_vehicleRayCaster);

    m_chassis->m_bulletRigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_afWorld->m_bulletWorld->addVehicle(m_vehicle);
    m_afWorld->addChild(this);

    m_vehicle->setCoordinateSystem(1, 2, 0);

    for (int i = 0 ; i < m_numWheels ; i++){
        btVector3 off = toBTvec(m_wheels[i].m_offset);
        btVector3 dir = toBTvec(m_wheels[i].m_downDirection);
        btVector3 axel_dir = toBTvec(m_wheels[i].m_axelDirection);

        off = T_oInc.inverse() * off;
        dir = T_oInc.getBasis().inverse() * dir;

        m_vehicle->addWheel(off, dir, axel_dir, m_wheels[i].m_suspensionRestLength, m_wheels[i].m_radius, m_tuning, m_wheels[i].m_isFront);
    }

    for (int i = 0 ; i < m_numWheels ; i++){
        btWheelInfo& wheelInfo = m_vehicle->getWheelInfo(i);
        wheelInfo.m_suspensionStiffness = m_wheels[i].m_suspensionStiffness;
        wheelInfo.m_wheelsDampingRelaxation = m_wheels[i].m_suspensionDamping;
        wheelInfo.m_wheelsDampingCompression = m_wheels[i].m_suspensionCompression;
        wheelInfo.m_frictionSlip = m_wheels[i].m_friction;
        wheelInfo.m_rollInfluence = m_wheels[i].m_rollInfluence;
    }

    return result;
}


///
/// \brief afVehicle::afExecuteCommand
/// \param dt
///
void afVehicle::afExecuteCommand(double dt){
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    ambf_msgs::VehicleCmd af_cmd = m_afVehicleCommPtr->get_command();

    int maxWheelCount;

    if (af_cmd.brake == true){
        for (int i = 0 ; i < m_numWheels ; i++){
            m_vehicle->applyEngineForce(0.0, i);
            m_vehicle->setBrake(m_wheels[i].m_max_brake_power, i);
        }
    }
    else{
        for (int i = 0 ; i < m_numWheels ; i++){
            m_vehicle->setBrake(0.0, i);
        }

        maxWheelCount = af_cmd.wheel_power.size() <= m_numWheels ? af_cmd.wheel_power.size() : m_numWheels;

        for (int i = 0 ; i < maxWheelCount ; i++){
            double val = af_cmd.wheel_power[i];
            val = cClamp(val, -m_wheels[i].m_max_engine_power, m_wheels[i].m_max_engine_power);
            m_vehicle->applyEngineForce(val, i);
        }

        maxWheelCount = af_cmd.wheel_brake.size() <= m_numWheels ? af_cmd.wheel_brake.size() : m_numWheels;

        for (int i = 0 ; i < maxWheelCount ; i++){
            double val = af_cmd.wheel_brake[i];
            val = cClamp(val, 0.0, m_wheels[i].m_max_brake_power);
            m_vehicle->setBrake(val, i);
        }
    }

    maxWheelCount = af_cmd.wheel_steering.size() <= m_numWheels ? af_cmd.wheel_steering.size() : m_numWheels;

    for (int i = 0 ; i < maxWheelCount ; i++){
        double val = af_cmd.wheel_steering[i];
        val = cClamp(val, m_wheels[i].m_low_steering_lim, m_wheels[i].m_high_steering_lim);
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
void afVehicle::updatePositionFromDynamics(){
    for (int i = 0; i < m_numWheels ; i++){
        m_vehicle->updateWheelTransform(i, true);
        btTransform btTrans = m_vehicle->getWheelInfo(i).m_worldTransform;
        cTransform cTrans = toCtransform(btTrans);
        if (m_wheels[i].m_wheelBodyType == afWheel::WheelBodyType::MESH){
            m_wheels[i].m_mesh->setLocalTransform(cTrans);
        }
        else if (m_wheels[i].m_wheelBodyType == afWheel::WheelBodyType::RIGID_BODY){
//            m_wheels[i].m_wheelBody->m_bulletRigidBody->setWorldTransform(btTrans);
            m_wheels[i].m_wheelBody->m_bulletRigidBody->getMotionState()->setWorldTransform(btTrans);
        }
        else{
            // We have an invalid wheel. Skip.
        }

    }

    // Update the Local Transform
    setLocalTransform(m_chassis->getLocalTransform());

#ifdef C_ENABLE_AMBF_COMM_SUPPORT
    if (m_afVehicleCommPtr.get() != nullptr){

        afUpdateTimes(m_afWorld->getWallTime(), m_afWorld->getSimulationTime());
        m_afVehicleCommPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
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
int afDepthPointCloud::setup(int a_width, int a_height, int a_numFields)
{
    if ( (a_width <= 0) || (a_height <= 0) || (a_numFields <= 0) ){
        // PRINT SOME ERROR MESSAGE
        return -1;
    }

    m_width = a_width;
    m_height = a_height;
    m_numFields = a_numFields;
    m_data = (float*) malloc(m_width * m_height * m_numFields * sizeof(float));

    return 1;
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


}
//------------------------------------------------------------------------------
