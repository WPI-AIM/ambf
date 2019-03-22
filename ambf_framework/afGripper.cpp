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

    \author:    <http://www.aimlab.wpi.edu>
    \author:    <amunawar@wpi.edu>
    \author:    Adnan Munawar
    \version:   $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "afGripper.h"
#include <string.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#define PI 3.14159
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;
//------------------------------------------------------------------------------

///
/// \brief afBody::set_angle
/// \param angle
/// \param dt
///
void afGripperLink::setAngle(double &angle, double dt){
    // Since it's not desireable to control the exact angle of multiple joints in the gripper.
    // We override the set angle method for grippers to simplify the angle bound. 0 for closed
    // and 1 for open and everything in between is scaled.
    if (m_parentBodies.size() == 0){
        double clipped_angle = cClamp(angle, 0.0, 1.0);
        for (size_t jnt = 0 ; jnt < m_joints.size() ; jnt++){
            double ang = m_joints[jnt]->m_lower_limit + clipped_angle * (m_joints[jnt]->m_higher_limit - m_joints[jnt]->m_lower_limit);
            m_joints[jnt]->commandPosition(ang);
        }
    }
}

///
/// \brief afBody::set_angle
/// \param angles
/// \param dt
///
void afGripperLink::setAngle(std::vector<double> &angles, double dt){
    // Since it's not desireable to control the exact angle of multiple joints in the gripper.
    // We override the set angle method for grippers to simplify the angle bound. 0 for closed
    // and 1 for open and everything in between is scaled.
    if (m_parentBodies.size() == 0){
        double jntCmdSize = m_joints.size() < angles.size() ? m_joints.size() : angles.size();
        for (size_t jnt = 0 ; jnt < jntCmdSize ; jnt++){
            double clipped_angle = cClamp(angles[jnt], 0.0, 1.0);
            m_joints[jnt]->commandPosition(clipped_angle);
        }

    }
}

///
/// \brief afGripper::loadMultiBody
/// \param a_file
/// \param a_gripper_name
/// \param a_suffix_name
/// \return
///
bool afGripper::loadMultiBody(std::string a_gripper_config_file, std::string a_gripper_name, std::string a_suffix_name){
    m_gripper_name = a_gripper_name;
    m_suffix_name = a_suffix_name;

    YAML::Node multiBodyNode;
    try{
        multiBodyNode = YAML::LoadFile(a_gripper_config_file);
    }catch(std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << a_gripper_config_file << std::endl;
        return 0;
    }

    // Declare all the yaml parameters that we want to look for
    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];

    boost::filesystem::path mb_cfg_dir = boost::filesystem::path(a_gripper_config_file).parent_path();

    afGripperLinkPtr tmpBody;
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

    afRigidBodyMap* _rbMap = m_afWorld->getRigidBodyMap();
    size_t totalBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalBodies; ++i) {
        tmpBody = new afGripperLink(m_afWorld);
        std::string body_name = multiBodyRidigBodies[i].as<std::string>();
//        printf("Loading body: %s \n", body_name .c_str());
        if (tmpBody->loadRidigBody(a_gripper_config_file.c_str(), body_name, this)){
            (*_rbMap)[m_multibody_namespace + body_name.c_str()] = tmpBody;
        }
    }

    afJointMap* _jntMap = m_afWorld->getJointMap();
    afJoint *tmpJoint;
    size_t totalJoints = multiBodyJoints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint(m_afWorld);
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
        //        printf("Loading body: %s \n", jnt_name.c_str());
        if (tmpJoint->loadJoint(a_gripper_config_file.c_str(), jnt_name, this)){
            (*_jntMap)[m_multibody_namespace + jnt_name] = tmpJoint;
            // Disable the IPC Control Switch
//            tmpJoint->m_ipc_ctrl_swtch1 = false;
        }
    }

    // Pass the tmpBody, which is any link in the loaded gripper to get the root
    // parent
    m_rootLink = static_cast<afGripperLinkPtr>(m_afWorld->getRootRigidBody(tmpBody));
    if (m_rootLink == NULL){
        std::cerr << "WARNING, NO ROOT PARENT EXISTS \n";
    }
    else{
        std::string sfx = m_suffix_name;
        sfx.erase(remove_if(sfx.begin(), sfx.end(), isspace), sfx.end());
        m_rootLink->afObjectCreate(m_gripper_name + sfx, m_multibody_namespace);
    }

    return true;
}

///
/// \brief afGripper::getRootRigidBody
/// \return
///
afGripperLinkPtr afGripper::getRootRigidBody(){
    if (m_rootLink == NULL){
        std::cerr << "WARNING, NO ROOT PARENT EXISTS \n";
    }
    return m_rootLink;
}

///
/// \brief afGripper::~afGripper
///
afGripper::~afGripper(){
    afRigidBodyMap* _rbMap = m_afWorld->getRigidBodyMap();
    afRigidBodyMap::const_iterator lIt = _rbMap->begin();
    for ( ; lIt != _rbMap->end() ; ++lIt){
        delete lIt->second;
    }
    afJointMap* _jntMap =  m_afWorld->getJointMap();
    afJointMap::const_iterator jIt = _jntMap->begin();
    for (; jIt != _jntMap->end() ; ++jIt){
        delete jIt->second;
    }
    }
}
//------------------------------------------------------------------------------
