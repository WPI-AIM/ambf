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
#include "afInputDevices.h"
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
/// \brief SimulationParams::SimulationParams
///
AsynchronousDataStructure::AsynchronousDataStructure(){
    m_posRef.set(0,0,0);
    m_posRefOrigin.set(0, 0, 0);
    m_rotRef.identity();
    m_rotRefOrigin.identity();
}



///
/// \brief PhysicalDevice::create_cursor
/// \param a_world
/// \return
///
cShapeSphere* PhysicalInputDevice::createCursor(cBulletWorld* a_world){
    m_cursor = new cShapeSphere(0.05);
    m_cursor->setShowEnabled(true);
    m_cursor->setShowFrame(true);
    m_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_cursor->setMaterial(mat);
    a_world->addChild(m_cursor);
    return m_cursor;
}


///
/// \brief PhysicalDevice::~PhysicalDevice
///
PhysicalInputDevice::~PhysicalInputDevice(){
}

///
/// \brief PhysicalDevice::loadPhysicalDevice
/// \param pd_node
/// \param node_name
/// \param mB
/// \param name_remapping_idx
/// \return
///
bool PhysicalInputDevice::loadPhysicalDevice(std::string pd_config_file, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice, InputDevices* a_iD){
    YAML::Node baseNode;
    try{
        baseNode = YAML::LoadFile(pd_config_file);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO PHYSICAL DEVICE CONFIG: " << pd_config_file << std::endl;
        return 0;
    }
    if (baseNode.IsNull()) return false;

    YAML::Node basePDNode = baseNode[node_name];
    return loadPhysicalDevice(&basePDNode, node_name, hDevHandler, simDevice, a_iD);
}

///
/// \brief PhysicalDevice::loadPhysicalDevice
/// \param pd_node
/// \param node_name
/// \param mB
/// \param name_remapping_idx
/// \return
///
bool PhysicalInputDevice::loadPhysicalDevice(YAML::Node *pd_node, std::string node_name, cHapticDeviceHandler* hDevHandler, SimulatedInputDevice* simDevice, InputDevices* a_iD){
    YAML::Node physicaDeviceNode = *pd_node;
    if (physicaDeviceNode.IsNull()){
        std::cerr << "ERROR: PHYSICAL DEVICE'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    YAML::Node pDHardwareName = physicaDeviceNode["hardware name"];
    YAML::Node pDHapticGain = physicaDeviceNode["haptic gain"];
    YAML::Node pDWorkspaceScaling = physicaDeviceNode["workspace scaling"];
    YAML::Node pDSimulatedGripper = physicaDeviceNode["simulated multibody"];
    YAML::Node pDRootLink = physicaDeviceNode["root link"];
    YAML::Node pDLocation = physicaDeviceNode["location"];

    std::string _hardwareName = "";
    K_lh = 0;
    K_ah = 0;
    // Initialize Default Buttons
    act_1_btn = 0;
    act_2_btn = 1;
    mode_next_btn = 2;
    mode_prev_btn = 3;

    m_workspaceScale = 10;
    std::string _simulatedMBConfig = "";
    std::string _rootLinkName = "";
    cVector3d _position(0, 0, 0);
    cMatrix3d _orientation;
    _orientation.identity();

    // For the simulated gripper, the user can specify a MultiBody config to load.
    // We shall load this file as a proxy for Physical Input device in the simulation.
    // We shall get the root link of this multibody (baselink) and set Cartesian Position
    // control on this body.

    // Further, the user can sepcify a root link for the MultiBody config file. If this
    // is defined we shall infact use the specific link which can be different from
    // the bodies base link.

    // A second use case arises, in which the user doesnt want to provide a config file
    // but want to bind the physical input device to an existing multibody in the simulation.
    // In this case, the user should specify the root link only and we shall try to find a
    // body in simulation by that name. Following on, the use shall then be able to control
    // that link in Position control and control all the joints lower in heirarchy.

    bool _simulatedMBDefined = false;
    bool _rootLinkDefined = false;

    if (pDHardwareName.IsDefined()){
        _hardwareName = pDHardwareName.as<std::string>();
    }
    else{
        std::cerr << "ERROR: PHYSICAL DEVICE : \"" << node_name << "\" HARDWARE NAME NOT DEFINED, IGNORING \n";
        return 0;
    }

    if (pDWorkspaceScaling.IsDefined()){
        m_workspaceScale = pDWorkspaceScaling.as<double>();
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" WORKSPACE SCALE NOT DEFINED \n";
    }

    if (pDHapticGain.IsDefined()){
        K_lh = pDHapticGain["linear"].as<double>();
        K_ah = pDHapticGain["angular"].as<double>();

        // clamp the force output gain to the max device stiffness
        double _maxStiffness = m_hInfo.m_maxLinearStiffness / m_workspaceScale;
        K_lh = cMin(K_lh, _maxStiffness);
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" HAPTIC GAINES NOT DEFINED \n";
    }

    if (pDSimulatedGripper.IsDefined()){
        boost::filesystem::path _mb_filename = _simulatedMBConfig;
        _mb_filename = pDSimulatedGripper.as<std::string>();

        if (_mb_filename.is_relative()){
            _mb_filename = a_iD->getBasePath() / _mb_filename;
        }

        _simulatedMBConfig = _mb_filename.c_str();

        _simulatedMBDefined = true;
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" SIMULATED GRIPPER FILENAME NOT DEFINED \n";
    }

    if (pDRootLink.IsDefined()){
        _rootLinkName = pDRootLink.as<std::string>();
        _rootLinkDefined = true;
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" ROOT LINK NAME NOT DEFINED \n";
    }


    if (!_rootLinkDefined && !_simulatedMBDefined){
        std::cerr << "ERROR: PHYSICAL DEVICE : \"" << node_name << "\" REQUIRES EITHER A \"simulated multibody\""
                                                                   "or a \"root link\" TO DISPLAY A PROXY IN SIMULATION \n";
        return 0;
    }

    int nDevs = hDevHandler->getNumDevices();

    for (int dIdx = 0 ; dIdx < nDevs ; dIdx++){
        // First check if this index has already been claimed or not.
        if (!a_iD->checkClaimedDeviceIdx(dIdx)){
            hDevHandler->getDeviceSpecifications(m_hInfo, dIdx);

            if (m_hInfo.m_modelName.compare(_hardwareName) == 0){
                // This is our device. Let's load it up
                hDevHandler->getDevice(m_hDevice, dIdx);
                m_hDevice->open();
                // Now add the device index in a comman place
                // to help devices that are loaded afterwards
                a_iD->addClaimedDeviceIndex(dIdx);
                break;
            }
        }
    }

    if (_simulatedMBDefined){
        if (!simDevice->loadSimulatedGripper(_simulatedMBConfig, m_hInfo.m_modelName, m_hInfo.m_modelName)){
            return 0;
        }

        // If multibody is defined, then the root link has to be searched in the defined multibody
        if (_rootLinkDefined){
            simDevice->m_rootLink = simDevice->getAFRigidBodyLocal(_rootLinkName);
        }
        else{
            simDevice->m_rootLink = simDevice->getRootAFRigidBodyLocal();
        }
    }
    // If only root link is defined, we are going to look for it in the global space
    else if (_rootLinkDefined){
        if (a_iD->getAFWorld()->getAFRigidBody(_rootLinkName, false)){
            simDevice->m_rootLink = a_iD->getAFWorld()->getAFRigidBody(_rootLinkName);
        }
    }

    if (pDLocation.IsDefined()){
        YAML::Node posNode = pDLocation["position"];
        YAML::Node rpyNode = pDLocation["orientation"];
        assignXYZ(&posNode, &_position);
        cVector3d _rpy;
        assignRPY(&rpyNode, &_rpy);
        _orientation.setExtrinsicEulerRotationRad(_rpy.x(), _rpy.y(), _rpy.z(), C_EULER_ORDER_ZYX);
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" LOCATION NOT DEFINED \n";
        // In this case, take the current position of the root link and set it as initial
        // reference pose
        _position = simDevice->m_rootLink->getLocalPos();
        _orientation = simDevice->m_rootLink->getLocalRot();
    }

    simDevice->m_posRef = _position/ m_workspaceScale;
    simDevice->m_posRefOrigin = _position / m_workspaceScale;
    simDevice->m_rotRef = _orientation;
    simDevice->m_rotRefOrigin = _orientation;

    createAfCursor(a_iD->getAFWorld()->s_bulletWorld, "Tracker");
    return 1;
}

///
/// \brief PhysicalDevice::create_af_cursor
/// \param a_world
/// \param a_name
/// \return
///
cBulletSphere* PhysicalInputDevice::createAfCursor(cBulletWorld *a_world, std::string a_name){
    m_af_cursor = new cBulletSphere(a_world, 0.05, a_name);
    m_af_cursor->setShowEnabled(true);
    m_af_cursor->setShowFrame(true);
    m_af_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_af_cursor->setMaterial(mat);
    a_world->addChild(m_af_cursor);
    return m_af_cursor;
}

///
/// \brief PhysicalDevice::measured_pos
/// \return
///
cVector3d PhysicalInputDevice::measuredPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getPosition(m_pos);
    updateCursorPose();
    return m_pos;
}

///
/// \brief PhysicalDevice::measured_pos_last
/// \return
///
cVector3d PhysicalInputDevice::measuredPosPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posPreClutch;
}

///
/// \brief PhysicalDevice::set_pos_preclutch
/// \param a_pos
///
void PhysicalInputDevice::setPosPreclutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posPreClutch = a_pos;
}

///
/// \brief PhysicalDevice::measured_rot
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getRotation(m_rot);
    return m_rot;
}

///
/// \brief PhysicalDevice::measured_rot_last
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRotPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotPreClutch;
}

///
/// \brief PhysicalDevice::set_rot_preclutch
/// \param a_rot
///
void PhysicalInputDevice::setRotPreclutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotPreClutch = a_rot;
}


///
/// \brief PhysicalDevice::measuredPosCamPreclutch
/// \return
///
cVector3d PhysicalInputDevice::measuredPosCamPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posCamPreClutch;
}

///
/// \brief PhysicalDevice::setPosCamPreclutch
/// \param a_pos
///
void PhysicalInputDevice::setPosCamPreclutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posCamPreClutch = a_pos;
}

///
/// \brief PhysicalDevice::measuredRotCamPreclutch
/// \return
///
cMatrix3d PhysicalInputDevice::measuredRotCamPreclutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotCamPreClutch;
}

///
/// \brief PhysicalDevice::setRotCamPreclutch
/// \param a_rot
///
void PhysicalInputDevice::setRotCamPreclutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotCamPreClutch = a_rot;
}

///
/// \brief PhysicalDevice::update_cursor_pose
///
void PhysicalInputDevice::updateCursorPose(){
    if(m_cursor){
        m_cursor->setLocalPos(m_pos * m_workspaceScale);
        m_cursor->setLocalRot(m_rot);
    }
    if(m_af_cursor){
        m_af_cursor->setLocalPos(m_pos * m_workspaceScale);
        m_af_cursor->setLocalRot(m_rot);
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_af_cursor->m_afObjectPtr->set_userdata_desc("haptics frequency");
        m_af_cursor->m_afObjectPtr->set_userdata(m_freq_ctr.getFrequency());
#endif
    }
}

///
/// \brief PhysicalDevice::measured_lin_vel
/// \return
///
cVector3d PhysicalInputDevice::measuredVelLin(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getLinearVelocity(m_vel);
    return m_vel;
}

///
/// \brief PhysicalDevice::mearured_ang_vel
/// \return
///
cVector3d PhysicalInputDevice::mearuredVelAng(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getAngularVelocity(m_avel);
    return m_avel;
}

///
/// \brief PhysicalDevice::measured_gripper_angle
/// \return
///
double PhysicalInputDevice::measuredGripperAngle(){
    std::lock_guard<std::mutex> lock(m_mutex);
    double angle;
    m_hDevice->getGripperAngleRad(angle);
    return angle;
}

///
/// \brief PhysicalDevice::is_button_pressed
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressed(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    return status;
}

///
/// \brief PhysicalDevice::is_button_press_rising_edge
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressRisingEdge(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_rising[button_index] ^ status){
        if (!m_btn_prev_state_rising[button_index]){
            m_btn_prev_state_rising[button_index] = true;
            return true;
        }
        else{
            m_btn_prev_state_rising[button_index] = false;
        }
    }
    return false;
}

///
/// \brief PhysicalDevice::is_button_press_falling_edge
/// \param button_index
/// \return
///
bool PhysicalInputDevice::isButtonPressFallingEdge(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_falling[button_index] ^ status){
        if (m_btn_prev_state_falling[button_index]){
            m_btn_prev_state_falling[button_index] = false;
            return true;
        }
        else{
            m_btn_prev_state_falling[button_index] = true;
        }
    }
    return false;
}

///
/// \brief PhysicalDevice::apply_wrench
/// \param force
/// \param torque
///
void PhysicalInputDevice::applyWrench(cVector3d force, cVector3d torque){
    std::lock_guard<std::mutex> lock(m_mutex);
    force = force * m_dev_force_enabled;
    torque = torque * m_dev_force_enabled;
    m_hDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief SimulatedGripper::SimulatedGripper
/// \param a_chaiWorld
///
SimulatedInputDevice::SimulatedInputDevice(afWorldPtr a_afWorld): afMultiBody (a_afWorld){
    m_gripper_angle = 0.5;
    P_lc_ramp = 0;
    P_ac_ramp = 0;
}

///
/// \brief SimulatedGripper::loadFromAMBF
/// \param a_gripper_name
/// \param a_device_name
/// \return
///
bool SimulatedInputDevice::loadSimulatedGripper(std::string a_config_filename, std::string a_gripper_name, std::string a_device_name){
    bool res = loadMultiBody(a_config_filename);
    m_rootLink = getRootAFRigidBodyLocal();

    m_rigidGrippingConstraints.resize(m_rootLink->getAFSensors().size());
    m_softGrippingConstraints.resize(m_rootLink->getAFSensors().size());
    // Initialize all the constraint to null ptr
    for (int sIdx = 0 ; sIdx < m_rigidGrippingConstraints.size() ; sIdx++){
        m_rigidGrippingConstraints[sIdx] = 0;
        m_softGrippingConstraints[sIdx] = 0;
    }
    return res;
}

///
/// \brief SimulatedGripper::measured_pos
/// \return
///
cVector3d SimulatedInputDevice::measuredPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalPos();
}

///
/// \brief SimulatedGripper::measured_rot
/// \return
///
cMatrix3d SimulatedInputDevice::measuredRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalRot();
}

///
/// \brief SimulatedGripper::update_measured_pose
///
void SimulatedInputDevice::updateMeasuredPose(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pos  = m_rootLink->getLocalPos();
    m_rot = m_rootLink->getLocalRot();
}

///
/// \brief SimulatedGripper::setGripperAngle
/// \param angle
/// \param dt
///
void SimulatedInputDevice::setGripperAngle(double angle, double dt){
    // Since it's not desireable to control the exact angle of multiple joints in the gripper.
    // We override the set angle method for grippers to simplify the angle bound. 0 for closed
    // and 1 for open and everything in between is scaled.
    if (m_rootLink->m_parentBodies.size() == 0){
        double clipped_angle = cClamp(angle, 0.0, 1.0);
        for (size_t jnt = 0 ; jnt < m_rootLink->m_joints.size() ; jnt++){
            double ang = m_rootLink->m_joints[jnt]->getLowerLimit() + clipped_angle * (m_rootLink->m_joints[jnt]->getUpperLimit()  - m_rootLink->m_joints[jnt]->getLowerLimit());
            m_rootLink->m_joints[jnt]->commandPosition(ang);
        }
    }
}

///
/// \brief SimulatedGripper::offset_gripper_angle
/// \param offset
///
void SimulatedInputDevice::offsetGripperAngle(double offset){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gripper_angle += offset;
    m_gripper_angle = cClamp(m_gripper_angle, 0.0, 1.0);
}

///
/// \brief SimulatedGripper::is_wrench_set
/// \return
///
bool SimulatedInputDevice::isWrenchSet(){
    btVector3 f = m_rootLink->m_bulletRigidBody->getTotalForce();
    btVector3 n = m_rootLink->m_bulletRigidBody->getTotalTorque();
    if (f.isZero()) return false;
    else return true;
}

///
/// \brief SimulatedGripper::clear_wrench
///
void SimulatedInputDevice::clearWrench(){
    m_rootLink->m_bulletRigidBody->clearForces();
}


///
/// \brief InputDevices::InputDevices
/// \param a_bullet_world
/// \param a_max_load_devs
///
InputDevices::InputDevices(afWorldPtr a_afWorld){
    m_deviceHandler = NULL;
    m_afWorld = a_afWorld;

    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

///
/// \brief InputDevices::~InputDevices
///
InputDevices::~InputDevices(){
}


void InputDevices::addClaimedDeviceIndex(int a_devIdx){
    if (!checkClaimedDeviceIdx(a_devIdx)){
        m_devicesClaimed.push_back(a_devIdx);
    }
}


bool InputDevices::checkClaimedDeviceIdx(int a_devIdx){
    bool _claimed = false;
    for (int idx = 0 ; idx < m_devicesClaimed.size() ; idx++){
        if (a_devIdx == m_devicesClaimed[idx]){
            _claimed = true;
            break;
        }
    }
    return _claimed;
}


///
/// \brief InputDevices::loadInputDevices
/// \param a_input_devices_config
/// \param a_max_load_devs
/// \return
///
bool InputDevices::loadInputDevices(std::string a_input_devices_config, int a_max_load_devs){
    if (a_input_devices_config.empty()){
        a_input_devices_config = m_afWorld->getInputDevicesConfig();
    }
    YAML::Node inputDevicesNode;
    try{
        inputDevicesNode = YAML::LoadFile(a_input_devices_config);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO CONFIG FILE: " << a_input_devices_config << std::endl;
        return 0;
    }

    YAML::Node inputDevices = inputDevicesNode["input devices"];

    m_basePath = boost::filesystem::path(a_input_devices_config).parent_path();

    if (!inputDevices.IsDefined()){
        return 0;
    }

    if (a_max_load_devs > 0){
        m_deviceHandler.reset(new cHapticDeviceHandler());
        a_max_load_devs = a_max_load_devs < inputDevices.size() ? a_max_load_devs : inputDevices.size();
        //        std::cerr << "Num of devices " << m_num_devices << std::endl;
        for (uint devIdx = 0; devIdx < a_max_load_devs; devIdx++){

            PhysicalInputDevice* pD = new PhysicalInputDevice();
            SimulatedInputDevice* sD = new SimulatedInputDevice(m_afWorld);

            std::string devKey = inputDevices[devIdx].as<std::string>();
            YAML::Node devNode = inputDevicesNode[devKey];

            if (pD->loadPhysicalDevice(&devNode, devKey, m_deviceHandler.get(), sD, this)){
                PhySimDevicePair dgPair;
                dgPair.m_physicalDevice = pD;
                dgPair.m_simulatedDevice = sD;
                dgPair.m_name = pD->m_hInfo.m_modelName;
                m_psDevicePairs.push_back(dgPair);
            }
        }
        m_numDevices = m_psDevicePairs.size();
    }
    else{
        m_numDevices = 0;
    }
    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

///
/// \brief InputDevices::get_device_gripper_pairs
/// \return
///
std::vector<PhySimDevicePair*> InputDevices::getDeviceGripperPairs(std::vector<std::string> a_device_names){
    std::vector<PhySimDevicePair*> req_dg_Pairs;
    std::vector<PhySimDevicePair>::iterator dgIt;
    for(int req_name_Idx = 0 ; req_name_Idx < a_device_names.size() ; req_name_Idx++){
        std::string req_dev_name = a_device_names[req_name_Idx];
        bool _found_req_device = false;
        for(dgIt = m_psDevicePairs.begin(); dgIt != m_psDevicePairs.end() ; ++dgIt){
            if( dgIt->m_name.compare(req_dev_name) == 0 ){
                req_dg_Pairs.push_back(&(*dgIt));
                _found_req_device = true;
                //                cerr << "INFO, DEVICE GRIPPER PAIR FOUND DEVICE \"" << req_dev_name << "\"" << endl;
            }
        }
        if (!_found_req_device){
            std::cerr << "INFO, DEVICE GRIPPER PAIR: \"" << req_dev_name << "\" NOT FOUND" << std::endl;
        }
    }

    return req_dg_Pairs;

}

///
/// \brief InputDevices::get_all_device_gripper_pairs
/// \return
///
std::vector<PhySimDevicePair*> InputDevices::getAllDeviceGripperPairs(){
    std::vector<PhySimDevicePair*> req_dg_Pairs;
    std::vector<PhySimDevicePair>::iterator dgIt;
    for(dgIt = m_psDevicePairs.begin(); dgIt != m_psDevicePairs.end() ; ++dgIt){
        req_dg_Pairs.push_back(&(*dgIt));
    }
    return req_dg_Pairs;
}


///
/// \brief InputDevices::next_mode
///
void InputDevices::nextMode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief InputDevices::prev_mode
///
void InputDevices::prevMode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief InputDevices::close_devices
///
void InputDevices::closeDevices(){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        m_psDevicePairs[devIdx].m_physicalDevice->m_hDevice->close();
    }
}


///
/// \brief InputDevices::increment_K_lh
/// \param a_offset
/// \return
///
double InputDevices::increment_K_lh(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_psDevicePairs[devIdx].m_physicalDevice->K_lh + a_offset <= 0)
        {
            m_psDevicePairs[devIdx].m_physicalDevice->K_lh = 0.0;
        }
        else{
            m_psDevicePairs[devIdx].m_physicalDevice->K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_psDevicePairs[m_numDevices-1].m_physicalDevice->K_lh;
        g_btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief InputDevices::increment_K_ah
/// \param a_offset
/// \return
///
double InputDevices::increment_K_ah(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_psDevicePairs[devIdx].m_physicalDevice->K_ah + a_offset <=0){
            m_psDevicePairs[devIdx].m_physicalDevice->K_ah = 0.0;
        }
        else{
            m_psDevicePairs[devIdx].m_physicalDevice->K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_psDevicePairs[m_numDevices-1].m_physicalDevice->K_ah;
        g_btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief InputDevices::increment_P_lc
/// \param a_offset
/// \return
///
double InputDevices::increment_P_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getP_lin();
        if (_gain + a_offset <=0){
            sG->m_controller.setP_lin(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setP_lin( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "P_lc = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_P_ac
/// \param a_offset
/// \return
///
double InputDevices::increment_P_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getP_ang();
        if (_gain + a_offset <=0){
            sG->m_controller.setP_ang(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setP_ang( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "P_ac = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_D_lc
/// \param a_offset
/// \return
///
double InputDevices::increment_D_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getD_lin();
        if (_gain + a_offset <=0.01){
            // Keep a small value of Angular gain to avoid controller singularity
            sG->m_controller.setD_lin(0.01);
            _temp = 0.01;
        }
        else{
            sG->m_controller.setD_lin( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "D_lc = " + cStr(_temp, 4);
    return _temp;
}


///
/// \brief InputDevices::increment_D_ac
/// \param a_offset
/// \return
///
double InputDevices::increment_D_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_psDevicePairs[devIdx].m_simulatedDevice->m_rootLink;
        double _gain = sG->m_controller.getD_ang();
        if (_gain + a_offset <=0){
            sG->m_controller.setD_ang(0.0);
            _temp = 0.0;
        }
        else{
            sG->m_controller.setD_ang( _gain + a_offset);
            _temp = _gain + a_offset;
        }
    }

    g_btn_action_str = "D_ac = " + cStr(_temp, 4);
    return _temp;
}

}
//------------------------------------------------------------------------------
