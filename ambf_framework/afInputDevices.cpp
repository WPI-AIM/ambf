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

int afCollateralControlManager::s_inputDeviceCount = 0;

///
/// \brief afSharedDataStructure::afSharedDataStructure
///
afSharedDataStructure::afSharedDataStructure(){
    m_posRef.set(0,0,0);
    m_posRefOrigin.set(0, 0, 0);
    m_rotRef.identity();
    m_rotRefOrigin.identity();
}


///
/// \brief afSharedDataStructure::setPosRef
/// \param a_pos
///
void afSharedDataStructure::setPosRef(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posRef = a_pos;
}


///
/// \brief afSharedDataStructure::setRotRef
/// \param a_rot
///
void afSharedDataStructure::setRotRef(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotRef = a_rot;
}


///
/// \brief afSharedDataStructure::setPosRefOrigin
/// \param a_pos
///
void afSharedDataStructure::setPosRefOrigin(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posRefOrigin = a_pos;
}


///
/// \brief afSharedDataStructure::setRotRefOrigin
/// \param a_rot
///
void afSharedDataStructure::setRotRefOrigin(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotRefOrigin = a_rot;
}


///
/// \brief afSharedDataStructure::getPosRef
/// \return
///
cVector3d afSharedDataStructure::getPosRef(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posRef;
}

///
/// \brief afSharedDataStructure::getRotRef
/// \return
///
cMatrix3d afSharedDataStructure::getRotRef(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotRef;
}



///
/// \brief afSharedDataStructure::getPosRef
/// \return
///
cVector3d afSharedDataStructure::getPosRefOrigin(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posRefOrigin;
}

///
/// \brief afSharedDataStructure::getRotRef
/// \return
///
cMatrix3d afSharedDataStructure::getRotRefOrigin(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotRefOrigin;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////   PHYSICAL DEVICE   ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


///
/// \brief afPhysicalDevice::~afPhysicalDevice
///
afPhysicalDevice::~afPhysicalDevice(){
}

///
/// \brief afPhysicalDevice::loadPhysicalDevice
/// \param pd_config_file
/// \param node_name
/// \param hDevHandler
/// \param simDevice
/// \param a_iD
/// \return
///
bool afPhysicalDevice::loadPhysicalDevice(std::string pd_config_file, std::string node_name, cHapticDeviceHandler* hDevHandler, afSimulatedDevice* simDevice, afCollateralControlManager* a_iD){
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
/// \brief afPhysicalDevice::loadPhysicalDevice
/// \param pd_node
/// \param node_name
/// \param hDevHandler
/// \param simDevice
/// \param a_iD
/// \return
///
bool afPhysicalDevice::loadPhysicalDevice(YAML::Node *pd_node, std::string node_name, cHapticDeviceHandler* hDevHandler, afSimulatedDevice* simDevice, afCollateralControlManager* a_iD){
    YAML::Node physicaDeviceNode = *pd_node;
    if (physicaDeviceNode.IsNull()){
        std::cerr << "ERROR: PHYSICAL DEVICE'S "<< node_name << " YAML CONFIG DATA IS NULL\n";
        return 0;
    }

    YAML::Node pDHardwareName = physicaDeviceNode["hardware name"];
    YAML::Node pDHapticGain = physicaDeviceNode["haptic gain"];
    YAML::Node pDControllerGain = physicaDeviceNode["controller gain"];
    YAML::Node pDEnableJointControl = physicaDeviceNode["enable joint control"];
    YAML::Node pDDeadband = physicaDeviceNode["deadband"];
    YAML::Node pDMaxForce = physicaDeviceNode["max force"];
    YAML::Node pDMaxJerk = physicaDeviceNode["max jerk"];
    YAML::Node pDWorkspaceScaling = physicaDeviceNode["workspace scaling"];
    YAML::Node pDSimulatedGripper = physicaDeviceNode["simulated multibody"];
    YAML::Node pDRootLink = physicaDeviceNode["root link"];
    YAML::Node pDLocation = physicaDeviceNode["location"];
    YAML::Node pDOrientationOffset = physicaDeviceNode["orientation offset"];
    YAML::Node pDButtonMapping = physicaDeviceNode["button mapping"];
    YAML::Node pDVisible = physicaDeviceNode["visible"];
    YAML::Node pDVisibleSize = physicaDeviceNode["visible size"];
    YAML::Node pDVisibleColor = physicaDeviceNode["visible color"];
    YAML::Node pDPairCameras = physicaDeviceNode["pair cameras"];

    std::string _hardwareName = "";
    K_lh = 0;
    K_ah = 0;
    // Initialize Default Buttons
    m_buttons.A1 = 0;
    m_buttons.A2 = 1;
    m_buttons.G1 = -1;
    m_buttons.NEXT_MODE = 2;
    m_buttons.PREV_MODE = 3;

    m_workspaceScale = 10;
    std::string _simulatedMBConfig = "";
    std::string _rootLinkName = "";
    cVector3d position(0, 0, 0);
    cMatrix3d rotation;
    rotation.identity();

    // For the simulated gripper, the user can specify a MultiBody config to load.
    // We shall load this file as a proxy for Physical Input device in the simulation.
    // We shall get the root link of this multibody (baselink) and set Cartesian Position
    // control on this body.

    // Further, the user can sepcify a root link for the MultiBody config file. If this
    // is defined we shall infact use the specific link which can be different from
    // the bodies base link.

    // A second use case arises, in which the user doesnt want to provide a config file
    // but wants to bind the physical input device to an existing multibody in the simulation.
    // In this case, the user should specify just the root link and we shall try to find a
    // body in simulation matching that name. Once succesful we shall then be able to control
    // that link/body in Position control mode and control all the joints lower in heirarchy.

    bool _simulatedMBDefined = false;
    bool _rootLinkDefined = false;
    bool _enableJointControl = true; // Be default enable the joint control of simulated dynamic body

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
//        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" ROOT LINK NAME NOT DEFINED \n";
    }


    if (!_rootLinkDefined && !_simulatedMBDefined){
        std::cerr << "ERROR: PHYSICAL DEVICE : \"" << node_name << "\" REQUIRES EITHER A \"simulated multibody\""
                                                                   "or a \"root link\" TO DISPLAY A PROXY IN SIMULATION \n";
        return 0;
    }

    int nDevs = hDevHandler->getNumDevices();
    bool _devFound = false;

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
                _devFound = true;
                break;
            }
            else{
                std::shared_ptr<cGenericHapticDevice> gHD;
                if (hDevHandler->getDevice(gHD, dIdx)){
                    // Workaround for proper cleanup
                    gHD->open();
                    gHD->close();
                }
            }
        }
    }

    if(!_devFound){
        return 0;
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

    if (pDDeadband.IsDefined()){
        double _deadBand = pDDeadband.as<double>();
        if (_deadBand < 0){
            std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" DEAD BAND MUST BE POSITIVE, IGNORING \n";
        }
        else{
            m_deadBand = _deadBand;
        }
    }

    if (pDMaxForce.IsDefined()){
        double _maxForce = pDMaxForce.as<double>();
        if (_maxForce < m_deadBand){
            std::cerr << "WARNING: MAX FORCE : \"" << node_name << "\" MUST BE GREATER THAN MIN FORCE, IGNORING \n";
        }
        else{
            m_maxForce = _maxForce;
        }
    }
    else{
        // If not specified, use the value specified in the devices source file
        m_maxForce = m_hInfo.m_maxLinearForce;
    }

    if (pDMaxJerk.IsDefined()){
        double _maxJerk = pDMaxJerk.as<double>();
        if (_maxJerk < 0){
            std::cerr << "WARNING: MAX JERK : \"" << node_name << "\" MUST BE POSITIVE, IGNORING \n";
        }
        else{
            m_maxJerk = _maxJerk;
        }
    }

    if (_simulatedMBDefined){
        if (!simDevice->loadMultiBody(_simulatedMBConfig, false)){
            m_hDevice->close();
            return 0;
        }
        boost::filesystem::path p(_simulatedMBConfig);
        m_afWorld->addAFMultiBody(simDevice,
                                  p.stem().string() + afUtils::getNonCollidingIdx(p.stem().string(), m_afWorld->getAFMultiBodyMap()));

        // If multibody is defined, then the root link has to be searched in the defined multibody
        if (_rootLinkDefined){
            simDevice->m_rootLink = simDevice->getAFRigidBodyLocal(_rootLinkName);
        }
        else{
            simDevice->m_rootLink = simDevice->getRootAFRigidBodyLocal();
        }
    }
    // If only the root link is defined, we are going to look for it in the global space
    else if (_rootLinkDefined){
        if (a_iD->getAFWorld()->getAFRigidBody(_rootLinkName, false)){
            simDevice->m_rootLink = a_iD->getAFWorld()->getAFRigidBody(_rootLinkName);
        }
    }

    if (simDevice->m_rootLink){
        // Now check if the controller gains have been defined. If so, override the controller gains
        // defined for the rootlink of simulate end effector
        bool linGainsDefined = false;
        bool angGainsDefined = false;
        if (pDControllerGain.IsDefined()){
            // Should we consider disable the controller for the physical device if a controller has been
            // defined using the Physical device??

            // Check if the linear controller is defined
            if (pDControllerGain["linear"].IsDefined()){
                double _P, _D;
                _P = pDControllerGain["linear"]["P"].as<double>();
                _D = pDControllerGain["linear"]["D"].as<double>();
                m_controller.setLinearGains(_P, 0, _D);
                m_controller.m_positionOutputType = afControlType::force;
                linGainsDefined = true;
            }

            // Check if the angular controller is defined
            if(pDControllerGain["angular"].IsDefined()){
                double _P, _D;
                _P = pDControllerGain["angular"]["P"].as<double>();
                _D = pDControllerGain["angular"]["D"].as<double>();
                m_controller.setAngularGains(_P, 0, _D);
                m_controller.m_orientationOutputType = afControlType::force;
                angGainsDefined = true;
            }
        }
        if(!linGainsDefined){
            // If not controller gains defined for this physical device's simulated body,
            // copy over the gains from the Physical device
            m_controller.setLinearGains(simDevice->m_rootLink->m_controller.getP_lin(),
                                        0,
                                        simDevice->m_rootLink->m_controller.getD_lin());

            m_controller.m_positionOutputType = simDevice->m_rootLink->m_controller.m_positionOutputType;
        }
        if (!angGainsDefined){
            // If not controller gains defined for this physical device's simulated body,
            // copy over the gains from the Physical device
            m_controller.setAngularGains(simDevice->m_rootLink->m_controller.getP_ang(),
                                         0,
                                         simDevice->m_rootLink->m_controller.getD_ang());

            m_controller.m_orientationOutputType = simDevice->m_rootLink->m_controller.m_orientationOutputType;
        }

        // Read the flag to enable disable the joint control of SDE from this input device
        // Defaults to enabled
        if (pDEnableJointControl.IsDefined()){
            _enableJointControl = pDEnableJointControl.as<bool>();
        }

        simDevice->m_rigidGrippingConstraints.resize(simDevice->m_rootLink->getAFSensors().size());
        simDevice->m_softGrippingConstraints.resize(simDevice->m_rootLink->getAFSensors().size());
        enableJointControl(_enableJointControl);
        // Initialize all the constraint to null ptr
        for (int sIdx = 0 ; sIdx < simDevice->m_rigidGrippingConstraints.size() ; sIdx++){
            simDevice->m_rigidGrippingConstraints[sIdx] = 0;
            simDevice->m_softGrippingConstraints[sIdx] = 0;
        }

        std::string _modelName = '/' + m_hInfo.m_modelName;
        std::replace(_modelName.begin(), _modelName.end(), ' ', '_');

        a_iD->s_inputDeviceCount++;
        std::string _pDevName = "physical_device_" + std::to_string(a_iD->s_inputDeviceCount) + _modelName;
//        createAfCursor(a_iD->getAFWorld(),
//                       _pDevName,
//                       simDevice->getNamespace(),
//                       simDevice->m_rootLink->getMinPublishFrequency(),
//                       simDevice->m_rootLink->getMaxPublishFrequency());

        // Only a simulated body is defined for the Simulated Device would be create an afComm Instace.
        // Since an existing root body is bound to the physical device whose afComm should already be
        // running
        if(_simulatedMBDefined){
            std::string _sDevName = "simulated_device_" + std::to_string(a_iD->s_inputDeviceCount) + _modelName;
            simDevice->m_rootLink->afCreateCommInstance(afCommType::RIGID_BODY,
                                                        _sDevName,
                                                        m_afWorld->resolveGlobalNamespace(simDevice->getNamespace()),
                                                        simDevice->m_rootLink->getMinPublishFrequency(),
                                                        simDevice->m_rootLink->getMaxPublishFrequency());
        }
    }
    else{
        m_hDevice->close();
        return 0;
    }

    if (pDLocation.IsDefined()){
        YAML::Node posNode = pDLocation["position"];
        YAML::Node rpyNode = pDLocation["orientation"];
        position = toXYZ<cVector3d>(&posNode);
        cVector3d _rpy;
        _rpy = toRPY<cVector3d>(&rpyNode);
        rotation.setExtrinsicEulerRotationRad(_rpy.x(), _rpy.y(), _rpy.z(), C_EULER_ORDER_XYZ);

        simDevice->m_rootLink->setLocalPos(position);
        simDevice->m_rootLink->setLocalRot(rotation);
        simDevice->m_rootLink->setInitialPosition(position);
        simDevice->m_rootLink->setInitialRotation(rotation);
        m_simRotInitial = rotation;
    }
    else{
        std::cerr << "WARNING: PHYSICAL DEVICE : \"" << node_name << "\" LOCATION NOT DEFINED \n";
        // In this case, take the current position of the root link and set it as initial
        // reference pose
        position = simDevice->m_rootLink->getLocalPos();
        rotation = simDevice->m_rootLink->getLocalRot();
        m_simRotInitial = rotation;
    }

    simDevice->setPosRef(position);
    simDevice->setPosRefOrigin(position);
    simDevice->setRotRef(rotation);
    simDevice->setRotRefOrigin(rotation);
    m_gripper_pinch_btn = 0;

    if (pDOrientationOffset.IsDefined()){
            cVector3d rpy_offset;
            rpy_offset = toRPY<cVector3d>(&pDOrientationOffset);
            m_simRotOffset.setExtrinsicEulerRotationRad(rpy_offset.x(), rpy_offset.y(), rpy_offset.z(), C_EULER_ORDER_XYZ);
    }
    else{
        m_simRotOffset.identity();
    }

    m_simRotOffset.transr(m_simRotOffsetInverse);

    if (pDButtonMapping.IsDefined()){
        if (pDButtonMapping["a1"].IsDefined()){
            m_buttons.A1 = pDButtonMapping["a1"].as<int>();
        }
        if (pDButtonMapping["a2"].IsDefined()){
            m_buttons.A2 = pDButtonMapping["a2"].as<int>();
        }
        if (pDButtonMapping["g1"].IsDefined()){
            m_buttons.G1 = pDButtonMapping["g1"].as<int>();
        }
        if (pDButtonMapping["next mode"].IsDefined()){
            m_buttons.NEXT_MODE = pDButtonMapping["next mode"].as<int>();
        }
        if (pDButtonMapping["prev mode"].IsDefined()){
            m_buttons.PREV_MODE = pDButtonMapping["prev mode"].as<int>();
        }
    }

    if(pDPairCameras.IsDefined()){
        for(int i = 0 ; i < pDPairCameras.size() ; i++){
            std::string camName = pDPairCameras[i].as<std::string>();
            afCameraPtr cameraPtr = m_afWorld->getAFCamera(camName);
            m_pairedCameraNames.push_back(camName);
        }
    }

    m_showMarker = false;
    m_markerSize = 0.05;
    if (pDVisible.IsDefined()){
        m_showMarker = pDVisible.as<bool>();
    }

    if (pDVisibleSize.IsDefined()){
        m_markerSize = pDVisibleSize.as<double>();
    }

    if (m_showMarker)
    {
        cCreateSphere(m_refSphere, m_markerSize);
        m_refSphere->m_material->setRed();
        m_refSphere->setShowFrame(true);
        m_refSphere->setFrameSize(m_markerSize * 5);
        m_afWorld->addChild(m_refSphere);
    }

    return true;
}


///
/// \brief afPhysicalDevice::createAfCursor
/// \param a_afWorld
/// \param a_name
/// \param a_namespace
/// \param minPF
/// \param maxPF
///
void afPhysicalDevice::createAfCursor(afWorldPtr a_afWorld, std::string a_name, std::string a_namespace, int minPF, int maxPF){
    cMesh* tempMesh = new cMesh();
    // create object
    cCreateSphere(tempMesh, 0.05, 32, 32);
    // create display list
    tempMesh->setUseDisplayList(true);
    // invalidate display list
    tempMesh->markForUpdate(false);
    tempMesh->setShowEnabled(true);
    tempMesh->setShowFrame(true);
    tempMesh->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    tempMesh->setMaterial(mat);
    m_afCursor = new afRigidBody(a_afWorld);
    m_afCursor->m_meshes->push_back(tempMesh);
    a_afWorld->addChild(m_afCursor);
    m_afCursor->afCreateCommInstance(afCommType::OBJECT,
                                     a_name, m_afWorld->resolveGlobalNamespace(a_namespace),
                                     minPF,
                                     maxPF);
    m_afWorld = a_afWorld;
}


///
/// \brief afPhysicalDevice::measuredPos
/// \return
///
cVector3d afPhysicalDevice::getPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getPosition(m_pos);
    return m_pos;
}


///
/// \brief afPhysicalDevice::getPosClutched
/// \return
///
cVector3d afPhysicalDevice::getPosClutched(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posClutched;
}


///
/// \brief afPhysicalDevice::getPosPreClutch
/// \return
///
cVector3d afPhysicalDevice::getPosPreClutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posPreClutch;
}


///
/// \brief afPhysicalDevice::getPosCamPreClutch
/// \return
///
cVector3d afPhysicalDevice::getPosCamPreClutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posCamPreClutch;
}


///
/// \brief afPhysicalDevice::getRot
/// \return
///
cMatrix3d afPhysicalDevice::getRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getRotation(m_rot);
    return m_rot;
}


///
/// \brief afPhysicalDevice::getRotClutched
/// \return
///
cMatrix3d afPhysicalDevice::getRotClutched(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotClutched;
}


///
/// \brief afPhysicalDevice::getRotPreClutch
/// \return
///
cMatrix3d afPhysicalDevice::getRotPreClutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotPreClutch;
}


///
/// \brief afPhysicalDevice::measuredRotCamPreclutch
/// \return
///
cMatrix3d afPhysicalDevice::getRotCamPreClutch(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotCamPreClutch;
}


///
/// \brief afPhysicalDevice::setPosClutched
/// \param a_pos
///
void afPhysicalDevice::setPosClutched(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posClutched = a_pos;
}


///
/// \brief afPhysicalDevice::setPosPreClutch
/// \param a_pos
///
void afPhysicalDevice::setPosPreClutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posPreClutch = a_pos;
}


///
/// \brief afPhysicalDevice::setPosCamPreClutch
/// \param a_pos
///
void afPhysicalDevice::setPosCamPreClutch(cVector3d a_pos){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_posCamPreClutch = a_pos;
}


///
/// \brief afPhysicalDevice::setRotClutched
/// \param a_rot
///
void afPhysicalDevice::setRotClutched(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotClutched = a_rot;
}


///
/// \brief afPhysicalDevice::setRotPreClutch
/// \param a_rot
///
void afPhysicalDevice::setRotPreClutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotPreClutch = a_rot;
}


///
/// \brief afPhysicalDevice::setRotCamPreclutch
/// \param a_rot
///
void afPhysicalDevice::setRotCamPreClutch(cMatrix3d a_rot){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotCamPreClutch = a_rot;
}


///
/// \brief afPhysicalDevice::getSimRotInitial
/// \return
///
cMatrix3d afPhysicalDevice::getSimRotInitial(){
    return m_simRotInitial;
}


///
/// \brief afPhysicalDevice::getSimRotOffset
/// \return
///
cMatrix3d afPhysicalDevice::getSimRotOffset(){
    return m_simRotOffset;
}


///
/// \brief afPhysicalDevice::getSimRotOffsetInverse
/// \return
///
cMatrix3d afPhysicalDevice::getSimRotOffsetInverse(){
    return m_simRotOffsetInverse;
}

///
/// \brief afPhysicalDevice::updateCursorPose
///
void afPhysicalDevice::updateCursorPose(){
    if(m_afCursor){
        m_afCursor->setLocalPos(m_pos * m_workspaceScale);
        m_afCursor->setLocalRot(m_rot);
#ifdef C_ENABLE_AMBF_COMM_SUPPORT
        m_afCursor->m_afObjectCommPtr->set_userdata_desc("haptics frequency");
        m_afCursor->m_afObjectCommPtr->set_userdata(m_freq_ctr.getFrequency());
#endif
    }
}


///
/// \brief afPhysicalDevice::getLinVel
/// \return
///
cVector3d afPhysicalDevice::getLinVel(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getLinearVelocity(m_vel);
    return m_vel;
}


///
/// \brief afPhysicalDevice::getAngVel
/// \return
///
cVector3d afPhysicalDevice::getAngVel(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hDevice->getAngularVelocity(m_avel);
    return m_avel;
}


///
/// \brief afPhysicalDevice::measuredGripperAngle
/// \return
///
double afPhysicalDevice::getGripperAngle(){
    std::lock_guard<std::mutex> lock(m_mutex);
    double angle;
    m_hDevice->getGripperAngleRad(angle);
    return angle;
}

///
/// \brief afPhysicalDevice::isButtonPressed
/// \param button_index
/// \return
///
bool afPhysicalDevice::isButtonPressed(int button_index){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    return status;
}

///
/// \brief afPhysicalDevice::isButtonPressRisingEdge
/// \param button_index
/// \return
///
bool afPhysicalDevice::isButtonPressRisingEdge(int button_index){
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
/// \brief afPhysicalDevice::isButtonPressFallingEdge
/// \param button_index
/// \return
///
bool afPhysicalDevice::isButtonPressFallingEdge(int button_index){
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
/// \brief afPhysicalDevice::applyWrench
/// \param force
/// \param torque
///
void afPhysicalDevice::applyWrench(cVector3d force, cVector3d torque){
    std::lock_guard<std::mutex> lock(m_mutex);
    force = force * m_dev_force_enabled;
    torque = torque * m_dev_force_enabled;
    m_hDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////   SIMULATED DEVICE   ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief afSimulatedDevice::afSimulatedDevice
/// \param a_afWorld
///
afSimulatedDevice::afSimulatedDevice(afWorldPtr a_afWorld): afMultiBody (a_afWorld){
    m_gripper_angle = 0.5;
    P_lc_ramp = 0;
    P_ac_ramp = 0;
}


///
/// \brief afSimulatedDevice::measuredPos
/// \return
///
cVector3d afSimulatedDevice::getPos(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalPos();
}

///
/// \brief afSimulatedDevice::measuredRot
/// \return
///
cMatrix3d afSimulatedDevice::getRot(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rootLink->getLocalRot();
}

///
/// \brief afSimulatedDevice::updateMeasuredPose
///
void afSimulatedDevice::updatePose(){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pos  = m_rootLink->getLocalPos();
    m_rot = m_rootLink->getLocalRot();
}

///
/// \brief afSimulatedDevice::setGripperAngle
/// \param angle
///
void afSimulatedDevice::setGripperAngle(double angle){
    // Since it's not desireable to control the exact angle of multiple joints in the gripper.
    // We override the set angle method for grippers to simplify the angle bound. 0 for closed
    // and 1 for open and everything in between is scaled.
    double clipped_angle = cClamp(angle, 0.0, 1.0);
    for (size_t jntIdx = 0 ; jntIdx < m_rootLink->m_CJ_PairsAll.size() ; jntIdx++){
        afJointPtr joint = m_rootLink->m_CJ_PairsAll[jntIdx].m_childJoint;
        double ang = joint->getLowerLimit() + clipped_angle * (joint->getUpperLimit() - joint->getLowerLimit());
        joint->commandPosition(ang);
    }
}

///
/// \brief afSimulatedDevice::offsetGripperAngle
/// \param offset
///
void afSimulatedDevice::offsetGripperAngle(double offset){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gripper_angle += offset;
    m_gripper_angle = cClamp(m_gripper_angle, 0.0, 1.0);
}

///
/// \brief afSimulatedDevice::isWrenchSet
/// \return
///
bool afSimulatedDevice::isWrenchSet(){
    btVector3 f = m_rootLink->m_bulletRigidBody->getTotalForce();
    btVector3 n = m_rootLink->m_bulletRigidBody->getTotalTorque();
    if (f.isZero()) return false;
    else return true;
}

///
/// \brief afSimulatedDevice::clearWrench
///
void afSimulatedDevice::clearWrench(){
    m_rootLink->m_bulletRigidBody->clearForces();
}


///
/// \brief afInputDevices::afInputDevices
/// \param a_afWorld
///
afCollateralControlManager::afCollateralControlManager(afWorldPtr a_afWorld){
    m_deviceHandler = nullptr;
    m_afWorld = a_afWorld;

    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

///
/// \brief afInputDevices::~afInputDevices
///
afCollateralControlManager::~afCollateralControlManager(){
    for (int i = 0 ; i < m_collateralControlUnits.size() ; i++){
        if (m_collateralControlUnits[i].m_physicalDevicePtr != nullptr){
            delete m_collateralControlUnits[i].m_physicalDevicePtr;
        }
        if (m_collateralControlUnits[i].m_simulatedDevicePtr != nullptr){
            delete m_collateralControlUnits[i].m_simulatedDevicePtr;
        }
    }
}


void afCollateralControlManager::addClaimedDeviceIndex(int a_devIdx){
    if (!checkClaimedDeviceIdx(a_devIdx)){
        m_devicesClaimed.push_back(a_devIdx);
    }
}

///
/// \brief afInputDevices::checkClaimedDeviceIdx
/// \param a_devIdx
/// \return
///
bool afCollateralControlManager::checkClaimedDeviceIdx(int a_devIdx){
    bool _claimed = false;
    for (int idx = 0 ; idx < m_devicesClaimed.size() ; idx++){
        if (a_devIdx == m_devicesClaimed[idx]){
            _claimed = true;
            break;
        }
    }
    return _claimed;
}

bool afCollateralControlManager::pairCamerasToCCU(afCollateralControlUnit& a_ccuPtr){
    afPhysicalDevice* pD = a_ccuPtr.m_physicalDevicePtr;

    for(int i = 0 ; i < pD->m_pairedCameraNames.size() ; i++){
        std::string camName = pD->m_pairedCameraNames[i];
        afCameraPtr camPtr = m_afWorld->getAFCamera(camName);
        if(camPtr){
            // Create labels for the contextual controlling devices for each Window-Camera Pair
            cFontPtr font = NEW_CFONTCALIBRI20();
            cLabel* devFreqLabel = new cLabel(font);
            devFreqLabel->m_fontColor.setBlack();
            devFreqLabel->setFontScale(0.8);
            devFreqLabel->m_fontColor.setGreenLime();
            a_ccuPtr.m_devFreqLabel = devFreqLabel;
            camPtr->m_devHapticFreqLabels.push_back(devFreqLabel);
            camPtr->getFrontLayer()->addChild(devFreqLabel);

            camPtr->m_controllingDevNames.push_back(
                        a_ccuPtr.m_name);

            a_ccuPtr.m_cameras.push_back(camPtr);
        }
    }

    // If no cameras are specified, maybe pair all the cameras?
    // Can be commented out.
    if(pD->m_pairedCameraNames.size() == 0){
        afCameraVec camVec = m_afWorld->getAFCameras();
        for(int i = 0 ; i < camVec.size() ; i++){
            afCameraPtr camPtr = camVec[i];
            // Create labels for the contextual controlling devices for each Window-Camera Pair
            cFontPtr font = NEW_CFONTCALIBRI20();
            cLabel* devFreqLabel = new cLabel(font);
            devFreqLabel->m_fontColor.setBlack();
            devFreqLabel->setFontScale(0.8);
            devFreqLabel->m_fontColor.setGreenLime();
            a_ccuPtr.m_devFreqLabel = devFreqLabel;
            camPtr->m_devHapticFreqLabels.push_back(devFreqLabel);
            camPtr->getFrontLayer()->addChild(devFreqLabel);

            camPtr->m_controllingDevNames.push_back(
                        a_ccuPtr.m_name);

            a_ccuPtr.m_cameras.push_back(camPtr);
        }
    }
    return true;
}


///
/// \brief afInputDevices::loadInputDevices
/// \param a_input_devices_config
/// \param a_max_load_devs
/// \return
///
bool afCollateralControlManager::loadInputDevices(std::string a_input_devices_config, int a_max_load_devs){
    std::vector<int> devIdxes;
    for (int i = 0 ; i < a_max_load_devs ; i++){
        devIdxes.push_back(i);
    }
    return loadInputDevices(a_input_devices_config, devIdxes);
}


///
/// \brief afInputDevices::loadInputDevices
/// \param a_inputdevice_config
/// \param a_device_indices
/// \return
///
bool afCollateralControlManager::loadInputDevices(std::string a_input_devices_config, std::vector<int> a_device_indices){
    if (a_input_devices_config.empty()){
        a_input_devices_config = m_afWorld->getInputDevicesConfig();
    }
    YAML::Node inputDevicesNode;
    try{
        inputDevicesNode = YAML::LoadFile(a_input_devices_config);
    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE: " << a_input_devices_config << std::endl;
        return 0;
    }

    YAML::Node inputDevices = inputDevicesNode["input devices"];

    m_basePath = boost::filesystem::path(a_input_devices_config).parent_path();

    if (!inputDevices.IsDefined()){
        return 0;
    }

    bool load_status = false;

    int valid_dev_idxs = cMin(a_device_indices.size(), inputDevices.size());
    if (valid_dev_idxs > 0){
        m_deviceHandler.reset(new cHapticDeviceHandler());
        for (int i = 0; i < valid_dev_idxs; i++){
            int devIdx = a_device_indices[i];
            if (devIdx >=0 && devIdx < inputDevices.size()){
                afPhysicalDevice* pD = new afPhysicalDevice(m_afWorld);
                afSimulatedDevice* sD = new afSimulatedDevice(m_afWorld);

                // Load the device specified in the afInputDevice yaml file
                std::string devKey = inputDevices[devIdx].as<std::string>();
                YAML::Node devNode = inputDevicesNode[devKey];

                if (pD->loadPhysicalDevice(&devNode, devKey, m_deviceHandler.get(), sD, this)){
                    afCollateralControlUnit ccu;
                    ccu.m_physicalDevicePtr = pD;
                    ccu.m_simulatedDevicePtr = sD;
                    ccu.m_name = devKey;
                    pairCamerasToCCU(ccu);
                    m_collateralControlUnits.push_back(ccu);
                    load_status = true;
                }
                else
                {
                    std::cerr << "WARNING: FAILED TO LOAD DEVICE: \"" << devKey << "\"\n";
                    load_status = false;
                    delete pD;
                    delete sD;
                }
            }
            else{
                std::cerr << "ERROR: DEVICE INDEX : \"" << devIdx << "\" > \"" << inputDevices.size() << "\" NO. OF DEVICE SPECIFIED IN \"" << a_input_devices_config << "\"\n";
                load_status = false;
            }
        }
    }
    else{
        std::cerr << "ERROR: SIZE OF DEVICE INDEXES : \"" << a_device_indices.size() << "\" > NO. OF DEVICE SPECIFIED IN \"" << a_input_devices_config << "\"\n";
        load_status = false;
    }

    m_numDevices = m_collateralControlUnits.size();
    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
    return load_status;
}


///
/// \brief afInputDevices::getDeviceGripperPairs
/// \param a_device_names
/// \return
///
std::vector<afCollateralControlUnit*> afCollateralControlManager::getCollateralControlUnits(std::vector<std::string> a_device_names){
    std::vector<afCollateralControlUnit*> req_dg_Pairs;
    std::vector<afCollateralControlUnit>::iterator dgIt;
    for(int req_name_Idx = 0 ; req_name_Idx < a_device_names.size() ; req_name_Idx++){
        std::string req_dev_name = a_device_names[req_name_Idx];
        bool _found_req_device = false;
        for(dgIt = m_collateralControlUnits.begin(); dgIt != m_collateralControlUnits.end() ; ++dgIt){
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
/// \brief afInputDevices::getAllDeviceGripperPairs
/// \return
///
std::vector<afCollateralControlUnit*> afCollateralControlManager::getAllCollateralControlUnits(){
    std::vector<afCollateralControlUnit*> req_dg_Pairs;
    std::vector<afCollateralControlUnit>::iterator dgIt;
    for(dgIt = m_collateralControlUnits.begin(); dgIt != m_collateralControlUnits.end() ; ++dgIt){
        req_dg_Pairs.push_back(&(*dgIt));
    }
    return req_dg_Pairs;
}

///
/// \brief afInputDevices::nextMode
///
void afCollateralControlManager::nextMode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    m_btn_action_str = "";
    m_cam_btn_pressed = false;
    m_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief afInputDevices::prevMode
///
void afCollateralControlManager::prevMode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    m_btn_action_str = "";
    m_cam_btn_pressed = false;
    m_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

///
/// \brief afInputDevices::closeDevices
///
void afCollateralControlManager::closeDevices(){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_hDevice->close();
    }
}


///
/// \brief afInputDevices::increment_K_lh
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_K_lh(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_lh + a_offset <= 0)
        {
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_lh = 0.0;
        }
        else{
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_collateralControlUnits[m_numDevices-1].m_physicalDevicePtr->K_lh;
        m_btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief afInputDevices::increment_K_ah
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_K_ah(double a_offset){
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        if (m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_ah + a_offset <=0){
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_ah = 0.0;
        }
        else{
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_collateralControlUnits[m_numDevices-1].m_physicalDevicePtr->K_ah;
        m_btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

///
/// \brief afInputDevices::increment_P_lc
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_P_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_collateralControlUnits[devIdx].m_simulatedDevicePtr->m_rootLink;
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

    m_btn_action_str = "P_lc = " + cStr(_temp, 4);
    return _temp;
}

///
/// \brief afInputDevices::increment_P_ac
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_P_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_collateralControlUnits[devIdx].m_simulatedDevicePtr->m_rootLink;
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

    m_btn_action_str = "P_ac = " + cStr(_temp, 4);
    return _temp;
}

///
/// \brief afInputDevices::increment_D_lc
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_D_lc(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_collateralControlUnits[devIdx].m_simulatedDevicePtr->m_rootLink;
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

    m_btn_action_str = "D_lc = " + cStr(_temp, 4);
    return _temp;
}

///
/// \brief afInputDevices::increment_D_ac
/// \param a_offset
/// \return
///
double afCollateralControlManager::increment_D_ac(double a_offset){
    double _temp = a_offset;
    for (int devIdx = 0 ; devIdx < m_numDevices ; devIdx++){
        afRigidBodyPtr sG = m_collateralControlUnits[devIdx].m_simulatedDevicePtr->m_rootLink;
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

    m_btn_action_str = "D_ac = " + cStr(_temp, 4);
    return _temp;
}

}
//------------------------------------------------------------------------------
