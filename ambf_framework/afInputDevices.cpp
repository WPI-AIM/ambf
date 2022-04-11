//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "afInputDevices.h"
#include "afConversions.h"
#include <string.h>
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

bool afPhysicalDevice::createFromAttribs(afInputDeviceAttributes *a_attribs)
{
    afInputDeviceAttributes& attribs = *a_attribs;

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

    m_workspaceScale = attribs.m_workspaceScale;

    int nDevs = m_CCU_Manager->m_deviceHandler->getNumDevices();
    bool devFound = false;

    for (int dIdx = 0 ; dIdx < nDevs ; dIdx++){
        // First check if this index has already been claimed or not.
        if (m_CCU_Manager->checkClaimedDeviceIdx(dIdx) == false){
            m_CCU_Manager->m_deviceHandler->getDeviceSpecifications(m_hInfo, dIdx);

            if (m_hInfo.m_modelName.compare(attribs.m_hardwareName) == 0){
                // This is our device. Let's load it up
                m_CCU_Manager->m_deviceHandler->getDevice(m_hDevice, dIdx);
                m_hDevice->open();
                // Now add the device index in a comman place
                // to help devices that are loaded afterwards
                m_CCU_Manager->addClaimedDeviceIndex(dIdx);
                devFound = true;
                break;
            }
            else{
                std::shared_ptr<cGenericHapticDevice> gHD;
                if (m_CCU_Manager->m_deviceHandler->getDevice(gHD, dIdx)){
                    // Workaround for proper cleanup
                    gHD->open();
                    gHD->close();
                }
            }
        }
    }

    if(!devFound){
        return 0;
    }

    m_controller.createFromAttribs(&attribs.m_controllerAttribs);

    // clamp the force output gain to the max device stiffness
    m_controller.P_lin = cMin(m_controller.P_lin, m_hInfo.m_maxLinearStiffness / m_workspaceScale);

    double _deadBand = attribs.m_deadBand;
    if (_deadBand < 0){
        std::cerr << "WARNING! PHYSICAL DEVICE : \"" << attribs.m_hardwareName << "\" DEAD BAND MUST BE POSITIVE, IGNORING \n";
    }
    else{
        m_deadBand = _deadBand;
    }

    // If not specified, use the value specified in the devices source file
    m_maxForce = m_hInfo.m_maxLinearForce;

    m_maxJerk = attribs.m_maxJerk;

    m_gripper_pinch_btn = 0;

    m_simRotOffset << attribs.m_orientationOffset.getRotation();

    m_simRotOffset.transr(m_simRotOffsetInverse);

    m_buttons = attribs.m_buttons;

    m_showMarker = attribs.m_visible;
    m_markerSize = attribs.m_visibleSize;

    if (m_showMarker)
    {
        cCreateSphere(m_refSphere, m_markerSize);
        m_refSphere->m_material->setRed();
        m_refSphere->setShowFrame(true);
        m_refSphere->setFrameSize(m_markerSize * 5);
        m_CCU_Manager->getAFWorld()->addSceneObjectToWorld(m_refSphere);
    }


    m_CCU_Manager->s_inputDeviceCount++;

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
    m_afCursor = new afRigidBody(a_afWorld, nullptr);
    m_afCursor->m_visualMesh = new cMultiMesh();
    m_afCursor->m_visualMesh->m_meshes->push_back(tempMesh);
    a_afWorld->addSceneObjectToWorld(m_afCursor->m_visualMesh);
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
afSimulatedDevice::afSimulatedDevice(afWorldPtr a_afWorld, afPhysicalDevice* pD): afModel (a_afWorld){
    m_gripper_angle = 0.5;
    P_lc_ramp = 0;
    P_ac_ramp = 0;
    m_phyDev = pD;
}

bool afSimulatedDevice::createFromAttribs(afSimulatedDeviceAttribs *a_attribs)
{
    afSimulatedDeviceAttribs& attribs = *a_attribs;

    if (attribs.m_rootLinkDefined == false && attribs.m_sdeDefined == false){
        std::cerr << "ERROR! PHYSICAL DEVICE BINDING REQUIRES EITHER A \"simulated multibody\" "
                     "or a \"root link\" TO DISPLAY A PROXY IN SIMULATION \n";
        return 0;
    }

    for (int i = 0 ; i < attribs.m_modelAttribs.m_rigidBodyAttribs.size() ; i++){
        attribs.m_modelAttribs.m_rigidBodyAttribs[i].m_communicationAttribs.m_passive = true;
    }

    if (attribs.m_sdeDefined){
        if (afModel::createFromAttribs(&attribs.m_modelAttribs) == false){
            return 0;
        }

        // If multibody is defined, then the root link has to be searched in the defined multibody
        if (attribs.m_rootLinkDefined){
            m_rootLink = getRigidBody(attribs.m_rootLinkName, false);
        }
        else{
            m_rootLink = getRootRigidBody();
        }
    }
    // If only the root link is defined, we are going to look for it in the global space
    else if (attribs.m_rootLinkDefined){
        m_rootLink = m_afWorld->getRigidBody(attribs.m_rootLinkName, false);
    }

    if (m_rootLink != nullptr){
        // Now check if the controller gains have been defined. If so, override the controller gains
        // defined for the rootlink of simulate end effector
        if (attribs.m_overrideController){
            // Should we consider disable the controller for the physical device if a controller has been
            // defined using the Physical device??
            m_rootLink->m_controller.createFromAttribs(&attribs.m_controllerAttribs);
        }

        m_grippingConstraint = new afConstraintActuator(m_afWorld, this);
        afConstraintActuatorAttributes constraintAttribs;
        constraintAttribs.m_communicationAttribs.m_passive = true;
        constraintAttribs.m_identificationAttribs.m_namespace = attribs.m_identificationAttribs.m_namespace;
        constraintAttribs.m_identificationAttribs.m_name = attribs.m_identificationAttribs.m_name + "_constraint_actuator";
        constraintAttribs.m_identifier = attribs.m_identifier + "_constraint_actuator";
        constraintAttribs.m_visible = false;
        constraintAttribs.m_hierarchyAttribs.m_parentName = m_rootLink->getQualifiedIdentifier();

        if (m_grippingConstraint->createFromAttribs(&constraintAttribs) == false){
            // PRINT SOME WARNING OR ERROR
        }

        enableJointControl(attribs.m_enableJointControl);

        std::string modelName = '/' + m_phyDev->m_hInfo.m_modelName;
        std::replace(modelName.begin(), modelName.end(), ' ', '_');

//        std::string _pDevName = "physical_device_" + std::to_string(m_CCU_Manager->s_inputDeviceCount) + _modelName;
//        createAfCursor(a_iD->getAFWorld(),
//                       _pDevName,
//                       simDevice->getNamespace(),
//                       simDevice->m_rootLink->getMinPublishFrequency(),
//                       simDevice->m_rootLink->getMaxPublishFrequency());

        // Only a simulated body is defined for the Simulated Device would be create an afComm Instace.
        // Since an existing root body is bound to the physical device whose afComm should already be
        // running
        if(attribs.m_sdeDefined){
            m_rootLink->setPassive(false);
            m_rootLink->setNamespace(m_rootLink->getNamespace() + "/simulated_device/");
            m_rootLink->loadCommunicationPlugin(m_rootLink, a_attribs);
        }

        // Initialize the default controller to be the force controller
        m_rootLink->m_activeControllerType = afControlType::FORCE;
    }
    else{
        cerr << "ERROR! FAILED TO LOAD ROOT LINK FOR MODEL " << attribs.m_modelAttribs.m_filePath.c_str() << endl;
        return 0;
    }

    cTransform location;
    if (attribs.m_overrideLocation){
        location << attribs.m_kinematicAttribs.m_location;
        m_rootLink->setLocalTransform(location);
        m_rootLink->setInitialTransform(location);
        m_simRotInitial = location.getLocalRot();
    }
    else{
        location = m_rootLink->getLocalTransform();
    }

    setPosRef(location.getLocalPos());
    setPosRefOrigin(location.getLocalPos());
    setRotRef(location.getLocalRot());
    setRotRefOrigin(location.getLocalRot());
    m_simRotInitial = location.getLocalRot();

    return true;
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
/// \brief afSimulatedDevice::getSimRotInitial
/// \return
///
cMatrix3d afSimulatedDevice::getSimRotInitial(){
    return m_simRotInitial;
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
//    m_deviceHandler = new cHapticDeviceHandler();
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

    if (m_deviceHandler){
        delete m_deviceHandler;
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


///
/// \brief afCollateralControlManager::createFromAttribs
/// \param a_attribs
/// \return
///
bool afCollateralControlManager::createFromAttribs(vector<afTeleRoboticUnitAttributes> *a_attribsVec){

    bool load_status = false;

    if (a_attribsVec->size() > 0){
        m_deviceHandler = new cHapticDeviceHandler();
    }

    for (int i = 0; i < a_attribsVec->size(); i++){
        afTeleRoboticUnitAttributes tuAttrib = (*a_attribsVec)[i];
        string devName = tuAttrib.m_iidAttribs.m_hardwareName;
        afPhysicalDevice* pD = new afPhysicalDevice(this);
        afSimulatedDevice* sD = new afSimulatedDevice(m_afWorld, pD);

        if (pD->createFromAttribs(&tuAttrib.m_iidAttribs)){
            if (sD->createFromAttribs(&tuAttrib.m_sdeAttribs)){
                m_afWorld->addModel(sD);
                afCollateralControlUnit ccu;
                ccu.m_physicalDevicePtr = pD;
                ccu.m_simulatedDevicePtr = sD;
                ccu.m_name = devName;
                ccu.pairCameras(m_afWorld, tuAttrib.m_pairedCamerasNames);
                m_collateralControlUnits.push_back(ccu);
                load_status = true;
            }
            else{
                std::cerr << "WARNING! FAILED TO MODEL " << tuAttrib.m_sdeAttribs.m_filePath.c_str() << " FOR DEVICE: \"" << devName << "\"\n";
                load_status = false;
            }
        }
        else
        {
            std::cerr << "WARNING! FAILED TO LOAD DEVICE: \"" << devName << "\"\n";
            load_status = false;
        }

        if (load_status == false){
            delete pD;
            delete sD;
        }
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
        if (m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_lin + a_offset <= 0)
        {
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_lin = 0.0;
        }
        else{
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_lin += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_collateralControlUnits[m_numDevices-1].m_physicalDevicePtr->m_controller.P_lin;
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
        if (m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_ang + a_offset <=0){
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_ang = 0.0;
        }
        else{
            m_collateralControlUnits[devIdx].m_physicalDevicePtr->m_controller.P_ang += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_numDevices > 0){
        a_offset = m_collateralControlUnits[m_numDevices-1].m_physicalDevicePtr->m_controller.P_ang;
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



bool afCollateralControlUnit::pairCameras(afWorldPtr a_afWorld, std::vector<string> a_cameraNames)
{
    for(int i = 0 ; i < a_cameraNames.size() ; i++){
        std::string camName = a_cameraNames[i];
        afCameraPtr camPtr = a_afWorld->getCamera(camName);
        if(camPtr){
            // Create labels for the contextual controlling devices for each Window-Camera Pair
            cFontPtr font = NEW_CFONTCALIBRI20();
            cLabel* devFreqLabel = new cLabel(font);
            devFreqLabel->m_fontColor.setBlack();
            devFreqLabel->setFontScale(0.8);
            devFreqLabel->m_fontColor.setGreenLime();
            m_devFreqLabel = devFreqLabel;
            camPtr->m_devHapticFreqLabels.push_back(devFreqLabel);
            camPtr->getFrontLayer()->addChild(devFreqLabel);

            camPtr->m_controllingDevNames.push_back(m_name);

            m_cameras.push_back(camPtr);
        }
    }

    // If no cameras are specified, maybe pair all the cameras?
    // Can be commented out.
    if(a_cameraNames.size() == 0){
        afCameraVec camVec = a_afWorld->getCameras();
        for(int i = 0 ; i < camVec.size() ; i++){
            afCameraPtr camPtr = camVec[i];
            // Create labels for the contextual controlling devices for each Window-Camera Pair
            cFontPtr font = NEW_CFONTCALIBRI20();
            cLabel* devFreqLabel = new cLabel(font);
            devFreqLabel->m_fontColor.setBlack();
            devFreqLabel->setFontScale(0.8);
            devFreqLabel->m_fontColor.setGreenLime();
            m_devFreqLabel = devFreqLabel;
            camPtr->m_devHapticFreqLabels.push_back(devFreqLabel);
            camPtr->getFrontLayer()->addChild(devFreqLabel);
            camPtr->m_controllingDevNames.push_back(m_name);

            m_cameras.push_back(camPtr);
        }
    }
    return true;
}
}
//------------------------------------------------------------------------------
