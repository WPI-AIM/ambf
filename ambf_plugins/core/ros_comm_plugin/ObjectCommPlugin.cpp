#include "ObjectCommPlugin.h"

void afRigidBodyState::setChildrenNames(afRigidBodyPtr afRBPtr){
    int num_children = afRBPtr->m_CJ_PairsActive.size();
    if (num_children > 0){
        if (m_childrenNames.size() != num_children){
            m_childrenNames.resize(num_children);
        }
        for (size_t i = 0 ; i < num_children ; i++){
            m_childrenNames[i] = afRBPtr->m_CJ_PairsActive[i].m_childBody->getName();
        }
    }
}


void afRigidBodyState::setJointNames(afRigidBodyPtr afRBPtr){
    int num_joints = afRBPtr->m_CJ_PairsActive.size();
    if (num_joints > 0){
        if (m_jointNames.size() != num_joints){
            m_jointNames.resize(num_joints);
        }
        for (size_t i = 0 ; i < num_joints ; i++){
            m_jointNames[i] = afRBPtr->m_CJ_PairsActive[i].m_childJoint->getName();
        }
    }
}

void afRigidBodyState::setJointPositions(afRigidBodyPtr afRBPtr){
    int num_jnts = afRBPtr->m_CJ_PairsActive.size();
    if (num_jnts > 0){
        if(m_jointPositions.size() != num_jnts){
            m_jointPositions.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_jointPositions[i] = afRBPtr->m_CJ_PairsActive[i].m_childJoint->getPosition();
        }
    }
}

void afRigidBodyState::setJointVelocities(afRigidBodyPtr afRBPtr){
    int num_jnts = afRBPtr->m_CJ_PairsActive.size();
    if (num_jnts > 0){
        if(m_jointVelocities.size() != num_jnts){
            m_jointVelocities.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_jointVelocities[i] = afRBPtr->m_CJ_PairsActive[i].m_childJoint->getVelocity();
        }
    }
}


void afRigidBodyState::setJointEfforts(afRigidBodyPtr afRBPtr){
    int num_jnts = afRBPtr->m_CJ_PairsActive.size();
    if (num_jnts > 0){
        if(m_jointEfforts.size() != num_jnts){
            m_jointEfforts.resize(num_jnts);
        }
        for (size_t i = 0 ; i < num_jnts ; i++){
            m_jointEfforts[i] = afRBPtr->m_CJ_PairsActive[i].m_childJoint->getEffort();
        }
    }
}

#ifdef AF_ENABLE_AMBF_COMM_SUPPORT

int afObjectCommunicationPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;

    if (m_objectPtr == nullptr){
        cerr << "ERROR! OBJECT IS NULLPTR, FAILED TO INITIALIZE COMMUNICATION PLUGIN." << endl;
        return 0;
    }

    string objName = m_objectPtr->getName() + m_objectPtr->getGlobalRemapIdx();
    string objNamespace = m_objectPtr->getNamespace();
    string objQualifiedIdentifier = m_objectPtr->getQualifiedIdentifier() + m_objectPtr->getGlobalRemapIdx();
    int minFreq = m_objectPtr->getMinPublishFrequency();
    int maxFreq = m_objectPtr->getMaxPublishFrequency();
    double timeOut = 0.5;

    bool success = false;

    switch (m_objectPtr->getType()) {
    case afType::ACTUATOR:
    {
        m_actuatorCommPtr.reset(new ambf_comm::Actuator(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_actuatorCommPtr->set_identifier(objQualifiedIdentifier);
        afActuatorPtr actPtr = (afActuatorPtr)m_objectPtr;
        switch (actPtr->m_actuatorType) {
        case afActuatorType::CONSTRAINT:
            m_actuatorCommPtr->set_type("CONSTRAINT");
            break;
        default:
            break;
        }
        success = true;

    }
        break;
    case afType::CAMERA:
    {
        m_cameraCommPtr.reset(new ambf_comm::Camera(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_cameraCommPtr->set_identifier(objQualifiedIdentifier);
        success = true;
    }
        break;
    case afType::LIGHT:
    {
        m_lightCommPtr.reset(new ambf_comm::Light(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_lightCommPtr->set_identifier(objQualifiedIdentifier);
        success = true;
    }
        break;
    case afType::OBJECT:
    {
        m_objectCommPtr.reset(new ambf_comm::Object(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_objectCommPtr->set_identifier(objQualifiedIdentifier);
        success = true;
    }
        break;
    case afType::RIGID_BODY:
    {
        m_rigidBodyCommPtr.reset(new ambf_comm::RigidBody(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_rigidBodyCommPtr->set_identifier(objQualifiedIdentifier);
        success = true;
    }
        break;
    case afType::SENSOR:
    {
        m_sensorCommPtr.reset(new ambf_comm::Sensor(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_sensorCommPtr->set_identifier(objQualifiedIdentifier);
        afSensorPtr senPtr = (afSensorPtr) m_objectPtr;
        switch (senPtr->m_sensorType) {
        case afSensorType::RAYTRACER:
            m_sensorCommPtr->set_type("PROXIMITY");
            break;
        case afSensorType::RESISTANCE:
            m_sensorCommPtr->set_type("RESISTANCE");
            break;
        default:
            break;
        }
        success = true;
    }
        break;
    case afType::VEHICLE:
    {
        m_vehicleCommPtr.reset(new ambf_comm::Vehicle(objName, objNamespace, minFreq, maxFreq, timeOut));
        m_vehicleCommPtr->set_identifier(objQualifiedIdentifier);
        success = true;
    }
        break;
    case afType::POINT_CLOUD:
    {
        afPointCloudPtr pcPtr = (afPointCloudPtr)m_objectPtr;
        m_pointCloudCommPtr.reset(new ambf_comm::PointCloudHandler(pcPtr->m_topicName));
        success = true;
    }
        break;
    default:
    {
        cerr << "WARNING! COMMUNICATION TYPE FOR OBJECT NAMED " << objName << " OF TYPE: " << m_objectPtr->getTypeAsStr() << " NOT IMPLEMENTED YET. IGNORING." << endl;
        return -1;
    }
        break;
    }
    return success;
}

void afObjectCommunicationPlugin::graphicsUpdate()
{
    switch (m_objectPtr->getType()) {
    case afType::POINT_CLOUD:{
        afPointCloudPtr pcPtr = (afPointCloudPtr)m_objectPtr;
        pointCloudFetchCommand(pcPtr, 0.001);
        pointCloudUpdateState(pcPtr, 0.001);
    }
        break;
    default:
        break;
    }
}

void afObjectCommunicationPlugin::physicsUpdate(double dt)
{
    switch (m_objectPtr->getType()) {
    case afType::ACTUATOR:{
        afActuatorPtr actPtr = (afActuatorPtr)m_objectPtr;
        actuatorFetchCommand(actPtr, dt);
        actuatorUpdateState(actPtr, dt);
    }
        break;
    case afType::CAMERA:{
        afCameraPtr camPtr = (afCameraPtr)m_objectPtr;
        cameraFetchCommand(camPtr, dt);
        cameraUpdateState(camPtr, dt);
    }
        break;
    case afType::LIGHT:{
        afLightPtr lightPtr = (afLightPtr)m_objectPtr;
        lightFetchCommand(lightPtr, dt);
        lightUpdateState(lightPtr, dt);
    }
        break;
//    case afType::OBJECT:{
//        afBaseObjectPtr objPtr = (afBaseObjectPtr)m_objectPtr;
//        objectUpdateState(objPtr, dt);
//        objectFetchCommand(objPtr, dt);
//    }
        break;
    case afType::RIGID_BODY:
    {
        afRigidBodyPtr rbPtr = (afRigidBodyPtr)m_objectPtr;
        rigidBodyFetchCommand(rbPtr, dt);
        rigidBodyUpdateState(rbPtr, dt);
    }
        break;
    case afType::SENSOR:{
        afSensorPtr senPtr = (afSensorPtr)m_objectPtr;
        sensorFetchCommand(senPtr, dt);
        sensorUpdateState(senPtr, dt);
    }
        break;
    case afType::VEHICLE:{
        afVehiclePtr vehPtr = (afVehiclePtr)m_objectPtr;
        vehicleFetchCommand(vehPtr, dt);
        vehicleUpdateState(vehPtr, dt);
    }
        break;
    default:
        break;
    }

}

bool afObjectCommunicationPlugin::close()
{

    return 1;
}

void afObjectCommunicationPlugin::setTimeStamps(const double a_wall_time, const double a_sim_time, const double a_system_time)
{
    switch (m_objectPtr->getType()) {
    case afType::ACTUATOR:
    {
        m_actuatorCommPtr->set_wall_time(a_wall_time);
        m_actuatorCommPtr->set_sim_time(a_sim_time);
        m_actuatorCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::CAMERA:
    {
        m_cameraCommPtr->set_wall_time(a_wall_time);
        m_cameraCommPtr->set_sim_time(a_sim_time);
        m_cameraCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::LIGHT:
    {
        m_lightCommPtr->set_wall_time(a_wall_time);
        m_lightCommPtr->set_sim_time(a_sim_time);
        m_lightCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::OBJECT:
    {
        m_objectCommPtr->set_wall_time(a_wall_time);
        m_objectCommPtr->set_sim_time(a_sim_time);
        m_objectCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::RIGID_BODY:
    {
        m_rigidBodyCommPtr->set_wall_time(a_wall_time);
        m_rigidBodyCommPtr->set_sim_time(a_sim_time);
        m_rigidBodyCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::SENSOR:
    {
        m_sensorCommPtr->set_wall_time(a_wall_time);
        m_sensorCommPtr->set_sim_time(a_sim_time);
        m_sensorCommPtr->set_time_stamp(a_system_time);
    }
        break;
    case afType::VEHICLE:
    {
        m_vehicleCommPtr->set_wall_time(a_wall_time);
        m_vehicleCommPtr->set_sim_time(a_sim_time);
        m_vehicleCommPtr->set_time_stamp(a_system_time);
    }
        break;
    }
}

void afObjectCommunicationPlugin::actuatorFetchCommand(afActuatorPtr actPtr, double)
{
    switch (actPtr->m_actuatorType) {
    case afActuatorType::CONSTRAINT:
    {
        ambf_msgs::ActuatorCmd cmd = m_actuatorCommPtr->get_command();
        afConstraintActuatorPtr castPtr = (afConstraintActuatorPtr)actPtr;
        if (cmd.actuate){
            if (castPtr->isActuated()){
                // Constraint is active. Ignore request
                return;
            }
            else if (cmd.use_sensor_data){
                std::string sensorName = cmd.sensor_identifier.data;
                afSensorPtr senPtr = actPtr->m_afWorld->getSensor(sensorName);
                if (senPtr){
                    castPtr->actuate(senPtr);
                }
                else{
                    cerr << "ERROR! IN ACTUATOR CALLBACK " << castPtr->getName() <<
                            ", REQUESTED SENSOR NAME " << sensorName << " NOT FOUND. IGNORING!" << endl;
                }
            }
            else{
                string body_name = cmd.body_name.data;
                if (cmd.use_offset){
                    // Offset of constraint (joint) in sensed body (child)
                    btTransform T_jINc;
                    T_jINc.setOrigin(btVector3(cmd.body_offset.position.x,
                                               cmd.body_offset.position.y,
                                               cmd.body_offset.position.z));

                    T_jINc.setRotation(btQuaternion(cmd.body_offset.orientation.x,
                                                    cmd.body_offset.orientation.y,
                                                    cmd.body_offset.orientation.z,
                                                    cmd.body_offset.orientation.w));
                    castPtr->actuate(body_name, T_jINc);
                }
                else{
                    castPtr->actuate(body_name);
                }
            }
        }
        else{
            castPtr->deactuate();
        }
    }

        break;
    default:
        break;
    }
}

void afObjectCommunicationPlugin::actuatorUpdateState(afActuatorPtr actPtr, double)
{
    m_actuatorCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    m_actuatorCommPtr->set_parent_name(actPtr->m_parentName);
    m_actuatorCommPtr->m_writeMtx.unlock();
    m_actuatorCommPtr->enableComm();
    m_write_count++;
}

void afObjectCommunicationPlugin::cameraFetchCommand(afCameraPtr camPtr, double dt)
{
    if (m_cameraCommPtr.get() != nullptr){
        ambf_msgs::CameraCmd m_afCommand = m_cameraCommPtr->get_command();

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
            camPtr->setLocalPos(pos);
            camPtr->setLocalRot(rot_mat);
        }
        m_read_count++;
        if(m_read_count % (camPtr->m_afWorld->m_updateCounterLimit) == 0){
            // We may update the params intermittently
            m_cameraCommPtr->update_params_from_server();
            if (m_cameraCommPtr->m_paramsChanged){
                // Clear the flag so it can be used for testing again
                m_cameraCommPtr->m_paramsChanged = false;

                double near_plane = m_cameraCommPtr->get_near_plane();
                double far_plane = m_cameraCommPtr->get_far_plane();
                double field_view_angle = m_cameraCommPtr->get_field_view_angle();
                double orthographic_view_width = m_cameraCommPtr->get_orthographic_view_width();
                double stereo_eye_separation = m_cameraCommPtr->get_steteo_eye_separation();
                double stereo_focal_length = m_cameraCommPtr->get_steteo_focal_length();

                string parent_name = m_cameraCommPtr->get_parent_name();

                camPtr->getInternalCamera()->setClippingPlanes(near_plane, far_plane);

                camPtr->resolveParent(parent_name);

                switch (m_cameraCommPtr->get_projection_type()) {
                case ambf_comm::ProjectionType::PERSPECTIVE:
                    if (field_view_angle == 0){
                        field_view_angle = 0.7;
                        m_paramsSet = false;
                    }
                    camPtr->getInternalCamera()->setFieldViewAngleRad(field_view_angle);
                    camPtr->setOrthographic(false);
                    break;
                case ambf_comm::ProjectionType::ORTHOGRAPHIC:
                    if (orthographic_view_width == 0){
                        orthographic_view_width = 10.0;
                        m_paramsSet = false;
                    }
                    camPtr->getInternalCamera()->setOrthographicView(orthographic_view_width);
                    camPtr->setOrthographic(true);
                    break;
                default:
                    break;
                }

                switch (m_cameraCommPtr->get_view_mode()) {
                case ambf_comm::ViewMode::MONO:
                    camPtr->getInternalCamera()->setStereoMode(cStereoMode::C_STEREO_DISABLED);
                    break;
                case ambf_comm::ViewMode::STEREO:
                    camPtr->getInternalCamera()->setStereoMode(cStereoMode::C_STEREO_PASSIVE_LEFT_RIGHT);
                    camPtr->getInternalCamera()->setStereoEyeSeparation(stereo_eye_separation);
                    camPtr->getInternalCamera()->setStereoFocalLength(stereo_focal_length);
                    break;
                default:
                    break;
                }
            }

            m_read_count = 0;
        }
    }

}

void afObjectCommunicationPlugin::cameraUpdateState(afCameraPtr camPtr, double dt)
{
    if (m_paramsSet == false){
        cCamera* cCamPtr = camPtr->getInternalCamera();
        m_cameraCommPtr->set_near_plane(cCamPtr->getNearClippingPlane());
        m_cameraCommPtr->set_far_plane(cCamPtr->getFarClippingPlane());
        m_cameraCommPtr->set_field_view_angle(cCamPtr->getFieldViewAngleRad());
        m_cameraCommPtr->set_orthographic_view_width(cCamPtr->getOrthographicViewWidth());
        m_cameraCommPtr->set_steteo_eye_separation(cCamPtr->getStereoEyeSeparation());
        m_cameraCommPtr->set_steteo_focal_length(cCamPtr->getStereoFocalLength());
        m_cameraCommPtr->set_parent_name(camPtr->m_parentName);

        if (camPtr->getInternalCamera()->isViewModePerspective()){
            m_cameraCommPtr->set_projection_type(ambf_comm::ProjectionType::PERSPECTIVE);
        }
        else{
            m_cameraCommPtr->set_projection_type(ambf_comm::ProjectionType::ORTHOGRAPHIC);
        }

        if (camPtr->m_stereoMode == C_STEREO_DISABLED){
            m_cameraCommPtr->set_view_mode(ambf_comm::ViewMode::MONO);
        }
        else{
            m_cameraCommPtr->set_view_mode(ambf_comm::ViewMode::STEREO);;
        }

        m_cameraCommPtr->set_params_on_server();
        m_paramsSet = true;
    }

    m_cameraCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    cVector3d localPos = camPtr->getLocalPos();
    m_cameraCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(camPtr->getLocalRot());
    m_cameraCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    if (m_write_count % camPtr->m_afWorld->m_updateCounterLimit == 0){
        m_cameraCommPtr->set_parent_name(camPtr->m_parentName);
        m_write_count = 0;
    }
    m_cameraCommPtr->m_writeMtx.unlock();
    m_cameraCommPtr->enableComm();
    m_write_count++;
}

void afObjectCommunicationPlugin::jointFetchCommand(afJointPtr jointPtr, double dt)
{

}

void afObjectCommunicationPlugin::jointUpdateState(afJointPtr jointPtr, double dt)
{

}

void afObjectCommunicationPlugin::lightFetchCommand(afLightPtr lightPtr, double dt)
{
    ambf_msgs::LightCmd m_afCommand = m_lightCommPtr->get_command();

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
        lightPtr->setLocalPos(pos);
        lightPtr->setLocalRot(rot_mat);
    }
    m_read_count++;
    if(m_read_count % lightPtr->m_afWorld->m_updateCounterLimit == 0){
        // We may update the params intermittently
        m_lightCommPtr->update_params_from_server();
        if (m_lightCommPtr->m_paramsChanged){
            // Clear the flag so it can be used for testing again
            m_lightCommPtr->m_paramsChanged = false;

            double cutoff_angle = m_lightCommPtr->get_cuttoff_angle();
            string parent_name = m_lightCommPtr->get_parent_name();

            lightPtr->setCutOffAngle(cutoff_angle);

            lightPtr->resolveParent(parent_name);
        }

        m_read_count = 0;
    }
}

void afObjectCommunicationPlugin::lightUpdateState(afLightPtr lightPtr, double dt)
{
    if (m_paramsSet == false){
        m_lightCommPtr->set_cuttoff_angle(lightPtr->getCutOffAngle());
        m_lightCommPtr->set_type(ambf_comm::LightType::SPOT);
        m_lightCommPtr->set_parent_name(lightPtr->m_parentName);

        m_lightCommPtr->set_params_on_server();
        m_paramsSet = true;
    }

    m_lightCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    cVector3d localPos = lightPtr->getLocalPos();
    m_lightCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(lightPtr->getLocalRot());
    m_lightCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    if (m_write_count % lightPtr->m_afWorld->m_updateCounterLimit == 0){
        m_lightCommPtr->set_parent_name(lightPtr->m_parentName);
        m_write_count = 0;
    }
    m_lightCommPtr->m_writeMtx.unlock();
    m_lightCommPtr->enableComm();
    m_write_count++;
}

void afObjectCommunicationPlugin::rigidBodyFetchCommand(afRigidBodyPtr afRBPtr, double dt)
{
    btRigidBody* btRBPtr = afRBPtr->m_bulletRigidBody;
    btVector3 force, torque;
    ambf_msgs::RigidBodyCmd afCommand = m_rigidBodyCommPtr->get_command();

    // IF THE COMMAND IS OF TYPE FORCE
    switch (afCommand.cartesian_cmd_type) {
    case ambf_msgs::RigidBodyCmd::TYPE_FORCE:{
        afRBPtr->m_activeControllerType = afControlType::FORCE;
        if (afRBPtr->m_bulletRigidBody){
            force.setValue(afCommand.wrench.force.x,
                           afCommand.wrench.force.y,
                           afCommand.wrench.force.z);

            torque.setValue(afCommand.wrench.torque.x,
                            afCommand.wrench.torque.y,
                            afCommand.wrench.torque.z);

            afRBPtr->m_bulletRigidBody->applyCentralForce(force);
            afRBPtr->m_bulletRigidBody->applyTorque(torque);
        }
    }
        break;
    case ambf_msgs::RigidBodyCmd::TYPE_POSITION:{
        afRBPtr->m_activeControllerType = afControlType::POSITION;
        // If the body is kinematic, we just want to control the position
        if (btRBPtr->isStaticOrKinematicObject()){
            btTransform Tcommand;
            Tcommand.setOrigin(btVector3(afCommand.pose.position.x,
                                         afCommand.pose.position.y,
                                         afCommand.pose.position.z));

            Tcommand.setRotation(btQuaternion(afCommand.pose.orientation.x,
                                              afCommand.pose.orientation.y,
                                              afCommand.pose.orientation.z,
                                              afCommand.pose.orientation.w));

            //                If the current pose is the same as before, ignore. Otherwise, update pose and collision AABB.
            if ((btRBPtr->getWorldTransform().getOrigin() - Tcommand.getOrigin()).norm() > 0.00001 ||
                    btRBPtr->getWorldTransform().getRotation().angleShortestPath(Tcommand.getRotation()) > 0.0001){
                // Compensate for the inertial offset
                Tcommand = Tcommand * afRBPtr->getInertialOffsetTransform();
                //                    cerr << "Updating Static Object Pose \n";
                btRBPtr->getMotionState()->setWorldTransform(Tcommand);
                btRBPtr->setWorldTransform(Tcommand);
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
            b_trans = afRBPtr->getCOMTransform();

            cur_pos = b_trans.getOrigin();
            cur_rot.setRotation(b_trans.getRotation());
            cmd_pos.setValue(afCommand.pose.position.x,
                             afCommand.pose.position.y,
                             afCommand.pose.position.z);
            if( cmd_rot_quat.length() < 0.9 || cmd_rot_quat.length() > 1.1 ){
                cerr << "WARNING: BODY \"" << afRBPtr->getName() << "'s\" rotation quaternion command"
                                                        " not normalized" << endl;
                if (cmd_rot_quat.length() < 0.1){
                    cmd_rot_quat.setW(1.0); // Invalid Quaternion
                }
            }
            cmd_rot.setRotation(cmd_rot_quat);

            btVector3 pCommand, rCommand;
            // Use the internal Cartesian Position Controller to Compute Output
            pCommand = afRBPtr->m_controller.computeOutput<btVector3>(cur_pos, cmd_pos, dt);
            // Use the internal Cartesian Rotation Controller to Compute Output
            rCommand = afRBPtr->m_controller.computeOutput<btVector3>(cur_rot, cmd_rot, dt);

            if (afRBPtr->m_controller.m_positionOutputType == afControlType::FORCE){
                // IF PID GAINS WERE DEFINED, USE THE PID CONTROLLER
                // Use the internal Cartesian Position Controller
                btRBPtr->applyCentralForce(pCommand);
                btRBPtr->applyTorque(rCommand);
            }
            else{
                // ELSE USE THE VELOCITY INTERFACE
                btRBPtr->setLinearVelocity(pCommand);
                btRBPtr->setAngularVelocity(rCommand);
            }
        }
    }
        break;
    case ambf_msgs::RigidBodyCmd::TYPE_VELOCITY:{
        btVector3 lin_vel, ang_vel;
        afRBPtr->m_activeControllerType = afControlType::VELOCITY;
        if (btRBPtr){
            lin_vel.setValue(afCommand.twist.linear.x,
                             afCommand.twist.linear.y,
                             afCommand.twist.linear.z);

            ang_vel.setValue(afCommand.twist.angular.x,
                             afCommand.twist.angular.y,
                             afCommand.twist.angular.z);

            // If the body is kinematic, we just want to control the position
            if (btRBPtr->isStaticOrKinematicObject()){
                btTransform Tcommand, Tcurrent;
                Tcurrent = afRBPtr->getCOMTransform();
                btVector3 posCmd = Tcurrent.getOrigin() + lin_vel * dt;
                btVector3 rotCmd = ang_vel * dt;
                btQuaternion rotQ;
                rotQ.setEulerZYX(rotCmd.z(), rotCmd.y(), rotCmd.x());
                Tcommand.setOrigin(posCmd);

                Tcommand.setRotation(rotQ * Tcurrent.getRotation());

                // Compensate for the inertial offset
                Tcommand = Tcommand * afRBPtr->getInertialOffsetTransform();
                btRBPtr->getMotionState()->setWorldTransform(Tcommand);
                btRBPtr->setWorldTransform(Tcommand);

            }
            else{
                btRBPtr->setLinearVelocity(lin_vel);
                btRBPtr->setAngularVelocity(ang_vel);
            }
        }
    }
        break;
    default:
        break;
    }


    size_t jntCmdSize = afCommand.joint_cmds.size();
    if (jntCmdSize > 0){
        size_t jntCmdCnt = afRBPtr->m_CJ_PairsActive.size() < jntCmdSize ? afRBPtr->m_CJ_PairsActive.size() : jntCmdSize;
        for (size_t jntIdx = 0 ; jntIdx < jntCmdCnt ; jntIdx++){
            // A joint can be controller in three different modes, Effort, Positon or Velocity.
            afJointPtr joint = afRBPtr->m_CJ_PairsActive[jntIdx].m_childJoint;
            double jnt_cmd = afCommand.joint_cmds[jntIdx];
            switch (afCommand.joint_cmds_types[jntIdx]) {
            case ambf_msgs::RigidBodyCmd::TYPE_FORCE:
                joint->commandEffort(jnt_cmd);
                break;
            case ambf_msgs::RigidBodyCmd::TYPE_POSITION:
                joint->commandPosition(jnt_cmd);
                break;
            case ambf_msgs::RigidBodyCmd::TYPE_VELOCITY:
                joint->commandVelocity(jnt_cmd);
                break;
            default:
                cerr << "WARNING! FOR JOINT \"" <<
                        afRBPtr->m_CJ_PairsActive[jntIdx].m_childJoint->getName() <<
                        " \" COMMAND TYPE NOT UNDERSTOOD, SUPPORTED TYPES ARE 0 -> FORCE, 1 -> POSITION, 2 -> VELOCITY " <<
                        endl;
                break;
            }
        }
    }
}

void afObjectCommunicationPlugin::rigidBodyUpdateState(afRigidBodyPtr afRBPtr, double dt)
{
    m_rigidBodyCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    btRigidBody* btRBPtr = afRBPtr->m_bulletRigidBody;
    cQuaternion q;
    q.fromRotMat(afRBPtr->getLocalRot());

    // Update the Pose
    cVector3d localPos = afRBPtr->getLocalPos();
    m_rigidBodyCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    m_rigidBodyCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    // Update the Wrench
    btVector3 force = afRBPtr->m_estimatedForce;
    btVector3 torque = afRBPtr->m_estimatedTorque;
    m_rigidBodyCommPtr->cur_force(force.x(), force.y(), force.z());
    m_rigidBodyCommPtr->cur_torque(torque.x(), torque.y(), torque.z());

    btVector3 v = btRBPtr->getLinearVelocity();
    btVector3 a = btRBPtr->getAngularVelocity();

    // Updated the Twist
    m_rigidBodyCommPtr->cur_linear_velocity(v.x(), v.y(), v.z());
    m_rigidBodyCommPtr->cur_angular_velocity(a.x(), a.y(), a.z());

    // Since the mass and inertia aren't going to change that often, write them
    // out intermittently
    if (m_write_count % afRBPtr->m_afWorld->m_updateCounterLimit == 0){
        m_rigidBodyCommPtr->set_mass(afRBPtr->getMass());
        btVector3 inertia = afRBPtr->getInertia();
        m_rigidBodyCommPtr->set_principal_inertia(inertia.x(), inertia.y(), inertia.z());
    }

    ambf_msgs::RigidBodyCmd afCommand = m_rigidBodyCommPtr->get_command();
    // We can set this body to publish it's children joint names in either its AMBF Description file or
    // via it's afCommand using ROS Message
    if (afRBPtr->m_publish_joint_names == true || afCommand.publish_joint_names == true){
        if (afRBPtr->m_publish_joint_names == false){
            afRBPtr->m_publish_joint_names = true;
            m_rbState.setJointNames(afRBPtr);
        }
        // Since joint names aren't going to change that often
        // change the field less so often
        if (m_write_count % afRBPtr->m_afWorld->m_updateCounterLimit == 0){
            m_rbState.setJointNames(afRBPtr);
        }
    }

    // We can set this body to publish joint positions in either its AMBF Description file or
    // via it's afCommand using ROS Message
    if (afRBPtr->m_publish_joint_positions == true || afCommand.publish_joint_positions == true){
        m_rbState.setJointPositions(afRBPtr);
        m_rbState.setJointVelocities(afRBPtr);
        m_rbState.setJointEfforts(afRBPtr);
    }

    // We can set this body to publish it's children names in either its AMBF Description file or
    // via it's afCommand using ROS Message
    if (afRBPtr->m_publish_children_names == true || afCommand.publish_children_names == true){
        if (afRBPtr->m_publish_children_names == false){
            afRBPtr->m_publish_children_names = true;
            m_rbState.setChildrenNames(afRBPtr);
        }
        // Since children names aren't going to change that often
        // change the field less so often
        if (m_write_count % afRBPtr->m_afWorld->m_updateCounterLimit == 0){
            m_rbState.setChildrenNames(afRBPtr);
            m_write_count = 0;
        }
    }

    m_rigidBodyCommPtr->set_children_names(m_rbState.m_childrenNames);
    m_rigidBodyCommPtr->set_joint_names(m_rbState.m_jointNames);
    m_rigidBodyCommPtr->set_joint_positions(m_rbState.m_jointPositions);
    m_rigidBodyCommPtr->set_joint_velocities(m_rbState.m_jointVelocities);
    m_rigidBodyCommPtr->set_joint_efforts(m_rbState.m_jointEfforts);

    m_rigidBodyCommPtr->m_writeMtx.unlock();
    m_rigidBodyCommPtr->enableComm();
    m_write_count++;
}

void afObjectCommunicationPlugin::sensorFetchCommand(afSensorPtr senPtr, double dt)
{

}

void afObjectCommunicationPlugin::sensorUpdateState(afSensorPtr senPtr, double dt)
{
    m_sensorCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    switch (senPtr->m_sensorType) {
    case afSensorType::RAYTRACER:
    case afSensorType::RESISTANCE:
    case afSensorType::RANGE:
    {
        afRayTracerSensorPtr raySenPtr = (afRayTracerSensorPtr) senPtr;
        m_sensorCommPtr->set_count(raySenPtr->getCount());
        m_sensorCommPtr->set_parent_name(raySenPtr->m_parentName);
        m_sensorCommPtr->set_range(raySenPtr->m_range);
        cVector3d pos = raySenPtr->getLocalPos();
        cMatrix3d rot = raySenPtr->getLocalRot();
        cQuaternion quat;
        quat.fromRotMat(rot);
        m_sensorCommPtr->cur_position(pos.x(), pos.y(), pos.z());
        m_sensorCommPtr->cur_orientation(quat.x, quat.y, quat.z, quat.w);

        vector<bool> triggers;
        triggers.resize(raySenPtr->getCount());

        vector<string> sensed_obj_names;
        sensed_obj_names.resize(raySenPtr->getCount());

        vector<double> measurements;
        measurements.resize(raySenPtr->getCount());

        for (int i = 0 ; i < raySenPtr->getCount() ; i++){
            triggers[i] = raySenPtr->isTriggered(i);;
            measurements[i] = raySenPtr->getDepthFraction(i);
            if (triggers[i]){
                switch (raySenPtr->getSensedBodyType(i)) {
                case afBodyType::RIGID_BODY:
                    sensed_obj_names[i] = raySenPtr->getSensedRigidBody(i)->getName();
                    break;
                case afBodyType::SOFT_BODY:
                    sensed_obj_names[i] = raySenPtr->getSensedSoftBody(i)->getName();
                default:
                    sensed_obj_names[i] = "";
                    break;
                }
            }
        }

        m_sensorCommPtr->set_range(raySenPtr->m_range);
        m_sensorCommPtr->set_triggers(triggers);
        m_sensorCommPtr->set_measurements(measurements);
        m_sensorCommPtr->set_sensed_objects(sensed_obj_names);

    }
        break;
    default:
        break;
    }
    m_sensorCommPtr->m_writeMtx.unlock();
    m_sensorCommPtr->enableComm();
    m_write_count++;

}

void afObjectCommunicationPlugin::vehicleFetchCommand(afVehiclePtr vehPtr, double)
{
    ambf_msgs::VehicleCmd af_cmd = m_vehicleCommPtr->get_command();

    int maxWheelCount;
    int actualWheelCount = vehPtr->getWheelCount();

    if (af_cmd.brake == true){
        for (int i = 0 ; i < actualWheelCount ; i++){
            vehPtr->setWheelPower(i, 0.0);
        }
        vehPtr->engageBrake();
    }
    else{
        vehPtr->releaseBrake();

        maxWheelCount = af_cmd.wheel_power.size() <= actualWheelCount ? af_cmd.wheel_power.size() : actualWheelCount;

        for (int i = 0 ; i < maxWheelCount ; i++){
            vehPtr->setWheelPower(i, af_cmd.wheel_power[i]);
        }

        maxWheelCount = af_cmd.wheel_brake.size() <= actualWheelCount ? af_cmd.wheel_brake.size() : actualWheelCount;

        for (int i = 0 ; i < maxWheelCount ; i++){
            vehPtr->setWheelBrake(i, af_cmd.wheel_brake[i]);
        }
    }

    maxWheelCount = af_cmd.wheel_steering.size() <= actualWheelCount ? af_cmd.wheel_steering.size() : actualWheelCount;

    for (int i = 0 ; i < maxWheelCount ; i++){
        vehPtr->setWheelSteering(i, af_cmd.wheel_steering[i]);
    }



    // Apply forces and torques on the chassis
    btVector3 force(af_cmd.chassis_wrench.force.x,
                    af_cmd.chassis_wrench.force.y,
                    af_cmd.chassis_wrench.force.z);
    btVector3 torque(af_cmd.chassis_wrench.torque.x,
                     af_cmd.chassis_wrench.torque.y,
                     af_cmd.chassis_wrench.torque.z);

    if (force.length() > 0.0){
        vehPtr->setChassisForce(force);
    }
    if (torque.length() > 0.0){
        vehPtr->setChassisTorque(torque);
    }
}

void afObjectCommunicationPlugin::vehicleUpdateState(afVehiclePtr vehPtr, double dt)
{
    m_vehicleCommPtr->m_writeMtx.lock();
    setTimeStamps(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime(), m_objectPtr->getCurrentTimeStamp());
    cVector3d localPos = vehPtr->getLocalPos();
    m_vehicleCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(vehPtr->getLocalRot());
    m_vehicleCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    // Since the mass and inertia aren't going to change that often, write them
    // out intermittently
    if (m_write_count % vehPtr->m_afWorld->m_updateCounterLimit == 0){
        m_vehicleCommPtr->set_wheel_count(vehPtr->getWheelCount());
        m_vehicleCommPtr->set_mass(vehPtr->getMass());
        btVector3 inertia = vehPtr->getInertia();
        m_vehicleCommPtr->set_principal_inertia(inertia.x(), inertia.y(), inertia.z());
    }
    m_vehicleCommPtr->m_writeMtx.unlock();
    m_vehicleCommPtr->enableComm();
    m_write_count++;
}

void afObjectCommunicationPlugin::pointCloudFetchCommand(afPointCloudPtr pointCloudPtr, double)
{
    sensor_msgs::PointCloudPtr pcPtr = m_pointCloudCommPtr->get_point_cloud();
    if(pcPtr){
        double radius = m_pointCloudCommPtr->get_radius();
        pointCloudPtr->m_mpPtr->setPointSize(radius);
        int pc_size = pcPtr->points.size();
        int diff = pc_size - pointCloudPtr->m_mpSize;
        string frame_id = pcPtr->header.frame_id;

        if (pointCloudPtr->m_parentName.compare(frame_id) != 0 ){
            // First remove any existing parent
            if (pointCloudPtr->m_mpPtr->getParent() != nullptr){
                pointCloudPtr->m_mpPtr->getParent()->removeChild(pointCloudPtr->m_mpPtr);
            }

            afRigidBodyPtr pBody = pointCloudPtr->m_afWorld->getRigidBody(frame_id);
            if(pBody){
//                pBody->addChildObject(this);
                pBody->m_visualMesh->addChild(pointCloudPtr->m_mpPtr);
            }
            else{
                // Parent not found.
                cerr << "WARNING! FOR POINT CLOUD \""<< pointCloudPtr->m_topicName <<
                        "\" PARENT BODY \"" << frame_id <<
                        "\" NOT FOUND" << endl;
            }
        }

        pointCloudPtr->m_parentName = frame_id;

        if (diff >= 0){
            // PC array has either increased in size or the same size as MP array
            for (int pIdx = 0 ; pIdx < pointCloudPtr->m_mpSize ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                pointCloudPtr->m_mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
            }

            // Now add the new PC points to MP
            for (int pIdx = pointCloudPtr->m_mpSize ; pIdx < pc_size ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                pointCloudPtr->m_mpPtr->newPoint(pcPos);
            }
        }
        else{
            // PC array has decreased in size as compared to MP array
            for (int pIdx = 0 ; pIdx < pc_size ; pIdx++){
                cVector3d pcPos(pcPtr->points[pIdx].x,
                                pcPtr->points[pIdx].y,
                                pcPtr->points[pIdx].z);
                pointCloudPtr->m_mpPtr->m_points->m_vertices->setLocalPos(pIdx, pcPos);
            }

            for (int pIdx = pointCloudPtr->m_mpSize ; pIdx > pc_size ; pIdx--){
                pointCloudPtr->m_mpPtr->removePoint(pIdx-1);
            }
        }
        pointCloudPtr->m_mpSize = pc_size;

    }
}

void afObjectCommunicationPlugin::pointCloudUpdateState(afPointCloudPtr pointCloudPtr, double)
{
}

void afObjectCommunicationPlugin::volumeFetchCommand(afVolumePtr volPtr, double dt)
{

}

void afObjectCommunicationPlugin::volumeUpdateState(afVolumePtr volPtr, double dt)
{

}
#endif

