#include "CommunicationPlugins.h"

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
image_transport::ImageTransport* afCameraVideoStreamerPlugin::s_imageTransport = nullptr;
#endif

int afObjectCommunicationPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;

    if (m_objectPtr == nullptr){
        cerr << "ERROR! OBJECT IS NULLPTR, FAILED TO INITIALIZE COMMUNICATION PLUGIN" << endl;
        return 0;
    }

    string objName = m_objectPtr->getName() + m_objectPtr->getGlobalRemapIdx();
    string objNamespace = m_objectPtr->getNamespace();
    int minFreq = m_objectPtr->getMinPublishFrequency();
    int maxFreq = m_objectPtr->getMaxPublishFrequency();
    double timeOut = 0.5;

    bool success = false;


#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    switch (m_objectPtr->getType()) {
    case afType::ACTUATOR:
    {
        m_afActuatorCommPtr.reset(new ambf_comm::Actuator(objName, objNamespace, minFreq, maxFreq, timeOut));
        afActuatorPtr actPtr = (afActuatorPtr)m_objectPtr;
        switch (actPtr->m_actuatorType) {
        case afActuatorType::CONSTRAINT:
            m_afActuatorCommPtr->set_type("CONSTRAINT");
            break;
        default:
            break;
        }
        success = true;

    }
        break;
    case afType::CAMERA:
    {
        m_afCameraCommPtr.reset(new ambf_comm::Camera(objName, objNamespace, minFreq, maxFreq, timeOut));
        success = true;
    }
        break;
    case afType::LIGHT:
    {
        m_afLightCommPtr.reset(new ambf_comm::Light(objName, objNamespace, minFreq, maxFreq, timeOut));
        success = true;
    }
        break;
    case afType::OBJECT:
    {
        m_afObjectCommPtr.reset(new ambf_comm::Object(objName, objNamespace, minFreq, maxFreq, timeOut));
        success = true;
    }
        break;
    case afType::RIGID_BODY:
    {
        m_afRigidBodyCommPtr.reset(new ambf_comm::RigidBody(objName, objNamespace, minFreq, maxFreq, timeOut));
        success = true;
    }
        break;
    case afType::SENSOR:
    {
        m_afSensorCommPtr.reset(new ambf_comm::Sensor(objName, objNamespace, minFreq, maxFreq, timeOut));
        afSensorPtr senPtr = (afSensorPtr) m_objectPtr;
        switch (senPtr->m_sensorType) {
        case afSensorType::RAYTRACER:
            m_afSensorCommPtr->set_type("PROXIMITY");
            break;
        case afSensorType::RESISTANCE:
            m_afSensorCommPtr->set_type("RESISTANCE");
            break;
        default:
            break;
        }
        success = true;
    }
        break;
    case afType::VEHICLE:
    {
        m_afVehicleCommPtr.reset(new ambf_comm::Vehicle(objName, objNamespace, minFreq, maxFreq, timeOut));
        success = true;
    }
        break;
    case afType::POINT_CLOUD:
    {
        afPointCloudPtr pcPtr = (afPointCloudPtr)m_objectPtr;
        m_afPointCloudCommPtr.reset(new ambf_comm::PointCloudHandler(pcPtr->m_topicName));
        success = true;
    }
        break;
    default:
    {
        cerr << "ERROR! COMMUNICATION TYPE FOR OBJECT NAMED " << objName << " OF TYPE: " << m_objectPtr->getTypeAsStr() << " NOT IMPLEMENTED YET" << endl;
        return -1;
    }
        break;
    }
#endif
    return success;
}

void afObjectCommunicationPlugin::graphicsUpdate()
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
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
#endif
}

void afObjectCommunicationPlugin::physicsUpdate(double dt)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    afUpdateTimes(m_objectPtr->m_afWorld->getWallTime(), m_objectPtr->m_afWorld->getSimulationTime());
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
    case afType::OBJECT:{
        afRigidBodyPtr objPtr = (afRigidBodyPtr)m_objectPtr;
//        objectFetchCommand(actPtr, dt);
//        objectUpdateState(actPtr, dt);
    }
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

#endif
}

bool afObjectCommunicationPlugin::close()
{

    return 1;
}

void afObjectCommunicationPlugin::afUpdateTimes(const double a_wall_time, const double a_sim_time)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    switch (m_commType) {
    case afType::ACTUATOR:
    {
        m_afActuatorCommPtr->set_wall_time(a_wall_time);
        m_afActuatorCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::CAMERA:
    {
        m_afCameraCommPtr->set_wall_time(a_wall_time);
        m_afCameraCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::LIGHT:
    {
        m_afLightCommPtr->set_wall_time(a_wall_time);
        m_afLightCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::OBJECT:
    {
        m_afObjectCommPtr->set_wall_time(a_wall_time);
        m_afObjectCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::RIGID_BODY:
    {
        m_afRigidBodyCommPtr->set_wall_time(a_wall_time);
        m_afRigidBodyCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::SENSOR:
    {
        m_afSensorCommPtr->set_wall_time(a_wall_time);
        m_afSensorCommPtr->set_sim_time(a_sim_time);
    }
        break;
    case afType::VEHICLE:
    {
        m_afVehicleCommPtr->set_wall_time(a_wall_time);
        m_afVehicleCommPtr->set_sim_time(a_sim_time);
    }
        break;
    }
#endif
}

void afObjectCommunicationPlugin::actuatorFetchCommand(afActuatorPtr actPtr, double)
{
    switch (actPtr->m_actuatorType) {
    case afActuatorType::CONSTRAINT:
    {
        ambf_msgs::ActuatorCmd cmd = m_afActuatorCommPtr->get_command();
        afConstraintActuatorPtr castPtr = (afConstraintActuatorPtr)actPtr;
        if (cmd.actuate){
            if (castPtr->isActuated()){
                // Constraint is active. Ignore request
                return;
            }
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
    m_afActuatorCommPtr->set_name(actPtr->getName());
    m_afActuatorCommPtr->set_parent_name(actPtr->m_parentName);
}

void afObjectCommunicationPlugin::cameraFetchCommand(afCameraPtr camPtr, double dt)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
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
            camPtr->setLocalPos(pos);
            camPtr->setLocalRot(rot_mat);
        }
        m_read_count++;
        if(m_read_count % (camPtr->m_afWorld->m_updateCounterLimit) == 0){
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

                string parent_name = m_afCameraCommPtr->get_parent_name();

                camPtr->getInternalCamera()->setClippingPlanes(near_plane, far_plane);

                camPtr->resolveParent(parent_name);

                switch (m_afCameraCommPtr->get_projection_type()) {
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

                switch (m_afCameraCommPtr->get_view_mode()) {
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
#endif

}

void afObjectCommunicationPlugin::cameraUpdateState(afCameraPtr camPtr, double dt)
{
    if (m_paramsSet == false){
        cCamera* cCamPtr = camPtr->getInternalCamera();
        m_afCameraCommPtr->set_near_plane(cCamPtr->getNearClippingPlane());
        m_afCameraCommPtr->set_far_plane(cCamPtr->getFarClippingPlane());
        m_afCameraCommPtr->set_field_view_angle(cCamPtr->getFieldViewAngleRad());
        m_afCameraCommPtr->set_orthographic_view_width(cCamPtr->getOrthographicViewWidth());
        m_afCameraCommPtr->set_steteo_eye_separation(cCamPtr->getStereoEyeSeparation());
        m_afCameraCommPtr->set_steteo_focal_length(cCamPtr->getStereoFocalLength());
        m_afCameraCommPtr->set_parent_name(camPtr->m_parentName);

        if (camPtr->getInternalCamera()->isViewModePerspective()){
            m_afCameraCommPtr->set_projection_type(ambf_comm::ProjectionType::PERSPECTIVE);
        }
        else{
            m_afCameraCommPtr->set_projection_type(ambf_comm::ProjectionType::ORTHOGRAPHIC);
        }

        if (camPtr->m_stereoMode == C_STEREO_DISABLED){
            m_afCameraCommPtr->set_view_mode(ambf_comm::ViewMode::MONO);
        }
        else{
            m_afCameraCommPtr->set_view_mode(ambf_comm::ViewMode::STEREO);;
        }

        m_afCameraCommPtr->set_params_on_server();
        m_paramsSet = true;
    }

    cVector3d localPos = camPtr->getLocalPos();
    m_afCameraCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(camPtr->getLocalRot());
    m_afCameraCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    m_write_count++;

    if (m_write_count % camPtr->m_afWorld->m_updateCounterLimit == 0){
        m_afCameraCommPtr->set_parent_name(camPtr->m_parentName);
        m_write_count = 0;
    }
}

void afObjectCommunicationPlugin::jointFetchCommand(afJointPtr jointPtr, double dt)
{

}

void afObjectCommunicationPlugin::jointUpdateState(afJointPtr jointPtr, double dt)
{

}

void afObjectCommunicationPlugin::lightFetchCommand(afLightPtr lightPtr, double dt)
{
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
        lightPtr->setLocalPos(pos);
        lightPtr->setLocalRot(rot_mat);
    }
    m_read_count++;
    if(m_read_count % lightPtr->m_afWorld->m_updateCounterLimit == 0){
        // We may update the params intermittently
        m_afLightCommPtr->update_params_from_server();
        if (m_afLightCommPtr->m_paramsChanged){
            // Clear the flag so it can be used for testing again
            m_afLightCommPtr->m_paramsChanged = false;

            double cutoff_angle = m_afLightCommPtr->get_cuttoff_angle();
            string parent_name = m_afLightCommPtr->get_parent_name();

            lightPtr->setCutOffAngle(cutoff_angle);

            lightPtr->resolveParent(parent_name);
        }

        m_read_count = 0;
    }
}

void afObjectCommunicationPlugin::lightUpdateState(afLightPtr lightPtr, double dt)
{
    if (m_paramsSet == false){
        m_afLightCommPtr->set_cuttoff_angle(lightPtr->getCutOffAngle());
        m_afLightCommPtr->set_type(ambf_comm::LightType::SPOT);
        m_afLightCommPtr->set_parent_name(lightPtr->m_parentName);

        m_afLightCommPtr->set_params_on_server();
        m_paramsSet = true;
    }

    cVector3d localPos = lightPtr->getLocalPos();
    m_afLightCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(lightPtr->getLocalRot());
    m_afLightCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    m_write_count++;

    if (m_write_count % lightPtr->m_afWorld->m_updateCounterLimit == 0){
        m_afLightCommPtr->set_parent_name(lightPtr->m_parentName);
        m_write_count = 0;
    }
}

void afObjectCommunicationPlugin::rigidBodyFetchCommand(afRigidBodyPtr afRBPtr, double dt)
{
    btRigidBody* btRBPtr = afRBPtr->m_bulletRigidBody;
    btVector3 force, torque;
    ambf_msgs::RigidBodyCmd afCommand = m_afRigidBodyCommPtr->get_command();

    // IF THE COMMAND IS OF TYPE FORCE
    switch (afCommand.cartesian_cmd_type) {
    case ambf_msgs::RigidBodyCmd::TYPE_FORCE:{
        afRBPtr->m_activeControllerType =  afControlType::FORCE;
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
            btRBPtr->getMotionState()->getWorldTransform(b_trans);

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
    btRigidBody* btRBPtr = afRBPtr->m_bulletRigidBody;
    cQuaternion q;
    q.fromRotMat(afRBPtr->m_visualMesh->getLocalRot());

    // Update the Pose
    cVector3d localPos = afRBPtr->getLocalPos();
    m_afRigidBodyCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    m_afRigidBodyCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    // Update the Wrench
    btVector3 force = afRBPtr->m_estimatedForce;
    btVector3 torque = afRBPtr->m_estimatedTorque;
    m_afRigidBodyCommPtr->cur_force(force.x(), force.y(), force.z());
    m_afRigidBodyCommPtr->cur_torque(torque.x(), torque.y(), torque.z());

    btVector3 v = btRBPtr->getLinearVelocity();
    btVector3 a = btRBPtr->getAngularVelocity();

    // Updated the Twist
    m_afRigidBodyCommPtr->cur_linear_velocity(v.x(), v.y(), v.z());
    m_afRigidBodyCommPtr->cur_angular_velocity(a.x(), a.y(), a.z());

    // Since the mass and inertia aren't going to change that often, write them
    // out intermittently
    if (m_write_count % afRBPtr->m_afWorld->m_updateCounterLimit == 0){
        m_afRigidBodyCommPtr->set_mass(afRBPtr->getMass());
        btVector3 inertia = afRBPtr->getInertia();
        m_afRigidBodyCommPtr->set_principal_inertia(inertia.x(), inertia.y(), inertia.z());
    }

    ambf_msgs::RigidBodyCmd afCommand = m_afRigidBodyCommPtr->get_command();
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

#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    m_afRigidBodyCommPtr->set_children_names(m_rbState.m_childrenNames);
    m_afRigidBodyCommPtr->set_joint_names(m_rbState.m_jointNames);
    m_afRigidBodyCommPtr->set_joint_positions(m_rbState.m_jointPositions);
    m_afRigidBodyCommPtr->set_joint_velocities(m_rbState.m_jointVelocities);
    m_afRigidBodyCommPtr->set_joint_efforts(m_rbState.m_jointEfforts);
#endif

    m_write_count++;
}

void afObjectCommunicationPlugin::sensorFetchCommand(afSensorPtr senPtr, double dt)
{

}

void afObjectCommunicationPlugin::sensorUpdateState(afSensorPtr senPtr, double dt)
{
    switch (senPtr->m_sensorType) {
    case afSensorType::RAYTRACER:
    case afSensorType::RESISTANCE:
    case afSensorType::RANGE:
    {
        afRayTracerSensorPtr raySenPtr = (afRayTracerSensorPtr) senPtr;
        m_afSensorCommPtr->set_count(raySenPtr->getCount());
        m_afSensorCommPtr->set_name(raySenPtr->getName());
        m_afSensorCommPtr->set_parent_name(raySenPtr->m_parentName);
        m_afSensorCommPtr->set_range(raySenPtr->m_range);
        cVector3d pos = raySenPtr->getLocalPos();
        cMatrix3d rot = raySenPtr->getLocalRot();
        cQuaternion quat;
        quat.fromRotMat(rot);
        m_afSensorCommPtr->cur_position(pos.x(), pos.y(), pos.z());
        m_afSensorCommPtr->cur_orientation(quat.x, quat.y, quat.z, quat.w);

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

        m_afSensorCommPtr->set_range(raySenPtr->m_range);
        m_afSensorCommPtr->set_triggers(triggers);
        m_afSensorCommPtr->set_measurements(measurements);
        m_afSensorCommPtr->set_sensed_objects(sensed_obj_names);

    }
        break;
    default:
        break;
    }

}

void afObjectCommunicationPlugin::vehicleFetchCommand(afVehiclePtr vehPtr, double)
{
    ambf_msgs::VehicleCmd af_cmd = m_afVehicleCommPtr->get_command();

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
    cVector3d localPos = vehPtr->getLocalPos();
    m_afVehicleCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    cQuaternion q;
    q.fromRotMat(vehPtr->getLocalRot());
    m_afVehicleCommPtr->cur_orientation(q.x, q.y, q.z, q.w);

    // Since the mass and inertia aren't going to change that often, write them
    // out intermittently
    if (m_write_count % vehPtr->m_afWorld->m_updateCounterLimit == 0){
        m_afVehicleCommPtr->set_wheel_count(vehPtr->getWheelCount());
        m_afVehicleCommPtr->set_mass(vehPtr->getMass());
        btVector3 inertia = vehPtr->getInertia();
        m_afVehicleCommPtr->set_principal_inertia(inertia.x(), inertia.y(), inertia.z());
    }
}

void afObjectCommunicationPlugin::pointCloudFetchCommand(afPointCloudPtr pointCloudPtr, double)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    sensor_msgs::PointCloudPtr pcPtr = m_afPointCloudCommPtr->get_point_cloud();
    if(pcPtr){
        double radius = m_afPointCloudCommPtr->get_radius();
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

#endif
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

int afWorldCommunicationPlugin::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs)
{
    m_worldPtr = a_afWorld;

    if (m_worldPtr == nullptr){
        cerr << "ERROR! WORLD IS NULLPTR, FAILED TO INITIALIZE COMMUNICATION PLUGIN" << endl;
        return 0;
    }

    string objName = m_worldPtr->getName();
    string objNamespace = m_worldPtr->getNamespace();
    int minFreq = m_worldPtr->getMinPublishFrequency();
    int maxFreq = m_worldPtr->getMaxPublishFrequency();
    double timeOut = 0.5;

    bool success = false;

#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    m_afWorldCommPtr.reset(new ambf_comm::World(objName, objNamespace, minFreq, maxFreq, timeOut));
    success = true;
#endif

    return success;
}

void afWorldCommunicationPlugin::graphicsUpdate()
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    if (m_paramsSet == false){
        // Create a default point cloud to listen to
        m_afWorldCommPtr->append_point_cloud_topic(m_worldPtr->getQualifiedName() + "/" + "point_cloud");
        m_afWorldCommPtr->set_params_on_server();
        m_paramsSet = true;
    }
#endif
}

void afWorldCommunicationPlugin::physicsUpdate(double dt)
{
    worldFetchCommand(m_worldPtr, dt);
    worldUpdateState(m_worldPtr, dt);

}

bool afWorldCommunicationPlugin::close()
{
    afROSNode::destroyNode();
    return 1;
}

void afWorldCommunicationPlugin::worldFetchCommand(afWorldPtr worldPtr, double)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT

    // If throttling is enabled, wait here until the step clock is toggled before
    // progressing towards next step
    while (!m_afWorldCommPtr->step_sim()){
        usleep(1);
    }

    m_read_count++;
    if(m_read_count % worldPtr->m_updateCounterLimit == 0){
        m_afWorldCommPtr->update_params_from_server();
        if (m_afWorldCommPtr->m_paramsChanged){
            // Do the stuff

            vector<string> def_topics = m_afWorldCommPtr->get_defunct_topic_names();
            vector<string> new_topics = m_afWorldCommPtr->get_new_topic_names();

            for (int i = 0 ; i < def_topics.size() ; i++){
                string topic_name = def_topics[i];
                if (worldPtr->m_pcMap.find(topic_name) != worldPtr->m_pcMap.end()){
                    // Cleanup
                    afPointCloudPtr afPC = worldPtr->m_pcMap.find(topic_name)->second;
                    worldPtr->m_pcMap.erase(topic_name);
                    delete afPC;
                }
            }

            for (int i = 0 ; i < new_topics.size() ; i++){
                string topic_name = new_topics[i];
                afPointCloudPtr afPC = new afPointCloud(worldPtr);
                afPC->m_topicName = topic_name;
                afPC->loadCommunicationPlugin(afPC, nullptr);
                worldPtr->m_pcMap[topic_name] = afPC;
            }
        }
        m_read_count = 0;
    }

#endif

}

void afWorldCommunicationPlugin::worldUpdateState(afWorldPtr worldPtr, double dt)
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    m_afWorldCommPtr->set_sim_time(worldPtr->getSimulationTime());
    m_afWorldCommPtr->set_wall_time(worldPtr->getWallTime());
    m_afWorldCommPtr->set_loop_freq(1000);
    m_afWorldCommPtr->set_num_devices(0);
#endif
}

int afCameraDepthStreamerPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;
    m_cameraPtr = (afCameraPtr)a_afObjectPtr;
    afCameraAttributes* camAttribs = (afCameraAttributes*) a_objectAttribs;

    m_depthPointCloudMsg.reset(new sensor_msgs::PointCloud2());
    m_depthPointCloudModifier = new sensor_msgs::PointCloud2Modifier(*m_depthPointCloudMsg);
    m_depthPointCloudModifier->setPointCloud2FieldsByString(2, "xyz", "rgb");
    m_depthPointCloudModifier->resize(camAttribs->m_publishImageResolution.m_width*camAttribs->m_publishImageResolution.m_height);
    m_rosNode = afROSNode::getNode();
    m_depthPointCloudPub = m_rosNode->advertise<sensor_msgs::PointCloud2>(m_cameraPtr->getQualifiedName() + "/DepthData", 1);

    m_publishInterval = camAttribs->m_publishDepthInterval;

    return 1;
}

void afCameraDepthStreamerPlugin::graphicsUpdate()
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    if (m_write_count % m_publishInterval == 0){
        sensor_msgs::PointCloud2Iterator<float> pcMsg_x(*m_depthPointCloudMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> pcMsg_y(*m_depthPointCloudMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> pcMsg_z(*m_depthPointCloudMsg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_r(*m_depthPointCloudMsg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_g(*m_depthPointCloudMsg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_b(*m_depthPointCloudMsg, "b");

        int width = m_cameraPtr->m_depthBufferColorImage->getWidth();
        int height = m_cameraPtr->m_depthBufferColorImage->getHeight();

        for (int idx = 0 ; idx < width * height ; idx++, ++pcMsg_x, ++pcMsg_y, ++pcMsg_z, ++pcMsg_r, ++pcMsg_g, ++pcMsg_b){
            double noise;
            if (m_cameraPtr->getDepthNoiseModel()->isEnabled()){
                noise = m_cameraPtr->getDepthNoiseModel()->generate();
            }
            else{
                noise = 0.0;
            }
            *pcMsg_x = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 0];
            *pcMsg_y = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 1];
            *pcMsg_z = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 2] + noise;

            *pcMsg_r = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 0];
            *pcMsg_g = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 1];
            *pcMsg_b = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 2];
        }

        m_depthPointCloudMsg->header.frame_id = m_cameraPtr->getName();
        m_depthPointCloudMsg->header.stamp = ros::Time::now();
        m_depthPointCloudPub.publish(m_depthPointCloudMsg);
    }
#endif
    m_write_count++;
}

void afCameraDepthStreamerPlugin::physicsUpdate(double)
{

}

bool afCameraDepthStreamerPlugin::close()
{
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    if (m_depthPointCloudModifier != nullptr){
        delete m_depthPointCloudModifier;
        m_depthPointCloudModifier = 0;
    }
#endif
    return true;
}

int afCameraVideoStreamerPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;
    m_cameraPtr = (afCameraPtr)a_afObjectPtr;
    afCameraAttributes* camAttribs = (afCameraAttributes*) a_objectAttribs;
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    m_rosNode = afROSNode::getNode();
    if (s_imageTransport == nullptr){
        s_imageTransport = new image_transport::ImageTransport(*m_rosNode);
    }
    m_imagePublisher = s_imageTransport->advertise(m_cameraPtr->getQualifiedName() + "/ImageData", 1);
#endif

    m_publishInterval = camAttribs->m_publishImageInterval;
    return 1;
}

void afCameraVideoStreamerPlugin::graphicsUpdate()
{
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    if (m_write_count % m_publishInterval == 0){
        // UGLY HACK TO FLIP ONCES BEFORE PUBLISHING AND THEN AGAIN AFTER TO HAVE CORRECT MAPPING
        // WITH THE COLORED DETPH POINT CLOUD
        m_cameraPtr->m_bufferColorImage->flipHorizontal();
        m_imageMatrix = cv::Mat(m_cameraPtr->m_bufferColorImage->getHeight(), m_cameraPtr->m_bufferColorImage->getWidth(), CV_8UC4, m_cameraPtr->m_bufferColorImage->getData());
        cv::cvtColor(m_imageMatrix, m_imageMatrix, cv::COLOR_RGBA2RGB);
        sensor_msgs::ImagePtr rosMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_imageMatrix).toImageMsg();
        rosMsg->header.stamp = ros::Time::now();
        m_imagePublisher.publish(rosMsg);
        m_cameraPtr->m_bufferColorImage->flipHorizontal();
    }
#endif
    m_write_count++;
}

void afCameraVideoStreamerPlugin::physicsUpdate(double)
{

}

bool afCameraVideoStreamerPlugin::close()
{
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    if (s_imageTransport != nullptr){
        delete s_imageTransport;
        s_imageTransport = nullptr;
    }
#endif
    return true;
}
