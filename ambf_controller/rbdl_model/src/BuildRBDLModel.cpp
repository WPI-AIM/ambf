#include "rbdl_model/BuildRBDLModel.h"

const RigidBodyDynamics::JointType blender_to_rbdl_joint_type_enum(BlenderJointType enumVal)
{
    if (enumVal == BlenderJointType::undefined) return JointTypeUndefined;
    else if (enumVal == BlenderJointType::revolute) return JointTypeRevolute;
    else if (enumVal == BlenderJointType::prismatic) return JointTypePrismatic;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeRevoluteX;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeRevoluteY;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeRevoluteZ;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeSpherical;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeEulerZYX;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeEulerXYZ;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeEulerYXZ;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeTranslationXYZ;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeFloatingBase;
    else if (enumVal == BlenderJointType::fixed) return JointTypeFixed;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeHelical;
//    else if (enumVal == BlenderJointType::revolute) return JointType1DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointType2DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointType3DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointType4DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointType5DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointType6DoF;
//    else if (enumVal == BlenderJointType::revolute) return JointTypeCustom;
}

//light_param_enum_to_str(LightParamsEnum::cuttoff_angle)

BuildRBDLModel::BuildRBDLModel(std::string actuator_config_file) {
    actuator_config_file_ = actuator_config_file;

    try{
        baseNode_ = YAML::LoadFile(actuator_config_file_);

    }catch (std::exception &e){
        std::cerr << "[Exception]: " << e.what() << std::endl;
        std::cerr << "ERROR! FAILED TO ACTUATOR CONFIG: " << actuator_config_file_ << std::endl;
        return;
    }
    if (baseNode_.IsNull()) return;
    if(!this->getBodies()) return;
    if(!this->getJoints()) return;
//    if(!this->findRootNode()) return;

//    this->BuildModel();
}




bool BuildRBDLModel::getBodies()
{
    Utilities utilities;

    YAML::Node rigidBodies = baseNode_["bodies"];
    if(!rigidBodies.IsDefined()) return false;
    size_t totalRigidBodies = rigidBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        std::string body_name_expanded = rigidBodies[i].as<std::string>();
        YAML::Node body_yaml = baseNode_[body_name_expanded];
        std::string body_name;

        if(body_yaml.IsDefined()) body_name = utilities.trimTrailingSpaces(body_yaml["name"]);
        bodyParamObjectMap_.insert(std::make_pair(body_name, new BodyParam(baseNode_[body_name_expanded])));
    }
    return true;
}



bool BuildRBDLModel::getJoints()
{
    YAML::Node joints = baseNode_["joints"];
    if(!joints.IsDefined()) return false;

    Utilities utilities;
    size_t totalJoints = joints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        std::string joint_name_expanded = joints[i].as<std::string>();
        YAML::Node joint_yaml = baseNode_[joint_name_expanded];
        std::string joint_name;
        if(joint_yaml.IsDefined()) joint_name = utilities.trimTrailingSpaces(joint_yaml["name"]);
//        std::cout << "joint_name: " << joint_name << std::endl;
        std::string parent_name;

        YAML::Node name = baseNode_[joint_name_expanded]["name"];
        if(name.IsDefined()) {
            YAML::Node parent = baseNode_[joint_name_expanded]["parent"];
            if(parent.IsDefined()) parent_name = utilities.trimTrailingSpaces(parent);
            utilities.eraseSubStr(parent_name, "BODY");
        }

        jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
        jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name_expanded])));
    }
    return true;
}


bool BuildRBDLModel::findRootNode() {
    std::unordered_set<std::string> parentNodeSet;
    std::unordered_set<std::string> childNodeSet;

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
    std::unordered_map<std::string, jointParamPtr>::iterator ptr;

    for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            jointParamPtr jointParamptr = ptr->second;
            parentNodeSet.emplace(jointParamptr->Parent());
            childNodeSet.emplace(jointParamptr->Child());
        }
    }


    for (std::unordered_set<std::string>::iterator it=childNodeSet.begin(); it!=childNodeSet.end(); ++it) {
        if(parentNodeSet.find(*it) != parentNodeSet.end()) parentNodeSet.erase(*it);
    }

    if(parentNodeSet.size() < 1) {
        std::cout << "No root node found, invalid model." << std::endl;
        return false;
    }
    if(parentNodeSet.size() > 1) {
        std::cout << "Found more than one root, make sure that model in YAML file has just one root." << std::endl;
        return false;
    }

    std::unordered_set<std::string>::iterator it = parentNodeSet.begin();
    rootRigidBody_ = *it;
    std::cout << "root node: " << *it << std::endl;

    return true;
}

//void BuildRBDLModel::BuildModel() {
//    rbdl_check_api_version(RBDL_API_VERSION);

//    Model *model = NULL;

//    unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
//    Body body_a, body_b, body_c, body_d;
//    Joint joint_a, joint_b, joint_c, joint_d;

//    // Initializing Body properties
//    double mass{1.};
//    Vector3d com(0., -0.344, 0.);
//    Vector3d inertia(0.058907, 0.003295, 0.058907);

//    model = new Model();

//    model->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

//    body_a = Body(mass, com, inertia); /* mass, com, inertia*/
//    joint_a = Joint(JointTypeFixed);

//    Matrix3_t body_a_rot;
//    body_a_rot << 0., 0., 1.,
//        0., -1., 0.,
//        1., 0., 0.;
//    Vector3d body_a_trans(0.001, -0.36, -0.222);
//    SpatialTransform body_a_tf(body_a_rot, body_a_trans);
//    body_a_id = model->AddBody(0, body_a_tf, joint_a, body_a);

//    body_b = Body(mass, com, inertia);
//    joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

//    Matrix3_t body_b_rot;
//    body_b_rot << 0., -1., 0.,
//        1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_b_trans(0.139, 0.138, 0.);
//    SpatialTransform body_b_tf(body_b_rot, body_b_trans);

//    body_b_id = model->AddBody(body_a_id, body_b_tf, joint_b, body_b);

//    body_c = Body(mass, com, inertia);
//    joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

//    Matrix3_t body_c_rot;
//    body_c_rot << 0., 1., 0.,
//        -1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_c_trans(-0.141, -0.832, 0.);
//    SpatialTransform body_c_tf(body_c_rot, body_c_trans);

//    body_c_id = model->AddBody(body_b_id, body_c_tf, joint_c, body_c);

//    body_d = Body(mass, com, inertia);
//    joint_d = Joint(
//        JointTypeRevolute,
//        Vector3d(0., 0., 1.));

//    Matrix3_t body_d_rot;
//    body_d_rot << 0., 1., 0.,
//        -1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_d_trans(-0.14, -0.83, 0.);
//    SpatialTransform body_d_tf(body_d_rot, body_d_trans);

//    body_d_id = model->AddBody(body_c_id, body_d_tf, joint_d, body_d);

//    VectorNd Q = VectorNd::Zero(model->dof_count);
//    // Q(0) = 0.2;
//    // Q(1) = 0.2;
//    // Q(2) = 0.2;
//    VectorNd QDot = VectorNd::Zero(model->dof_count);
//    VectorNd Tau = VectorNd::Zero(model->dof_count);
//    VectorNd QDDot = VectorNd::Zero(model->dof_count);

//    // ForwardDynamics(*model, Q, QDot, Tau, QDDot);

//    // std::cout << QDDot.transpose() << std::endl;

//    InverseDynamics(*model, Q, QDot, QDDot, Tau);
//    std::cout << Tau << std::endl;

//    delete model;
//}

bool BuildRBDLModel::BuildModel() {
    rbdl_check_api_version(RBDL_API_VERSION);

//    unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
//    Body body_a, body_b, body_c, body_d;
//    Joint joint_a, joint_b, joint_c, joint_d;

//    // Initializing Body properties
    double mass{1.};
    Vector3d com(0., -0.344, 0.);
    Vector3d inertia(0.058907, 0.003295, 0.058907);

    RBDLmodel_ = new Model();
    RBDLmodel_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction
    Matrix3_t body_rot;
    body_rot << 0., 0., 1.,
        0., -1., 0.,
        1., 0., 0.;

////    body_a = Body(mass, com, inertia); /* mass, com, inertia*/
//    joint_a = Joint(JointTypeFixed);


//    Vector3d body_trans(0.001, -0.36, -0.222);
//    SpatialTransform body_tf(body_rot, boda_trans);
////    body_a_id = RBDLmodel_->AddBody(0, body_tf, joint_a, body_a);


//    body_b = Body(mass, com, inertia);
//    joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

//    Matrix3_t body_b_rot;
//    body_b_rot << 0., -1., 0.,
//        1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_b_trans(0.139, 0.138, 0.);
//    SpatialTransform body_b_tf(body_b_rot, body_b_trans);

//    body_b_id = RBDLmodel_->AddBody(body_a_id, body_b_tf, joint_b, body_b);

//    body_c = Body(mass, com, inertia);
//    joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

//    Matrix3_t body_c_rot;
//    body_c_rot << 0., 1., 0.,
//        -1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_c_trans(-0.141, -0.832, 0.);
//    SpatialTransform body_c_tf(body_c_rot, body_c_trans);

//    body_c_id = RBDLmodel_->AddBody(body_b_id, body_c_tf, joint_c, body_c);BodyParam

//    body_d = Body(mass, com, inertia);
//    joint_d = Joint(
//        JointTypeRevolute,
//        Vector3d(0., 0., 1.));

//    Matrix3_t body_d_rot;
//    body_d_rot << 0., 1., 0.,
//        -1., 0., 0.,
//        0., 0., 1.;
//    Vector3d body_d_trans(-0.14, -0.83, 0.);
//    SpatialTransform body_d_tf(body_d_rot, body_d_trans);

//    body_d_id = RBDLmodel_->AddBody(body_c_id, body_d_tf, joint_d, body_d);

//Joint(JointTypeRevolute, Vector3d(0., 0., 1.))

//    std::cout << "jointParamObjectMap_.size() is " << jointParamObjectMap_.size() << std::endl;

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
    std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

//    for (outter_map_itr = jointParamObjectMap_.begin(); outter_map_itr != jointParamObjectMap_.end(); outter_map_itr++) {
//        std::string parent_node_name = outter_map_itr->first;
//        std::cout << "parent_node_name: " << parent_node_name << std::endl << "child_node_name: " ;
//        for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {
//            jointParamPtr jointparamPtr = inner_map_itr->second;
//            std::cout << jointparamPtr->Child() << ", ";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << "---------------------------------" << std::endl;

    std::unordered_set<std::string> ancestry_set;
    std::unordered_set<std::string>::iterator ancestry_set_itr;

    std::unordered_map<std::string, Body *> rbdlObjectMap;
    std::unordered_map<std::string, Body *> ::iterator rbdl_object_map_itr;

    ancestry_set.emplace(rootRigidBody_);
    while(!ancestry_set.empty()) {
        ancestry_set_itr = ancestry_set.begin();
        std::string parent = *ancestry_set_itr;

        ancestry_set.erase(*ancestry_set_itr);
//        std::cout << "parent: " << parent << std::endl;
//        mass = (bodyParamObjectMap_[parent])->Mass();
        rbdl_object_map_itr = rbdlObjectMap.find((parent));

//        com(0., -0.344, 0.);
//        inertia(0.058907, 0.003295, 0.058907);
        rbdlObjectMap.insert(std::make_pair(parent, new Body(mass, com, inertia)));

        outter_map_itr = jointParamObjectMap_.find(parent);
        if(outter_map_itr != jointParamObjectMap_.end()) {
            std::cout << "parent: " << parent << ", its children: ";

            for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {

                std::string child = inner_map_itr->second->Child();
                std::cout << child << ", ";


                rbdl_object_map_itr = rbdlObjectMap.find((child));
                if(rbdl_object_map_itr == rbdlObjectMap.end()) {
                    ancestry_set.emplace(child);
                }
            }
            std::cout << std::endl;
        }

    }
    return true;
}


void BuildRBDLModel::printBody() {
    std::unordered_map<std::string, bodyParamPtr>::iterator body_map_itr;
    for (body_map_itr = bodyParamObjectMap_.begin(); body_map_itr != bodyParamObjectMap_.end(); body_map_itr++) {
        std::cout << body_map_itr->first << std::endl;
    }
//    std::cout << std::endl;
}

void BuildRBDLModel::printJoint() {
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator outter_map_itr;
    std::unordered_map<std::string, jointParamPtr>::iterator inner_map_itr;

    for (outter_map_itr = jointParamObjectMap_.begin(); outter_map_itr != jointParamObjectMap_.end(); outter_map_itr++) {
        std::string parent_node_name = outter_map_itr->first;
        std::cout << ", parent_node_name: " << parent_node_name << std::endl << "child_node_name: " ;
        for (inner_map_itr = outter_map_itr->second.begin(); inner_map_itr != outter_map_itr->second.end(); inner_map_itr++) {
            std::string joint_name = inner_map_itr->first;
            std::cout << "joint_name: " << joint_name;
            jointParamPtr jointparamPtr = inner_map_itr->second;
            std::cout << jointparamPtr->Child() << ", ";

        }
        std::cout << std::endl;
    }
}


void BuildRBDLModel::cleanUp() {
    std::unordered_map<std::string, bodyParamPtr>::iterator bodyParamMapIt;
    for ( bodyParamMapIt = bodyParamObjectMap_.begin(); bodyParamMapIt != bodyParamObjectMap_.end(); ++bodyParamMapIt ) {
        bodyParamMapIt->second->~BodyParam();
    }

    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> >::iterator itr;
    std::unordered_map<std::string, jointParamPtr>::iterator ptr;

    for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            ptr->second->~JointParam();
        }
    }

//    delete RBDLmodel_;
}

BuildRBDLModel::~BuildRBDLModel(void){

}
