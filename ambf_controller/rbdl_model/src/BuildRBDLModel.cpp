#include "rbdl_model/BuildRBDLModel.h"

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
    if(!this->findRootNode()) return;

    this->BuildModel();
}




bool BuildRBDLModel::getBodies()
{
    YAML::Node rigidBodies = baseNode_["bodies"];
    if(!rigidBodies.IsDefined()) return false;
    size_t totalRigidBodies = rigidBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        std::string node_name = rigidBodies[i].as<std::string>();
        bodyParamObjectMap_.insert(std::make_pair(node_name, new BodyParam(baseNode_[node_name])));
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
        std::string joint_name = joints[i].as<std::string>();
        std::string parent_name;

        YAML::Node name = baseNode_[joint_name]["name"];
        if(name.IsDefined()) {
            YAML::Node parent = baseNode_[joint_name]["parent"];
            if(parent.IsDefined()) parent_name = utilities.trimTrailingSpaces(parent);
        }

        jointParamObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointParamPtr>()));
        jointParamObjectMap_[parent_name].insert(std::make_pair(joint_name, new JointParam(baseNode_[joint_name])));
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
//    double mass{1.};
//    Vector3d com(0., -0.344, 0.);
//    Vector3d inertia(0.058907, 0.003295, 0.058907);

//    RBDLmodel_ = new Model();

//    RBDLmodel_->gravity = Vector3d(0., 0., -9.81); // in my case should set in the Z direction

////    body_a = Body(mass, com, inertia); /* mass, com, inertia*/
//    joint_a = Joint(JointTypeFixed);

//    Matrix3_t body_a_rot;
//    body_rot << 0., 0., 1.,
//        0., -1., 0.,
//        1., 0., 0.;
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

//    body_c_id = RBDLmodel_->AddBody(body_b_id, body_c_tf, joint_c, body_c);

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




//    std::cout << "jointParamObjectMap_.size() is " << jointParamObjectMap_.size() << std::endl;

//    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>>::iterator itr;
//    std::unordered_map<std::string, jointParamPtr>::iterator ptr;

//    for (itr = jointParamObjectMap_.begin(); itr != jointParamObjectMap_.end(); itr++) {
//        std::string parent_node_name = itr->first;
//        std::cout << "parent_node_name: " << parent_node_name << std::endl << "child_node_name: " ;
//        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
//            jointParamPtr jointparamPtr = ptr->second;
//            std::cout << jointparamPtr->Child() << ", ";
//        }
//        std::cout << std::endl;
//    }
    return true;
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

    delete RBDLmodel_;
}

BuildRBDLModel::~BuildRBDLModel(void){

}
