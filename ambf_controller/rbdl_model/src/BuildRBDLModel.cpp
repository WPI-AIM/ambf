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


void BuildRBDLModel::BuildModel() {
    RigidBodyDynamics::Model *rbdl_model = NULL;
//    rbdl_model->gravity = Vector3d(0., 0., -9.81);
//    rbdl_model->gravity = Vector3d(0., 0., -9.81);


//    std::unordered_map<std::string, std::unordered_map<std::string, jointPtr> >::iterator itr;
//    std::unordered_map<std::string, jointPtr>::iterator ptr;
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
    std::unordered_map<std::string, jointParamPtr>::iterator jointParamMapIt;
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
}

BuildRBDLModel::~BuildRBDLModel(void){

}
