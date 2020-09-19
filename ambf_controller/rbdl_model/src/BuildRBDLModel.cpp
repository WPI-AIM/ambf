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

//    this->BuildBodyTree();
}

bool BuildRBDLModel::getBodies()
{
    YAML::Node rigidBodies = baseNode_["bodies"];
    if(!rigidBodies.IsDefined()) return false;
    size_t totalRigidBodies = rigidBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        std::string node_name = rigidBodies[i].as<std::string>();
        bodyObjectMap_.insert(std::make_pair(node_name, new Body(baseNode_[node_name])));
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

        jointObjectMap_.insert(std::make_pair(parent_name, std::unordered_map<std::string, jointPtr>()));
        jointObjectMap_[parent_name].insert(std::make_pair(joint_name, new Joint(baseNode_[joint_name])));
    }
    return true;
}


bool BuildRBDLModel::findRootNode() {
    std::unordered_map<std::string, jointPtr>::iterator jointMapIt;
    std::unordered_set<std::string> parentSet;
    std::unordered_set<std::string> childSet;

    std::unordered_map<std::string, std::unordered_map<std::string, jointPtr> >::iterator itr;
    std::unordered_map<std::string, jointPtr>::iterator ptr;

    for (itr = jointObjectMap_.begin(); itr != jointObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            jointPtr jointptr = ptr->second;
            parentSet.emplace(jointptr->Parent());
            childSet.emplace(jointptr->Child());
        }
    }


    for (std::unordered_set<std::string>::iterator it=childSet.begin(); it!=childSet.end(); ++it) {
        if(parentSet.find(*it) != parentSet.end()) parentSet.erase(*it);
    }

    if(parentSet.size() < 1) {
        std::cout << "No root node found, invalid model." << std::endl;
        return false;
    }
    if(parentSet.size() > 1) {
        std::cout << "Found more than one root, make sure that model in YAML file has just one root." << std::endl;
        return false;
    }

    std::unordered_set<std::string>::iterator it = parentSet.begin();
    rootRigidBody_ = *it;
    std::cout << "root node: " << *it << std::endl;

    return true;
}


void BuildRBDLModel::cleanUp() {
    std::unordered_map<std::string, bodyPtr>::iterator bodyMapIt;
    for ( bodyMapIt = bodyObjectMap_.begin(); bodyMapIt != bodyObjectMap_.end(); ++bodyMapIt ) {
        bodyMapIt->second->~Body();
    }

    std::unordered_map<std::string, std::unordered_map<std::string, jointPtr> >::iterator itr;
    std::unordered_map<std::string, jointPtr>::iterator ptr;

    for (itr = jointObjectMap_.begin(); itr != jointObjectMap_.end(); itr++) {
        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            ptr->second->~Joint();
        }
    }
}

BuildRBDLModel::~BuildRBDLModel(void){

}
