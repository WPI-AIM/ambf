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

    size_t totalJoints = joints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        std::string joint_name = joints[i].as<std::string>();
        jointObjectMap_.insert(std::make_pair(joint_name, new Joint(baseNode_[joint_name])));
    }
    return true;
}


void BuildRBDLModel::cleanUp() {
    std::unordered_map<std::string, bodyPtr>::iterator bodyMapIt;
    for ( bodyMapIt = bodyObjectMap_.begin(); bodyMapIt != bodyObjectMap_.end(); ++bodyMapIt ) {
        bodyMapIt->second->~Body();
    }

    std::unordered_map<std::string, jointPtr>::iterator jointMapIt;
    for ( jointMapIt = jointObjectMap_.begin(); jointMapIt != jointObjectMap_.end(); ++jointMapIt ) {
        jointMapIt->second->~Joint();
    }
}


//template <typename TMap>
//void BuildRBDLModel::cleanUpHelper(TMap* a_map) {
//    typename TMap::iterator oIt = a_map->begin();
//    for (; oIt != a_map->end() ; ++oIt){
//        if(instanceof<Body>(oIt->second)) {
//           std::cout << "c is instance of Body class" << std::endl;
////           (dynamic_cast<Body*>(oIt->second))->~Body();
////           (oIt->second)->~Body();
//           (dynamic_cast<Body*>(oIt->second))->~Body();
////           (dynamic_cast<Body&>(oIt->second))->~Body();
////           dynamic_cast<AA&>(a).aa();
////        } else if(instanceof<Joint>(oIt->second)) {
////           std::cout << "c is instance of Joint class" << std::endl;
//        }
//    }
//}

BuildRBDLModel::~BuildRBDLModel(void){

}
