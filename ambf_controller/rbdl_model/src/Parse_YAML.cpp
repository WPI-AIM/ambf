#include "rbdl_model/Parse_YAML.h"

ParseYAML::ParseYAML(std::string actuator_config_file) {
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
//    if(!this->getJoints()) return;
}

bool ParseYAML::getBodies()
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

bool ParseYAML::getJoints()
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


void ParseYAML::cleanUp() {
    std::unordered_map<std::string, bodyPtr>::iterator itr;
    for(itr = bodyObjectMap_.begin(); itr != bodyObjectMap_.end(); itr) {
        bodyPtr handler = itr->second;
        handler->~Body();
    }
}

ParseYAML::~ParseYAML(void){

}
