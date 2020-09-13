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
}

void ParseYAML::getBodies()
{
    YAML::Node multiBodyRidigBodies = baseNode_["bonode_namedies"];
    size_t totalRigidBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        std::string node_name = multiBodyRidigBodies[i].as<std::string>();
        bodyObjectMap_.insert(std::make_pair(node_name, new Body(baseNode_[node_name])));
    }
}

