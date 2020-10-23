//#include "rbdl/Body.h"
//#include <yaml-cpp/yaml.h>
//#include <boost/filesystem/path.hpp>
//#include <fstream>
#include<iostream>
//#include <thread>
#include "rbdl_model/BuildRBDLModel.h"

int main(int argc, char* argv[])
{
//    const std::string actuator_config_file = "/home/shreyas/ambf/ambf_models/descriptions/multi-bodies/robots/blender-psm.yaml";
    const std::string actuator_config_file = "/localcodebase/ambf_addon_updated/PSM/PSM_with_body_rotation.yaml";
//    const std::string actuator_config_file = "/home/shreyas/ambf/ambf_models/descriptions/multi-bodies/robots/blender-ecm.yaml";
//    const std::string actuator_config_file = "/home/shreyas/ambf/ambf_models/descriptions/multi-bodies/robots/blender-mtm.yaml";
//    const std::string actuator_config_file = "/home/shreyas/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka.yaml";
//    const std::string actuator_config_file = "/home/shreyas/Downloads/simulator/exohuman.yaml";

    BuildRBDLModel buildRBDLModel(actuator_config_file);
//    buildRBDLModel.printBody();

//    buildRBDLModel.printJoint();
//    buildRBDLModel.cleanUp();
//    py.getBodies();

//    YAML::Node baseNode;
//    const std::string actuator_config_file = "/home/shreyas/ambf/ambf_models/descriptions/multi-bodies/robots/blender-psm.yaml";
//    try{
//        baseNode = YAML::LoadFile(actuator_config_file);

//    }catch (std::exception &e){
//        std::cerr << "[Exception]: " << e.what() << std::endl;
//        std::cerr << "ERROR! FAILED TO ACTUATOR CONFIG: " << actuator_config_file << std::endl;
//        return 0;
//    }
//    if (baseNode.IsNull()) return false;

//    YAML::Node baseActuatorrNode = baseNode[node_name];

//    YAML::Node sensorNamespace = sensorNode["namespace"];
//    YAML::Node bodyNamespace = bodyNode["namespace"];
//    if (bodyNamespace.IsDefined()){
//        m_namespace = afUtils::removeAdjacentBackSlashes(bodyNamespace.as<std::string>());
//    }

//    ///
//    /// \brief afUtils::removeDoubleBackSlashes
//    /// \param a_name
//    /// \return
//    ///
//    std::string afUtils::removeAdjacentBackSlashes(std::string a_name){
//        std::string cleaned_name;
//        int last_back_slash_idx = -2;
//        for (int i = 0; i < a_name.length() ; i++){
//            if (a_name[i] == '/'){
//                if (i - last_back_slash_idx > 1){
//                    cleaned_name.push_back(a_name[i]);
//                }
//                last_back_slash_idx = i;
//            }
//            else{
//                cleaned_name.push_back(a_name[i]);
//            }
//        }
//        return cleaned_name;
//    }

	return 0;
}
