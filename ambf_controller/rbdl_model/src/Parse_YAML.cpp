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


/////
///// \brief afMultiBody::getRootRigidBody
///// \param a_bodyPtr
///// \return
/////
//afRigidBodyPtr ParseYAML::getRootAFRigidBody(afRigidBodyPtr a_bodyPtr){
//    if (!a_bodyPtr){
//        std::cerr << "ERROR, BODY PTR IS NULL, CAN\'T LOOK UP ROOT BODIES" << std::endl;
//        return 0;
//    }

//    /// Find Root Body
//    afRigidBodyPtr rootParentBody;
//    std::vector<int> bodyParentsCount;
//    size_t rootParents = 0;
//    if (a_bodyPtr->m_parentBodies.size() == 0){
//        rootParentBody = a_bodyPtr;
//        rootParents++;
//    }
//    else{
//        bodyParentsCount.resize(a_bodyPtr->m_parentBodies.size());
//        std::vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
//        for (int parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
//            if ((*rIt)->m_parentBodies.size() == 0){
//                rootParentBody = (*rIt);
//                rootParents++;
//            }
//            bodyParentsCount[parentNum] = (*rIt)->m_parentBodies.size();
//        }
//    }

//    // In case no root parent is found, it is understood that
//    // the multibody chain is cyclical, perhaps return
//    // the body with least number of parents
//    if (rootParents == 0){
//        auto minLineage = std::min_element(bodyParentsCount.begin(), bodyParentsCount.end());
//        int idx = std::distance(bodyParentsCount.begin(), minLineage);
//        rootParentBody = a_bodyPtr->m_parentBodies[idx];
//        rootParents++;
//        std::cerr << "WARNING! CYCLICAL CHAIN OF BODIES FOUND WITH NO UNIQUE PARENT, RETURING THE BODY WITH LEAST PARENTS";
//    }

//    if (rootParents > 1)
//        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

//    return rootParentBody;
//}


void ParseYAML::getBodies()
{
//    YAML::Node baseActuatorrNode = baseNode_[node_name];

//    YAML::Node sensorNamespace = sensorNode["namespace"];
//    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodyRidigBodies = baseNode_["bodies"];
//    std::cout << multiBodyRidigBodies.as<std::string>() << std::endl;
//    size_t totalRigidBodies = multiBodyRidigBodies.size();
//    for (size_t i = 0; i < totalRigidBodies; ++i) {
////        rBodyPtr = new afRigidBody(m_afWorld);
//        std::string rb_name = multiBodyRidigBodies[i].as<std::string>();
//        std::cerr << i << ", " << rb_name <<std::endl;

//    }
    std::string node_name = multiBodyRidigBodies[0].as<std::string>();
//    YAML::Node bodyNode = baseNode_[node_name];
//    std::cout << bodyNode["name"] << std::endl;
    Body body1(baseNode_[node_name]);


//    if (bodyNamespace.IsDefined()){
//        m_namespace = afUtils::removeAdjacentBackSlashes(bodyNamespace.as<std::string>());
//    }

}
