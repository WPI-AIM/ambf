#include "rbdl_model/Utilities.h"

Utilities::Utilities()
{

}

template <>
///
/// \brief toXYZ<cVector3d>
/// \param node
/// \return
///
cVector3d Utilities::toXYZ<cVector3d>(YAML::Node* node){
    cVector3d v;
    v.x((*node)["x"].as<double>());
    v.y((*node)["y"].as<double>());
    v.z((*node)["z"].as<double>());
    return v;
}

template<>
///
/// \brief toRPY<cVector3>
/// \param node
/// \return
///
cVector3d Utilities::toRPY<cVector3d>(YAML::Node *node){
    cVector3d v;
    v.x((*node)["r"].as<double>());
    v.y((*node)["p"].as<double>());
    v.z((*node)["y"].as<double>());
    return v;
}


std::string Utilities::trimTrailingSpaces(YAML::Node bodyNode) {
    std::string m_name;
    if(bodyNode.IsDefined()){
        m_name = bodyNode.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }
    return m_name;
}


Utilities::~Utilities(void) {

}
