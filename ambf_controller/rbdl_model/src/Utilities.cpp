#include "rbdl_model/Utilities.h"

Utilities::Utilities()
{

}

///
/// \brief toVector3d
/// \param node
/// \return
///
Vector3d Utilities::toXYZ(YAML::Node* node){
    Vector3d v;
    v(0) = (*node)["x"].as<double>();
    v(1) = (*node)["y"].as<double>();
    v(2) = (*node)["z"].as<double>();
    return v;
}


///
/// \brief toVector3d
/// \param node
/// \return
///
Vector3d Utilities::toRPY(YAML::Node* node){
    Vector3d v;
    v(0) = (*node)["r"].as<double>();
    v(1) = (*node)["p"].as<double>();
    v(2) = (*node)["y"].as<double>();
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
