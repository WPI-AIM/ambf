#include "rbdl_model/Body.h"

template <>
///
/// \brief toXYZ<cVector3d>
/// \param node
/// \return
///
cVector3d toXYZ<cVector3d>(YAML::Node* node){
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
cVector3d toRPY<cVector3d>(YAML::Node *node){
    cVector3d v;
    v.x((*node)["r"].as<double>());
    v.y((*node)["p"].as<double>());
    v.z((*node)["y"].as<double>());
    return v;
}

Body::Body(YAML::Node bodyNode)
{
//    name_ = name;
    // Declare all the yaml parameters that we want to look for
    name_ = trimTrailingSpaces(bodyNode["name"]);
    mass_ = bodyNode["mass"].as<double>();
    collision_margin_ = bodyNode["collision margin"].as<double>();
    scale_ = bodyNode["scale"].as<double>();

    YAML::Node location_position = bodyNode["location"]["position"];
    location_position_ = toXYZ<cVector3d>(&location_position);

    YAML::Node location_orientation = bodyNode["location"]["orientation"];
    location_orientation_ = toRPY<cVector3d>(&location_orientation);


    YAML::Node inertial_offset_position_ = bodyNode["inertial offset"]["position"];
    location_position_ = toXYZ<cVector3d>(&inertial_offset_position_);

    YAML::Node inertial_offset_orientation = bodyNode["inertial offset"]["orientation"];
    inertial_offset_orientation_ = toRPY<cVector3d>(&inertial_offset_orientation);

    passive_ = bodyNode["passive"].as<bool>();
    friction_rolling_ = bodyNode["friction"]["rolling"].as<double>();
    friction_static_ = bodyNode["friction"]["static"].as<double>();

    damping_angular_ = bodyNode["damping"]["angular"].as<double>();
    damping_linear_ = bodyNode["damping"]["linear"].as<double>();

}

std::string Body::trimTrailingSpaces(YAML::Node bodyNode) {
    std::string m_name;
    if(bodyNode.IsDefined()){
        m_name = bodyNode.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }
    return m_name;
}

Body::~Body() {

}
