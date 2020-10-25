#include "rbdl_model/Utilities.h"

Utilities::Utilities()
{

}


///
/// \brief toVector3d Inertia
/// \param node
/// \return
///
Vector3d Utilities::toXYZInertia(YAML::Node* node){
    Vector3d v;
    v(0) = (*node)["ix"].as<double>();
    v(1) = (*node)["iy"].as<double>();
    v(2) = (*node)["iz"].as<double>();
    return v;
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


///
/// \brief Matrix3_t
/// \param node
/// \return
///
Matrix3_t Utilities::toRotation(YAML::Node* node) {
    Matrix3_t m;

    m(0, 0) = (*node)["xx"].as<double>();
    m(0, 1) = (*node)["xy"].as<double>();
    m(0, 2) = (*node)["xz"].as<double>();

    m(1, 0) = (*node)["yx"].as<double>();
    m(1, 1) = (*node)["yy"].as<double>();
    m(1, 2) = (*node)["yz"].as<double>();

    m(2, 0) = (*node)["zx"].as<double>();
    m(2, 1) = (*node)["zy"].as<double>();
    m(2, 2) = (*node)["zz"].as<double>();

    return m;
}

std::string Utilities::trimTrailingSpaces(YAML::Node bodyNode) {
    std::string m_name;
    if(bodyNode.IsDefined()){
        m_name = bodyNode.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }
    return m_name;
}

/*
 * Erase First Occurrence of given  substring from main string.
 */
void Utilities::eraseSubStr(std::string & mainStr, const std::string & toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}

/*
 * Erase all Occurrences of given substring from main string.
 */
void Utilities::eraseAllSubStr(std::string & mainStr, const std::string & toErase)
{
    size_t pos = std::string::npos;
    // Search for the substring in string in a loop untill nothing is found
    while ((pos  = mainStr.find(toErase) )!= std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}

Utilities::~Utilities(void) {

}
