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
