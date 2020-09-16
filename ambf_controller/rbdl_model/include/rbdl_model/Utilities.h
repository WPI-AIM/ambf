#ifndef UTILITIES_H
#define UTILITIES_H
#include<chai3d.h>
#include <yaml-cpp/yaml.h>

//template <typename T>
/////
///// \brief toXYZ
///// \param node
///// \return
/////
//T toXYZ(YAML::Node* node);



//template <typename T>
/////
///// \brief toRPY
///// \param node
///// \param v
///// \return
/////
//T toRPY(YAML::Node* node);

using namespace chai3d;
class Utilities
{
public:
    Utilities();
    template <typename T>
    ///
    /// \brief toXYZ
    /// \param node
    /// \return
    ///
    T toXYZ(YAML::Node* node);



    template <typename T>
    ///
    /// \brief toRPY
    /// \param node
    /// \param v
    /// \return
    ///
    T toRPY(YAML::Node* node);

    std::string trimTrailingSpaces(YAML::Node bodyNode);

    ~Utilities(void);
};

#endif // UTILITIES_H
