#ifndef UTILITIES_H
#define UTILITIES_H
#include <chai3d.h>
#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>
#include <iostream>
#include "rbdl/rbdl_math.h"
#include <algorithm>

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;

class Utilities
{
public:
    Utilities();

    Vector3d toXYZ(YAML::Node* node);
    Vector3d toRPY(YAML::Node* node);
    Vector3d toXYZInertia(YAML::Node* node);
    Matrix3_t toRotation(YAML::Node* node);

    std::string trimTrailingSpaces(YAML::Node bodyNode);
    void eraseSubStr(std::string & mainStr, const std::string & toErase);
    void eraseAllSubStr(std::string & mainStr, const std::string & toErase);

    ~Utilities(void);
};

#endif // UTILITIES_H
