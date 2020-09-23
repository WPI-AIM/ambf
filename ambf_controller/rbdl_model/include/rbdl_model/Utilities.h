#ifndef UTILITIES_H
#define UTILITIES_H
#include<chai3d.h>
#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>
#include <iostream>
#include "rbdl/rbdl_math.h"

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;

class Utilities
{
public:
    Utilities();

    Vector3d toXYZ(YAML::Node* node);
    Vector3d toRPY(YAML::Node* node);

    std::string trimTrailingSpaces(YAML::Node bodyNode);

    ~Utilities(void);
};

#endif // UTILITIES_H
