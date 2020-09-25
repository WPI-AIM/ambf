#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unordered_map>
#include "rbdl_model/BodyParam.h"
#include "rbdl_model/JointParam.h"
#include "rbdl_model/Utilities.h"
#include <queue>
#include<rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>

#include <unordered_set>
using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;

//------------------------------------------------------------------------------
typedef BodyParam* bodyParamPtr;
typedef JointParam* jointParamPtr;
//------------------------------------------------------------------------------

enum class BlenderJointType {
    undefined,
    revolute,
    prismatic,
    fixed,
    continuous,
    linear_spring,
    torsion_spring,
    p2p
};

class BuildRBDLModel
{
public:
    BuildRBDLModel(const std::string actuator_config_file);

//    Model GetRBDLModel();

    void printBody();
    void printJoint();
    void cleanUp();

    ~BuildRBDLModel(void);
private:
    bool getBodies();
    bool getJoints();
    bool findRootNode();
    bool BuildBodyTree();
    bool BuildModel();

    YAML::Node baseNode_;
    std::string actuator_config_file_;
    std::string rootRigidBody_;
    Model *RBDLmodel_ = NULL;

    std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_;


};

#endif // PARSE_YAML_H
