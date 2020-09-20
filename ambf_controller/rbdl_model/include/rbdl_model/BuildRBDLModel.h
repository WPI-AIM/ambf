#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unordered_map>
#include "rbdl_model/BodyParam.h"
#include "rbdl_model/JointParam.h"
#include<rbdl/rbdl.h>
#include "rbdl/rbdl_math.h"

#include <unordered_set>
//using namespace RigidBodyDynamics;
//using namespace Math;

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

//------------------------------------------------------------------------------
typedef BodyParam* bodyParamPtr;
typedef JointParam* jointParamPtr;
//------------------------------------------------------------------------------

class BuildRBDLModel
{
public:
    BuildRBDLModel(const std::string actuator_config_file);

    void BuildModel();
    void cleanUp();

    ~BuildRBDLModel(void);
private:
    bool getBodies();
    bool getJoints();
    bool findRootNode();
    bool BuildBodyTree();


    YAML::Node baseNode_;
    std::string actuator_config_file_;
    std::string rootRigidBody_;

    std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr> > jointParamObjectMap_;


};

#endif // PARSE_YAML_H
