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
typedef RigidBodyDynamics::Body* rbdlBodyptr;
//------------------------------------------------------------------------------

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
    void addDummyRootJoint();
    bool buildBodyTree();
    bool buildModel();

    YAML::Node baseNode_;
    std::string actuator_config_file_;
    std::string rootRigidBody_;

    const std::string root_parent_name_ = "world";
    std::string root_joint_name_;
    Model *RBDLmodel_ = NULL;

    std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;

    //                 <parent,                       <jointname, jointParamPtr>>
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_;
    const RigidBodyDynamics::JointType getRBDLJointType(std::string joint_type);
    unsigned int addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name);
};

#endif // PARSE_YAML_H
