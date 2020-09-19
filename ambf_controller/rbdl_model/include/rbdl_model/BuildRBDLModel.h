#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <rbdl_model/Body.h>
#include <unordered_map>
#include "rbdl_model/Body.h"
#include "rbdl_model/Joint.h"

#include <unordered_set>
//#include<set>
//#include <algorithm>
//#include <iterator>

//#include <memory>
//#include <vector>
//#include <string>

//------------------------------------------------------------------------------
typedef Body* bodyPtr;
typedef Joint* jointPtr;
//------------------------------------------------------------------------------

class BuildRBDLModel
{
public:
    BuildRBDLModel(const std::string actuator_config_file);
    bool getBodies();
    bool getJoints();
    bool findRootNode();
    bool BuildBodyTree();
    void cleanUp();

    ~BuildRBDLModel(void);
private:
    YAML::Node baseNode_;
    std::string actuator_config_file_;

    //Hardcoded for now. Write a function to find it out for any model.
    std::string rootRigidBody_;

    std::unordered_map<std::string, bodyPtr> bodyObjectMap_;
//    std::unordered_map<std::string, jointPtr> jointObjectMap_;
    std::unordered_map<std::string, std::unordered_map<std::string, jointPtr> > jointObjectMap_;
};

#endif // PARSE_YAML_H
