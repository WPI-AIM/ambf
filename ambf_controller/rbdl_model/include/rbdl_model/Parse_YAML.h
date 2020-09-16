#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include<iostream>
#include<rbdl_model/Body.h>
#include <unordered_map>
#include "rbdl_model/Body.h"
#include "rbdl_model/Joint.h"

//#include <memory>
//#include <vector>
//#include <string>

//------------------------------------------------------------------------------
typedef Body* bodyPtr;
typedef Joint* jointPtr;
//------------------------------------------------------------------------------

class ParseYAML
{
public:
    ParseYAML(const std::string actuator_config_file);
    bool getBodies();
    bool getJoints();
    void cleanUp();

    ~ParseYAML(void);
private:
    YAML::Node baseNode_;
    std::string actuator_config_file_;

    std::unordered_map<std::string, bodyPtr> bodyObjectMap_;
    std::unordered_map<std::string, jointPtr> jointObjectMap_;
};

#endif // PARSE_YAML_H
