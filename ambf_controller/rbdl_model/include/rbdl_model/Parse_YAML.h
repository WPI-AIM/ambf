#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include<iostream>
#include<rbdl_model/Body.h>
#include <unordered_map>
//#include <memory>
//#include <vector>
//#include <string>

//------------------------------------------------------------------------------
typedef Body* bodyPtr;
//------------------------------------------------------------------------------

class ParseYAML
{
public:
    ParseYAML(const std::string actuator_config_file);
    void getBodies();

private:
    YAML::Node baseNode_;
    std::string actuator_config_file_;

    std::unordered_map<std::string, bodyPtr> bodyObjectMap_;
};

#endif // PARSE_YAML_H
