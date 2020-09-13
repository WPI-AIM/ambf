#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include<iostream>
#include<rbdl_model/Body.h>


class ParseYAML
{
public:
    ParseYAML(const std::string actuator_config_file);
    void getBodies();

private:
//    afRigidBodyPtr ParseYAML::getRootAFRigidBody(afRigidBodyPtr a_bodyPtr);


    YAML::Node baseNode_;
    std::string actuator_config_file_;


};

#endif // PARSE_YAML_H
