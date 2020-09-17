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

class BuildRBDLModel
{
public:
    BuildRBDLModel(const std::string actuator_config_file);
    bool getBodies();
    bool getJoints();
    void cleanUp();

    ~BuildRBDLModel(void);
private:
    YAML::Node baseNode_;
    std::string actuator_config_file_;

    std::unordered_map<std::string, bodyPtr> bodyObjectMap_;
    std::unordered_map<std::string, jointPtr> jointObjectMap_;

//    template<typename Base, typename T>
//    inline bool instanceof(const T*) {
//       return std::is_base_of<Base, T>::value;
//    }

//    template <typename TMap>
//    void cleanUpHelper(TMap* a_map);
};

#endif // PARSE_YAML_H
