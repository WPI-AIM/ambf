#include "rbdl_model/JointParam.h"

JointParam::JointParam(YAML::Node jointNode)
{
    // Declare all the yaml parameters that we want to look for
    Utilities utilities;

    YAML::Node name = jointNode["name"];
    if(name.IsDefined()) name_ = utilities.trimTrailingSpaces(name);

    YAML::Node parent = jointNode["parent"];
    if(parent.IsDefined()) parent_ = utilities.trimTrailingSpaces(parent);

    YAML::Node child = jointNode["child"];
    if(child.IsDefined()) child_ = utilities.trimTrailingSpaces(child);

    YAML::Node parent_pivot = jointNode["parent pivot"];
    if(parent_pivot.IsDefined()) parent_pivot_ = utilities.toXYZ<cVector3d>(&parent_pivot);

    YAML::Node parent_axis = jointNode["parent axis"];
    if(parent_axis.IsDefined()) parent_axis_ = utilities.toXYZ<cVector3d>(&parent_axis);

    YAML::Node child_pivot = jointNode["child pivot"];
    if(parent_pivot.IsDefined()) child_pivot_ = utilities.toXYZ<cVector3d>(&child_pivot);

    YAML::Node child_axis = jointNode["child axis"];
    if(parent_axis.IsDefined()) child_axis_ = utilities.toXYZ<cVector3d>(&child_axis);

    YAML::Node joint_limits = jointNode["joint limits"];
    if(joint_limits.IsDefined()) {
        YAML::Node high = joint_limits["high"];
        if(high.IsDefined()) joint_limits_high_ = high.as<double>();

        YAML::Node low = joint_limits["low"];
        if(low.IsDefined()) joint_limits_low_ = low.as<double>();
    }

    YAML::Node passive = jointNode["passive"];
    if(passive.IsDefined()) passive_ = passive.as<bool>();

    YAML::Node detached = jointNode["detached"];
    if(detached.IsDefined()) detached_ = detached.as<bool>();

    YAML::Node type = jointNode["type"];
    if(type.IsDefined()) type_ = utilities.trimTrailingSpaces(type);

    YAML::Node damping = jointNode["damping"];
    if(damping.IsDefined()) damping_ = jointNode["damping"].as<double>();

    YAML::Node offset = jointNode["offset"];
    if(offset.IsDefined()) offset_ = jointNode["offset"].as<double>();
}

JointParam::~JointParam(void) {

}
