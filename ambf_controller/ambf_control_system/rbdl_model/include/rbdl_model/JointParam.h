#ifndef JOINTPARAM_H
#define JOINTPARAM_H
#include <rbdl_model/Utilities.h>

class JointParam
{
public:
    JointParam(YAML::Node jointNode);
    JointParam(std::string name, std::string parent_name, std::string child, Vector3d parent_axis, Vector3d parent_pivot, std::string type);

    ~JointParam(void);

    inline std::string Name() { return name_; }
    inline std::string Parent() { return parent_; }
    inline std::string Child() { return child_; }
    inline Vector3d ParentAxis() { return parent_axis_; }
    inline Vector3d ParentPivot() { return parent_pivot_; }
    inline std::string Type() { return type_; }

private:
    std::string name_;
    std::string parent_;
    std::string child_;
    Vector3d parent_axis_;
    Vector3d parent_pivot_;
    Vector3d child_axis_;
    Vector3d child_pivot_;
    double joint_limits_high_{0.0};
    double joint_limits_low_{0.0};
    bool passive_{false};
    bool detached_{false};
    std::string type_;
    double damping_{0.0};
    double offset_{0.0};


};

#endif // JOINTS_H
