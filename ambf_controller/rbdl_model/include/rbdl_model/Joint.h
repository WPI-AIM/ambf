#ifndef JOINT_H
#define JOINT_H
#include<rbdl_model/Utilities.h>

class Joint
{
public:
    Joint(YAML::Node jointNode);
    ~Joint(void);

    inline std::string Name() { return name_; }
    inline std::string Parent() { return parent_; }
    inline std::string Child() { return child_; }

private:
    std::string name_;
    std::string parent_;
    std::string child_;
    cVector3d parent_axis_;
    cVector3d parent_pivot_;
    cVector3d child_axis_;
    cVector3d child_pivot_;
    double joint_limits_high_{0.0};
    double joint_limits_low_{0.0};
    bool passive_{false};
    bool detached_{false};
    std::string type_;
    double damping_{0.0};
    double offset_{0.0};
};

#endif // JOINTS_H
