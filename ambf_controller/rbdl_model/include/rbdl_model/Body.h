#ifndef BODY_H
#define BODY_H
#include <iostream>
#include <rbdl_model/Utilities.h>

class Body
{
public:
    Body(YAML::Node bodyNode);
    ~Body(void);
private:

    std::string name_;
    double mass_{0.0};
    double collision_margin_{0.0};
    double scale_{0.0};
    cVector3d location_position_;
    cVector3d location_orientation_;
    cVector3d inertial_offset_position_;
    cVector3d inertial_offset_orientation_;
    bool passive_{false};
    double friction_rolling_{0.0};
    double friction_static_{0.0};
    double damping_angular_{0.0};
    double damping_linear_{0.0};

    std::string trimTrailingSpaces(YAML::Node bodyNode);
};

#endif // AFRIGIDBODY_H
