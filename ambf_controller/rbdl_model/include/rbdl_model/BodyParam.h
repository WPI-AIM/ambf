#ifndef BODYPARAM_H
#define BODYPARAM_H
#include <iostream>
#include <rbdl_model/Utilities.h>

class BodyParam
{
public:
    BodyParam(YAML::Node bodyNode);
    ~BodyParam(void);
    inline double Mass() { return mass_; }
    inline Vector3d InertialOffsetOrientation() { return inertial_offset_orientation_; }
    inline Vector3d Inertia() { return inertia_; }

private:

    std::string name_;
    double mass_{0.0};
    double collision_margin_{0.0};
    double scale_{0.0};

    Vector3d location_position_;
    Vector3d location_orientation_;
    Vector3d inertial_offset_position_;

    //substitute this value to com in RBDL
    Vector3d inertial_offset_orientation_;
    bool passive_{false};
    double friction_rolling_{0.0};
    double friction_static_{0.0};
    double damping_angular_{0.0};
    double damping_linear_{0.0};

    //substitute this value to inertia in RBDL, this is got from rbdl client
    Vector3d inertia_;

    std::string trimTrailingSpaces(YAML::Node bodyNode);
};

#endif // AFRIGIDBODY_H
