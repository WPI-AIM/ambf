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
    void Inertia(Vector3d inertia);

private:

    std::string name_;
    double mass_{0.00000001};
    Vector3d inertia_;
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

    std::string trimTrailingSpaces(YAML::Node bodyNode);

};

#endif // AFRIGIDBODY_H
