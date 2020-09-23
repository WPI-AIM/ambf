#ifndef RBDLBODY_H
#define RBDLBODY_H
#include "rbdl_model/Utilities.h"

class RBDLBody
{
public:
    RBDLBody(std::string name, double mass, Vector3d inertial_offset_position, Vector3d inertial_offset_orientation)
        : name_(name), mass_(mass), inertial_offset_position_(inertial_offset_position), inertial_offset_orientation_(inertial_offset_orientation) {}

private:
    std::string name_;
    double mass_{0.0};
    Vector3d location_position_;
    Vector3d location_orientation_;
    Vector3d inertial_offset_position_;
    Vector3d inertial_offset_orientation_;

};

#endif // RBDLBODY_H
