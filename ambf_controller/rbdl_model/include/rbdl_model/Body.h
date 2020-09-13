#ifndef BODY_H
#define BODY_H
#include <iostream>
#include "chai3d.h"
#include <yaml-cpp/yaml.h>
#include <x86intrin.h>
#include <cstdint>

//static const size_t AVX_alignment = 32;

//struct __m128_wrapper {
//    typedef __m128 T;
//};
//Ref to fix warning https://stackoverflow.com/questions/41676311/implication-of-gcc-warning-ignoring-attributes-on-template-argument-wignored

template <typename T>
///
/// \brief toXYZ
/// \param node
/// \return
///
T toXYZ(YAML::Node* node);



template <typename T>
///
/// \brief toRPY
/// \param node
/// \param v
/// \return
///
T toRPY(YAML::Node* node);



using namespace chai3d;

class Body
{
public:
    Body(YAML::Node bodyNode);
//    ~Body(void);
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
