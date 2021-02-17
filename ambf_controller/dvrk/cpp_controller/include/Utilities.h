#ifndef UTILITIES_H
#define UTILITIES_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<cmath>
#include <stdlib.h>

using namespace Eigen;
class Utilities
{
public:
    Utilities();
    float get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);
    float get_random_between_range(float low, float high);
    Eigen::Matrix3f rotation_from_euler(float roll, float pitch, float yaw);
    Eigen::Vector3f rpy_from_rotation(Eigen::Matrix3f R);
    Eigen::Matrix4f get_frame(Eigen::Matrix3f R, Vector3f T);
    ~Utilities(void);
};

#endif // UTILITIES_H
