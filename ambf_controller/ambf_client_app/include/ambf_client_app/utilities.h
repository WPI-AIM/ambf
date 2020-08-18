#ifndef UTILITIES_H
#define UTILITIES_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<cmath>

using namespace Eigen;
class Utilities
{
public:
    Utilities();
    float get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector);

    ~Utilities(void);
};

#endif // UTILITIES_H
