#ifndef DH_H
#define DH_H

#include<iostream>
#include<Eigen/Dense>

using namespace Eigen;

class DH
{
public:
    DH(const double alpha, const double a, const double theta, const double d, const double offset, const std::string joint_type)
        : alpha_(alpha), a_(a), theta_(theta), d_(d), offset_(offset), joint_type_(joint_type) {}

    Matrix4f mat_from_dh(double alpha, double a, double theta, double d, double offset);
    Matrix4f get_trans();

    ~DH(void);
private:
    const double alpha_{0.0};
    const double a_{0.0};
    const double theta_{0.0};
    const double d_{0.0};
    const double offset_{0.0};
    const std::string joint_type_;
};


#endif // DH_H
