#include "DH.h"


Matrix4f DH::mat_from_dh(double alpha, double a, double theta, double d, double offset) {
    Matrix4f mat;


    float ca = cos(alpha);
    float sa = sin(alpha);

    if(joint_type_.compare("ROTATIONAL") == 0) {
        theta += offset;
    } else if(joint_type_.compare("PRISMATIC") == 0) {
        d += offset;
    } else nullptr;

    float ct = cos(theta);
    float st = sin(theta);


    mat <<  ct     ,    -st    ,      0,          a,
            st * ca,    ct * ca,    -sa,    -d * sa,
            st * sa,    ct * sa,     ca,     d * ca,
            0      ,          0,      0,           1;

   return mat;
}

Matrix4f DH::get_trans() {
    return mat_from_dh(alpha_, a_, theta_, d_, offset_);

}

DH::~DH(void) {}
