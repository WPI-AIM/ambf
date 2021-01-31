#include "ambf_client_app/EcmIK.h"

ECM_IK::ECM_IK() {}

std::vector<float> ECM_IK::compute_IK(Matrix4f T_4_0) {
    Utilities Utilities;

    Matrix4f T_PinchJoint_4;
    T_PinchJoint_4 << Matrix4f::Identity();


    float j1 = std::atan2(T_4_0(0, 3), -1.0 * T_4_0(2, 3));

    float xz_diag = std::sqrt(std::pow(T_4_0(0, 3), 2) + std::pow(T_4_0(2, 3), 2));
    float j2 = -1 * std::atan2(T_4_0(1, 3), xz_diag);


    float j3 = (T_4_0.block<3, 1>(0, 3)).norm() + (L_rcc_ - L_scopelen_);


    ECM_FK ecm_fk;
    Matrix4f T_4_0_FK = ecm_fk.compute_FK(std::vector<float>{j1, j2, j3});

    Matrix3f R_IK_in_FK = (T_4_0_FK.block<3, 3>(0, 0)).inverse() * (T_4_0.block<3, 3>(0, 0));


    float j4 = Utilities.rpy_from_rotation(R_IK_in_FK)[2];
    return std::vector<float>{j1, j2, j3, j4};
}

ECM_IK::~ECM_IK(void) {}
