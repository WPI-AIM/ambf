#include "ambf_client_app/ecm_ik.h"

ECM_IK::ECM_IK() {}

std::vector<float> ECM_IK::compute_IK(Matrix4f T_4_0) {
    Utilities Utilities;
//    Pinch Joint
    Matrix4f T_PinchJoint_4;
    T_PinchJoint_4 << Matrix4f::Identity();

//    float j1 = std::atan2(T_PalmJoint_0(0, 3), sign * T_PalmJoint_0(2, 3));
//    j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

    float j1 = std::atan2(T_4_0(0, 3), -1.0 * T_4_0(2, 3));
//    j1 = math.atan2(T_4_0.p[0], -T_4_0.p[2])


//    xz_diag = math.sqrt(T_4_0.p[0]**2 + T_4_0.p[2]**2)
//    j2 = -math.atan2(T_4_0.p[1], xz_diag)
    float xz_diag = std::sqrt(std::pow(T_4_0(0, 3), 2) + std::pow(T_4_0(2, 3), 2));
    float j2 = -1 * std::atan2(T_4_0(1, 3), xz_diag);

//    j3 = T_4_0.p.Norm() + (L_rcc - L_scopelen)

//    float insertion_depth = (T_PalmJoint_0.block<3, 1>(0, 3)).norm();

//    Eigen::Vector3f T_4_0_p = (T_4_0.block<3, 1>(0, 3));
//    std::cout << "T_4_0_p: " << T_4_0_p(0) << ", "  << T_4_0_p(1) << ", "  << T_4_0_p(2) << std::endl;

    float j3 = (T_4_0.block<3, 1>(0, 3)).norm() + (L_rcc_ - L_scopelen_);


//    T_4_0_FK = convert_mat_to_frame(compute_FK([j1, j2, j3, 0]))
//    R_IK_in_FK = T_4_0_FK.M.Inverse() * T_4_0.M

//    j4 = R_IK_in_FK.GetRPY()[2]

    ECM_FK ecm_fk;
    Matrix4f T_4_0_FK = ecm_fk.compute_FK(std::vector<float>{j1, j2, j3});

//    R_IK_in_FK = T_4_0_FK.M.Inverse() * T_4_0.M
    Matrix3f R_IK_in_FK = (T_4_0_FK.block<3, 3>(0, 0)).inverse() * (T_4_0.block<3, 3>(0, 0));

//    std::cout << "R_IK_in_FK: " << std::endl;
//    std::cout << R_IK_in_FK << std::endl;


//    Vector3f ea = R_IK_in_FK.eulerAngles(2, 1, 0);
//    Matrix3f n;
//    n = AngleAxisf(ea[0], Vector3f::UnitX())
//      * AngleAxisf(ea[1], Vector3f::UnitY())
//      * AngleAxisf(ea[2], Vector3f::UnitZ());

//    std::cout << "n: " << std::endl;
//    std::cout << n << std::endl;

//    double d = 3.14159265358979;
//    std::cout<<"roll: " <<d / (M_PI/std::atan2( R_IK_in_FK(2,1), R_IK_in_FK(2,2))) <<std::endl;
//    std::cout<<"pitch: " <<d / (M_PI/std::atan2( -R_IK_in_FK(2,0), std::pow( R_IK_in_FK(2,1)*R_IK_in_FK(2,1) +R_IK_in_FK(2,2)*R_IK_in_FK(2,2) ,0.5  )  )) <<std::endl;

//    float yaw;
//    yaw = M_PI / (M_PI/std::atan2( R_IK_in_FK(1,0),R_IK_in_FK(0,0) )) * 1.0;
//    yaw = std::atan2( R_IK_in_FK(1,0),R_IK_in_FK(0,0) );
//    std::cout<< "yaw: " << yaw  <<std::endl;;


    float j4 = Utilities.rpy_from_rotation(R_IK_in_FK)[2];
    return std::vector<float>{j1, j2, j3, j4};
}

ECM_IK::~ECM_IK(void) {}
