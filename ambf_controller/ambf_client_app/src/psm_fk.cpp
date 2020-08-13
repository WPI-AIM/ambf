#include "ambf_client_app/psm_fk.h"

const std::string joint_type_enum_to_str(JointType enumVal)
{
    if (enumVal == JointType::ROTATIONAL) return "ROTATIONAL";
    else if (enumVal == JointType::PRISMATIC) return "PRISMATIC";
}

Matrix4f PSM_FK::compute_FK() {
    int row_n = sizeof(dh_params_) / sizeof(dh_params_[0]);
    for(int row = 0; row < row_n; row++) {
        DH_Vector_.push_back(new DH(dh_params_[row][0], dh_params_[row][1], dh_params_[row][2], dh_params_[row][3], dh_params_[row][4], joint_type_enum_to_str((JointType)dh_params_[row][5])));
    }


    Matrix4f T_1_0 = DH_Vector_[0]->get_trans();
    Matrix4f T_2_1 = DH_Vector_[1]->get_trans();
    Matrix4f T_3_2 = DH_Vector_[2]->get_trans();
    Matrix4f T_4_3 = DH_Vector_[3]->get_trans();
    Matrix4f T_5_4 = DH_Vector_[4]->get_trans();
    Matrix4f T_6_5 = DH_Vector_[5]->get_trans();
    Matrix4f T_7_6 = DH_Vector_[6]->get_trans();

    Matrix4f T_2_0 = T_1_0 * T_2_1;
    Matrix4f T_3_0 = T_2_0 * T_3_2;
    Matrix4f T_4_0 = T_3_0 * T_4_3;
    Matrix4f T_5_0 = T_4_0 * T_5_4;
    Matrix4f T_6_0 = T_5_0 * T_6_5;
    Matrix4f T_7_0 = T_6_0 * T_7_6;

    return T_7_0;
}
