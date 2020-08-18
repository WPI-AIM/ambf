#include "ambf_client_app/psm_fk.h"

PSM_FK::PSM_FK() {}

const std::string joint_type_enum_to_str(JointType enumVal)
{
    if (enumVal == JointType::ROTATIONAL) return "ROTATIONAL";
    else if (enumVal == JointType::PRISMATIC) return "PRISMATIC";
}

Matrix4f PSM_FK::compute_FK(std::vector<float> joint_pos) {
    DH_Vector_.clear();

    int joint_pos_n = joint_pos.size();
    for(int i = 0; i < joint_pos_n; i++)
        dh_params_[i][2] = joint_pos[i];

    if(joint_pos_n > 2) {
        dh_params_[2][2] = 0.0;
        dh_params_[2][3] = joint_pos[2];
    }

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


    if(joint_pos_n == 1) return T_1_0;
    if(joint_pos_n == 2) return T_2_0;
    if(joint_pos_n == 3) return T_3_0;
    if(joint_pos_n == 4) return T_4_0;
    if(joint_pos_n == 5) return T_5_0;
    if(joint_pos_n == 6) return T_6_0;
    if(joint_pos_n == 7) return T_7_0;
}

void PSM_FK::cleanup() {
    for(DH *dh : DH_Vector_)
        dh->~DH();
}

PSM_FK::~PSM_FK(void){
    cleanup();
}
