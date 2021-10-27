#include "ecm_ik_test.h"

ECM::ECM(){}

const std::string joint_type_enum_to_str(JointType enumVal)
{
    if (enumVal == JointType::ROTATIONAL) return "ROTATIONAL";
    else if (enumVal == JointType::PRISMATIC) return "PRISMATIC";
}

Matrix4f ECM::computeFK(std::vector<float> joint_pos) {
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

    Matrix4f T_2_0 = T_1_0 * T_2_1;
    Matrix4f T_3_0 = T_2_0 * T_3_2;
    Matrix4f T_4_0 = T_3_0 * T_4_3;

    if(joint_pos_n == 1) return T_1_0;
    if(joint_pos_n == 2) return T_2_0;
    if(joint_pos_n == 3) return T_3_0;
    if(joint_pos_n == 4) return T_4_0;
}


std::vector<float> ECM::computeIK(Matrix4f T_4_0) {
    Utilities utilities;

    Matrix4f T_PinchJoint_4;
    T_PinchJoint_4 << Matrix4f::Identity();


    float j1 = std::atan2(T_4_0(0, 3), -1.0 * T_4_0(2, 3));

    float xz_diag = std::sqrt(std::pow(T_4_0(0, 3), 2) + std::pow(T_4_0(2, 3), 2));
    float j2 = -1 * std::atan2(T_4_0(1, 3), xz_diag);

    float j3 = (T_4_0.block<3, 1>(0, 3)).norm() + (L_rcc_ - L_scopelen_);

    Matrix4f T_4_0_FK = this->computeFK(std::vector<float>{j1, j2, j3});

    Matrix3f R_IK_in_FK = (T_4_0_FK.block<3, 3>(0, 0)).inverse() * (T_4_0.block<3, 3>(0, 0));

    float j4 = utilities.rpy_from_rotation(R_IK_in_FK)[2];
    return std::vector<float>{j1, j2, j3, j4};
}



void ECM::testIK() {
    Client client("ecm_ik_test");
    client.connect();
    usleep(20000);


    vector<string> object_names = client.getRigidBodyNames();

    std::cout << "object_names" <<std::endl;
    for(std::string object_name : object_names)
        std::cout << object_name << ", ";
    std::cout << std::endl;

    rigidBodyPtr b = client.getRigidBody("ecm/baselink", true);
    rigidBodyPtr target_fk_handler = client.getRigidBody("ecm/target_fk", true);
    rigidBodyPtr target_ik_handler = client.getRigidBody("ecm/target_ik", true);
    usleep(1000000);

    Utilities utilities;


    Vector3f P_0_w;
    P_0_w[0]= b->get_pos()[0];
    P_0_w[1]= b->get_pos()[1];
    P_0_w[2]= b->get_pos()[2];

    Eigen::Matrix3f R_0_w = utilities.rotation_from_euler(b->get_rpy()[0], b->get_rpy()[1], b->get_rpy()[2]);

    Eigen::Matrix4f T_0_w = utilities.get_frame(R_0_w, P_0_w);

    int n_poses = 20;
    for(int i = 0; i < n_poses; i++) {
        std::vector<float> desired_q;

        for(std::vector<float> joint_limit : ECM_JOINT_LIMITS_) {
            float low = joint_limit[0];
            float high = joint_limit[1];
            desired_q.emplace_back(utilities.get_random_between_range(low, high));
        }

        Matrix4f T_4_0 = this->computeFK(desired_q);


        if(target_ik_handler) {
            Eigen::Matrix4f T_4_w = T_0_w * T_4_0;
            target_ik_handler->set_pos(T_4_w(0, 3), T_4_w(1, 3), T_4_w(2, 3));

            Eigen::Vector3f r_4_w_rpy = utilities.rpy_from_rotation(T_4_w.block<3,3>(0,0));
            target_ik_handler->set_rpy(r_4_w_rpy[0], r_4_w_rpy[1], r_4_w_rpy[2]);
        }

        std::vector<float> computed_q = this->computeIK(T_4_0);


        if(target_fk_handler) {
            Eigen::Matrix4f T_4_0_fk = this->computeFK(computed_q);

            Eigen::Matrix4f T_4_w_fk = T_0_w * T_4_0_fk;
            target_fk_handler->set_pos(T_4_w_fk(0, 3), T_4_w_fk(1, 3), T_4_w_fk(2, 3));

            Eigen::Vector3f r_4_w_rpy_fk = utilities.rpy_from_rotation(T_4_w_fk.block<3,3>(0,0));
            target_fk_handler->set_rpy(r_4_w_rpy_fk[0], r_4_w_rpy_fk[1], r_4_w_rpy_fk[2]);
        }

        b->set_joint_pos<std::string>(              "baselink-yawlink", computed_q[0]);
        b->set_joint_pos<std::string>(         "yawlink-pitchbacklink", computed_q[1]);
        b->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
        b->set_joint_pos<std::string>(    "maininsertionlink-toollink", computed_q[3]);

        std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << std::endl;
        std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << std::endl;
        std::cout << "diff   : "
                  << std::roundf(desired_q[0] - computed_q[0]) << ", "
                  << std::roundf(desired_q[1] - computed_q[1]) << ", "
                  << std::roundf(desired_q[2] - computed_q[2]) << ", "
                  << std::roundf(desired_q[3] - computed_q[3]) << ", "
                  << std::endl;

        usleep(250000);
    }
}

void ECM::cleanup() {
    for(DH *dh : DH_Vector_)
        dh->~DH();
}

ECM::~ECM(void){
    cleanup();
}

int main(int argc, char* argv[])
{
    ECM ecm;
    ecm.testIK();

    return 0;
}
