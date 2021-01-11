#include "ambf_client_app/ecm_ik_test.h"

EcmIkTest::EcmIkTest() { }

//std::vector<float> EcmIkTest::test_IK(const std::vector<float> joint_angles) {
void EcmIkTest::test_IK() {
    Client client;
    client.connect();
    usleep(20000);


    vector<string> object_names = client.getRigidBodyNames();

    std::cout << "object_names" <<std::endl;
    for(std::string object_name : object_names)
        std::cout << object_name << ", ";
    std::cout << std::endl;

    rigidBodyPtr b = client.getARigidBody("ecm/baselink", true);
    rigidBodyPtr target_fk_handler = client.getARigidBody("ecm/target_fk", true);
    rigidBodyPtr target_ik_handler = client.getARigidBody("ecm/target_ik", true);
    usleep(1000000);

    ECM_FK ecm_fk;
    ECM_IK ecm_ik;
    Utilities utilities;

    std::vector<std::vector<float>> ECM_JOINT_LIMITS = ecm_fk.getJointsLimit();


    Vector3f P_0_w;
    P_0_w[0]= b->get_pos()[0];
    P_0_w[1]= b->get_pos()[1];
    P_0_w[2]= b->get_pos()[2];

    Eigen::Matrix3f R_0_w = utilities.rotation_from_euler(b->get_rpy()[0], b->get_rpy()[1], b->get_rpy()[2]);

    Eigen::Matrix4f T_0_w = utilities.get_frame(R_0_w, P_0_w);

    int n_poses = 5;
    for(int i = 0; i < n_poses; i++) {
        std::vector<float> desired_q;

        for(std::vector<float> joint_limit : ECM_JOINT_LIMITS) {
            float low = joint_limit[0];
            float high = joint_limit[1];
            desired_q.emplace_back(utilities.get_random_between_range(low, high));
        }

        Matrix4f T_4_0 = ecm_fk.compute_FK(desired_q);


        if(target_ik_handler) {
            Eigen::Matrix4f T_4_w = T_0_w * T_4_0;
            target_ik_handler->set_pos(T_4_w(0, 3), T_4_w(1, 3), T_4_w(2, 3));

            Eigen::Vector3f r_4_w_rpy = utilities.rpy_from_rotation(T_4_w.block<3,3>(0,0));
            target_ik_handler->set_rpy(r_4_w_rpy[0], r_4_w_rpy[1], r_4_w_rpy[2]);
        }

        std::vector<float> computed_q = ecm_ik.compute_IK(T_4_0);


        if(target_fk_handler) {
            Eigen::Matrix4f T_4_0_fk = ecm_fk.compute_FK(computed_q);

            Eigen::Matrix4f T_4_w_fk = T_0_w * T_4_0_fk;
            target_fk_handler->set_pos(T_4_w_fk(0, 3), T_4_w_fk(1, 3), T_4_w_fk(2, 3));

            Eigen::Vector3f r_4_w_rpy_fk = utilities.rpy_from_rotation(T_4_w_fk.block<3,3>(0,0));
            target_fk_handler->set_rpy(r_4_w_rpy_fk[0], r_4_w_rpy_fk[1], r_4_w_rpy_fk[2]);
        }

        b->set_joint_pos<std::string>(              "baselink-yawlink", computed_q[0]);
        b->set_joint_pos<std::string>(         "yawlink-pitchbacklink", computed_q[1]);
        b->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
        b->set_joint_pos<std::string>(    "maininsertionlink-toollink", computed_q[3]);

//        std::vector<float> joint_angles_calculated = ecm_ik.compute_IK(T_4_0);

        std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << std::endl;
        std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << std::endl;
        std::cout << "diff   : "
                  << std::roundf(desired_q[0] - computed_q[0]) << ", "
                  << std::roundf(desired_q[1] - computed_q[1]) << ", "
                  << std::roundf(desired_q[2] - computed_q[2]) << ", "
                  << std::roundf(desired_q[3] - computed_q[3]) << ", "
                  << std::endl;

        usleep(1000000);
    }

    client.cleanUp();
}



int main(int argc, char* argv[])
{
    EcmIkTest ecm_ik_test;
    ecm_ik_test.test_IK();

    return 0;
}
