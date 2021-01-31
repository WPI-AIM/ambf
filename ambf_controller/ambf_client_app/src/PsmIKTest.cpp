#include "ambf_client_app/PsmIKTest.h"
PSM_IK_test::PSM_IK_test() {}

std::vector<float> PSM_IK_test::test_IK_psm(const std::vector<float> desired_q) {
//    We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
//    in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)

    PSM_FK psm_fk;

    Matrix4f T_7_0 = psm_fk.compute_FK(desired_q);

    PSM_IK psm_ik;
    std::vector<float> computed_q = psm_ik.compute_IK(T_7_0);

//    std::cout << "desired_q: ";
//    for(float q : joint_angles)
//        std::cout << q << ", ";

//    std::cout << std::endl;

//    std::cout << "q_computed: ";
//    for(float q : computed_q)
//        std::cout << q << ", ";

//    std::cout << std::endl;

    std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << ", " <<  desired_q[4] << ", " <<  desired_q[5] << ", " <<  desired_q[6] << ", " <<  desired_q[7] << std::endl;
    std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << ", " << computed_q[4] << ", " << computed_q[5] << ", " << computed_q[6] << ", " << computed_q[7] << std::endl;
    std::cout << "diff   : "
              << std::roundf(desired_q[0] - computed_q[0]) << ", "
              << std::roundf(desired_q[1] - computed_q[1]) << ", "
              << std::roundf(desired_q[2] - computed_q[2]) << ", "
              << std::roundf(desired_q[3] - computed_q[3]) << ", "
              << std::roundf(desired_q[4] - computed_q[4]) << ", "
              << std::roundf(desired_q[5] - computed_q[5]) << ", "
              << std::roundf(desired_q[6] - computed_q[6]) << ", "
              << std::roundf(desired_q[7] - computed_q[7]) << ", "
              << std::endl;

//    psm_fk.~PSM_FK();
//    psm_ik.~PSM_IK();

    return computed_q;
}

void PSM_IK_test::test_ambf_psm() {
//    Client client;
//    client.connect();
//    usleep(20000);

//    vector<string> object_names = client.getRigidBodyNames();

//    std::cout << "object_names" <<std::endl;
//    for(std::string object_name : object_names)
//        std::cout << object_name << ", ";
//    std::cout << std::endl;


//    rigidBodyPtr b = client.getARigidBody("psm/baselink", true);
//    rigidBodyPtr target_fk_handler = client.getARigidBody("psm/target_fk", true);
//    rigidBodyPtr target_ik_handler = client.getARigidBody("psm/target_ik", true);
//    usleep(1000000);

    PSM_FK psm_fk;
    PSM_IK psm_ik;
    Utilities utilities;

    std::vector<std::vector<float>> PSM_JOINT_LIMITS = psm_fk.getJointsLimit();


//    Vector3f P_0_w;
//    P_0_w[0]= b->get_pos()[0];
//    P_0_w[1]= b->get_pos()[1];
//    P_0_w[2]= b->get_pos()[2];

//    Eigen::Matrix3f R_0_w = utilities.rotation_from_euler(b->get_rpy()[0], b->get_rpy()[1], b->get_rpy()[2]);

//    Eigen::Matrix4f T_0_w = utilities.get_frame(R_0_w, P_0_w);

    int n_poses = 5;
    for(int i = 0; i < n_poses; i++) {
        std::vector<float> desired_q;

        for(std::vector<float> joint_limit : PSM_JOINT_LIMITS) {
            float low = joint_limit[0];
            float high = joint_limit[1];
            desired_q.emplace_back(utilities.get_random_between_range(low, high));
        }
        Matrix4f T_7_0 = psm_fk.compute_FK(desired_q);


//        if(target_ik_handler) {
//            Eigen::Matrix4f T_7_w = T_0_w * T_7_0;
//            target_ik_handler->set_pos(T_7_w(0, 3), T_7_w(1, 3), T_7_w(2, 3));

//            Eigen::Vector3f r_7_w_rpy = utilities.rpy_from_rotation(T_7_w.block<3,3>(0,0));
//            target_ik_handler->set_rpy(r_7_w_rpy[0], r_7_w_rpy[1], r_7_w_rpy[2]);
//        }
        std::vector<float> computed_q = psm_ik.compute_IK(T_7_0);

//        if(target_fk_handler) {
//            Eigen::Matrix4f T_7_0_fk = psm_fk.compute_FK(computed_q);

//            Eigen::Matrix4f T_7_w_fk = T_0_w * T_7_0_fk;
//            target_fk_handler->set_pos(T_7_w_fk(0, 3), T_7_w_fk(1, 3), T_7_w_fk(2, 3));

//            Eigen::Vector3f r_7_w_rpy_fk = utilities.rpy_from_rotation(T_7_w_fk.block<3,3>(0,0));
//            target_fk_handler->set_rpy(r_7_w_rpy_fk[0], r_7_w_rpy_fk[1], r_7_w_rpy_fk[2]);
//        }

//        b->set_joint_pos<std::string>("baselink-yawlink", computed_q[0]);
//        b->set_joint_pos<std::string>("yawlink-pitchbacklink", computed_q[1]);
//        b->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
//        b->set_joint_pos<std::string>("maininsertionlink-toolrolllink", computed_q[3]);
//        b->set_joint_pos<std::string>("toolrolllink-toolpitchlink", computed_q[4]);
//        b->set_joint_pos<std::string>("toolpitchlink-toolgripper1link", computed_q[5]);
//        b->set_joint_pos<std::string>("toolpitchlink-toolgripper2link", 0.0);

        std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << ", " <<  desired_q[4] << ", " <<  desired_q[5] << ", " <<  desired_q[6] << ", " <<  desired_q[7] << std::endl;
        std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << ", " << computed_q[4] << ", " << computed_q[5] << ", " << computed_q[6] << ", " << computed_q[7] << std::endl;
        std::cout << "diff   : "
                  << std::roundf(desired_q[0] - computed_q[0]) << ", "
                  << std::roundf(desired_q[1] - computed_q[1]) << ", "
                  << std::roundf(desired_q[2] - computed_q[2]) << ", "
                  << std::roundf(desired_q[3] - computed_q[3]) << ", "
                  << std::roundf(desired_q[4] - computed_q[4]) << ", "
                  << std::roundf(desired_q[5] - computed_q[5]) << ", "
                  << std::roundf(desired_q[6] - computed_q[6]) << ", "
                  << std::roundf(desired_q[7] - computed_q[7]) << ", "
                  << std::endl;
        usleep(1000000);
    }

//    client.cleanUp();
}

int main(int argc, char* argv[])
{
//    const std::vector<float> joint_angles = { -0.0946, 0.4866, 0.1653, -0.19296815, 0.43227411, 0.21431145, 0 };
//    const std::vector<float> joint_angles = {-1.3008, 0.7967, 0.1589, -2.32745577, -0.37737629, -1.08002556, 0};
    const std::vector<float> joint_angles = {-0.0516, -0.729, 0.0244, 2.66425039, -1.09197187, 0.46506824, 0};
    PSM_IK_test psm_ik_test;
    psm_ik_test.test_IK_psm(joint_angles);
//    psm_ik_test.test_ambf_psm();

    return 0;
}
