#include "ambf_client_app/psm_ik_test.h"
PSM_IK_test::PSM_IK_test() {}

std::vector<float> PSM_IK_test::test_IK_psm(const std::vector<float> joint_angles) {
//    We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
//    in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)

//    const std::vector<float> joint_angles = { -0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0 };

    PSM_FK psm_fk;

    Matrix4f T_7_0 = psm_fk.compute_FK(joint_angles);

    PSM_IK psm_ik;
    std::vector<float> computed_q = psm_ik.compute_IK(T_7_0);

    std::cout << "q_actual: ";
    for(float q : joint_angles)
        std::cout << q << ", ";

    std::cout << std::endl;

    std::cout << "q_computed: ";
    for(float q : computed_q)
        std::cout << q << ", ";

    std::cout << std::endl;



//    psm_fk.~PSM_FK();
//    psm_ik.~PSM_IK();

    return computed_q;
}

void PSM_IK_test::test_ambf_psm(const std::vector<float> computed_q) {
    Client client;
    client.connect();
    usleep(20000);
//    client.printSummary();

    vector<string> object_names = client.getRigidBodyNames();

    std::cout << "object_names" <<std::endl;
    for(std::string object_name : object_names)
        std::cout << object_name << ", ";
    std::cout << std::endl;

    string psm_baselink = "psm/baselink";
    rigidBodyPtr psm_baselink_handler = client.getARigidBody(psm_baselink, true);
    usleep(1000000);

    std::vector<std::string> base_children = psm_baselink_handler->get_children_names();
    psm_baselink_handler->set_joint_pos<std::string>("baselink-yawlink", computed_q[0]);
    psm_baselink_handler->set_joint_pos<std::string>("yawlink-pitchbacklink", computed_q[1]);
    psm_baselink_handler->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
    psm_baselink_handler->set_joint_pos<std::string>("maininsertionlink-toolrolllink", computed_q[3]);
    psm_baselink_handler->set_joint_pos<std::string>("toolrolllink-toolpitchlink", computed_q[4]);
    psm_baselink_handler->set_joint_pos<std::string>("toolpitchlink-toolgripper1link", computed_q[5]);
    psm_baselink_handler->set_joint_pos<std::string>("toolpitchlink-toolgripper2link", 0.0);


    psm_baselink_handler->set_joint_pos<int>(0, 0.0);


    for(string name : base_children) {
        cout << "name: " << name << "\n";
    }

    client.cleanUp();
}

int main(int argc, char* argv[])
{
    const std::vector<float> joint_angles = { -0.3, 0.2, 0.1, -0.9, 0.0, -1.2, 0.0 };
    PSM_IK_test psm_ik_test;
    const std::vector<float> computed_q = psm_ik_test.test_IK_psm(joint_angles);
//    psm_ik_test.test_ambf_psm(computed_q);
//    psm_ik_test.test_IK_psm(joint_angles);

    return 0;
}