#include "ambf_client_app/ik_test.h"



IK_test::IK_test() {}

std::vector<float> IK_test::test_IK(const std::vector<float> joint_angles) {
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


    psm_fk.~PSM_FK();
    psm_ik.~PSM_IK();

    return computed_q;
}

void IK_test::test_ambf_psm() {
    const std::vector<float> joint_angles = { -0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0 };
    test_IK(joint_angles);

}

int main(int argc, char* argv[])
{
//    Client client;
//    client.connect();

    IK_test ik_test;
    ik_test.test_ambf_psm();

    return 0;
}
