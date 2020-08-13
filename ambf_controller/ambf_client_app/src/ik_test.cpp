#include "ambf_client_app/ik_test.h"

#include "ambf_client/ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

IK_test::IK_test() {}

void IK_test::test_IK() {
//    We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
//    in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)

    const std::vector<double> joint_angles = { -0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0 };

    PSM_FK psm_fk(joint_angles);

    Matrix4f T_7_0 = psm_fk.compute_FK();

    std::cout << "T_7_0: " << std::endl << T_7_0 << std::endl;
}

int main(int argc, char* argv[])
{
//    Client client;
//    client.connect();

    IK_test ik_test;
    ik_test.test_IK();

    return 0;
}
