#include"ambf_client_test/ambf_client_test.h"

Ambf_Test::Ambf_Test() {
    client_.connect();

//    client_.printSummary();
    string psm_baselink = "psm/baselink";
    cout << "psm_baselink: " << psm_baselink << "\n";
    psm_baselink_handler_ = client_.getARigidBody(psm_baselink, true);
    usleep(1000000);

}

//Ambf_Test::~Ambf_Test(void){
//    client_.~Client();
//}

TEST_F(Ambf_Test, TestPSMBaselink) {
    EXPECT_FLOAT_EQ(psm_baselink_handler_->get_joint_pos(0), -1.605f);
//    EXPECT_FLOAT_EQ(psm_baselink_handler_->get_joint_pos(1), -1.605f);
//    EXPECT_FLOAT_EQ(route_planner.CalculateHValue(mid_node), 0.58903033);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
