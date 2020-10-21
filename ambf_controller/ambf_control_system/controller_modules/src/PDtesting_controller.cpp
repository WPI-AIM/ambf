#include "gtest/gtest.h"
#include "controller_modules/PDController.h"
#include "Eigen/Core"



class Controller_Test: public ::testing::Test 
{
    protected:
        Controller_Test();
    

};

Controller_Test::Controller_Test()
{
    std::cout<<"hello";
}



TEST_F(Controller_Test, TestSetGain )
{
    Eigen::MatrixXd Kp(2,2);
    Kp(0,0) = 5;
    Kp(1,0) = 10;
    Kp(0,1) = 10;
    Kp(1,1) = 16;
    Eigen::MatrixXd Kd(2,2);
    Kd(0,0) = 7;
    Kd(1,0) = 6;
    Kd(0,1) = 6;
    Kd(1,1) = 7;

    Eigen::MatrixXd Kp2(3,3);
    Kp(0,0) = 5;
    Kp(1,0) = 10;
    Kp(0,1) = 10;
    Kp(1,1) = 16;
    Eigen::MatrixXd Kd2(3,3);
    Kd(0,0) = 7;
    Kd(1,0) = 6;
    Kd(0,1) = 6;
    Kd(1,1) = 7;

    PDController controller(Kp,Kd);

    EXPECT_FALSE(controller.setKd(Kd2));
    EXPECT_FALSE(controller.setKp(Kp2));

    EXPECT_TRUE(controller.setKd(Kd));
    EXPECT_TRUE(controller.setKp(Kp));
}

TEST_F(Controller_Test, TestTauMath )
{
    Eigen::MatrixXd Kp(2,2);
    Kp(0,0) = 5;
    Kp(1,0) = 10;
    Kp(0,1) = 10;
    Kp(1,1) = 16;
    Eigen::MatrixXd Kd(2,2);
    Kd(0,0) = 7;
    Kd(1,0) = 6;
    Kd(0,1) = 6;
    Kd(1,1) = 7;
    PDController controller(Kp,Kd);
    Eigen::VectorXd e(2),ed(2),tau(2);
    e(0) = 5;
    e(1) = 5.2;
    ed(0) = -6.5;
    ed(1) = 3.2;
    controller.calc_tau(e,ed,tau);

    EXPECT_FLOAT_EQ(tau(0), 50.7f);
    EXPECT_FLOAT_EQ(tau(1), 116.6f);

}




//TEST_F(Controller_Test, TestTauSize )
//{
//    Eigen::MatrixXd Kp(2,2);
//    Kp(0,0) = 5;
//    Kp(1,0) = 10;
//    Kp(0,1) = 10;
//    Kp(1,1) = 16;
//    Eigen::MatrixXd Kd(2,2);
//    Kd(0,0) = 7;
//    Kd(1,0) = 6;
//    Kd(0,1) = 6;
//    Kd(1,1) = 7;
//    PDController controller(Kp,Kd);
//    Eigen::VectorXd e(3),ed(2),tau(2);
//    e(0) = 5;
//    e(1) = 5.2;
//    e(2) = 5.5;
//    ed(0) = -6.5;
//    ed(1) = 3.2;
//    controller.calculate(e,ed,tau);
//    std::cout<<tau(0)<<'\n';
//    std::cout<<tau(1)<<'\n';
//    std::cout<<tau(2)<<'\n';
//    EXPECT_FLOAT_EQ(tau(0), 0.0f);
//    EXPECT_FLOAT_EQ(tau(1), 0.0f);

//}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// #include "gtest/gtest.h"
// #include <math.h>  


// class Controller_Test: public ::testing::Test {
// protected:
//     Controller_Test();
//     ~Controller_Test(void);


// };




// int main(int argc, char **argv) {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
