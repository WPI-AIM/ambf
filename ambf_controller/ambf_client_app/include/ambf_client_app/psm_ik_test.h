#ifndef IK_TEST_H
#define IK_TEST_H

#include<ros/ros.h>
#include <ros/master.h>

#include "ambf_client/ambf_client.h"
#include "ambf_client_app/psm_fk.h"
#include "ambf_client_app/psm_ik.h"

class PSM_IK_test
{
public:
    PSM_IK_test();
    std::vector<float> test_IK_psm(const std::vector<float> joint_angles);
    void test_ambf_psm(const std::vector<float> computed_q);

//    ~IK_test();
private:
    const float L_rcc_ = 0.4389;
    const float L_tool_ = 0.416;
    const float L_pitch2yaw_ = 0.009;
    const float L_yaw2ctrlpnt_ = 0.0106;

};

#endif // IK_TEST_H
