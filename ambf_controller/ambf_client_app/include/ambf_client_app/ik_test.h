#ifndef IK_TEST_H
#define IK_TEST_H

#include<ros/ros.h>
#include <ros/master.h>

#include "ambf_client/ambf_client.h"
#include "ambf_client_app/psm_fk.h"
#include "ambf_client_app/psm_ik.h"

class IK_test
{
public:
    IK_test();
    std::vector<float> test_IK(const std::vector<float> joint_angles);
    void test_ambf_psm(const std::vector<float> computed_q);

//    ~IK_test();
//private:


};

#endif // IK_TEST_H
