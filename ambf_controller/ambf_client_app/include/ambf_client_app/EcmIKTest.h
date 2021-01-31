#ifndef ECM_IK_TEXT_H
#define ECM_IK_TEXT_H

#include<ros/ros.h>
#include <ros/master.h>

#include "ambf_client/ambf_client.h"
#include "ambf_client_app/EcmFK.h"
#include "ambf_client_app/EcmIK.h"

class EcmIkTest
{
public:
    EcmIkTest();

//    std::vector<float> test_IK(const std::vector<float> joint_angles);
    void test_IK();
    void test_ambf_ecm(const std::vector<float> computed_q);
};

#endif // ECM_IK_TEXT_H
