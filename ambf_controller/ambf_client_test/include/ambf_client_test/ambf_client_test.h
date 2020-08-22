#ifndef AMBF_CLIENT_TEST_H
#define AMBF_CLIENT_TEST_H

#include <ambf_client/ambf_client.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <gtest/gtest.h>

class Ambf_Test: public ::testing::Test {
protected:
    Ambf_Test();
//    ~Ambf_Test(void);

    Client client_;
    rigidBodyPtr psm_baselink_handler_;
};

#endif // AMBF_CLIENT_TEST_H
