#ifndef ECM_H
#define ECM_H

#include<vector>
#include<cmath>
#include<iostream>

#include "ambf_client/ambf_client.h"
#include "DH.h"
#include "Utilities.h"

enum class JointType{
    ROTATIONAL,
    PRISMATIC,
};


class ECM
{
public:
    ECM();
    Matrix4f computeFK(std::vector<float> joint_pos);
    std::vector<float> computeIK(Matrix4f T_4_0);
    void testIK();

    void cleanup();
    ~ECM(void);

private:
    std::vector<DH *> DH_Vector_;
    const float L_rcc_ = 0.3822;
    const float L_scopelen_ = 0.385495;


    float dh_params_[4][6] = {
   //     alpha,      a,      theta,                d,                  offset,     joint_type
        { M_PI_2,     0.0,    0.0,                  0.0,                M_PI_2,     0},
        { -M_PI_2,    0.0,    0.0,                  0.0,                -M_PI_2,    0},
        { M_PI_2,     0.0,    0.0,                  0.0,                -L_rcc_,    1},
        { 0.0,        0.0,    0.0,                  L_scopelen_,            0.0,    0}
    };

    std::vector<std::vector<float>> ECM_JOINT_LIMITS_ = {
        {  (float) (-91.96 * (M_PI / 180.0)),  (float) (91.96 * (M_PI / 180.0))},
        {  (float) (-60.00 * (M_PI / 180.0)),  (float) (60.00 * (M_PI / 180.0))},
        {                    0.0,   0.24               },
        { (float) (-175.00 * (M_PI / 180.0)), (float) (175.00 * (M_PI / 180.0))}
    };

};

#endif // ECM_H
