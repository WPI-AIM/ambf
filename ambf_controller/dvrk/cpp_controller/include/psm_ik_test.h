#ifndef PSM_H
#define PSM_H

#include "DH.h"
#include "Utilities.h"
#include "ambf_client/ambf_client.h"
#include<vector>
#include<cmath>
#include<iostream>

enum class JointType{
    ROTATIONAL,
    PRISMATIC,
};


//THIS IS THE FK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
//SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
//MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. BASED ON
//THE FRAME ATTACHMENT IN THE DVRK MANUAL THE CORRECT DH CAN FOUND IN THIS FILE

//ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
//ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
//R_7_0 = [ 0,  1,  0 ]
//      = [ 1,  0,  0 ]
//      = [ 0,  0, -1 ]
//Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

//You need to provide a list of joint positions. If the list is less that the number of joint
//i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided



class PSM
{
public:
    PSM();
    Matrix4f computeFK(std::vector<float> joint_pos);
    std::vector<float> computeIK(Matrix4f T_7_0);
    void testAmbfPsm();
    std::vector<float> testIK(const std::vector<float> joint_angles);

    void cleanup();
//    std::vector<std::vector<float>> getJointsLimit() { return PSM_JOINT_LIMITS_; }

    ~PSM(void);
private:
    std::vector<DH *> DH_Vector_;

    const float L_rcc_ = 0.4389;
    const float L_tool_ = 0.416;
    const float L_pitch2yaw_ = 0.009; //Fixed length from the palm joint to the pinch joint
    const float L_yaw2ctrlpnt_ = 0.0106; //Fixed length from the pinch joint to the pinch tip
    const float L_tool2rcm_offset_= 0.0229; //Delta between tool tip and the Remote Center of Motion

    float dh_params_[7][6] = {
   //     alpha,      a,             theta,  d,              offset,     joint_type
        { M_PI_2,     0.0,           0.0,    0.0,             M_PI_2,     0},
        { -M_PI_2,    0.0,           0.0,    0.0,            -M_PI_2,     0},
        { M_PI_2,     0.0,           0.0,    0.0,            -L_rcc_,     1},
        { 0.0,        0.0,           0.0,    L_tool_,            0.0,     0},
        { -M_PI_2,    0.0,           0.0,    0.0,            -M_PI_2,     0},
        { -M_PI_2,    L_pitch2yaw_,  0.0,    0.0,            -M_PI_2,     0},
        { -M_PI_2,    0.0,           0.0,    L_yaw2ctrlpnt_,  M_PI_2,     0}
    };

    std::vector<std::vector<float>> PSM_JOINT_LIMITS_ = {
        {  (float) (-91.96 * (M_PI / 180.0)),  (float) (91.96 * (M_PI / 180.0))},
        {  (float) (-60.00 * (M_PI / 180.0)),  (float) (60.00 * (M_PI / 180.0))},
        {                                0.0,                              0.24},
        { (float) (-175.00 * (M_PI / 180.0)), (float) (175.00 * (M_PI / 180.0))},
        { (float) ( -90.00 * (M_PI / 180.0)), (float) ( 90.00 * (M_PI / 180.0))},
        { (float) ( -85.00 * (M_PI / 180.0)), (float) ( 85.00 * (M_PI / 180.0))},
        {                                0.0,                               0.0}
    };
};


#endif // PSM_H
