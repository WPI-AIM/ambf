#ifndef PSM_FK_H
#define PSM_FK_H
#define _GLIBCXX11_USE_C99_MATH

#include "ambf_client_app/dh.h"
#include<vector>
#include<cmath>


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



class PSM_FK
{
public:
    PSM_FK(std::vector<double> joint_angles) : joint_angles_(std::move(joint_angles)) {}
    Matrix4f compute_FK();


private:
    std::vector<DH *> DH_Vector_;
    const std::vector<double> joint_angles_;
    const double dh_params_[7][6] = {
   //     alpha,      a,      theta,                d,                  offset,     joint_type
        { M_PI_2,     0.0,    joint_angles_[0],     0.0,                M_PI_2,     0},
        { -M_PI_2,    0.0,    joint_angles_[1],     0.0,                -M_PI_2,    0},
        { M_PI_2,     0.0,    0.0,                  joint_angles_[2],   -0.4318,    1},
        { 0.0,        0.0,    joint_angles_[3],     0.4162,             0.0,        0},
        { -M_PI_2,    0.0,    joint_angles_[4],     0.0,                -M_PI_2,    0},
        { -M_PI_2,    0.0091, joint_angles_[5],     0.0,                -M_PI_2,    0},
        { -M_PI_2,    0.0,    0.0,                  0.0102,             M_PI_2,     0}
    };
};


#endif // PSM_FK_H
