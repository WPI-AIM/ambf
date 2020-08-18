#ifndef PSM_IK_H
#define PSM_IK_H

#include<ambf_client_app/psm_fk.h>
#include<ambf_client_app/utilities.h>

//THIS IS THE IK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
//SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
//MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. CHECK THE FK
//FILE TO FIND THE CORRECT DH PARAMS BASED ON THE FRAME ATTACHMENT IN THE DVRK MANUAL

//ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
//ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
//R_7_0 = [ 0,  1,  0 ]
//      = [ 1,  0,  0 ]
//      = [ 0,  0, -1 ]
//Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

//Read the frames, positions and rotation as follows, T_A_B, means that this
//is a Transfrom of frame A with respect to frame B. Similarly P_A_B is the
//Position Vector of frame A's origin with respect to frame B's origin. And finally
//R_A_B is the rotation matrix representing the orientation of frame B with respect to
//frame A.

//If you are confused by the above description, consider the equations below to make sense of it
//all. Suppose we have three frames A, B and C. The following equations will give you the L.H.S

//1) T_C_A = T_B_A * T_C_B
//2) R_A_C = inv(R_B_A * R_C_B)
//3) P_C_A = R_B_A * R_C_B * P_C

//For Postions, the missing second underscore separated quantity means that it is expressed in local
//coodinates. Rotations, and Transforms are always to defined a frame w.r.t to some
//other frame so this is a special case for only positions. Consider the example

//P_B indiciates a point expressed in B frame.

//Now there are two special cases that are identified by letter D and N. The first characeter D indiciates a
//difference (vector) of between two points, specified by the first and second underscore separater (_) strings,
//expressed in the third underscore separated reference. I.e.

//D_A_B_C
//This means the difference between Point A and  B expressed in C. On the other hand the letter N indicates
//the direction, and not specifically the actually
//measurement. So:

//N_A_B_C

//is the direction between the difference of A and B expressed in C.


class PSM_IK
{
public:
    PSM_IK();
    std::vector<float> compute_IK(Matrix4f T_7_0);
    ~PSM_IK(void);

private:
    const double palm_length_ = 0.0091; // Fixed length from the palm joint to the pinch joint
    const double pinch_length_ = 0.0102; // Fixed length from the pinch joint to the pinch tip
    const double tool_rcm_offset_ = 0.0156; // Delta between tool tip and the Remote Center of Motion


};

#endif // PSM_IK_H
