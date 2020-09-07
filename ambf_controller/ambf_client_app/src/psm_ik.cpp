#include "ambf_client_app/psm_ik.h"

PSM_IK::PSM_IK() {}

std::vector<float> PSM_IK::compute_IK(Matrix4f T_7_0) {

//    Pinch Joint
    Matrix4f T_PinchJoint_7;
    T_PinchJoint_7 << Matrix4f::Identity();
    T_PinchJoint_7(2,3) = -pinch_length_;

//    Pinch Joint in Origin
    Matrix4f T_PinchJoint_0 = T_7_0 * T_PinchJoint_7;
//    It appears from the geometry of the robot, that the palm joint is always in the ZY
//    plane of the end effector frame (7th Frame)
//    This is the logic that should give us the direction of the palm link and then
//    we know the length of the palm link so we can keep going back to find the shaftTip (PalmJoint)
//    position

//    Convert the vector from base to pinch joint in the pinch joint frame
    Vector3f P_PinchJoint_local;
    P_PinchJoint_local = (T_PinchJoint_0.block<3, 3>(0, 0)).inverse() *  T_PinchJoint_0.block<3, 1>(0, 3);


    VectorXf N_PalmJoint_PinchJoint(3);
    N_PalmJoint_PinchJoint = P_PinchJoint_local;
    N_PalmJoint_PinchJoint(0) = 0.0;
    N_PalmJoint_PinchJoint.normalize();

    Utilities utilities;
    float angle = utilities.get_angle(N_PalmJoint_PinchJoint, Vector3f(0, 0, -1),
                        Vector3f(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()));

    //If the angle between the two vectors is > 90 Degree, we should move in the opposite direction
    if(angle > M_PI_2)
        N_PalmJoint_PinchJoint = -N_PalmJoint_PinchJoint;

    N_PalmJoint_PinchJoint *= palm_length_;

    Matrix4f T_PalmJoint_PinchJoint;
    T_PalmJoint_PinchJoint << Matrix4f::Identity();
    T_PalmJoint_PinchJoint.block<3, 1>(0, 3) = N_PalmJoint_PinchJoint;

//    Get the shaft tip or the Palm's Joint position
    Matrix4f T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint;

// Calculate insertion_depth to check if the tool is past the RCM
   float insertion_depth = (T_PalmJoint_0.block<3, 1>(0, 3)).norm();

   int sign = 0;
   if(insertion_depth <= tool_rcm_offset_) sign = 1;
   else if(insertion_depth > tool_rcm_offset_) sign = -1;


//    Now having the end point of the shaft or the PalmJoint, we can calculate some
//    angles as follows

    float j1 = std::atan2(T_PalmJoint_0(0, 3), sign * T_PalmJoint_0(2, 3));
    float j2 = -std::atan2(T_PalmJoint_0(1, 3), sign * T_PalmJoint_0(2, 3));
    float j3 = insertion_depth + tool_rcm_offset_;

//    Calculate j4
//    This is an important case and has to be dealt carefully. Based on some inspection, we can find that
//    we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
//    the only two vectors that are orthogonal at all configurations of the EE.
    Vector3f cross_palmlink_x7_0 = (T_7_0.block<3, 1>(0, 0)).cross(T_PinchJoint_0.block<3, 1>(0, 3) - T_PalmJoint_0.block<3, 1>(0, 3));

    PSM_FK psm_fk;
    Matrix4f T_3_0 = psm_fk.compute_FK(std::vector<float>{j1, j2, j3});
    float j4 = utilities.get_angle(cross_palmlink_x7_0, T_3_0.block<3, 1>(0, 1), -T_3_0.block<3, 1>(0, 2));

    //Calculate j5
    //This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
    Matrix4f T_4_0 = psm_fk.compute_FK(std::vector<float>{j1, j2, j3, j4});
    float j5 = utilities.get_angle(T_PinchJoint_0.block<3, 1>(0, 3), T_4_0.block<3, 1>(0, 2), -T_4_0.block<3, 1>(0, 1));

//    Calculate j6
//    This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
    Matrix4f T_5_0 = psm_fk.compute_FK(std::vector<float>{j1, j2, j3, j4, j5});
    float j6 = utilities.get_angle(T_7_0.block<3, 1>(0, 2), T_5_0.block<3, 1>(0, 0), -T_5_0.block<3, 1>(0, 1));

    utilities.~Utilities();
//    psm_fk.~PSM_FK();
    return std::vector<float>{j1, j2, j3, j4, j5, j6};
}

PSM_IK::~PSM_IK(void) {}
