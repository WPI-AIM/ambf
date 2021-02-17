#include "psm_ik_test.h"

PSM::PSM() {}

const std::string joint_type_enum_to_str(JointType enumVal)
{
    if (enumVal == JointType::ROTATIONAL) return "ROTATIONAL";
    else if (enumVal == JointType::PRISMATIC) return "PRISMATIC";
}

Matrix4f PSM::computeFK(std::vector<float> joint_pos) {
    DH_Vector_.clear();

    int joint_pos_n = joint_pos.size();
    for(int i = 0; i < joint_pos_n; i++)
        dh_params_[i][2] = joint_pos[i];

    if(joint_pos_n > 2) {
        dh_params_[2][2] = 0.0;
        dh_params_[2][3] = joint_pos[2];
    }

    int row_n = sizeof(dh_params_) / sizeof(dh_params_[0]);
    for(int row = 0; row < row_n; row++) {
        DH_Vector_.push_back(new DH(dh_params_[row][0], dh_params_[row][1], dh_params_[row][2], dh_params_[row][3], dh_params_[row][4], joint_type_enum_to_str((JointType)dh_params_[row][5])));
    }

    Matrix4f T_1_0 = DH_Vector_[0]->get_trans();
    Matrix4f T_2_1 = DH_Vector_[1]->get_trans();
    Matrix4f T_3_2 = DH_Vector_[2]->get_trans();
    Matrix4f T_4_3 = DH_Vector_[3]->get_trans();
    Matrix4f T_5_4 = DH_Vector_[4]->get_trans();
    Matrix4f T_6_5 = DH_Vector_[5]->get_trans();
    Matrix4f T_7_6 = DH_Vector_[6]->get_trans();

    Matrix4f T_2_0 = T_1_0 * T_2_1;
    Matrix4f T_3_0 = T_2_0 * T_3_2;
    Matrix4f T_4_0 = T_3_0 * T_4_3;
    Matrix4f T_5_0 = T_4_0 * T_5_4;
    Matrix4f T_6_0 = T_5_0 * T_6_5;
    Matrix4f T_7_0 = T_6_0 * T_7_6;


    if(joint_pos_n == 1) return T_1_0;
    if(joint_pos_n == 2) return T_2_0;
    if(joint_pos_n == 3) return T_3_0;
    if(joint_pos_n == 4) return T_4_0;
    if(joint_pos_n == 5) return T_5_0;
    if(joint_pos_n == 6) return T_6_0;
    if(joint_pos_n == 7) return T_7_0;
}



std::vector<float> PSM::computeIK(Matrix4f T_7_0) {
    Utilities utilities;

    // Pinch Joint
    Eigen::Matrix3f R_PinchJoint_7 = utilities.rotation_from_euler(0.0, 0.0, 0.0);
    Vector3f P_PinchJoint_7(0.0, 0.0, -1.0);
    Eigen::Matrix4f T_PinchJoint_7 = utilities.get_frame(R_PinchJoint_7, P_PinchJoint_7 * L_yaw2ctrlpnt_);


    //Pinch Joint in Origin
    Eigen::Matrix4f T_PinchJoint_0 = T_7_0 * T_PinchJoint_7;

    //It appears from the geometry of the robot, that the palm joint is always in the ZY
    //plane of the end effector frame (7th Frame)
    //This is the logic that should give us the direction of the palm link and then
    //we know the length of the palm link so we can keep going back to find the shaftTip (PalmJoint)
    //position

    //Convert the vector from base to pinch joint in the pinch joint frame
    Vector3f P_PinchJoint_local = (T_PinchJoint_0.block<3, 3>(0, 0)).inverse() *  T_PinchJoint_0.block<3, 1>(0, 3);

    //Now we can trim the value along the x axis to get a projection along the YZ plane as mentioned above
    Vector3f N_PalmJoint_PinchJoint(3);
    N_PalmJoint_PinchJoint = -P_PinchJoint_local;
    N_PalmJoint_PinchJoint(0) = 0.0;
    N_PalmJoint_PinchJoint.normalize();

    Eigen::Matrix4f T_PalmJoint_PinchJoint = utilities.get_frame(utilities.rotation_from_euler(0.0, 0.0, 0.0), N_PalmJoint_PinchJoint * L_pitch2yaw_);

    //Get the shaft tip or the Palm's Joint position
    Eigen::Matrix4f T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint;

    // Calculate insertion_depth to check if the tool is past the RCM
    float insertion_depth = (T_PalmJoint_0.block<3, 1>(0, 3)).norm();

    //Now having the end point of the shaft or the PalmJoint, we can calculate some
    //angles as follows
    float xz_diagonal = std::sqrt(std::pow(T_PalmJoint_0(0, 3), 2) + std::pow(T_PalmJoint_0(2, 3), 2));
    float yz_diagonal = std::sqrt(std::pow(T_PalmJoint_0(1, 3), 2) + std::pow(T_PalmJoint_0(2, 3), 2));

    float j1 = std::atan2(T_PalmJoint_0(0, 3), -T_PalmJoint_0(2, 3));
    float j2 = -std::atan2(T_PalmJoint_0(1, 3), xz_diagonal);
    float j3 = insertion_depth + L_tool2rcm_offset_;

    // Calculate j4
    // This is an important case and has to be dealt carefully. Based on some inspection, we can find that
    // we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
    // the only two vectors that are orthogonal at all configurations of the EE.
    Vector3f cross_palmlink_x7_0 = (T_7_0.block<3, 1>(0, 0)).cross(T_PinchJoint_0.block<3, 1>(0, 3) - T_PalmJoint_0.block<3, 1>(0, 3));

    Matrix4f T_3_0 = this->computeFK(std::vector<float>{j1, j2, j3});
    float j4 = utilities.get_angle(cross_palmlink_x7_0, T_3_0.block<3, 1>(0, 1), -T_3_0.block<3, 1>(0, 2));

    //Calculate j5
    //This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
    Matrix4f T_4_0 = this->computeFK(std::vector<float>{j1, j2, j3, j4});
    float j5 = utilities.get_angle(T_PinchJoint_0.block<3, 1>(0, 3), T_4_0.block<3, 1>(0, 2), -T_4_0.block<3, 1>(0, 1));

    // Calculate j6
    // This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
    Matrix4f T_5_0 = this->computeFK(std::vector<float>{j1, j2, j3, j4, j5});
    float j6 = utilities.get_angle(T_7_0.block<3, 1>(0, 2), T_5_0.block<3, 1>(0, 0), -T_5_0.block<3, 1>(0, 1));

    return std::vector<float>{j1, j2, j3, j4, j5, j6};
}


std::vector<float> PSM::testIK(const std::vector<float> desired_q) {
//    We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
//    in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)

    Matrix4f T_7_0 = this->computeFK(desired_q);


    std::vector<float> computed_q = this->computeIK(T_7_0);


    std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << ", " <<  desired_q[4] << ", " <<  desired_q[5] << ", " <<  desired_q[6] << ", " <<  desired_q[7] << std::endl;
    std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << ", " << computed_q[4] << ", " << computed_q[5] << ", " << computed_q[6] << ", " << computed_q[7] << std::endl;
    std::cout << "diff   : "
              << std::roundf(desired_q[0] - computed_q[0]) << ", "
              << std::roundf(desired_q[1] - computed_q[1]) << ", "
              << std::roundf(desired_q[2] - computed_q[2]) << ", "
              << std::roundf(desired_q[3] - computed_q[3]) << ", "
              << std::roundf(desired_q[4] - computed_q[4]) << ", "
              << std::roundf(desired_q[5] - computed_q[5]) << ", "
              << std::roundf(desired_q[6] - computed_q[6]) << ", "
              << std::roundf(desired_q[7] - computed_q[7]) << ", "
              << std::endl;

    return computed_q;
}

//This have some bugs, needs further refining.
void PSM::testAmbfPsm() {
    Client client("psm_ik_test");
    client.connect();
    usleep(20000);

    vector<string> object_names = client.getRigidBodyNames();

    std::cout << "object_names" <<std::endl;
    for(std::string object_name : object_names)
        std::cout << object_name << ", ";
    std::cout << std::endl;


    rigidBodyPtr b = client.getRigidBody("psm/baselink", true);
    rigidBodyPtr target_fk_handler = client.getRigidBody("psm/target_fk", true);
    rigidBodyPtr target_ik_handler = client.getRigidBody("psm/target_ik", true);
    usleep(1000000);

    Utilities utilities;

    Vector3f P_0_w;
    P_0_w[0]= b->get_pos()[0];
    P_0_w[1]= b->get_pos()[1];
    P_0_w[2]= b->get_pos()[2];

    Eigen::Matrix3f R_0_w = utilities.rotation_from_euler(b->get_rpy()[0], b->get_rpy()[1], b->get_rpy()[2]);

    Eigen::Matrix4f T_0_w = utilities.get_frame(R_0_w, P_0_w);

    int n_poses = 5;
    for(int i = 0; i < n_poses; i++) {
        std::vector<float> desired_q;

        for(std::vector<float> joint_limit : PSM_JOINT_LIMITS_) {
            float low = joint_limit[0];
            float high = joint_limit[1];
            desired_q.emplace_back(utilities.get_random_between_range(low, high));
        }
        Matrix4f T_7_0 = this->computeFK(desired_q);


        if(target_ik_handler) {
            Eigen::Matrix4f T_7_w = T_0_w * T_7_0;
            target_ik_handler->set_pos(T_7_w(0, 3), T_7_w(1, 3), T_7_w(2, 3));

            Eigen::Vector3f r_7_w_rpy = utilities.rpy_from_rotation(T_7_w.block<3,3>(0,0));
            target_ik_handler->set_rpy(r_7_w_rpy[0], r_7_w_rpy[1], r_7_w_rpy[2]);
        }
        std::vector<float> computed_q = this->computeIK(T_7_0);

        if(target_fk_handler) {
            Eigen::Matrix4f T_7_0_fk = this->computeFK(computed_q);

            Eigen::Matrix4f T_7_w_fk = T_0_w * T_7_0_fk;
            target_fk_handler->set_pos(T_7_w_fk(0, 3), T_7_w_fk(1, 3), T_7_w_fk(2, 3));

            Eigen::Vector3f r_7_w_rpy_fk = utilities.rpy_from_rotation(T_7_w_fk.block<3,3>(0,0));
            target_fk_handler->set_rpy(r_7_w_rpy_fk[0], r_7_w_rpy_fk[1], r_7_w_rpy_fk[2]);
        }

        b->set_joint_pos<std::string>("baselink-yawlink", computed_q[0]);
        b->set_joint_pos<std::string>("yawlink-pitchbacklink", computed_q[1]);
        b->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
        b->set_joint_pos<std::string>("maininsertionlink-toolrolllink", computed_q[3]);
        b->set_joint_pos<std::string>("toolrolllink-toolpitchlink", computed_q[4]);
        b->set_joint_pos<std::string>("toolpitchlink-toolgripper1link", computed_q[5]);
        b->set_joint_pos<std::string>("toolpitchlink-toolgripper2link", 0.0);

        std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << ", " <<  desired_q[4] << ", " <<  desired_q[5] << ", " <<  desired_q[6] << ", " <<  desired_q[7] << std::endl;
        std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << ", " << computed_q[4] << ", " << computed_q[5] << ", " << computed_q[6] << ", " << computed_q[7] << std::endl;
        std::cout << "diff   : "
                  << std::roundf(desired_q[0] - computed_q[0]) << ", "
                  << std::roundf(desired_q[1] - computed_q[1]) << ", "
                  << std::roundf(desired_q[2] - computed_q[2]) << ", "
                  << std::roundf(desired_q[3] - computed_q[3]) << ", "
                  << std::roundf(desired_q[4] - computed_q[4]) << ", "
                  << std::roundf(desired_q[5] - computed_q[5]) << ", "
                  << std::roundf(desired_q[6] - computed_q[6]) << ", "
                  << std::roundf(desired_q[7] - computed_q[7]) << ", "
                  << std::endl;
        usleep(500000);
    }

    client.cleanUp();
}

void PSM::cleanup() {
    for(DH *dh : DH_Vector_)
        dh->~DH();
}

PSM::~PSM(void){
    cleanup();
}

int main(int argc, char* argv[])
{
    const std::vector<float> joint_angles = {-0.0516, -0.729, 0.0244, 2.66425039, -1.09197187, 0.46506824, 0};
    PSM psm;
//    psm.testIK(joint_angles);
    psm.testAmbfPsm();

    return 0;
}
