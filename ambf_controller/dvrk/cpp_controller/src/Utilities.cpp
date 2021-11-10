#include "Utilities.h"

Utilities::Utilities() {}

float Utilities::get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector) {
    float angle = 0.0;
    vec_a.normalize();
    vec_b.normalize();

    Vector3f cross_ab = vec_a.cross(vec_b);
    float vdot = vec_a.dot(vec_b);

//    # Check if the vectors are in the same direction
    if(1.0 - vdot < 0.000001) angle = 0.0;
    else if (1.0 + vdot < 0.000001) angle = M_PI;
    else angle = acos(vdot);

    if(up_vector(0) != std::numeric_limits<float>::min() &&
       up_vector(1) != std::numeric_limits<float>::min() &&
       up_vector(2) != std::numeric_limits<float>::min()) {
        float same_dir = cross_ab.dot(up_vector);

        if(same_dir < 0.0) angle = -angle;
    }

    return angle;
}

float Utilities::get_random_between_range(float low, float high) {
    if(high < low) {
        std::cout << "Make sure low is equal or less than high" << std::endl;
        return  std::numeric_limits<float>::min();
    }

    std::srand (time(NULL));
    float seed = ((double)rand()) / ((double)RAND_MAX) * high + low;

    return seed;
}



Eigen::Matrix3f Utilities::rotation_from_euler(float roll, float pitch, float yaw) {

    // roll and pitch and yaw in radians
    float su = std::sin(roll);
    float cu = std::cos(roll);
    float sv = std::sin(pitch);
    float cv = std::cos(pitch);
    float sw = std::sin(yaw);
    float cw = std::cos(yaw);

    Eigen::Matrix3f Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;

    return Rot_matrix;
}

Eigen::Vector3f Utilities::rpy_from_rotation(Eigen::Matrix3f R) {
    Eigen::Vector3f rpy;
    rpy[0] = std::atan2(R(2, 1), R(2, 2));
    rpy[1] = std::atan2(-R(2, 0), std::pow( R(2, 1)*R(2, 1) +R(2, 2)*R(2, 2), 0.5));
    rpy[2] = std::atan2( R(1, 0), R(0, 0));

    return rpy;
}

Eigen::Matrix4f Utilities::get_frame(Eigen::Matrix3f R, Vector3f T){
    Eigen::Matrix4f Trans;
    Trans.setIdentity();

    Trans.block<3,3>(0,0) = R;
    Trans.block<3,1>(0,3) = T;

    return Trans;
}

Utilities::~Utilities(void) {}
