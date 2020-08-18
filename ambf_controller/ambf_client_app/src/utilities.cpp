#include "ambf_client_app/utilities.h"

Utilities::Utilities() {}

float Utilities::get_angle(Vector3f vec_a, Vector3f vec_b, Vector3f up_vector) {
    float angle = 0.0;
    vec_a.normalize();
    vec_b.normalize();

    Vector3f cross_ab = vec_a.cross(vec_b);
    float vdot = vec_a.dot(vec_b);
//    std::cout << "a, b: " << std::endl << vec_a << std::endl  << vec_b << std::endl;
//    std::cout << "vdot : " << vdot << std::endl << std::endl;
//    std::cout << "cross_ab : " << std::endl << cross_ab << std::endl;

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

Utilities::~Utilities(void) {}
