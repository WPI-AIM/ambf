#ifndef AMBF_RAL_TF_COMPAT_H
#define AMBF_RAL_TF_COMPAT_H

#include <ambf_ral/ambf_ral.h>

#if AMBF_ROS1

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#elif AMBF_ROS2

#if (AMBF_ROS_DISTRO == AMBF_ROS_GALACTIC)
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace tf {
using Transform = tf2::Transform;
using Vector3 = tf2::Vector3;
using Quaternion = tf2::Quaternion;
using Matrix3x3 = tf2::Matrix3x3;

inline void quaternionMsgToTF(const AMBF_RAL_MSG(geometry_msgs, Quaternion) & in, Quaternion & out) {
    tf2::fromMsg(in, out);
}

inline void quaternionTFToMsg(const Quaternion & in, AMBF_RAL_MSG(geometry_msgs, Quaternion) & out) {
    out = tf2::toMsg(in);
}

inline void pointTFToMsg(const Vector3 & in, AMBF_RAL_MSG(geometry_msgs, Point) & out) {
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}

inline void vector3MsgToTF(const AMBF_RAL_MSG(geometry_msgs, Vector3) & in, Vector3 & out) {
    out.setX(in.x);
    out.setY(in.y);
    out.setZ(in.z);
}

inline void vector3TFToMsg(const Vector3 & in, AMBF_RAL_MSG(geometry_msgs, Vector3) & out) {
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}
}

#endif

#endif
