from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from mtmFK import *
import rospy

# THIS IS THE IK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. CHECK THE FK
# FILE TO FIND THE CORRECT DH PARAMS BASED ON THE FRAME ATTACHMENT IN THE DVRK MANUAL

# ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
# ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
# R_7_0 = [ 0,  1,  0 ]
#       = [ 1,  0,  0 ]
#       = [ 0,  0, -1 ]
# Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

# Read the frames, positions and rotation as follows, T_A_B, means that this
# is a Transfrom of frame A with respect to frame B. Similarly P_A_B is the
# Position Vector of frame A's origin with respect to frame B's origin. And finally
# R_A_B is the rotation matrix representing the orientation of frame B with respect to
# frame A.

# If you are confused by the above description, consider the equations below to make sense of it
# all. Suppose we have three frames A, B and C. The following equations will give you the L.H.S

# 1) T_C_A = T_B_A * T_C_B
# 2) R_A_C = inv(R_B_A * R_C_B)
# 3) P_C_A = R_B_A * R_C_B * P_C

# For Postions, the missing second underscore separated quantity means that it is expressed in local
# coodinates. Rotations, and Transforms are always to defined a frame w.r.t to some
# other frame so this is a special case for only positions. Consider the example

# P_B indiciates a point expressed in B frame.

# Now there are two special cases that are identified by letter D and N. The first characeter D indiciates a
# difference (vector) of between two points, specified by the first and second underscore separater (_) strings,
# expressed in the third underscore separated reference. I.e.

# D_A_B_C
# This means the difference between Point A and  B expressed in C. On the other hand the letter N indicates
# the direction, and not specifically the actually
# measurement. So:
#
# N_A_B_C
#
# is the direction between the difference of A and B expressed in C.


def compute_IK(T_7_0):
    L_arm = 0.278828
    L_forearm = 0.363867
    L_h = 0.147733

    j1 = math.atan2(T_7_0.p[0], -T_7_0.p[1])

    l1 = L_arm
    l2 = math.sqrt(L_forearm ** 2 + L_h ** 2)
    angle_offset = math.asin(L_h/l2)

    x = -T_7_0.p[2]
    y = math.sqrt(T_7_0.p[0] ** 2 + T_7_0.p[1] ** 2)

    d = math.sqrt(x ** 2 + y ** 2)
    a1 = math.atan2(y, x)
    a2 = math.acos((l1 ** 2 - l2 ** 2 + d ** 2) / (2.0 * l1 * d))
    q2 = -math.acos((l1 ** 2 + l2 ** 2 - d ** 2) / (2.0 * l1 * l2))

    j2 = a1 - a2
    j3 = q2 - angle_offset + PI_2

    T_7_0_FK = convert_mat_to_frame(compute_FK([j1, j2, j3, 0, 0, 0, 0]))

    R_IK_in_FK = T_7_0_FK.M.Inverse() * T_7_0.M

    # https://www.geometrictools.com/Documentation/EulerAngles.pdf
    r = R_IK_in_FK

    theta_y = 0
    theta_x = 0
    theta_z = 0
    if r[0, 2] < 1.0:
        if r[0, 2] > -1.0:
            theta_y = math.asin(r[0, 2])
            theta_x = math.atan2(-r[1, 2], r[2, 2])
            theta_z = math.atan2(-r[0, 1], r[0, 0])
        else:
            theta_y = -PI_2
            theta_x = math.atan2(r[1, 0], r[1, 1])
            theta_z = 0
    else:
        theta_y = PI_2
        theta_x = math.atan2(r[1, 0], r[1, 1])
        theta_z = 0

    j7 = theta_z
    j5 = -theta_y
    j4 = theta_x
    j6 = 0

    return [j1, j2, j3, j4, j5, j6, j7]
