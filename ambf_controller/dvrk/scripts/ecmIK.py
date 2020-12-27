from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from ecmFK import *
import rospy

# THIS IS THE IK FOR THE ECM

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


def compute_IK(T_4_0):
    L_rcc = 0.3822
    L_scopelen = 0.385495

    j1 = math.atan2(T_4_0.p[0], -T_4_0.p[2])

    xz_diag = math.sqrt(T_4_0.p[0]**2 + T_4_0.p[2]**2)
    j2 = -math.atan2(T_4_0.p[1], xz_diag)

    j3 = T_4_0.p.Norm() + (L_rcc - L_scopelen)

    T_4_0_FK = convert_mat_to_frame(compute_FK([j1, j2, j3, 0]))
    R_IK_in_FK = T_4_0_FK.M.Inverse() * T_4_0.M

    j4 = R_IK_in_FK.GetRPY()[2]

    # str = '\n**********************************'*3
    # print(str)
    # print("Joint 1: ", round(j1, 3))
    # print("Joint 2: ", round(j2, 3))
    # print("Joint 3: ", round(j3, 3))
    # print("Joint 4: ", round(j4, 3))

    # T_4_0_req = convert_frame_to_mat(T_4_0)
    # T_4_0_req = round_transform(T_4_0_req, 3)
    # print('Requested Pose: \n', T_4_0_req)
    # T_4_0_computed = compute_FK([j1, j2, j3, j4, j5, j6, 0])
    # round_transform(T_4_0_computed, 3)
    # print('Computed Pose: \n', T_4_0_computed)

    return [j1, j2, j3, j4]
