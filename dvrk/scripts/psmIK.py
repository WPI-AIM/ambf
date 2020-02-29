from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
import rospy


def get_angle(vec_a, vec_b):
    vec_a.Normalize()
    vec_b.Normalize()
    vcross = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        return 0.0
    # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        return np.pi
    else:
        return math.acos(vdot)


def round_vec(vec, precision=4):
    for i in range(3):
        vec[i] = round(vec[i], precision)
    return vec

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
    palm_length = 0.05 # Fixed length from the palm joint to the pinch joint
    pinch_length = 0.05 # Fixed length from the pinch joint to the pinch tip

    # Pinch Joint
    T_PinchJoint_7 = Frame(Rotation.RPY(0, 0, 0), pinch_length * Vector(0, 0, -1))
    # Pinch Joint in Origin
    T_PinchJoint_0 = T_7_0 * T_PinchJoint_7
    # Get the x vector
    Rx_PinchJoint_0 = T_PinchJoint_7.M.UnitX()
    # Get the angle between the computed X vector and the origin's z vector
    angle = get_angle(Rx_PinchJoint_0, Vector(0, 0, 1))

    if angle == 0 or angle == np.pi:
        # Due to the geometry of the PSM, in such cases, the correction is always
        # tangential to the global z axes.
        x = -T_7_0.p[0]
        y = -T_7_0.p[1]
        z = 0.0
        N_PalmJoint_PinchJoint_0 = Vector(x, y, z)
        N_PalmJoint_PinchJoint_0.Normalize()
    else:
        # Take the cross product with the world z axes to get the tangential
        orthognal_dir = Rx_PinchJoint_0 * Vector(0, 0, 1)
        # Take another cross product to get the vector along the Palm link
        N_PalmJoint_PinchJoint_0 = orthognal_dir * Rx_PinchJoint_0
        # Lets make sure this vector is normalized
        N_PalmJoint_PinchJoint_0.Normalize()

    # Reorient in the Palm Link Frame
    R_PinchJoint_0 = T_PinchJoint_0.M
    P_PalmJoint_PinchJoint = R_PinchJoint_0.Inverse() * N_PalmJoint_PinchJoint_0
    # Add another frame to account for Palm link length
    T_PalmJoint_PinchJoint = Frame(Rotation.RPY(0, 0, 0), P_PalmJoint_PinchJoint * palm_length)
    # Get the shaft tip or the Palm's Joint position
    T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

    # Now this should be the position of the point along the RC
    print("Point Along the SHAFT: ", T_PalmJoint_0.p)

    # Now having the end point of the shaft or the PalmJoint, we can calculate some
    # angles as follows
    xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
    print ('XZ Diagonal: ', xz_diagonal)
    j1 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / xz_diagonal)

    yz_diagonal = math.sqrt(T_PalmJoint_0.p[1] ** 2 + T_PalmJoint_0.p[2] ** 2)
    print('YZ Diagonal: ', yz_diagonal)
    j2 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / yz_diagonal)

    j3 = -T_PalmJoint_0.p[2]

    # Calculate j4
    # The x vector or y vector of the Pinch Joint rotation mat can be used to calculate the 4th, or tool roll angle
    x_vec = T_PalmJoint_0.M.UnitX()
    print("J4, Rx_ShaftTip_0: ", x_vec)

    ref_vector = Vector(0, 1, 0)
    # Check if the compare vector is along global z, in this case, we need to find another vector
    if abs(1.0 - x_vec[2]) < 0.00001:
        compare_vec = T_PalmJoint_0.M.UnitZ()
    # Otherwise proceed as normal and chose the x vector
    else:
        compare_vec = T_PalmJoint_0.M.UnitX()

    compare_vec[2] = 0
    compare_vec.Normalize()
    print("J4, Compare Vector Normalized: ", compare_vec)
    j4 = np.sign(compare_vec[0]) * get_angle(ref_vector, compare_vec)

    # Calculate j5
    a = T_PalmJoint_0.p.Norm()
    b = (T_PalmJoint_0.p - T_PinchJoint_0.p).Norm()
    c = T_PinchJoint_0.p.Norm()
    print("A: ", a, "B: ", b, "C: ", c)
    print("Val: ", (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))
    j5 = np.pi - math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b))

    # Calculate j6
    # We really don't need FK for this. All we need is the angle between the vector from ShaftTip (PalmJoint)
    # to PalmTip (PinchJoint) and the vector from PalmTip(PinchJoint) to PinchTip. Incidentally, the vector from
    # from PalmTip(PinchJoint) to PinchTip is also the z axes of the rotation matrix specified by T_7_0

    P_PalmJoint_0 = T_PalmJoint_0.p
    P_PinchJoint_0 = T_PinchJoint_0.p
    P_PinchTip_0 = T_7_0.p

    a_0 = P_PinchJoint_0 - P_PalmJoint_0
    b_0 = P_PinchTip_0 - P_PinchJoint_0

    R_tool_roll = Rotation.RPY(0, 0, j4)

    a = R_tool_roll.Inverse() * a_0
    b = R_tool_roll.Inverse() * b_0

    print('A :', round_vec(a))
    print('B :', round_vec(b))

    j6 = - np.sign(b[0]) * get_angle(a_0, b_0)

    str = '\n*************************'*3
    print(str)
    print("Joint 1: ", j1)
    print("Joint 2: ", j2)
    print("Joint 3: ", j3)
    print("Joint 4: ", j4)
    print("Joint 5: ", j5)
    print("Joint 6: ", j6)


req_rot = Rotation.RPY(np.pi+0.5, 0, np.pi/2)
req_pos = Vector(0.0, 0.0, -0.2)
T_7_0 = Frame(req_rot, req_pos)

compute_IK(T_7_0)

