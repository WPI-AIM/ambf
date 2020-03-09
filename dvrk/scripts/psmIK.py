from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from psmFK import *
import rospy


def get_angle(vec_a, vec_b):
    vec_a.Normalize()
    vec_b.Normalize()
    vcross = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
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


def convert_frame_to_mat(frame):
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]

    for i in range(3):
        np_mat[i, 3] = frame.p[i]

    return np_mat


def convert_mat_to_frame(mat):
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame[(i, j)] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame


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
    palm_length = 0.0091 # Fixed length from the palm joint to the pinch joint
    pinch_length = 0.0102 # Fixed length from the pinch joint to the pinch tip
    tool_rcm_offset = 0.0156 # Delta between tool tip and the Remote Center of Motion

    # Pinch Joint
    T_PinchJoint_7 = Frame(Rotation.RPY(0, 0, 0), pinch_length * Vector(0, 0, -1))
    # Pinch Joint in Origin
    T_PinchJoint_0 = T_7_0 * T_PinchJoint_7

    # It appears from the geometry of the robot, that the palm joint is always in the ZY
    # plane of the end effector frame (7th Frame)
    # This is the logic that should give us the direction of the palm link and then
    # we know the length of the palm link so we can keep going back to find the shaftTip (PalmJoint)
    # position

    # Convert the vector from base to pinch joint in the pinch joint frame
    print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
    P_PinchJoint_local = T_PinchJoint_0.M.Inverse() * T_PinchJoint_0.p
    print("P_PinchJoint_local: ", round_vec(P_PinchJoint_local))
    # Now we can trim the value along the x axis to get a projection along the YZ plane as mentioned above
    N_PalmJoint_PinchJoint = P_PinchJoint_local
    N_PalmJoint_PinchJoint[0] = 0
    N_PalmJoint_PinchJoint.Normalize()
    # We can check the angle to see if things make sense
    angle = get_angle(N_PalmJoint_PinchJoint, Vector(0, 0, -1))
    print("Palm Link Angle in Pinch YZ Plane: ", angle)

    # If the angle between the two vectors is > 90 Degree, we should move in the opposite direction
    if angle > np.pi/2:
        N_PalmJoint_PinchJoint = -N_PalmJoint_PinchJoint

    # Add another frame to account for Palm link length
    print("N_PalmJoint_PinchJoint: ", round_vec(N_PalmJoint_PinchJoint))
    T_PalmJoint_PinchJoint = Frame(Rotation.RPY(0, 0, 0), N_PalmJoint_PinchJoint * palm_length)
    print("P_PalmJoint_PinchJoint: ", round_vec(T_PalmJoint_PinchJoint.p))
    # Get the shaft tip or the Palm's Joint position
    T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

    print("P_PalmJoint_0: ", round_vec(T_PalmJoint_0.p))
    print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
    # Now this should be the position of the point along the RC
    # print("Point Along the SHAFT: ", T_PalmJoint_0.p)

    # Now having the end point of the shaft or the PalmJoint, we can calculate some
    # angles as follows
    # xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print ('XZ Diagonal: ', xz_diagonal)
    # j1 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / xz_diagonal)
    j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

    # yz_diagonal = math.sqrt(T_PalmJoint_0.p[1] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print('YZ Diagonal: ', yz_diagonal)
    # j2 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / yz_diagonal)
    j2 = -math.atan2(T_PalmJoint_0.p[1], -T_PalmJoint_0.p[2])

    print("")
    j3 = T_PalmJoint_0.p.Norm() + tool_rcm_offset

    # Calculate j4
    # This is an important case and has to be dealt carefully. Based on some inspection, we can find that
    # we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
    # the only two vectors that are orthogonal by design at all configurations of the EE.
    cross_x7_palmlink_0 = (T_PinchJoint_0.p - T_PalmJoint_0.p) * T_7_0.M.UnitX()

    # To get j4, compare the above vector with -Y axes of T_3_0
    T_3_0 = convert_mat_to_frame(compute_FK([j1, j2, j3]))
    j4 = get_angle(cross_x7_palmlink_0, -T_3_0.M.UnitY())
    print("J4 ANGLE: ", j4)

    # Calculate j5
    a_0 = T_PalmJoint_0.p.Norm()
    b_0 = (T_PinchJoint_0.p - T_PalmJoint_0.p).Norm()
    c_0 = T_PinchJoint_0.p.Norm()

    # Compute the FK till joint 4 (Tool Roll Joint)
    T_ToolRoll_0 = convert_mat_to_frame(compute_FK([j1, j2, j3, j4]))
    R_ToolRoll_0 = T_ToolRoll_0.M

    b1 = R_ToolRoll_0.Inverse() * (T_PalmJoint_0.p)
    b2 = R_ToolRoll_0.Inverse() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

    print("B1: ", b1)
    print("B2: ", b2)

    # To determine the sign of J5, we have to cancel the effect of rotation due
    # to tool roll joint (J3) so that we can only check in a fixed plane. Now
    # we need to take the cross product of the vector from the PalmJoint to
    # PinchJoint and the vector from based to PalmJoint. Since we cancel
    # the effect of the tool roll joint, if the cross between these two vectors
    # is along the +x axes then J5 is -ve and vice-verse. We just have to be
    # careful about the when these two vectors are collinear. For that case
    # we explicitly check the angle between these two vectors
    sign = 1
    ang = get_angle(b1, b2)
    if ang == 0:
        sign = 1
    else:
        cross_prod = b1 * b2
        print("b1 x b2: ", cross_prod)
        if cross_prod[1] < 0.0:
            sign = -1
        elif cross_prod[1] > 0.0:
            sign = 1

    j5 = sign * (np.pi - math.acos((a_0 ** 2 + b_0 ** 2 - c_0 ** 2) / (2 * a_0 * b_0)))

    # Calculate j6
    # We really don't need FK for this. All we need is the angle between the vector from ShaftTip (PalmJoint)
    # to PalmTip (PinchJoint) and the vector from PalmTip(PinchJoint) to PinchTip. Incidentally, the vector from
    # from PalmTip(PinchJoint) to PinchTip is also the z axes of the rotation matrix specified by T_7_0

    P_PalmJoint_0 = T_PalmJoint_0.p
    P_PinchJoint_0 = T_PinchJoint_0.p
    P_PinchTip_0 = T_7_0.p

    D_PinchJoint_PalmJoint_0 = P_PinchJoint_0 - P_PalmJoint_0
    D_PinchTip_PinchJoint_0 = P_PinchTip_0 - P_PinchJoint_0

    D_PinchJoint_PalmJoint_local = R_ToolRoll_0.Inverse() * D_PinchJoint_PalmJoint_0
    D_PinchTip_PinchJoint_local = R_ToolRoll_0.Inverse() * D_PinchTip_PinchJoint_0

    print('D_PinchJoint_PalmJoint: ', D_PinchJoint_PalmJoint_0)
    print('D_PinchTip_PinchJoint: ', D_PinchTip_PinchJoint_0)
    print('D_PinchJoint_PalmJoint_local: ', D_PinchJoint_PalmJoint_local)
    print('D_PinchTip_PinchJoint_local: ', D_PinchTip_PinchJoint_local)

    # Since now in local coordinates, set the y value to 0 since this joint is limited to xz plane
    D_PinchJoint_PalmJoint_local[1] = 0
    D_PinchTip_PinchJoint_local[1] = 0

    b1 = R_ToolRoll_0.Inverse() * (T_PinchJoint_0.p - T_PalmJoint_0.p)
    b2 = R_ToolRoll_0.Inverse() * (T_7_0.p - T_PinchJoint_0.p)

    # We use a similar logic as the one used in J5 to determine the sign of joint 6. But instead of the
    # cross product being in the +ve or -ve X axes, it will be in +ve or -ve Y axes.
    sign = 1
    ang = get_angle(b2, b1)
    if ang == 0:
        sign = 1
    else:
        cross_prod = b1 * b2
        print("b1 x b2: ", cross_prod)
        if cross_prod[1] < 0.0:
            sign = -1
        elif cross_prod[1] > 0.0:
            sign = 1

    j6 = sign * get_angle(D_PinchJoint_PalmJoint_local, D_PinchTip_PinchJoint_local)

    str = '\n*************************'*3
    print(str)
    print("Joint 1: ", round(j1, 3))
    print("Joint 2: ", round(j2, 3))
    print("Joint 3: ", round(j3, 3))
    print("Joint 4: ", round(j4, 3))
    print("Joint 5: ", round(j5, 3))
    print("Joint 6: ", round(j6, 3))

    T_7_0_req = convert_frame_to_mat(T_7_0)
    T_7_0_req = round_transform(T_7_0_req, 3)
    print 'Requested Pose: \n', T_7_0_req
    T_7_0_computed = compute_FK([j1, j2, j3, j4, j5, j6])
    round_transform(T_7_0_computed, 3)
    print'Computed Pose: \n', T_7_0_computed


rx = Rotation.RPY(0, 0.0, 0.0)
ry = Rotation.RPY(0.0, np.pi/2, 0.0)
rz = Rotation.RPY(0.0, 0.0, np.pi/4)

tip_offset_rot = Rotation.RPY(np.pi, 0, np.pi/2)
req_rot = tip_offset_rot * rz * ry * rx
req_pos = Vector(0.0, 0.0, -0.3)
T_7_0 = Frame(req_rot, req_pos)
# print "REQ POSE \n", round_transform(convert_frame_to_mat(T_7_0), 3), "\n\n--------\n\n"
compute_IK(T_7_0)

