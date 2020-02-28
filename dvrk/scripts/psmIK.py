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


def compute_IK(T_7_0):
    pinch_offset_delta = 0.05
    yaw_offset_delta = 0.05

    # Pinch Offset
    T_PO_7 = Frame(Rotation.RPY(0, 0, 0), pinch_offset_delta * Vector(0, 0, -1))

    # Pinch Offset in Origin
    T_PO_0 = T_7_0 * T_PO_7

    # Get the x vector
    Rx_PO_0 = T_PO_7.M.UnitX()

    # Get the angle between the computer X vector and the global z vector
    angle = get_angle(Rx_PO_0, Vector(0, 0, 1))

    if angle == 0 or angle == np.pi:
        # Due to the geometry of the PSM, in such cases, the correction is always
        # tangential to the global z axes.
        x = -T_7_0.p[0]
        y = -T_7_0.p[1]
        z = 0.0
        yaw_offset_dir_in_world = Vector(x, y, z)
        yaw_offset_dir_in_world.Normalize()
    else:
        # Take the cross product to get the first orthonormal
        yaw_offset_orth_dir = Rx_PO_0 * Vector(0, 0, 1)
        # Take another cross product to get the vector along the yaw link
        yaw_offset_dir_in_world = yaw_offset_orth_dir * Rx_PO_0
        # Lets make sure this vector is normalized
        yaw_offset_dir_in_world.Normalize()

    # Reorient in the Yaw Link Frame
    yaw_offset_dir_in_yaw_link = T_PO_0.M.Inverse() * yaw_offset_dir_in_world

    # Add another frame to accound from yaw link offset
    T_YO_PO = Frame(Rotation.RPY(0, 0, 0), yaw_offset_dir_in_yaw_link * yaw_offset_delta)

    T_ShaftTip_0 = T_7_0 * T_PO_7 * T_YO_PO

    # Now this should be the position of the point along the RC
    print("Point Along the SHAFT: ", T_ShaftTip_0.p)

    # Now having the end point of the shaft, the end point of the yaw link, we can calculate some
    # angles as follows
    xz_diagonal = math.sqrt(T_ShaftTip_0.p[0] ** 2 + T_ShaftTip_0.p[2] ** 2)
    print ('XZ Diagonal: ', xz_diagonal)
    j1 = np.sign(T_ShaftTip_0.p[0]) * math.acos(-T_ShaftTip_0.p[2] / xz_diagonal)

    yz_diagonal = math.sqrt(T_ShaftTip_0.p[1] ** 2 + T_ShaftTip_0.p[2] ** 2)
    print('YZ Diagonal: ', yz_diagonal)
    j2 = np.sign(T_ShaftTip_0.p[0]) * math.acos(-T_ShaftTip_0.p[2] / yz_diagonal)

    j3 = -T_ShaftTip_0.p[2]

    # Calculate j4
    # The x vector or y vector of the yaw offset rotation mat can be used to calculate the 4th, or tool roll angle
    x_vec = T_ShaftTip_0.M.UnitX()
    print("J4, Rx_ShaftTip_0: ", x_vec)

    ref_vector = Vector(0, 1, 0)
    # Check if the compare vector is along global z, in this case, we need to find another vector
    if abs(1.0 - x_vec[2]) < 0.00001:
        compare_vec = T_ShaftTip_0.M.UnitZ()
    # Otherwise proceed as normal and chose the x vector
    else:
        compare_vec = T_ShaftTip_0.M.UnitX()

    compare_vec[2] = 0
    compare_vec.Normalize()
    print("J4, Compare Vector Normalized: ", compare_vec)
    j4 = np.sign(compare_vec[0]) * get_angle(ref_vector, compare_vec)

    # Calculate j5
    a = T_ShaftTip_0.p.Norm()
    b = (T_ShaftTip_0.p - T_PO_0.p).Norm()
    c = T_PO_0.p.Norm()
    print("A: ", a, "B: ", b, "C: ", c)
    print("Val: ", (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))
    j5 = np.pi - math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b))

    print("Joint 1: ", j1)
    print("Joint 2: ", j2)
    print("Joint 3: ", j3)
    print("Joint 4: ", j4)
    print("Joint 5: ", j5)


req_rot = Rotation.RPY(np.pi, 0, np.pi/2 + 0.5)
req_pos = Vector(0.0, 0.0, -0.2)
T_7_0 = Frame(req_rot, req_pos)

compute_IK(T_7_0)

