from PyKDL import Vector, Rotation, Frame
import numpy as np
import rospy


def compute_ik(T_7_0):
    pinch_offset_pos = Vector(0, 0, -0.05)
    yaw_offset_delta = 0.05

    # print(T_6_0)

    # Pinch Offset
    T_PO_7 = Frame(Rotation.RPY(0, 0, 0), pinch_offset_pos)

    # Pinch Offset in Origin
    T_PO_0 = T_7_0 * T_PO_7

    # Get the x vector

    Rx_PO_0 = T_PO_7.M.UnitX()

    # Take the cross product to get the first orthonormal
    orth = Rx_PO_0 * Vector(0, 0, 1)

    # Take another cross product to get the vector along the yaw link
    orth = orth * Rx_PO_0

    # Lets make sure this vector is normalized
    orth.Normalize()

    # Reorient in the Yaw Link Frame
    orth = T_PO_0.M.Inverse() * orth

    # Add another frame to accound from yaw link offset
    T_YO_PO = Frame(Rotation.RPY(0, 0, 0), orth * yaw_offset_delta)

    T_E_0 = T_7_0 * T_PO_7 * T_YO_PO

    # Now this should be the position of the point along the RC
    print("Point Along the SHAFT")
    print(T_E_0.p)

    def computeIK(pose):
        pass


req_rot = Rotation.RPY(np.pi, 0, np.pi/2 + 0.15)
req_pos = Vector(0, 0, -0.5)
T_7_0 = Frame(req_rot, req_pos)

compute_ik(T_7_0)

