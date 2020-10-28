from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from psmFK import *
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

T_PalmJoint_0 = None


def get_T_PalmJoint_0():
    global T_PalmJoint_0
    return T_PalmJoint_0


def compute_IK(T_7_0):
    global T_PalmJoint_0
    L_pitch2yaw = 0.009 # Fixed length from the palm joint to the pinch joint
    L_yaw2ctrlpnt = 0.0106 # Fixed length from the pinch joint to the pinch tip
    L_tool2rcm_offset = 0.0229 # Delta between tool tip and the Remote Center of Motion

    # Pinch Joint
    T_PinchJoint_7 = Frame(Rotation.RPY(0, 0, 0), L_yaw2ctrlpnt * Vector(0.0, 0.0, -1.0))
    # Pinch Joint in Origin
    T_PinchJoint_0 = T_7_0 * T_PinchJoint_7

    # It appears from the geometry of the robot, that the palm joint is always in the ZY
    # plane of the end effector frame (7th Frame)
    # This is the logic that should give us the direction of the palm link and then
    # we know the length of the palm link so we can keep going back to find the shaftTip (PalmJoint)
    # position

    # Convert the vector from base to pinch joint in the pinch joint frame
    # print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
    R_0_PinchJoint = T_PinchJoint_0.M.Inverse()
    P_PinchJoint_local = R_0_PinchJoint * T_PinchJoint_0.p
    # print("P_PinchJoint_local: ", round_vec(P_PinchJoint_local))
    # Now we can trim the value along the x axis to get a projection along the YZ plane as mentioned above
    N_PalmJoint_PinchJoint = -P_PinchJoint_local
    N_PalmJoint_PinchJoint[0] = 0
    N_PalmJoint_PinchJoint.Normalize()

    # We can check the angle to see if things make sense
    # angle = get_angle(N_PalmJoint_PinchJoint, Vector(0, 0, -1))
    # print("Palm Link Angle in Pinch YZ Plane: ", angle)

    # # If the angle between the two vectors is > 90 Degree, we should move in the opposite direction
    # if angle > np.pi/2:
    #     N_PalmJoint_PinchJoint = N_PalmJoint_PinchJoint
    #
    # print angle

    # Add another frame to account for Palm link length
    # print("N_PalmJoint_PinchJoint: ", round_vec(N_PalmJoint_PinchJoint))
    T_PalmJoint_PinchJoint = Frame(Rotation.RPY(0, 0, 0), N_PalmJoint_PinchJoint * L_pitch2yaw)
    # print("P_PalmJoint_PinchJoint: ", round_vec(T_PalmJoint_PinchJoint.p))
    # Get the shaft tip or the Palm's Joint position
    T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

    # print("P_PalmJoint_0: ", round_vec(T_PalmJoint_0.p))
    # print("P_PinchJoint_0: ", round_vec(T_PinchJoint_0.p))
    # Now this should be the position of the point along the RC
    # print("Point Along the SHAFT: ", T_PalmJoint_0.p)

    # Calculate insertion_depth to check if the tool is past the RCM
    insertion_depth = T_PalmJoint_0.p.Norm()

    # Now having the end point of the shaft or the PalmJoint, we can calculate some
    # angles as follows
    xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print ('XZ Diagonal: ', xz_diagonal)

    yz_diagonal = math.sqrt(T_PalmJoint_0.p[1] ** 2 + T_PalmJoint_0.p[2] ** 2)
    # # print('YZ Diagonal: ', yz_diagonal)

    j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])

    # j2 = np.sign(T_PalmJoint_0.p[0]) * math.acos(-T_PalmJoint_0.p[2] / yz_diagonal)
    j2 = -math.atan2(T_PalmJoint_0.p[1], xz_diagonal)

    j3 = insertion_depth + L_tool2rcm_offset

    # Calculate j4
    # This is an important case and has to be dealt carefully. Based on some inspection, we can find that
    # we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
    # the only two vectors that are orthogonal at all configurations of the EE.
    cross_palmlink_x7_0 = T_7_0.M.UnitX() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

    # To get j4, compare the above vector with Y axes of T_3_0
    T_3_0 = convert_mat_to_frame(compute_FK([j1, j2, j3]))
    j4 = get_angle(cross_palmlink_x7_0, T_3_0.M.UnitY(), up_vector=-T_3_0.M.UnitZ())

    # Calculate j5
    # This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
    T_4_0 = convert_mat_to_frame(compute_FK([j1, j2, j3, j4]))
    j5 = get_angle(T_PinchJoint_0.p - T_PalmJoint_0.p, T_4_0.M.UnitZ(), up_vector=-T_4_0.M.UnitY())

    # Calculate j6
    # This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
    T_5_0 = convert_mat_to_frame(compute_FK([j1, j2, j3, j4, j5]))
    j6 = get_angle(T_7_0.M.UnitZ(), T_5_0.M.UnitX(), up_vector=-T_5_0.M.UnitY())

    str = '\n**********************************'*3
    # print(str)
    # print("Joint 1: ", round(j1, 3))
    # print("Joint 2: ", round(j2, 3))
    # print("Joint 3: ", round(j3, 3))
    # print("Joint 4: ", round(j4, 3))
    # print("Joint 5: ", round(j5, 3))
    # print("Joint 6: ", round(j6, 3))

    # T_7_0_req = convert_frame_to_mat(T_7_0)
    # T_7_0_req = round_transform(T_7_0_req, 3)
    # print('Requested Pose: \n', T_7_0_req)
    # T_7_0_computed = compute_FK([j1, j2, j3, j4, j5, j6, 0])
    # round_transform(T_7_0_computed, 3)
    # print('Computed Pose: \n', T_7_0_computed)

    return [j1, j2, j3, j4, j5, j6]
