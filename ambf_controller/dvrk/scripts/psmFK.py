import numpy as np
from utilities import *
from kinematics import *

# THIS IS THE FK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. BASED ON
# THE FRAME ATTACHMENT IN THE DVRK MANUAL THE CORRECT DH CAN FOUND IN THIS FILE

# ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
# ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
# R_7_0 = [ 0,  1,  0 ]
#       = [ 1,  0,  0 ]
#       = [ 0,  0, -1 ]
# Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided
def compute_FK(joint_pos):
    j = [0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    # The last frame is fixed

    L_rcc = 0.4389
    L_tool = 0.416
    L_pitch2yaw = 0.009
    L_yaw2ctrlpnt = 0.0106

    # PSM DH Params
    link1 = DH(alpha=PI_2, a=0, theta=j[0], d=0, offset=PI_2, joint_type='R', convention='MODIFIED')
    link2 = DH(alpha=-PI_2, a=0, theta=j[1], d=0, offset=-PI_2, joint_type='R', convention='MODIFIED')
    link3 = DH(alpha=PI_2, a=0, theta=0, d=j[2], offset=-L_rcc, joint_type='P', convention='MODIFIED')
    link4 = DH(alpha=0, a=0, theta=j[3], d=L_tool, offset=0, joint_type='R', convention='MODIFIED')
    link5 = DH(alpha=-PI_2, a=0, theta=j[4], d=0, offset=-PI_2, joint_type='R', convention='MODIFIED')
    link6 = DH(alpha=-PI_2, a=L_pitch2yaw, theta=j[5], d=0, offset=-PI_2, joint_type='R', convention='MODIFIED')
    link7 = DH(alpha=-PI_2, a=0, theta=0, d=L_yaw2ctrlpnt, offset=PI_2, joint_type='R', convention='MODIFIED')

    T_1_0 = link1.get_trans()
    T_2_1 = link2.get_trans()
    T_3_2 = link3.get_trans()
    T_4_3 = link4.get_trans()
    T_5_4 = link5.get_trans()
    T_6_5 = link6.get_trans()
    T_7_6 = link7.get_trans()

    # print("\nT_4_3: ")
    # print(T_4_3)

    T_2_0 = np.matmul(T_1_0, T_2_1)
    T_3_0 = np.matmul(T_2_0, T_3_2)
    T_4_0 = np.matmul(T_3_0, T_4_3)
    T_5_0 = np.matmul(T_4_0, T_5_4)
    T_6_0 = np.matmul(T_5_0, T_6_5)
    T_7_0 = np.matmul(T_6_0, T_7_6)

    # print("RETURNING FK FOR LINK ", len(joint_pos))

    if len(joint_pos) == 1:
        return T_1_0

    elif len(joint_pos) == 2:
        return T_2_0

    elif len(joint_pos) == 3:
        return T_3_0

    elif len(joint_pos) == 4:
        return T_4_0

    elif len(joint_pos) == 5:
        return T_5_0

    elif len(joint_pos) == 6:
        return T_6_0

    elif len(joint_pos) == 7:
        return T_7_0


class DH:
    def __init__(self, alpha, a, theta, d, offset, joint_type):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.offset = offset
        self.joint_type = joint_type

    def mat_from_dh(self, alpha, a, theta, d, offset):

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        if self.joint_type == 'R':
            theta = theta + offset
        elif self.joint_type == 'P':
            d = d + offset
        else:
            assert type == 'P' and type == 'R'

        ct = np.cos(theta)
        st = np.sin(theta)

        mat = np.mat([
            [ct     , -st     ,  0 ,  a],
            [st * ca,  ct * ca, -sa, -d * sa],
            [st * sa,  ct * sa,  ca,  d * ca],
            [0      ,  0      ,  0 ,  1]
        ])
        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d, self.offset)


# T_7_0 = compute_FK([-0.5, 0, 0.2, 0, 0, 0])
#
# print T_7_0
# print "\n AFTER ROUNDING \n"
# print(round_mat(T_7_0, 4, 4, 3))

