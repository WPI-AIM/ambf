import numpy as np
from utilities import *
from kinematics import *

# THIS IS THE FK FOR THE ECM THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL.

# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided
def compute_FK(joint_pos):
    j = [0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    L_rcc = 0.3822
    L_scopelen = 0.385495

    # ECM DH Params
    link1 = DH(alpha=PI_2, a=0, theta=j[0], d=0, offset=PI_2, joint_type='R', convention='MODIFIED')
    link2 = DH(alpha=-PI_2, a=0, theta=j[1], d=0, offset=-PI_2, joint_type='R', convention='MODIFIED')
    link3 = DH(alpha=PI_2, a=0, theta=0, d=j[2], offset=-L_rcc, joint_type='P', convention='MODIFIED')
    link4 = DH(alpha=0, a=0, theta=j[3], d=L_scopelen, offset=0, joint_type='R', convention='MODIFIED')

    T_1_0 = link1.get_trans()
    T_2_1 = link2.get_trans()
    T_3_2 = link3.get_trans()
    T_4_3 = link4.get_trans()


    T_2_0 = np.matmul(T_1_0, T_2_1)
    T_3_0 = np.matmul(T_2_0, T_3_2)
    T_4_0 = np.matmul(T_3_0, T_4_3)

    # print("RETURNING FK FOR LINK ", len(joint_pos))

    if len(joint_pos) == 1:
        return T_1_0

    elif len(joint_pos) == 2:
        return T_2_0

    elif len(joint_pos) == 3:
        return T_3_0

    elif len(joint_pos) == 4:
        return T_4_0


# T_4_0 = compute_FK([-0.5, 0, 0.2, 0, 0, 0])
#
# print T_4_0
# print "\n AFTER ROUNDING \n"
# print(round_mat(T_4_0, 4, 4, 3))

