import numpy as np
import numpy.linalg as la

PI = np.pi
PI_2 = np.pi/2


# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided
def compute_FK(joint_pos):
    j = [0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]
    # PSM DH Params
    link1 = DH(alpha=PI_2, a=0, theta=j[0], d=0, offset=PI_2)
    link2 = DH(alpha=-PI_2, a=0, theta=j[1], d=0, offset=-PI_2)
    link3 = DH(alpha=PI_2, a=0, theta=0, d=j[2], offset=-0.4318, type='P')
    link4 = DH(alpha=0, a=0, theta=j[3], d=0.4162, offset=0)
    link5 = DH(alpha=-PI_2, a=0, theta=j[4], d=0, offset=-PI_2)
    link6 = DH(alpha=-PI_2, a=0.0091, theta=j[5], d=0, offset=-PI_2)
    link7 = DH(alpha=-PI_2, a=0, theta=0, d=0.0102, offset=PI_2)

    T_1_0 = link1.get_trans()
    T_2_1 = link2.get_trans()
    T_3_2 = link3.get_trans()
    T_4_3 = link4.get_trans()
    T_5_4 = link5.get_trans()
    T_6_5 = link6.get_trans()
    T_7_6 = link7.get_trans()

    T_2_0 = np.matmul(T_1_0, T_2_1)
    T_3_0 = np.matmul(T_2_0, T_3_2)
    T_4_0 = np.matmul(T_3_0, T_4_3)
    T_5_0 = np.matmul(T_4_0, T_5_4)
    T_6_0 = np.matmul(T_5_0, T_6_5)
    T_7_0 = np.matmul(T_6_0, T_7_6)

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
        return T_7_0


def round_mat(mat, rows, cols, precision=4):
    for i in range(0, rows):
        for j in range(0, cols):
            mat[i, j] = round(mat[i, j], precision)

    return mat


def round_transform(mat, precision=4):
    return round_mat(mat, 4, 4, precision)


def round_vec(vec, precision=4):
    return round_mat(vec, 3, 1, precision)


class DH:
    def __init__(self, alpha, a, theta, d, offset, type='R'):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.offset = offset
        self.type = type

    def mat_from_dh(self, alpha, a, theta, d, offset, type):
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        if type == 'R':
            theta = theta + offset
        elif type == 'P':
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
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d, self.offset, self.type)


T_7_0 = compute_FK([-0.5, 0, 0.2, 0, 0, 0])

print T_7_0
print "\n AFTER ROUNDING \n"
print(round_mat(T_7_0, 4, 4, 3))

