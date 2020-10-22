import numpy as np


class DH:
    def __init__(self, alpha, a, theta, d, offset, joint_type, convention='STANDARD'):
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.offset = offset
        self.joint_type = joint_type
        if convention in ['STANDARD', 'MODIFIED']:
            self.convention = convention
        else:
            raise ('ERROR, DH CONVENTION NOT UNDERSTOOD')

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

        if self.convention == 'STANDARD':
            mat = np.mat([
                [ct, -st * ca,  st * sa,  a * ct],
                [st,  ct * ca, -ct * sa,  a * st],
                [0,        sa,       ca,       d],
                [0,         0,        0,       1]
            ])
        elif self.convention == 'MODIFIED':
            mat = np.mat([
                [ct, -st, 0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0, 0, 0, 1]
            ])
        else:
            raise ('ERROR, DH CONVENTION NOT UNDERSTOOD')

        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d, self.offset)


def enforce_limits(j_raw, joint_lims):
    num_joints = len(j_raw)
    j_limited = [0.0]*num_joints

    for idx in range(num_joints):
        min_lim = joint_lims[idx][0]
        max_lim = joint_lims[idx][1]
        j_limited[idx] = max(min_lim, min(j_raw[idx], max_lim))

    return j_limited