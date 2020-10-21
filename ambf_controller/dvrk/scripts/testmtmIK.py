from mtmIK import *
from ambf_client import Client
import time


class JointSpaceTrajectory:
    def __init__(self):
        self._num_traj_points = 11
        self._num_joints = 7
        self._traj_points = [[0] * self._num_joints] * self._num_traj_points
        self._traj_points[0] = [-0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0]
        self._traj_points[1] = [-0.1, 0.1, 0.1, 0.1, -0.1, 0.0, 0.1]
        self._traj_points[2] = [-0.3, 0.3, 0.1, 0.3, -0.3, 0.0, 0.3]
        self._traj_points[3] = [-0.5, 0.5, 0.15, 0.5, -0.5, 0.0, 0.5]
        self._traj_points[4] = [-0.7, 0.7, 0.20, 0.7, -0.7, 0.0, 0.7]
        self._traj_points[5] = [-0.9, 0.9, 0.22, 0.9, -0.9, 0.0, 0.9]
        self._traj_points[6] = [0.1, -0.1, 0.22, -0.1, 0.1, 0.0, -0.1]
        self._traj_points[7] = [0.3, -0.3, 0.20, -0.3, 0.3, 0.0, -0.3]
        self._traj_points[8] = [0.5, -0.5, 0.15, -0.5, 0.5, 0.0, -0.5]
        self._traj_points[9] = [0.7, -0.7, 0.1, -0.7, 0.7, 0.0, -0.7]
        self._traj_points[10] = [0.9, -0.9, 0.1, -0.9, 0.9, 0.0, -0.9]

    def get_num_traj_points(self):
        return self._num_traj_points

    def get_num_joints(self):
        return self._num_joints

    def get_traj_at_point(self, point):
        if point < self._num_traj_points:
            return self._traj_points[point]
        else:
            raise ValueError


def test_ik():
    js_traj = JointSpaceTrajectory()
    num_points = js_traj.get_num_traj_points()
    num_joints = 7
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_7_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_7_0))
        computed_q = round_vec(computed_q, 4)

        errors = [0] * js_traj.get_num_joints()
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('******')
        print ('Case:', i)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]


def test_ambf_mtm():
    c = Client()
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('TopPanel')
    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'TopPanel-OutPitchShoulder', 0
    #  'OutPitchShoulder-ArmParallel', 1
    #  'ArmParallel-BottomArm', 2
    #  'BottomArm-WristPlatform', 3
    #  'WristPlatform-WristPitch', 4
    #  'WristPitch-WristYaw', 5
    #  'WristYaw-WristRoll', 6

    js_traj = JointSpaceTrajectory()
    num_points = js_traj.get_num_traj_points()
    num_joints = 7
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_7_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_7_0))

        # print('SETTING JOINTS: ')
        # print(computed_q)

        b.set_joint_pos('TopPanel-OutPitchShoulder', computed_q[0])
        b.set_joint_pos('OutPitchShoulder-ArmParallel', computed_q[1])
        b.set_joint_pos('ArmParallel-BottomArm', computed_q[2])
        b.set_joint_pos('BottomArm-WristPlatform', computed_q[3])
        b.set_joint_pos('WristPlatform-WristPitch', computed_q[4])
        b.set_joint_pos('WristPitch-WristYaw', computed_q[5])
        b.set_joint_pos('WristYaw-WristRoll', computed_q[6])

        errors = [0]*num_joints
        for j in range(7):
            errors[j] = test_q[j] - computed_q[j]
        print ('******')
        print ('Case:', i)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]

        time.sleep(1.0)

    time.sleep(3.0)


if __name__ == "__main__":
    # test_ik()
    test_ambf_mtm()
