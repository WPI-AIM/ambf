from psmIK import *
from ambf_client import Client
import time


class JointSpaceTrajectory:
    def __init__(self):
        # There are 7 frames for DH for the PSM (no D.O.F for 7th Joint). Thus we are going to provide
        # 7 joint values to the PSM FK which gives back T_7_0 instead of T_6_0.
        self._num_traj_points = 10
        self._num_joints = 7
        self._traj_points = [[0] * self._num_joints] * self._num_traj_points
        self._traj_points[0] = [-0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.0]
        self._traj_points[1] = [-0.3, 0.3, 0.1, 0.3, -0.3, -0.3, 0.0]
        self._traj_points[2] = [-0.5, 0.5, 0.15, 0.5, -0.5, -0.5, 0.0]
        self._traj_points[3] = [-0.7, 0.7, 0.20, 0.7, -0.7, -0.7, 0.0]
        self._traj_points[4] = [-0.9, 0.9, 0.22, 0.9, -0.9, -0.9, 0.0]
        self._traj_points[5] = [0.1, -0.1, 0.22, -0.1, 0.1, 0.1, 0.0]
        self._traj_points[6] = [0.3, -0.3, 0.20, -0.3, 0.3, 0.3, 0.0]
        self._traj_points[7] = [0.5, -0.5, 0.15, -0.5, 0.5, 0.5, 0.0]
        self._traj_points[8] = [0.7, -0.7, 0.1, -0.7, 0.7, 0.7, 0.0]
        self._traj_points[9] = [0.9, -0.9, 0.1, -0.9, 0.9, 0.9, 0.0]

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
    num_joints = 6
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


def test_ambf_psm():
    c = Client()
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('baselink')
    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b

    js_traj = JointSpaceTrajectory()
    num_points = js_traj.get_num_traj_points()
    num_joints = 6
    for i in range(num_points):

        test_q = js_traj.get_traj_at_point(i)
        T_7_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_7_0))

        # print('SETTING JOINTS: ')
        # print(computed_q)

        b.set_joint_pos('baselink-yawlink', computed_q[0])
        b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        b.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
        b.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
        b.set_joint_pos('toolpitchlink-toolgripper1link', computed_q[5])
        b.set_joint_pos('toolpitchlink-toolgripper2link', -computed_q[5])

        errors = [0]*num_joints
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('******')
        print ('Case:', i)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]

        time.sleep(1.0)

    time.sleep(3.0)


if __name__ == "__main__":
    # test_ik()
    test_ambf_psm()
