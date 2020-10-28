import numpy as np


class JointSpaceTrajectory:
    def __init__(self, num_traj_points, num_joints, joint_limits=None):
        self._num_traj_points = num_traj_points
        self._num_joints = num_joints
        self._traj_points = np.random.uniform(-1.0, 1.0, size=(self._num_traj_points, self._num_joints))

        if joint_limits is not None:
            joint_biases = [0]*self._num_joints
            joint_ranges = [0]*self._num_joints
            for i in range(self._num_joints):
                joint_biases[i] = joint_limits[i][0]
                joint_ranges[i] = joint_limits[i][1] - joint_limits[i][0]

            for j in range(self._num_traj_points):
                for k in range(self._num_joints):
                    normalized_val = (1.0 + self._traj_points[j, k])/2.0
                    self._traj_points[j, k] = joint_biases[k] + normalized_val * joint_ranges[k]

    def print_trajectory(self):
        print self._traj_points

    def get_num_traj_points(self):
        return self._num_traj_points

    def get_num_joints(self):
        return self._num_joints

    def get_traj_at_point(self, point):
        if point < self._num_traj_points:
            return self._traj_points[point]
        else:
            raise ValueError