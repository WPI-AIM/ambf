import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from geometry_msgs.msg import Wrench, WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from argparse import ArgumentParser
from PyKDL import Vector, Rotation


class WatchDog(object):
    def __init__(self, time_out=0.1):
        self._expire_duration = rospy.Duration.from_sec(time_out)
        self._next_cmd_expected_time = rospy.Time.now()
        self._initialized = False

    def acknowledge_wd(self):
        self._initialized = True
        self._next_cmd_expected_time = rospy.Time.now() + self._expire_duration

    def is_wd_expired(self):
        if rospy.Time.now() > self._next_cmd_expected_time and self._initialized:
            return True
        else:
            return False

    def console_print(self, class_name):
        if self._initialized:
            print('Watch Dog Expired, Resetting {} command'.format(class_name))
            self._initialized = False


def rot_z(q):
    r = np.mat([[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1]])
    return r


class TaskToJointSpace:
    def __init__(self, arm_name='MTMR'):
        self._num_rows = 6
        self._num_cols = 7
        self._jac_spa = np.zeros((self._num_rows, self._num_cols))
        self._jac_bod = np.zeros((self._num_rows, self._num_cols))

        self._new_data = 0

        self._rate = rospy.Rate(500)
        self._joint_state = JointState()
        self._cart_state = JointState()
        self._state_cmd = JointState()

        self._wrench_cmd = np.zeros((6, 1))
        self._joint_torques_cmd = np.zeros((7, 1))

        self._jac_spa_sub = rospy.Subscriber('/dvrk/' + arm_name + '/jacobian_spatial', Float64MultiArray, self.jac_spa_cb, queue_size=1)
        self._jac_bod_sub = rospy.Subscriber('/dvrk/' + arm_name + '/jacobian_body', Float64MultiArray, self.jac_bod_cb, queue_size=1)
        self._js_sub = rospy.Subscriber('/dvrk/' + arm_name + '/state_joint_current', JointState, self.js_cb, queue_size=1)
        self._cs_sub = rospy.Subscriber('/dvrk/' + arm_name + '/position_cartesian_current', PoseStamped, self.cs_cb, queue_size=1)
        self._wrench_bod_sub = rospy.Subscriber('/ambf/' + arm_name + '/set_wrench_body', Wrench, self.wrench_bod_cb, queue_size=1)

        self._torque_pub = rospy.Publisher('/dvrk/' + arm_name + '/set_effort_joint', JointState, queue_size=5)

        self._wd = WatchDog(0.1)

        self.l4_o = np.identity(3)

        self.l5_o = np.mat([[0, 0, -1], [0, 1, 0], [1, 0, 0]])

        self.l6_o = np.mat([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

        self.l7_o = np.mat([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

        self.Kp = 0.3
        self.Kd = 0

        theta = 0
        self._rot_offset = np.mat([[+1.0, +0.0, +0.0],
                                   [+0.0, +np.cos(theta), +np.sin(theta)],
                                   [+0.0, -np.sin(theta), +np.cos(theta)]])

        self._ee_rot = Rotation()

    def set_k_gain(self, val):
        self.Kp = val

    def set_d_gain(self, val):
        self.Kd = val

    def jac_spa_cb(self, msg):
        self._new_data = True
        for r in range(0, self._num_rows):
            for c in range(0, self._num_cols):
                self._jac_spa[r][c] = msg.data[r + (6 * c)]
        pass

    def jac_bod_cb(self, msg):
        for r in range(0, self._num_rows):
            for c in range(0, self._num_cols):
                self._jac_bod[r][c] = msg.data[r + (6 * c)]
        pass

    def js_cb(self, msg):
        self._joint_state = msg

    def cs_cb(self, msg):
        self._cart_state = msg
        self._ee_rot = Rotation.Quaternion(msg.pose.orientation.x,
                                           msg.pose.orientation.y,
                                           msg.pose.orientation.z,
                                           msg.pose.orientation.w)

    def wrench_bod_cb(self, msg):
        force = np.zeros((3, 1))

        force[0] = msg.force.x
        force[1] = msg.force.y
        force[2] = msg.force.z

        force = np.matmul(self._rot_offset, force)

        rel_force = Vector(force[0], force[1], force[2])

        abs_force = self._ee_rot.Inverse() * rel_force

        print abs_force[0], abs_force[1], abs_force[2]

        self._wrench_cmd[0] = abs_force[0]
        self._wrench_cmd[1] = abs_force[1]
        self._wrench_cmd[2] = abs_force[2]

        self._wrench_cmd[3] = msg.torque.x
        self._wrench_cmd[4] = msg.torque.y
        self._wrench_cmd[5] = msg.torque.z

        self._wd.acknowledge_wd()

    def clear_wrench(self):
        self._wrench_cmd[0] = 0
        self._wrench_cmd[1] = 0
        self._wrench_cmd[2] = 0

        self._wrench_cmd[3] = 0
        self._wrench_cmd[4] = 0
        self._wrench_cmd[5] = 0

    def print_jacobian_spatial(self):
        jac = np.zeros(self._jac_spa.shape)
        for r in range(0, self._num_rows):
            for c in range(0, self._num_cols):
                jac[r][c] = round(self._jac_spa[r][c], 3)
        print(jac)

    def print_jacobian_body(self):
        jac = np.zeros(self._jac_bod.shape)
        for r in range(0, self._num_rows):
            for c in range(0, self._num_cols):
                jac[r][c] = round(self._jac_bod[r][c], 3)
        print(jac)

    def convert_force_to_torque_spa(self, force):
        torque = np.matmul(self._jac_spa.transpose(), force)
        return torque

    def convert_force_to_torque_bod(self, force):
        torque = np.matmul(self._jac_bod.transpose(), force)
        return torque

    def command_torques(self, torques):
        self._state_cmd.effort = torques.flatten().tolist()
        self._torque_pub.publish(self._state_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self._joint_torques_cmd = self.convert_force_to_torque_bod(self._wrench_cmd)

            r4 = rot_z(self._joint_state.position[3])
            r5 = rot_z(self._joint_state.position[4])
            r6 = rot_z(self._joint_state.position[5])
            r7 = rot_z(self._joint_state.position[6])

            re = self.l4_o * r4 * self.l5_o * r5 * self.l6_o * r6 * self.l7_o * r7
            ve = re[:, 2]
            v4 = r4[:, 0]

            e = (np.pi / 2) - np.arccos(np.dot(v4.transpose(), ve))
            tau4 = self.Kp * e - self.Kd * self._joint_state.velocity[3]
            np.clip(tau4, -0.3, 0.3)

            # self._joint_torques_cmd[3] = tau4[0, 0]
            self._joint_torques_cmd[3] = 0
            self._joint_torques_cmd[4] = 0
            self._joint_torques_cmd[5] = 0
            self._joint_torques_cmd[6] = 0

            # Apply the relation between J2 and J3
            self._joint_torques_cmd[1] = self._joint_torques_cmd[1]
            # self._joint_torques_cmd[1] = -self._joint_torques_cmd[1]
            self._joint_torques_cmd[2] = self._joint_torques_cmd[2]
            self.command_torques(self._joint_torques_cmd)

            if self._wd.is_wd_expired():
                self.clear_wrench()

            self._rate.sleep()


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()

    parser.add_argument('-a', action='store', dest='arm_name', help='Specify Arm Name',
                        default='MTMR')
    parser.add_argument('-k', action='store', dest='K_gain', help='Specify Linear Gain',
                        default=0.3)
    parser.add_argument('-d', action='store', dest='D_gain', help='Specify Damping Gain',
                        default=0.0)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    rospy.init_node('dvrk_arm_effort_controller')

    tjs = TaskToJointSpace(parsed_args.arm_name)
    tjs.set_k_gain(float(parsed_args.K_gain))
    tjs.set_d_gain(float(parsed_args.D_gain))

    print('Initialized')

    time.sleep(0.2)
    # tjs.print_jacobian_body()
    # tjs.print_jacobian_spatial()
    tjs.run()


if __name__ == "__main__":
    main()
