#!/usr/bin/env python
from tf import transformations
from ambf_msgs.msg import ObjectState
from ambf_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy


class Object(WatchDog):
    def __init__(self, a_name):
        super(Object, self).__init__()
        self._name = ''
        self._state = ObjectState()
        self._cmd = ObjectCmd()
        self._pub = None
        self._sub = None
        self.pub_flag = True
        self._active = False

    def ros_cb(self, data):
        self._state = data

    def is_active(self):
        return self._active

    def set_active(self):
        self._active = True

    def set_pos(self, px, py, pz):
        self._cmd.enable_position_controller = True
        self._cmd.pose.position.x = px
        self._cmd.pose.position.y = py
        self._cmd.pose.position.z = pz

        self._cmd.pose.orientation = self._state.pose.orientation

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def set_rpy(self, roll, pitch, yaw):
        self._cmd.enable_position_controller = True
        quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'szyx')
        self.set_rot(quat)

    def set_rot(self, quat):
        self._cmd.enable_position_controller = True
        self._cmd.pose.orientation.x = quat[0]
        self._cmd.pose.orientation.y = quat[1]
        self._cmd.pose.orientation.z = quat[2]
        self._cmd.pose.orientation.w = quat[3]

        self._cmd.pose.position = self._state.pose.position

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def set_pose(self, pose):
        self._cmd.enable_position_controller = True
        self._cmd.pose = pose

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def set_joint_pos(self, idx, pos):
        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Joint Index {} should be between 0-{}'.format(idx, n_jnts)
            return

        if len(self._cmd.joint_cmds) != n_jnts:
            self._cmd.joint_cmds = [0]*n_jnts
            for j_idx in range(0, n_jnts):
                self._cmd.joint_cmds[j_idx] = 0.0
            self._cmd.position_controller_mask = [0]*n_jnts

        self._cmd.joint_cmds[idx] = pos
        self._cmd.position_controller_mask[idx] = True

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def set_joint_effort(self, idx, effort):
        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Joint Index %s should be between 0-%s'.format(idx, n_jnts)
            return

        if len(self._cmd.joint_cmds) != n_jnts:
            self._cmd.joint_cmds = [0] * n_jnts
            for j_idx in range(0, n_jnts):
                self._cmd.joint_cmds[j_idx] = 0.0
            self._cmd.position_controller_mask = [0]*n_jnts

        self._cmd.joint_cmds[idx] = effort
        self._cmd.position_controller_mask[idx] = False

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def get_joint_pos(self, idx, pos):
        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Joint Index %s should be between 0-%s'.format(idx, n_jnts)
            return

        return self._state.joint_positions[idx]

    def get_num_joints(self):
        return len(self._state.joint_positions)

    def get_num_children(self):
        return len(self._state.children_names)

    def get_joint_names(self):
        jnt_names = self._state.joint_names
        return jnt_names

    def get_children_names(self):
        chldrn_names = self._state.children_names
        return chldrn_names

    def pose_command(self, px, py, pz, roll, pitch, yaw, *jnt_cmds):
        quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'szyx')
        self._cmd.enable_position_controller = True
        self._cmd.pose.position.x = px
        self._cmd.pose.position.y = py
        self._cmd.pose.position.z = pz
        self._cmd.pose.orientation.x = quat[0]
        self._cmd.pose.orientation.y = quat[1]
        self._cmd.pose.orientation.z = quat[2]
        self._cmd.pose.orientation.w = quat[3]

        self._cmd.joint_cmds = [jnt for jnt in jnt_cmds]
        self._cmd.header.stamp = rospy.Time.now()

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def set_force(self, fx, fy, fz):
        nx = self._cmd.wrench.torque.x
        ny = self._cmd.wrench.torque.y
        nz = self._cmd.wrench.torque.z
        self.wrench_command(fx, fy, fz, nx, ny, nz)

    def set_torque(self, nx, ny, nz):
        fx = self._cmd.wrench.force.x
        fy = self._cmd.wrench.force.y
        fz = self._cmd.wrench.force.z
        self.wrench_command(fx, fy, fz, nx, ny, nz)

    def wrench_command(self, fx, fy, fz, nx, ny, nz):
        self._cmd.enable_position_controller = False
        self._cmd.wrench.force.x = fx
        self._cmd.wrench.force.y = fy
        self._cmd.wrench.force.z = fz
        self._cmd.wrench.torque.x = nx
        self._cmd.wrench.torque.y = ny
        self._cmd.wrench.torque.z = nz

        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def get_sim_step(self):
        return self._state.sim_step

    def clear_cmd(self):
        self._cmd.wrench.force.x = 0
        self._cmd.wrench.force.y = 0
        self._cmd.wrench.force.z = 0
        self._cmd.wrench.torque.x = 0
        self._cmd.wrench.torque.y = 0
        self._cmd.wrench.torque.z = 0

    def get_pos(self):
        return self._state.pose.position

    def get_rot(self):
        return self._state.pose.orientation

    def get_rpy(self):
        quat = self._state.pose.orientation
        rpy = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return rpy

    def get_pose(self):
        quat = self._state.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        rpy = transformations.euler_from_quaternion(explicit_quat, 'szyx')
        pose = [self._state.pose.position.x,
                self._state.pose.position.y,
                self._state.pose.position.z,
                rpy[0],
                rpy[1],
                rpy[2]]
        return pose

    def set_name(self, name):
        self._name = name

    def get_name(self):
        return self._name

    def run_publisher(self):
        if self.pub_flag:
            if self.is_wd_expired():
                self.console_print(self._name)
                self.clear_cmd()
            self._pub.publish(self._cmd)

