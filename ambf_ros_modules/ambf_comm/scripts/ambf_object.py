#!/usr/bin/env python
from tf import transformations
from geometry_msgs.msg import Pose
from ambf_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy


class Object(WatchDog):
    def __init__(self, a_name):
        super(Object, self).__init__()
        self.time_stamp = []
        self.sim_step = 0
        self.name = a_name
        self.pose = Pose()
        self.cmd = ObjectCmd()
        self.pub = None
        self.sub = None
        self.pub_flag = True
        self._active = False

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def ros_cb(self, data):
        self.name = data.name.data
        self.pose = data.pose
        self.time_stamp = data.header.stamp
        self.sim_step = data.sim_step

    def pose_command(self, px, py, pz, roll, pitch, yaw, *jnt_cmds):
        quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'szyx')
        self.cmd.pos_ctrl = True
        self.cmd.pose.position.x = px
        self.cmd.pose.position.y = py
        self.cmd.pose.position.z = pz
        self.cmd.pose.orientation.x = quat[0]
        self.cmd.pose.orientation.y = quat[1]
        self.cmd.pose.orientation.z = quat[2]
        self.cmd.pose.orientation.w = quat[3]

        self.cmd.joint_cmds = [jnt for jnt in jnt_cmds]
        self.cmd.header.stamp = rospy.Time.now()

        self.pub.publish(self.cmd)
        self.acknowledge_wd()

    def wrench_command(self, fx, fy, fz, nx, ny, nz):
        self.cmd.wrench.force.x = fx
        self.cmd.wrench.force.y = fy
        self.cmd.wrench.force.z = fz
        self.cmd.wrench.torque.x = nx
        self.cmd.wrench.torque.y = ny
        self.cmd.wrench.torque.z = nz

        self.pub.publish(self.cmd)
        self.acknowledge_wd()

    def get_sim_step(self):
        return self.sim_step

    def clear_cmd(self):
        self.cmd.wrench.force.x = 0
        self.cmd.wrench.force.y = 0
        self.cmd.wrench.force.z = 0
        self.cmd.wrench.torque.x = 0
        self.cmd.wrench.torque.y = 0
        self.cmd.wrench.torque.z = 0

    def get_pose(self):
        quat = self.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        rpy = transformations.euler_from_quaternion(explicit_quat, 'szyx')
        pose = [self.pose.position.x,
                self.pose.position.y,
                self.pose.position.z,
                rpy[0],
                rpy[1],
                rpy[2]]
        return pose

    def run_publisher(self):
        if self.pub_flag:
            if self.is_wd_expired():
                self.console_print(self.name)
                self.clear_cmd()
            self.pub.publish(self.cmd)

