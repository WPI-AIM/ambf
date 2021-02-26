#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020, AMBF
#     (https://github.com/WPI-AIM/ambf)
#
#     All rights reserved.
#
#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:
#
#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.
#
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped, Wrench, Vector3
from sensor_msgs.msg import Joy, JointState
# rom geomagic_control.msg import DeviceFeedback, DeviceButtonEvent
# from phantom_omni.msg import DeviceFeedback, DeviceButtonEvent
from PyKDL import Frame, Vector, Rotation
import sys
import time

global _gripper_state
_gripper_state = True


# Utilities
def kdl_frame_to_msg_pose(kdl_pose):
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def msg_pose_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f


# Init Relevant MTM
class ProxyMTM:
    def __init__(self, arm_name):
        pose_str = '/dvrk/' + arm_name + '/position_cartesian_current'
        wrench_str = '/dvrk/' + arm_name + '/set_wrench_body'
        gripper_str = '/dvrk/' + arm_name + '/state_gripper_current'
        status_str = '/dvrk/' + arm_name + '/status'

        self.base_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.tip_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))

        self._mtm_arm_type = None

        if arm_name == 'MTMR':
            self._mtm_arm_type = 0
            self.base_frame.M = Rotation.RPY(0.0, 0, 1.57079)
            self.tip_frame.M = Rotation.RPY(0, -1.57079, 0)
        elif arm_name == 'MTML':
            self._mtm_arm_type = 1
            self.base_frame.M = Rotation.RPY(0.0, 0, 1.57079)
            self.tip_frame.M = Rotation.RPY(0, -1.57079, 0)
        else:
            print('SPECIFIED ARM: ', arm_name)
            print('WARNING, MTM ARM TYPE NOT UNDERSTOOD, SHOULD BE MTMR or MTML')
        pass

        self.cur_frame = Frame()
        self.cur_frame.p = Vector(0, 0, 0)
        self.cur_frame.M = Rotation.Quaternion(0, 0, 0, 1)
        self.buttons = 0

        # ###################add

        self._commanded_force = Vector(0, 0, 0)

        self._gripper_min_angle = -3.16
        self._gripper_max_angle = 1.2
        self._gripper_angle = JointState()
        self._gripper_angle.position.append(0)

        self._pose_pub = rospy.Publisher(pose_str, PoseStamped, queue_size=1)
        self._gripper_pub = rospy.Publisher(gripper_str, JointState, queue_size=1)
        self._status_pub = rospy.Publisher(status_str, Empty, queue_size=1)

        self._force_sub = rospy.Subscriber(wrench_str, Wrench, self.force_cb, queue_size=10)
        pass

    def force_cb(self, msg):
        self._commanded_force[0] = msg.force.x
        self._commanded_force[1] = msg.force.y
        self._commanded_force[2] = msg.force.z

    def set_base_frame(self, frame):
        self.base_frame = frame
        pass

    def set_tip_frame(self, frame):
        self.tip_frame = frame
        pass

    def set_gripper_angle(self, angle):
        global _gripper_state

        range = self._gripper_max_angle - self._gripper_min_angle
        self._gripper_angle.position[0] = self._gripper_min_angle + range * angle
        self._gripper_pub.publish(self._gripper_angle)

    def get_commanded_force(self):
        return self._commanded_force

    def set_pos(self, a, b, c):
        self.cur_frame.p = Vector(a, b, c)
        pose = self.base_frame.Inverse() * self.cur_frame * self.tip_frame
        msg = kdl_frame_to_msg_pose(pose)
        self._pose_pub.publish(msg)

# used for rotate the gripper
    def set_angle(self, a, b, c):
        self.cur_frame.M = Rotation.RPY(a, b, c)
        pose = self.base_frame.Inverse() * self.cur_frame * self.tip_frame
        msg = kdl_frame_to_msg_pose(pose)
        self._pose_pub.publish(msg)

    def get_pose(self):
        return self.cur_frame

    def publish_status(self):
        self._status_pub.publish(Empty())

    def test_angle(self):
        min_a = -1.57
        max_a = 1.57
        span = max_a - min_a
        steps = 100
        sleep_s = 0.1

        print 'Testing Roll'
        for i in range(steps):
            val = min_a + span * i / steps
            self.set_angle(val, 0, 0)
            print('Angle', val)
            time.sleep(sleep_s)

        print 'Testing Pitch'
        for i in range(steps):
            val = min_a + span * i / steps
            self.set_angle(0, val, 0)
            print('Angle', val)
            time.sleep(sleep_s)

        print 'Testing Yaw'
        for i in range(steps):
            val = min_a + span * i / steps
            self.set_angle(0, 0, val)
            print('Angle', val)
            time.sleep(sleep_s)
