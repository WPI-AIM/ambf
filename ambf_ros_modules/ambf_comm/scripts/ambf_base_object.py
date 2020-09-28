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

from transformations import quaternion_from_euler, euler_from_quaternion
from ambf_msgs.msg import ObjectState
from ambf_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy
from geometry_msgs.msg import Pose, Wrench


class BaseObject(WatchDog):
    def __init__(self, a_name, time_out):
        """
        Constructor
        :param a_name:
        """
        super(BaseObject, self).__init__(time_out)  # Set duration of Watchdog expiry
        self._name = a_name
        self._state = None
        self._cmd = None
        self._pub = None
        self._sub = None
        self.pub_flag = True
        self._active = False
        self._pose_cmd_set = False  # Flag to check if a Pose command has been set from the Object
        self.object_type = "BASE_OBJECT"
        self.body_type = "KINEMATIC"

    def ros_cb(self, data):
        """
        Call function for ROS topics
        :param data:
        :return:
        """
        self._state = data

    def is_active(self):
        """
        Flag to check if the cb for this Object is active or not
        :return:
        """
        return self._active

    def get_sim_step(self):
        """
        The step of AMBF Simulator
        :return:
        """
        return self._state.sim_step

    def get_num_of_children(self):
        """
        Get the number of children that this object has. Make sure the children reporting is enabled in the
        AMBF Config file by setting the "publish children names: True" in the object description
        :return:
        """
        return len(self._state.children_names)

    def get_children_names(self):
        """
        Get the name of children of this object. Make sure the children reporting is enabled in the
        AMBF Config file by setting the "publish children names: True" in the object description
        :return:
        """
        children_names = self._state.children_names
        return children_names

    def get_pos(self):
        """
        Get the position in the parent frame for this object in parent frame
        :return:
        """
        return self._state.pose.position

    def get_rot(self):
        """
        Get the rotation as quaternion for this object in parent frame
        :return:
        """
        return self._state.pose.orientation

    def get_rpy(self):
        """
        Get the rotation as Fixed RPY for this object
        :return:
        """
        quat = self._state.pose.orientation
        # Edited python3 code
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # Initial python2 code
        # rpy = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return rpy

    def get_pose(self):
        """
        Get the pose as Geometry_msgs/Pose of this object in it's parent frame
        :return:
        """
        quat = self._state.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        # Edited python3 code
        rpy = euler_from_quaternion(explicit_quat, 'szyx')
        # Initial python2 code
        # rpy = transformations.euler_from_quaternion(explicit_quat, 'szyx')
        pose = [self._state.pose.position.x,
                self._state.pose.position.y,
                self._state.pose.position.z,
                rpy[0],
                rpy[1],
                rpy[2]]
        return pose

    def get_pos_command(self):
        """
        Get the commanded position of this object
        :return:
        """
        if self._pose_cmd_set:
            return self._cmd.pose.position
        else:
            return self._state.pose.position

    def get_rot_command(self):
        """
        Get the rotation command of this object
        :return:
        """
        if self._pose_cmd_set:
            return self._cmd.pose.orientation
        else:
            return self._state.pose.orientation

    def get_name(self):
        """
        Get the name of this object
        :return:
        """
        return self._name

    def get_parent_name(self):
        """
        Get the parents name if any
        :return:
        """
        return self._state.parent_name

    def set_name(self, name):
        """
        Set the name for this object
        :param name:
        :return:
        """
        self._name = name

    def set_active(self):
        """Mark this object as active"""
        self._active = True

    def set_pos(self, px, py, pz):
        """
        Set the Position of this object in parent frame. If a previous Pose command had been
        set, the orientation from that command will be used, else, the orientation of the actual
        object that is retrieved from it's state shall be used
        :param px:
        :param py:
        :param pz:
        :return:
        """
        _pose_cmd = Pose()
        _pose_cmd.position.x = px
        _pose_cmd.position.y = py
        _pose_cmd.position.z = pz
        _pose_cmd.orientation = self.get_rot_command()

        self.set_pose(_pose_cmd)

    def set_rpy(self, roll, pitch, yaw):
        """
        Set the Rotation in RPY of this object in parent frame. If a previous Pose command had been
        set, the position from that command will be used, else, the position of the actual
        object that is retrieved from it's state shall be used
        :param roll:
        :param pitch:
        :param yaw:
        :return:
        """
        # Edited python3 code
        quat = quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        # Initial python2 code
        # quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        self.set_rot(quat)

    def set_rot(self, quat):
        """
        Set the Rotation in Quaternion of this object in parent frame. If a previous Pose command had been
        set, the position from that command will be used, else, the position of the actual
        object that is retrieved from it's state shall be used
        :param quat:
        :return:
        """
        _pose_cmd = Pose()
        _pose_cmd.position = self.get_pos_command()
        _pose_cmd.orientation.x = quat[0]
        _pose_cmd.orientation.y = quat[1]
        _pose_cmd.orientation.z = quat[2]
        _pose_cmd.orientation.w = quat[3]

        self.set_pose(_pose_cmd)

    def set_pose(self, pose):
        """
        Set the pose of this object in parent frame
        :param pose:
        :return:
        """
        self._cmd.enable_position_controller = True
        self._cmd.pose = pose

        self._apply_command()
        self._pose_cmd_set = True

    def _apply_command(self):
        """
        Internal function to synchronized with the publisher and update watchdog
        :return:
        """
        self._cmd.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def _clear_command(self):
        """
        Clear wrench if watchdog is expired. Children should override this method
        :return:
        """

    def run_publisher(self):
        """
        Run the publisher in a thread
        :return:
        """
        if self.pub_flag:
            if self.is_wd_expired():
                # self.console_print(self._name)
                self._clear_command()
            self._pub.publish(self._cmd)
