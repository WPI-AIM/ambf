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
from ambf_base_object import BaseObject
from geometry_msgs.msg import Pose, Wrench


class Object(BaseObject):
    def __init__(self, a_name, time_out=0.1):
        """
        Constructor
        :param a_name:
        """
        super(Object, self).__init__(a_name, time_out)  # Set duration of Watchdog expiry
        self.object_type = "DEFAULT_OBJECT"
        self.body_type = "DYNAMIC"
        self._wrench_cmd_set = False  # Flag to check if a Wrench command has been set from the Object

    def is_joint_idx_valid(self, joint_idx):
        """
        :param joint_idx:
        :return:
        """
        n_jnts = len(self._state.joint_positions)
        if joint_idx in range(n_jnts):
            return True
        else:
            # Index invalid
            print('ERROR! Requested Joint Idx of \"' + str(joint_idx) +
                  '\" outside valid range [0 - ' + str(n_jnts - 1) + ']')
            return False

    def get_joint_idx_from_name(self, joint_name):
        """
        :param joint_name:
        :return:
        """
        joint_names = self._state.joint_names
        if joint_name in joint_names:
            joint_idx = joint_names.index(joint_name)
            return joint_idx
        else:
            print('ERROR! Requested Joint \"' + str(joint_name) + '\" not found in list of joints:')
            print(joint_names)
            return None

    def get_joint_name_from_idx(self, joint_idx):
        """
        :param joint_idx:
        :return:
        """
        if self.is_joint_idx_valid(joint_idx):
            joint_name = self._state.joint_names[joint_idx]
            return joint_name

    def get_joint_pos(self, joint_name_or_idx):
        """
        Get the joint position of a specific joint at idx. Check joint names to see indexes
        :param joint_name_or_idx:
        :return:
        """
        if isinstance(joint_name_or_idx, str):
            joint_idx = self.get_joint_idx_from_name(joint_name_or_idx)
        else:
            joint_idx = joint_name_or_idx

        if self.is_joint_idx_valid(joint_idx):
            return self._state.joint_positions[joint_idx]
        else:
            return None

    def get_all_joint_pos(self):
        """
                Get the joint position of a specific joint at idx. Check joint names to see indexes
                :param idx:
                :return:
                """
        n_jnts = len(self._state.joint_positions)
        joints = []
        for idx in range(n_jnts):
            joints.append(self._state.joint_positions[idx])

        return joints

    def get_num_joints(self):
        """
        Get the number of joints for this object
        :return:
        """
        return len(self._state.joint_positions)

    def get_joint_names(self):
        """
        Get the joint names if any for this object. Make sure the joint name reporting is enabled in the
        AMBF Config file by setting the "publish joint names: True" in the object description
        :return:
        """
        jnt_names = self._state.joint_names
        return jnt_names

    def get_force_command(self):
        """
        Get the commanded force of this object
        :return:
        """
        return self._cmd.wrench.force

    def get_torque_command(self):
        """
        Get the commanded torque of this object
        :return:
        """
        return self._cmd.wrench.torque

    def get_publish_children_name_flag(self):
        """
        Check if the object is publishing children names
        :return:
        """
        return self._cmd.publish_children_names

    def get_publish_joint_names_flag(self):
        """
        Check if the object is publishing joint names
        :return:
        """
        return self._cmd.publish_joint_names

    def get_publish_joint_positions_flag(self):
        """
        Check if the object is publishing joint poisitions
        :return:
        """
        return self._cmd.publish_joint_positions

    def set_publish_children_names_flag(self, state):
        """
        Set children name publishing state
        """
        self._cmd.publish_children_names = state

    def set_publish_joint_names_flag(self, state):
        """
        set joint name publishing state
        """
        self._cmd.publish_joint_names = state

    def set_publish_joint_positions_flag(self, state):
        """
        set joint position publishing state
        """
        self._cmd.publish_joint_positions = state

    def set_active(self):
        """Mark this object as active"""
        self._active = True

    def get_inertia(self):
        """
        Get the inertia the body
        """
        return self._state.pInertia

    def set_joint_pos(self, joint_name_or_idx, q):
        """
        Set the joint position based on the index or names. Check the get_joint_names to see the list of
        joint names for indexes
        :param joint_name_or_idx:
        :param q:
        :return:
        """
        # edited python3 code
        if isinstance(joint_name_or_idx, str):
            joint_idx = self.get_joint_idx_from_name(joint_name_or_idx)
        else:
            joint_idx = joint_name_or_idx

        if self.is_joint_idx_valid(joint_idx):
            n_jnts = self.get_num_joints()
            if len(self._cmd.joint_cmds) != n_jnts:
                self._cmd.joint_cmds = [0.0]*n_jnts
                self._cmd.position_controller_mask = [0]*n_jnts

            self._cmd.joint_cmds[joint_idx] = q
            self._cmd.position_controller_mask[joint_idx] = True
            self._apply_command()

    def set_force(self, fx, fy, fz):
        """
        Set the Force for of this object in parent frame. If a previous Wrench command had been
        set, the torque from that command will be used
        :param fx:
        :param fy:
        :param fz:
        :return:
        """
        _wrench_cmd = Wrench()
        _wrench_cmd.force.x = fx
        _wrench_cmd.force.y = fy
        _wrench_cmd.force.z = fz
        _wrench_cmd.torque = self.get_torque_command()
        self.set_wrench(_wrench_cmd)

    def set_torque(self, nx, ny, nz):
        """
        Set the Torque for of this object in parent frame. If a previous Wrench command had been
        set, the force from that command will be used
        :param nx:
        :param ny:
        :param nz:
        :return:
        """
        _wrench_cmd = Wrench()
        _wrench_cmd.force = self.get_force_command()
        _wrench_cmd.torque.x = nx
        _wrench_cmd.torque.y = ny
        _wrench_cmd.torque.z = nz
        self.set_wrench(_wrench_cmd)

    def set_joint_effort(self, joint, effort):
        """
        Set the joint effort based on the index or name. Check the get_joint_names to see the list of
        joint names for indexes
        :param joint:
        :param effort:
        :return:
        """
        # edited python3 code
        if isinstance(joint, str):
            # Initial code for python2
            # if isinstance(joint, basestring):

            joint_names = self._state.joint_names
            if joint not in joint_names:
                print(joint + " is not a joint")
                return
            idx = joint_names.index(joint)
        else:
            idx = joint

        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print('Requested Joint Index ' + str(idx) + ' outside valid range [0 - ' + str(n_jnts - 1) + ']')
            return

        if len(self._cmd.joint_cmds) != n_jnts:
            self._cmd.joint_cmds = [0.0] * n_jnts
            self._cmd.position_controller_mask = [0]*n_jnts

        self._cmd.joint_cmds[idx] = effort
        self._cmd.position_controller_mask[idx] = False

        self._apply_command()

    def set_wrench(self, wrench):
        """
        Set the wrench for this object in the parent frame
        :param wrench:
        :return:
        """
        self._cmd.enable_position_controller = False
        self._cmd.wrench = wrench

        self._apply_command()
        self._wrench_cmd_set = True

    def pose_command(self, px, py, pz, roll, pitch, yaw, *jnt_cmds):
        """
        Same as set_pose but customized of OpenAI's GYM for a single call set method
        :param px:
        :param py:
        :param pz:
        :param roll:
        :param pitch:
        :param yaw:
        :param jnt_cmds:
        :return:
        """
        # Edited python3 code
        quat = quaternion_from_euler(roll, pitch, yaw, 'szyx')
        # Initial python2 code
        # quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'szyx')
        self._cmd.enable_position_controller = True
        self._cmd.pose.position.x = px
        self._cmd.pose.position.y = py
        self._cmd.pose.position.z = pz
        self._cmd.pose.orientation.x = quat[0]
        self._cmd.pose.orientation.y = quat[1]
        self._cmd.pose.orientation.z = quat[2]
        self._cmd.pose.orientation.w = quat[3]

        self._cmd.joint_cmds = [jnt for jnt in jnt_cmds]

        self._apply_command()

    def wrench_command(self, fx, fy, fz, nx, ny, nz):
        """
        Same as set_wrench but customized for OpenAI's GYM for a single call method
        :param fx:
        :param fy:
        :param fz:
        :param nx:
        :param ny:
        :param nz:
        :return:
        """
        self._cmd.enable_position_controller = False
        self._cmd.wrench.force.x = fx
        self._cmd.wrench.force.y = fy
        self._cmd.wrench.force.z = fz
        self._cmd.wrench.torque.x = nx
        self._cmd.wrench.torque.y = ny
        self._cmd.wrench.torque.z = nz

        self._apply_command()
        self._wrench_cmd_set = True

    def _clear_command(self):
        """
        Clear wrench if watchdog is expired
        :return:
        """
        self._cmd.wrench.force.x = 0
        self._cmd.wrench.force.y = 0
        self._cmd.wrench.force.z = 0
        self._cmd.wrench.torque.x = 0
        self._cmd.wrench.torque.y = 0
        self._cmd.wrench.torque.z = 0
