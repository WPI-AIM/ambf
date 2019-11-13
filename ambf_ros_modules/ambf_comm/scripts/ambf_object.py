#!/usr/bin/env python
#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019, AMBF
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

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

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
from tf import transformations
from ambf_msgs.msg import ObjectState
from ambf_msgs.msg import ObjectCmd
from watch_dog import WatchDog
import rospy
from geometry_msgs.msg import Pose, Wrench


class Object(WatchDog):
    def __init__(self, a_name):
        """
        Constructor
        :param a_name:
        """
        super(Object, self).__init__(time_out=0.1)  # Set duration of Watchdog expiry
        self._name = ''
        self._state = ObjectState()
        self._cmd = ObjectCmd()
        self._pub = None
        self._sub = None
        self.pub_flag = True
        self._active = False
        self._pose_cmd_set = False  # Flag to check if a Pose command has been set from the Object
        self._wrench_cmd_set = False  # Flag to check if a Wrench command has been set from the Object

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

    def get_joint_pos(self, idx):
        """
        Get the joint position of a specific joint at idx. Check joint names to see indexes
        :param idx:
        :return:
        """
        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Requested Joint Index ' + str(idx) + ' outside valid range [0 - ' + str(n_jnts - 1) + ']'
            return

        return self._state.joint_positions[idx]

    def get_all_joint_pos(self):
        """
                Get the joint position of a specific joint at idx. Check joint names to see indexes
                :param idx:
                :return:
                """
        n_jnts = len(self._state.joint_positions)
        joints = []
        for idx in xrange(n_jnts):
            joints.append(self._state.joint_positions[idx])

        return joints

    def get_num_joints(self):
        """
        Get the number of joints for this object
        :return:
        """
        return len(self._state.joint_positions)

    def get_num_of_children(self):
        """
        Get the number of children that this object has. Make sure the children reporting is enabled in the
        AMBF Config file by setting the "publish children names: True" in the object description
        :return:
        """
        return len(self._state.children_names)

    def get_joint_names(self):
        """
        Get the joint names if any for this object. Make sure the joint name reporting is enabled in the
        AMBF Config file by setting the "publish joint names: True" in the object description
        :return:
        """
        jnt_names = self._state.joint_names
        return jnt_names

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
        rpy = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return rpy

    def get_pose(self):
        """
        Get the pose as Geometry_msgs/Pose of this object in it's parent frame
        :return:
        """
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

    def get_name(self):
        """
        Get the name of this object
        :return:
        """
        return self._name

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
        quat = transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
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

    def set_joint_pos(self, joint, pos):
        """
        Set the joint position based on the index or names. Check the get_joint_names to see the list of
        joint names for indexes
        :param joint:
        :param pos:
        :return:
        """

        if isinstance(joint, basestring):

            joint_names = self._state.joint_names
            if joint not in joint_names:
                print joint + " is not a joint"
            idx = joint_names.index(joint)
        else:
            idx = joint

        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Requested Joint Index ' + str(idx) + ' outside valid range [0 - ' + str(n_jnts - 1) + ']'
            return

        if len(self._cmd.joint_cmds) != n_jnts:
            self._cmd.joint_cmds = [0.0]*n_jnts
            self._cmd.position_controller_mask = [0]*n_jnts

        self._cmd.joint_cmds[idx] = pos
        self._cmd.position_controller_mask[idx] = True

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

        if isinstance(joint, basestring):

            joint_names = self._state.joint_names
            if joint not in joint_names:
                print joint +  " is not a joint"
                return
            idx = joint_names.index(joint)
        else:
            idx = joint

        n_jnts = len(self._state.joint_positions)

        if not 0 <= idx < n_jnts:
            # Index invalid
            print 'Requested Joint Index ' + str(idx) + ' outside valid range [0 - ' + str(n_jnts - 1) + ']'
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
        Clear wrench if watchdog is expired
        :return:
        """
        self._cmd.wrench.force.x = 0
        self._cmd.wrench.force.y = 0
        self._cmd.wrench.force.z = 0
        self._cmd.wrench.torque.x = 0
        self._cmd.wrench.torque.y = 0
        self._cmd.wrench.torque.z = 0

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

    def get_inertia(self):
        """
        Get the inertia the body
        """
        return self._state.pInertia