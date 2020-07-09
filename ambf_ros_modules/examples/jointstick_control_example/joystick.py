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

from ambf_client import Client
import time
import rospy
from sensor_msgs.msg import Joy
from itertools import cycle
from argparse import ArgumentParser


# Class that maps the axes and buttons. Change the attributes to alter direction and button configuration
class JSConfiguration:
    def __init__(self):
        # These indexes are used to map to specific axes of the joystick
        # 1
        self.x_axes_idx = 1
        self.y_axes_idx = 0
        self.z_axes_idx = 3

        # 2
        self.ro_axes_idx = 0
        self.pi_axes_idx = 1
        self.ya_axes_idx = 3

        # Direction of Axes. Should be either 1.0 or -1.0
        # 3
        self.x_axes_dir = 1.0
        self.y_axes_dir = -1.0
        self.z_axes_dir = -1.0

        # 4
        self.ro_axes_dir = 1.0
        self.pi_axes_dir = 1.0
        self.ya_axes_dir = 1.0

        # If Joystick has less than 6 axes, we should have to switch between controlling position and orientation
        # If you Joystick has has 6 axes. Set this to 6 and alter #2 and optionally #4.
        self.num_axes = 3

        # Button idx that switches between Pos and Rot control if JoyStick
        # doesn't have 6 Axes
        self.input_mode_switch_button_idx = 0

        # Button to switch between Force (F)/ Position (P) and Velocity (V) control mode
        self.control_mode_switch_button_idx = 1


# Init everything related to SpaceNav
class JoyStickDevice:
    # The name should include the full qualified prefix. I.e.
    def __init__(self, joystick_topic=None):
        if joystick_topic:
            topic_name = joystick_topic
        else:
            topic_name = '/joy'

        self._active = False
        self._scale = 0.001
        self.grey_button_pressed = False
        self.white_button_pressed = False

        self.input_iterator = cycle(range(2))
        self.input_mode = self.input_iterator.next()

        self.control_iterator = cycle(range(3))
        self.control_mode = self.control_iterator.next()

        # Organized as x,y,z, roll, pitch, yaw
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.ro = 0.0
        self.pi = 0.0
        self.ya = 0.0

        self.x_pre = 0.0
        self.y_pre = 0.0
        self.z_pre = 0.0

        self.ro_pre = 0.0
        self.pi_pre = 0.0
        self.ya_pre = 0.0

        self.jsc = JSConfiguration()

        self._pose_sub = rospy.Subscriber(topic_name, Joy, self.joy_cb, queue_size=10)

        self._msg_counter = 0

    def joy_cb(self, msg):
        if self.jsc.num_axes >= 6:
            self.x = msg.axes[self.jsc.x_axes_idx]
            self.y = msg.axes[self.jsc.y_axes_idx]
            self.z = msg.axes[self.jsc.z_axes_idx]
            self.ro = msg.axes[self.jsc.ro_axes_idx]
            self.pi = msg.axes[self.jsc.pi_axes_idx]
            self.ya = msg.axes[self.jsc.ya_axes_idx]

        else:
            # Switch between controlling Position and Orientation based on
            # a button trigger
            if self.input_mode == 0: # Change Position on Axes callback
                # Process Rotation
                # Process Position
                self.x_pre = self.x
                self.y_pre = self.y
                self.z_pre = self.z

                self.ro_pre = self.ro
                self.ro_pre = self.pi
                self.ro_pre = self.ya

                self.x = msg.axes[self.jsc.x_axes_idx] * self.jsc.x_axes_dir
                self.y = msg.axes[self.jsc.y_axes_idx] * self.jsc.y_axes_dir
                self.z = msg.axes[self.jsc.z_axes_idx] * self.jsc.z_axes_dir

                self.ro = 0.0
                self.pi = 0.0
                self.ya = 0.0
            elif self.input_mode == 1: # Change Orientation on Axes callback
                # Process Position
                self.x_pre = self.x
                self.y_pre = self.y
                self.z_pre = self.z

                self.ro_pre = self.ro
                self.ro_pre = self.pi
                self.ro_pre = self.ya

                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

                self.ro = msg.axes[self.jsc.ro_axes_idx] * self.jsc.ro_axes_dir
                self.pi = msg.axes[self.jsc.pi_axes_idx] * self.jsc.pi_axes_dir
                self.ya = msg.axes[self.jsc.ya_axes_idx] * self.jsc.ya_axes_dir

        if msg.buttons[self.jsc.input_mode_switch_button_idx]:
            self.input_mode = self.input_iterator.next()
            print('SWITCHING INPUT MODE (0: POSITION, 1: ORIENTATION) TO ', self.input_mode)

        if msg.buttons[self.jsc.control_mode_switch_button_idx]:
            self.control_mode = self.control_iterator.next()
            print('SWITCHING CONTROL MODE (0: FORCE, 1: POSE, 2: VELOCITY) TO ', self.control_mode)

        self._active = True
        pass


class ObjectControl:
    def __init__(self, obj_name, joystick_ifc, client_name):
        if client_name:
            self.client = Client(client_name)
        else:
            self.client = Client()
        self.client.connect()
        time.sleep(0.3)
        self.obj_handle = self.client.get_obj_handle(obj_name)
        time.sleep(0.3)

        if self.obj_handle.object_type == 'RIGID_BODY':
            self._wrench_supported = True
            self._pose_supported = True
            self._twist_supported = True
        else:
            self._wrench_supported = False
            self._pose_supported = True
            self._twist_supported = False

        print('The Object is of type', self.obj_handle.object_type)
        print('\tWrench Supported ?', self._wrench_supported)
        print('\tPose Supported ?', self._pose_supported)
        print('\tTwist Supported ?', self._twist_supported)

        # Scaling for F/P/V modes. Change at will
        self.force_scale = 500
        self.pose_scale = 1
        self.velocity_scale = 10

        self.control_ifc = joystick_ifc

    def scale_input(self, joystick, scale):
        x = joystick.x * scale
        y = joystick.y * scale
        z = joystick.z * scale
        ro = joystick.ro * scale
        pi = joystick.pi * scale
        ya = joystick.ya * scale

        return x, y, z, ro, pi, ya

    def run(self):
        while not rospy.is_shutdown():

            if self.control_ifc.control_mode == 0: # Force Control
                if self._wrench_supported:
                    x, y, z, ro, pi, ya = self.scale_input(self.control_ifc, self.force_scale)
                    self.obj_handle.set_force(x, y, z)
                    self.obj_handle.set_torque(ro, pi, ya)
            elif self.control_ifc.control_mode == 1: # Position Control
                if self._pose_supported:
                    x, y, z, ro, pi, ya = self.scale_input(self.control_ifc, self.pose_scale)
                    self.obj_handle.set_pos(x, y, z)
                    self.obj_handle.set_rpy(ro, pi, ya)
            elif self.control_ifc.control_mode == 2: # Velocity Control
                if self._twist_supported:
                    x, y, z, ro, pi, ya = self.scale_input(self.control_ifc, self.velocity_scale)
                    self.obj_handle.set_linear_vel(x, y, z)
                    self.obj_handle.set_angular_vel(ro, pi, ya)
            else:
                print('CANNOT UNDERSTAND CARTESIAN CONTROL MODE. SUPPORTED MODES ARE 0, 1, 2 FOR F, P, V')

            time.sleep(0.001)


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-o', action='store', dest='obj_name', help='Specify AMBF Obj Name')
    parser.add_argument('-j', action='store', dest='joy_topic_name', help='Specify Joystick Topic name', default='/joy')
    parser.add_argument('-a', action='store', dest='client_name', help='Client Name. If running multiple instances of '
                                                                       'client', default=None)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    joystick = JoyStickDevice(joystick_topic=parsed_args.joy_topic_name)
    oc = ObjectControl(parsed_args.obj_name, joystick, parsed_args.client_name)

    oc.run()


if __name__ == '__main__':
    main()
