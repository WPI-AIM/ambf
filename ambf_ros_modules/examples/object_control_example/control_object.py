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
from obj_control_gui import ObjectGUI
from jnt_control_gui import JointGUI
import rospy
from argparse import ArgumentParser


class ObjectControl:
    def __init__(self, obj_name, client_name, c_space_ctrl, j_space_ctrl, initial_xyz=None,
                 initial_rpy=None, range_xyz=None, range_rpy=None, resolution=None):
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

        self._ctrl_c_space = c_space_ctrl
        self._ctrl_j_space = j_space_ctrl

        if self._ctrl_c_space is True:
            self.obj_gui = ObjectGUI(obj_name, initial_xyz, initial_rpy, range_xyz, range_rpy, resolution)

        self._n_jnts = 0

        if self._ctrl_j_space is True:
            jnt_names = self.obj_handle.get_joint_names()
            self._n_jnts = len(jnt_names)
            self.jnt_gui = JointGUI(obj_name, self._n_jnts, jnt_names)

    def run(self):
        while not rospy.is_shutdown():

            if self._ctrl_c_space is True:
                self.obj_gui.App.update()
                px = self.obj_gui.px * self.obj_gui.get_px_scale()
                py = self.obj_gui.py * self.obj_gui.get_py_scale()
                pz = self.obj_gui.pz * self.obj_gui.get_pz_scale()
                rx = self.obj_gui.rx * self.obj_gui.get_rx_scale()
                ry = self.obj_gui.ry * self.obj_gui.get_ry_scale()
                rz = self.obj_gui.rz * self.obj_gui.get_rz_scale()
                if self.obj_gui.cartesian_mode == 0:
                    if self._wrench_supported:
                        self.obj_handle.set_force(px, py, pz)
                        self.obj_handle.set_torque(rx, ry, rz)
                elif self.obj_gui.cartesian_mode == 1:
                    if self._pose_supported:
                        self.obj_handle.set_pos(px, py, pz)
                        self.obj_handle.set_rpy(rx, ry, rz)
                elif self.obj_gui.cartesian_mode == 2:
                    if self._twist_supported:
                        self.obj_handle.set_linear_vel(px, py, pz)
                        self.obj_handle.set_angular_vel(rx, ry, rz)
                else:
                    print('CANNOT UNDERSTAND CARTESIAN CONTROL MODE. SUPPORTED MODES ARE 0, 1, 2 FOR F, P, V')

            if self._ctrl_j_space is True:
                self.jnt_gui.App.update()
                for i in range(self._n_jnts):
                    try:
                        cmd_scale = float(self.jnt_gui.cmd_scales[i].get())
                    except ValueError:
                        cmd_scale = 1.0

                    scaled_cmd = cmd_scale * self.jnt_gui.jnt_cmds[i]
                    if self.jnt_gui.jnt_mode[i] == 0:
                        self.obj_handle.set_joint_effort(i, scaled_cmd)
                    elif self.jnt_gui.jnt_mode[i] == 1:
                        self.obj_handle.set_joint_pos(i, scaled_cmd)
                    elif self.jnt_gui.jnt_mode[i] == 2:
                        self.obj_handle.set_joint_vel(i, scaled_cmd)
                    else:
                        print('CANNOT UNDERSTAND JOINT CONTROL MODE. SUPPORTED MODES ARE 0, 1, 2 FOR F, P, V')

            time.sleep(0.001)


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-o', action='store', dest='obj_name', help='Specify AMBF Obj Name')
    parser.add_argument('-c', action='store', dest='enable_cartesian_control', help='Enable Control of Cartesian Space',
                        default='False')
    parser.add_argument('-j', action='store', dest='enable_joint_control', help='Enable Control of Joint Space',
                        default='True')
    parser.add_argument('-a', action='store', dest='client_name', help='Client Name',
                        default=None)
    parser.add_argument('--ixyz', action='store', dest='initial_xyz', help='Initial XYZ',
                        default='0, 0, 0')
    parser.add_argument('--irpy', action='store', dest='initial_rpy', help='Initial RPY',
                        default='0, 0, 0')
    parser.add_argument('--rxyz', action='store', dest='range_xyz', help='Range XYZ',
                        default='1')
    parser.add_argument('--rrpy', action='store', dest='range_rpy', help='Range RPY',
                        default='3.14')
    parser.add_argument('--res', action='store', dest='resolution', help='Resolution',
                        default='0.001')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    initial_xyz = [float(f) for f in parsed_args.initial_xyz.split(',')]
    initial_rpy = [float(f) for f in parsed_args.initial_rpy.split(',')]
    ec = parsed_args.enable_cartesian_control
    if ec.lower() in ['true', '1', 't']:
        enable_c_ctrl = True
    elif ec.lower() in ['false', '0', 'f']:
        enable_c_ctrl = False
    else:
        enable_c_ctrl = False

    ej = parsed_args.enable_joint_control
    if ej.lower() in ['true', '1', 't']:
        enable_j_ctrl = True
    elif ej.lower() in ['false', '0', 'f']:
        enable_j_ctrl = False
    else:
        enable_j_ctrl = False

    range_xyz = float(parsed_args.range_xyz)
    range_rpy = float(parsed_args.range_rpy)
    resolution = float(parsed_args.resolution)

    oc = ObjectControl(parsed_args.obj_name,
                       parsed_args.client_name,
                       enable_c_ctrl,
                       enable_j_ctrl,
                       initial_xyz,
                       initial_rpy,
                       range_xyz,
                       range_rpy,
                       resolution)
    oc.run()


if __name__ == '__main__':
    main()
