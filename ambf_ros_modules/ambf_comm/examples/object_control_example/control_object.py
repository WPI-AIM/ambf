#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019
#     (aimlab.wpi.edu)

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

#     \author    <aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

from ambf_client import Client
import time
import obj_control_gui as ObjCtrlGUI
import jnt_control_gui as JntCtrlGUI
import rospy
from argparse import ArgumentParser


class ObjectControl:
    def __init__(self, obj_name, c_space_ctrl, j_space_ctrl):
        self.client = Client()
        self.client.connect()
        time.sleep(0.3)
        self.obj_handle = self.client.get_obj_handle(obj_name)
        time.sleep(0.3)

        self._ctrl_c_space = c_space_ctrl
        self._ctrl_j_space = j_space_ctrl

        if self._ctrl_c_space:
            self.obj_gui = ObjCtrlGUI
            self.obj_gui.init(obj_name)

        self._n_jnts = 0

        if self._ctrl_j_space:
            jnt_names = self.obj_handle.get_joint_names()
            self._n_jnts = len(jnt_names)
            self.jnt_gui = JntCtrlGUI
            self.jnt_gui.init(obj_name, self._n_jnts, jnt_names)

    def run(self):
        while not rospy.is_shutdown():

            if self._ctrl_c_space:
                self.obj_gui.App.update()
                self.obj_handle.set_pos(self.obj_gui.x, self.obj_gui.y, self.obj_gui.z)
                self.obj_handle.set_rpy(self.obj_gui.roll, self.obj_gui.pitch, self.obj_gui.yaw)

            if self._ctrl_j_space:
                self.jnt_gui.App.update()
                for i in range(self._n_jnts):
                    if not self.jnt_gui.jnt_mode[i]:
                        self.obj_handle.set_joint_effort(i, self.jnt_gui.jnt_cmd[i])
                    else:
                        self.obj_handle.set_joint_pos(i, self.jnt_gui.jnt_cmd[i])

            time.sleep(0.001)


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-o', action='store', dest='obj_name', help='Specify AMBF Obj Name')
    parser.add_argument('-c', action='store', dest='enable_cartesian_control', help='Enable Control of Cartesian Space',
                        default=False)
    parser.add_argument('-j', action='store', dest='enable_joint_control', help='Enable Control of Joint Space',
                        default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args
    oc = ObjectControl(parsed_args.obj_name, parsed_args.enable_cartesian_control, parsed_args.enable_joint_control)
    oc.run()


if __name__ == '__main__':
    main()







