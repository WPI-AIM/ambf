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

from ambf_msgs.msg import WorldState, WorldCmd
from watch_dog import WatchDog


class World(WatchDog):
    def __init__(self, a_name):
        super(World, self).__init__(2.0)
        self._state = WorldState()
        self._name = a_name
        self._cmd = WorldCmd()
        self._cmd.enable_step_throttling = False
        self._pub = None
        self._sub = None
        self._pub_flag = True
        self._active = False

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def enable_throttling(self, data):
        self._cmd.enable_step_throttling = data

    def set_num_step_skips(self, n):
        if n <= 0 or n > 100:
            raise ValueError
        self._cmd.n_skip_steps = n

    def ros_cb(self, data):
        self._state = data

    def update(self):
        self._cmd.step_clock = not self._cmd.step_clock
        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def clear_cmd(self):
        self._cmd.enable_step_throttling = False
        pass

    def set_name(self, name):
        self._name = name

    def get_name(self):
        return self._name

    def run_publisher(self):
        if self._pub_flag:
            if self.is_wd_expired():
                self.console_print('World')
                self.clear_cmd()
            self._pub.publish(self._cmd)
