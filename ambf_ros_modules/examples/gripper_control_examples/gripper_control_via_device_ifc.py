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

from proxy_device import ProxyMTM
import rospy
import time
import postion_control_util as PU

rospy.init_node('test')
time.sleep(1.0)
mtmr = ProxyMTM('MTMR')
mtmr.publish_status()
mtmr.set_pos(0, 0, -1.3)
mtmr.set_pos(0, 0, 0.0)
rate = rospy.Rate(100)

PU.init()
App = PU.get_app_handle()

while not rospy.is_shutdown():
    App.update()
    mtmr.set_pos(PU.x, PU.y, PU.z)
    mtmr.set_angle(PU.roll, PU.pitch, PU.yaw)
    mtmr.set_gripper_angle(PU.gripper)
    rate.sleep()
