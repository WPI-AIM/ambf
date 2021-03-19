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
import numpy as np
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument('-a', action='store', dest='arm_name', help='Mock Arm Name. Should be MTMR or MTML', default='MTMR')
parser.add_argument('-n', action='store', dest='node_name', help='ROS Node Name', default='_node')

parsed_args = parser.parse_args()
print('Specified Arguments')
print(parsed_args)

rospy.init_node(parsed_args.arm_name + parsed_args.node_name)
time.sleep(1.0)
if not parsed_args.arm_name in ['MTMR', 'MTML']:
    raise ValueError

mock_mtm = ProxyMTM(parsed_args.arm_name)
mock_mtm.publish_status()
mock_mtm.set_pos(0, 0, -1.3)
mock_mtm.set_pos(0, 0, 0.0)
rate = rospy.Rate(100)

PU.init()
App = PU.get_app_handle()

start_time = rospy.Time.now()
last_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
min_dt = rospy.Duration(0.001)
while not rospy.is_shutdown():
    App.update()
    mock_mtm.publish_status()
    mock_mtm.set_pos(PU.x, PU.y, PU.z)
    mock_mtm.set_orientation(PU.roll, PU.pitch, PU.yaw)
    mock_mtm.set_gripper_angle(PU.gripper)
    dt = rospy.Time.now() - start_time
    # Set dt threshold to avoid setting twist on first iteration
    if dt >= min_dt:
        twist = (np.array([PU.x, PU.y, PU.z, PU.roll, PU.pitch, PU.yaw]) - last_state) / dt.to_sec()
        mock_mtm.set_twist(twist[0], twist[1], twist[2], twist[3], twist[4], twist[5])
    last_state = np.array([PU.x, PU.y, PU.z, PU.roll, PU.pitch, PU.yaw])
    rate.sleep()
