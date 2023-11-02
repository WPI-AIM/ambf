#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2023 Akhil Deo <adeo1@jhu.edu>


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


#     \author    <adeo1@jhu.edu>
#     \author    Akhil Deo
#     \version   1.0
# */
# //==============================================================================

from ambf_client import Client
import rospy
from PyKDL import Rotation, Frame, Vector
import socket
import json
import time
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
import numpy as np
import sys
import signal


UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))


_client = Client()
_client.connect()
print(_client.get_obj_names())
w = _client.get_world_handle()
w.reset_bodies()
psm1 = PSM(_client, 'psm1')
psm2 = PSM(_client, 'psm2')
ecm = ECM(_client, 'CameraFrame')
psm_arms = {"left": psm1,
            "right": psm2}

# The PSMs can be controlled either in joint space or cartesian space. For the
# latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.


def signal_handler(signum, frame):
    print("\nCtrl+C clicked!")
    exit(1)


signal.signal(signal.SIGINT, signal_handler)


def set_arm_xyz(data_dict, arm):
    if arm == 'right':
        return Vector(data_dict['x'] + 0.1, data_dict['y'] + 0.5, data_dict['z'] - 1.3)
    else:
        return Vector(data_dict['x'], data_dict['y'], data_dict['z'] - 1.3)


def set_slider(data_dict_slider, robot_arm_slider, arm, cur_slider):
    if arm == 'right' and data_dict_slider['slider'] != cur_slider:
        robot_arm_slider.set_jaw_angle(data_dict['slider'])
        return data_dict['slider']
    elif arm == 'left' and data_dict_slider['slider'] != cur_slider_left:
        robot_arm_slider.set_jaw_angle(data_dict['slider'])
        return data_dict['slider']


print("Starting TeleOp")
print("Comment out ensuing print statements for better performance")

rate = rospy.Rate(60)
cur_slider_left = 0.5
cur_slider_right = 0.5

cmd_xyz_right = Vector(0.1, 0.5, -1.3)
cmd_xyz_left = Vector(0, 0, -1.3)

while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    if data is not None:
        data_dict = json.loads(data)
        # print(data_dict)
        if data_dict['test'] == 'true':
            print(data)
            ip_addr = addr[0]
            print(ip_addr)
            sock.sendto(data, (ip_addr, 8080))
        elif data_dict['camera'] == 'true':
            ecm.servo_jp([data_dict['yaw'], data_dict['pitch'],
                         data_dict['insert'], data_dict['roll']])
        else:
            robot_arm = psm_arms[data_dict['arm']]
            if data_dict['arm'] == 'right':
                cmd_rpy = Rotation.RPY(-1 * data_dict['yaw'] + np.pi,
                                       data_dict['pitch'], data_dict['roll'] - (np.pi / 4))
                if data_dict['transformation'] == 'true':
                    cmd_xyz_right = set_arm_xyz(data_dict, 'right')
                robot_arm.servo_cp(Frame(cmd_rpy, cmd_xyz_right))
                cur_slider_right = set_slider(
                    data_dict, robot_arm, 'right', cur_slider_right)
            else:
                cmd_rpy = Rotation.RPY(data_dict['pitch'],
                                       data_dict['yaw'], data_dict['roll'])
                if data_dict['transformation'] == 'true':
                    cmd_xyz_left = set_arm_xyz(data_dict, 'left')
                robot_arm.servo_cp(Frame(cmd_rpy, cmd_xyz_left))
                cur_slider_left = set_slider(
                    data_dict, robot_arm, 'left', cur_slider_left)
    rate.sleep()
