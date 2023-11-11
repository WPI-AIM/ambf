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
psms = {"left": psm1,
            "right": psm2}

# The PSMs can be controlled either in joint space or cartesian space. For the
# latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.

def signal_handler(signum, frame):
    print("\nCtrl+C clicked!")
    exit(1)


signal.signal(signal.SIGINT, signal_handler)

def set_psm_translation(psm_info, psm):
    if psm == 'right':
        return Vector(psm_info['x'] + 0.1, psm_info['y'] + 0.5, psm_info['z'] - 1.3)
    return Vector(psm_info['x'], psm_info['y'], psm_info['z'] - 1.3)

def set_end_effector(psm_info, psm_end_effector, psm, end_effector):
    if psm == 'right' and psm_info['end_effector'] != end_effector:
        psm_end_effector.set_jaw_angle(psm_info['end_effector'])
        return psm_info['end_effector']
    elif psm == 'left' and psm_info['end_effector'] != end_effector_left:
        psm_end_effector.set_jaw_angle(psm_info['end_effector'])
        return psm_info['end_effector']

print("Starting TeleOp")
print("Comment out pose print statements for better performance")


rate = rospy.Rate(60)

end_effector_left = 0.5
end_effector_right = 0.5
translation_right = Vector(0.1, 0.5, -1.3)
translation_left = Vector(0, 0, -1.3)

while not rospy.is_shutdown():
    data,_ = sock.recvfrom(1024)
    if data is not None:
        print(data)
        psm_info = json.loads(data)
        if psm_info['camera'] == 'true':
            ecm.servo_jp([psm_info['yaw'], psm_info['pitch'], psm_info['insert'], psm_info['roll']])
        else:
            psm = psms[psm_info['psm']]
            if psm_info['psm'] == 'right':
                cmd_rpy = Rotation.RPY(-1 * psm_info['yaw'] + np.pi, psm_info['pitch'], psm_info['roll'] - (np.pi / 4))
                if psm_info['transformation'] == 'true':
                    translation_right = set_psm_translation(psm_info, 'right')
                psm.servo_cp(Frame(cmd_rpy, translation_right))
                end_effector_right = set_end_effector(psm_info, psm, 'right', end_effector_right)
            else:
                cmd_rpy = Rotation.RPY(psm_info['pitch'], psm_info['yaw'], psm_info['roll'])
                if psm_info['transformation'] == 'true':
                    translation_left = set_psm_translation(psm_info, 'left')
                psm.servo_cp(Frame(cmd_rpy, translation_left))
                end_effector_left = set_end_effector(psm_info, psm, 'left', end_effector_left)
    rate.sleep()
