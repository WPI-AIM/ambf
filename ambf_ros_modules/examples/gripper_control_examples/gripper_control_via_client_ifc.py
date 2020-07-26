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
import postion_control_util as PU
import rospy

c = Client()
c.connect()

gripper_base = c.get_obj_handle('r_gripper_palm_link')
sensor = c.get_obj_handle('Proximity0')
actuator = c.get_obj_handle('Constraint0')

time.sleep(0.3)

PU.init()
App = PU.get_app_handle()

# Set the joint limits. We shall use them to normalize the gripper open and close angle.
# I copied these values from pr2 gripper.yaml file, for a different file, just use the appropriate limits
joint_limits = [[-0.3, 0.548], [-0.3, 0.548], [-0.3, 0.548], [-0.3, 0.548]]

grasped = False

while not rospy.is_shutdown():
    App.update()
    gripper_base.set_pos(PU.x, PU.y, PU.z)
    gripper_base.set_rpy(PU.roll, PU.pitch, PU.yaw)

    # Since we are handling the body using the Python cleint. We will have to set all the joints manually
    for i in range(gripper_base.get_num_joints()):
        jnt_range = joint_limits[i][1] - joint_limits[i][0]
        adjusted_val = joint_limits[i][0] + PU.gripper * jnt_range
        gripper_base.set_joint_pos(i, adjusted_val)

        # We can also keep checking the sensor if it triggers
        if sensor.is_triggered(0) and PU.gripper < 0.5:
            sensed_obj = sensor.get_sensed_object(0)
            if not grasped:
                actuator.actuate(sensed_obj)
                grasped = True
                print('Grasping Sensed Object Names', sensed_obj)
        else:
            actuator.deactuate()
            grasped = False
