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
from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
from PyKDL import Rotation, Vector
import position_control_utility as PU

global state_msg, cmd_msg, active

object_name = '/ambf/env/gripper3/Cylinder/'


# State CB
def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True


# Define TK App
PU.init()
App = PU.get_app_handle()

# Defaults
state_msg = ObjectState
active = False
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0

# Initialize ROS
rospy.init_node('ambf_control_test')

sub = rospy.Subscriber(object_name + 'State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher(object_name + 'Command', ObjectCmd, queue_size=1)
rate = rospy.Rate(1000)

K_lin = 100.0
D_lin = 20.0
K_ang = 5
D_ang = 1

last_delta_pos = Vector(0, 0, 0)
delta_pos = Vector(0, 0, 0)
last_pos = Vector(0, 0, 0)

m_drot_prev = Rotation.RPY(0, 0, 0)
m_drot = Rotation.RPY(0, 0, 0)
last_rot = Rotation.RPY(0, 0, 0)

cmd_msg = ObjectCmd()
cmd_msg.enable_position_controller = False

dt = 0.001

cur_time = rospy.Time.now().to_sec()
torque = Vector(0, 0, 0)
last_torque = Vector(0, 0, 0)
d_torque = Vector(0, 0, 0)

# Main Loop
while not rospy.is_shutdown():
    App.update()
    last_time = cur_time
    cur_time = rospy.Time.now().to_sec()
    dt = cur_time - last_time
    if dt < 0.001:
        dt = 0.001
    if active:
        cur_rot = Rotation.Quaternion(state_msg.pose.orientation.x,
                                        state_msg.pose.orientation.y,
                                        state_msg.pose.orientation.z,
                                        state_msg.pose.orientation.w)
        cur_pos = Vector(state_msg.pose.position.x,
                         state_msg.pose.position.y,
                         state_msg.pose.position.z)
        cmd_pos = Vector(PU.x, PU.y, PU.z)
        cmd_rot = Rotation.RPY(PU.roll, PU.pitch, PU.yaw)

        last_pos = cur_pos
        last_delta_pos = delta_pos
        delta_pos = cmd_pos - cur_pos
        delta_delta_pos = (delta_pos - last_delta_pos) / dt
        # print delta_pos, last_delta_pos
        # print  (D_lin * delta_delta_pos) / dtp
        force = PU.K_lin * delta_pos + PU.D_lin * delta_delta_pos

        m_drot_prev = m_drot
        m_drot = cur_rot.Inverse() * cmd_rot
        m_ddrot = m_drot_prev.Inverse() * m_drot

        [d_rot_angle, rot_axis] = m_drot.GetRotAngle()
        [dd_rot_angle, d_rot_axis] = m_ddrot.GetRotAngle()

        last_torque = torque
        torque = PU.K_ang * rot_axis * d_rot_angle
        d_torque = PU.D_ang * ((torque - last_torque)/PU.K_ang) / dt

        net_torque = cur_rot * (torque + d_torque)

        cmd_msg.wrench.force.x = force[0]
        cmd_msg.wrench.force.y = force[1]
        cmd_msg.wrench.force.z = force[2]

        cmd_msg.wrench.torque.x = net_torque[0]
        cmd_msg.wrench.torque.y = net_torque[1]
        cmd_msg.wrench.torque.z = net_torque[2]

        pub.publish(cmd_msg)

        rate.sleep()





