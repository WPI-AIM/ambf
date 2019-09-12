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
from geometry_msgs.msg import Pose
from PyKDL import Rotation, Frame, Vector

global openhmd_state, occulus_state, occulus_state_valid, openhmd_state_valid

openhmd_state = Pose()
occulus_state = ObjectState()
occulus_cmd = ObjectCmd()
occulus_state_valid = False
openhmd_state_valid = False


def openhmd_cb(msg):
    global openhmd_state, openhmd_state_valid
    openhmd_state = msg
    openhmd_state_valid = True


def occulus_cb(msg):
    global occulus_state, occulus_state_valid
    occulus_state = msg
    occulus_state_valid = True


def main():
    global openhmd_state, occulus_state, openhmd_state_valid, occulus_state_valid
    rospy.init_node('occulus_view')
    openhmd_sub = rospy.Subscriber("/openhmd/pose", Pose, openhmd_cb)
    occulus_sub = rospy.Subscriber("/ambf/env/default_camera/State", ObjectState, occulus_cb, queue_size=1)
    occulus_pub = rospy.Publisher("/ambf/env/default_camera/Command", ObjectCmd, queue_size=1)

    rate = rospy.Rate(60)
    counter = 0
    start = rospy.get_time()
    _first = True
    openhmd_initial_rot = Rotation()
    occulus_initial_rot = Rotation()
    R_pre = Rotation()
    R_aInr_offset = Rotation().RPY(0, -1.57079, -1.57079)
    # open
    while not rospy.is_shutdown():
        if openhmd_state_valid and occulus_state_valid:
            if _first:
                _first = False
                openhmd_initial_rot = Rotation.Quaternion(openhmd_state.orientation.x,
                                                          openhmd_state.orientation.y,
                                                          openhmd_state.orientation.z,
                                                          openhmd_state.orientation.w)

                occulus_initial_rot = Rotation.Quaternion(occulus_state.pose.orientation.x,
                                                          occulus_state.pose.orientation.y,
                                                          occulus_state.pose.orientation.z,
                                                          occulus_state.pose.orientation.w)

                R_pre = openhmd_initial_rot * R_aInr_offset * occulus_initial_rot.Inverse()

            else:
                occulus_cmd.pose.position = occulus_state.pose.position
                openhmd_rot = Rotation.Quaternion(openhmd_state.orientation.x,
                                                  openhmd_state.orientation.y,
                                                  openhmd_state.orientation.z,
                                                  openhmd_state.orientation.w)
                delta_rot = R_pre.Inverse() * openhmd_rot * R_aInr_offset
                # delta_rot = openhmd_rot
                occulus_cmd.pose.orientation.x = delta_rot.GetQuaternion()[0]
                occulus_cmd.pose.orientation.y = delta_rot.GetQuaternion()[1]
                occulus_cmd.pose.orientation.z = delta_rot.GetQuaternion()[2]
                occulus_cmd.pose.orientation.w = delta_rot.GetQuaternion()[3]
                occulus_cmd.enable_position_controller = True

        occulus_pub.publish(occulus_cmd)
        counter = counter + 1
        if counter % 60 == 0:
            print "- Publishing Occulus Pose ", format( round(rospy.get_time() - start, 3)), 's'
        rate.sleep()


if __name__ == "__main__":
    main()
