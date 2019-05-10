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
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Empty
import numpy as np
from PyKDL import Rotation
import tf_conversions
import sys

r = Rotation()
r.RPY(0,0,0)
rPose = PoseStamped()
lPose = PoseStamped()
rPose.pose.position.z = -0.30
lPose.pose.position.z = -0.20
rPose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(3.058663, -1.055021, -1.500306))
lPose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(3.058663, -1.055021, -1.500306))

tc = 4.0
scale = 1/15.0

if len(sys.argv) > 1:
    tc = float(sys.argv[1])
    print 'Setting Time Constant to {}'.format(sys.argv[1])
if len(sys.argv) > 2:
    scale = float(sys.argv[2])
    print 'Setting Scale to {}'.format(sys.argv[2])


rospy.init_node('mtm_ambf_test')
repub = rospy.Publisher('/dvrk/MTMR/status', Empty, queue_size=1)
rpub = rospy.Publisher('/dvrk/MTMR/position_cartesian_current', PoseStamped, queue_size=10)
lepub = rospy.Publisher('/dvrk/MTML/status', Empty, queue_size=1)
lpub = rospy.Publisher('/dvrk/MTML/position_cartesian_current', PoseStamped, queue_size=10)
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    rPose.pose.position.x = -0.180 + (np.sin(tc * t) * scale)
    rPose.pose.position.y = -0.016 + (np.cos(tc * t) * scale)
    rPose.pose.position.z = -0.300 + (np.sin(tc * t) * scale * 0.5)
    repub.publish(Empty())

    lPose.pose.position.x = +0.180 + (np.sin(tc * t) * scale)
    lPose.pose.position.y = -0.016 + (np.cos(tc * t) * scale)
    lPose.pose.position.z = -0.200 + (np.sin(tc * t) * scale * 0.5)
    repub.publish(Empty())

    rpub.publish(rPose)
    lpub.publish(lPose)
    rate.sleep()

rpub.unregister()
lpub.unregister()
