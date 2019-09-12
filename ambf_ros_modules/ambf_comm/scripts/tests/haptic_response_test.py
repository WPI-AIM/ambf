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
import numpy as np
import sys
from ambf_msgs.msg import ObjectState, WorldState

g1State = ObjectState()
g2State = ObjectState()
wState = WorldState()

global g1D, g2D


class DataCollection():
    def __init__(self, name):
        self.data_lim = 1000
        if len(sys.argv) > 1:
            self.data_lim = int(sys.argv[1])
            print 'Taking {} data points'.format(sys.argv[1])
        self.data = np.zeros(self.data_lim)
        self.ctr = 0
        self.name = name

    def add_data(self, d):
        if self.ctr < self.data_lim:
            self.data[self.ctr] = d
            self.ctr = self.ctr + 1

    def compute_data_metrics(self):
        mean = np.mean(self.data)
        std_dev = np.std(self.data)

        print '----------------------------------'
        print 'Data Metrics for {}'.format(self.name)
        print 'Mean = {}'.format(mean)
        print 'Std Deviation = {}'.format(std_dev)
        print '----------------------------------'

    def is_done(self):
        if self.ctr < self.data_lim:
            return False
        else:
            return True


g1D = DataCollection('Gripper1')

g2D = DataCollection('Gripper2')


def g1_cb(msg):
    global g1D
    g1D.add_data(msg.userdata[0])
    pass


def g2_cb(msg):
    global g2D
    g2D.add_data(msg.userdata[0])
    pass


def w_cb(msg):
    #
    pass


rospy.init_node('haptic_dynamic_response_test')
g1_sub = rospy.Subscriber('/ambf/env/Gripper1/State', ObjectState, g1_cb, queue_size=1)
g2_sub = rospy.Subscriber('/ambf/env/Gripper2/State', ObjectState, g2_cb, queue_size=1)
w_sub = rospy.Subscriber('/ambf/env/World/State', WorldState, w_cb, queue_size=1)
rate = rospy.Rate(10)

while not g1D.is_done() or not g2D.is_done():
    rate.sleep()

g1D.compute_data_metrics()
g2D.compute_data_metrics()
g1_sub.unregister()
g2_sub.unregister()



