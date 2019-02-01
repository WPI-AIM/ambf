#!/usr/bin/env python
from ambf_msgs.msg import WorldCmd, WorldState
import rospy
import time

cmd = WorldCmd()
global cb_ctr, step, pre_step
cb_ctr = 0
step = 0
pre_step = 0


def cb(data):
    global cb_ctr, step, pre_step
    step = data.sim_step
    cb_ctr +=1
    if cb_ctr % 1000 == 0:
        print step


def main():
    global pre_step, step
    rospy.init_node('duplex_comm_text')
    sub = rospy.Subscriber('/ambf/env/World/State', WorldState, callback=cb, queue_size=10)
    pub = rospy.Publisher('/ambf/env/World/Command', WorldCmd, queue_size=10)
    rate = rospy.Rate(5000)
    cmd.enable_step_throttling = True
    cmd.n_skip_steps = 1
    pre_step = step
    while not rospy.is_shutdown():
        dstep = 0
        while dstep < cmd.n_skip_steps:
            dstep = step - pre_step
            time.sleep(0.0001)
        cmd.step_clock = not cmd.step_clock
        pre_step = step
        if dstep > cmd.n_skip_steps:
            print 'Jumped {} steps, default is {}'.format(dstep, cmd.n_skip_steps)
        pub.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    main()