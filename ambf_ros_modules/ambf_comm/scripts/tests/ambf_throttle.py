import rospy
from ambf_msgs.msg import WorldCmd
import sys


cmd = WorldCmd()
cmd.enable_step_throttling = True
cmd.step_clock = True
cmd.n_skip_steps = 1

if len(sys.argv) > 1:
    print 'Clock Rate Specified as {}'.format(sys.argv[1])
    clock_rate = float(sys.argv[1])
else:
    clock_rate = 50

rospy.init_node('ambf_throttle_test')
pub = rospy.Publisher('/ambf/env/World/Command', WorldCmd, queue_size=1)
rate = rospy.Rate(clock_rate)
while not rospy.is_shutdown():
    cmd.step_clock = not cmd.step_clock
    pub.publish(cmd)
    rate.sleep()

pub.unregister()