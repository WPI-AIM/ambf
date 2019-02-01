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
