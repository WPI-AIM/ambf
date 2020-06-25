from mimic import ProxyMTM
import rospy
import time
import postion_control_util as PU

rospy.init_node('test')
time.sleep(1.0)
mtmr = ProxyMTM('MTMR')
mtmr.publish_status()
mtmr.set_pos(0, 0, -1.3)
mtmr.set_pos(0, 0, 0.0)
rate = rospy.Rate(100)

PU.init()
App = PU.get_app_handle()

while not rospy.is_shutdown():
    App.update()
    mtmr.set_pos(PU.x, PU.y, PU.z)
    mtmr.set_angle(PU.roll, PU.pitch, PU.yaw)
    mtmr.set_gripper_angle(PU.gripper)
    rate.sleep()