import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time
import sys


js_msg = JointState()
js_cmd = JointState()
qs = np.zeros(7)
vs = np.zeros(7)
Kp = 0.3
Kd = 0.1


def js_cb(msg):
    global qs, js_msg, vs
    js_msg = msg
    qs = js_msg.position
    vs = js_msg.velocity


def rot_z(q):
    r = np.mat([[np.cos(q),-np.sin(q), 0],
                  [np.sin(q), np.cos(q), 0],
                  [0, 0, 1]])
    return r


def main(argv):
    arm_name = argv[0]
    if arm_name != 'MTMR' and arm_name != 'MTML':
        print('Specified arm is: ', arm_name)
        raise Exception('Error, specify MTML or MTMR as input arg')
    node_name = arm_name + '_null_space_test'
    rospy.init_node(node_name)
    sub = rospy.Subscriber('/dvrk/' + arm_name + '/state_joint_current', JointState, js_cb)
    pub = rospy.Publisher('/dvrk/' + arm_name + '/set_effort_joint', JointState, queue_size=10)
    js_cmd.effort = [0, 0, 0, 0, 0, 0, 0]
    l4_o = np.identity(3)
    l5_o = np.mat([[ 0, 0,-1],
                    [ 0, 1, 0],
                    [ 1, 0, 0]])

    l6_o = np.mat([[ 0, 0, 1],
                    [ 0, 1, 0],
                    [-1, 0, 0]])

    l7_o = np.mat([[ 1, 0, 0],
                    [ 0, 0,-1],
                    [ 0, 1, 0]])

    e = 0.0
    e_pre = e
    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        r4 = rot_z(qs[3])
        r5 = rot_z(qs[4])
        r6 = rot_z(qs[5])
        r7 = rot_z(qs[6])

        re = l4_o * r4 * l5_o * r5 * l6_o * r6 * l7_o * r7
        # print 'EE Matrix'
        # print re
        ve = re[:, 2]
        v4 = r4[:, 0]

        e_pre = e
        e = (np.pi/2) - np.arccos(np.dot(v4.transpose(), ve))
        de = e - e_pre
        tau4 = Kp * e - Kd * vs[3]
        np.clip(tau4, -0.3, 0.3)
        js_cmd.effort[3] = tau4[0, 0]
        pub.publish(js_cmd)
        r.sleep()


if __name__ == "__main__":
    main(sys.argv[1:])
