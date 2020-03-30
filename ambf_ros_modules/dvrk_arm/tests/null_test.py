import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import sys
from argparse import ArgumentParser


js_msg = JointState()
js_cmd = JointState()
qs = np.zeros(7)
vs = np.zeros(7)
Kp = 0.5
Kd = 0.1
arm_name = 'MTMR'
robot_ready = False


def js_cb(msg):
    global qs, js_msg, vs
    js_msg = msg
    qs = js_msg.position
    vs = js_msg.velocity

def state_cb(msg):
    global robot_ready
    if msg.data == 'READY':
        robot_ready = True
    else:
        robot_ready = False

    print(msg.data)


def rot_z(q):
    r = np.mat([[np.cos(q),-np.sin(q), 0],
                  [np.sin(q), np.cos(q), 0],
                  [0, 0, 1]])
    return r


def main():
    global arm_name, Kp, Kd, robot_ready
    # Begin Argument Parser Code
    parser = ArgumentParser()

    parser.add_argument('-a', action='store', dest='arm_name', help='Specify Arm Name',
                        default='MTMR')
    parser.add_argument('-p', action='store', dest='Kp', help='Specify Linear Gain',
                        default=0.5)
    parser.add_argument('-d', action='store', dest='Kd', help='Specify Damping Gain',
                        default=0.1)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    arm_name = parsed_args.arm_name

    if arm_name != 'MTMR' and arm_name != 'MTML':
        print('Specified arm is: ', arm_name)
        raise Exception('Error, specify MTML or MTMR as input arg')

    Kp = float(parsed_args.Kp)
    Kd = float(parsed_args.Kd)

    node_name = arm_name + '_null_space_test'
    rospy.init_node(node_name)
    joint_state_sub = rospy.Subscriber('/dvrk/' + arm_name + '/state_joint_current', JointState, js_cb)
    robot_state_sub = rospy.Subscriber('/dvrk/' + arm_name + '/current_state', String, state_cb, queue_size=1)

    joint_state_pub = rospy.Publisher('/dvrk/' + arm_name + '/set_effort_joint', JointState, queue_size=10)
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

        if qs[4] > np.pi/2:
            direction = -1
        else:
            direction = 1

        re = l4_o * r4 * l5_o * r5 * l6_o * r6 * l7_o * r7
        # print 'EE Matrix'
        # print re
        ve = re[:, 2]
        v4 = r4[:, 0]

        e_pre = e
        e = (np.pi/2) - np.arccos(np.dot(v4.transpose(), ve))
        de = e - e_pre
        tau4 = Kp * direction * e - Kd * vs[3]
        np.clip(tau4, -0.3, 0.3)
        js_cmd.effort[3] = tau4[0, 0]

        if robot_ready:
            joint_state_pub.publish(js_cmd)
        r.sleep()


if __name__ == "__main__":
    main()




