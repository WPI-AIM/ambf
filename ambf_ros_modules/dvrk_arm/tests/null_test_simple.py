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
arm_name = 'MTMR'
robot_ready = False
namespace = '/dvrk/'


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
    parser.add_argument('-p4', action='store', dest='Kp_4', help='Specify Wrist Platform (Joint 4) Linear Gain',
                        default=0.5)
    parser.add_argument('-d4', action='store', dest='Kd_4', help='Specify Wirst Platform (Joint 4) Damping Gain',
                        default=0.05)
    parser.add_argument('-l4', action='store', dest='lim_4', help='Specify Wrist Platform (Joint 4) Torque Limit',
                        default=0.2)
    parser.add_argument('-p6', action='store', dest='Kp_6', help='Specify Wrist Yaw (Joint 6) Linear Gain',
                        default=0.0)
    parser.add_argument('-d6', action='store', dest='Kd_6', help='Specify Wrist Yaw (Joint 6) Damping Gain',
                        default=0.0)
    parser.add_argument('-l6', action='store', dest='lim_6', help='Specify Wrist Yaw (Joint 6) Torque Limit',
                        default=0.01)
    parser.add_argument('-n', action='store', dest='namespace', help='ROS Namespace',
                        default='dvrk')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    arm_name = parsed_args.arm_name

    if arm_name != 'MTMR' and arm_name != 'MTML':
        print('Specified arm is: ', arm_name)
        raise Exception('Error, specify MTML or MTMR as input arg')

    # Wrist Platform (Joint 4)
    Kp_4 = float(parsed_args.Kp_4)
    Kd_4 = float(parsed_args.Kd_4)
    lim_4 = float(parsed_args.lim_4)

    # Wrist Yaw (Joint 6)
    Kp_6 = float(parsed_args.Kp_6)
    Kd_6 = float(parsed_args.Kd_6)
    lim_6 = float(parsed_args.lim_6)

    node_name = arm_name + '_null_space_test'
    rospy.init_node(node_name)
    joint_state_sub = rospy.Subscriber(namespace + arm_name + '/state_joint_current', JointState, js_cb)
    robot_state_sub = rospy.Subscriber(namespace + arm_name + '/current_state', String, state_cb, queue_size=1)

    joint_state_pub = rospy.Publisher(namespace + arm_name + '/set_effort_joint', JointState, queue_size=10)
    js_cmd.effort = [0, 0, 0, 0, 0, 0, 0]

    e = 0.0
    e_pre = e
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():

        if robot_ready:

            # Limits for Wrist Pitch (Joint 5)
            a_lim_5 = -1.5
            b_lim_5 = 1.2
            c_lim_5 = 1.8

            sign = 1
            if a_lim_5 < qs[4] <= b_lim_5 :
                sign = 1
            elif b_lim_5 < qs[4] < c_lim_5:
                range = c_lim_5 - b_lim_5
                normalized_val = (qs[4] - b_lim_5) / range
                centerd_val = normalized_val - 0.5
                sign = -centerd_val * 2
                print('MID VAL:', sign)
                #sign = 0
            else:
                sign = -1

            e = qs[5]
            tau_4 = Kp_4 * e * sign - Kd_4 * vs[3]
            tau_4 = np.clip(tau_4, -lim_4, lim_4)

            tau_6 = -Kp_6 * qs[5] - Kd_6 * vs[5]
            tau_6 = np.clip(tau_6, -lim_6, lim_6)
            js_cmd.effort[3] = tau_4
            js_cmd.effort[5] = tau_6
            joint_state_pub.publish(js_cmd)
            #print 'PITCH JOINT: ', qs[4]
            # print 'YAW JOINT: ', tau6
            # print(round(e, 2))
            # print('YAW :', qs[3], 'PITCH: ', qs[4], ' | YAW: ', qs[5])
        r.sleep()

if __name__ == "__main__":
    main()
