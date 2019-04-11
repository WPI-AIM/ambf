from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
from PyKDL import Rotation, Frame, Vector


global state_msg, cmd_msg, active

state_msg = ObjectState
active = False

def ambf_cb(msg):
    global  state_msg, cmd_msg, active
    state_msg = msg
    active = True

rospy.init_node('ambf_control_test')

sub = rospy.Subscriber('/ambf/env/CirclePuzzle/State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher('/ambf/env/CirclePuzzle/Command', ObjectCmd, queue_size=1)

rate = rospy.Rate(1000)

cmd_pos = Vector(0, 0, 0)
cmd_rot = Rotation.RPY(0,0.8,0)
cmd_pose = Frame(cmd_rot, cmd_pos)

K_lin = 100.0
D_lin = 20.0
K_ang = 5
D_ang = 1

last_delta_pos = Vector(0, 0, 0)
delta_pos = Vector(0, 0, 0)
last_pos = Vector(0, 0, 0)

m_drot_prev = Rotation.RPY(0, 0, 0)
m_drot = Rotation.RPY(0, 0, 0)
last_rot = Rotation.RPY(0, 0, 0)

cmd_msg = ObjectCmd()
cmd_msg.enable_position_controller = False

dt = 0.001

cur_time = rospy.Time.now().to_sec()

while not rospy.is_shutdown():
    last_time = cur_time
    cur_time = rospy.Time.now().to_sec()
    dt = cur_time - last_time
    if dt < 0.001:
        dt = 0.001
    if active:
        cur_rot = Rotation.Quaternion(state_msg.pose.orientation.x,
                                        state_msg.pose.orientation.y,
                                        state_msg.pose.orientation.z,
                                        state_msg.pose.orientation.w)
        cur_pos = Vector(state_msg.pose.position.x,
                         state_msg.pose.position.y,
                         state_msg.pose.position.z)

        last_pos = cur_pos
        last_delta_pos = delta_pos
        delta_pos = cmd_pos - cur_pos
        delta_delta_pos = (delta_pos - last_delta_pos) / dt
        # print  (D_lin * delta_delta_pos) / dtp
        force = K_lin * delta_pos + D_lin * delta_delta_pos

        m_drot_prev = m_drot
        m_drot = cur_rot.Inverse() * cmd_rot
        m_ddrot = m_drot_prev.Inverse() * m_drot

        [d_rot_angle, rot_axis] = m_drot.GetRotAngle()
        [dd_rot_angle, d_rot_axis] = m_ddrot.GetRotAngle()

        torque = K_ang * rot_axis * d_rot_angle + (D_ang * d_rot_axis * dd_rot_angle / dt)
        # print  (D_lin * delta_delta_pos) / dtp
        # print d_rot_angle

        torque = cur_rot.Inverse() * torque

        cmd_msg.wrench.force.x = force[0]
        cmd_msg.wrench.force.y = force[1]
        cmd_msg.wrench.force.z = force[2]

        cmd_msg.wrench.torque.x = torque[0]
        cmd_msg.wrench.torque.y = torque[1]
        cmd_msg.wrench.torque.z = torque[2]

        pub.publish(cmd_msg)

        rate.sleep()





