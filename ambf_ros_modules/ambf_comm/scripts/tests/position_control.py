from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
from PyKDL import Rotation, Vector
import position_control_utility as PU

global state_msg, cmd_msg, active

object_name = '/ambf/env/CirclePuzzle/'


# State CB
def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True


# Define TK App
PU.init()
App = PU.get_app_handle()

# Defaults
state_msg = ObjectState
active = False
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0

# Initialize ROS
rospy.init_node('ambf_control_test')

sub = rospy.Subscriber(object_name + 'State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher(object_name + 'Command', ObjectCmd, queue_size=1)
rate = rospy.Rate(1000)

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
torque = Vector(0, 0, 0)
last_torque = Vector(0, 0, 0)
d_torque = Vector(0, 0, 0)

# Main Loop
while not rospy.is_shutdown():
    App.update()
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
        cmd_pos = Vector(PU.x, PU.y, PU.z)
        cmd_rot = Rotation.RPY(PU.roll, PU.pitch, PU.yaw)

        last_pos = cur_pos
        last_delta_pos = delta_pos
        delta_pos = cmd_pos - cur_pos
        delta_delta_pos = (delta_pos - last_delta_pos) / dt
        # print  (D_lin * delta_delta_pos) / dtp
        force = PU.K_lin * delta_pos + PU.D_lin * delta_delta_pos

        m_drot_prev = m_drot
        m_drot = cur_rot.Inverse() * cmd_rot
        m_ddrot = m_drot_prev.Inverse() * m_drot

        [d_rot_angle, rot_axis] = m_drot.GetRotAngle()
        [dd_rot_angle, d_rot_axis] = m_ddrot.GetRotAngle()

        last_torque = torque
        torque = PU.K_ang * rot_axis * d_rot_angle
        d_torque = PU.D_ang * ((torque - last_torque)/PU.K_ang) / dt

        net_torque = cur_rot * (torque + d_torque)

        cmd_msg.wrench.force.x = force[0]
        cmd_msg.wrench.force.y = force[1]
        cmd_msg.wrench.force.z = force[2]

        cmd_msg.wrench.torque.x = net_torque[0]
        cmd_msg.wrench.torque.y = net_torque[1]
        cmd_msg.wrench.torque.z = net_torque[2]

        pub.publish(cmd_msg)

        rate.sleep()





