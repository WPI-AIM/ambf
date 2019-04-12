from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
from PyKDL import Rotation, Frame, Vector
from Tkinter import *

global state_msg, cmd_msg, active
global K_ang, D_ang
global x, y, z
global roll, pitch, yaw


def x_cb(val):
    global x
    x = float(val)


def y_cb(val):
    global y
    y = float(val)


def z_cb(val):
    global z
    z = float(val)


def roll_cb(val):
    global roll
    roll = float(val)


def pitch_cb(val):
    global pitch
    pitch = float(val)


def yaw_cb(val):
    global yaw
    yaw = float(val)


def k_ang_cb(val):
    global K_ang
    K_ang = float(val)


def d_ang_cb(val):
    global D_ang
    D_ang = float(val)


def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True


app = Tk()
_width = 20
_length = 300
_resolution = 0.1

x_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=x_cb)
x_slider.pack()
x_label = Label(app, text="x")
x_label.pack()

y_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=y_cb)
y_slider.pack()
y_label = Label(app, text="y")
y_label.pack()

z_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=z_cb)
z_slider.pack()
z_label = Label(app, text="z")
z_label.pack()

roll_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=roll_cb)
roll_slider.pack()
roll_label = Label(app, text="roll")
roll_label.pack()

pitch_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=pitch_cb)
pitch_slider.pack()
pitch_label = Label(app, text="pitch")
pitch_label.pack()

yaw_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=yaw_cb)
yaw_slider.pack()
yaw_label = Label(app, text="yaw")
yaw_label.pack()

p_ang_slider = Scale(app, from_=0.1, to=20, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=k_ang_cb)
p_ang_slider.set(5)
p_ang_slider.pack()
ka_label = Label(app, text="K ang")
ka_label.pack()

d_ang_slider = Scale(app, from_=0, to=10, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=d_ang_cb)
d_ang_slider.set(0.5)
d_ang_slider.pack()
da_label = Label(app, text="D ang")
da_label.pack()

state_msg = ObjectState
active = False
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0

rospy.init_node('ambf_control_test')

sub = rospy.Subscriber('/ambf/env/CirclePuzzle/State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher('/ambf/env/CirclePuzzle/Command', ObjectCmd, queue_size=1)

rate = rospy.Rate(1000)

cmd_pos = Vector(0, 0, 0)
cmd_rot = Rotation.RPY(0,0.8,0)

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

while not rospy.is_shutdown():
    app.update()
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
        cmd_pos = Vector(x, y, z)
        cmd_rot = Rotation.RPY(roll, pitch, yaw)

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

        last_torque = torque
        torque = K_ang * rot_axis * d_rot_angle
        d_torque = D_ang * ((torque - last_torque)/K_ang) / dt

        net_torque = cur_rot * (torque + d_torque)

        cmd_msg.wrench.force.x = force[0]
        cmd_msg.wrench.force.y = force[1]
        cmd_msg.wrench.force.z = force[2]

        cmd_msg.wrench.torque.x = net_torque[0]
        cmd_msg.wrench.torque.y = net_torque[1]
        cmd_msg.wrench.torque.z = net_torque[2]

        pub.publish(cmd_msg)

        rate.sleep()





