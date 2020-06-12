import functools
from Tkinter import *

App = Tk()
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
x_slider = None
y_slider = None
z_slider = None
roll_slider = None
pitch_slider = None
yaw_slider = None
cartesian_mode = 0


def init(obj_name):
    global x, y, z
    global roll, pitch, yaw
    global App
    create_gui(App, obj_name)


# Def Init Function
def get_app_handle():
    global App
    return App


# Define Callbacks for Tkinter GUI Sliders
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


def zero_all_cb():
    zero_xyz_cb()
    zero_rpy_cb()


def zero_xyz_cb():
    global x, y, z, roll, pitch, yaw
    global x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider
    x = 0
    y = 0
    z = 0
    x_slider.set(0)
    y_slider.set(0)
    z_slider.set(0)


def zero_rpy_cb():
    global x, y, z, roll, pitch, yaw
    global x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider
    roll = 0
    pitch = 0
    yaw = 0
    roll_slider.set(0)
    pitch_slider.set(0)
    yaw_slider.set(0)


# Define Callbacks for Tkinter GUI Slider
def effort_button_cb():
    global cartesian_mode
    cartesian_mode = 0


# Define Callbacks for Tkinter GUI Slider
def position_button_cb():
    global cartesian_mode
    cartesian_mode = 1


# Define Callbacks for Tkinter GUI Slider
def velocity_button_cb():
    global cartesian_mode
    cartesian_mode = 2


def create_gui(app, obj_name):
    global x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider, cartesian_mode
    _width = 20
    _length = 300
    _resolution = 0.001
    # Define Sliders and Labels

    v = IntVar(value=0)
    eff_cb = Radiobutton(app, text="Effort", variable=v, indicatoron=False, value=0,
                         command=effort_button_cb)
    eff_cb.grid(row=0, column=0)

    pos_cb = Radiobutton(app, text="Position", variable=v, indicatoron=False, value=1,
                         command=position_button_cb)
    pos_cb.grid(row=0, column=1)

    vel_cb = Radiobutton(app, text="Velocity", variable=v, indicatoron=False, value=2,
                         command=velocity_button_cb)
    vel_cb.grid(row=0, column=2)

    min = -100
    max = 100
    x_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=x_cb)
    x_slider.grid(row=1, column=1)
    x_label = Label(app, text="x")
    x_label.grid(row=2, column=1)

    y_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=y_cb)
    y_slider.grid(row=3, column=1)
    y_label = Label(app, text="y")
    y_label.grid(row=4, column=1)

    z_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=z_cb)
    z_slider.grid(row=5, column=1)
    z_label = Label(app, text="z")
    z_label.grid(row=6, column=1)

    zero_xyz = Button(app, width=_width, command=zero_xyz_cb)
    zero_xyz.grid(row=7, column=1)
    zero_xyz_label = Label(app, text="ZERO XYZ")
    zero_xyz_label.grid(row=8, column=1)

    roll_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                        command=roll_cb)
    roll_slider.grid(row=9, column=1)
    roll_label = Label(app, text="roll")
    roll_label.grid(row=10, column=1)

    pitch_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=pitch_cb)
    pitch_slider.grid(row=11, column=1)
    pitch_label = Label(app, text="pitch")
    pitch_label.grid(row=12, column=1)

    yaw_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                       command=yaw_cb)
    yaw_slider.grid(row=13, column=1)
    yaw_label = Label(app, text="yaw")
    yaw_label.grid(row=14, column=1)

    zero_rpy = Button(app, width=_width, command=zero_rpy_cb)
    zero_rpy.grid(row=15, column=1)
    zero_rpy_label = Label(app, text="ZERO RPY")
    zero_rpy_label.grid(row=16, column=1)

    zero_all = Button(app, width=_width, command=zero_all_cb)
    zero_all.grid(row=17, column=1)
    zero_all_label = Label(app, text="ZERO")
    zero_all_label.grid(row=18, column=1)
