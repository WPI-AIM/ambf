
import rospy
import time
import functools
import Tkinter as tk
from Tkinter import *

App = Tk()
jnt_cmd = []
jnt_mode = []
cmd_scale = []


def init(obj_name, num_jnts, jnt_names):
    global jnt_cmd
    global App
    create_gui(App, obj_name, num_jnts, jnt_names)


# Def Init Function
def get_app_handle():
    global App
    return App


# Define Callbacks for Tkinter GUI Sliders
def scale_cb(val, idx):
    global cmd_scale
    cmd_scale[idx] = float(val)


# Define Callbacks for Tkinter GUI Sliders
def slider_cb(val, idx):
    global jnt_cmd
    jnt_cmd[idx] = float(val)


# Define Callbacks for Tkinter GUI Sliders
def effort_button_cb(idx):
    global jnt_mode
    jnt_mode[idx] = 0


# Define Callbacks for Tkinter GUI Sliders
def position_button_cb(idx):
    global jnt_mode
    jnt_mode[idx] = 1


# Define Callbacks for Tkinter GUI Sliders
def velocity_button_cb(idx):
    global jnt_mode
    jnt_mode[idx] = 2


def create_gui(app, obj_name, num_jnts, jnt_names):
    global jnt_cmd, jnt_mode, cmd_scale
    _width = 20
    _length = 300
    _resolution = 0.001
    # Define Sliders and Labels
    sliders = []
    check_buttons = []
    jnt_cmd = [0.0]*num_jnts
    jnt_mode = [0]*num_jnts
    cmd_scale = [0]*num_jnts
    min = -10
    max = 10
    for i in range(2*num_jnts):
        if i % 2 == 0:
            sv = StringVar()
            scale_input = Entry(app, textvariable=sv)
            scale_input.grid(row=i, column=1)
            sv.set("1.0")
            cmd_scale[i/2] = sv

            slider = Scale(app, from_=min, to=max, resolution=_resolution, orient=HORIZONTAL,
                             command=functools.partial(slider_cb, idx=i/2))
            slider.grid(row=i, column=2)
            sliders.append(slider)

            v = IntVar(value=0)
            eff_cb = Radiobutton(app, text="Effort", variable=v, indicatoron=False, value=0,
                              command=functools.partial(effort_button_cb, idx=i/2))
            eff_cb.grid(row=i, column=3)

            pos_cb = Radiobutton(app, text="Position", variable=v, indicatoron=False, value=1,
                              command=functools.partial(position_button_cb, idx=i/2))
            pos_cb.grid(row=i, column=4)

            vel_cb = Radiobutton(app, text="Velocity", variable=v, indicatoron=False, value=2,
                              command=functools.partial(velocity_button_cb, idx=i/2))
            vel_cb.grid(row=i, column=5)

        else:
            label = Label(app, text=jnt_names[i/2])
            label.grid(row=i, columnspan=5)


    # zero_all = Button(app, width=_width, command=functools.partial(slider_cb, idx=i))
    # zero_all.pack(expand=YES, fill=Y)
    # zero_all_label = Label(app, text="ZERO")
    # zero_all_label.pack(expand=YES, fill=Y)
