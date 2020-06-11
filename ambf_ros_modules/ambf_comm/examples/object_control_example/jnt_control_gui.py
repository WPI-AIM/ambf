
import rospy
import time
import functools
import Tkinter as tk
from Tkinter import *

App = Tk()
jnt_cmd = []
jnt_mode = []


def init(obj_name, num_jnts, jnt_names):
    global jnt_cmd
    global App
    create_gui(App, obj_name, num_jnts, jnt_names)


# Def Init Function
def get_app_handle():
    global App
    return App


# Define Callbacks for Tkinter GUI Sliders
def slider_cb(val, idx):
    global jnt_cmd
    jnt_cmd[idx] = float(val)


# Define Callbacks for Tkinter GUI Sliders
def check_button_cb(idx):
    global jnt_mode
    jnt_mode[idx] = not jnt_mode[idx]


def create_gui(app, obj_name, num_jnts, jnt_names):
    global jnt_cmd, jnt_mode
    _width = 20
    _length = 300
    _resolution = 0.001
    # Define Sliders and Labels
    sliders = []
    check_buttons = []
    jnt_cmd = [0.0]*num_jnts
    jnt_mode = [0]*num_jnts
    for i in range(num_jnts):
        label = Label(app, text=jnt_names[i])
        # label.pack(expand=YES, fill=Y)
        label.grid(row=i, column=0)

        slider = Scale(app, from_=-1, to=1, resolution=_resolution, orient=HORIZONTAL,
                         command=functools.partial(slider_cb, idx=i))
        # slider.pack(side=tk.LEFT)
        slider.grid(row=i, column=1)
        sliders.append(slider)

        v = BooleanVar()
        cb = Checkbutton(app, text="Effort/Position", variable=v, command=functools.partial(check_button_cb, idx=i))
        # cb.pack(side=tk.RIGHT)
        cb.grid(row=i, column=2)
        check_buttons.append(cb)


    # zero_all = Button(app, width=_width, command=functools.partial(slider_cb, idx=i))
    # zero_all.pack(expand=YES, fill=Y)
    # zero_all_label = Label(app, text="ZERO")
    # zero_all_label.pack(expand=YES, fill=Y)
