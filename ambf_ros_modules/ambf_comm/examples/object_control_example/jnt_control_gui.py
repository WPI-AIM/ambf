
import rospy
import time
import functools
from Tkinter import *

App = Tk()
jnt_cmd = []


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


def create_gui(app, obj_name, num_jnts, jnt_names):
    global jnt_cmd
    _width = 20
    _length = 300
    _resolution = 0.001
    # Define Sliders and Labels
    sliders = []
    jnt_cmd = [0.0]*num_jnts
    for i in range(num_jnts):
        slider = Scale(app, from_=-1, to=1, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=functools.partial(slider_cb, idx=i))
        slider.pack(expand=YES, fill=Y)
        label = Label(app, text=jnt_names[i])
        label.pack(expand=YES, fill=Y)
        sliders.append(slider)

    # zero_all = Button(app, width=_width, command=functools.partial(slider_cb, idx=i))
    # zero_all.pack(expand=YES, fill=Y)
    # zero_all_label = Label(app, text="ZERO")
    # zero_all_label.pack(expand=YES, fill=Y)
