#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020, AMBF
#     (https://github.com/WPI-AIM/ambf)
#
#     All rights reserved.
#
#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:
#
#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.
#
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

import rospy
import time
from Tkinter import *

App = Tk()
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
gripper = 0
x_slider = None
y_slider = None
z_slider = None
roll_slider = None
pitch_slider = None
yaw_slider = None
gripper_slider = None


def init():
    global x, y, z
    global roll, pitch, yaw
    global App
    create_gui(App)


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


def gripper_cb(val):
    global gripper
    gripper = float(val)


def create_gui(app):
    global x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider
    _width = 20
    _length = 300
    _resolution = 0.001
    # Define Sliders and Labels
    x_slider = Scale(app, from_=-1, to=1, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=x_cb)
    x_slider.pack(expand=YES, fill=Y)
    x_label = Label(app, text="x")
    x_label.pack(expand=YES, fill=Y)

    y_slider = Scale(app, from_=-1, to=1, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=y_cb)
    y_slider.pack(expand=YES, fill=Y)
    y_label = Label(app, text="y")
    y_label.pack(expand=YES, fill=Y)

    z_slider = Scale(app, from_=-1, to=1, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                     command=z_cb)
    z_slider.pack(expand=YES, fill=Y)
    z_label = Label(app, text="z")
    z_label.pack(expand=YES, fill=Y)

    zero_xyz = Button(app, width=_width, command=zero_xyz_cb)
    zero_xyz.pack(expand=YES, fill=Y)
    zero_xyz_label = Label(app, text="ZERO XYZ")
    zero_xyz_label.pack(expand=YES, fill=Y)

    roll_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                        command=roll_cb)
    roll_slider.pack(expand=YES, fill=Y)
    roll_label = Label(app, text="roll")
    roll_label.pack(expand=YES, fill=Y)

    pitch_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=pitch_cb)
    pitch_slider.pack(expand=YES, fill=Y)
    pitch_label = Label(app, text="pitch")
    pitch_label.pack(expand=YES, fill=Y)

    yaw_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                       command=yaw_cb)
    yaw_slider.pack(expand=YES, fill=Y)
    yaw_label = Label(app, text="yaw")
    yaw_label.pack(expand=YES, fill=Y)

    zero_rpy = Button(app, width=_width, command=zero_rpy_cb)
    zero_rpy.pack(expand=YES, fill=Y)
    zero_rpy_label = Label(app, text="ZERO RPY")
    zero_rpy_label.pack(expand=YES, fill=Y)

    gripper_slider = Scale(app, from_=0, to=1, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=gripper_cb)
    gripper_slider.pack(expand=YES, fill=Y)
    gripper_label = Label(app, text="gripper")
    gripper_label.pack(expand=YES, fill=Y)

    zero_all = Button(app, width=_width, command=zero_all_cb)
    zero_all.pack(expand=YES, fill=Y)
    zero_all_label = Label(app, text="ZERO")
    zero_all_label.pack(expand=YES, fill=Y)
