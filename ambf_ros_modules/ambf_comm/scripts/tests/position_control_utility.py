
#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019, AMBF
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

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

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
from Tkinter import *

App = Tk()
x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
K_lin = 100
D_lin = 1
K_ang = 5
D_ang = 1


def init():
    global K_ang, D_ang
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


def k_lin_cb(val):
    global K_lin
    K_lin = float(val)


def d_lin_cb(val):
    global D_lin
    D_lin = float(val)


def k_ang_cb(val):
    global K_ang
    K_ang = float(val)


def d_ang_cb(val):
    global D_ang
    D_ang = float(val)


def create_gui(app):
    _width = 20
    _length = 300
    _resolution = 0.1
    # Define Sliders and Labels
    x_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=x_cb)
    x_slider.pack(expand=YES, fill=Y)
    x_label = Label(app, text="x")
    x_label.pack(expand=YES, fill=Y)

    y_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=y_cb)
    y_slider.pack(expand=YES, fill=Y)
    y_label = Label(app, text="y")
    y_label.pack(expand=YES, fill=Y)

    z_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=z_cb)
    z_slider.pack(expand=YES, fill=Y)
    z_label = Label(app, text="z")
    z_label.pack(expand=YES, fill=Y)

    roll_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=roll_cb)
    roll_slider.pack(expand=YES, fill=Y)
    roll_label = Label(app, text="roll")
    roll_label.pack(expand=YES, fill=Y)

    pitch_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=pitch_cb)
    pitch_slider.pack(expand=YES, fill=Y)
    pitch_label = Label(app, text="pitch")
    pitch_label.pack(expand=YES, fill=Y)

    yaw_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=yaw_cb)
    yaw_slider.pack(expand=YES, fill=Y)
    yaw_label = Label(app, text="yaw")
    yaw_label.pack(expand=YES, fill=Y)

    p_lin_slider = Scale(app, from_=0.1, to=2000, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=k_lin_cb)
    p_lin_slider.set(100)
    p_lin_slider.pack()
    kl_label = Label(app, text="K lin")
    kl_label.pack()

    d_lin_slider = Scale(app, from_=0, to=100, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=d_lin_cb)
    d_lin_slider.set(10)
    d_lin_slider.pack()
    dl_label = Label(app, text="D lin")
    dl_label.pack()

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
