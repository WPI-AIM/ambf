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

import functools
from Tkinter import *


class ObjectGUI:
    def __init__(self, obj_name, initial_xyz, initial_rpy, range_xyz, range_rpy, resolution):
        self.App = Tk()
        if initial_xyz:
            self.initial_xyz = initial_xyz
        else:
            self.initial_xyz = [0.0, 0.0, 0.0]

        if initial_rpy:
            self.initial_rpy = initial_rpy
        else:
            self.initial_rpy = [0, 0, 0]

        if range_xyz:
            self.range_xyz = range_xyz
        else:
            self.range_xyz = 1.0

        if range_rpy:
            self.range_rpy = range_rpy
        else:
            self.range_rpy = 3.14

        if resolution:
            self.resolution = resolution
        else:
            self.resolution = 0.0001

        self.px = self.initial_xyz[0]
        self.py = self.initial_xyz[1]
        self.pz = self.initial_xyz[2]
        self.rx = self.initial_rpy[0]
        self.ry = self.initial_rpy[1]
        self.rz = self.initial_rpy[2]

        self.px_slider = None
        self.py_slider = None
        self.pz_slider = None
        self.rx_slider = None
        self.ry_slider = None
        self.rz_slider = None

        self.px_scale = None
        self.py_scale = None
        self.pz_scale = None
        self.rx_scale = None
        self.ry_scale = None
        self.rz_scale = None

        self.cartesian_mode = 0
        self.create_gui(self.App, obj_name)

    # Def Init Function
    def get_app_handle(self):
        return self.App

    # Define Callbacks for Tkinter GUI Sliders
    def px_cb(self, val):
        self.px = float(val)

    def py_cb(self, val):
        self.py = float(val)

    def pz_cb(self, val):
        self.pz = float(val)

    def rx_cb(self, val):
        self.rx = float(val)

    def ry_cb(self, val):
        self.ry = float(val)

    def rz_cb(self, val):
        self.rz = float(val)

    def get_px_scale(self):
        try:
            scale = float(self.px_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def get_py_scale(self):
        try:
            scale = float(self.py_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def get_pz_scale(self):
        try:
            scale = float(self.pz_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def get_rx_scale(self):
        try:
            scale = float(self.rx_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def get_ry_scale(self):
        try:
            scale = float(self.ry_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def get_rz_scale(self):
        try:
            scale = float(self.rz_scale.get())
        except ValueError:
            scale = 1.0
        return scale

    def zero_all_cb(self):
        self.zero_p_cb()
        self.zero_r_cb()

    def zero_p_cb(self):
        self.px = self.initial_xyz[0]
        self.py = self.initial_xyz[1]
        self.pz = self.initial_xyz[2]
        self.px_slider.set(self.px)
        self.py_slider.set(self.py)
        self.pz_slider.set(self.pz)

    def zero_r_cb(self):
        self.rx = self.initial_rpy[0]
        self.ry = self.initial_rpy[1]
        self.rz = self.initial_rpy[2]
        self.rx_slider.set(self.rx)
        self.ry_slider.set(self.ry)
        self.rz_slider.set(self.rz)

    # Define Callbacks for Tkinter GUI Slider
    def effort_button_cb(self):
        global cartesian_mode
        self.cartesian_mode = 0

    # Define Callbacks for Tkinter GUI Slider
    def position_button_cb(self):
        global cartesian_mode
        self.cartesian_mode = 1

    # Define Callbacks for Tkinter GUI Slider
    def velocity_button_cb(self):
        global cartesian_mode
        self.cartesian_mode = 2

    def create_gui(self, app, obj_name):
        _width = 20
        _length = 300
        _resolution = 0.0001
        # Define Sliders and Labels

        row_count = 0
        obj_label = Label(app, text='CONTROLLING OBJECT: ' + obj_name, fg="Red")
        obj_label.grid(row=row_count, columnspan=2, pady=5)

        row_count = row_count + 1

        v = IntVar(value=0)
        eff_cb = Radiobutton(app, text="Effort", variable=v, indicatoron=False, value=0,
                             command=self.effort_button_cb)
        eff_cb.grid(row=row_count, column=0)

        pos_cb = Radiobutton(app, text="Position", variable=v, indicatoron=False, value=1,
                             command=self.position_button_cb)
        pos_cb.grid(row=row_count, column=1)

        vel_cb = Radiobutton(app, text="Velocity", variable=v, indicatoron=False, value=2,
                             command=self.velocity_button_cb)
        vel_cb.grid(row=row_count, column=2)

        row_count = row_count + 1

        min_v = self.initial_xyz[0] - self.range_xyz / 2.0
        max_v = self.initial_xyz[0] + self.range_xyz / 2.0
        self.px_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.px_cb)
        self.px_slider.grid(row=row_count, column=1)

        self.px_slider.set(self.px)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.px_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='PX Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="PX")
        label.grid(row=row_count, column=1, pady=5)

        row_count = row_count + 1

        min_v = self.initial_xyz[1] - self.range_xyz / 2.0
        max_v = self.initial_xyz[1] + self.range_xyz / 2.0
        self.py_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.py_cb)
        self.py_slider.grid(row=row_count, column=1)

        self.py_slider.set(self.py)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.py_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='PY Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="PY")
        label.grid(row=row_count, column=1)

        row_count = row_count + 1
        min_v = self.initial_xyz[2] - self.range_xyz / 2.0
        max_v = self.initial_xyz[2] + self.range_xyz / 2.0
        self.pz_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.pz_cb)
        self.pz_slider.grid(row=row_count, column=1)
        self.pz_slider.set(self.pz)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.pz_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='PZ Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="PZ")
        label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz = Button(app, width=_width, command=self.zero_p_cb)
        zero_xyz.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz_label = Label(app, text="Reset Position")
        zero_xyz_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[0] - self.range_rpy / 2.0
        max_v = self.initial_rpy[0] + self.range_rpy / 2.0
        self.rx_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.rx_cb)
        self.rx_slider.grid(row=row_count, column=1)
        self.rx_slider.set(self.rx)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.rx_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='RX Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="RX")
        label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[1] - self.range_rpy / 2.0
        max_v = self.initial_rpy[1] + self.range_rpy / 2.0
        self.ry_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.ry_cb)
        self.ry_slider.grid(row=row_count, column=1)
        self.ry_slider.set(self.ry)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.ry_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='RY Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="RY")
        label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[2] - self.range_rpy / 2.0
        max_v = self.initial_rpy[2] + self.range_rpy / 2.0
        self.rz_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.rz_cb)
        self.rz_slider.grid(row=row_count, column=1)
        self.rz_slider.set(self.rz)

        sv = StringVar()
        scale_input = Entry(app, textvariable=sv)
        scale_input.grid(row=row_count, column=0)
        sv.set("1.0")
        self.rz_scale = sv

        row_count = row_count + 1

        scale_label = Label(app, text='RZ Cmd Scale')
        scale_label.grid(row=row_count, column=0)

        label = Label(app, text="RZ")
        label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy = Button(app, width=_width, command=self.zero_r_cb)
        zero_rpy.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy_label = Label(app, text="Reset Rotation")
        zero_rpy_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all = Button(app, width=_width, command=self.zero_all_cb)
        zero_all.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all_label = Label(app, text="Reset All")
        zero_all_label.grid(row=row_count, column=1)
