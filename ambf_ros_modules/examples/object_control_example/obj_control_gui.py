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

        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        self.ro = self.initial_rpy[0]
        self.pi = self.initial_rpy[1]
        self.ya = self.initial_rpy[2]
        self.x_slider = None
        self.y_slider = None
        self.z_slider = None
        self.ro_slider = None
        self.pi_slider = None
        self.ya_slider = None

        self.cartesian_mode = 0
        self.create_gui(self.App, obj_name)

    # Def Init Function
    def get_app_handle(self):
        return self.App

    # Define Callbacks for Tkinter GUI Sliders
    def x_cb(self, val):
        self.x = float(val)

    def y_cb(self, val):
        self.y = float(val)

    def z_cb(self, val):
        self.z = float(val)

    def roll_cb(self, val):
        self.ro = float(val)

    def pitch_cb(self, val):
        self.pi = float(val)

    def yaw_cb(self, val):
        self.ya = float(val)

    def zero_all_cb(self):
        self.zero_xyz_cb()
        self.zero_rpy_cb()

    def zero_xyz_cb(self):
        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        self.x_slider.set(self.x)
        self.y_slider.set(self.y)
        self.z_slider.set(self.z)

    def zero_rpy_cb(self):
        self.ro = self.initial_rpy[0]
        self.pi = self.initial_rpy[1]
        self.ya = self.initial_rpy[2]
        self.ro_slider.set(self.ro)
        self.pi_slider.set(self.pi)
        self.ya_slider.set(self.ya)

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
        self.x_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.x_cb)
        self.x_slider.grid(row=row_count, column=1)

        self.x_slider.set(self.x)

        row_count = row_count + 1

        x_label = Label(app, text="x")
        x_label.grid(row=row_count, column=1, pady=5)

        row_count = row_count + 1

        min_v = self.initial_xyz[1] - self.range_xyz / 2.0
        max_v = self.initial_xyz[1] + self.range_xyz / 2.0
        self.y_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.y_cb)
        self.y_slider.grid(row=row_count, column=1)

        self.y_slider.set(self.y)

        row_count = row_count + 1

        y_label = Label(app, text="y")
        y_label.grid(row=row_count, column=1)

        row_count = row_count + 1
        min_v = self.initial_xyz[2] - self.range_xyz / 2.0
        max_v = self.initial_xyz[2] + self.range_xyz / 2.0
        self.z_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.z_cb)
        self.z_slider.grid(row=row_count, column=1)
        self.z_slider.set(self.z)

        row_count = row_count + 1

        z_label = Label(app, text="z")
        z_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz = Button(app, width=_width, command=self.zero_xyz_cb)
        zero_xyz.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz_label = Label(app, text="Reset Position")
        zero_xyz_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[0] - self.range_rpy / 2.0
        max_v = self.initial_rpy[0] + self.range_rpy / 2.0
        self.ro_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.roll_cb)
        self.ro_slider.grid(row=row_count, column=1)
        self.ro_slider.set(self.ro)

        row_count = row_count + 1

        roll_label = Label(app, text="roll")
        roll_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[1] - self.range_rpy / 2.0
        max_v = self.initial_rpy[1] + self.range_rpy / 2.0
        self.pi_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.pitch_cb)
        self.pi_slider.grid(row=row_count, column=1)
        self.pi_slider.set(self.pi)

        row_count = row_count + 1

        pitch_label = Label(app, text="pitch")
        pitch_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        min_v = self.initial_rpy[2] - self.range_rpy / 2.0
        max_v = self.initial_rpy[2] + self.range_rpy / 2.0
        self.ya_slider = Scale(app, from_=min_v, to=max_v, resolution=self.resolution, width=_width, length=_length, orient=HORIZONTAL,
                               command=self.yaw_cb)
        self.ya_slider.grid(row=row_count, column=1)
        self.ya_slider.set(self.ya)

        row_count = row_count + 1

        yaw_label = Label(app, text="yaw")
        yaw_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy = Button(app, width=_width, command=self.zero_rpy_cb)
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
