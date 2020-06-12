import functools
from Tkinter import *


class ObjectGUI:
    def __init__(self, obj_name):
        self.App = Tk()
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x_slider = None
        self.y_slider = None
        self.z_slider = None
        self.roll_slider = None
        self.pitch_slider = None
        self.yaw_slider = None
        self.cartesian_mode = 0
        self.create_gui(self.App, obj_name)

    # Def Init Function
    def get_app_handle(self):
        return self.App

    # Define Callbacks for Tkinter GUI Sliders
    def x_cb(self, val):
        self.x = float(val)

    def y_cb(self, val):
        global y
        self.y = float(val)

    def z_cb(self, val):
        global z
        self.z = float(val)

    def roll_cb(self, val):
        global roll
        self.roll = float(val)

    def pitch_cb(self, val):
        global pitch
        self.pitch = float(val)

    def yaw_cb(self, val):
        global yaw
        self.yaw = float(val)

    def zero_all_cb(self):
        self.zero_xyz_cb()
        self.zero_rpy_cb()

    def zero_xyz_cb(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_slider.set(0)
        self.y_slider.set(0)
        self.z_slider.set(0)

    def zero_rpy_cb(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_slider.set(0)
        self.pitch_slider.set(0)
        self.yaw_slider.set(0)

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
        _resolution = 0.001
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

        min = -100
        max = 100
        self.x_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.x_cb)
        self.x_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        x_label = Label(app, text="x")
        x_label.grid(row=row_count, column=1, pady=5)

        row_count = row_count + 1

        self.y_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.y_cb)
        self.y_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        y_label = Label(app, text="y")
        y_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        self.z_slider = Scale(app, from_=min, to=max, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                         command=self.z_cb)
        self.z_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        z_label = Label(app, text="z")
        z_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz = Button(app, width=_width, command=self.zero_xyz_cb)
        zero_xyz.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_xyz_label = Label(app, text="ZERO XYZ")
        zero_xyz_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        self.roll_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                            command=self.roll_cb)
        self.roll_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        roll_label = Label(app, text="roll")
        roll_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        self.pitch_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                             command=self.pitch_cb)
        self.pitch_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        pitch_label = Label(app, text="pitch")
        pitch_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        self.yaw_slider = Scale(app, from_=-2, to=2, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL,
                           command=self.yaw_cb)
        self.yaw_slider.grid(row=row_count, column=1)

        row_count = row_count + 1

        yaw_label = Label(app, text="yaw")
        yaw_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy = Button(app, width=_width, command=self.zero_rpy_cb)
        zero_rpy.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_rpy_label = Label(app, text="ZERO RPY")
        zero_rpy_label.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all = Button(app, width=_width, command=self.zero_all_cb)
        zero_all.grid(row=row_count, column=1)

        row_count = row_count + 1

        zero_all_label = Label(app, text="ZERO")
        zero_all_label.grid(row=row_count, column=1)
