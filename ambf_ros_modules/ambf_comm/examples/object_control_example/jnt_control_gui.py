import functools
from Tkinter import *


class JointGUI:
    def __init__(self, obj_name, num_jnts, jnt_names):
        self.App = Tk()
        self.jnt_cmd = []
        self.jnt_mode = []
        self.cmd_scale = []

        self.create_gui(self.App, obj_name, num_jnts, jnt_names)

    def get_app_handle(self):
        return self.App

    # Define Callbacks for Tkinter GUI Sliders
    def scale_cb(self, val, idx):
        global cmd_scale
        self.cmd_scale[idx] = float(val)

    # Define Callbacks for Tkinter GUI Sliders
    def slider_cb(self, val, idx):
        global jnt_cmd
        self.jnt_cmd[idx] = float(val)

    # Define Callbacks for Tkinter GUI Sliders
    def effort_button_cb(self, idx):
        global jnt_mode
        self.jnt_mode[idx] = 0

    # Define Callbacks for Tkinter GUI Sliders
    def position_button_cb(self, idx):
        global jnt_mode
        self.jnt_mode[idx] = 1

    # Define Callbacks for Tkinter GUI Sliders
    def velocity_button_cb(self, idx):
        global jnt_mode
        self.jnt_mode[idx] = 2

    def create_gui(self, app, obj_name, num_jnts, jnt_names):
        _width = 20
        _length = 300
        _resolution = 0.001
        # Define Sliders and Labels
        sliders = []
        check_buttons = []
        self.jnt_cmd = [0.0]*num_jnts
        self.jnt_mode = [0]*num_jnts
        self.cmd_scale = [0]*num_jnts

        # obj_label = Label(app, text='CONTROLLING OBJECT: ' + obj_name, fg="Red")
        # obj_label.pack(row=0, columnspan=2, pady=5)

        min = -10
        max = 10
        for i in range(2*num_jnts):
            if i % 2 == 0:
                sv = StringVar()
                scale_input = Entry(app, textvariable=sv)
                scale_input.grid(row=i, column=1)
                sv.set("1.0")
                self.cmd_scale[i/2] = sv

                slider = Scale(app, from_=min, to=max, resolution=_resolution, orient=HORIZONTAL,
                                 command=functools.partial(self.slider_cb, idx=i/2))
                slider.grid(row=i, column=2)
                sliders.append(slider)

                v = IntVar(value=0)
                eff_cb = Radiobutton(app, text="Effort", variable=v, indicatoron=False, value=0,
                                  command=functools.partial(self.effort_button_cb, idx=i/2))
                eff_cb.grid(row=i, column=3)

                pos_cb = Radiobutton(app, text="Position", variable=v, indicatoron=False, value=1,
                                  command=functools.partial(self.position_button_cb, idx=i/2))
                pos_cb.grid(row=i, column=4)

                vel_cb = Radiobutton(app, text="Velocity", variable=v, indicatoron=False, value=2,
                                  command=functools.partial(self.velocity_button_cb, idx=i/2))
                vel_cb.grid(row=i, column=5)

            else:
                label = Label(app, text=jnt_names[i/2])
                label.grid(row=i, columnspan=5)
