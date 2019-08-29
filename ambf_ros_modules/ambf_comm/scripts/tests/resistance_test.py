from ambf_client import Client
import numpy as np
import time
import matplotlib.pyplot as plt

depth = -1.4
scale = 1.0
ts = 1.0 / 1000.0
episode = 20.0
dt = 0.01

trajectory_type = 1
y_min = -1.
y_max = 1.

c = Client()
c.connect()
box = c.get_obj_handle('BoxTool')
# diff = c.get_obj_handle('diff')

box.set_rpy(0, -1.57079, 0)

n_steps = int(episode / dt)
t = 0

t_series = []
error_series = []

# sinosoidal trajectory
if trajectory_type == 0:
    box.set_pos(0, 0, depth)
    time.sleep(2.0)
    y_start = box.get_pos().y

    for i in range(0, n_steps):
        t = i * ts
        y_command = y_start + scale * np.sin(t)
        y_state = box.get_pos().y
        box.set_pos(0, y_command, depth)
        box.set_rpy(0, -1.57079, 0)
        dy = y_command - y_state
        # diff.set_pos(0, dy, 0)
        time.sleep(dt)

# Line trajectory
elif trajectory_type == 1:
    box.set_pos(0, y_min, depth)
    time.sleep(2.0)
    y_command = box.get_pos().y

    for i in range(0, n_steps):
        ds = (y_max - y_min) / float(n_steps)
        y_state = box.get_pos().y
        y_command = y_command + ds
        box.set_pos(0, y_command, depth)
        box.set_rpy(0, -1.57079, 0)
        dy = y_command - y_state
        error_series.append(dy)
        t = t+dt
        t_series.append(t)
        # diff.set_pos(0, dy, 0)
        time.sleep(dt)

plt.plot(t_series, error_series)
plt.grid(True)
# plt.show()
plt.legend(['eT'])
plt.ylabel('Tangential Error (eT)')
plt.xlabel('Time (s)')
plt.show()