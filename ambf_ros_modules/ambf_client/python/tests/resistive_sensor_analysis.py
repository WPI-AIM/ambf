from ambf_client import Client
import time
import numpy as np
import matplotlib.pyplot as plt


c = Client()

c.connect()

b = c.get_obj_handle('BoxTool')

n = 1000

p_arr = np.zeros((n, 3))
t_arr = np.zeros((n, 1))

time.sleep(0.3)

b.set_pos(0, 0, 0)
time.sleep(1.0)
b.set_rpy(0, -1.57, 0)
time.sleep(2.0)

b.set_force(0,0,0)
time.sleep(8.0)

for i in range(0, n):
    p = b.get_pos()
    p_arr[i, :] = np.array([p.x, p.y, p.z])
    t_arr[i] = time.clock()
    time.sleep(0.005)

plt.rcParams.update({'font.size': 15})

plt.plot(t_arr, p_arr)
plt.grid(True)
plt.ylim([-0.2, 0.2])
plt.xlabel('Time (s)')
plt.ylabel('Positions (m)')
plt.legend(['X', 'Y', 'Z'])
plt.show()
