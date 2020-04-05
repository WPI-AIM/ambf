from ambf_client import Client
import numpy as np
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
from psmIK import compute_IK, test_ik
from psmFK import compute_FK
from transformations import euler_from_matrix
import time
import random


_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
# You can print the names of objects found. We should see all the links found
print(_client.get_obj_names())
psm_handle = _client.get_obj_handle('psm/baselink')
time.sleep(5)
current_joint = np.zeros(7)
current_joint_pos = np.zeros(7)
edited_joint_pos = np.zeros(7)
prev_pos = 0
count = 0
try:
    while True:
         joints_to_control = np.array(['baselink-yawlink', 'yawlink-pitchbacklink',
                                            'pitchendlink-maininsertionlink', 'maininsertionlink-toolrolllink',
                                            'toolrolllink-toolpitchlink', 'toolpitchlink-toolgripper1link',
                                            'toolpitchlink-toolgripper2link'])
         for val, jt_name in enumerate(joints_to_control):
             current_joint_pos[val] = round(psm_handle.get_joint_pos(jt_name), 3)
         # print("Joint 0 pos is ", current_joint_pos[0])
         fk_tip = compute_FK(current_joint_pos)
         xyz_pos = fk_tip[0:3, 3].reshape((1, 3))
         print(xyz_pos)
         req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
         # rand_number = np.random.uniform(-1.0, 1.0)
         # for val, jt_name in enumerate(joints_to_control):
         #     if val == 0:
         #        # print("error is ", prev_pos-current_joint_pos[0])
         #        cmd_pos = current_joint_pos[val] - rand_number
         #        psm_handle.set_joint_pos(jt_name, cmd_pos)
         #        # psm_handle.set_joint_pos(jt_name, 0.2)
         #
         #     # if val == 2:
         #     #     psm_handle.set_joint_pos(jt_name, 0.12)
         #     #     psm_handle.set_joint_pos(joints_to_control[0], 0.2)
         #     else:
         #         psm_handle.set_joint_pos(jt_name, 0)

         # count -= 0.05
         # if count < -0.3:
         #     count = 0

         # print("Random number is ", rand_number)
         # psm_handle.set_joint_pos(joints_to_control[0], c)
         # time.sleep(0.5)
         # prev_pos = cmd_pos
         # print(c)
except KeyboardInterrupt:
    _client.clean_up()
    pass

# mat = np.array([[-0.27636201, -0.12028724,  0.95349621],
#  [-0.12288987,  0.98841486,  0.08907388  ],
#  [-0.95316427, -0.09255839, -0.2879424  ]])
# eul = euler_from_matrix(mat)
# print("eul is ", eul)
# Lastly to cleanup
# new state is  [[-0.27636201 -0.12028724  0.95349621  0.10978525]
#  [-0.12288987  0.98841486  0.08907388  0.05940633]
#  [-0.95316427 -0.09255839 -0.2879424  -0.03949025]
#  [ 0.          0.          0.          1.        ]]