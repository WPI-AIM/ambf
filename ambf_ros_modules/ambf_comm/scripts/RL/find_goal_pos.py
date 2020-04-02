from ambf_client import Client
import numpy as np
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
from psmIK import compute_IK, test_ik
from psmFK import compute_FK
from transformations import euler_from_matrix
import time


_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
# You can print the names of objects found. We should see all the links found
print(_client.get_obj_names())
psm_handle = _client.get_obj_handle('psm/baselink')
time.sleep(5)
current_joint = np.zeros(6)
current_joint_pos = np.zeros(6)
edited_joint_pos = np.zeros(6)
count = 0
try:
    while True:
         # joints_to_control = np.array(['baselink-yawlink', 'yawlink-pitchbacklink',
                                           # 'pitchendlink-maininsertionlink', 'maininsertionlink-toolrolllink',
                                           # 'toolrolllink-toolpitchlink', 'toolpitchlink-toolgripper2link'])
         joints_to_control = np.array(['baselink-yawlink', 'yawlink-pitchbacklink',
                                       'pitchendlink-maininsertionlink', 'maininsertionlink-toolrolllink',
                                       'toolrolllink-toolpitchlink', 'toolpitchlink-toolgripper2link'])
         for val, jt_name in enumerate(joints_to_control):
             current_joint_pos[val] = round(psm_handle.get_joint_pos(jt_name), 3)
         # fk_tip = compute_FK(current_joint_pos)
         # print("Joint pos ", current_joint_pos)
         # # eul = euler_from_matrix(fk_tip[0:3, 0:3])
         # # print("fka angle is  ", fk_tip)
         # xyz_pos = fk_tip[0:3, 3].reshape((1, 3))
         # # (-1.570821806172054, 0.000903827565258327, 3.141588156751553)
         # req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
         # # print("new state is ", xyz_pos, "angle is ", req_rot)
         # new_state_joint_pos, _ = test_ik(xyz_pos[0, 0], xyz_pos[0, 1], xyz_pos[0, 2],
         #                                  req_rot[0], req_rot[1], req_rot[2])
         # # print("Computed joint pos ", new_state_joint_pos)
         # edited_joint_pos = new_state_joint_pos
         # if not -np.pi <= new_state_joint_pos[3] <= np.pi:
         #    edited_joint_pos[3] = new_state_joint_pos[3] + np.pi
         # if not -np.pi <= new_state_joint_pos[4] <= np.pi:
         #    edited_joint_pos[4] = new_state_joint_pos[4] + np.pi
         # # edited_joint_pos[5] = new_state_joint_pos[5] + np.pi
         # print("Computed joint pos ", edited_joint_pos)
         #
         # comp_fk = compute_FK(new_state_joint_pos)
         # eul2 = euler_from_matrix(comp_fk[0:3, 0:3], axes='szyx')
         # print("Computed FK ", comp_fk[0:3, 3].reshape((1, 3)), "angle ", eul2)
         # for val, jt_name in enumerate(joints_to_control):
         #     psm_handle.set_joint_pos(jt_name, new_state_joint_pos[val])
         # for val, jt_name in enumerate(joints_to_control):
         #     psm_handle.set_joint_pos(jt_name, current_joint_pos[val])
         # print("Current joint values ", current_joint_pos)
         # for val, jt_name in enumerate(joints_to_control):
         #     current_joint[val] = psm_handle.get_joint_pos(jt_name)
         # for val, jt_name in enumerate(joints_to_control):
         #     current_joint_pos[val] = psm_handle.get_joint_pos(jt_name)
         # for val, jt_name in enumerate(joints_to_control):
         val = 3
         psm_handle.set_joint_pos(joints_to_control[val], -3.0456)
             # if val == 2:
             #    psm_handle.set_joint_pos(jt_name, current_joint[val] + count)
             #    count += 0.005
             #    if count > 0.2:
             #        count = 0
             # if val == 2:
             #     psm_handle.set_joint_pos(jt_name, 0.12)
             #     psm_handle.set_joint_pos(joints_to_control[0], 0.2)
             # else:
             #    psm_handle.set_joint_pos(jt_name, 0)
         # print("Current pos ", current_joint_pos)
         c = psm_handle.get_joint_pos(joints_to_control[val])
         print(c)
         # if count ==10000:

         # psm_handle.set_joint_pos(joints_to_control[0], c)
         time.sleep(0.1)
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