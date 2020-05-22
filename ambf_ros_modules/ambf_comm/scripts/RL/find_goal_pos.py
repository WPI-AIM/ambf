from ambf_client import Client
import numpy as np
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
# from psmIK import compute_IK, test_ik
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
current_joint_pos = np.zeros(7)
joint_pos = np.zeros(7)
prev_pos = 0
count = 0
error_threshold = 0.05

try:
    while True:
        joints_to_control = np.array(['baselink-yawlink',
                                      'yawlink-pitchbacklink',
                                      'pitchendlink-maininsertionlink',
                                      'maininsertionlink-toolrolllink',
                                      'toolrolllink-toolpitchlink',
                                      'toolpitchlink-toolgripper1link',
                                      'toolpitchlink-toolgripper2link'])

        desired_pos = [0.0, 0., 0.2, 0, 0, 0., 0.]
        # for joint_idx, jt_name in enumerate(joints_to_control):
        psm_handle.set_joint_pos(joints_to_control[0], desired_pos[0])
        psm_handle.set_joint_pos(joints_to_control[2], desired_pos[2])
        # tip (-0.17, 0.17) (-0.16, 0.13)  (-0.1,-0.03)
        # for joint_idx, jt_name in enumerate(joints_to_control):
        #     error_in_pos = 1
        #     while abs(error_in_pos) > error_threshold:
        #         error_in_pos = psm_handle.get_joint_pos(jt_name) - desired_pos[joint_idx]
        #         # print("Error in pos ", joint_idx, psm_handle.get_joint_pos(jt_name), values[joint_idx], error_in_pos)
        #         time.sleep(0.00001)

        for joint_idx, jt_name in enumerate(joints_to_control):
            joint_pos[joint_idx] = round(psm_handle.get_joint_pos(jt_name), 3)
        # print("Joints pos is ", joint_pos)


        # for joint_idx, jt_name in enumerate(joints_to_control):
        #     current_joint_pos[joint_idx] = round(psm_handle.get_joint_pos(jt_name), 3)
        # print("Joint 0 pos is ", current_joint_pos[0])
        fk_tip = compute_FK(joint_pos)
        xyz_pos = fk_tip[0:3, 3].reshape((1, 3))
        print("tip pos", xyz_pos, "joint pos ", joint_pos)
        # req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
        # rand_number = np.random.uniform(-1.0, 1.0)

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