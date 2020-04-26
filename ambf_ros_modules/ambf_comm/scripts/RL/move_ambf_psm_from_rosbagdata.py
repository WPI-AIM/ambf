from ambf_client import Client
import numpy as np
import time
from psmFK import compute_FK
from psmIK import *


if __name__ == "__main__":

    # file = "/home/vignesh/Thesis_Suture_data/trial2/suture_data_trial2/2020-02-25_20:02:45.832953.bag"
    # topic = "/dvrk/PSM2/position_cartesian_current"
    # data = getData(file, topic)
    # print(data)
    data = np.load('rosbag_data.npy')
    print(data.shape)
    print(data[0])
    _client = Client()
    _client.connect()
    print(_client.get_obj_names())
    psm_handle = _client.get_obj_handle('psm/baselink')
    time.sleep(5)
    joint_pos = np.zeros(7)
    count = 0
    error_threshold = 0.01
    # desired_pos = np.array([[0.5, -0.33, 0.149, 0, 0, 0., 0.],
    #                         [-1.605, 0.026, 0.1, 1.569, -1.572, 0., -0.]])
    joint_thresh = joint_pos
    counter = 0
    joints_to_control = np.array(['baselink-yawlink',
                                  'yawlink-pitchbacklink',
                                  'pitchendlink-maininsertionlink',
                                  'maininsertionlink-toolrolllink',
                                  'toolrolllink-toolpitchlink',
                                  'toolpitchlink-toolgripper1link',
                                  'toolpitchlink-toolgripper2link'])

    current_end_effector_frame = compute_FK(np.zeros(7))
    desired_end_effector_frame = current_end_effector_frame
    for i in range(0, data.shape[0]):
        for j in range(3):
            print("data ij ", data[i, j])
            desired_end_effector_frame[j, 3] = round(data[i, j], 3)
        desired_joint_pos = np.around(compute_IK(convert_mat_to_frame(desired_end_effector_frame)), 3)
        desired_joint_pos = np.append(desired_joint_pos, 0)
        # print(desired_joint_pos)
        count = 0

        while True:

            for joint_idx, jt_name in enumerate(joints_to_control):
                psm_handle.set_joint_pos(jt_name, desired_joint_pos[joint_idx])
                joint_pos[joint_idx] = round(psm_handle.get_joint_pos(jt_name), 3)

            error_in_pos = np.around(np.subtract(desired_joint_pos, joint_pos), decimals=3)
            # print("Error in pos ", error_in_pos)
            count += 1
            if np.all(np.abs(error_in_pos) <= 0.01) or count > 75:
                break