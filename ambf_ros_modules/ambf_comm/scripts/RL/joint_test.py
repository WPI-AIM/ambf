from ambf_client import Client
import numpy as np
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
joint_pos = np.zeros(7)
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

        desired_pos = [0.5, -0.33, 0.149, 0, 0, 0., 0.]

        count = 0
        while True:
            reached_joint_pos = np.zeros(7)
            for joint_idx, jt_name in enumerate(joints_to_control):
                psm_handle.set_joint_pos(jt_name, desired_pos[joint_idx])
                reached_joint_pos[joint_idx] = psm_handle.get_joint_pos(jt_name)

            error_in_pos = np.subtract(desired_pos, reached_joint_pos)
            print("error ", error_in_pos)
            count += 1
            if count > 50:
                print("Manually being set")
                for joint_idx, jt_name in enumerate(joints_to_control):
                    psm_handle.set_joint_pos(jt_name, reached_joint_pos[joint_idx] + error_in_pos[joint_idx])
            if np.all(np.abs(error_in_pos) < error_threshold):
                break

except KeyboardInterrupt:
    _client.clean_up()
    pass
