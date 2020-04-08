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
error_threshold = 0.5

desired_pos = np.array([[ 0.5  , -0.33,  0.149, 0    ,  0    , 0.,  0.],
                        [-1.605, 0.026,  0.1  , 1.569, -1.572, 0., -0.]])

joint_thresh = joint_pos

counter = 0
joints_to_control = np.array(['baselink-yawlink',
                              'yawlink-pitchbacklink',
                              'pitchendlink-maininsertionlink',
                              'maininsertionlink-toolrolllink',
                              'toolrolllink-toolpitchlink',
                              'toolpitchlink-toolgripper1link',
                              'toolpitchlink-toolgripper2link'])
try:
  i = 0
  while True:

    for joint_idx, jt_name in enumerate(joints_to_control):
      psm_handle.set_joint_pos(jt_name, desired_pos[int(i), joint_idx])
      joint_pos[joint_idx] = round(psm_handle.get_joint_pos(jt_name), 3)
    
    joint_thresh = np.abs(np.subtract(desired_pos[int(i)], joint_pos))
    
    if np.all(joint_thresh < error_threshold):
      break
      
      # if counter == 20:
      #   # i = not i
      #   counter = 0
    #
    # # Debug
    # print("desired: ", desired_pos[int(i)])
    # print("current: ", joint_pos)
    # print("error  : ", joint_thresh)
    # print("\n")
        

except KeyboardInterrupt:
  _client.clean_up()
  pass