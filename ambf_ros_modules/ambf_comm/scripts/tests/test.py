from ambf_comm import ChaiClient
import time
import math

client = ChaiClient()
client.create_objs_from_rostopics()
client.start()
obj = client.get_obj_handle('r_gripper_palm_link')
obj.set_active()

for i in range(1, 100000):
    obj.pose_command(1 * math.sin(i/200.0), 1 * math.cos(i/200.0), 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.001)


