from ambf_client import Client
from ambf_msgs.msg import ObjectCmd
import time
import math

obj_name = 'CenterPuzzle'

client = Client()
client.connect()
print client.get_obj_names()
obj = client.get_obj_handle(obj_name)

if obj:
    num_jnts = obj.get_num_joints()
    for i in range(0, 2000):
        obj.pose_command( 0.5 * math.sin(i/40.0), 0.5 * math.cos(i/40.0), 0, 0, 0, 0)
        if num_jnts > 0:
            obj.set_joint_pos(0, 0.5*math.sin(i/20.0))
        time.sleep(0.01)

else:
    print obj_name, 'not found'

