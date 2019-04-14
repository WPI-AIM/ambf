from ambf_client import Client
from ambf_msgs.msg import ObjectCmd
import time
oc = ObjectCmd()

oc.joint_cmds

client = Client()
client.create_objs_from_rostopics()
client.start()
obj = client.get_obj_handle('CenterPuzzle')
obj.set_active()

for i in range(0, 2000):
    obj.set_joint_pos(1, -1.2)
    obj.set_joint_pos(0, -0.9)
    time.sleep(0.01)

