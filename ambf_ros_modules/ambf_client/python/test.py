from ambf_client import Client
import time

c = Client()
c.connect()
time.sleep(1.0)
# print c.get_obj_names()
time.sleep(1.0)
b = c.get_obj_handle('baselink')
print b.get_name()