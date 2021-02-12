# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import numpy as np

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
raw_input("We can see what objects the client has found. Press Enter to continue...")
# You can print the names of objects found. We should see all the links found
print(_client.get_obj_names())

#Let's label the baselinks for the left and right arm of Raven2
l_handle =  _client.get_obj_handle('raven_2/base_link_L')
r_handle = _client.get_obj_handle('raven_2/base_link_R')

'''
#Next, let's label the wrists for the left and right arm of the Raven2
l_wrist_handle = _client.get_obj_handle('/ambf/env/raven_2/wrist_L')
r_wrist_handle = _client.get_obj_handle('/ambf/env/raven_2/wrist_R')

#Each link for each arm...
link_0 = _client.get_obj_handle('/ambf/env/raven_2/0_link')

l_link_1 = _client.get_obj_handle('/ambf/env/raven_2/link1_L')
l_link_2 = _client.get_obj_handle('/ambf/env/raven_2/link2_L')
l_link_3 = _client.get_obj_handle('/ambf/env/raven_2/link3_L')

r_link_1 = _client.get_obj_handle('/ambf/env/raven_2/link1_R')
r_link_2 = _client.get_obj_handle('/ambf/env/raven_2/link2_R')
r_link_3 = _client.get_obj_handle('/ambf/env/raven_2/link3_R')

#Each grasper...
l_grasper_1 = _client.get_obj_handle('/ambf/env/raven_2/grasper1_L')
l_grasper_2 = _client.get_obj_handle('/ambf/env/raven_2/grasper2_L')

r_grasper_1 = _client.get_obj_handle('/ambf/env/raven_2/grasper1_R')
r_grasper_2 = _client.get_obj_handle('/ambf/env/raven_2/grasper2_R')

#Instrument Shaft...
l_instr_shaft = _client.get_obj_handle('/ambf/env/raven_2/instrument_shaft_L')
r_instr_shaft = _client.get_obj_handle('/ambf/env/raven_2/instrument_shaft_R')
'''
# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)

print('\n\n----')
raw_input("Let's Get Some Pose Info. Press Enter to continue...")
# Not we can print the pos and rotation of object in the World Frame
print('Left Base Pos:')
print(l_handle.get_pos())
print(' ')
print('Right Base Pos:')
print(r_handle.get_pos())

print('\n\n----')
raw_input("Let's get Joints and Children Info. Press Enter to continue...")
# We can get the number of children and joints connected to each object as

l_num_joints = l_handle.get_num_joints() # Get the number of joints of this object
r_num_joints = l_handle.get_num_joints()
l_children = l_handle.get_children_names()
r_children = r_handle.get_children_names()

print('Left Num Joints: ')
print(l_num_joints)

print(' ')
print('Right Num Joints: ')
print(r_num_joints)

print(' ')
print('Left Children: ')
print(l_children)

print(' ')
print('Right Children: ')
print(r_children)

'''
Left Children:
	link1_L --> 0
	link2_L --> 1
	link3_L --> 2
	instrument_shaft_L --> 3
	wrist_L --> 4
	grasper1_L --> 5
	grasper2_L --> 6
Right Children:
	link1_R --> 0
	link2_R --> 1
	link3_R --> 2
	instrument_shaft_R --> 3
	wrist_R --> 4
	grasper1_R --> 5
	grasper2_R --> 6	
'''
print('\n\n----')
raw_input("Control Left joint positions. Press Enter to continue...")
'''
l_handle.set_joint_pos(0,0)
l_handle.set_joint_pos(1,0)
l_handle.set_joint_pos(2,0)
l_handle.set_joint_pos(4,0)
l_handle.set_joint_pos(5,0)
l_handle.set_joint_pos(6,0)
time.sleep(0.2)
print('\n\n----')
raw_input("Control Right joint positions. Press Enter to continue...")
r_handle.set_joint_pos(0,-0.1)
r_handle.set_joint_pos(1,0.2)
r_handle.set_joint_pos(2,0.01)
r_handle.set_joint_pos(4,-0.02)
r_handle.set_joint_pos(5,0.0006)
r_handle.set_joint_pos(6,-0.003)

l_handle.set_joint_pos(0,-0.01)
print("-0.01")
time.sleep(5)
l_handle.set_joint_pos(0,0.1)
print("0.1")
time.sleep(5)
l_handle.set_joint_pos(0,-0.01)
print(-0.01)

raw_input("Set force on Left Instrument Shaft for 5 secs. Press Enter to continue...")
'''
'''
l_instr_shaft = _client.get_obj_handle('/ambf/env/raven_2/instrument_shaft_L')
for i in range(0,500):
	l_instr_shaft.set_force(0,0,10)
	time.sleep(0.01)
'''
"""
===== Go Home Function =====
returns raven2 to starting postion
"""
start_jp = np.zeros((2,7)) #indexed at 0
delta_jp = np.zeros((2,7)) 

#starting positions --> need to be finalized
home_joints = [math.pi*(3/4), math.pi*(3/5), -0.09, math.pi*(3/4), 0, math.pi*(1/6), math.pi*(1/6)]
def go_home(first_entry, arm, count):
	'''
	first entry --> bool
	arm --> bool (true for left, false for right)
	count --> int
	'''
	if(arm): #decides which arm...
			state = l_handle
	else:
			state = r_handle
	#if first time calling go home
	if(first_entry):
		for i in range(state.get_num_joints()):
			start_jp[arm][i] = state.get_joint_pos(i)
			delta_jp[arm][i] = home_joints[i] - state.get_joint_pos(i)
	#gradualizes movement from a to b
	scale = min(float(1.0*count/1000), float(1.0))
	#array containing distance to go to start point
	diff_jp = np.zeros((1,7))

	#sets position for each joint
	for i in range(state.get_num_joints()):
		state.set_joint_pos(i, scale * delta_jp[arm][i] + start_jp[arm][i])
		diff_jp[0][i] = abs(home_joints[i] - state.get_joint_pos(i))
	#in progress, indicates when arm is honed
	max_value = max(diff_jp[0])
	if(max_value < 0.1): homed = True 
	else: homed = False 
	
	return homed

#displays start position for first joint
print(l_handle.get_joint_pos(0))
print(r_handle.get_joint_pos(0))

#updates joint position
for i in range(1000):
	if not i:
		homed_l = go_home(True, True, i)
		homed_r = go_home(True, False, i)
	else:
		homed_l = go_home(False, True, i)
		homed_r =go_home(False, False, i)
	time.sleep(0.01)
print(homed_l, homed_r)

#clean up

print('\n\n----')
raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

