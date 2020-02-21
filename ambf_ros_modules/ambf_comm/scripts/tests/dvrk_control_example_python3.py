# Import the Client from ambf_client package
from ambf_client import Client
import time

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
input("We can see what objects the client has found. Press Enter to continue...")
# You can print the names of objects found. We should see all the links found
print(_client.get_obj_names())

# Lets get a handle to PSM and ECM, as we can see in the printed
# object names, 'ecm/baselink' and 'psm/baselink' should exist
ecm_handle = _client.get_obj_handle('ecm/baselink')
psm_handle = _client.get_obj_handle('psm/baselink')

# Similarly we can get a handle to any link lower in the hierarchy rather
# than the root link. Let's get a handle to MTMs wrist platform link
mtm_wrist_handle = _client.get_obj_handle('mtm/WristYaw')

# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)

print('\n\n----')
input("Let's Get Some Pose Info. Press Enter to continue...")
# Not we can print the pos and rotation of object in the World Frame
print('ECM Base Pos:')
print(ecm_handle.get_pos())

print(' ')
print('PSM Base Rotation as Quaternion:')
print(psm_handle.get_rot())

print(' ')
print('MTM Wrist Fixed Rotation:')
print(mtm_wrist_handle.get_rpy())

print('\n\n----')
input("Let's get Joints and Children Info. Press Enter to continue...")
# We can get the number of children and joints connected to each object as
ecm_num_joints = ecm_handle.get_num_joints() # Get the number of joints of this object
psm_children_names = psm_handle.get_children_names() # Get a list of children names belonging to this obj
print('Number of Joints in ECM:')
print(ecm_num_joints)

print(' ')
print('Name of PSM\'s children:')
print(psm_children_names)

print('\n\n----')
input("Control ECMs joint positions. Press Enter to continue...")
# Now let's control some joints
# The 1st joint, which the ECM Yaw
ecm_handle.set_joint_pos(0, 0)
# The 2nd joint, which is the ECM Pitch
ecm_handle.set_joint_pos(1, -0.2)
# The 3rd Kinematic or 7th Actual Joint, which is the Prismatic Insertion Joint
ecm_handle.set_joint_pos(6, 0.1)

print('\n\n----')
input("Mixed Pos and Effort control of PSM\'s joints. Press Enter to continue...")
# For the PSM let's control some in position and some in effort mode
# The 1st joint, which the PSM Yaw
psm_handle.set_joint_effort(0, 0.5)
# The 3rd Kinematic joint, which is the PSM Insertion Joint
psm_handle.set_joint_pos(3, -0.2)

print('\n\n----')
input("Set force on MTM's Wrist Yaw link for 5 secs. Press Enter to continue...")
# Let's directly control the forces and torques on the mtmWristYaw Link
# Notice that these are in the world frame. Another important thing to notice
# is that unlike position control, forces control requires a continuous update
# to meet a watchdog timing condition otherwise the forces are reset Null. This
# is purely for safety reasons to prevent unchecked forces in case of malfunctioning
# python client code
for i in range(0, 500):
    mtm_wrist_handle.set_force(0, 0, 10) # Set 10 N in the World Z axis
    time.sleep(0.01) # Run the loop for 10 seconds

print('\n\n----')
input("Set wrench on MTM's Wrist Yaw link for 5 secs. Press Enter to continue...")
# Similarly we can individually apply the torque
for i in range(0, 500):
    mtm_wrist_handle.set_torque(0, 3, 0) # Set 10 Nm in the World Y axis
    time.sleep(0.01) # Run the loop for 10 seconds

print('\n\n----')
input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()