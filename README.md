# Asynchronous Multi-Body Framework (AMBF)
### Author: Adnan Munawar
#### Email: amunawar@wpi.edu


# Wiki:
Please checkout the Wiki for examples on how to interact with bodies and additional deatails of subcomponents

## Description:
This multi-body framework offers real-time dynamic simulation of multi-bodies (robots, free
bodies and multi-link puzzles) and real-time haptic interaction with multiple haptic devices
(CHAI-3D) (including dVRK Manipulators and Razer Hydras). It also provides a python client for training NN and
RL Agents on real-time data from running simulation. This framework is built around several
external tools that include an extended version of CHAI-3D (developed along-side AMBF), BULLET-Physics, Open-GL, GLFW and yaml-cpp, pyyaml and Eigen to
name a few.

## Usage:
### Building:
To build the framework (Linux and Mac-OS):
```
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
cd build
cmake ..
make
```

If you are using Linux, then after succesful build, source the devel to add the ros modules
is the workspace using

`source ./devel/setup.bash`

You can also set the source file in your .bashrc with the following command:

`echo "source ~/ambf/build/devel/setup.bash" >> ~/.bashrc`

### Running the Simulator:
After successful build, running the simulator is easy. Follow the steps below. Depending
on what OS you're using.

```
cd ~/ambf/bin/<os>
./ambf_simulator
```

### Note:
The AMBF Simulator uses the yaml file located in `ambf/ambf_models/descriptions/launch.yaml` to
load all the runtime models, haptic device end-effectors and the world. You can see the contents
of this file by opening it in the text editor of your choice. The most important thing to see is
the field `multibody configs:`. The uncommented config file will be run at runtime and multiple
config files can be run at the same time by uncommenting them. You can play around with a few config
files to see how they work. 

### Interacting with the Robots/Multi-Bodies in the Simulator:
There are multiple way of interacting with the bodies in simulator. If you are using Linux, the best 
and most convinient way is to use ros messages.

## East to Use Python Client
For full feature set of the AMBF Simulator, it is advised that you install it on Linux (Ubuntu) 16,17 or 18. Other variants might be supported but have not yet been tested.

### The AMBF Python Client
This simplest way to interact with simulated bodies, robots/multi-bodies, kinematic and visual objects in the AMBF simulator is by using the high-speed Asynchronous Communication that is implemented via ROS-topics in the AMBF Framework Library. One can use either using C++ or Python for creating applications for this purpose. For ease of interaction we provide a convenient Python Client which can be used as follows:

## 
Start the AMBF Simulator with your choice of Multi-Body config file that can be directly set in the `ambf_models/launch.yaml` file.

In your python application

```python
# Import the Client from ambf_comm package
from ambf_comm import Client
import time

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication 
_client.connect()

# You can print the names of objects found
print(_client.get_obj_names())

# Lets say from the list of printed names, we want to get the 
# handle to an object names "Torus"
torus_obj = _client.get_obj_handle('Torus')

# Now you can use the torus_obj to set and get its position, rotation,
# Pose etc. If the object has joints, you can also control them
# in either position control mode or open loop effort mode. You can even mix and
# match the joints commands 
torus_obj.set_pos(0, 0, 0) # Set the XYZ Pos in obj's parent frame
torus_obj.set_rpy(1.5, 0.7, .0) # Set the Fixed RPY in parent frame
time.sleep(5) # Sleep for a while to see the effect of the command before moving on

# Other methods to control the obj position include
# torus_obj.set_pose(pose_cmd) # Where pose_cmd is of type Geometry_msgs/Pose
# torus_obj.set_rot(quaterion) # Where quaternion is a list in the order of [qx, qy, qz, qw]
# Finally all the position control params can be controlled in a single method call
# torus_obj.pose_command(px, py, pz, roll, pitch, yaw, *jnt_cmds)

# We can just as easily get the pose information of the obj
cur_pos = torus_obj.get_pos() # xyx position in parent frame
cur_rot = torus_obj.get_rot() # Quaternion in parent frame
cur_rpy = torus_obj.get_rpy() # Fixed RPY in parent frame

# Similarly you can directly control the wrench acting on the obj by
# The key difference is that it's the user's job to update the forces
# and torques in a loop otherwise the wrench in cleared after an internal
# watchdog timer expires if a new command is not set. This is for safety
# reasons where a user shouldn't set a wrench and then forget about it.
for i in range(0, 5000):
    torus_obj.set_force(5, -5, 10) # Set the force in the parent frame
    torus_obj.set_torque(0, 0, 0.8) # Set the torque in the parent frame
    time.sleep(0.001) # Sleep for a while to see the effect of the command before moving on

# Similar to the pose_command, one can assign the force in a single method call
# torus_obj.wrench_command(fx, fy, fz, nx, ny, nz) in the parent frame

# We can get the number of children and joints connected to this body as
num_joints = torsus_obj.get_num_joints() # Get the number of joints of this object
children_names = tosrus_obj.get_chilren_names # Get a list of children names belonging to this obj

print(num_joints)
print(children_names)

# If the obj has some joints, we can control them as follows
if num_joints > 1:
    torus_obj.set_joint_pos(0, 0.5) # The the joints at idx 0 to 0.5 Radian
    torus_obj.set_joint_effort(1, 5) # Set the effort of joint at idx 1 to 5 Nm
    time.sleep(2) # Sleep for a while to see the effect of the command before moving on


# Lastly to cleanup
_client.clean_up()
```

