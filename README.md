# Asynchronous Multi-Body Framework (AMBF)

Author: [Adnan Munawar](https://github.com/adnanmunawar) (amunawar@wpi.edu)


#### Build Status  

![ambf-2.0](https://github.com/WPI-AIM/ambf/workflows/ambf-2.0/badge.svg?branch=ambf-2.0)

### 1. [Discussions](https://github.com/WPI-AIM/ambf/discussions):
Please checkout the [discussions](https://github.com/WPI-AIM/ambf/discussions) tab for questions, suggestions, and connecting with the community.


### 2. [Wiki](https://github.com/WPI-AIM/ambf/wiki):
Please check out the [Wiki](https://github.com/WPI-AIM/ambf/wiki) for in-depth details about **AMBF**, its components, examples, and concepts. You can also check out the video below for a brief rundown of some of the features of AMBF.

[![AMBF Simulator](https://img.youtube.com/vi/9REJVK1mwEU/maxresdefault.jpg)](https://www.youtube.com/watch?v=9REJVK1mwEU&t=0s)


### 3. Description:
This multi-body framework offers a real-time dynamic simulation of robots, free
bodies, and multi-link puzzles coupled with real-time haptic interaction via several haptic devices
(CHAI-3D) (including dVRK Manipulators and Razer Hydras). It also provides a Python client for training NN and
RL Agents on real-time data with the simulation in the loop. This framework is built around several
external tools that include an extended version of CHAI-3D (developed alongside **AMBF**), BULLET-Physics, Open-GL, GLFW, yaml-cpp, pyyaml, and Eigen to name a few. Each external library has its license that can be found in the corresponding subfolder.

### 4. Featured Projects:

These are some example projects that are developed on/using **AMBF**. Please click on the project title to navigate to the project webpage.

#### 4.1 [Bone Drilling Simulator](https://github.com/LCSR-SICKKIDS/volumetric_drilling):

https://user-images.githubusercontent.com/5005445/199542980-4732e80d-4274-448e-a680-435182046b20.mp4

The bone drilling simulator also provides stereoscopic view of supported Virtual Reality (VR) Head Mounted Displays (HMDs):

https://user-images.githubusercontent.com/5005445/199543694-d9a2ded3-c716-4a5e-8a5f-b74e54d55d23.mp4

#### 4.2 [Surgical Robotics Challenge](https://github.com/collaborative-robotics/surgical_robotics_challenge):

https://user-images.githubusercontent.com/5005445/199545181-894dc156-6da2-4b63-8c90-54b8f3dd23f9.mp4

#### 4.3 [Space Robotics Challenge](https://github.com/adnanmunawar/space_robotics_challenge):

https://user-images.githubusercontent.com/5005445/199545275-104b27ea-be60-4d6a-b8ca-9351deed2df0.mp4



### 5. Usage:
#### 5.1 Tested Platforms:
AMBF has been tested on **Ubuntu 16.04**, **Ubuntu 18.04** and **Ubuntu 20.04**. We need a few extra steps on **Ubuntu 14.04**, please create an issue if you would like to get instructions for that.

Even though it is recommended to use Linux for the full feature set of **AMBF** using ROS, **AMBF** has been tested on **MacOS Maverick** and **MacOS Mojave** without ROS support.


#### 5.2 Building:

##### 5.2.1 Linux (Ubuntu 16.04, 18.04, 20.04):
Install the following dependencies if not present:

```
sudo apt install libasound2-dev libgl1-mesa-dev xorg-dev
```

**Optional but recommended:** Install the appropriate `ROS 1` version for your specific Linux distribution based on the instructions here http://wiki.ros.org/ROS/Installation.
Source the ROS installation by following the instructions in Section `1.5 Environment Setup` here (`http://wiki.ros.org/noetic/Installation/Ubuntu`). Note: Change the ROS version based on which ROS you have installed.

Now we can proceed to build **AMBF**:
```
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
cd build
cmake ..
make
```

**Optional but recommended (If building with ROS support):** Source the correct folder to achieve system-wide availability of **AMBF** ROS modules.

```
cd ~/ambf/build/
source ./devel/setup.bash
```

You can also permanently add the install location in your `~/.bashrc` with the following command:

```
echo "source ~/ambf/build/devel/setup.bash" >> ~/.bashrc
# Then either reload the terminal or run `. ~/.bashrc` for the changes to take effect
```
> NOTE: If you wish to use AMBF's ROS packages (e.g. ambf_client or ambf_msgs) within a separate ROS package (e.g., "your_catkin_ws"), this can be done fairly simply, you just have to take care to do the following in the correct order: 
> 1. Ensure that AMBF is sourced in your terminal. 
> 1. Ensure "your_catkin_ws" has not been built (```catkin clean``` and/or remove ```build``` and ```devel``` if it has been built). 
> 1. Build "your_catkin_ws". 
> 
> Under the hood this will do a caktin workspace overlay, which will make sure AMBF is sourced whenever you source "your_catkin_ws" and that sourcing "your_catkin_ws" does not make the AMBF packages "unsource"/get overwritten in the environment variables. 

Finally, to execute `ambf_simulator` without having to be in the `ambf/bin/lin-x86_64` directory, one can set an alias at the end of the `~/.bashrc` file.

```
# Open the ~/.bashrc file in a text editor
# At the end of the file add
alias ambf_simulator=~/ambf/bin/lin-x86_64/ambf_simulator
# Save and close the file and reload by either relaunching the terminal or typing 
. ~/.bashrc
```

##### 5.2.2 Mac OS
If you don't have Boost libraries, you will need to install them as follows

1. Install **Xcode** from App Store
2. Install **command-line tools** by running this in the terminal
`xcode-select --install`
3. Install **Homebrew** view running this in terminal
`/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`
4. Install **boost** by running the following in the terminal
`brew install boost`


To build the framework (Linux and Mac-OS):
```
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
cd build
cmake ..
make
```

#### 5.3 Running AMBF:
Having completed the steps above running **AMBF** is easy.

On **Linux OS**, open a terminal and run `roscore`.

```bash
roscore
```

Then depending on what OS you're using simply follow the commands below in a new terminal:

```
cd ~/ambf/bin/<os>
./ambf_simulator
```

#### 5.4 Launching Specific AMBF Description Format (ADF) Files:
There are two ways to launch an ADF file:
1. Using the integer index of the filename specified in the launch file
2. Providing the explicit filename(s).

##### 5.4.1 Using the Integer Index in the launch file
The -l command-line argument can be used to launch a specific ADF file at launch using indexing. The ADF files are defined in [ambf_models/descriptions/launch.yaml](https://github.com/WPI-AIM/ambf/blob/master/ambf_models/descriptions/launch.yaml) and are commented with indices for ease of identification. By default, launching the simulator without the `-l` command line argument loads the first `1` ADF file defined in the (`launch.yaml`)[./ambf_models/descriptions/launch.yaml]. To launch a specific ADF file you can use the `-l` flag with its integer index as follows:

```
cd ~/ambf/bin/<os>
./ambf_simulator -l 4
```
This command will launch the 4th ADF file defined in the [`launch.yaml`](./ambf_models/descriptions/launch.yaml) file. To launch multiple ADF files, you can use a comma-separated list (without spaces in between) of integers indices e.g.`./ambf_simulator -l 1,6,10`. This in turn would load the ADF files defined at 1, 6, and the 10th index in the `launch.yaml` file.

##### 5.4.2 Providing the fully qualified filename
The second option is to use the `-a` flag. For example, if one has an AMBF description file in the home directory `/users/potato/tests/robot.yaml`, this file can be launched directly as follows

```
cd ~/ambf/bin/<os>
./ambf_simulator -a /users/potato/tests/robot.yaml
```

Similarly, as is the case with the `-l` flag, multiple filenames can be launched by comma-separated values. E.g.


```
cd ~/ambf/bin/<os>
./ambf_simulator -a /users/potato/tests/robot.yaml,/users/potato/tests/car.yaml
```

Lastly, the `-l` and `-a` flags can be used together to launch some files based on the index and some based on the filenames.

##### 5.4.3 Note:
The entry point to **AMBF** is via the [launch](./ambf_models/descriptions/launch.yaml) file located in `ambf/ambf_models/descriptions/launch.yaml`. This is a meta-data file that contains filepaths of a world description file, an input-device file, and scene-data files (that may define a group of links, joints, sensors, actuators, cameras, lights, etc.).

### 6 Creating custom **AMBF** Description Format (ADF) Files:
Robots and scene data are defined using the ADF files. These can be created either by hand or by using the [`blender-ambf`](https://github.com/WPI-AIM/ambf_addon) addon. Please refer to its [documentation](https://github.com/WPI-AIM/ambf_addon) for loading and creating ADF files in Blender.

### 7 Interacting with the Robots/Bodies in AMBF:
There are multiple ways of interacting with the bodies in the simulator. If you are using Linux, the
provided Python client offers a convenient user interface and robust API.

For the full feature set of the **AMBF**, it is advised that you install it on Linux (Ubuntu) 16, 17, or 18. Other variants might be supported but have not yet been tested.

#### 7.1 The AMBF Python Client
The simplest way to interact with bodies in the **AMBF** is by using the high-speed Asynchronous Communication Interface that is implemented via ROS-topics in the AMBF Framework Library. One can use either C++ or Python for this purpose. A convenient Python Client is provided for easy interaction.

##
Start **AMBF** with the desired ADF file.

Then, in your Python application

```python
# Import the Client from ambf_comm package
# You might have to do: pip install gym
from ambf_client import Client
import time

# Create an instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

# You can print the names of objects found
print(_client.get_obj_names())

# Lets say from the list of printed names, we want to get the
# handle to an object named "Torus"
torus_obj = _client.get_obj_handle('Torus')

# Now you can use the torus_obj to set and get its position, rotation,
# Pose etc. If the object has joints, you can also control them
# in either position control mode or open-loop effort mode. You can even mix and
# match the joints commands
torus_obj.set_pos(0, 0, 0) # Set the XYZ Pos in obj's World frame
torus_obj.set_rpy(1.5, 0.7, .0) # Set the Fixed RPY in World frame
time.sleep(5) # Sleep for a while to see the effect of the command before moving on

# Other methods to control the obj position include
# torus_obj.set_pose(pose_cmd) # Where pose_cmd is of type Geometry_msgs/Pose
# torus_obj.set_rot(quaternion) # Where quaternion is a list in the order of [qx, qy, qz, qw]
# Finally all the position control params can be controlled in a single method call
# torus_obj.pose_command(px, py, pz, roll, pitch, yaw, *jnt_cmds)

# We can just as easily get the pose information of the obj
cur_pos = torus_obj.get_pos() # xyx position in World frame
cur_rot = torus_obj.get_rot() # Quaternion in World frame
cur_rpy = torus_obj.get_rpy() # Fixed RPY in World frame

# Similarly you can directly control the wrench acting on the obj by
# The key difference is that it's the user's job to update the forces
# and torques in a loop otherwise the wrench is cleared after an internal
# watchdog timer expires if a new command is not set. This is for safety
# reasons where a user shouldn't set a wrench and then forget about it.
for i in range(0, 5000):
    torus_obj.set_force(5, -5, 10) # Set the force in the World frame
    torus_obj.set_torque(0, 0, 0.8) # Set the torque in the World frame
    time.sleep(0.001) # Sleep for a while to see the effect of the command before moving on

# Similar to the pose_command, one can assign the force in a single method call
# torus_obj.wrench_command(fx, fy, fz, nx, ny, nz) in the World frame

# We can get the number of children and joints connected to this body as
num_joints = torus_obj.get_num_joints() # Get the number of joints of this object
children_names = torus_obj.get_children_names() # Get a list of children's names belonging to this obj

print(num_joints)
print(children_names)

# If the obj has some joints, we can control them as follows
if num_joints > 1:
    torus_obj.set_joint_pos(0, 0.5) # The joints at idx 0 to 0.5 Radian
    torus_obj.set_joint_effort(1, 5) # Set the effort of the joint at idx 1 to 5 Nm
    time.sleep(2) # Sleep for a while to see the effect of the command before moving on


# Lastly to cleanup
_client.clean_up()
```

### 8 Raven and DVRK Kinematics Controller Client
See [here](/ambf_controller/README.md) for more information.

### 9 AMBF Network Setup:
To subscribe and publish data using **AMBF** over multiple machines, the following steps would need to be followed:
1. Check the connectivity between the machines (example: using ssh and ping)
2. Edit the `/etc/hosts` and add the hostnames of the machines, so that the machines can find each other over the network (example: similar to [Adding hostname to /etc/hosts](https://www.howtogeek.com/howto/27350/beginner-geek-how-to-edit-your-hosts-file/))
3. Set the ROS environment variable in the local machine to the host using `export ROS_MASTER_URI=http://hostIPaddress:11311` (ex: export ROS_MASTER_URI=http://112.115.256.121:11311)
4. Now you should be able to send and receive ROS messages over the two machines and control **AMBF**.
5. If you face any firewall issues or if you are unable to receive/publish any ROS topics over the two machines, follow the next step.
6. Open a terminal and type the command:  `sudo apt-get install gufw`
7. Next type `sudo gufw` (type the password when prompted) and ensure both the Incoming and Outgoing traffic is allowed.

### 10 Docker with visualization

Please follow these [instructions](https://github.com/collaborative-robotics/docker-ambf).

## Citation
If this work is helpful for your research, please use the following reference for citation:
```
@INPROCEEDINGS{8968568,
author={A. {Munawar} and Y. {Wang} and R. {Gondokaryono} and G. S. {Fischer}},
booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
title={A Real-Time Dynamic Simulator and an Associated Front-End Representation Format for Simulating Complex Robots and Environments},
year={2019},
volume={},
number={},
pages={1875-1882},
keywords={},
doi={10.1109/IROS40897.2019.8968568},
ISSN={2153-0858},
month={Nov},}
```
