# Asynchronous Multi-Body Framework (AMBF)
### 1. Author: [Adnan Munawar](https://github.com/adnanmunawar) (amunawar@wpi.edu)

#### Build Status  

![ambf-1.0](https://github.com/WPI-AIM/ambf/workflows/ambf-1.0/badge.svg?branch=ambf-1.0)


### 2. [Wiki](https://github.com/WPI-AIM/ambf/wiki):
Please checkout the [Wiki](https://github.com/WPI-AIM/ambf/wiki) for indepth details about AMBF, it's components, examples and concepts. You can also checkout the video below for a brief run down of some of the features of AMBF

[![AMBF Simulator](https://img.youtube.com/vi/9REJVK1mwEU/maxresdefault.jpg)](https://www.youtube.com/watch?v=9REJVK1mwEU&t=0s)


### 3. Description:
This multi-body framework offers real-time dynamic simulation of multi-bodies (robots, free
bodies and multi-link puzzles) coupled with real-time haptic interaction via several haptic devices
(CHAI-3D) (including dVRK Manipulators and Razer Hydras). It also provides a python client for training NN and
RL Agents on real-time data with simulation in the loop. This framework is built around several
external tools that include an extended version of CHAI-3D (developed along-side AMBF), BULLET-Physics, Open-GL, GLFW, yaml-cpp, pyyaml and Eigen to name a few. Each external library has it's own license that can be found in the corresponding subfolder.

### 4. Usage:
#### 4.1 Tested Platforms:
AMBF has been tested on **Ubuntu 16.04** and **Ubuntu 18.04**. We need a few extra steps on **Ubuntu 14.04**, please create an issue if you would like to get instructions for that.

Even though it is recommended to use Linux for the full feature set of AMBF Simulator using ROS, AMBF has been tested on **MacOS Maverick** and **MacOS Mojave** without ROS support.


#### 4.2 Building:
On Linux machines, you might need to install the `libasound2-dev` package and external libraries dependencies.

```
sudo apt install libasound2-dev libgl1-mesa-dev xorg-dev
```

Boost libraires ship with Ubuntu systems, but on Mac OS, you might need to install them explicitly.

For this purpose, on **Mac OS**, if you don't have Boost

1. Install **Xcode** from App Store
2. Install **command line tools** by running in terminal
`xcode-select --install`
3. Install **Homebrew** view running this in terminal
`/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`

4. Install **boost** by running the following in the terminal
`brew install boost`


To build the framework (Linux and Mac-OS) and get various addon scripts (blender, yaml <-> urdf):
```
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
git submodule update --init --recursive
cd build
cmake ..
make
```

On Linux systems, please source the correct folder to achieve system wide availability of AMBF ROS modules.
While in the build folder, you can run:

`source ./devel/setup.bash`

You can also permanently add the install location in your .bashrc with the following command:

`echo "source ~/ambf/build/devel/setup.bash" >> ~/.bashrc`

#### 4.3 Running the Simulator:
Having succesfully completed the steps above running is Simulator is easy. Depending
on what OS you're using simply follow the commands below:

```
cd ~/ambf/bin/<os>
./ambf_simulator
```

#### 4.4 Launching Specific Multibodies:
There are two ways to launch multibodies:
1. Using the integer index of the filename specified in the launch file
2. Providing the explicit filename(s).

##### 4.4.1 Using the Integer Index in the launch file
The -l command line argument can be used to launch a specific multibody at launch using indexing. The multibodies are defined in [ambf_models/descriptions/launch.yaml](https://github.com/WPI-AIM/ambf/blob/master/ambf_models/descriptions/launch.yaml) and are commented with indices for ease of identification. As a default behaviour, launching the simulator without the `-l` command line argument loads the first multi-body defined in the `launch.yaml`. To launch a specific multi-body you can use the `-l` flag with the integer index of the multi-body as follows:

```
cd ~/ambf/bin/<os>
./ambf_simulator -l 4
```
This command will launch the 4th body defined in the `launch.yaml` file. To launch multiple multi-bodies, you can use a comma separated list (without spaces in between) of integers indices e.g.`./ambf_simulator -l 1,6,10`. This in turn would load the multi-bodies defined at 1, 6 and the 10th index in the `launch.yaml` file.

##### 4.4.2 Providing the fully qualified filename
The second option is to use the `-a` flag. For example, if one has an AMBF description file in the home directory `/users/potato/tests/robot.yaml`, this file can be launched directly as follows

```
cd ~/ambf/bin/<os>
./ambf_simulator -a /users/potato/tests/robot.yaml
```

Similarly, as it the case with the `-l` flag, multiple filenames can be launch by comma separated values. E.g.


```
cd ~/ambf/bin/<os>
./ambf_simulator -a /users/potato/tests/robot.yaml,/users/potato/tests/car.yaml
```

Lastly, the `-l` and `-a` flags can be used together to launch some files based on the index and some based on the filenames.

##### 4.4.3 Note:
The AMBF Simulator uses the yaml file located in `ambf/ambf_models/descriptions/launch.yaml` to
load robot/multi-body models, haptic device end-effectors and the world. You can see the contents
of this file by launching it in your favourite text editor. For an initial overview, the most important thing is the field `multibody configs:`. The uncommented config file(s) will be launced at startup. Multiple
config files can be launched at the same time by uncommenting them. You can play around with a few config
files to see how they work.

### 5 Interacting with the Robots/Multi-Bodies in the Simulator:
There are multiple way of interacting with the bodies in simulator. If you are using Linux, the
provided Python client offers a convenient user interface and robust API.

For full feature set of the AMBF Simulator, it is advised that you install it on Linux (Ubuntu) 16, 17 or 18. Other variants might be supported but have not yet been tested.

#### 5.1 The AMBF Python Client
This simplest way to interact with simulated bodies, robots/multi-bodies, kinematic and visual objects in the AMBF simulator is by using the high-speed Asynchronous Communication that is implemented via ROS-topics in the AMBF Framework Library. One can use either C++ or Python for this purpose. For ease of interaction we provide a convenient Python Client which can be used as follows:

##
Start the AMBF Simulator with your choice of Multi-Body config file that (set in the `multi bodies` field in the `ambf_models/launch.yaml` file).

In your python application

```python
# Import the Client from ambf_comm package
# You might have to do: pip install gym
from ambf_client import Client
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
torus_obj.set_pos(0, 0, 0) # Set the XYZ Pos in obj's World frame
torus_obj.set_rpy(1.5, 0.7, .0) # Set the Fixed RPY in World frame
time.sleep(5) # Sleep for a while to see the effect of the command before moving on

# Other methods to control the obj position include
# torus_obj.set_pose(pose_cmd) # Where pose_cmd is of type Geometry_msgs/Pose
# torus_obj.set_rot(quaterion) # Where quaternion is a list in the order of [qx, qy, qz, qw]
# Finally all the position control params can be controlled in a single method call
# torus_obj.pose_command(px, py, pz, roll, pitch, yaw, *jnt_cmds)

# We can just as easily get the pose information of the obj
cur_pos = torus_obj.get_pos() # xyx position in World frame
cur_rot = torus_obj.get_rot() # Quaternion in World frame
cur_rpy = torus_obj.get_rpy() # Fixed RPY in World frame

# Similarly you can directly control the wrench acting on the obj by
# The key difference is that it's the user's job to update the forces
# and torques in a loop otherwise the wrench in cleared after an internal
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
children_names = torus_obj.get_children_names() # Get a list of children names belonging to this obj

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

### 6 Raven and DVRK Kinematics Controller Client
See [here](/ambf_controller/README.md) for more information.

### 7 AMBF Network Setup:
In order to subscribe and publish data using AMBF over multiple machines, the following steps would need to be followed:
1. Check the connectivity between the machines (example: using ssh and ping)
2. Edit the `/etc/hosts` and add the hostnames of the machines, so that the machines can find each other over the network (example: similar to [Adding host name to /etc/hosts](https://www.howtogeek.com/howto/27350/beginner-geek-how-to-edit-your-hosts-file/))
3. Set the ROS environment variable in local machine to the host using `export ROS_MASTER_URI=http://hostIPaddress:11311` (ex: export ROS_MASTER_URI=http://112.115.256.121:11311)
4. Now you should be able to send and receive ROS messages over the two machines and control AMBF.
5. If you face any firewall issues or if you are unable to receive/publish any ROS topics over the two machines, follow the next step.
6. Open a terminal and type the command:  `sudo apt-get install gufw`
7. Next type `sudo gufw` (type the password when prompted) and ensure both the Incoming and Outgoing traffic is allowed.

### 8 Docker
In order to use the docker file, follow the instructions [here](https://docs.docker.com/install/) to install docker on your system. To run the file:  

```bash
cd ~/
git clone https://github.com/WPI-AIM/ambf.git && cd ambf
sudo service docker start
docker build --rm -f "Dockerfile" -t ambf:latest "."
docker run --rm -it  ambf:latest
cd /ambf/bin/lin-x86_64/
./ambf_simulator -g off
```

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
