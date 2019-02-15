# Asynchronous Multi-Body Framework (AMBF)
### Author: Adnan Munawar
#### Email: amunawar@wpi.edu

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
\\ TODO : How to control the robots using message publishers

