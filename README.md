# Asynchronous Multi-Body Framework (AMBF)

Author: [Adnan Munawar](https://github.com/adnanmunawar) (amunawa2@jh.edu)


#### Build Status:
[![ubuntu-24.04-ros-jazzy](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-24.04-jazzy.yaml/badge.svg?branch=ambf-3.0)](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-24.04-jazzy.yaml)
[![ubuntu-22.04-ros-humble](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-22.04-humble.yaml/badge.svg?branch=ambf-3.0)](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-22.04-humble.yaml)
[![ubuntu-20.04-ros-noetic](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-20.04-noetic.yaml/badge.svg?branch=ambf-3.0)](https://github.com/WPI-AIM/ambf/actions/workflows/ubuntu-20.04-noetic.yaml)

## 1. Description:
The Asynchronous Multi-Body Framework (AMBF) provides real-time dynamic simulation of robots, and soft-bodies coupled with real-time haptic interaction via several haptic devices
(CHAI-3D) (including dVRK Manipulators and Razer Hydras). It also provides a Python client for training NN and
RL Agents on real-time data with the simulation in the loop. This framework is built around several
external tools that include an extended version of CHAI-3D (developed alongside **AMBF**), BULLET-Physics, Open-GL, GLFW, yaml-cpp, pyyaml, and Eigen to name a few. Each external library has its license that can be found in the corresponding subfolder.


### 2. [Wiki](https://github.com/WPI-AIM/ambf/wiki):
Please refer to the [Wiki](https://github.com/WPI-AIM/ambf/wiki) for usages, examples, and concepts.

[![AMBF Simulator](https://img.youtube.com/vi/9REJVK1mwEU/maxresdefault.jpg)](https://www.youtube.com/watch?v=9REJVK1mwEU&t=0s)



### 3. [Discussions](https://github.com/WPI-AIM/ambf/discussions):
Please refer to the [discussions](https://github.com/WPI-AIM/ambf/discussions) questions, and suggestions.


### 4. Featured Projects:

A list of some projects that are developed on/using **AMBF**. Please click on the project title to navigate to the project webpage.

#### 4.1 [Bone Drilling Simulator](https://github.com/LCSR-SICKKIDS/volumetric_drilling):

https://user-images.githubusercontent.com/5005445/199542980-4732e80d-4274-448e-a680-435182046b20.mp4

The bone drilling simulator also provides stereoscopic view of supported Virtual Reality (VR) Head Mounted Displays (HMDs):

https://user-images.githubusercontent.com/5005445/199543694-d9a2ded3-c716-4a5e-8a5f-b74e54d55d23.mp4

#### 4.2 [Surgical Robotics Challenge](https://github.com/collaborative-robotics/surgical_robotics_challenge):

https://user-images.githubusercontent.com/5005445/199545181-894dc156-6da2-4b63-8c90-54b8f3dd23f9.mp4

#### 4.3 [Space Robotics Challenge](https://github.com/adnanmunawar/space_robotics_challenge):

https://user-images.githubusercontent.com/5005445/199545275-104b27ea-be60-4d6a-b8ca-9351deed2df0.mp4



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
