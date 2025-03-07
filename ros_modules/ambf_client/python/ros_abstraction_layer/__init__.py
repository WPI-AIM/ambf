#  Author(s):  Anton Deguet (source: https://github.com/collaborative-robotics/crtk_python_client/tree/devel/src/crtk)
#  Minor modifications by Adnan Munawar
#  Created on: 2016-05
#
# Copyright (c) 2016-2024 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

__all__ = []

# ros abstraction layer
import os
try:
    __ros_version_string = os.environ['ROS_VERSION']
except:
    __ros_version_string ='2'
    print('environment variable ROS_VERSION is not set, did you source your setup.bash?')

if __ros_version_string == '1':
    __all__.append('ros_ral1')
    from .ral_ros1 import ral
elif __ros_version_string == '2':
    __all__.append('ros_ral2')
    from .ral_ros2 import ral
else:
    print('environment variable ROS_VERSION must be either 1 or 2, did you source your setup.bash?')
