 ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ambf_client', 'ros_abstraction_layer'],
    scripts=[''],
    package_dir={'': 'python'}
)

setup(**d)
