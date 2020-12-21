 ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    name='dvrk_functions',
    packages=['psmFK', 'psmIK', 'utilities', 'kinematics'],
    package_dir={'': 'scripts'},
    author='Dhruv Kool Rajamani',
    author_email='dkoolrajamani@wpi.edu'
)

setup(**d)
