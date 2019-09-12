 ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ambf_comm', 'ambf_env', 'ambf_client', 'ambf_object', 'ambf_world', 'ambf_world'],
    scripts=[''],
    package_dir={'': 'scripts'}
)

setup(**d)
