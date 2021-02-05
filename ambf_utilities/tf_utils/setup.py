#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  name='tf_utils',
  version='0.0.1',
  package_dir={'': 'scripts'},
  packages=['frame', 'vector', 'rotation', 'wrench', 'twist']
)

setup(**d)