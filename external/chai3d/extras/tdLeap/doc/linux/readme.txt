
Leap Motion installation instructions
---------------------------------------


In order to use the Leap Motion device with CHAI3D on Linux, a little bit of configuration is required. The following instructions have been tested on Ubuntu 15.10 (64-bit), but should apply to most recent Linux distributions with minimal changes (if any). The following assumes that all relevant development tools, libraries and kernel headers have been installed on your system using your favorite package manager.


1. install the Leap Motion SDK (following Leap's instructions)


2. build and install the libtdLeap.so shared library

The libtdLeap.so shared library acts as an interface betwen CHAI3D and the Leap Motion libLeap.so library, and must be built and installed on your system. The 'linux-installation.sh' script will take care of that for you. Simply run the script (with superuser privileges) in a terminal.


3. that's it! Remember to run the CHAI3D demos with the 'sudo' prefix, as the Leap Motion requires superuser privileges, e.g.:

  cd chai3d/bin
  sudo ./01-mydevice



________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.
