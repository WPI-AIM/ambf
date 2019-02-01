
SensAble Omni installation instructions
---------------------------------------

In the following you will find instructions for:

- Open Haptics 3.0
- Open Haptics 3.3.1 (3D Touch Devices)




Open Haptics 3.0
----------------


In order to use SensAble Omni devices with CHAI3D on Linux with OpenHaptics 3.0, a little bit of configuration is required. The following instructions have been tested on Ubuntu 13.04 (64-bit), but should apply to most recent Linux distributions with minimal changes (if any). The following assumes that all relevant development tools, libraries and kernel headers have been installed on your system using your favorite package manager.


1. install SensAble OpenHaptics 3.0 packages (according to SensAble's instructions)


2. fix the dynamic library path

  sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
  sudo ln -s /usr/lib/libPHANToMIO.so.4 /usr/lib/libPHANToMIO.so
  sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11 /usr/lib/x86_64-linux-gnu/libraw1394.so.8

and install dependencies:

  sudo apt-get install libglw1-mesa libmotif4

This will allow you to run the Phantom configuration utility (PHANToMConfiguration) once steps 3 and 4 have been carried out.


3. spoof the /dev/raw1394 device

SensAble driver uses the deprecated raw1394 FireWire module. In order to make it work on a current Linux kernel, proceed as follows.
(sources/credits: http://ubuntuforums.org/archive/index.php/t-1750274.html)

The raw1394 device that the SensAble software expects can be linked to the existing /dev/fw0 (of /dev/fw1, ...), and must be assigned the right permissions: 

  sudo ln /dev/fw0 /dev/raw1394
  sudo chmod 0777 /dev/raw1394


4. create a (dummy) kernel module called raw1394
  
First, build the raw1394.ko dummy module (raw1394.c) using the Makefile provided in the current folder (hdPhantom/doc/linux). In a terminal, type:

  cd chai3d/extras/hdPhantom/doc/linux
  make 

Then, load the raw1394.ko dummy module with the command

  sudo insmod raw1394.ko

Please note the following important remarks about raw1394.ko:

  4.1 the module must be rebuilt everytime the kernel is updated by running:
  
    cd chai3d/extras/hdPhantom/doc/linux
    make clean
    make

  4.2 the module must be reloaded after each reboot. To make it load automatically, one option is to copy raw1394.ko to a permanent folder (e.g. /var/lib/dummy1394), and add the following line to /etc/rc.local (before the 'exit 0' line):

  insmod /var/lib/dummy1394/raw1394.ko
  

5. build and install the libhdPhantom.so shared library

The libhdPhantom.so shared library acts as an interface betwen CHAI3D and the SensAble libHD.so, and must be built and installed on your system. The 'linux-installation.sh' script will take care of that for you.


6. that's it! Remember to run the CHAI3D demos with the 'sudo' prefix, as the Sensable Omni require superuser privileges, e.g.:

  cd chai3d/bin
  sudo ./01-mydevice





Open Haptics 3.3.1
------------------


In order to use Geomagic Touch devices with CHAI3D on Linux with OpenHaptics 3.3.1, a little bit of configuration is required. The following instructions have been tested on Ubuntu 15.04 (64-bit), but should apply to most recent Linux distributions with minimal changes (if any). The following assumes that all relevant development tools, libraries and kernel headers have been installed on your system using your favorite package manager.


1. install Geomagic Touch device drivers and OpenHaptics 3.3.1 packages (according to 3DSystems instructions)


2. build and install the libhdPhantom.so shared library

The libhdPhantom.so shared library acts as an interface betwen CHAI3D and the SensAble libHD.so, and must be built and installed on your system. The 'linux-installation.sh' script will take care of that for you.


3. that's it! Remember to run the CHAI3D demos with the 'sudo' prefix, as the Sensable Omni require superuser privileges, e.g.:

  cd chai3d/bin
  sudo ./01-mydevice



________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.
