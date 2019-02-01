
Sixense Razor Hydra installation instructions
---------------------------------------------


In order to use Razor Hydra devices with CHAI3D on Linux, a little bit of configuration is required. The following instructions have been tested on Ubuntu 13.04 (64-bit), but should apply to most recent Linux distributions with minimal changes (if any).


1. install the libsixense.so shared library

The libsixense shared library must installed on your system. The 'linux-installation.sh' script will take care of that for you.


2. allow unprivileged users access to Razor Hydra devices (optional)

By default, Linux requires superuser privileges for RAW USB access, which the Razor Hydra requires. This can be changed by adding the appropriate rule to the Linux dynamic device management (udev). The attached file '12-razor-hydra.rules' contains the correct rule for the Razor Hydra. To enable it, simply copy it (with superuser privileges) to /etc/udev/rules.d, and restart 'udev' (or reboot your machine).


________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.
