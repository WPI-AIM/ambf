
Force Dimension devices installation instructions
-------------------------------------------------


In order to use the Force Dimension devices with CHAI3D on Linux, a little bit of configuration is required. The following instructions have been tested on Ubuntu 13.04 (64-bit), but should apply to most recent Linux distributions with minimal changes (if any).


1. allow unprivileged users access to Force Dimension devices (optional)

By default, Linux requires superuser privileges for RAW USB access, which the Force Dimension devices requires. This can be changed by adding the appropriate rule to the Linux dynamic device management (udev). The attached file '15-forcedimension.rules' contains the correct rules for all Force Dimension devices. To enable it, simply copy it (with superuser privileges) to /etc/udev/rules.d, and restart 'udev' (or reboot your machine).


________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.
