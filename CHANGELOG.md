## Nov 2025
1. Added support for geometry shaders
2. Added support for soft body anchoring in the ADF file
3. Added examples for rendering point clouds as sphere using geometry shaders
4. Updated many original ROS1 examples to use RAL (ROS Abstraction Layer) instead of rospy for ROS 1 and ROS 2 support
5. Moved the ros_comm_plugin in the ros_modules folder, which is now actually compiled as a plugin

## July 8 2024
1. Can set ERP and CFM for 6 DOF springs and non spring joints
2. Added flag to indicate the ERP and CFM has been set in the ADF file
3. Updating Ghost objects pose based on its parent's inertial pose for better synchronization.

## June 26 2024
1. Removed setting forces and torques in the Ghost Object update
2. Added ROS Communication Support for Ghost Objects
3. Create Contact Sensors and implemented their ROS communication
4. Created CHANGELOG file