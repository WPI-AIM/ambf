# Description
This package is a wrapper for easy integration of dVRK manipulators without having to remake 
the topics and publishers from scratch. Functions are overloaded to provide seamless integration 
from many different data types. The library is designed to be used with non-ROS programs as well.

## Author
Adnan Munawar: amunawar@wpi.edu

## Dependencies
Standalone, however, you do need a working **dvrk-ros** for anything useful from this package

## Usage

### Adding library to CMakeLists File
    find_package(catkin REQUIRED COMPONENTS dvrk_arm)
    include_directories(${catkin_INCLUDE_DIRS})
    ...
### Origin and Tip Tranfroms
Keeping in mind the intended use of the library for different applications and UI devices, we can 
set the **Origin Frame**, such that all the **End Effector Transforms** (+ Position and Orientaiton, Forces and Moments)
using the numerous overloaded functions are w.r.t the **Origin Frame**. 

Not only that, one might want the self defined directional coordinates of the End Effector (i.e. where should the x,y and z unit vectors of the tip be pointing). For this purpose, a **Tip Frame** can be affixed on the **End Effector Frame** which will case the **Tip Frame** to be reported w.r.t the **Origin Frame**.

#### Note:
Keep in mind that the **Origin Frame is set w.r.t to the Default Origin used by CISST-SAW and Tip Frame is set w.r.t the End Effector Frame used by CISST-SAW for the specific manipulator**

When setting the positions or forces on the dvrk Manipulators, the frames are already handled so you just specify
the values w.r.t to the frames you set.

### Conversion Function
The class **DVRK_Bridge** handles all the ros-communication.**DVRK_Bridge** currently accepts Pose, Joint and Wrench member-function pointers that exposes data retrieved from the manipulator to DVRK_Arm class.
``` c++
//Function Signature (const geometry_msgs::PoseStamed&)
poseConversion.assign_conversion_fcn(&DVRK_Arm::pose_fcn_cb, this);
//Function Signature (const sensor_msgs::JointState&)
jointConversion.assign_conversion_fcn(&DVRK_Arm::joint_state_fcn_cb, this);
//Function Signature (const geometry_msgs::WrenchStamed&)
wrenchConversion.assign_conversion_fcn(&DVRK_Arm::wrench_fcn_cb, this);
 ```   
The class **Conversion** is templated to allow more data-types to be incorporated with ease.
    
#### IMPORTANT
While setting just the position of **Origin Tranfrom** or **Tip Transform**, the orientation is not altered, it remains whatever it was set before, or if it wasn't set before, it remains identity matrix. Likewise, when the only the orientation is set, the position remains un-altered.

### Using the library (Example)
```c++
DVRK_Arm arm("MTMR");
sleep(1);
// The set_mode method was previously used to set the desired robot mode. It's deprecated now
arm.set_mode(arm._m_effort_mode);
sleep(1);
arm.set_force(1,0,0);
double x,y,z;
arm.get_position(x,y,z);
arm.set_mode(arm._m_cart_pos_mode);
sleep(1);
arm.move_cp_pos(-0.5,-0.25,-0.3);   
//Check if arm is available
bool check = arm._is_available();
 ```   
### Setting Origin and Tip Frames
```c++
DVRK_Arm arm("MTML");
// Lets say we want to know what the current Transform between tip and origin is
tf::Transform ee_tran_cur;
arm.measured_cp(ee_tran_cur);
// Now, we might want the origin trans to be placed at the tip, so all position and angular
// offsets are zero. Just take the cur_trans and set it as the origin trans;
arm.set_origin_frame(ee_tran_cur);
// Now, lets say, we want the EE trans to be orientated differently.
tf::Quaternion tip_quat;
tip_quat.setRPY(0, M_PI/2, 0);
arm.affix_tip_frame_rot(tip_quat);
// We can also move the tip frame anywhere we want.
arm.affix_tip_frame_pos(-1,0,2);
```
