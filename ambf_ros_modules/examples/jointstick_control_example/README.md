### README

Examples on how to use these script.

The main file in this folder is called `joystick.py`. This is a generic
script that can be used to control any object in the running AMBF simulation.
The script leverages the Python Client to get the object handle and then uses
the input from the ROS Joy Node to control the Position and Rotation of the object.
There are three different control modes, namely, Wrench control, Pose Control and
Twist control. These three modes can be switched by a specific button on the joystick.
This button can be configured in the python script.

If one is using a game controller that has less than 6 axes, then the script allows
a user to leverage a specific button to switch between position and orientation control.
Similar to the button for changing the control mode, the button for changing from
position to orientation control can also be changed.

### Usage Example

1. We just have to load any robot in the simulation first. For this example, lets
use the dVRK PSM. You can run this robot as:

```
cd ambf/bin/<os>
./ambf_simulator -l 5
```

You should see the simulation with the PSM in it.

2. Now we can run the Python file as.

```
cd object_control_example
python control_object.py -o baselink
```

A GUI with sliders will popup. You can use the sliders and the Effort / Position
 / Velocity to control each joint in F / P / V mode. There is a scale next to
 each slider to scale the command of the slider's value as each slider is bound
 to the range [-1, 1].

 You can simply replace step 1 with the robot of you choice and then proceed to
 step 2 altering the argument for `-o` flag to reflect the name of link (body) from where you
 intend to control the joints. You are not limited to having ONLY the robot you intend
 to control in the simulation, infact, you can have many different robots in the simulation, the
 script doesn't bother with that, it just cares about the object name that you pass
 on to it and it will find all of its joints lower in order from that body onwards.

 ### Command Line Arguments (Flags)
 The script takes in three flags (two of them are optional).

 1. `-o (string):` The name of the object to load (For the above example baselink is the base of the PSM)+
 4. `-a (string):` Optional client `rosnode` name. If you are running multiple of these scripts, you should give each a different client name.

 The `-c` flag launches another GUI slider sliders to control the Pose of the
 object provided by `-o` in F / P / V mode. Notice that the Cartesian level control
 is only for the object supplied by `-o`.
