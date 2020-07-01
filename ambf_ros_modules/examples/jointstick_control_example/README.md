### README

Examples on how to use this script.

The main file is `joystick.py`. This is a generic
script that can be used to control any object in the running AMBF simulation.
The script leverages the Python Client to get the object handle and then uses
the input from the ROS Joy Node to control the Position and Rotation of the object.
There are three different `control modes`, namely, Wrench control, Pose Control and
Twist control. These three modes can be switched by a specific button on the `Joystick`.
This button can be configured in the python script.

If one is using a game controller that has less than 6 axes, then the script allows
a user to leverage a specific button to switch between position and orientation control.
Similar to the button for changing the `control mode`, a button is assigned to switch between
position to orientation control .

### Usage Example

1. We just have to load any robot in the simulation first. For this example, lets use
a robot with a non fixed base so lets load the car example as follows:

``` bash
cd ambf/bin/<os>
./ambf_simulator -l 1
```

You should see the simulation with a car in it.

2. We must run the ROS node so that it spawns the `JoyStick` driver and streams
its state data. You can find more detail here:

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

To sum things up. Make sure you have the ROS `joy` package installed. Also make
sure that your `Joystick` is connected to the computer via USB/Ethernet/Bluetooth etc.

Commonly the `Joystick` should be device `/dev/js0` in your system, however, if
it's not the default device, then use the link above to figure out which one is it. Lets say you find out that your device is `/dev/js1`.
Simply open a terminal and type:

```rosparam set joy_node/dev "/dev/input/js1" ```

Then run the `joy` node as:

```rosrun joy joy_node```


3. Now we can run the Python script in a different terminal as.

``` bash
cd joystick_control_example
python joystick.py -o Chassis
```

You should now be able control the `Wrench (F), Pose(P) and Twist(V)` of the object using the `Joystick`
axes. By default `button 0` is assigned for switching between position and orientation, and `button 1`
is assigned for switching between `F/P/V` modes.

 You can simply replace step 1 with the robot of you choice and then proceed to
 step 3 altering the argument for `-o` flag to reflect the name of object (body) that you want to control.
 You are not limited to having ONLY the robot you intend
 to control in the simulation, infact, you can have many different robots as the
 script doesn't bother with that. The script only cares about the object name that you pass
 on to it and it will control that specific object.

 ### Command Line Arguments (Flags)
 The script takes in three flags (two of them are optional).

  1. `-o (string):` The name of the object to load (For the above example baselink is the base of the PSM)
  2. `-j (string):` The name joystick topic (Default `/joy`)
  3. `-a (string):` Optional client `rosnode` name. If you are running multiple of these scripts, you should give each a different client name.
