### README

Examples on how to use these script.

The main file in this folder is called `control_object.py`. This is a generic
script that can be used to control any object (both Joint level and Cartesian
level) in the running AMBF simulation. The script leverages the Python Client
to get the object handle and all of its joints and then spawns a GUI (using
  Python TKinter) to let you control the object.

### Usage Example

1. We just have to load any robot in the simulation first. For this example, lets
use the dVRK PSM. You can run this robot as:

``` bash
cd ambf/bin/<os>
./ambf_simulator -l 5
```

You should see the simulation with the PSM in it.

2. Now we can run the Python file as.

``` bash
cd object_control_example
python control_object.py -o baselink
```

A GUI with sliders will popup. You can use the sliders and the `Effort(F) / Position(P)
 / Velocity(V)` to control each joint in `F / P / V` mode. There is a scale next to
 each slider to scale the command of the slider's value as each slider is bound
 to the range [-1, 1].

 This is an example for what you should see.

<div style="text-align:center"><img src="Images/psm_control.png" title="PSM CONTROL VIA GUI" width="60%" ></div>


 You can simply replace step 1 with the robot of you choice and then proceed to
 step 2 altering the argument for `-o` flag to reflect the name of link (body) from where you
 intend to control the joints.  You are not limited to having ONLY the robot you intend
  to control in the simulation, infact, you can have many different robots as the
  script doesn't bother with that. The script only cares about the object name that you pass
  on to it and it will find all of the joints lower in order to control.

 ### Command Line Arguments (Flags)
 The script takes in three flags (two of them are optional).

1. `-o (string):` The name of the object to load (For the above example baselink is the base of the PSM)
2. `-j (bool):` Enable Joint Control GUI (Default: `True`)
3. `-c (bool):` Enable Cartesian Control GUI (Default: `True`)
4. `-a (string):` Optional client `rosnode` name. If you are running multiple of these scripts, you should give each a different client name.
5. `--ixyz (float[3])` Optional initial XYZ Sliders for Cartesian control GUI (Default `[0.0, 0.0, 0.0]`)
6. `--irpy (float[3])` Optional initial RPY Sliders for Cartesian control GUI (Default `[0.0, 0.0, 0.0]`)
7. `--rxyz (float)` Optional range of XYZ Sliders for Cartesian control GUI (Default `1.0`)
8. `--rrpy (float)` Optional range of RPY Sliders for Cartesian control GUI (Default `3.14` or `PI`)
9. `--res (float)` Optional resolution for each slider. (Default `0.001`)

 The `-c` flag launches another GUI slider sliders to control the Pose of the
 object provided by `-o` in `F / P / V` mode. Notice that the Cartesian level control
 is only for the object supplied by `-o`.
