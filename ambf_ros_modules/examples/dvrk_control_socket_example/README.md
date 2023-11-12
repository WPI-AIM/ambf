# dVRK Control via Socket

## 1. Purpose

Allows AMBF users to wirelessly control multibodies. This example specifically pertains to the surgical_robotics_challenge scene, but can be customized.

## 2. Usage

### 2.1 Setup

Follow the surgical_robotics_challenge [installation instructions](https://github.com/surgical-robotics-ai/surgical_robotics_challenge). Clone the surgical_robotics_challenge repo in the same directory as the ambf root folder.

Some terminology:
Let's call the absolute location of ambf root folder `<ambf>`.
Let's call the absolute location of surgical_robotics_challenge root folder `<surgical_robotics_challenge>`.
Let's call the absolute location of this folder `<dvrk_control_socket_example>`.

Please resolve the actual path of `<ambf>`, `<dvrk_control_socket_example>`, and `<surgical_robotics_challenge>` in the examples below.

### 2.2 Running:

#### First Step:

Start roscore in a new terminal

```bash
roscore
```

In another terminal, open the surgical robotics challenge scene

```bash
cd <ambf>/bin/lin-x86_64
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120
```

In another terminal

```bash
cd <dvrk_control_socket_example>
python3 dvrk_control_via_socket.py
```

This should launch the surgical robotics scene and allow for UDP packets to be sent to the 8080 port of your machine.

#### Second Step:

In a new terminal, find the ip address of your machine by running

```bash
ifconfig
```

If ip_addr represents the ip address of your machine, then direct UDP packets to ip_addr:8080

## 3. Understanding Message Format

The message should be formatted as a JSON String and should contain the following arguments

- **camera** String: Indicates if the transformation should manipulate the camera or the PSM. Accepted values are "true" and "false"
- **transformation** String: Indicates if the transformation includes translation details. Accepted values are "true" and "false"
- **psm** String: Indicates which psm in the scene should be manipulated. Accepted values are "left" and "right"
- **x** Double: Desired position of psm on the x-axis
- **y** Double: Desired position of psm on the y-axis
- **z** Double: Desired position of psm on the z-axis
- **roll** Double: Desired roll of psm or camera
- **pitch** Double: Desired pitch of psm or camera
- **yaw** Double: Desired yaw of psm or camera
- **insert** Double: Desired insertion value of camera (only used for camera movement)

An example packet message (unencoded) with every argument is

```
'{"x": 1.000, "y": 1.000, "z": 1.000, "roll": 1.000, "pitch": 1.000, "yaw": 1.000, "end_effector": 0.5, "camera": "false", "transformation": "true", "psm": "right", "insert": 0.05}'
```

If the single quote solution does not work, then use `\"` to represent a quotation mark in JSON string

```
"{\"x\": 1.000, \"y\": 1.000, \"z\": 1.000, \"roll\": 1.000, \"pitch\": 1.000, \"yaw\": 1.000, \"end_effector\": 0.5, \"camera\": \"false\", \"transformation\": \"true\", \"psm\": \"right\", \"insert\": 0.05}"
```

However, only a few arguments are necessary for each use case. Collect and send only what is necessary for efficiency purposes.

### 3.1 Transformation

Example JSON String for Transformation

```
'{"x": 1.000, "y": 1.000, "z": 1.000, "roll": 1.000, "pitch": 1.000, "yaw": 1.000, "end_effector": 0.5, "camera": "false", "transformation": "true", "psm": "right"}'
```

### 3.2 Rotation + End Effector

Example JSON String for Rotation

```
'{"roll": 1.000, "pitch": 1.000, "yaw": 1.000, "end_effector": 0.5, "camera": "false", "transformation": "false", "psm": "right"}'
```

### 3.3 Camera

```
'{"roll": 1.000, "pitch": 1.000, "yaw": 1.000, "camera": "true"}'
```
