Header header
int8 TYPE_FORCE=0
int8 TYPE_POSITION=1
int8 TYPE_VELOCITY=2
int8 cartesian_cmd_type # The type of cartesian cmd i.e. F/P/V
geometry_msgs/Pose pose
geometry_msgs/Wrench wrench
geometry_msgs/Twist twist
int8[] joint_cmds_types # For each joint, the type of cmd i.e. F/P/V
float32[] joint_cmds
bool publish_children_names
bool publish_joint_names
bool publish_joint_positions
