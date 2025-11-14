### Usage

This script is self explanatory. If using ROS 1, run `roscore` in a different terminal.

``` bash
roscore
```

Then run AMBF simulator with the toy-car2 as follows:

**Either change directory to this folder, or add the full path by replacing 
<this folder> in the command below**

``` bash
cd <ambf_bin>/
./ambf_simulator -l 1 -a <this_folder>/ADF/point_cloud.yaml
```

Then run the Python script as

``` bash
cd <path_to_this_folder>
python point_cloud_example.py
```

### OLD ROS 1 INSTRUCTIONS (NOT PLANNED FOR ROS 2)
1. When AMBF simulator launches it will listen on a specific topic for point cloud data. This topic is
`/ambf/env/World/point_cloud`. You may add topics by calling the ROS Param server as such

``` python
pc_topics = rospy.get_param('/ambf/env/World/point_cloud_topics')
print 'Existing Topics AMBF is listening to for Point Cloud'
print pc_topics
pc_topics.append('/ambf/env/World/another_point_cloud')
rospy.set_param('/ambf/env/World/point_cloud_topics', pc_topics)
```

2. If you would like to increase the size of a point cloud topic, you can set its size via the ROS Param Server.
The size is an integer value

``` python
pc_sizes = rospy.get_param('/ambf/env/World/point_cloud_radii')
pc_sizes.append(10) # 10 pt size for first PC
pc_sizes.append(20) # 20 pt size for second PC
rospy.set_param('/ambf/env/World/point_cloud_radii', pc_sizes)
```

3. For any point cloud, you can set its parent by simply setting its header field. You can dynamically change this parent name as well.

```python
from sensor_msgs.msg import PointCloud
msg = PointCloud()
msg.header.frame_id = 'BODY NAME'
```