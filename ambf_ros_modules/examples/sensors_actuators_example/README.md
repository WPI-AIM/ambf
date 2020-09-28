### Usage

This script is self explanatory. Run ROSCORE in a new terminal.

``` bash
roscore
```

Then for this specific example, run AMBF simulator
with the toy-car2 as follows

``` bash
cd <ambf_bin>/
./ambf_simulator -l 1
```

Then run this python script as

``` bash
cd <path_to_this_folder>
python point_cloud_example.py
```

### Notes
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

### TODO
1. At the moment the point cloud is redner via OpenGL points. We may add spherical points in the future.
2. The point clouds are rendered as black squares, Need to add support for colored points.
