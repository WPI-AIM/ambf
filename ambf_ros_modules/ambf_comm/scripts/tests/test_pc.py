import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from rospy import Rate

rospy.init_node('test_pc')

pub = rospy.Publisher('/test_pointcloud', PointCloud, queue_size=10)
pub2 = rospy.Publisher('/test_pointcloud2', PointCloud, queue_size=10)

p1 = Point32()
p1.x = 1.0
p1.y = 1.0
p1.z = 1.0

p2 = Point32()
p2.x = 1.0
p2.y = -1.0
p2.z = 1.0

p3 = Point32()
p3.x = -1.0
p3.y = 1.0
p3.z = 1.0

p4 = Point32()
p4.x = 0.5
p4.y = 0.5
p4.z = 0.5

p5 = Point32()
p5.x = 0.5
p5.y = -0.5
p5.z = 0.5

p6 = Point32()
p6.x = -0.5
p6.y = 0.5
p6.z = 0.5

msg = PointCloud()
msg.header.frame_id = '/ambf/env/BODY Chassis'
# msg.header.frame_id = '/ambf/env/BODY WheelFL'
msg.points.append(p1)
msg.points.append(p2)
msg.points.append(p3)


msg2 = PointCloud()
msg2.header.frame_id = '/ambf/env/BODY Chassis'
# msg2.header.frame_pid = '/ambf/env/BODY WheelFL'
msg2.points.append(p4)
msg2.points.append(p5)
msg2.points.append(p6)

r = Rate(10)

while not rospy.is_shutdown():
    pub.publish(msg)
    pub2.publish(msg2)
    r.sleep()
