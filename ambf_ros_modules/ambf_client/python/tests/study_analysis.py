import rosbag
import os
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def compute_start_and_end_times( t_start=-1, t_end=-1):
    if t_start is -1:
        t_start = rospy.Time(cur_bag.get_start_time())

    if t_end is -1:
        t_end = rospy.Time(cur_bag.get_end_time())

    return t_start, t_end


def compute_dist(topic_name, t_start=-1, t_end=-1):
    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    dist = 0.0
    num_msgs = cur_bag.get_message_count(topic_name)
    trajectory = np.zeros(shape=(num_msgs, 3))
    first = True
    last = np.array([0, 0, 0])
    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=[topic_name], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        trajectory[idx, :] = cur
        idx = idx + 1
        if first:
            last = cur
            first = False
            dist = 0.0
        dist = dist + 1000 * np.matmul((cur - last), (cur - last))
        last = cur
        # print dist
    return dist, trajectory


def get_force_trajectory(topic_name, t_start=-1, t_end=-1):
    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_msgs = cur_bag.get_message_count(topic_name)
    force_trajectory = np.zeros(shape=(num_msgs, 3))
    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=[topic_name], start_time=t_start, end_time=t_end):
        cur = np.array([msg.userdata[0], msg.userdata[1], msg.userdata[2]])
        force_trajectory[idx, :] = cur
        idx = idx + 1

    return force_trajectory


def compute_number_of_clutches(topic_name, t_start=-1, t_end=-1):
    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_clutches = 0.0
    clutch_pressed = False
    clutch_press_times = []
    t_press = 0
    for top, msg, t in cur_bag.read_messages(topics=[topic_name], start_time=t_start, end_time=t_end):
        if msg.buttons[0] is 1:
            clutch_pressed = True
            t_press = t

        if msg.buttons[0] is 0 and clutch_pressed is True:
            clutch_pressed = False
            clutch_press_times.append(t.to_sec() - t_press.to_sec())
            num_clutches = num_clutches + 1

    return num_clutches, clutch_press_times


print os.listdir('.')
os.chdir('./user_study_data/')
print os.listdir('.')[0]

os.chdir(os.listdir('.')[3])
os.chdir('./Reduced')
files = os.listdir('.')

for cur_file in files:
    print '--------------------------------'
    print '--------------------------------'
    print "Opening Bag file: ", cur_file

    print(cur_file.split('_'))
    control_type = cur_file.split('_')
    # control_type = control_type[1] + ' ' + control_type[2]
    print "Subject Name: ", control_type[1], "Control Type: ", control_type[2]

    cur_bag = rosbag.Bag(cur_file)

    print cur_bag.get_end_time() - cur_bag.get_start_time()

    ctr = 0
    final_time = 0
    for topic, msg, time in cur_bag.read_messages(['/ambf/env/user_study_time']):
        # print 'TOPIC: ', topic
        # print 'TIME: ', time
        # print 'MESSAGE: ', msg
        # print 'COUNTER: ', ctr
        ctr = ctr + 1
        final_time = time

    mtmr_path_len, mtmr_traj = compute_dist('/dvrk/MTMR/position_cartesian_current', t_end=final_time)
    mtml_path_len, mtml_traj = compute_dist('/dvrk/MTML/position_cartesian_current', t_end=final_time)
    mtmr_force_traj = get_force_trajectory('/ambf/env/simulated_device_1/MTML/State', t_end=final_time)
    mtml_force_traj = get_force_trajectory('/ambf/env/simulated_device_2/MTMR/State', t_end=final_time)

    print "MTMR Path Length: ", mtmr_path_len
    print "MTML Path Length: ", mtml_path_len
    print "Number of Clutches: ", compute_number_of_clutches('/dvrk/footpedals/clutch', t_end=final_time)

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(mtmr_traj[:, 0], mtmr_traj[:, 1], mtmr_traj[:, 2], 'red')
# ax.plot3D(mtml_traj[:, 0], mtml_traj[:, 1], mtml_traj[:, 2], 'green')
#
# plt.figure()
# plt.plot(mtml_traj)
# plt.plot(mtmr_traj)
#
# plt.figure()
# plt.plot(mtml_force_traj)
# plt.plot(mtmr_force_traj)
#
# plt.show()
