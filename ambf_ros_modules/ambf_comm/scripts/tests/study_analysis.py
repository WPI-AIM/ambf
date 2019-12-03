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


# print os.listdir('.')
os.chdir('./user_study_data/')
dir_names = os.listdir('.')

studies_dict = dict()
studies_dict['1A'] = []
studies_dict['2A'] = []
studies_dict['3A'] = []
studies_dict['1B'] = []
studies_dict['2B'] = []
studies_dict['3B'] = []

for dir_name in dir_names:
    print "***************************************************"
    print 'User Name: ', dir_name
    rel_dir_name = './' + dir_name + '/' + 'Reduced/'
    filenames = os.listdir(rel_dir_name)
    print 'Rel Dir Name', rel_dir_name
    print 'File Names', filenames

    # Soft Files in order
    files_dict = dict()
    files_dict['1A'] = 0
    files_dict['2A'] = 0
    files_dict['3A'] = 0
    files_dict['1B'] = 0
    files_dict['2B'] = 0
    files_dict['3B'] = 0

    sorted_files_idx = ['1A', '2A', '3A', '1B', '2B', '3B']
    sorted_files_list = []

    for cur_file in filenames:
        split_file = cur_file.split('_')
        files_dict[split_file[2]] = cur_file

    for i in range(0, 6):
        key = sorted_files_idx[i]
        sorted_files_list.append(files_dict[key])

    print 'Sorted Files List', sorted_files_list

    for cur_file in sorted_files_list:
        print '--------------------------------'
        print "Opening Bag file: ", cur_file

        print(cur_file.split('_'))
        split_filename = cur_file.split('_')
        subject_name = split_filename[1]
        study_type = split_filename[2]
        print "Subject Name: ", subject_name, "Control Type: ", study_type

        cur_bag = rosbag.Bag(rel_dir_name + cur_file)

        study_length = cur_bag.get_end_time() - cur_bag.get_start_time()

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
        mtmr_error_traj = get_force_trajectory('/ambf/env/simulated_device_1/MTML/State', t_end=final_time)
        mtml_error_traj = get_force_trajectory('/ambf/env/simulated_device_2/MTMR/State', t_end=final_time)
        sde_r_path_len = get_force_trajectory('/ambf/env/simulated_device_1/MTML/State', t_end=final_time)
        sde_l_path_len = get_force_trajectory('/ambf/env/simulated_device_2/MTMR/State', t_end=final_time)

        num_clutches = compute_number_of_clutches('/dvrk/footpedals/clutch', t_end=final_time)
        print "MTMR Path Length: ", mtmr_path_len
        print "MTML Path Length: ", mtml_path_len
        print "Number of Clutches: ", num_clutches
        print "Study Length: ", study_length

        study_dict = dict()
        study_dict["MTMR Path Length"] = mtmr_path_len
        study_dict["MTML Path Length"] = mtml_path_len
        study_dict["Number of Clutches"] = num_clutches
        study_dict["Study Length"] = study_length

        subject_dict = dict()
        subject_dict[subject_name] = study_dict

        studies_dict[study_type].append(subject_dict)


print studies_dict
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
# plt.plot(mtml_error_traj)
# plt.plot(mtmr_error_traj)
#
# plt.show()