import pandas as pd
import numpy as np
import csv
import math
import rosbag
import rospy
from numpy import linalg as LA


def getData(rosbag_path, ros_topic_name, pos=True):
    cur_bag = rosbag.Bag(rosbag_path)

    t_start = rospy.Time(cur_bag.get_start_time())
    t_end = rospy.Time(cur_bag.get_end_time())

    num_msgs = cur_bag.get_message_count(ros_topic_name)
    # For position
    if pos:
        data = np.zeros(shape=(num_msgs, 3))
    # For orientation (quaternion)
    else:
        data = np.zeros(shape=(num_msgs, 4))
    idx = 0

    for top, msg, t in cur_bag.read_messages(topics=[ros_topic_name],
                                             start_time=t_start,
                                             end_time=t_end):
        if pos:
            cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        else:
            cur = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w])
        data[idx, :] = cur
        idx = idx + 1

    return data


def create_state_action_pairs(data):
    steps = []
    fixed = []
    curr = data[0]
    steps.append(curr-curr)
    fixed.append(np.round(curr, 3))
    for j in range(0, len(data)-1):
        curr = round_nearest(data[j], 0.005)
        nxt = round_nearest(data[j+1], 0.005)
        interval = np.round(nxt-curr, 3)
        if abs(interval[0]) > 0 or abs(interval[1]) > 0 or abs(interval[2]) > 0:
            steps.append(interval)
            fixed.append(curr)

    return np.array(fixed), steps


def make_file(robot_states, robot_actions, csv_name):
    with open(csv_name + '.csv', mode='w') as f:
        writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for robot_state, robot_action in zip(robot_states, robot_actions):
            writer.writerow([robot_state[0], robot_state[1],  robot_state[2], robot_action[0],
                             robot_action[1], robot_action[2]])


def round_nearest(x, a):
    return np.round(np.round(x / a) * a, -int(math.floor(math.log10(a))))


def create_reward(data_values, goal_pos, file_dir):
    total_reward = 0
    list_of_reward = []
    print(data_values.shape)
    for i in range(data_values.shape[0]):
        # dist_x = np.subtract(final_state, actual_val[i, 0])
        # dist_y = np.subtract(final_state, actual_val[i, 1])
        # dist_z = np.subtract(final_state, actual_val[i, 2])
        # reward = math.sqrt(math.pow(dist_x, 2) + math.pow(dist_y, 2) + math.pow(dist_z, 2))
        cur_dist = LA.norm(np.subtract(goal_pos, data_values[i, 0:3]))
        # reward = round(1 - float(abs(cur_dist)/0.05)*0.5, 5)
        reward = -cur_dist
        if abs(cur_dist) < 0.000001:
            reward = 1

        total_reward += reward
        list_of_reward.append(reward)
    print(total_reward)
    # np.savetxt(file_dir + 'episode_returns.csv', total_reward, delimiter=',')
    # np.savetxt(file_dir + 'rewards.csv', np.array(list_of_reward), delimiter=',')
    np.save(file_dir + 'rewards', np.array(list_of_reward))
    np.save(file_dir + 'episode_returns', total_reward)


def create_obs_action_func(data_values, file_dir):
    obs = []
    actions = []
    print(data_values.shape)
    for i in range(data_values.shape[0]):
        obs.append(data_values[i, 0:3])
        actions.append(data_values[i, 3:6])
    print(obs)
    np.save(file_dir + 'observations', np.array(obs))
    np.save(file_dir + 'actions', np.array(actions))


if __name__ == "__main__":

    rosbag_file_dir = "/home/vignesh/Thesis_Suture_data/trial2/suture_data_trial2/2020-02-25_20:02:45.832953.bag"
    rostopic_name = "/dvrk/PSM2/position_cartesian_current"
    csv_file_name = "/home/vignesh/Thesis_Suture_data/trial2/user_data"
    # Read rosbag data
    data = getData(rosbag_file_dir, rostopic_name)
    # Create state action pairs
    robot_state, robot_action = create_state_action_pairs(data)
    # Write it to a csv file
    make_file(robot_state, robot_action, csv_file_name)
    # Assuming reward is reward = round(1 - float(abs(cur_dist)/0.3)*0.5, 5)
    file_dir = "/home/vignesh/Thesis_Suture_data/trial2/ambf_data/"
    traj_csv_name = "832953.csv"
    trajectory_data_values = pd.read_csv(file_dir + traj_csv_name).to_numpy()
    final_state = np.array([0.005, 0.054, -0.122])
    # create_obs_action_func(trajectory_data_values, file_dir)
    # create_reward(trajectory_data_values, final_state, file_dir)
    epi_start = np.array([True])
    for i in range(trajectory_data_values.shape[0]):
        epi_start = np.append(epi_start, False)
    np.save(file_dir + 'episode_starts', epi_start)



