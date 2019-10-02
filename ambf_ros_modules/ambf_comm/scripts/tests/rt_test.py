import rospy
from ambf_msgs.msg import WorldState, WorldCmd
import time
import matplotlib.pyplot as plt
import sys

global state, active

state = WorldState()
active = False


def world_cb(msg):
    global state, active

    active = True
    state = msg


def main():

    episode = 10.0
    if len(sys.argv) > 1:
        episode = float(sys.argv[1])

    global active, state
    rospy.init_node('my_node')
    sub = rospy.Subscriber('/ambf/env/World/State', WorldState, world_cb, queue_size=1)
    pub = rospy.Publisher('/ambf/env/World/Command', WorldCmd, queue_size=1)
    cmd = WorldCmd()
    cmd.enable_step_throttling = False
    rate = rospy.Rate(500)

    dyn_freq_series = []
    wall_time_series = []
    sim_time_series = []
    rtf_series = []
    time_series = []

    time.sleep(1)

    start_time = rospy.get_time()
    cur_time = start_time
    end_time = cur_time + episode
    print cur_time
    while not rospy.is_shutdown() and cur_time <= end_time:

        cur_time = rospy.get_time()
        if active:
            dyn_freq_series.append(state.dynamic_loop_freq)
            wall_time_series.append(state.wall_time)
            sim_time_series.append(state.sim_time)
            rtf_series.append(state.sim_time / state.wall_time)
            time_series.append(cur_time - start_time)

        pub.publish(cmd)
        rate.sleep()

    handle = plt.figure(1)
    ax1 = handle.add_subplot(211)
    ax2 = handle.add_subplot(212)

    ax1.plot(time_series, dyn_freq_series, 'g-')
    ax1.legend(['Dynamic Update Frequency'])
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Frequency (Hz)')
    ax1.grid(True)
    # ax1.draw()

    ax2.plot(time_series, rtf_series, 'r-')
    ax2.legend(['Real Time Factor (RTF)'])
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('RTF')
    ax2.set_ylim(0, 1.5)
    ax2.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
