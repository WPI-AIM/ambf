import rospy
import numpy as np
import sys
from ambf_msgs.msg import ObjectState, WorldState, WorldCmd
from matplotlib import pyplot
import datetime

wCmd = WorldCmd()
wCmd.enable_step_throttling = True
wCmd.step_clock = True
wCmd.n_skip_steps = 1

g1State = ObjectState()
g2State = ObjectState()
wState = WorldState()

global g1D, g2D, enable_collection


class DataCollection():
    def __init__(self, name):
        self.data_lim = 1000
        if len(sys.argv) > 1:
            self.data_lim = int(sys.argv[1])
            print 'Taking {} data points'.format(sys.argv[1])
        self.data = np.zeros(self.data_lim)
        self.ctr = 0
        self.name = name
        self._enable = False
        self._mean = 0
        self._std_dev = 0

    def add_data(self, d):
        if self.ctr < self.data_lim and self._enable:
            self.data[self.ctr] = d
            self.ctr = self.ctr + 1

    def reset(self):
        self.ctr = 0
        self.data = np.zeros(self.data_lim)
        self.disable_collection()

    def enable_collectoin(self):
        self._enable = True

    def disable_collection(self):
        self._enable = False

    def compute_data_metrics(self):
        self._mean = np.mean(self.data)
        self._std_dev = np.std(self.data)

        print '----------------------------------'
        print 'Data Metrics for {}'.format(self.name)
        print 'Mean = {}'.format(self._mean)
        print 'Std Deviation = {}'.format(self._std_dev)
        print '----------------------------------'

    def get_mean(self):
        return self._mean

    def get_std_dev(self):
        return self._std_dev

    def is_done(self):
        if self.ctr < self.data_lim:
            return False
        else:
            return True


def g1_cb(msg):
    global g1D
    g1D.add_data(msg.userdata[0])
    pass


def g2_cb(msg):
    global g2D
    g2D.add_data(msg.userdata[0])
    pass


def w_cb(msg):
    #
    pass


g1D = DataCollection('Gripper1')
g2D = DataCollection('Gripper2')
rospy.init_node('haptic_dynamic_response_test')
g1_sub = rospy.Subscriber('/ambf/env/Gripper1/State', ObjectState, g1_cb, queue_size=1)
g2_sub = rospy.Subscriber('/ambf/env/Gripper2/State', ObjectState, g2_cb, queue_size=1)
w_sub = rospy.Subscriber('/ambf/env/World/State', WorldState, w_cb, queue_size=1)
w_pub = rospy.Publisher('/ambf/env/World/Command', WorldCmd, queue_size=1)
g1_mean = []
g1_data = []
g2_data = []
g2_mean = []
g1_std_dev = []
g2_std_dev = []
rate_list = [1000, 800, 600, 400, 200, 150, 100, 80, 60, 50, 40, 30]
for r in rate_list:
    rate = rospy.Rate(r)
    g1D.reset()
    g2D.reset()
    collection_enabled = False
    print 'Setting Rate to {} Hz and testing'.format(r)
    time_offset = rospy.Time.now().to_sec() + 4.0
    while not g1D.is_done() or not g2D.is_done():
        wCmd.step_clock = not wCmd.step_clock
        w_pub.publish(wCmd)
        rate.sleep()
        if not collection_enabled and rospy.Time.now().to_sec() > time_offset:
            collection_enabled = True
            print 'Enabling Data Collection'
            g1D.enable_collectoin()
            g2D.enable_collectoin()
    g1D.compute_data_metrics()
    g2D.compute_data_metrics()
    g1_mean.append(g1D.get_mean())
    g2_mean.append(g2D.get_mean())
    g1_std_dev.append(g1D.get_std_dev())
    g2_std_dev.append(g2D.get_std_dev())
    g1_data.append(g1D.data)
    g2_data.append(g2D.data)

g1_sub.unregister()
g2_sub.unregister()
f = pyplot.figure()
ax = pyplot.axes()
plt = pyplot
plt.boxplot(g1_data)
plt.ylabel('(cm)')
plt.xlabel('Dynamic Loop Freq (Hz)')
ax.set_xticklabels(rate_list)
# for tick in ax.xaxis.get_major_ticks():
#                 tick.label.set_fontsize(20)
#                 tick.label.set_rotation(45)
# for tick in ax.yaxis.get_major_ticks():
#                 tick.label.set_fontsize(20)
plt.tight_layout()
ax.grid(True)
file_str = 'Dynamo-Haptic Response' + ':' + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
plt.savefig('./graphs/' + file_str + '.png', bbox_inches='tight', format='png', dpi=600)
plt.show()



