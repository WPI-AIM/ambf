from tkinter import *
import rosbag
from datetime import datetime
import subprocess
import shlex

import rospy
from std_msgs.msg import Time
import time

import threading


class UserStudy:
    def __init__(self):

        rospy.init_node('user_study_data')
        self._time_pub = rospy.Publisher('/ambf/env/user_study_time', Time, queue_size=1)
        self._time_msg = 0
        self._start_time = 0
        self._active = False
        self._time_pub_thread = 0
        self._my_bag = 0

        self._topic_names = ["/ambf/image_data/camera1/compressed",
                             "/ambf/env/World/State",
                             "/ambf/env/user_study/Base/State",
                             "/ambf/env/user_study/PuzzleRed/State",
                             "/ambf/env/user_study/PuzzleYellow/State",
                             "/ambf/env/camera1/State",
                             "/ambf/env/camera2/State",
                             "/ambf/env/simulated_device_1/MTML/State",
                             "/ambf/env/simulated_device_1/MTMR/State",
                             "/ambf/env/physical_device_1/MTML/State",
                             "/ambf/env/physical_device_2/MTMR/State",
                             "/ambf/env/physical_device_3/Falcon/State",
                             "/ambf/env/physical_device_4/Falcon/State",
                             "/dvrk/MTML/position_cartesian_current",
                             "/dvrk/MTMR/position_cartesian_current",
                             "/dvrk/footpedals/clutch",
                             "/dvrk/footpedals/coag",
                             "/dvrk/MTML/set_wrench_body",
                             "/dvrk/MTMR/set_wrench_body",
                             "/ambf/env/user_study_time"]

        self._topic_names_str = ""
        self._rosbag_filepath = 0
        self._rosbag_process = 0

        for name in self._topic_names:
            self._topic_names_str = self._topic_names_str + ' ' + name

    def call(self):
        if self._rosbag_filepath is 0:
            self._active = True
            self._start_time = rospy.Time.now()
            self._time_pub_thread = threading.Thread(target=self.time_pub_thread_func)
            self._time_pub_thread.start()
            print("Start Recording ROS Bag")
            date_time_str = str(datetime.now()).replace(' ', '_')
            self._rosbag_filepath = './user_study_data/' + e1.get() + '_' + date_time_str
            command = "rosbag record -O" + ' ' + self._rosbag_filepath + self._topic_names_str
            print "Running Command", command
            command = shlex.split(command)
            self._rosbag_process = subprocess.Popen(command)
        else:
            print "Already recording a ROSBAG file, please save that first before starting a new record"

    def save(self):

        if self._rosbag_filepath is not 0:
            self._rosbag_process.send_signal(subprocess.signal.SIGINT)
            print("Saving to:", self._rosbag_filepath)

            self._rosbag_filepath = 0
            self._active = False
        else:
            print("You should start recording first before trying to save")

    def time_pub_thread_func(self):

        while self._active:
            self._time_msg = rospy.Time.now() - self._start_time
            self._time_pub.publish(self._time_msg)
            time.sleep(0.05)




study = UserStudy()

master = Tk()
master.title("AMBF USER STUDY 1")
master.geometry('500x500')
Label(master, text='Human Subject Name').grid(row=0)

e1 = Entry(master)
e1.grid(row=0, column=1)

button_start = Button(master, text="Start Record", bg="green", fg="white", height=8, width=20, command=study.call)
button_stop = Button(master, text="Stop Record (SAVE)", bg="red", fg="white", height=8, width=20, command=study.save)
button_destroy = Button(master, text="Close App", bg="black", fg="white", height=8, width=20, command=master.destroy)

button_start.grid(row=20, column=1)
button_stop.grid(row=40, column=1)
button_destroy.grid(row=60, column=1)

master.mainloop()