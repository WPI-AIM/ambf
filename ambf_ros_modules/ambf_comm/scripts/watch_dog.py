#!/usr/bin/env python
import rospy


class WatchDog(object):
    def __init__(self, time_out=0.1):
        self.expire_duration = rospy.Duration.from_sec(time_out)
        self.next_cmd_expected_time = rospy.Time.now()
        self.initialized = False

    def acknowledge_wd(self):
        self.initialized = True
        self.next_cmd_expected_time = rospy.Time.now() + self.expire_duration

    def is_wd_expired(self):
        if rospy.Time.now() > self.next_cmd_expected_time and self.initialized:
            return True
        else:
            return False

    def console_print(self, class_name):
        if self.initialized:
            print 'Watch Dog Expired, Resetting {} command'.format(class_name)
            self.initialized = False