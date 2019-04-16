#!/usr/bin/env python
import rospy


class WatchDog(object):
    def __init__(self, time_out=0.1):
        self._expire_duration = rospy.Duration.from_sec(time_out)
        self._next_cmd_expected_time = rospy.Time.now()
        self._initialized = False

    def acknowledge_wd(self):
        self._initialized = True
        self._next_cmd_expected_time = rospy.Time.now() + self._expire_duration

    def is_wd_expired(self):
        if rospy.Time.now() > self._next_cmd_expected_time and self._initialized:
            return True
        else:
            return False

    def console_print(self, class_name):
        if self._initialized:
            print 'Watch Dog Expired, Resetting {} command'.format(class_name)
            self._initialized = False
