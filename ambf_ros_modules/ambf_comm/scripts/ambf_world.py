#!/usr/bin/env python
from ambf_msgs.msg import WorldState, WorldCmd
from watch_dog import WatchDog


class World(WatchDog):
    def __init__(self, a_name):
        super(World, self).__init__(2.0)
        self._state = WorldState()
        self._name = a_name
        self._cmd = WorldCmd()
        self._cmd.enable_step_throttling = False
        self._pub = None
        self._sub = None
        self._pub_flag = True
        self._active = False

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def enable_throttling(self, data):
        self._cmd.enable_step_throttling = data

    def set_num_step_skips(self, n):
        if n <= 0 or n > 100:
            raise ValueError
        self._cmd.n_skip_steps = n

    def ros_cb(self, data):
        self._state = data

    def update(self):
        self._cmd.step_clock = not self._cmd.step_clock
        self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def clear_cmd(self):
        self._cmd.enable_step_throttling = False
        pass

    def set_name(self, name):
        self._name = name

    def get_name(self):
        return self._name

    def run_publisher(self):
        if self._pub_flag:
            if self.is_wd_expired():
                self.console_print('World')
                self.clear_cmd()
            self._pub.publish(self._cmd)
