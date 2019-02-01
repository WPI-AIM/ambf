#!/usr/bin/env python
from ambf_msgs.msg import WorldState, WorldCmd
from watch_dog import WatchDog


class World(WatchDog):
    def __init__(self, a_name):
        super(World, self).__init__(2.0)
        self.state = WorldState()
        self.name = a_name
        self.cmd = WorldCmd()
        self.cmd.enable_step_throttling = False
        self.pub = None
        self.sub = None
        self.pub_flag = True
        self._active = False

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def enable_throttling(self, data):
        self.cmd.enable_step_throttling = data

    def set_num_step_skips(self, n):
        if n <= 0 or n > 100:
            raise ValueError
        self.cmd.n_skip_steps = n

    def ros_cb(self, data):
        self.state = data

    def update(self):
        self.cmd.step_clock = not self.cmd.step_clock
        self.pub.publish(self.cmd)
        self.acknowledge_wd()

    def clear_cmd(self):
        self.cmd.enable_step_throttling = False
        pass

    def run_publisher(self):
        if self.pub_flag:
            if self.is_wd_expired():
                self.console_print('World')
                self.clear_cmd()
            self.pub.publish(self.cmd)
