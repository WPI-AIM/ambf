#!/usr/bin/env python
from client import ChaiClient
from gym import spaces
import numpy as np
import math
import time
from world import World
from object import Object


class Observation:
    def __init__(self):
        self.state = []
        self.reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), self.reward, self.is_done, self.info


class ChaiEnv:
    def __init__(self):
        self.obj_handle = Object
        self.world_handle = World

        self.ambf_client = ChaiClient()
        self.ambf_client.create_objs_from_rostopics()
        # self.ambf_client.print_summary()
        self.n_skip_steps = 5
        self.ambf_client.start()
        self.enable_step_throttling = True
        self.obs = Observation()
        self.action_lims = np.array([30, 30, 30, 2, 2, 2])
        self.action_space = spaces.Box(-self.action_lims, self.action_lims)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(6,))

        self.Base = self.ambf_client.get_obj_handle('Base')

        pass

    def skip_sim_steps(self, int num):
        self.n_skip_steps = num
        self.world_handle.set_num_step_skips(num)

    def set_throttling_enable(self, check):
        self.enable_step_throttling = check
        self.world_handle.enable_throttling(check)

    def make(self, a_name):
        self.obj_handle = self.ambf_client.get_obj_handle(a_name)
        self.world_handle = self.ambf_client.get_obj_handle('World')
        self.world_handle.enable_throttling(self.enable_step_throttling)
        self.world_handle.set_num_step_skips(self.n_skip_steps)
        if self.obj_handle is None or self.world_handle is None:
            raise Exception

    def reset(self):
        action = [0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0]
        return self.step(action)[0]

    def step(self, action):
        action[0:3] = np.clip(action[0:3], -self.action_lims[0:3], self.action_lims[0:3])
        action[3:6] = np.clip(action[3:6], -self.action_lims[3:6], self.action_lims[3:6])
        assert len(action) == 6
        self.obj_handle.command(action[0],
                                  action[1],
                                  action[2],
                                  action[3],
                                  action[4],
                                  action[5])
        self.world_handle.update()
        self._update_observation()
        return self.obs.cur_observation()

    def render(self, mode):
        print ' I am a {} POTATO'.format(mode)

    def _update_observation(self):
        if self.enable_step_throttling:
            step_jump = 0
            while step_jump < self.n_skip_steps:
                step_jump = self.obj_handle.sim_step_cur - self.obj_handle.sim_step_pre_update
                time.sleep(0.00001)
            if step_jump > self.n_skip_steps:
                print 'WARN: Jumped {} steps, Default skip limit {} Steps'.format(step_jump, self.n_skip_steps)

        state = self.obj_handle.get_pose()
        self.obs.state = state
        self.obs.reward = self._calculate_reward(state)
        self.obs.is_done = self._check_if_done()
        self.obs.info = self._update_info()

    def _calculate_reward(self, state):
        cur_pose = state
        cdef double pos_epsilon = 0.5
        cdef double rot_epsilon = 0.1
        cdef double reward = 0.0
        if math.fabs(cur_pose[0]) < pos_epsilon and\
            math.fabs(cur_pose[1]) < pos_epsilon and\
            0.926 - math.fabs(cur_pose[2]) < pos_epsilon:
            reward = 100
        if math.fabs(cur_pose[3]) < rot_epsilon and\
            math.fabs(cur_pose[4]) < rot_epsilon and\
            math.fabs(cur_pose[5]) < rot_epsilon:
            reward += 50
        return reward

    def _check_if_done(self):
        return False

    def _update_info(self):
        return {}
