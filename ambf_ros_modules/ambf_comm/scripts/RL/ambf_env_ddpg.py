#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019, AMBF
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
from ambf_client import Client
from gym import spaces
import numpy as np
import math
import time
import gym
from gym.utils import seeding
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
from psmIK import compute_IK, test_ik
from psmFK import compute_FK
# from PyKDL import Vector, Rotation, Frame, dot
from transformations import euler_from_matrix



class Observation:
    def __init__(self):
        self.state = [0]*13
        self.dist = 0
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), self.reward, self.is_done, self.info


class AmbfEnv(gym.Env):
    def __init__(self):
        super(AmbfEnv, self).__init__()
        self.obj_handle = Object
        self.world_handle = World
        self.ambf_client = Client()
        self.ambf_client.connect()
        time.sleep(5)
        self.ambf_client.create_objs_from_rostopics()
        # time.sleep(5)
        self.n_skip_steps = 5
        self.enable_step_throttling = True
        self.action = []
        self.obs = Observation()
        self.initial_pos = self.obs.cur_observation()[0]
        self.previous_cartesian_pos = np.zeros((1, 3))
        self.previous_joint_pos = np.zeros(6)
        # Action Limit values: Z-> +-0.5 || X, Y-> +-0.05
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        self.n_actions = 6
        self.action_lims_low = np.array([-0.05, -0.05, -0.05, -0.05, -0.05, -0.05])
        self.action_lims_high = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.n_actions,), dtype="float32")
        self.observation_space = spaces.Box(-100, 100, shape=(13,))
        # self.states_lims_low = np.array([-1.605, -0.93556, -0.002444, -3.0456, -3.0414, -3.0481, -3.0498])
        # self.states_lims_high = np.array([1.5994, 0.94249, 0.24001, 3.0485, 3.0528, 3.0376, 3.0399])
        # self.observation_space = spaces.Box(self.states_lims_low, self.states_lims_high, shape=(13,), dtype=np.float32)
        self.joints_to_control = np.array(['baselink-yawlink', 'yawlink-pitchbacklink',
                                           'pitchendlink-maininsertionlink', 'maininsertionlink-toolrolllink',
                                           'toolrolllink-toolpitchlink', 'toolpitchlink-toolgripper2link'])
        # self.base_handle = self.ambf_client.get_obj_handle('PegBase')
        self.goal_state = [0.0, 0.0, -0.15, -1.57, 0, -3.138]
        self.prev_sim_step = 0
        self.count_for_print = 0
        pass
    #  0.0457060,  -0.01173, 0.01810

    def skip_sim_steps(self, num):
        self.n_skip_steps = num
        self.world_handle.set_num_step_skips(num)

    def set_throttling_enable(self, check):
        self.enable_step_throttling = check
        self.world_handle.enable_throttling(check)

    def make(self, a_name):
        self.obj_handle = self.ambf_client.get_obj_handle(a_name)
        self.world_handle = self.ambf_client.get_world_handle()
        self.world_handle.enable_throttling(self.enable_step_throttling)
        self.world_handle.set_num_step_skips(self.n_skip_steps)
        time.sleep(10)
        if self.obj_handle is None or self.world_handle is None:
            raise Exception

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        observation, _, _, _ = self.step(action)
        return observation
        # return self.initial_pos

    def step(self, action):
        assert len(action) == 6
        self.count_for_print += 1
        current_joint_pos = np.zeros(6)
        new_state_joint_pos = np.zeros(6)
        # print("type of actions ", type(action))
        action = [0.01*x for x in action]
        action = np.clip(action, self.action_lims_low, self.action_lims_high)
        # new_state = np.zeros(self.joints_to_control.shape)
        for val, jt_name in enumerate(self.joints_to_control):
            current_joint_pos[val] = self.obj_handle.get_joint_pos(jt_name)
            new_state_joint_pos[val] = np.add(current_joint_pos[val], action[val])
        # print("fk tip ", fk_tip)
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        # print("new state xyz pos ", new_state_xyz_pos.shape)
        if self.invalid_joint_pos(new_state_joint_pos):
            joint_pos = self.previous_joint_pos
            updated_state, rewards, done, info = self._update_observation(joint_pos, action, flag=1)
        else:
            joint_pos = new_state_joint_pos
            updated_state, rewards, done, info = self._update_observation(joint_pos, action, flag=0)

        for val, jt_name in enumerate(self.joints_to_control):
            self.obj_handle.set_joint_pos(jt_name, joint_pos[val])
        # req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
        # new_state_joint_pos, _ = test_ik(xyz_cartesian_pos[0, 0], xyz_cartesian_pos[0, 1], -xyz_cartesian_pos[0, 2],
        #                                  req_rot[0], req_rot[1], req_rot[2])
        # print("Set value is ", new_state_joint_pos)
        # if self.invalid_joint_pos(new_state_joint_pos):
        #     joint_pos = self.previous_joint_pos
        # else:
        #     joint_pos = new_state_joint_pos
        self.world_handle.update()
        fk_tip = compute_FK(joint_pos)
        xyz_cartesian_pos = fk_tip[0:3, 3].reshape((1, 3))
        self.previous_cartesian_pos = xyz_cartesian_pos
        self.previous_joint_pos = joint_pos
        # if self.count_for_print % 500 == 0:
        print("Previous pos ", self.previous_cartesian_pos, "Action is ", action,
              "new pos after action ", xyz_cartesian_pos)
        print("Reward is ", rewards)

        return np.array(updated_state), rewards, done, info

    def invalid_cartesian_pos(self, cart_pos):
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        # print("joint pos ", joint_pos)
        check_cart_val = np.all((-0.18 <= cart_pos[0, 0] <= 0.18) and (-0.1 <= cart_pos[0, 1] <= 0.1)
                                and (-0.175 < cart_pos[0, 2] < 0.0))
        # print("check val is ", check_val)
        if check_cart_val:
            return False
        else:
            return True

    def invalid_joint_pos(self, joint_pos):
        # self.states_lims_low = np.array([-1.605, -0.93556, -0.002444, -3.0456, -3.0414, -3.0481, -3.0498])
        # self.states_lims_high = np.array([1.5994, 0.94249, 0.24001, 3.0485, 3.0528, 3.0376, 3.0399])
        check_joint_val = np.all((-1.605 <= joint_pos[0] <= 1.5994) and (-0.93556 <= joint_pos[1] <= 0.94249)
                                 and (0.075 <= joint_pos[2] <= 0.24001) and (-3.0456 <= joint_pos[3] <= 3.0485)
                                 and (-3.0414 <= joint_pos[4] <= 3.0528) and (-3.0481 <= joint_pos[5] <= 3.0376))
        # print("check val is ", check_val)
        if check_joint_val:
            return False
        else:
            return True

    def render(self, mode):
        print(' I am {} Ironman'.format(mode))

    def _update_observation(self, state, action, flag):
        if self.enable_step_throttling:
            step_jump = 0
            while step_jump < self.n_skip_steps:
                step_jump = self.obj_handle.get_sim_step() - self.prev_sim_step
                time.sleep(0.00001)
            self.prev_sim_step = self.obj_handle.get_sim_step()
            if step_jump > self.n_skip_steps:
                print('WARN: Jumped {} steps, Default skip limit {} Steps'.format(step_jump, self.n_skip_steps))
        else:
            cur_sim_step = self.obj_handle.get_sim_step()
            step_jump = cur_sim_step - self.prev_sim_step
            self.prev_sim_step = cur_sim_step

        # state = self.obj_handle.get_pose() + [step_jump]
        # print("Individual terms", self.obj_handle.get_pose(), self.base_handle.get_pose(), step_jump)
        # print(type(self.obj_handle.get_pose()))
        state = state.tolist() + self.goal_state + [step_jump]
        # state = self.obj_handle.get_pose() + self.base_handle.get_pose() + [step_jump]
        # print("state is ", self.obj_handle.get_pose(),  self.goal_state)
        self.obs.state = state
        # Flag checks if it takes it out of the observation limits. Flag=1 means it is invalid move
        if flag == 0:
            self.obs.reward, self.obs.is_done = self._calculate_reward(state, action)
        else:
            self.obs.reward, self.obs.is_done = -1, False
        self.obs.info = self._update_info()
        return self.obs.state, self.obs.reward, self.obs.is_done, self.obs.info

    def _calculate_reward(self, state, action):
        prev_dist = self.obs.dist
        # print("State 0;3 ", state[0:3])
        fk_tip = compute_FK(state[0:6])
        cartesian_pos = fk_tip[0:3, 3]
        # req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
        cur_dist = LA.norm(np.subtract(state[6:9], cartesian_pos))

        # action_penalty = np.sum(np.square(action))
        done = False
        # reward = (prev_dist - cur_dist) - 4 * action_penalty
        # if abs(cur_dist) < 0.001:
        #     reward = 1
        #     done = True
        # elif abs(cur_dist) < 0.1:
        #     reward = 0.01
        # elif abs(cur_dist) < 0.5:
        #     reward = 0.001
        # else:
        #     reward = 0
        reward = 1 - float(abs(cur_dist)/0.3)*0.5
        if abs(cur_dist) < 0.25:
            reward = 1
            done = True
            self.reset()
        # print("Cur dist ", cur_dist)
        self.obs.dist = cur_dist
        return reward, done

    # def _check_if_done(self, dist):
    #     if :
    #         return True
    #     else:
    #         return False

    def _update_info(self):
        return {}


# old code trials for step
#  1
        # for val, jt_name in enumerate(self.joints_to_control):
        #     print("Current position is ", self.obj_handle.get_joint_pos(jt_name))
        #
        #     new_state[val] = self.obj_handle.get_joint_pos(jt_name) + action[val]
        #     print("New state is ", new_state[val], new_state.shape)
        #     # print("jt name and val is ", jt_name, val)
        #     self.obj_handle.set_joint_pos(jt_name, new_state[val])
        # self.obj_handle.set_joint_pos('baselink-yawlink', action[0])
        # self.obj_handle.set_joint_pos('yawlink-pitchbacklink', action[1])
        # self.obj_handle.set_joint_pos('pitchendlink-maininsertionlink', action[2])
        # self.obj_handle.set_joint_pos('maininsertionlink-toolrolllink', action[3])
        # self.obj_handle.set_joint_pos('toolrolllink-toolpitchlink', action[4])
        # self.obj_handle.set_joint_pos('toolpitchlink-toolgripper1link', action[5])
        # self.obj_handle.set_joint_pos('pitchendlink-maininsertionlink', action[3])

# 2

'''
#!/usr/bin/env python
#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019, AMBF
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
from ambf_client import Client
from gym import spaces
import numpy as np
import math
import time
import gym
from gym.utils import seeding
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
from psmIK import compute_IK, test_ik
from psmFK import compute_FK
# from PyKDL import Vector, Rotation, Frame, dot
from transformations import euler_from_matrix



class Observation:
    def __init__(self):
        self.state = [0]*13
        self.dist = 0
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return np.array(self.state), self.reward, self.is_done, self.info


class AmbfEnv(gym.Env):
    def __init__(self):
        super(AmbfEnv, self).__init__()
        self.obj_handle = Object
        self.world_handle = World
        self.ambf_client = Client()
        self.ambf_client.connect()
        time.sleep(5)
        self.ambf_client.create_objs_from_rostopics()
        # time.sleep(5)
        self.n_skip_steps = 5
        self.enable_step_throttling = True
        self.action = []
        self.obs = Observation()
        self.initial_pos = self.obs.cur_observation()[0]
        self.previous_cartesian_pos = np.zeros((1, 3))
        self.previous_joint_pos = np.zeros(6)
        # Action Limit values: Z-> +-0.5 || X, Y-> +-0.05
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        self.action_lims_low = np.array([-0.05, -0.05, -0.05])
        self.action_lims_high = np.array([0.05, 0.05, 0.05])
        self.action_space = spaces.Box(low=self.action_lims_low, high=self.action_lims_high)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(13,))
        # self.states_lims_low = np.array([-1.605, -0.93556, -0.002444, -3.0456, -3.0414, -3.0481, -3.0498])
        # self.states_lims_high = np.array([1.5994, 0.94249, 0.24001, 3.0485, 3.0528, 3.0376, 3.0399])
        # self.observation_space = spaces.Box(self.states_lims_low, self.states_lims_high, shape=(13,), dtype=np.float32)
        self.joints_to_control = np.array(['baselink-yawlink', 'yawlink-pitchbacklink',
                                           'pitchendlink-maininsertionlink', 'maininsertionlink-toolrolllink',
                                           'toolrolllink-toolpitchlink', 'toolpitchlink-toolgripper2link'])
        # self.base_handle = self.ambf_client.get_obj_handle('PegBase')
        self.goal_state = [0.0, 0.0, -0.15, -1.57, 0, -3.138]
        self.prev_sim_step = 0
        self.count_for_print = 0
        pass
    #  0.0457060,  -0.01173, 0.01810

    def skip_sim_steps(self, num):
        self.n_skip_steps = num
        self.world_handle.set_num_step_skips(num)

    def set_throttling_enable(self, check):
        self.enable_step_throttling = check
        self.world_handle.enable_throttling(check)

    def make(self, a_name):
        self.obj_handle = self.ambf_client.get_obj_handle(a_name)
        self.world_handle = self.ambf_client.get_world_handle()
        self.world_handle.enable_throttling(self.enable_step_throttling)
        self.world_handle.set_num_step_skips(self.n_skip_steps)
        time.sleep(10)
        if self.obj_handle is None or self.world_handle is None:
            raise Exception

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        action = [0.0, 0.0, 0.0]
        observation, _, _, _ = self.step(action)
        return observation
        # return self.initial_pos

    def step(self, action):
        assert len(action) == 3
        self.count_for_print += 1
        current_joint_pos = np.zeros(6)
        action = np.clip(action, self.action_lims_low, self.action_lims_high)
        self.action = action
        # new_state = np.zeros(self.joints_to_control.shape)
        for val, jt_name in enumerate(self.joints_to_control):
            current_joint_pos[val] = self.obj_handle.get_joint_pos(jt_name)
        fk_tip = compute_FK(current_joint_pos)
        # print("fk tip ", fk_tip)
        new_state_xyz_pos = np.add(fk_tip[0:3, 3].reshape(action.shape), action)
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        # print("new state xyz pos ", new_state_xyz_pos.shape)
        if self.invalid_cartesian_pos(new_state_xyz_pos):
            xyz_cartesian_pos = self.previous_cartesian_pos
        else:
            xyz_cartesian_pos = new_state_xyz_pos
        req_rot = euler_from_matrix(fk_tip[0:3, 0:3], axes='szyx')
        new_state_joint_pos, _ = test_ik(xyz_cartesian_pos[0, 0], xyz_cartesian_pos[0, 1], -xyz_cartesian_pos[0, 2],
                                         req_rot[0], req_rot[1], req_rot[2])
        # print("Set value is ", new_state_joint_pos)
        if self.invalid_joint_pos(new_state_joint_pos):
            joint_pos = self.previous_joint_pos
        else:
            joint_pos = new_state_joint_pos
        for val, jt_name in enumerate(self.joints_to_control):
            self.obj_handle.set_joint_pos(jt_name, joint_pos[val])

        self.world_handle.update()
        updated_state, rewards, done, info = self._update_observation(new_state_joint_pos, action)
        self.previous_cartesian_pos = xyz_cartesian_pos
        self.previous_joint_pos = joint_pos
        if self.count_for_print % 500 == 0:
            print("Previous pos ", self.previous_cartesian_pos, "Action is ", action,
                  "new pos after action ", xyz_cartesian_pos)
            print("Reward is ", rewards)

        return np.array(updated_state), rewards, done, info

    def invalid_cartesian_pos(self, cart_pos):
        # State limit values: Z-> -0.04 || X, Y -> +-0.1
        # print("joint pos ", joint_pos)
        check_cart_val = np.all((-0.18 <= cart_pos[0, 0] <= 0.18) and (-0.1 <= cart_pos[0, 1] <= 0.1)
                                and (-0.175 < cart_pos[0, 2] < 0.0))
        # print("check val is ", check_val)
        if check_cart_val:
            return False
        else:
            return True

    def invalid_joint_pos(self, joint_pos):
        # self.states_lims_low = np.array([-1.605, -0.93556, -0.002444, -3.0456, -3.0414, -3.0481, -3.0498])
        # self.states_lims_high = np.array([1.5994, 0.94249, 0.24001, 3.0485, 3.0528, 3.0376, 3.0399])
        check_joint_val = np.all((-1.605 <= joint_pos[0] <= 1.5994) and (-0.93556 <= joint_pos[1] <= 0.94249)
                                 and (-0.002444 <= joint_pos[2] <= 0.24001) and (-3.0456 <= joint_pos[3] <= 3.0485)
                                 and (-3.0414 <= joint_pos[4] <= 3.0528) and (-3.0481 <= joint_pos[5] <= 3.0376))
        # print("check val is ", check_val)
        if check_joint_val:
            return False
        else:
            return True

    def render(self, mode):
        print(' I am {} Ironman'.format(mode))

    def _update_observation(self, state, action):
        if self.enable_step_throttling:
            step_jump = 0
            while step_jump < self.n_skip_steps:
                step_jump = self.obj_handle.get_sim_step() - self.prev_sim_step
                time.sleep(0.00001)
            self.prev_sim_step = self.obj_handle.get_sim_step()
            if step_jump > self.n_skip_steps:
                print('WARN: Jumped {} steps, Default skip limit {} Steps'.format(step_jump, self.n_skip_steps))
        else:
            cur_sim_step = self.obj_handle.get_sim_step()
            step_jump = cur_sim_step - self.prev_sim_step
            self.prev_sim_step = cur_sim_step

        # state = self.obj_handle.get_pose() + [step_jump]
        # print("Individual terms", self.obj_handle.get_pose(), self.base_handle.get_pose(), step_jump)
        # print(type(self.obj_handle.get_pose()))
        state = state.tolist() + self.goal_state + [step_jump]
        # state = self.obj_handle.get_pose() + self.base_handle.get_pose() + [step_jump]
        # print("state is ", self.obj_handle.get_pose(),  self.goal_state)
        self.obs.state = state
        self.obs.reward = self._calculate_reward(state, action)
        self.obs.is_done = self._check_if_done()
        self.obs.info = self._update_info()
        return self.obs.state, self.obs.reward, self.obs.is_done, self.obs.info

    def _calculate_reward(self, state, action):
        prev_dist = self.obs.dist
        # print("State 0;3 ", state[0:3])
        cur_dist = LA.norm(np.subtract(state[6:9], state[0:3]))

        # action_penalty = np.sum(np.square(action))

        # reward = (prev_dist - cur_dist) - 4 * action_penalty
        if abs(cur_dist) < 0.001:
            reward = 10
        elif abs(cur_dist) < 0.1:
            reward = 0.5
        elif abs(cur_dist) < 1:
            reward = 0.1
        else:
            reward = 0
        # reward = -(prev_dist - cur_dist)

        # print("Cur dist ", cur_dist)
        self.obs.dist = cur_dist
        return reward

    def _check_if_done(self):
        return False

    def _update_info(self):
        return {}


# old code trials for step
#  1
        # for val, jt_name in enumerate(self.joints_to_control):
        #     print("Current position is ", self.obj_handle.get_joint_pos(jt_name))
        #
        #     new_state[val] = self.obj_handle.get_joint_pos(jt_name) + action[val]
        #     print("New state is ", new_state[val], new_state.shape)
        #     # print("jt name and val is ", jt_name, val)
        #     self.obj_handle.set_joint_pos(jt_name, new_state[val])
        # self.obj_handle.set_joint_pos('baselink-yawlink', action[0])
        # self.obj_handle.set_joint_pos('yawlink-pitchbacklink', action[1])
        # self.obj_handle.set_joint_pos('pitchendlink-maininsertionlink', action[2])
        # self.obj_handle.set_joint_pos('maininsertionlink-toolrolllink', action[3])
        # self.obj_handle.set_joint_pos('toolrolllink-toolpitchlink', action[4])
        # self.obj_handle.set_joint_pos('toolpitchlink-toolgripper1link', action[5])
        # self.obj_handle.set_joint_pos('pitchendlink-maininsertionlink', action[3])

# 2
'''