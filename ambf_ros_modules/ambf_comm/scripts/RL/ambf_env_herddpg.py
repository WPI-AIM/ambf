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
#     \author    <amunawar@wpi.edu>, <vvarier@wpi.edu>
#     \author    Adnan Munawar and Vignesh Manoj Varier
#     \version   0.1
# */
# //==============================================================================

from ambf_client import Client
from gym import spaces
import numpy as np
import copy
import time
import gym
from gym.utils import seeding
from ambf_world import World
from ambf_object import Object
from numpy import linalg as LA
# from psmIK import *
from psmFK import compute_FK
from transformations import euler_from_matrix
import rospy
from dvrk_functions.srv import *

class Observation:
    def __init__(self):
        # Goal based environment for HER
        self.state = {
            'observation': np.zeros(20),
            # Prismatic joint is set to 0.1 to ensure at least some part of robot tip goes past the cannula
            'achieved_goal': np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]),
            'desired_goal': np.zeros(6)
        }
        self.dist = 0
        self.reward = 0.0
        self.prev_reward = 0.0
        self.cur_reward = 0.0
        self.is_done = False
        self.info = {}
        self.sim_step_no = 0

    def cur_observation(self):
        return self.state, self.reward, self.is_done, self.info


class AmbfEnvHERDDPG(gym.GoalEnv):
    def __init__(self, action_space_limit, joints_to_control, goal_position_range, position_error_threshold,
                 goal_error_margin, joint_limits, workspace_limits, enable_step_throttling):
        super(AmbfEnvHERDDPG, self).__init__()
        # AMBF Initialization commands
        self.obj_handle = Object
        self.world_handle = World
        self.ambf_client = Client()
        self.ambf_client.connect()
        time.sleep(5)
        self.ambf_client.create_objs_from_rostopics()

        self.seed()
        self.n_skip_steps = 5
        self.enable_step_throttling = enable_step_throttling
        self.joint_limits = joint_limits
        self.workspace_limits = workspace_limits
        self.obs = Observation()
        self.initial_pos = copy.deepcopy(self.obs.cur_observation()[0])
        self.cmd_joint_pos = np.zeros(7)
        self.joints_to_control = joints_to_control
        self.goal_position_range = goal_position_range
        self.goal = np.array([0.0, 0.0, -0.1, 0.0, 0.0, 0.0])
        self.prev_sim_step = 0
        self.pos_error_threshold = position_error_threshold
        self.count_for_print = 0
        self.goal_error_margin = goal_error_margin
        self.n_actions = 3
        self.action_lims_low = -action_space_limit*np.ones(self.n_actions)
        self.action_lims_high = action_space_limit*np.ones(self.n_actions)
        self.action_space = spaces.Box(low=-action_space_limit, high=action_space_limit,
                                       shape=(self.n_actions,), dtype="float32")
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=self.initial_pos['achieved_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=self.initial_pos['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=self.initial_pos['observation'].shape, dtype='float32'),
        ))

    def skip_sim_steps(self, num):
        # Function to define the number of steps that can be skipped if Step Throttling is enabled
        self.n_skip_steps = num
        self.world_handle.set_num_step_skips(num)

    def set_throttling_enable(self, check):
        # Function to set the Step Throttling Boolean
        self.enable_step_throttling = check
        self.world_handle.enable_throttling(check)

    def make(self, a_name):
        # Function to create object handle for the robot and world in AMBF
        self.obj_handle = self.ambf_client.get_obj_handle(a_name)
        # self.base_handle = self.ambf_client.get_obj_handle('SampleObj')
        self.world_handle = self.ambf_client.get_world_handle()
        self.world_handle.enable_throttling(self.enable_step_throttling)
        self.world_handle.set_num_step_skips(self.n_skip_steps)
        time.sleep(2)
        if self.obj_handle is None or self.world_handle is None:
            raise Exception

    def seed(self, seed=None):
        # Random seed
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        # Function to reset the model
        # Sets the initial position of PSM
        self.set_initial_pos_func()
        # Type 1 Reset : Uses the previous reached state as the initial state for next iteration
        # action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # observation, _, _, _ = self.step(action)
        # Type 2 Reset : Sets the robot to a predefined initial state for each iteration
        initial_joint_pos, initial_state_vel = self.get_joint_pos_vel_func()
        end_effector_frame = compute_FK(initial_joint_pos)
        # Updates the observation to the initialized position
        observation, _, _, _ = self._update_observation(end_effector_frame, initial_joint_pos, initial_state_vel)
        # Samples a goal
        self.goal = self._sample_goal(observation)
        return observation

    def set_initial_pos_func(self):
        # Function to set the initial joint positions of the robot
        for joint_idx, jt_name in enumerate(self.joints_to_control):
            # Prismatic joint is set to different value to ensure at least some part of robot tip goes past the cannula
            if joint_idx == 2:
                self.obj_handle.set_joint_pos(jt_name, self.joint_limits['lower_limit'][2])
            else:
                self.obj_handle.set_joint_pos(jt_name, 0)
        time.sleep(0.5)

    def get_joint_pos_vel_func(self):
        # Function to compute Joint Position and Velocity of the robot
        joint_position = np.zeros(7)
        joint_velocity = np.zeros(7)
        for joint_idx, jt_name in enumerate(self.joints_to_control):
            joint_position[joint_idx] = self.obj_handle.get_joint_pos(jt_name)
            joint_velocity[joint_idx] = self.obj_handle.get_joint_vel(jt_name)

        return joint_position, joint_velocity

    def step(self, action):
        assert len(action) == 3
        # Counter for printing position, action and reward
        self.count_for_print += 1
        # Clipping the action value between the desired range
        action = np.clip(action, self.action_lims_low, self.action_lims_high)
        # currently using Python implementation of finding joint velocity
        current_joint_pos, state_vel = self.get_joint_pos_vel_func()
        current_end_effector_frame = compute_FK(current_joint_pos)
        # Compute the resulting state after applying the action from current state
        # Since end effector frame is a numpy matrix, converting it to an array
        new_state_cartesian_pos = np.add(np.asarray(current_end_effector_frame[0:3, 3]).reshape(-1), action)
        # Ensure the new state is within valid joint positions, if invalid then stay at the joint limit position
        desired_cartesian_pos = self.limit_cartesian_pos(new_state_cartesian_pos)
        # Creates the frame for new state S'(in the code shown below only position value is changing through action)
        desired_end_effector_frame = current_end_effector_frame
        for i in range(3):
            desired_end_effector_frame[i, 3] = desired_cartesian_pos[i]

        rospy.wait_for_service('compute_IK')
        computed_joint_pos = None
        try:
            compute_IK_service = rospy.ServiceProxy('compute_IK', ComputeIK)
            compute_IK_resp = compute_IK_service.call(ComputeIKRequest(convert_mat_to_frame(desired_end_effector_frame)))
            computed_joint_pos = compute_IK_resp.q_des
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        # computed_joint_pos = compute_IK(convert_mat_to_frame(desired_end_effector_frame))
        # Ensure the computed joint positions are within the limit of user set joint positions
        desired_joint_pos = self.limit_joint_pos(computed_joint_pos)
        # Ensures that PSM joints reach the desired joint positions
        self.set_commanded_joint_pos(desired_joint_pos)
        # Update state, reward, done flag and world values in the code
        updated_state, rewards, done, info = self._update_observation(desired_end_effector_frame,
                                                                      desired_joint_pos, state_vel)
        self.world_handle.update()
        # Print function for viewing the output intermittently
        if self.count_for_print % 10000 == 0:
            print("count ", self.count_for_print, "Action is ", action, " new joint pos ",
                  updated_state['observation'][6:13], " achieved goal ", updated_state['achieved_goal'],
                  " desired goal ", updated_state['desired_goal'])
            print("Reward is ", rewards)

        return updated_state, rewards, done, info

    def set_commanded_joint_pos(self, commanded_joint_pos):
        # Function to ensure the robot tip reaches the desired goal position before moving on to next iteration
        # Counter to avoid getting stuck in while loop because of very small errors in positions
        count_for_joint_pos = 0
        while True:
            reached_joint_pos = np.zeros(7)
            for joint_idx, jt_name in enumerate(self.joints_to_control):
                self.obj_handle.set_joint_pos(jt_name, commanded_joint_pos[joint_idx])
                reached_joint_pos[joint_idx] = self.obj_handle.get_joint_pos(jt_name)

            error_in_pos = np.around(np.subtract(commanded_joint_pos, reached_joint_pos), decimals=3)
            # Since Prismatic joint limits are smaller compared to the limits of other joints
            error_in_pos_joint2 = np.around(np.subtract(commanded_joint_pos[2], reached_joint_pos[2]), decimals=4)
            count_for_joint_pos += 1
            if (np.all(np.abs(error_in_pos) <= self.pos_error_threshold) and
                np.abs(error_in_pos_joint2) <= 0.5*self.pos_error_threshold) \
                    or count_for_joint_pos > 75:
                break

    def limit_cartesian_pos(self, cart_pos):
        # Limits the robot tip X, Y, Z positions
        limit_cartesian_pos_values = np.zeros(3)
        cartesian_pos_lower_limit = self.workspace_limits['lower_limit']
        cartesian_pos_upper_limit = self.workspace_limits['upper_limit']
        for axis in range(3):
            limit_cartesian_pos_values[axis] = np.clip(cart_pos[axis], cartesian_pos_lower_limit[axis],
                                                       cartesian_pos_upper_limit[axis])
        return limit_cartesian_pos_values

    def limit_joint_pos(self, joint_pos):
        # Limits the joint position values of the robot
        # dvrk_limits_low = np.array([-1.605, -0.93556, -0.002444, -3.0456, -3.0414, -3.0481, -3.0498])
        # dvrk_limits_high = np.array([1.5994, 0.94249, 0.24001, 3.0485, 3.0528, 3.0376, 3.0399])
        # Note: Joint 5 and 6, joint pos = 0, 0 is closed jaw and 0.5, 0.5 is open
        limit_joint_values = np.zeros(7)
        joint_lower_limit = self.joint_limits['lower_limit']
        joint_upper_limit = self.joint_limits['upper_limit']
        for joint_idx in range(len(joint_pos)):
            limit_joint_values[joint_idx] = np.clip(joint_pos[joint_idx], joint_lower_limit[joint_idx],
                                                    joint_upper_limit[joint_idx])

        return limit_joint_values

    def render(self, mode):
        # Not being utilized since AMBF runs in parallel for visualization
        print(' I am {} Ironman'.format(mode))

    def _update_observation(self, end_state, joint_state, state_vel):
        # Function to update all the values
        # Function implementing Step Throttling algorithm
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

        # Robot tip position and orientation
        cartesian_pos = end_state[0:3, 3]
        achieved_rot = np.array(euler_from_matrix(end_state[0:3, 0:3], axes='szyx')).reshape((3, 1))
        # Combine tip position and orientation to a single numpy array as achieved goal
        achieved_goal = np.asarray(np.concatenate((cartesian_pos.copy(), achieved_rot.copy()), axis=0)).reshape(-1)
        obs = np.asarray(np.concatenate((cartesian_pos.copy(), achieved_rot.copy(),
                                         joint_state.reshape((7, 1))), axis=0)).reshape(-1)
        # Combine the current state positions and velocity into a single array as observation
        observation = np.concatenate((obs, state_vel), axis=None)
        # Update the observation dictionary
        self.obs.state.update(observation=observation.copy(), achieved_goal=achieved_goal.copy(),
                              desired_goal=self.goal.copy())
        # Update info
        self.obs.info = self._update_info()
        # Compute the reward
        self.obs.reward = self.compute_reward(self.obs.state['achieved_goal'], self.goal, self.obs.info)
        self.obs.is_done = self._check_if_done()

        # Return the values to step function
        return self.obs.state, self.obs.reward, self.obs.is_done, self.obs.info

    def compute_reward(self, achieved_goal, goal, info):
        # Function to compute reward received by the agent
        # Find the distance between goal and achieved goal
        cur_dist = LA.norm(np.subtract(goal[0:3], achieved_goal[0:3]))

        # Continuous reward
        # reward = round(1 - float(abs(cur_dist)/0.3)*0.5, 5)
        # Sparse reward
        if abs(cur_dist) < self.goal_error_margin:
            reward = 1
        else:
            reward = -1
        self.obs.dist = cur_dist
        return reward

    def _sample_goal(self, observation):
        # Function to samples new goal positions and ensures its within the workspace of PSM
        rand_val_pos = np.around(np.add(observation['achieved_goal'][0:3],
                                        self.np_random.uniform(-self.goal_position_range,
                                                               self.goal_position_range,
                                                               size=3)),
                                 decimals=4)
        rand_val_pos[0] = np.around(np.clip(rand_val_pos[0], self.workspace_limits['lower_limit'][0],
                                            self.workspace_limits['upper_limit'][0]), decimals=4)
        rand_val_pos[1] = np.around(np.clip(rand_val_pos[1], self.workspace_limits['lower_limit'][1],
                                            self.workspace_limits['upper_limit'][1]), decimals=4)
        rand_val_pos[2] = np.around(np.clip(rand_val_pos[2], self.workspace_limits['lower_limit'][2],
                                            self.workspace_limits['upper_limit'][2]), decimals=4)
        # Uncomment below lines if individual limits need to be set for generating desired goal state
        '''
        rand_val_pos[0] = np.around(np.clip(rand_val_pos[0], -0.1388, 0.1319), decimals=4)
        rand_val_pos[1] = np.around(np.clip(rand_val_pos[1], -0.1319, 0.1388), decimals=4)
        rand_val_pos[2] = np.around(np.clip(rand_val_pos[2], -0.1935, -0.0477), decimals=4)
        rand_val_angle[0] = np.clip(rand_val_angle[0], -0.15, 0.15)
        rand_val_angle[1] = np.clip(rand_val_angle[1], -0.15, 0.15)
        rand_val_angle[2] = np.clip(rand_val_angle[2], -0.15, 0.15)
        '''
        # Provide the range for generating the desired orientation at the terminal state
        rand_val_angle = self.np_random.uniform(-1.5, 1.5, size=3)
        goal = np.concatenate((rand_val_pos, rand_val_angle), axis=None)
        return goal.copy()

    def _check_if_done(self):
        # Function to check if the episode was successful
        if abs(self.obs.dist) < self.goal_error_margin:
            return True
        else:
            return False

    def _update_info(self):
        # Can be used to Provide information for debugging purpose
        info = {'is_success': self._is_success()}
        return info

    def _is_success(self):
        # Function to check if the robot reached the desired goal within a predefined error margin
        if abs(self.obs.dist) < self.goal_error_margin:
            return True
        else:
            return False

