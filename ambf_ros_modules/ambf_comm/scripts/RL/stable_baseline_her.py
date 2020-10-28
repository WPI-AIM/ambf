#!/usr/bin/python3

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

import numpy as np
from stable_baselines import HER, DDPG
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines.bench import Monitor
from ambf_comm import AmbfEnvHERDDPG
import time
from stable_baselines.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
import os, sys
from datetime import datetime


def redirect_stdout(filepath=None):
    # os.makedirs('/home/admin/ambf/.logs')
    cdir = os.getcwd()
    basepath = os.path.join(cdir, '.logs')

    if not os.path.exists(basepath):
      os.makedirs(basepath)
    
    if filepath is None:
        now = datetime.now()
        filepath = 'log_' + now.strftime("%Y_%m_%d-%H_%M_%S.txt")
        filepath = os.path.join(basepath, filepath)
    
    err_filepath = filepath[:-4] + '_err.txt'

    if os.path.exists(filepath):
        filepath = filepath[:-4]
        filepath += now.strftime("_%H_%M_%S") + '.txt'

    sys.stdout = open(filepath, 'w')
    sys.stderr = open(err_filepath, 'w')
    print("Began logging")
    return

def main(training_env, eval_env=None, log_dir='./.logs/results'):
    
    os.makedirs(log_dir, exist_ok=True)

    # training_env = Monitor(training_env, log_dir)

    n_actions = training_env.action_space.shape[0]
    noise_std = 0.2
    # Currently using OU noise
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=noise_std * np.ones(n_actions))
    model_class = DDPG  # works also with SAC, DDPG and TD3

    rl_model_kwargs = {
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'action_noise': action_noise,
        'nb_train_steps': 300,
        'nb_rollout_steps': 100,
        'gamma': 0.95,
        'observation_range': (-1.5, 1.5),
        'random_exploration': 0.05,
        'normalize_observations': True,
        'critic_l2_reg': 0.01
    }

    # Available strategies (cf paper): future, final, episode, random
    model = HER('MlpPolicy', training_env, model_class, verbose=1, n_sampled_goal=4, goal_selection_strategy='future',
                buffer_size=int(1e5), batch_size=128, tensorboard_log="./ddpg_dvrk_tensorboard/", **rl_model_kwargs)
    # Reset the model
    training_env.reset()
    # Create callbacks
    checkpoint_callback = CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/") # save_path="./.model/model_checkpoint/") #save_freq=100000
    # eval_callback = EvalCallback(training_env, best_model_save_path='./ddpg_dvrk_tensorboard/best_model',
    #                             log_path=log_dir, eval_freq=500)
    callback = CallbackList([checkpoint_callback]) # , eval_callback])
    # Train the model
    model.learn(4000000, log_interval=100,
                callback=callback) 
    model.save("./her_robot_env")

    # NOTE:
    # If continuing learning from previous checkpoint,
    # Comment above chunk of code {model=HER(''') till model.save("./her_robot_env")} and uncomment below lines:
    # Replace the XXXXX below with the largest number present in (rl_model_) directory ./ddpg_dvrk_tensorboard/
    # remaining_training_steps = 4000000 - XXXXX
    # model_log_dir = './ddpg_dvrk_tensorboard/rl_model_XXXXX_steps.zip'
    # model = HER.load(model_log_dir, env=env)
    # # Reset the model
    # env.reset()
    # model.learn(remaining_training_steps, log_interval=100,
    #             callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    # model.save("./her_robot_env")


def load_model(eval_env):
    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load('./her_robot_env', env=eval_env)
    count = 0
    step_num_arr = []
    for _ in range(20):
        number_steps = 0
        obs = eval_env.reset()
        for _ in range(400):
            action, _ = model.predict(obs)
            obs, reward, done, _ = eval_env.step(action)
            number_steps += 1
            # print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
            if done:
                step_num_arr.append(number_steps)
                count += 1
                print("----------------It reached terminal state -------------------")
                break
    print("Robot reached the goal position successfully ", count, " times and the Average step count was ",
          np.average(np.array(step_num_arr)))


if __name__ == '__main__':
    # redirect_stdout()
    ENV_NAME = 'psm/baselink'
    env_kwargs = {
        'action_space_limit': 0.05,
        'joints_to_control': np.array(['baselink-yawlink',
                                       'yawlink-pitchbacklink',
                                       'pitchendlink-maininsertionlink',
                                       'maininsertionlink-toolrolllink',
                                       'toolrolllink-toolpitchlink',
                                       'toolpitchlink-toolgripper1link',
                                       'toolpitchlink-toolgripper2link']),
        'goal_position_range': 0.05,
        'position_error_threshold': 0.01,
        'goal_error_margin': 0.0075,
        'joint_limits': {
                            'lower_limit': np.array([-0.2, -0.2, 0.1, -1.5, -1.5, -1.5, -1.5]),
                            'upper_limit': np.array([0.2, 0.2, 0.24, 1.5, 1.5, 1.5, 1.5])
                        },
        'workspace_limits': {
                                'lower_limit': np.array([-0.04, -0.03, -0.2]),
                                'upper_limit': np.array([0.03, 0.04, -0.091])
                            },
        'enable_step_throttling': False,
    }
    # Training
    print("Creating training_env")
    ambf_env = AmbfEnvHERDDPG(**env_kwargs)
    time.sleep(5)
    ambf_env.make(ENV_NAME)
    ambf_env.reset()

    main(training_env=ambf_env) #, eval_env=eval_env)
    ambf_env.ambf_client.clean_up()

    # Evaluate learnt policy
    print("Creating eval_env")
    eval_env = AmbfEnvHERDDPG(**env_kwargs)
    time.sleep(5)
    eval_env.make(ENV_NAME)
    eval_env.reset()
    
    load_model(eval_env=eval_env)
    eval_env.ambf_client.clean_up()


