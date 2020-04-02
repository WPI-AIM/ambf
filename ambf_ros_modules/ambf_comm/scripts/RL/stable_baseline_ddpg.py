import gym
import numpy as np
import tensorflow as tf
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG
from ambf_comm import AmbfEnv
import time

ENV_NAME = 'psm/baselink'

# Get the environment and extract the number of actions.
env = AmbfEnv()
time.sleep(10)

env.make(ENV_NAME)
time.sleep(10)

env.reset()
i = 0
# the noise objects for DDPG
# print("actions space ", np.abs(env.action_space.low), np.abs(env.action_space.high))
n_actions = env.action_space.shape[0]
param_noise = None
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

model = DDPG(MlpPolicy, env, gamma=0.99, verbose=1, nb_train_steps=10, nb_rollout_steps=10, nb_eval_steps=10,
             param_noise=param_noise, batch_size=64, action_noise=action_noise, random_exploration=0.05,
             tensorboard_log="./ddpg_dvrk_tensorboard/", n_cpu_tf_sess=1, observation_range=(-3.05, 3.05))

model.learn(total_timesteps=300, log_interval=10)
model.save("ddpg_robot_1")
# print(model.actor_loss, model.critic_loss)
del model # remove to demonstrate saving and loading

model = DDPG.load("ddpg_robot_1")



obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    # env.render()
# for i in range(100):  # Total episodes to train
#     obs = env.reset()
#     # print("observation ", obs)
#     done = False
#     while not done:
#         action, states = model.predict(obs)
#         obs, rewards, done, info = env.step(action)
        # print("current state, action is ", _states, action, " resulting state ", obs)
        # if i % 100 == 0:
            # print("current state, action is ", states, action, " resulting state ", obs)
            # print(" reward is ", rewards)
        # env.render('ansi')
