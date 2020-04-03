import gym
import numpy as np
from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from ambf_comm import AmbfEnv
import time

ENV_NAME = 'psm/baselink'

# Get the environment and extract the number of actions.
env = AmbfEnv()

env.make(ENV_NAME)

env.reset()
i = 0
n_actions = env.action_space.shape[0]
noise_std = 0.2
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=noise_std * np.ones(n_actions))
model_class = DDPG  # works also with SAC, DDPG and TD3

# Available strategies (cf paper): future, final, episode, random
goal_selection_strategy = 'future' # equivalent to GoalSelectionStrategy.FUTURE
kwargs = {
    'actor_lr': 1e-3,
    'critic_lr': 1e-3,
    'action_noise': action_noise,
    'nb_rollout_steps': 1000,
    'tensorboard_log': "./ddpg_dvrk_tensorboard/",
    'gamma': 0.95
}
NB_TRAIN_EPS = 95000
EP_TIMESTEPS = 50
# Wrap the model
model = HER('MlpPolicy', env, model_class, n_sampled_goal=4, goal_selection_strategy='future', buffer_size=int(1e6),
            batch_size=256, **kwargs)
# model = HER('MlpPolicy', env, model_class, n_sampled_goal=4, goal_selection_strategy=goal_selection_strategy,
#                                                 verbose=1, batch_size=64)
# Train the model
model.learn(3000000, log_interval=10)

model.save("./her_robot_env")

# WARNING: you must pass an env
# or wrap your environment with HERGoalEnvWrapper to use the predict method
model = HER.load('./her_robot_env', env=env)

obs = env.reset()
for _ in range(100):
    action, _ = model.predict(obs)
    obs, reward, done, _ = env.step(action)
    if done:
        obs = env.reset()