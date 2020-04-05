import os
import numpy as np
from stable_baselines import HER, DQN, SAC, DDPG, TD3
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines.bench import Monitor
from ambf_comm import AmbfEnv
import time


def find_save_path(dir, trial_id):
    i=0
    while True:
        save_dir = dir+str(trial_id+i*100)+'/'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            break
        i+=1
    return save_dir


def main(env):
    # ENV_NAME = 'psm/baselink'
    #
    # # Get the environment and extract the number of actions.
    # env = AmbfEnv()
    #
    # env.make(ENV_NAME)
    #
    # env.reset()
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
        'nb_train_steps': 500,
        'nb_rollout_steps': 150,
        'nb_eval_steps': 500,
        'gamma': 0.95,
        'observation_range': (-3.05, 3.05),
        'random_exploration': 0.05
    }

    # Wrap the model
    trial_id = 1
    logdir = find_save_path("./ddpg_dvrk_tensorboard/", trial_id)
    os.makedirs(logdir, exist_ok=True)
    env = Monitor(env, logdir, allow_early_resets=True)

    model = HER('MlpPolicy', env, model_class, verbose=1, n_sampled_goal=4, goal_selection_strategy='future',
                buffer_size=int(1e6), batch_size=256, **kwargs)
    # model = HER('MlpPolicy', env, model_class, n_sampled_goal=4, goal_selection_strategy=goal_selection_strategy,
    #                                                 verbose=1, batch_size=64)
    env.reset()
    # Train the model
    model.learn(500000, log_interval=1, tb_log_name="./ddpg_dvrk_tensorboard/")
    # model.save(os.path.join(logdir, "HERDDPG"))
    model.save("./her_robot_env")

def load_model(env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load('./her_robot_env', env=env)

    obs = env.reset()
    for _ in range(100):
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        if done:
            obs = env.reset()


if __name__ == '__main__':

    ENV_NAME = 'psm/baselink'
    # Get the environment and extract the number of actions.
    env = AmbfEnv()
    time.sleep(5)
    env.make(ENV_NAME)
    env.reset()
    main(env=env)
    load_model(env=env)

