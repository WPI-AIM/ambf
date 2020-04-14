# import os
import numpy as np
from stable_baselines import HER, DDPG
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from ambf_comm import AmbfEnv
import time
from stable_baselines.common.callbacks import CheckpointCallback


def main(env):

    n_actions = env.action_space.shape[0]
    noise_std = 0.2
    # Currently using OU noise (need to check with NA and AP noise)
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=noise_std * np.ones(n_actions))
    model_class = DDPG  # works also with SAC, DDPG and TD3

    kwargs = {
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'action_noise': action_noise,
        'nb_train_steps': 500,
        'nb_rollout_steps': 150,
        'nb_eval_steps': 500,
        'gamma': 0.95,
        'observation_range': (-3.05, 3.05),
        'random_exploration': 0.05,
        'normalize_observations': True,
        'normalize_returns': True
    }

    # NOTE:
    # If continuing learning from previous checkpoint,
    # Comment model=HER(''') till model.save("./her_robot_env")  [lines 44:51] and uncomment below lines:
    # Replace the XXXXX below with the largest number present in (rl_model_) directory ./ddpg_dvrk_tensorboard/
    # remaining_training_steps = 4000000 - XXXXX
    # model_log_dir = './ddpg_dvrk_tensorboard/rl_model_XXXXX_steps.zip'
    # model = HER.load(model_log_dir, env=env)
    # Reset the model
    # env.reset()
    # model.learn(remaining_training_steps, log_interval=100,
    #             callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    # model.save("./her_robot_env")

    # Available strategies (cf paper): future, final, episode, random
    model = HER('MlpPolicy', env, model_class, verbose=1, n_sampled_goal=4, goal_selection_strategy='future',
                buffer_size=int(1e6), batch_size=128, tensorboard_log="./ddpg_dvrk_tensorboard/", **kwargs)
    # Reset the model
    env.reset()
    # Train the model
    model.learn(4000000, log_interval=100,
                callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    model.save("./her_robot_env")


def load_model(eval_env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load('./her_robot_env', env=eval_env)
    obs = eval_env.reset()
    for _ in range(10000):
        action, _ = model.predict(obs)
        obs, reward, done, _ = eval_env.step(action)
        print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
        if done:
            print("----------------It reached terminal state -------------------")
            time.sleep(5)
            obs = eval_env.reset()


if __name__ == '__main__':

    ENV_NAME = 'psm/baselink'
    # Training
    ambf_env = AmbfEnv()
    time.sleep(5)
    ambf_env.make(ENV_NAME)
    ambf_env.reset()
    main(env=ambf_env)
    ambf_env.ambf_client.clean_up()
    # Evaluate learnt policy
    eval_env = AmbfEnv()
    time.sleep(5)
    eval_env.make(ENV_NAME)
    eval_env.reset()
    load_model(eval_env=eval_env)
    eval_env.ambf_client.clean_up()


'''
def find_save_path(dir, trial_id):
    """
    Create a directory to save results and arguments. Adds 100 to the trial id if a directory already exists.
    Params
    ------
    - dir (str)
        Main saving directory
    - trial_id (int)
        Trial identifier
    """
    i=0
    while True:
        save_dir = dir+str(trial_id+i*100)+'/'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            break
        i+=1
    return save_dir
'''

