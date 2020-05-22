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
        'nb_train_steps': 300,
        'nb_rollout_steps': 100,
        'gamma': 0.95,
        'observation_range': (-1.5, 1.5),
        'random_exploration': 0.05,
        'normalize_observations': True,
        'critic_l2_reg': 0.01
    }

    # NOTE:
    # If continuing learning from previous checkpoint,
    # Comment model=HER(''') till model.save("./her_robot_env")  [lines 44:51] and uncomment below lines:
    # Replace the XXXXX below with the largest number present in (rl_model_) directory ./ddpg_dvrk_tensorboard/
    # remaining_training_steps = 4000000 - 700000
    # model_log_dir = './ddpg_dvrk_tensorboard/rl_model_700000_steps.zip'
    # model = HER.load(model_log_dir, env=env)
    # # Reset the model
    # env.reset()
    # model.learn(remaining_training_steps, log_interval=100,
    #             callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    # model.save("./her_robot_env")

    # Available strategies (cf paper): future, final, episode, random
    model = HER('MlpPolicy', env, model_class, verbose=1, n_sampled_goal=4, goal_selection_strategy='future',
                buffer_size=int(1e5), batch_size=128, tensorboard_log="./ddpg_dvrk_tensorboard/", **kwargs)
    # Reset the model
    env.reset()
    # Train the model
    model.learn(4000000, log_interval=100,
                callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    model.save("./her_robot_env")


def load_model(eval_env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load('./rl_model_400000_steps', env=eval_env)
    # model = HER.load('./her_robot_env', env=eval_env)
    count = 0
    step_num_arr = []
    # obs = eval_env.reset()
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
                # time.sleep(5)
                break
    print("It reached goal position successfully ", count, " Average step count ", step_num_arr,
          np.average(np.array(step_num_arr)))


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


