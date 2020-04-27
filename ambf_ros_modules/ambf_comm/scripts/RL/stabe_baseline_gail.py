from stable_baselines.gail import ExpertDataset
import numpy as np
from stable_baselines import HER, DDPG, PPO2
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from ambf_comm import AmbfEnv
import time
from stable_baselines.ddpg.policies import MlpPolicy

from stable_baselines.common.callbacks import CheckpointCallback


def main(env):

    n_actions = env.action_space.shape[0]
    noise_std = 0.2
    # Currently using OU noise (need to check with NA and AP noise)
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=noise_std * np.ones(n_actions))

    kwargs = {
        'actor_lr': 1e-3,
        'critic_lr': 1e-3,
        'action_noise': action_noise,
        'nb_train_steps': 300,
        'nb_rollout_steps': 150,
        'gamma': 0.95,
        'observation_range': (-1.5, 1.5),
        'random_exploration': 0.05,
        'normalize_observations': True
    }
    # Using only one expert trajectory
    # you can specify `traj_limitation=-1` for using the whole dataset
    file_dir = "/home/vignesh/Thesis_Suture_data/trial2/ambf_data/"
    dataset = ExpertDataset(expert_path=file_dir + 'expert_psm_data.npz',
                            traj_limitation=1, batch_size=32)
    model_class = DDPG  # works also with SAC, DDPG and TD3
    # model = DDPG(MlpPolicy, env, verbose=1, **kwargs)
    model = HER('MlpPolicy', env, model_class, verbose=1, n_sampled_goal=4, goal_selection_strategy='future',
                buffer_size=int(1e5), batch_size=128, tensorboard_log="./ddpg_dvrk_tensorboard/", **kwargs)
    # Pretrain the PPO2 model
    model.pretrain(dataset, n_epochs=1000)
    model.save("./gail_robot_env")

    # As an option, you can train the RL agent
    # model.learn(int(1e5))


def load_model(eval_env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = HER.load('./gail_robot_env', env=eval_env)
    reward_sum = 0.0

    # obs = eval_env.reset()
    for _ in range(20):
        obs = eval_env.reset()
        for _ in range(1000):
            action, _ = model.predict(obs)
            obs, reward, done, _ = eval_env.step(action)
            reward_sum += reward
            print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
            if done:
                print("----------------It reached terminal state -------------------")
                print(reward_sum)
                reward_sum = 0.0
                time.sleep(5)
                break


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