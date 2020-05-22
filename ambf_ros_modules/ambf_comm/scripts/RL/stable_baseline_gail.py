from stable_baselines.gail import ExpertDataset
import numpy as np
from stable_baselines import HER, DDPG
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from ambf_comm import AmbfEnv
import time
from stable_baselines.ddpg.policies import MlpPolicy

from stable_baselines.common.callbacks import CheckpointCallback


def main(env):

    n_actions = env.action_space.shape[0]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

    # Using only one expert trajectory
    # you can specify `traj_limitation=-1` for using the whole dataset
    file_dir = "/home/vignesh/Thesis_Suture_data/trial2/ambf_data/"
    dataset = ExpertDataset(expert_path=file_dir + 'expert_psm_data.npz',
                            traj_limitation=1, batch_size=32)

    model = DDPG(MlpPolicy, env, gamma=0.95, verbose=1, nb_train_steps=300, nb_rollout_steps=150,
                 param_noise=param_noise, batch_size=128, action_noise=action_noise, random_exploration=0.05,
                 normalize_observations=True, tensorboard_log="./ddpg_dvrk_tensorboard/", observation_range=(-1.5, 1.5))

    model.pretrain(dataset, n_epochs=1000)
    model.save("./gail_robot_env")

    # model.learn(total_timesteps=4000000, log_interval=100,
    #             callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))

    # As an option, you can train the RL agent
    # model.learn(int(1e5))


def load_model(eval_env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = DDPG.load('./gail_robot_env', env=eval_env)
    reward_sum = 0.0

    # obs = eval_env.reset()
    for _ in range(20):
        obs = eval_env.reset()
        for _ in range(1000):
            action, _ = model.predict(obs)
            obs, reward, done, _ = eval_env.step(action)
            reward_sum += reward
            print(obs[0:3], eval_env.goal, reward)
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