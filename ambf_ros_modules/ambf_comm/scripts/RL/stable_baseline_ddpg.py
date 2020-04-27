from stable_baselines.ddpg.policies import MlpPolicy
import numpy as np
from stable_baselines import HER, DDPG
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from ambf_comm import AmbfEnv
import time
from stable_baselines.common.callbacks import CheckpointCallback


def main(env):
    # the noise objects for DDPG
    # print("actions space ", np.abs(env.action_space.low), np.abs(env.action_space.high))
    n_actions = env.action_space.shape[0]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

    model = DDPG(MlpPolicy, env, gamma=0.95, verbose=1, nb_train_steps=300, nb_rollout_steps=150,
                 param_noise=param_noise, batch_size=128, action_noise=action_noise, random_exploration=0.05,
                 tensorboard_log="./ddpg_dvrk_tensorboard/", observation_range=(-1.5, 1.5))

    model.learn(total_timesteps=4000000, log_interval=100,
                callback=CheckpointCallback(save_freq=100000, save_path="./ddpg_dvrk_tensorboard/"))
    model.save("./ddpg_robot_env")

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


def load_model(eval_env):

    # WARNING: you must pass an env
    # or wrap your environment with HERGoalEnvWrapper to use the predict method
    model = DDPG.load('./ddpg_robot_env', env=eval_env)
    # obs = eval_env.reset()
    for _ in range(20):
        obs = eval_env.reset()
        for _ in range(1000):
            action, _ = model.predict(obs)
            obs, reward, done, _ = eval_env.step(action)
            print(obs['achieved_goal'][0:3], obs['desired_goal'][0:3], reward)
            if done:
                print("----------------It reached terminal state -------------------")
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



