import time
import os.path as osp
import gym
import tensorflow as tf
import numpy as np
from baselines.common.vec_env import VecFrameStack, VecNormalize, VecEnv
# from baselines.common.tf_util import get_session
from baselines import logger
from importlib import import_module
from ambf_comm import AmbfEnv


try:
    from mpi4py import MPI
except ImportError:
    MPI = None


def train(env, num_timesteps, alg):

    total_timesteps = int(num_timesteps)

    learn = import_module('.'.join(['baselines', alg, None]))

    model = learn(
        env=env,
        total_timesteps=total_timesteps,
        network='mlp'
    )

    return model, env


def main(env):

    log_path = '/home/vignesh/ambf/ambf_ros_modules/ambf_comm/scripts/RL/ddpg_dvrk_tensorboard/'
    save_path = '/home/vignesh/ambf/ambf_ros_modules/ambf_comm/scripts/RL/ddpg_dvrk_tensorboard/'

    num_timesteps = 4000000
    alg = 'her'
    model, env = train(env, num_timesteps, alg)

    if save_path is not None:
        save_path = osp.expanduser(save_path)
        model.save(save_path)
    play = False
    if play:
        logger.log("Running trained model")
        obs = env.reset()

        state = model.initial_state if hasattr(model, 'initial_state') else None
        dones = np.zeros((1,))

        episode_rew = np.zeros(env.num_envs) if isinstance(env, VecEnv) else np.zeros(1)
        while True:
            if state is not None:
                actions, _, state, _ = model.step(obs, S=state, M=dones)
            else:
                actions, _, _, _ = model.step(obs)

            obs, rew, done, _ = env.step(actions)
            episode_rew += rew
            env.render()
            done_any = done.any() if isinstance(done, np.ndarray) else done
            if done_any:
                for i in np.nonzero(done)[0]:
                    print('episode_rew={}'.format(episode_rew[i]))
                    episode_rew[i] = 0

    env.close()

    return model


if __name__ == '__main__':
    ENV_NAME = 'psm/baselink'
    # Training
    ambf_env = AmbfEnv()
    time.sleep(5)
    ambf_env.make(ENV_NAME)
    ambf_env.reset()
    main(env=ambf_env)
    main(ambf_env)
