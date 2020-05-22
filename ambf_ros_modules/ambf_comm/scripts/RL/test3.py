import numpy as np


actions = np.load('/home/vignesh/Downloads/expert_cartpole/actions.npy')
ep_ret = np.load('/home/vignesh/Downloads/expert_cartpole/episode_returns.npy')
epi_start = np.load('/home/vignesh/Downloads/expert_cartpole/episode_starts.npy')
obs = np.load('/home/vignesh/Downloads/expert_cartpole/obs.npy')
rewards = np.load('/home/vignesh/Downloads/expert_cartpole/rewards.npy')
# actions = np.load('/home/vignesh/Downloads/expert_pendulum/actions.npy')
# ep_ret = np.load('/home/vignesh/Downloads/expert_pendulum/episode_returns.npy')
# epi_start = np.load('/home/vignesh/Downloads/expert_pendulum/episode_starts.npy')
# obs = np.load('/home/vignesh/Downloads/expert_pendulum/obs.npy')
# rewards = np.load('/home/vignesh/Downloads/expert_pendulum/rewards.npy')
print(actions.shape, actions[0], type(actions))
print(ep_ret.shape, ep_ret[0], type(ep_ret))
print(epi_start.shape, epi_start[0], type(epi_start))
print(obs.shape, obs[0], type(obs))
print(rewards.shape, rewards[0], type(rewards))
# for i in range(len(epi_start)):
    # if epi_start[i] != True:
        # print(epi_start[i], i)
        # print(ep_ret[i], rewards[i])
# dic = {'actions': np.array([1, 2, 3]), 'rewards': np.array([4, 5, 6])}
# print(dic)
# np.savez('model.npz', **dic)
# print(output.reshape((11, 3)))
# np.savetxt('/home/vignesh/Thesis_Suture_data/trial2/SD/test.csv', output.reshape((11, 3)), delimiter=',')
actions2 = np.load('/home/vignesh/Thesis_Suture_data/trial2/ambf_data/actions.npy')
episode_returns2 = np.load('/home/vignesh/Thesis_Suture_data/trial2/ambf_data/episode_returns.npy')
episode_starts2 = np.load('/home/vignesh/Thesis_Suture_data/trial2/ambf_data/episode_starts.npy')
observations2 = np.load('/home/vignesh/Thesis_Suture_data/trial2/ambf_data/obs.npy')
rewards2 = np.load('/home/vignesh/Thesis_Suture_data/trial2/ambf_data/rewards.npy')
file_dir = "/home/vignesh/Thesis_Suture_data/trial2/ambf_data/"
print(actions2.shape, actions2[0], type(actions2))
print(episode_returns2.shape, episode_returns2[0], type(episode_returns2))
print(episode_starts2.shape, episode_starts2[0], type(episode_starts2))
print(observations2.shape, observations2[0], type(observations2))
print(rewards2.shape, rewards2[0], type(rewards2))