import time
import itertools
from collections import defaultdict
import numpy as np

import rllab.misc.logger as logger

from sandbox.gkahn.gcg.utils.utils import timeit

class RNNCriticReplayPool(object):

    def __init__(self, env_spec, env_horizon, N, gamma, size, obs_history_len, sampling_method,
                 save_rollouts=False, save_rollouts_observations=True, save_env_infos=False, replay_pool_params={}):
        """
        :param env_spec: for observation/action dimensions
        :param N: horizon length
        :param gamma: discount factor
        :param size: size of pool
        :param obs_history_len: how many previous obs to include when sampling? (= 1 is only current observation)
        :param sampling_method: how to sample the replay pool
        :param save_rollouts: for debugging
        """
        self._env_spec = env_spec
        self._env_horizon = env_horizon
        self._N = N
        self._gamma = gamma
        self._size = int(size)
        self._obs_history_len = obs_history_len
        self._sampling_method = sampling_method
        self._save_rollouts = save_rollouts
        self._save_rollouts_observations = save_rollouts_observations
        self._save_env_infos = save_env_infos
        self._replay_pool_params = replay_pool_params # TODO: hack

        ### buffer
        obs_shape = self._env_spec.observation_space.shape
        obs_dim = self._env_spec.observation_space.flat_dim
        action_dim = self._env_spec.action_space.flat_dim
        self._steps = np.empty((self._size,), dtype=np.int32)
        self._observations = np.empty((self._size, obs_dim), dtype=np.uint8 if self.obs_is_im else np.float64)
        self._actions = np.nan * np.ones((self._size, action_dim), dtype=np.float32)
        self._rewards = np.nan * np.ones((self._size,), dtype=np.float32)
        self._dones = np.ones((self._size,), dtype=bool) # initialize as all done
        self._env_infos = np.empty((self._size,), dtype=np.object)
        self._est_values = np.nan * np.ones((self._size,), dtype=np.float32)
        self._values = np.nan * np.ones((self._size,), dtype=np.float32)
        self._logprobs = np.nan * np.ones((self._size,), dtype=np.float32)
        self._sampling_indices = np.zeros((self._size,), dtype=bool)
        self._index = 0
        self._curr_size = 0

        ### keep track of statistics
        self._stats = defaultdict(int)
        if self.obs_is_im:
            self._obs_mean = (0.5 * 255) * np.ones((1, obs_dim))
            self._obs_orth = np.ones(obs_dim) / 255.

        ### logging
        self._last_done_index = 0
        self._log_stats = defaultdict(list)
        self._log_paths = []
        self._last_get_log_stats_time = None

    def __len__(self):
        return self._curr_size

    def _get_indices(self, start, end):
        if start <= end:
            return list(range(start, end))
        elif start > end:
            return list(range(start, len(self))) + list(range(end))

    def _get_prev_indices(self, end, length):
        if end - (length - 1) >= 0:
            preclipped_indices = list(range(end - length + 1, end + 1))
        else:
            if len(self) == self._size:
                preclipped_indices = list(range(len(self) - (length - 1 - end), len(self))) + list(range(0, end+1))
            else:
                preclipped_indices = list(range(0, end+1))

        # if len(preclipped_indices) == 0:
        #     import IPython; IPython.embed()
        indices = [preclipped_indices[-1]]
        for index in preclipped_indices[-2::-1]:
            if self._dones[index]:
                break
            indices.insert(0, index)
        # if np.any(np.array(indices) > len(self)):
        #     import IPython; IPython.embed()
        return indices

    @property
    def obs_is_im(self):
        return len(self._env_spec.observation_space.shape) > 1

    ##################
    ### Statistics ###
    ##################

    @property
    def statistics(self):
        stats = dict()
        for name, value, is_im in (('observations', self._observations[:len(self)], self.obs_is_im),
                                   ('actions', self._actions[:len(self)], False),
                                   ('rewards', self._rewards[:len(self)], False)):
            if not is_im:
                stats[name + '_mean'] = np.mean(value, axis=0)
                if np.shape(self._stats[name + '_mean']) is tuple():
                    stats[name + '_mean'] = np.array([stats[name + '_mean']])
                stats[name + '_cov'] = np.cov(value.T)
                if np.shape(stats[name + '_cov']) is tuple():
                    stats[name + '_cov'] = np.array([[stats[name + '_cov']]])
                orth, eigs, _ = np.linalg.svd(stats[name + '_cov'])
                stats[name + '_orth'] = orth / np.sqrt(eigs + 1e-5)
            else:
                assert(value.dtype == np.uint8)
                stats[name + '_mean'] = self._obs_mean
                stats[name + '_orth'] = self._obs_orth

        return stats

    @staticmethod
    def statistics_pools(replay_pools):
        pool_stats = [replay_pool.statistics for replay_pool in replay_pools]
        pool_lens = np.array([len(replay_pool) for replay_pool in replay_pools]).astype(float)
        pool_ratios = pool_lens / pool_lens.sum()

        stats = defaultdict(int)
        for ratio, pool_stat in zip(pool_ratios, pool_stats):
            for k, v in [(k, v) for (k, v) in pool_stat.items() if 'mean' in k or 'cov' in k]:
                stats[k] += ratio * v
        for name in ('observations', 'actions', 'rewards'):
            if name+'_cov' in stats.keys():
                orth, eigs, _ = np.linalg.svd(stats[name+'_cov'])
                stats[name+'_orth'] = orth / np.sqrt(eigs + 1e-5)
            else:
                stats[name+'_orth'] = pool_stats[0][name+'_orth']

        return stats

    ###################
    ### Add to pool ###
    ###################

    def store_observation(self, step, observation):
        assert (observation.dtype == self._observations.dtype)

        self._steps[self._index] = step
        self._observations[self._index, :] = self._env_spec.observation_space.flatten(observation)

    def _encode_observation(self, index):
        """ Encodes observation starting at index by concatenating obs_history_len previous """
        indices = self._get_indices(index - self._obs_history_len + 1, index + 1)  # plus 1 b/c inclusive
        observations = self._observations[indices]
        dones = self._dones[indices]

        encountered_done = False
        for i in range(len(dones) - 2, -1, -1):  # skip the most current frame since don't know if it's done
            encountered_done = encountered_done or dones[i]
            if encountered_done:
                observations[i, ...] = 0.

        return observations

    def encode_recent_observation(self):
        return self._encode_observation(self._index)

    def store_effect(self, action, reward, done, env_info, est_value, logprob, flatten_action=True, update_log_stats=True):
        self._actions[self._index, :] = self._env_spec.action_space.flatten(action) if flatten_action else action
        self._rewards[self._index] = reward
        self._dones[self._index] = done
        self._env_infos[self._index] = env_info if self._save_env_infos else None
        self._est_values[self._index] = est_value
        self._logprobs[self._index] = logprob
        if self._sampling_method == 'uniform':
            pass
        elif self._sampling_method == 'nonzero':
            curr_start_indices = self._get_prev_indices(self._index, self._N)
            if reward != 0:
                self._sampling_indices[curr_start_indices] = True
            elif len(self) > self._N:
                prev_start_indices = self._get_prev_indices(self._index - 1, self._N)
                if len(prev_start_indices) > 0 and \
                   np.all(self._rewards[prev_start_indices] == 0):
                    self._sampling_indices[prev_start_indices[0]] = False
        elif self._sampling_method == 'terminal':
            start_indices = self._get_prev_indices(self._index, self._N)
            if done:
                if len(self._get_indices(self._last_done_index, self._index)) == self._env_horizon - 1:
                    self._sampling_indices[start_indices] = False
                else:
                    self._sampling_indices[start_indices] = True
            else:
                self._sampling_indices[start_indices[0]] = False
        else:
            raise NotImplementedError
        self._index = (self._index + 1) % self._size
        self._curr_size = max(self._curr_size, self._index)

        ### compute values
        if done:
            indices = self._get_indices(self._last_done_index, self._index)
            rewards = self._rewards[indices]

            trace = 0
            values = []
            for r in rewards[::-1]:
                trace = r + self._gamma * trace
                values.insert(0, trace)
            self._values[indices] = values

        ### update log stats
        if update_log_stats and done:
            self._update_log_stats()

    def store_rollout(self, start_step, rollout):
        """ Directly store rollout (e.g. if loading in offpolicy data) """
        r_len = len(rollout['dones'])
        # update size first b/c indices depend on it
        if self._index + r_len > self._size:
            self._curr_size = self._size
        else:
            self._curr_size = max(self._curr_size, self._index + r_len)
        indices = self._get_indices(self._index, (self._index + r_len) % self._size)
        self._steps[indices] = list(range(start_step, start_step + r_len))
        self._observations[indices, :] = rollout['observations']
        self._actions[indices, :] = rollout['actions']
        self._rewards[indices] = rollout['rewards']
        self._dones[indices] = rollout['dones']
        self._logprobs[indices] = rollout['logprobs']
        # np.copyto(self._steps[indices], list(range(start_step, start_step + r_len)))
        # np.copyto(self._observations[indices, :], rollout['observations'])
        # np.copyto(self._actions[indices, :], rollout['actions'])
        # np.copyto(self._rewards[indices], rollout['rewards'])
        # np.copyto(self._dones[indices], rollout['dones'])
        self._index = (self._index + r_len) % self._size

        self._last_done_index = self._index

    ########################
    ### Sample from pool ###
    ########################

    def can_sample(self):
        return len(self) > self._obs_history_len and len(self) > self._N

    def _sample_start_indices(self, batch_size, only_completed_episodes):
        start_indices = []
        false_indices = self._get_indices(self._index - self._obs_history_len, self._index) + \
                        self._get_indices(self._index, self._index + self._N)
        if only_completed_episodes and self._last_done_index != self._index:
            false_indices += self._get_indices(self._last_done_index, self._index)

        if self._sampling_method == 'uniform':
            while len(start_indices) < batch_size:
                start_index = np.random.randint(low=0, high=len(self) - self._N)
                if start_index not in false_indices:
                    start_indices.append(start_index)
        elif self._sampling_method == 'nonzero' or self._sampling_method == 'terminal':
            nonzero_indices = np.nonzero(self._sampling_indices[:len(self) - self._N])[0] # terminal
            zero_indices = np.nonzero(self._sampling_indices[:len(self) - self._N] == 0)[0]
            frac_terminal = self._replay_pool_params['terminal']['frac']

            while len(start_indices) < batch_size:
                if len(nonzero_indices) == 0 or len(zero_indices) == 0:
                    start_index = np.random.randint(low=0, high=len(self) - self._N)
                else:
                    if len(start_indices) < frac_terminal * batch_size:
                        start_index = np.random.choice(nonzero_indices)
                    else:
                        start_index = np.random.choice(zero_indices)

                if start_index not in false_indices:
                    start_indices.append(start_index)
        else:
            raise NotImplementedError

        return start_indices

    def sample(self, batch_size, only_completed_episodes=False):
        """
        :return observations, actions, and rewards of horizon H+1
        """
        if not self.can_sample():
            return None

        steps, observations, actions, rewards, values, dones, logprobs = [], [], [], [], [], [], []

        start_indices = self._sample_start_indices(batch_size, only_completed_episodes)

        for start_index in start_indices:
            indices = self._get_indices(start_index, (start_index + self._N + 1) % self._curr_size)
            steps_i = self._steps[indices]
            observations_i = np.vstack([self._encode_observation(start_index), self._observations[indices[1:]]])
            actions_i = self._actions[indices]
            rewards_i = self._rewards[indices]
            values_i = self._values[indices]
            dones_i = self._dones[indices]
            logprobs_i = self._logprobs[indices]
            if np.any(dones_i[:-1]):
                # H = 3
                # observations = [0 1 2 3]
                # actions = [10 11 12 13]
                # rewards = [20 21 22 23]
                # dones = [False True False False]

                d_idx = np.argmax(dones_i)
                for j in range(d_idx + 1, len(dones_i)):
                    # observations_i[j+self._obs_history_len-1, :] = 0.
                    actions_i[j, :] = self._env_spec.action_space.flatten(self._env_spec.action_space.sample())
                    rewards_i[j] = 0.
                    values_i[j] = 0.
                    dones_i[j] = True

                # observations = [0 1 2 3]
                # actions = [10 11 rand rand]
                # rewards = [20 21 0 0]
                # dones = [False True True True]

            steps.append(np.expand_dims(steps_i, 0))
            observations.append(np.expand_dims(observations_i, 0))
            actions.append(np.expand_dims(actions_i, 0))
            rewards.append(np.expand_dims(rewards_i, 0))
            values.append(np.expand_dims(values_i, 0))
            dones.append(np.expand_dims(dones_i, 0))
            logprobs.append(np.expand_dims(logprobs_i, 0))

        steps = np.vstack(steps)
        observations = np.vstack(observations)
        actions = np.vstack(actions)
        rewards = np.vstack(rewards)
        values = np.vstack(values)
        dones = np.vstack(dones)
        logprobs = np.vstack(logprobs)

        # timeit.start('replay_pool:isfinite')
        # for k, arr in enumerate((observations, actions, rewards, dones)):
        #     if not np.all(np.isfinite(arr)):
        #         raise Exception
        #
        #     assert(np.all(np.isfinite(arr)))
        #     assert(arr.shape[0] == batch_size)
        #     assert(arr.shape[1] == self._N + 1)
        # timeit.stop('replay_pool:isfinite')

        return steps, observations, actions, rewards, values, dones, logprobs

    @staticmethod
    def sample_pools(replay_pools, batch_size, only_completed_episodes=False):
        """ Sample from replay pools (treating them as one big replay pool) """
        if not np.any([replay_pool.can_sample() for replay_pool in replay_pools]):
            return None

        steps, observations, actions, rewards, values, dones, logprobs = [], [], [], [], [], [], []

        # calculate ratio of pool sizes
        pool_lens = np.array([replay_pool.can_sample() * len(replay_pool) for replay_pool in replay_pools]).astype(float)
        pool_ratios = pool_lens / pool_lens.sum()
        # how many from each pool
        choices = np.random.choice(range(len(replay_pools)), size=batch_size, p=pool_ratios)
        batch_sizes = np.bincount(choices, minlength=len(replay_pools))
        # sample from each pool
        for i, replay_pool in enumerate(replay_pools):
            if batch_sizes[i] == 0:
                continue

            steps_i, observations_i, actions_i, rewards_i, values_i, dones_i, logprobs_i = \
                replay_pool.sample(batch_sizes[i], only_completed_episodes=only_completed_episodes)
            steps.append(steps_i)
            observations.append(observations_i)
            actions.append(actions_i)
            rewards.append(rewards_i)
            values.append(values_i)
            dones.append(dones_i)
            logprobs.append(logprobs_i)

        steps = np.vstack(steps)
        observations = np.vstack(observations)
        actions = np.vstack(actions)
        rewards = np.vstack(rewards)
        values = np.vstack(values)
        dones = np.vstack(dones)
        logprobs = np.vstack(logprobs)

        for arr in (steps, observations, actions, rewards, values, dones, logprobs):
            assert(len(arr) == batch_size)

        return steps, observations, actions, rewards, values, dones, logprobs

    ###############
    ### Logging ###
    ###############

    def _update_log_stats(self):
        indices = self._get_indices(self._last_done_index, self._index)

        ### update log
        rewards = self._rewards[indices]
        est_values = self._est_values[indices]
        values = self._values[indices]
        self._log_stats['FinalReward'].append(rewards[-1])
        self._log_stats['AvgReward'].append(np.mean(rewards))
        self._log_stats['CumReward'].append(np.sum(rewards))
        self._log_stats['EpisodeLength'].append(len(rewards))
        self._log_stats['EstValuesAvgDiff'].append(np.mean(est_values - values))
        self._log_stats['EstValuesMaxDiff'].append(np.max(est_values - values))
        self._log_stats['EstValuesMinDiff'].append(np.min(est_values - values))

        ## update paths
        if self._save_rollouts:
            self._log_paths.append({
                'steps': self._steps[indices],
                'observations': self._observations[indices] if self._save_rollouts_observations else None,
                'actions': self._actions[indices],
                'rewards': self._rewards[indices],
                'dones': self._dones[indices],
                'env_infos': self._env_infos[indices],
                'est_values': self._est_values[indices],
                'values': self._values[indices],
                'logprobs': self._logprobs[indices]
            })

        self._last_done_index = self._index

    def get_log_stats(self):
        self._log_stats['Time'] = [time.time() - self._last_get_log_stats_time] if self._last_get_log_stats_time else [0.]
        d = self._log_stats
        self._last_get_log_stats_time = time.time()
        self._log_stats = defaultdict(list)
        return d

    def get_recent_paths(self):
        paths = self._log_paths
        self._log_paths = []
        return paths

    @staticmethod
    def log_pools(replay_pools, prefix=''):
        def join(l):
            return list(itertools.chain(*l))
        all_log_stats = [replay_pool.get_log_stats() for replay_pool in replay_pools]
        log_stats = defaultdict(list)
        for k in all_log_stats[0].keys():
            log_stats[k] = join([ls[k] for ls in all_log_stats])
        logger.record_tabular(prefix+'CumRewardMean', np.mean(log_stats['CumReward']))
        logger.record_tabular(prefix+'CumRewardStd', np.std(log_stats['CumReward']))
        logger.record_tabular(prefix+'AvgRewardMean', np.mean(log_stats['AvgReward']))
        logger.record_tabular(prefix+'AvgRewardStd', np.std(log_stats['AvgReward']))
        logger.record_tabular(prefix+'FinalRewardMean', np.mean(log_stats['FinalReward']))
        logger.record_tabular(prefix+'FinalRewardStd', np.std(log_stats['FinalReward']))
        logger.record_tabular(prefix+'EpisodeLengthMean', np.mean(log_stats['EpisodeLength']))
        logger.record_tabular(prefix+'EpisodeLengthStd', np.std(log_stats['EpisodeLength']))
        logger.record_tabular(prefix+'EstValuesAvgDiffMean', np.mean(log_stats['EstValuesAvgDiff']))
        logger.record_tabular(prefix+'EstValuesAvgDiffStd', np.std(log_stats['EstValuesAvgDiff']))
        logger.record_tabular(prefix+'EstValuesMaxDiffMean', np.mean(log_stats['EstValuesMaxDiff']))
        logger.record_tabular(prefix+'EstValuesMaxDiffStd', np.std(log_stats['EstValuesMaxDiff']))
        logger.record_tabular(prefix+'EstValuesMinDiffMean', np.mean(log_stats['EstValuesMinDiff']))
        logger.record_tabular(prefix+'EstValuesMinDiffStd', np.std(log_stats['EstValuesMinDiff']))
        logger.record_tabular(prefix+'NumEpisodes', len(log_stats['EpisodeLength']))
        logger.record_tabular(prefix+'Time', np.mean(log_stats['Time']))

    @staticmethod
    def get_recent_paths_pools(replay_pools):
        return list(itertools.chain(*[rp.get_recent_paths() for rp in replay_pools]))
