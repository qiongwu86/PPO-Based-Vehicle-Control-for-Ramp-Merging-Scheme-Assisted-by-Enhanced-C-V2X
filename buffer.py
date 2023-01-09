import random

import numpy as np


class MainBuffer:
    def __init__(self):
        self.total_length = 500
        self.state = np.zeros([self.total_length, 9], dtype=np.float32)
        self.action = np.zeros([self.total_length, 2], dtype=np.float32)
        self.reward = np.zeros([self.total_length, 1], dtype=np.float32)
        self.discount_return = np.zeros([self.total_length, 1], dtype=np.float32)
        self.state_ = np.zeros([self.total_length, 9], dtype=np.float32)
        self.log_prob = np.zeros([self.total_length, 1], dtype=np.float32)
        self.gamma = 0.99
        self.ptr = 0

    def add_episode_data(self, episode_data):
        episode_size = len(episode_data[0])
        self.state[self.ptr: self.ptr + episode_size] += episode_data[0]
        self.action[self.ptr: self.ptr + episode_size] += episode_data[1]
        self.reward[self.ptr: self.ptr + episode_size] += episode_data[2]
        self.state_[self.ptr: self.ptr + episode_size] += episode_data[3]
        self.discount_return[self.ptr: self.ptr + episode_size] += episode_data[4]
        self.log_prob[self.ptr: self.ptr + episode_size] += episode_data[5]
        self.ptr += episode_size

    def full(self):
        return self.ptr > 400

    def get_mini_batch(self, batch_size):
        sample_index = random.sample(range(self.ptr), batch_size)
        return tuple((self.state[sample_index].copy(),
                      self.action[sample_index].copy(),
                      self.reward[sample_index].copy(),
                      self.state_[sample_index].copy(),
                      self.discount_return[sample_index].copy(),
                      self.log_prob[sample_index].copy()))

    def clear(self):
        self.state.fill(0.0)
        self.action.fill(0.0)
        self.reward.fill(0.0)
        self.state_.fill(0.0)
        self.discount_return.fill(0.0)
        self.log_prob.fill(0.0)
        self.ptr = 0


class SubBuffer:
    def __init__(self):
        self.total_length = 2000
        self.state = np.zeros([self.total_length, 9], dtype=np.float32)
        self.action = np.zeros([self.total_length, 2], dtype=np.float32)
        self.reward = np.zeros([self.total_length, 1], dtype=np.float32)
        self.discount_return = np.zeros([self.total_length, 1], dtype=np.float32)
        self.state_ = np.zeros([self.total_length, 9], dtype=np.float32)
        self.log_prob = np.zeros([self.total_length, 1], dtype=np.float32)
        self.gamma = 0.99
        self.ptr = 0

    def input(self, data_in_tuple):
        s, a, r, s_, log_prob = data_in_tuple
        if self.ptr is not self.total_length:
            self.state[self.ptr] += s
            self.action[self.ptr] += a
            self.reward[self.ptr] += r
            self.state_[self.ptr] += s_
            self.log_prob[self.ptr] += log_prob
            self.ptr += 1

        else:
            print("sub buffer is full")
            exit(0)

    def calculate_return(self):
        for i in reversed(range(self.ptr)):
            self.discount_return[i] = self.reward[i] + self.gamma*self.discount_return[i+1]

        return tuple((self.state[:self.ptr].copy(),
                      self.action[:self.ptr].copy(),
                      self.reward[:self.ptr].copy(),
                      self.state_[:self.ptr].copy(),
                      self.discount_return[:self.ptr].copy(),
                      self.log_prob[:self.ptr].copy()))

    def size(self):
        return self.ptr


