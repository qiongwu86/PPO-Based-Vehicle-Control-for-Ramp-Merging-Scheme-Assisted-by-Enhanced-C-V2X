import numpy as np
import torch.nn as nn
import torch.nn.functional as F

import buffer
import vehicle
import torch


class cacc_engine:
    def __init__(self):
        pass

    def generate_action(self, state):
        s, v, acc, exist, s_, v_ = state
        if exist:
            desSpacing = vehicle.Vehicle.headway_time * v
            actSpacing = s_ - s
            spacingErr = actSpacing - desSpacing
            speedErr = v_ - v
            spacingErr1 = speedErr + vehicle.Vehicle.headway_time * acc
            new_speed = self._generate_new_speed(v, spacingErr, spacingErr1, speedErr)
        else:
            new_speed = vehicle.Vehicle.expect_speed
        acc = np.clip((new_speed - v) / 0.1, vehicle.Vehicle.acc_min, vehicle.Vehicle.acc_max)
        return np.array([acc, 0.0])

    @staticmethod
    def _generate_new_speed(v, spacingErr, spacingErr1, speedErr):
        if 0 < spacingErr < 0.2 and speedErr < 0.1:
            return v + 0.45 * spacingErr + 0.125 * spacingErr1
        elif spacingErr < 0:
            return v + 1.0 * spacingErr + 0.05 * spacingErr1
        else:
            return v + 0.45 * spacingErr + 0.005 * spacingErr1


class rl_engine:
    def __init__(self, state_dim):
        self.buffer = buffer.MainBuffer()

        self.actor = Actor(state_dim)
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=1e-4)

        self.critic = Critic(state_dim)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=1e-4)

        self.action_scale = torch.tensor([vehicle.Vehicle.acc_max - vehicle.Vehicle.acc_min,
                                          vehicle.Vehicle.steer_max - vehicle.Vehicle.steer_min])
        self.action_mean = torch.tensor([0.5*(vehicle.Vehicle.acc_max+vehicle.Vehicle.acc_min),
                                         0.5*(vehicle.Vehicle.steer_max+vehicle.Vehicle.steer_min)])

        self.clip = 0.5

    def generate_action(self, state):
        state = torch.from_numpy(state).clone().view((1, -1))
        alpha, beta = self.actor(state)
        beta_dist = torch.distributions.Beta(alpha[0], beta[0])
        action = beta_dist.sample()
        log_prob = torch.sum(beta_dist.log_prob(action), 0, keepdim=True)
        action = self._scale_action(action)
        return action.clone().detach().numpy(), log_prob.clone().detach().numpy()
        # return np.array([0.0, 0.0]), np.array([1.0])

    def generate_action_for_test(self, state):
        state = torch.from_numpy(state).clone().view((1, -1))
        alpha, beta = self.actor(state)
        mode = alpha[0] / (alpha[0] + beta[0])
        action = self._scale_action(mode)
        return action.clone().detach().numpy()

    def _scale_action(self, beta_dist_sample):
        return (beta_dist_sample - 0.5) * self.action_scale + self.action_mean

    def _rescale_action(self, action):
        return (action - self.action_mean) / self.action_scale + 0.5

    # def add_episode_data(self, episode_data):
    #     pass

    def train(self, times_per_buffer, batch_size):
        print("training...")
        actor_losses = []
        critic_losses = []
        for t in range(times_per_buffer):
            print("\r train {0}/{1}".format(t+1, times_per_buffer), end="")
            mini_batch = self.buffer.get_mini_batch(batch_size)
            s = torch.from_numpy(mini_batch[0]).detach()
            a = self._rescale_action(torch.from_numpy(mini_batch[1]).detach())
            r = torch.from_numpy(mini_batch[2]).detach()
            s_ = torch.from_numpy(mini_batch[3]).detach()
            d_r = torch.from_numpy(mini_batch[4]).detach()
            l_p_old = torch.from_numpy(mini_batch[5]).detach()

            current_state_value = self.critic(s)
            critic_loss = torch.mean(torch.pow(d_r - current_state_value, 2))
            self.critic_opt.zero_grad()
            critic_loss.backward()
            self.critic_opt.step()
            critic_losses.append(critic_loss.detach())

            alpha, beta = self.actor(s)
            beta_dist = torch.distributions.Beta(alpha, beta)
            l_p = beta_dist.log_prob(a).sum(1, keepdim=True)

            ratios = torch.exp(l_p - l_p_old)
            adv = torch.detach(d_r - current_state_value)
            surr1 = ratios * adv
            surr2 = torch.clamp(ratios, 1-self.clip, 1+self.clip) * adv
            actor_loss = -torch.min(surr1, surr2).mean()
            self.actor_opt.zero_grad()
            actor_loss.backward()
            self.actor_opt.step()
            actor_losses.append(actor_loss.detach())
        print("\n train complete")
        return np.mean(actor_losses), np.mean(critic_losses)

    def save_model(self, name):
        torch.save(self.actor.state_dict(), "./model_param/" + name + "_actor_param.pkl")
        torch.save(self.critic.state_dict(), "./model_param/" + name + "_critic_param.pkl")


class Actor(nn.Module):
    def __init__(self, state_dim):
        super(Actor, self).__init__()
        self.state_dim = state_dim
        self.layer1 = torch.nn.Linear(state_dim, 256)
        self.layer2 = torch.nn.Linear(256, 256)

        self.alpha1 = torch.nn.Linear(256, 256)
        self.alpha2 = torch.nn.Linear(256, 2, bias=False)

        self.beta1 = torch.nn.Linear(256, 256)
        self.beta2 = torch.nn.Linear(256, 2, bias=False)

    def forward(self, state):
        state = self.pre_input(state)
        state = F.leaky_relu(self.layer1(state))
        state = F.leaky_relu(self.layer2(state))

        alpha = F.leaky_relu(self.alpha1(state))
        alpha = F.softplus(self.alpha2(alpha)) + 1.0

        beta = F.leaky_relu(self.beta1(state))
        beta = F.softplus(self.beta2(beta)) + 1.0

        return alpha, beta

    @staticmethod
    def pre_input(state):
        state /= torch.tensor([175, 3.75, 3.75, 20, 1, 20, 1, 20, 1])
        return state


class Critic(nn.Module):
    def __init__(self, state_dim):
        super(Critic, self).__init__()
        self.layer1 = nn.Linear(state_dim, 256)
        self.layer2 = nn.Linear(256, 256)
        self.layer3 = nn.Linear(256, 1)

    def forward(self, state):
        state = self.pre_input(state)
        state = F.leaky_relu((self.layer1(state)))
        state = F.leaky_relu((self.layer2(state)))
        value = self.layer3(state)
        return value

    @staticmethod
    def pre_input(state):
        state /= torch.tensor([175, 3.75, 3.75, 20, 1, 20, 1, 20, 1])
        return state

