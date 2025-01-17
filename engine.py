import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import math
import buffer
import vehicle
import torch
import road

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
            # new_speed = v
        acc = np.clip((new_speed - v) / 0.1, vehicle.Vehicle.acc_min, vehicle.Vehicle.acc_max)
        return np.array([acc, 0.0])

    @staticmethod
    def _generate_new_speed(v, spacingErr, spacingErr1, speedErr):
        if 0 < spacingErr < 0.2 and speedErr < 0.1:
            return v + 0.45 * spacingErr + 0.125 * spacingErr1
        elif spacingErr < 0:
            return v + 0.45 * spacingErr + 0.05 * spacingErr1
        else:
            return v + 0.005 * spacingErr + 0.05 * spacingErr1


class cacc_TP_engine:
    def __init__(self):
        # 两点控制参数
        self.k_f = 0.5  # 远点比例系数
        self.k_n = 0.05  # 近点比例系数
        self.k_I = 0.02  # 积分比例系数
        self.integral_errors = {}  # 维护每辆车的积分误差 {vehicle_id: integral_error}

    def generate_action(self, state, vehicle_id):
        """
        联合 CACC 和两点算法控制车辆。

        参数：
            state: tuple - 状态信息 (x, y, theta, v, acc, exist, s_, v_)
            centerline: list - 道路中心线坐标 [(x1, y1), (x2, y2), ...]
            far_distance: float - 远点距离

        返回：
            action: np.array - 控制输出 [acceleration, steering_angle]
        """
        # 状态分解
        if vehicle_id not in self.integral_errors:
            self.integral_errors[vehicle_id] = 0  # 初始化积分误差为 0

        centerline = road.get_center_line()
        far_distance = 20
        near_distance = 5
        s, x, y, theta, v, acc, exist, s_, v_ = state
        print('theta', theta)

        # === CACC 速度控制 ===
        if exist:  # 有前车
            des_spacing = vehicle.Vehicle.headway_time * v  # 理想车距
            act_spacing = s_ - s  # 实际车距
            spacing_err = act_spacing - des_spacing  # 距离误差
            speed_err = v_ - v  # 速度误差
            spacing_err1 = speed_err + vehicle.Vehicle.headway_time * acc
            new_speed = self._generate_new_speed(v, spacing_err, spacing_err1, speed_err)
        else:  # 无前车，保持期望速度
            new_speed = vehicle.Vehicle.expect_speed

        # 计算加速度，限制在范围内
        acc = np.clip((new_speed - v) / 0.1, vehicle.Vehicle.acc_min, vehicle.Vehicle.acc_max)

        # === 计算远点和近点 ===
        far_point = self.get_far_point((x, y), centerline, far_distance)  # 计算远点
        near_point = self.calculate_near_point((x, y), centerline, near_distance)  # 计算近点
        print('x,y, near, far', x, y, near_point, far_point)

        # === 两点转向角控制 ===
        steering_angle = self._generate_steering_angle(near_point, far_point, (x, y), theta, vehicle_id)

        # 返回动作
        return np.array([acc, steering_angle])

    @staticmethod
    def _generate_new_speed(v, spacing_err, spacing_err1, speed_err):
        """
        根据 CACC 逻辑生成目标速度。
        """
        if 0 < spacing_err < 0.2 and speed_err < 0.1:
            return v + 0.45 * spacing_err + 0.125 * spacing_err1
        elif spacing_err < 0:
            return v + 0.45 * spacing_err + 0.05 * spacing_err1
        else:
            return v + 0.005 * spacing_err + 0.05 * spacing_err1

    def calculate_near_point(self, vehicle_pos, centerline, near_distance):
        """
        计算近点：车辆前方 5 米处的全局坐标。
        """
        target_x = vehicle_pos[0] + near_distance
        centerline_array = np.array(centerline)  # 转为数组
        distances = np.abs(centerline_array[:, 0] - target_x)  # x 坐标距离
        nearest_index = np.argmin(distances)  # 最近点索引
        return centerline_array[nearest_index]  # 返回最近点的坐标
        # if (-575 < x < -375) or (-175 < x < 100):
        #     direction_global = delta
        # else:
        #     direction_global = delta + 15 * math.pi / 180
        #
        # # 距离为 5 米
        # distance = 5
        # delta_x = distance * np.cos(direction_global)
        # delta_y = distance * np.sin(direction_global)
        #
        # # 计算全局坐标
        # x_near = x + delta_x
        # y_near = y + delta_y
        #
        # return np.array([x_near, y_near])

    def get_far_point(self, vehicle_pos, centerline, far_distance):
        """
        根据车辆位置和道路中心线计算远点的全局坐标。
        """
        target_x = vehicle_pos[0] + far_distance
        centerline_array = np.array(centerline)  # 转为数组
        distances = np.abs(centerline_array[:, 0] - target_x)  # x 坐标距离
        nearest_index = np.argmin(distances)  # 最近点索引
        return centerline_array[nearest_index]  # 返回最近点的坐标

    def _generate_steering_angle(self, near_point, far_point, vehicle_pos, vehicle_theta, vehicle_id):
        """
        使用两点算法计算转向角。
        """
        # 计算远点方向误差
        delta_y_f = np.arctan2(far_point[1] - vehicle_pos[1], far_point[0] - vehicle_pos[0]) - vehicle_theta
        # 计算近点方向误差
        delta_y_n = np.arctan2(near_point[1] - vehicle_pos[1], near_point[0] - vehicle_pos[0]) - vehicle_theta

        # 更新积分误差
        self.integral_errors[vehicle_id] += delta_y_n * vehicle.Vehicle.headway_time  # 假设时间步长为 0.1 秒
        self.integral_errors[vehicle_id] = np.clip(self.integral_errors[vehicle_id], -1, 1)  # 限制积分误差

        # 综合计算转向角
        steering_angle = self.k_f * delta_y_f + self.k_n * delta_y_n + self.k_I * self.integral_errors[vehicle_id]
        steering_angle = np.clip(steering_angle, vehicle.Vehicle.steer_min, vehicle.Vehicle.steer_max)
        print('yf, yn, integral_errors', delta_y_f, delta_y_n, self.integral_errors[vehicle_id])
        return steering_angle


class rl_engine:
    def __init__(self, state_dim):
        self.buffer = buffer.MainBuffer()

        self.actor = Actor(state_dim, False)
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=5e-5)

        self.critic = Critic(state_dim, False)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=5e-5)

        self.action_scale = torch.tensor([vehicle.Vehicle.acc_max - vehicle.Vehicle.acc_min,
                                          vehicle.Vehicle.steer_max - vehicle.Vehicle.steer_min])
        self.action_mean = torch.tensor([0.5*(vehicle.Vehicle.acc_max+vehicle.Vehicle.acc_min),
                                         0.5*(vehicle.Vehicle.steer_max+vehicle.Vehicle.steer_min)])

        self.clip = 0.2

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
            # critic
            current_state_value = self.critic(s)
            critic_loss = torch.mean(torch.pow(d_r - current_state_value, 2))
            self.critic_opt.zero_grad()
            critic_loss.backward()
            self.critic_opt.step()
            critic_losses.append(critic_loss.detach())

        # actor
        for t in range(times_per_buffer):
            print("\r train {0}/{1}".format(t + 1, times_per_buffer), end="")
            mini_batch = self.buffer.get_mini_batch(batch_size)
            s = torch.from_numpy(mini_batch[0]).detach()
            a = self._rescale_action(torch.from_numpy(mini_batch[1]).detach())
            r = torch.from_numpy(mini_batch[2]).detach()
            s_ = torch.from_numpy(mini_batch[3]).detach()
            d_r = torch.from_numpy(mini_batch[4]).detach()
            l_p_old = torch.from_numpy(mini_batch[5]).detach()

            alpha, beta = self.actor(s)
            beta_dist = torch.distributions.Beta(alpha, beta)
            l_p = beta_dist.log_prob(a).sum(1, keepdim=True)
            current_state_value = self.critic(s).detach()
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

    def train2(self, times_per_buffer, batch_size):
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
            # critic
            next_state_value = self.critic(s_)
            current_state_value = self.critic(s)
            critic_loss = torch.mean(torch.pow(r + self.buffer.gamma*next_state_value - current_state_value, 2))
            self.critic_opt.zero_grad()
            critic_loss.backward()
            self.critic_opt.step()
            critic_losses.append(critic_loss.detach())
            # actor
            alpha, beta = self.actor(s)
            beta_dist = torch.distributions.Beta(alpha, beta)
            l_p = beta_dist.log_prob(a).sum(1, keepdim=True)

            ratios = torch.exp(l_p - l_p_old)
            adv = torch.detach(r + self.buffer.gamma*next_state_value - current_state_value)
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

    def prep_train(self):
        self.actor.train()
        self.critic.train()

    def prep_eval(self):
        self.actor.eval()
        self.critic.eval()


class Actor(nn.Module):
    def __init__(self, state_dim, batch_norm=False):
        super(Actor, self).__init__()
        self.state_dim = state_dim
        self.batch_norm = batch_norm

        if not self.batch_norm:
            self.layer1 = torch.nn.Linear(state_dim, 256)
            self.layer2 = torch.nn.Linear(256, 256)

            self.alpha1 = torch.nn.Linear(256, 256)
            self.alpha2 = torch.nn.Linear(256, 2, bias=False)

            self.beta1 = torch.nn.Linear(256, 256)
            self.beta2 = torch.nn.Linear(256, 2, bias=False)
        else:
            self.layer1 = torch.nn.Linear(state_dim, 256, bias=False)
            self.bn1 = torch.nn.BatchNorm1d(256)
            self.layer2 = torch.nn.Linear(256, 256, bias=False)
            self.bn2 = torch.nn.BatchNorm1d(256)

            self.alpha1 = torch.nn.Linear(256, 256, bias=False)
            self.bna1 = torch.nn.BatchNorm1d(256)
            self.alpha2 = torch.nn.Linear(256, 2, bias=False)

            self.beta1 = torch.nn.Linear(256, 256, bias=False)
            self.bnb1 = torch.nn.BatchNorm1d(256)
            self.beta2 = torch.nn.Linear(256, 2, bias=False)

    def forward(self, state):
        if not self.batch_norm:
            state = self.pre_input(state)
            state = F.leaky_relu(self.layer1(state))
            state = F.leaky_relu(self.layer2(state))

            alpha = F.leaky_relu(self.alpha1(state))
            alpha = F.softplus(self.alpha2(alpha)) + 1.0

            beta = F.leaky_relu(self.beta1(state))
            beta = F.softplus(self.beta2(beta)) + 1.0
        else:
            state = self.pre_input(state)
            state = F.leaky_relu(self.bn1(self.layer1(state)))
            state = F.leaky_relu(self.bn2(self.layer2(state)))

            alpha = F.leaky_relu(self.bna1(self.alpha1(state)))
            alpha = F.softplus(self.alpha2(alpha)) + 1.0

            beta = F.leaky_relu(self.bnb1(self.beta1(state)))
            beta = F.softplus(self.beta2(beta)) + 1.0

        return alpha, beta

    @staticmethod
    def pre_input(state):
        state_copy = state.clone().detach()
        state_copy /= torch.tensor([175, 3.75, 3.75, 20, 1, 20, 1, 20, 1])
        return state_copy


class Critic(nn.Module):
    def __init__(self, state_dim, batch_norm=False):
        super(Critic, self).__init__()
        self.batch_norm = batch_norm
        if not self.batch_norm:
            self.layer1 = nn.Linear(state_dim, 256)
            self.layer2 = nn.Linear(256, 256)
            self.layer3 = nn.Linear(256, 1)
        else:
            self.layer1 = nn.Linear(state_dim, 256, bias=False)
            self.bn1 = torch.nn.BatchNorm1d(256)
            self.layer2 = nn.Linear(256, 256, bias=False)
            self.bn2 = torch.nn.BatchNorm1d(256)
            self.layer3 = nn.Linear(256, 1)

    def forward(self, state):
        if not self.batch_norm:
            state = self.pre_input(state)
            state = F.leaky_relu((self.layer1(state)))
            state = F.leaky_relu((self.layer2(state)))
            value = self.layer3(state)
        else:
            state = self.pre_input(state)
            state = F.leaky_relu(self.bn1(self.layer1(state)))
            state = F.leaky_relu(self.bn2(self.layer2(state)))
            value = self.layer3(state)
        return value

    @staticmethod
    def pre_input(state):
        state_copy = state.clone().detach()
        state_copy /= torch.tensor([175, 3.75, 3.75, 20, 1, 20, 1, 20, 1])
        return state_copy

