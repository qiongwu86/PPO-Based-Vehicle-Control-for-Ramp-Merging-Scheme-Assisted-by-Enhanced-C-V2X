import math
import numpy as np

import utilities
from vehicle import Vehicle, Mode
import random
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial


class Environment:
    vehicle_random_num = [6, 7]

    def __init__(self, cacc_engine, rl_engine):
        self.cacc_engine = cacc_engine
        self.rl_engine = rl_engine

        self.all_vehicles = Vehicle.all_vehicle

        self.id_count = 0

        self.main_road_spawn_count = 0
        self.merge_road_spawn_count = 0

        # self.buffer = None

    def _env_init(self):
        self.id_count -= self.id_count
        self.all_vehicles.clear()
        self.main_road_spawn_count = random.randint(10, 15)
        self.merge_road_spawn_count = random.randint(10, 15)

    def step(self):
        self._env_init()
        self.rl_engine.buffer.clear()

        while not self.rl_engine.buffer.full():
            self._add_new()

            vehicle_to_state = dict()
            for v in self.all_vehicles:
                vehicle_to_state[v] = v.get_state()

            vehicle_to_action = dict()
            vehicle_to_log_porb = dict()
            for v in self.all_vehicles:
                if v.mode is Mode.MERGE_RL:
                    vehicle_to_action[v], vehicle_to_log_porb[v] \
                        = self.rl_engine.generate_action(vehicle_to_state[v])
                else:
                    vehicle_to_action[v] = self.cacc_engine.generate_action(vehicle_to_state[v])

            for v in self.all_vehicles:
                v.input_action(vehicle_to_action[v])

            vehicle_to_state_ = dict()
            for v in self.all_vehicles:
                if v.mode is Mode.MERGE_RL:
                    vehicle_to_state_[v] = v.get_state()

            vehicle_to_reward = dict()
            for v in self.all_vehicles:
                if v.mode is Mode.MERGE_RL:
                    vehicle_to_reward[v], done = self._calculate_reward(v,
                                                                        vehicle_to_state[v],
                                                                        vehicle_to_action[v],
                                                                        vehicle_to_state_[v])
                    v.buffer.input((vehicle_to_state[v],
                                    vehicle_to_action[v],
                                    vehicle_to_reward[v],
                                    vehicle_to_state_[v],
                                    vehicle_to_log_porb[v]))

                    if done:
                        self.rl_engine.buffer.add_episode_data(v.buffer.calculate_return())
                        print("\r buffer size = ", self.rl_engine.buffer.ptr, end="")

            for v in self.all_vehicles:
                v.check_and_delete_self()

            for v in self.all_vehicles:
                v.change_mode()

        print("buffer is full")

    def _add_new(self):
        self.main_road_spawn_count -= 1
        self.merge_road_spawn_count -= 1
        if self.main_road_spawn_count == 0:
            self.main_road_spawn_count += random.randint(10, 15)
            Vehicle(self.id_count, Mode.MAIN_CACC)
            self.id_count += 1
        if self.merge_road_spawn_count == 0:
            self.merge_road_spawn_count += random.randint(10, 15)
            Vehicle(self.id_count, Mode.MERGE_CACC)
            self.id_count += 1

    @staticmethod
    def _calculate_reward(vehicle, state, action, state_):
        if vehicle.coll_with_edge() or vehicle.coll_with_other():
            return np.array([-100.0]), True
        if vehicle.x > 0:
            return np.array([100.0]), True
        last_body_angle = state[4]
        x, y_rear, y_front, speed, body_angle = state_[0: 5]
        prev_dist, prev_delta_speed = state_[5: 7]
        foll_dist, foll_delta_speed = state_[7: 9]

        x_reward = -abs(x / 175)
        y_reward = -(1 - abs(x/175)) * (abs(y_rear) + abs(y_front)) * 0.125
        speed_reward = math.exp(-abs(speed - Vehicle.expect_speed)) - 1
        body_angle_reward = -pow(body_angle / math.pi, 2) \
                            - abs(body_angle - last_body_angle) * 50

        expect_dist_with_prev = Vehicle.headway_time * vehicle.calculate_longitude_speed()
        prev_reward = np.clip(math.exp(prev_dist - expect_dist_with_prev) - 1, -1, 0) + \
                      math.exp(-abs(prev_delta_speed)) - 1.0

        expect_dist_with_foll = Vehicle.headway_time * (vehicle.calculate_longitude_speed() - foll_delta_speed)
        foll_reward = np.clip(math.exp(foll_dist - expect_dist_with_foll) - 1, -1, 0) + \
                      math.exp(-abs(foll_delta_speed)) - 1.0

        action_reward = -pow(action[0], 2) - pow(action[1] / Vehicle.steer_max, 2)

        return np.array([(x_reward + y_reward +
                          speed_reward + body_angle_reward +
                          prev_reward + foll_reward +
                          action_reward*2) / 6], dtype=np.float32), \
            False

    def draw_trace(self, name):
        self._env_init()

        main_veh_num, merge_veh_num = random.sample(Environment.vehicle_random_num, 2)

        start_point = random.uniform(0, 5)
        for i in range(main_veh_num):
            v = Vehicle(i, Mode.MAIN_CACC)
            v.x = -(375 + start_point)
            start_point += 200 / (main_veh_num-1) + random.uniform(-3, 3)

        start_point = random.uniform(0, 5)
        for i in range(main_veh_num, main_veh_num + merge_veh_num):
            v = Vehicle(i, Mode.MERGE_CACC)
            v.x = -(375 + start_point)
            start_point += 200 / (merge_veh_num-1) + random.uniform(-3, 3)

        all_data = list()
        while len(Vehicle.all_vehicle) != 0:
            vehicle_to_state = dict()
            for v in self.all_vehicles:
                vehicle_to_state[v] = v.get_state()

            one_time_data = list()
            for v in self.all_vehicles:
                one_time_data.append(v.state_for_draw())

            all_data.append(one_time_data)

            vehicle_to_action = dict()
            for v in self.all_vehicles:
                if v.mode is Mode.MERGE_RL:
                    vehicle_to_action[v], _ \
                        = self.rl_engine.generate_action(vehicle_to_state[v])
                else:
                    vehicle_to_action[v] = self.cacc_engine.generate_action(vehicle_to_state[v])

            for v in self.all_vehicles:
                v.input_action(vehicle_to_action[v])

            for v in self.all_vehicles:
                v.check_and_delete_self()

            for v in self.all_vehicles:
                v.change_mode()

        print("collect data done")

        fig = plt.figure()
        ax = plt.axes()
        ax.set_aspect(1)

        plt.xlim(-175, 0)
        plt.ylim(-60, 10)

        utilities.draw_road()

        all_rect = dict()
        first_data = all_data[0]
        for one_vehicle in first_data:
            v_id = one_vehicle[0]
            all_rect[v_id] = plt.Rectangle((one_vehicle[2], one_vehicle[3]), 4.5, 2.0)
            ax.add_patch(all_rect[v_id])

        anime = FuncAnimation(fig=fig, func=partial(utilities.update,
                                                    obj_dict=all_rect,
                                                    obj_data=all_data), frames=len(all_data),  interval=1000 / 60)

        anime.save("./gifs/demo_" + name + ".gif")
        plt.close()
