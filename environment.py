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

        self.id_count = 0

        self.main_road_spawn_count = 0
        self.merge_road_spawn_count = 0

    def _env_init(self):
        self.id_count -= self.id_count
        Vehicle.all_vehicle.clear()
        self.main_road_spawn_count = random.randint(10, 15) + 5
        self.merge_road_spawn_count = random.randint(10, 15) + 5

    def step(self):
        self.rl_engine.buffer.clear()

        print("collecting data for train...")
        total_episodes = 0
        while not self.rl_engine.buffer.full():
            self._env_init()

            main_veh_num, merge_veh_num = random.sample(Environment.vehicle_random_num, 2)

            start_point = random.uniform(0, 5)
            for i in range(main_veh_num):
                v = Vehicle(i, Mode.MAIN_CACC)
                v.x = -(375 + start_point)
                start_point += 200 / (main_veh_num - 1) + random.uniform(-3, 3)

            start_point = random.uniform(0, 5)
            for i in range(main_veh_num, main_veh_num + merge_veh_num):
                v = Vehicle(i, Mode.MERGE_CACC)
                v.x = -(375 + start_point)
                start_point += 200 / (merge_veh_num - 1) + random.uniform(-3, 3)

            while len(Vehicle.all_vehicle) != 0:
                vehicle_to_state = dict()
                for v in Vehicle.all_vehicle:
                    vehicle_to_state[v] = v.get_state()

                vehicle_to_action = dict()
                vehicle_to_log_porb = dict()
                for v in Vehicle.all_vehicle:
                    if v.mode is Mode.MERGE_RL:
                        vehicle_to_action[v], vehicle_to_log_porb[v] \
                            = self.rl_engine.generate_action(vehicle_to_state[v])
                    else:
                        vehicle_to_action[v] = self.cacc_engine.generate_action(vehicle_to_state[v])

                for v in Vehicle.all_vehicle:
                    v.input_action(vehicle_to_action[v])

                vehicle_to_state_ = dict()
                for v in Vehicle.all_vehicle:
                    if v.mode is Mode.MERGE_RL:
                        vehicle_to_state_[v] = v.get_state()

                vehicle_to_reward = dict()
                for v in Vehicle.all_vehicle:
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
                            # max_x = -575
                            # for v_ in Vehicle.all_vehicle:
                            #     if v_.x > max_x:
                            #         max_x = v_.x
                            total_episodes += 1
                            self.rl_engine.buffer.add_episode_data(v.buffer.calculate_return())
                            print("\r buffer size = {0:5.0f}, total {1:3.0f} episodes.".
                                  format(self.rl_engine.buffer.ptr, total_episodes), end="")
                            # print(", max x:", max_x, end="")

                del_vehicles = []
                for v in Vehicle.all_vehicle:
                    if v.check_and_if_delete_self():
                        del_vehicles.append(v)
                for v in del_vehicles:
                    Vehicle.all_vehicle.remove(v)

                for v in Vehicle.all_vehicle:
                    v.change_mode()

        print("\n buffer is full")

    def _add_new(self):
        self.main_road_spawn_count -= 1
        self.merge_road_spawn_count -= 1
        if self.main_road_spawn_count == 0:
            self.main_road_spawn_count += random.randint(10, 15) + 5
            Vehicle(self.id_count, Mode.MAIN_CACC)
            self.id_count += 1
        if self.merge_road_spawn_count == 0:
            self.merge_road_spawn_count += random.randint(10, 15) + 5
            Vehicle(self.id_count, Mode.MERGE_CACC)
            self.id_count += 1

    @staticmethod
    def _calculate_reward(vehicle, state, action, state_):
        if vehicle.coll_with_edge() or vehicle.coll_with_other() or (state_[3] <= 5.0):
            return np.array(
                [0.0 +
                 1.0 * state_[0] -
                 (1) * 5 *
                 ((abs(state_[1] / 4.625) if state_[1] < 0 else abs(state_[1] / 0.875)) +
                  (abs(state_[2] / 4.625) if state_[2] < 0 else abs(state_[2] / 0.875)))]), True
        if vehicle.x > 0:
            return np.array([300.0]), True
        last_body_angle = state[4]
        x, y_rear, y_front, speed, body_angle = state_[0: 5]
        prev_dist, prev_delta_speed = state_[5: 7]
        foll_dist, foll_delta_speed = state_[7: 9]

        x_reward = -abs(x / 175) * 0
        y_reward = -(1 - abs(x / 175)) * \
                   ((pow(y_rear / 4.625, 2) if y_rear < 0 else pow(y_rear / 0.875, 2))
                    + (pow(y_front / 4.625, 2) if y_front < 0 else pow(y_front / 0.875, 2)))
        speed_reward = (math.exp(-abs(speed - Vehicle.expect_speed)) - 1) * 0.1
        body_angle_reward = -3 * pow(body_angle / math.pi, 2) \
                            - abs(body_angle - last_body_angle) * 7

        prev_dist_diff = prev_dist - Vehicle.headway_time * vehicle.calculate_longitude_speed()
        prev_reward = 5 * (np.clip(-pow(prev_dist_diff / 5, 2), -1, 0)
                           if prev_dist_diff < 0
                           else (np.exp(-prev_dist_diff) - 1)) + (math.exp(-abs(prev_delta_speed)) - 1.0) * 0.1

        foll_dist_diff = foll_dist - Vehicle.headway_time * (vehicle.calculate_longitude_speed() - foll_delta_speed)
        foll_reward = 5 * (np.clip(-pow(foll_dist_diff / 5, 2), -1, 0)
                           if foll_dist_diff < 0
                           else (np.exp(-foll_dist_diff) - 1)) + (math.exp(-abs(foll_delta_speed)) - 1.0) * 0.1

        action_reward = -pow(action[0] / (0.5 * (Vehicle.acc_max - Vehicle.acc_min)), 2) \
                        - pow(action[1] / (0.5 * (Vehicle.steer_max - Vehicle.steer_min)), 2)

        return np.array([(x_reward + y_reward +
                          speed_reward + body_angle_reward +
                          prev_reward + foll_reward +
                          action_reward * 2) / 6], dtype=np.float32), \
               False

    def draw_trace(self, name):
        self._env_init()

        main_veh_num, merge_veh_num = random.sample(Environment.vehicle_random_num, 2)

        start_point = random.uniform(0, 5)
        for i in range(main_veh_num):
            v = Vehicle(i, Mode.MAIN_CACC)
            v.x = -(375 + start_point)
            start_point += 200 / (main_veh_num - 1) + random.uniform(-3, 3)

        start_point = random.uniform(0, 5)
        for i in range(main_veh_num, main_veh_num + merge_veh_num):
            v = Vehicle(i, Mode.MERGE_CACC)
            v.x = -(375 + start_point)
            start_point += 200 / (merge_veh_num - 1) + random.uniform(-3, 3)

        all_data = list()
        while len(Vehicle.all_vehicle) != 0:
            vehicle_to_state = dict()
            for v in Vehicle.all_vehicle:
                vehicle_to_state[v] = v.get_state()

            one_time_data = list()
            for v in Vehicle.all_vehicle:
                one_time_data.append(v.state_for_draw())

            all_data.append(one_time_data)

            vehicle_to_action = dict()
            for v in Vehicle.all_vehicle:
                if v.mode is Mode.MERGE_RL:
                    vehicle_to_action[v] = self.rl_engine.generate_action_for_test(vehicle_to_state[v])
                else:
                    vehicle_to_action[v] = self.cacc_engine.generate_action(vehicle_to_state[v])

            for v in Vehicle.all_vehicle:
                v.input_action(vehicle_to_action[v])

            del_vehicles = []
            for v in Vehicle.all_vehicle:
                if v.check_and_if_delete_self():
                    del_vehicles.append(v)
            for v in del_vehicles:
                Vehicle.all_vehicle.remove(v)

            for v in Vehicle.all_vehicle:
                v.change_mode()

        print("\r collect data done")

        fig = plt.figure()
        ax = plt.axes()
        ax.set_aspect(1)

        plt.xlim(-175, 0)
        plt.ylim(-10, 10)

        rect = plt.Rectangle((-575, -60), 675, 100, color='g')
        ax.add_patch(rect)

        utilities.draw_road()

        all_rect = dict()
        first_data = all_data[0]
        for one_vehicle in first_data:
            v_id = one_vehicle[0]
            all_rect[v_id] = plt.Rectangle((one_vehicle[2], one_vehicle[3]), 4.5, 2.0, color='r')
            ax.add_patch(all_rect[v_id])

        anime = FuncAnimation(fig=fig, func=partial(utilities.update,
                                                    obj_dict=all_rect,
                                                    obj_data=all_data), frames=len(all_data), interval=1000 / 60)

        anime.save("./gifs/demo_" + name + ".gif")
        plt.close()
