import torch
import numpy as np
import vehicle
import engine
import random


class Environment:
    def __int__(self, env_param):
        self.cacc_engine = engine.cacc_engine()
        self.rl_engine = engine.rl_engine()

        self.main_road_vehicles = list()
        self.merge_road_vehicles_in_cacc = list()
        self.merge_road_vehicles_in_rl = list()

        self.main_road_spawn_count = 0
        self.merge_road_spawn_count = 0

        self.id_count = 0

        self.buffer = None

    def env_init(self):
        self.id_count = 0

        self.main_road_vehicles.clear()
        self.merge_road_vehicles_in_rl.clear()
        self.merge_road_vehicles_in_cacc.clear()

        self.main_road_spawn_count += random.randint(10, 20)
        self.merge_road_spawn_count += random.randint(10, 20)

    def step(self):
        self._add_new()

        # get actions
        main_road_vehicle_to_action = dict()
        for _vehicle in self.main_road_vehicles:
            engine_input = _vehicle.get_cacc_engine_input()
            action = self.cacc_engine.gen_action(engine_input)
            main_road_vehicle_to_action[_vehicle] = action

        merge_road_vehicle_in_cacc_to_action = dict()
        for _vehicle in self.merge_road_vehicles_in_cacc:
            engine_input = _vehicle.get_cacc_engine_input()
            action = self.cacc_engine.gen_action(engine_input)
            merge_road_vehicle_in_cacc_to_action[_vehicle] = action

        merge_road_vehicle_in_rl_to_action = dict()
        merge_road_vehicle_in_rl_to_rl_state = dict()
        for _vehicle in self.merge_road_vehicles_in_rl:
            engine_input = _vehicle.get_rl_engine_input()
            action = self.rl_engine.gen_action(engine_input)
            merge_road_vehicle_in_rl_to_action[_vehicle] = action

            # rl_state = _vehicle.get_rl_state()
            rl_state = engine_input
            merge_road_vehicle_in_rl_to_rl_state[_vehicle] = rl_state

        # execute actions
        for _vehicle, _action in main_road_vehicle_to_action:
            _vehicle.execute_action(_action)

        for _vehicle, _action in merge_road_vehicle_in_cacc_to_action:
            _vehicle.execute_action(_action)

        for _vehicle, _action in merge_road_vehicle_in_rl_to_action:
            _vehicle.execute_action(_action)

        # get reward and next state and check done
        merge_road_vehicle_in_rl_done = []
        for _vehicle in self.merge_road_vehicles_in_rl:
            rl_state_ = _vehicle.get_rl_state()
            # reward, done = self._calculate_reward(rl_state, action)
            reward, done = self._calculate_reward(rl_state_)
            t = (merge_road_vehicle_in_rl_to_rl_state[_vehicle],
                 merge_road_vehicle_in_rl_to_action[_vehicle],
                 reward,
                 rl_state_)
            _vehicle.input_tuple(t)
            if done:
                merge_road_vehicle_in_rl_done.append(_vehicle)

        # check_done
        main_road_vehicle_done = []
        for _vehicle in self.main_road_vehicles:
            done = _vehicle.check_done()
            if done:
                main_road_vehicle_done.append(_vehicle)

        merge_road_vehicle_in_cacc_done = []
        for _vehicle in self.merge_road_vehicles_in_cacc:
            done = _vehicle.check_done()
            if done:
                merge_road_vehicle_in_cacc_done.append(_vehicle)

        self._del_done_vehicle(main_road_vehicle_done, merge_road_vehicle_in_cacc_done, merge_road_vehicle_in_rl_done)

        self._change_mode()

        return self.buffer.size()

    def _del_done_vehicle(self, main_road_vehicle_done, merge_road_vehicle_in_cacc_done, merge_road_vehicle_in_rl_done):
        for _vehicle in main_road_vehicle_done:
            self.main_road_vehicles.remove(_vehicle)
            del _vehicle
        for _vehicle in merge_road_vehicle_in_cacc_done:
            self.merge_road_vehicles_in_cacc.remove(_vehicle)
            del _vehicle
        for _vehicle in merge_road_vehicle_in_rl_done:
            self.merge_road_vehicles_in_rl.remove(_vehicle)
            self.buffer.input_data(_vehicle.get_data())
            del _vehicle

    def _add_new(self):
        self.main_road_spawn_count -= 1
        self.merge_road_spawn_count -= 1
        if self.main_road_spawn_count == 0:
            self.main_road_spawn_count += random.randint(10, 20)
            self.main_road_vehicles.append(vehicle.Vehicle(self.id_count))
            self.id_count += 1
        if self.merge_road_spawn_count == 0:
            self.merge_road_spawn_count += random.randint(10, 20)
            self.merge_road_vehicles_in_cacc.append(vehicle.Vehicle(self.id_count))
            self.id_count += 1

    def _calculate_reward(self, rl_state_):
        return 0.0, False

    def _change_mode(self):
        for _vehicle in self.merge_road_vehicles_in_cacc:
            pass

        for _vehicle in self.merge_road_vehicles_in_rl:
            pass
