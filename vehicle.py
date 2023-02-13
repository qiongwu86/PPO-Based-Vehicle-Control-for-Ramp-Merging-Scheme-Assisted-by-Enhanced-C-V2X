import enum
import math
import utilities
import numpy as np
import random
import buffer


RADIA_15 = 15*math.pi / 180
TAN_15 = math.tan(RADIA_15)
COS_15 = math.cos(RADIA_15)
L = 200 / COS_15


class Mode(enum.Enum):
    MAIN_CACC = 0
    MERGE_CACC = 1
    MERGE_RL = 2


class Vehicle:

    all_vehicle = list()
    acc_max = 3.0
    acc_min = -3.0
    steer_max = 15*math.pi/180
    steer_min = -15*math.pi/180
    headway_time = 1.0
    sensing_distance = 100.0
    expect_speed = 23.0
    init_speed = 15.0
    speed_max = 25

    def __init__(self, ego_id: int, mode: Mode):
        self.id = ego_id
        self.mode = mode
        self.buffer = None
        self.x = 0
        self.y = 0
        self.body_angle = 0
        # self.last_body_angle = 0 # 用于计算delta_body_angle
        self.steer = 0
        self.acc = 0
        self.speed = Vehicle.init_speed + random.uniform(-3, 3)
        if mode is Mode.MAIN_CACC:
            self.x = -575.0
            self.y = 0.0
        elif mode is Mode.MERGE_CACC:
            self.x = -575.0
            self.y = -3.75 - TAN_15*200.0
        Vehicle.all_vehicle.append(self)
        if self.mode is Mode.MERGE_CACC:
            self.buffer = buffer.SubBuffer()

    def change_mode(self):
        if self.mode is Mode.MAIN_CACC:
            return
        elif self.mode is Mode.MERGE_CACC:
            if 0 > self.x > -175.0:
                self.mode = Mode.MERGE_RL
                # 判断安全距离
            return
        else:
            if self.x > 0:
                self.mode = Mode.MERGE_CACC
            return

    def check_and_if_delete_self(self):
        if self.x > 100:
            return True

        if self.mode is not Mode.MERGE_RL:
            return False

        if self.coll_with_edge() or self.coll_with_other():
            return True

        if self.speed < 5.0:
            return True

    def coll_with_edge(self):
        four_points = utilities.get_four_points(self.x, self.y, self.body_angle)
        for point in four_points:
            x, y = point
            if x < -175.0:
                return True
            elif -175.0 < x < 0 and (not (-1.5*3.75 < y < 0.5*3.75)):
                return True
            elif x > 0 and (not (-0.5*3.75 < y < 0.5*3.75)):
                return True
        return False

    def coll_with_other(self):
        for other_vehicle in Vehicle.all_vehicle:
            if other_vehicle is self:
                continue

            if utilities.check_two_vehicles_collision((self.x, self.y, self.body_angle),
                                                      (other_vehicle.x, other_vehicle.y, other_vehicle.body_angle)):
                return True

        return False

    def get_state(self):
        self_proj = self.get_proj()
        self_longitude_speed = self.calculate_longitude_speed()
        prev_proj = self_proj+Vehicle.sensing_distance
        foll_proj = self_proj-Vehicle.sensing_distance
        prev = None
        foll = None

        for v in Vehicle.all_vehicle:
            # prev, foll
            if v is self:
                continue
            v_proj = v.get_proj()
            if self_proj < v_proj < prev_proj:
                prev_proj = v_proj
                prev = v
            elif foll_proj < v_proj < self_proj:
                foll_proj = v_proj
                foll = v

        if self.mode is Mode.MERGE_RL:
            self_state = (self_proj,
                          self.y,
                          self.y+4.5*math.sin(self.body_angle),
                          self.speed,
                          self.body_angle)

            if prev is None:
                prev_state = (self_longitude_speed*Vehicle.headway_time, 0.0)
            else:
                prev_state = (prev_proj - self_proj,
                              prev.calculate_longitude_speed() - self_longitude_speed)

            if foll is None:
                foll_state = (self_longitude_speed*Vehicle.headway_time, 0.0)
            else:
                foll_state = (self_proj - foll_proj,
                              self_longitude_speed - foll.calculate_longitude_speed())

            return np.array(self_state + prev_state + foll_state, dtype=np.float32)

        else:
            if prev is None:
                return np.array((self_proj, self.speed, self.acc, False, 0.0, 0.0), dtype=np.float32)
            else:
                return np.array((self_proj, self.speed, self.acc, True, prev_proj, prev.speed), dtype=np.float32)

    def input_action(self, action):
        if self.mode is Mode.MAIN_CACC:
            assert action[1] == 0.0
            self.acc = np.clip(action[0], Vehicle.acc_min, Vehicle.acc_max)
            self.steer = np.clip(action[1], Vehicle.steer_min, Vehicle.steer_max)

            self.x += 0.1*self.speed*math.cos(self.body_angle)
            self.y += 0.1*self.speed*math.sin(self.body_angle)

            self.body_angle += 0.1*self.speed*math.tan(self.steer) / 4.5
            self.speed = np.clip(self.speed+0.1*self.acc, 0, Vehicle.speed_max)
        else:
            last_x = self.x
            self.acc = np.clip(action[0], Vehicle.acc_min, Vehicle.acc_max)
            self.steer = np.clip(action[1], Vehicle.steer_min, Vehicle.steer_max)

            self.x += 0.1*self.speed*math.cos(self.body_angle)
            self.y += 0.1*self.speed*math.sin(self.body_angle)

            self.body_angle += 0.1*self.speed*math.tan(self.steer) / 4.5
            self.speed = np.clip(self.speed + 0.1 * self.acc, 0, Vehicle.speed_max)

            if last_x < -375 < self.x:
                self.body_angle = RADIA_15
                temp = COS_15 * (self.x + 375)
                self.x = -375 + temp
                self.y += TAN_15 * temp

            if last_x < -175 < self.x:
                self.body_angle = 0
                temp = np.linalg.norm([self.x + 175, self.y + 3.75])
                self.x = -175 + temp
                self.y = -3.75

            if last_x < 0 < self.x:
                self.body_angle = 0

    def get_proj(self):
        if self.mode is Mode.MAIN_CACC:
            return self.x
        else:
            if self.x > -175:
                return self.x
            elif self.x < -375:
                return -(175 + L + (-375 - self.x))
            else:
                return -(175 + (-175-self.x) / COS_15)

    def calculate_longitude_speed(self):
        if self.x < -175 or self.x > 0:
            return self.speed
        else:
            return self.speed*math.cos(self.body_angle)

    def state_for_draw(self):
        return tuple((self.id, Mode(self.mode).value, self.x, self.y, self.body_angle))
