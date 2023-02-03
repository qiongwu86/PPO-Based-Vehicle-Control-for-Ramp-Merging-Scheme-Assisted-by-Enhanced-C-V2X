import math

import matplotlib.pyplot as plt
import numpy as np

SIN_15 = math.sin(15*math.pi/180)
COS_15 = math.cos(15*math.pi/180)
RAID_15 = 15*math.pi/180
TAN_15 = math.tan(RAID_15)


FOUR_POINTS = np.array([[4.5, 1.0],
                        [0.0, 1.0],
                        [0.0, -1.0],
                        [4.5, -1.0]])


def get_four_points(x: float, y: float, body_angle: float):
    rotate_mat = np.array([[math.cos(body_angle), -math.sin(body_angle)],
                           [math.sin(body_angle), math.cos(body_angle)]])
    rotated_points = np.matmul(rotate_mat, FOUR_POINTS.T).T
    return rotated_points + np.array((x, y))


def check_two_vehicles_collision(v1_state, v2_state):
    if np.linalg.norm([v1_state[0] - v2_state[0], v1_state[1] - v2_state[1]]) > 15:
        return False

    v1_four_points = get_four_points(v1_state[0], v1_state[1], v1_state[2])
    v2_four_points = get_four_points(v2_state[0], v2_state[1], v2_state[2])

    four_angle = np.array([v1_state[2],
                           v1_state[2] + math.pi * 0.5,
                           v2_state[2],
                           v2_state[2] + math.pi * 0.5])

    four_vector = np.array([np.cos(four_angle), np.sin(four_angle)]).T

    for i in range(4):
        v1_proj = np.sum(four_vector[i] * v1_four_points, 1)
        v1_min, v1_max = np.min(v1_proj), np.max(v1_proj)

        v2_proj = np.sum(four_vector[i] * v2_four_points, 1)
        v2_min, v2_max = np.min(v2_proj), np.max(v2_proj)

        if v1_max < v2_min or v1_min > v2_max:
            return False

    return True


def update(frame_num, obj_dict, obj_data):
    current_data = obj_data[frame_num]
    for one_vehicle_data in current_data:
        v_id, mode, x, y, body_angle = one_vehicle_data
        x, y = get_central(x, y, body_angle)
        # print(body_angle, mode)
        obj_dict[v_id].set_xy((x, y))
        obj_dict[v_id].set_angle(body_angle*180/math.pi)


def get_central(x, y, body_angle):
    return x+math.sin(body_angle), y-math.cos(body_angle)


def draw_road():
    temp = TAN_15 * 200
    plt.plot([-575, 100], [0.5 * 3.75, 0.5 * 3.75], color='b')
    plt.plot([-575, -175], [-0.5 * 3.75, -0.5 * 3.75], color='b')
    plt.plot([-575, -375], [-0.5 * 3.75 - temp, -0.5 * 3.75 - temp], color='b')
    plt.plot([-375, -175], [-0.5 * 3.75 - temp, -0.5 * 3.75], color='b')
    plt.plot([-575, -375], [-1.5 * 3.75 - temp, -1.5 * 3.75 - temp], color='b')
    plt.plot([-375, -175], [-1.5 * 3.75 - temp, -1.5 * 3.75], color='b')
    plt.plot([-175, 0], [-1.5 * 3.75, -1.5 * 3.75], color='b')
    plt.plot([0, 0], [-1.5 * 3.75, -0.5 * 3.75], color='b')
    plt.plot([0, 100], [-0.5 * 3.75, -0.5 * 3.75], color='b')
