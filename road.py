import matplotlib.pyplot as plt
import numpy as np

# 定义常量
RADIA_15 = 15 * np.pi / 180
TAN_15 = np.tan(RADIA_15)
COS_15 = np.cos(RADIA_15)
L = 200 / COS_15

def get_center_line():
    center_line = []

    # 主路中心线
    x_main = np.linspace(-575, 100, 200)  # 主路只从 -175 到 100
    y_main = np.zeros_like(x_main)  # 主路中心线始终在 y = 0
    center_line.extend(zip(x_main, y_main))

    # # 岔路斜线段1
    # x_branch1 = np.linspace(-575, -375, 1000)
    # y_branch1 = -TAN_15 * 200 - 3.75
    # center_line.extend(zip(x_branch1, [y_branch1] * len(x_branch1)))
    #
    # # 岔路斜线段2
    # x_branch2 = np.linspace(-375, -175, 1000)
    # y_start_branch2 = -TAN_15 * 200 - 3.75
    # y_end_branch2 = -3.75  # 修正为岔路中心点
    # slope_branch2 = (y_end_branch2 - y_start_branch2) / (200)  # 斜率计算
    # y_branch2 = slope_branch2 * (x_branch2 + 375) + y_start_branch2  # 中心线公式
    # center_line.extend(zip(x_branch2, y_branch2))
    #
    # # 主路交汇中心线
    # x_merge = np.linspace(-175, 0, 1000)
    # y_start_merge = -3.75  # 起点是岔路中心点
    # y_end_merge = 0  # 终点是主路中心点
    # slope_merge = (y_end_merge - y_start_merge) / 175  # 斜率计算
    # y_merge = slope_merge * (x_merge + 175) + y_start_merge  # 中心线公式
    # center_line.extend(zip(x_merge, y_merge))
    #
    # x_main = np.linspace(0, 100, 500)  # 主路只从 -175 到 100
    # y_main = np.zeros_like(x_main)  # 主路中心线始终在 y = 0
    # center_line.extend(zip(x_main, y_main))

    return np.array(center_line)

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

    # 绘制中心线
    center_line = get_center_line()
    plt.plot(center_line[:, 0], center_line[:, 1], '--r', label='Center Line')

    plt.axis('equal')
    plt.legend()
    # plt.show()

# 调用绘制函数
draw_road()