import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from matplotlib.backends.backend_pdf import PdfPages
def read_data(file_path):
    """
    从文件读取数据，返回 NumPy 数组，自动忽略表头并支持制表符分隔
    :param file_path: 数据文件路径
    :return: NumPy 数组
    """
    data = []
    with open(file_path, mode='r') as f:
        lines = f.readlines()
        for line in lines[1:]:  # 从第二行开始读取，忽略表头
            data.append([float(x) for x in line.strip().split(",")])  # 按制表符分隔数据
    return np.array(data)

# 假设需要对比的车辆 ID 列表
veh_in_merge = [7, 8, 9, 10, 11, 12]

original_folder = './data'
compare_folder = './compare_data'

# 定义输出文件夹
output_folder = './images/compare'  # 输出图片存储路径

# 从文件夹读取数据
def load_all_data(veh_ids, folder):
    """
    从指定文件夹中读取所有车辆数据
    :param veh_ids: 车辆 ID 列表
    :param folder: 数据文件夹路径
    :return: 字典，键为车辆 ID，值为车辆对应的数据
    """
    all_data = {}
    start_points = dict()
    for veh_id in veh_ids:
        file_path = os.path.join(folder, f'vehicle_{veh_id}_data.csv')
        if os.path.exists(file_path):
            all_data[veh_id] = read_data(file_path)
        else:
            print(f"数据文件 {file_path} 不存在，跳过该车辆数据！")
    return all_data

# 调用读取数据)
original_data = load_all_data(veh_in_merge, original_folder)
compare_data = load_all_data(veh_in_merge, compare_folder)
start_points_original = {}
end_points_original = {}
start_points_compare = {}
end_points_compare = {}

# 计算 original_data 的起始和结束点
for veh_id in veh_in_merge:
    data = original_data[veh_id]
    start_point_original = np.where(data[:, 1] > -175.0)[0][0]
    end_point_original = np.where(data[:, 1] > 0)[0][0]
    start_points_original[veh_id] = start_point_original
    end_points_original[veh_id] = end_point_original

# 计算 compare_data 的起始和结束点
for veh_id in veh_in_merge:
    data = compare_data[veh_id]
    start_point_compare = np.where(data[:, 1] > -175.0)[0][0]
    end_point_compare = np.where(data[:, 1] > 0)[0][0]
    start_points_compare[veh_id] = start_point_compare
    end_points_compare[veh_id] = end_point_compare


# 自定义格式化函数
def degree_formatter(x, pos):
    """将 y 轴刻度格式化为带角度符号和正负号的形式"""
    return f"{x:+.1f}°"  # 保留 1 位小数，带正负号和角度符号

# 绘制并保存图片函数
def plot_data_with_extremes(y_column, title, y_label, filename, to_degrees=False):
    """
    绘制图表，并标记最大值、最小值，以及计算差值，并保存图片
    :param y_column: 数据中的列索引，用于作为 y 轴
    :param title: 图表标题
    :param y_label: y 轴标签
    :param filename: 保存图片的文件名
    :param to_degrees: 是否将数据从弧度转换为角度
    """
    plt.figure(figsize=(12, 8))

    # 存储最大值和最小值
    max_diff, min_diff = None, None  # 用于存储最大值和最小值的差值

    # 原始数据绘制（统一样式）
    for veh_id in veh_in_merge:
        one_veh_data_original = original_data[veh_id]
        start_idx_original = start_points_original[veh_id]
        end_idx_original = end_points_original[veh_id]
        y_data = one_veh_data_original[start_idx_original:end_idx_original, y_column]
        if to_degrees:
            y_data = np.degrees(y_data)  # 转换为角度
        x_data = one_veh_data_original[start_idx_original:end_idx_original, 0]
        plt.plot(x_data, y_data, color='blue', linestyle='-', alpha=0.8, label="RL" if veh_id == veh_in_merge[0] else "")

        # 标记最大值和最小值
        max_idx = np.argmax(y_data)
        min_idx = np.argmin(y_data)
        plt.scatter(x_data[max_idx], y_data[max_idx], color='blue', s=100, label="Max RL" if veh_id == veh_in_merge[0] else "")
        plt.scatter(x_data[min_idx], y_data[min_idx], color='blue', s=100, label="Min RL" if veh_id == veh_in_merge[0] else "")

        # 存储最大值和最小值
        original_max = y_data[max_idx]
        original_min = y_data[min_idx]

    # 对比数据绘制（统一样式）
    for veh_id in veh_in_merge:
        one_veh_data_compare = compare_data[veh_id]
        start_idx_compare = start_points_compare[veh_id]
        end_idx_compare = end_points_compare[veh_id]
        y_data = one_veh_data_compare[start_idx_compare:end_idx_compare, y_column]
        if to_degrees:
            y_data = np.degrees(y_data)  # 转换为角度
        x_data = one_veh_data_compare[start_idx_compare:end_idx_compare, 0]
        plt.plot(x_data, y_data, color='red', linestyle='--', alpha=0.8, label="CACC-TP" if veh_id == veh_in_merge[0] else "")

        # 标记最大值和最小值
        max_idx = np.argmax(y_data)
        min_idx = np.argmin(y_data)
        plt.scatter(x_data[max_idx], y_data[max_idx], color='red', s=100, label="Max CACC-TP" if veh_id == veh_in_merge[0] else "")
        plt.scatter(x_data[min_idx], y_data[min_idx], color='red', s=100, label="Min CACC-TP" if veh_id == veh_in_merge[0] else "")

        # 计算最大值和最小值差值
        compare_max = y_data[max_idx]
        compare_min = y_data[min_idx]
        max_diff = abs(compare_max - original_max)
        min_diff = abs(compare_min - original_min)

    # 显示最大值和最小值的差值
    if max_diff is not None and min_diff is not None:
        unit = "°" if to_degrees else ""  # 根据是否是角度制决定单位
        plt.text(0.02, 0.95, f"Max Value Difference: {max_diff:.2f}{unit}", transform=plt.gca().transAxes, fontsize=12, color='green')
        plt.text(0.02, 0.90, f"Min Value Difference: {min_diff:.2f}{unit}", transform=plt.gca().transAxes, fontsize=12, color='green')

    # 设置图例和标签
    plt.grid()
    plt.title(title)
    plt.xlabel("Time [ms]")
    if to_degrees:
        plt.ylabel(f"{y_label} (°)")  # y 轴标签包含角度符号
        plt.gca().yaxis.set_major_formatter(FuncFormatter(degree_formatter))  # 应用 y 轴刻度格式化器
    else:
        plt.ylabel(y_label)
    plt.legend(loc='best')

    # 创建保存目录
    output_dir = "compare-images"
    os.makedirs(output_dir, exist_ok=True)  # 如果文件夹不存在则创建
    plt.savefig(os.path.join(output_dir, filename))  # 保存图片
    plt.close()

def plot_single_vehicle(veh_id, y_column, title, y_label, filename, to_degrees=False):
    """
    绘制单个车辆的对比图（如 ID 为 7 的车辆），并保存图片
    :param veh_id: 车辆 ID
    :param y_column: 数据中的列索引，用于作为 y 轴
    :param title: 图表标题
    :param y_label: y 轴标签
    :param filename: 保存图片的文件名
    :param to_degrees: 是否将数据从弧度转换为角度
    """
    plt.figure(figsize=(12, 8))

    # 获取原始数据
    one_veh_data_original = original_data[veh_id]
    start_idx_original = start_points_original[veh_id]
    end_idx_original = end_points_original[veh_id]
    y_data_original = one_veh_data_original[start_idx_original:end_idx_original, y_column]
    if to_degrees:
        y_data_original = np.degrees(y_data_original)  # 转换为角度
    x_data_original = one_veh_data_original[start_idx_original:end_idx_original, 0]
    plt.plot(x_data_original, y_data_original, color='blue', linestyle='-', alpha=0.8, label="RL")

    # 标记最大值和最小值
    max_idx_original = np.argmax(y_data_original)
    min_idx_original = np.argmin(y_data_original)
    plt.scatter(x_data_original[max_idx_original], y_data_original[max_idx_original], color='blue', s=100, label="Max RL")
    plt.scatter(x_data_original[min_idx_original], y_data_original[min_idx_original], color='blue', s=100, label="Min RL")

    # 获取对比数据
    one_veh_data_compare = compare_data[veh_id]
    start_idx_compare = start_points_compare[veh_id]
    end_idx_compare = end_points_compare[veh_id]
    y_data_compare = one_veh_data_compare[start_idx_compare:end_idx_compare, y_column]
    if to_degrees:
        y_data_compare = np.degrees(y_data_compare)  # 转换为角度
    x_data_compare = one_veh_data_compare[start_idx_compare:end_idx_compare, 0]
    plt.plot(x_data_compare, y_data_compare, color='red', linestyle='--', alpha=0.8, label="CACC-TP")

    # 标记最大值和最小值
    max_idx_compare = np.argmax(y_data_compare)
    min_idx_compare = np.argmin(y_data_compare)
    plt.scatter(x_data_compare[max_idx_compare], y_data_compare[max_idx_compare], color='red', s=100, label="Max CACC-TP")
    plt.scatter(x_data_compare[min_idx_compare], y_data_compare[min_idx_compare], color='red', s=100, label="Min CACC-TP")

    # 计算最大值和最小值差值
    max_diff = abs(y_data_compare[max_idx_compare] - y_data_original[max_idx_original])
    min_diff = abs(y_data_compare[min_idx_compare] - y_data_original[min_idx_original])

    # 显示最大值和最小值的差值
    unit = "°" if to_degrees else ""  # 根据是否是角度制决定单位
    plt.text(0.02, 0.90, f"Max Value Difference: {max_diff:.2f}{unit}", transform=plt.gca().transAxes, fontsize=12, color='green')
    plt.text(0.02, 0.85, f"Min Value Difference: {min_diff:.2f}{unit}", transform=plt.gca().transAxes, fontsize=12, color='green')

    # 设置图例和标签
    plt.grid()
    plt.title(title)
    plt.xlabel("Time [ms]")
    if to_degrees:
        plt.ylabel(f"{y_label} (°)")  # y 轴标签包含角度符号
        plt.gca().yaxis.set_major_formatter(FuncFormatter(degree_formatter))  # 应用 y 轴刻度格式化器
    else:
        plt.ylabel(y_label)
    plt.legend(loc='lower right')

    # 创建保存目录
    output_dir = "compare-images"
    os.makedirs(output_dir, exist_ok=True)  # 如果文件夹不存在则创建
    plt.savefig(os.path.join(output_dir, filename))  # 保存图片
    plt.close()

# 绘制并保存三张图
plot_data_with_extremes(2, "time-y plot", "y [m]", "time-y-plot.png")  # 第 2 列
plot_data_with_extremes(5, "time-body angle plot", "body angle", "time-body-angle-plot.png", to_degrees=True)  # 第 6 列，弧度制转角度制
plot_data_with_extremes(6, "time-steer plot", "steer", "time-steer-plot.png", to_degrees=True)  # 第 7 列，弧度制转角度制
plot_single_vehicle(
    veh_id=7,
    y_column=2,  # y 轴为第二列
    title="Vehicle 7 time-y plot",
    y_label="y [m]",
    filename="vehicle-7-time-y-plot.png"
)
