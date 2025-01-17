import matplotlib.pyplot as plt
from matplotlib import ticker


aoi_threshold = [40, 60, 80, 100, 120]
pos_error_threshold = [2, 3, 4, 5]
dis_threshold = [50, 100, 150, 200]
inf_vehs = [0, 20, 40]
run = [11, 22, 33, 44, 55]
run2 = [1, 2, 3, 4, 5]


std_aoi_file_name_template = "./results_mod/{0}_{1}_AoI_over"
mod_aoi_file_name_template = "./results_mod/{0}_{1}_AoI_over"

std_pos_file_name_template = "./results_mod/{0}_{1}_position_error"
mod_pos_file_name_template = "./results_mod/{0}_{1}_position_error"


# 位置误差
pos_error_dict = dict()
for i in inf_vehs:
    pos_error_dict[i] = dict()

    for dis in dis_threshold:
        pos_error_dict[i][dis] = dict()
        pos_error_dict[i][dis]['std'] = list()
        pos_error_dict[i][dis]['mod'] = list()

for infNum in inf_vehs:
    for dis in dis_threshold:
        # std
        for pos_error in pos_error_threshold:
            pos_error_of_one_pos_th = 0.0
            for runParm in run:
                std_file = std_pos_file_name_template.format(infNum, runParm)
                lines = open(std_file, 'r').readlines()
                line = lines[1+(int(dis/50.0)-1)*6 + pos_error][:-1].split("  ")
                pos_error_of_one_pos_th += (int(line[0]) / int(line[1]))
            pos_error_dict[infNum][dis]['std'].append(pos_error_of_one_pos_th/5)
#         mod
        for pos_error in pos_error_threshold:
            pos_error_of_one_pos_th = 0.0
            for runParm in run2:
                mod_file = mod_pos_file_name_template.format(infNum, runParm)
                lines = open(mod_file, 'r').readlines()
                line = (lines[1+(int(dis/50.0)-1)*6 + pos_error][: -1]).split("  ")
                pos_error_of_one_pos_th += (int(line[0]) / int(line[1]))
            pos_error_dict[infNum][dis]['mod'].append(pos_error_of_one_pos_th/5)
    fig, ax = plt.subplots()
    plt.plot(pos_error_threshold, pos_error_dict[infNum][50]['std'], marker="+", color='r')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][100]['std'], marker="o", color='r')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][150]['std'], marker="v", color='r')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][200]['std'], marker="^", color='r')

    plt.plot(pos_error_threshold, pos_error_dict[infNum][50]['mod'], marker="+", color='g')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][100]['mod'], marker="o", color='g')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][150]['mod'], marker="v", color='g')
    plt.plot(pos_error_threshold, pos_error_dict[infNum][200]['mod'], marker="^", color='g')

    plt.legend(["std-50m", "std-100m", "std-150m", "std-200m", "mod-50m", "mod-100m", "mod-150m", "mod-200m"])
    plt.grid()
    plt.xlabel("threshold[m]")
    plt.ylabel("position error rate[%]")

    ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1))

    plt.savefig("./images/" + str(infNum) + "position_error_rate.jpg")
    plt.show()
    # plt.close()


# Aoi误差
aoi_dict = dict()
for i in inf_vehs:
    aoi_dict[i] = dict()

    for dis in dis_threshold:
        aoi_dict[i][dis] = dict()
        aoi_dict[i][dis]['std'] = list()
        aoi_dict[i][dis]['mod'] = list()

for infNum in inf_vehs:
    for dis in dis_threshold:
        # std
        for aoi_over in aoi_threshold:
            aoi_th_of_one_pos_th = 0.0
            for runParm in run:
                std_file = std_aoi_file_name_template.format(infNum, runParm)
                lines = open(std_file, 'r').readlines()
                line = lines[1+(int(dis/50.0)-1)*6 + int((aoi_over-20.0)/20.0)][:-1].split("  ")
                aoi_th_of_one_pos_th += (int(line[0]) / int(line[1]))
            aoi_dict[infNum][dis]['std'].append(aoi_th_of_one_pos_th/5)
        # mod
        for aoi_over in aoi_threshold:
            aoi_th_of_one_pos_th = 0.0
            for runParm in run2:
                mod_file = mod_aoi_file_name_template.format(infNum, runParm)
                lines = open(mod_file, 'r').readlines()
                line = lines[1+(int(dis/50.0)-1)*6 + int((aoi_over-20.0)/20.0)][:-1].split("  ")
                aoi_th_of_one_pos_th += (int(line[0]) / int(line[1]))
            aoi_dict[infNum][dis]['mod'].append(aoi_th_of_one_pos_th/5)
    fig, ax = plt.subplots()
    plt.plot(aoi_threshold, aoi_dict[infNum][50]['std'], marker="+", color='r')
    plt.plot(aoi_threshold, aoi_dict[infNum][100]['std'], marker="o", color='r')
    plt.plot(aoi_threshold, aoi_dict[infNum][150]['std'], marker="v", color='r')
    plt.plot(aoi_threshold, aoi_dict[infNum][200]['std'], marker="^", color='r')

    plt.plot(aoi_threshold, aoi_dict[infNum][50]['mod'], marker="+", color='g')
    plt.plot(aoi_threshold, aoi_dict[infNum][100]['mod'], marker="o", color='g')
    plt.plot(aoi_threshold, aoi_dict[infNum][150]['mod'], marker="v", color='g')
    plt.plot(aoi_threshold, aoi_dict[infNum][200]['mod'], marker="^", color='g')

    plt.legend(["std-50m", "std-100m", "std-150m", "std-200m", "mod-50m", "mod-100m", "mod-150m", "mod-200m"])
    plt.grid()
    plt.xlabel("threshold[ms]")
    plt.ylabel("over-time rate[%]")

    ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1))

    plt.savefig("./images/" + str(infNum) + "aoi_over_rate.jpg")
    plt.show()
    # plt.close()


std_aoi = []
mod_aoi = []
for infNum in inf_vehs:
    std_temp = 0.0
    mod_temp = 0.0
    for runParm in run:
        std_file = std_aoi_file_name_template.format(infNum, runParm)
        line = open(std_file, 'r').readlines()[-1][:-1]
        std_temp += float(line.split("= ")[-1])
    std_aoi.append(std_temp/5)
    for runParm in run2:
        mod_file = mod_aoi_file_name_template.format(infNum, runParm)
        line = open(mod_file, 'r').readlines()[-1][:-1]
        mod_temp += float(line.split("= ")[-1])
    mod_aoi.append(mod_temp/5)
print(std_aoi)
print(mod_aoi)


