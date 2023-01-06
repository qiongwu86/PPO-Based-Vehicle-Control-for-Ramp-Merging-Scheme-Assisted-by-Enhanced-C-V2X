# from matplotlib import pyplot as plt
# from matplotlib.animation import FuncAnimation
# from functools import partial
# import math
#
# fig, ax = plt.subplots()
# rect = plt.Rectangle((0, 0), 2, 1)
# plt.xlim(-10, 10)
# plt.ylim(-10, 10)
# plt.axis('equal')
# ax.add_patch(rect)
# # rect.set
#
#
# def update(frame_num, obj_list=None):
#     for obj in obj_list:
#         obj.set_xy((0.1*frame_num, 0.1*frame_num))
#         obj.set_angle(frame_num*10*math.pi/180)
#
#
# anime = FuncAnimation(fig=fig, func=partial(update, obj_list=[rect, ]), frames=50, interval=1000/60)
#
# anime.save('rect.gif')

import vehicle


all_veh = list()
all_veh.append(vehicle.Vehicle())
all_veh.append(vehicle.Vehicle())

print('121212')

v = all_veh.pop()
del v
v = all_veh.pop()
del v
