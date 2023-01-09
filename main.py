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
import matplotlib.pyplot as plt

import engine
import environment

CACC_ENG = engine.cacc_engine()
RL_ENG = engine.rl_engine(9)
env = environment.Environment(CACC_ENG, RL_ENG)

actor_loss = []
critic_loss = []

for i in range(10000):
    print("------------------------------------------------------")
    print("episode {0}/{1}".format(i+1, 10000))
    env.step()
    al, cl = RL_ENG.train(50, 256)
    actor_loss.append(al)
    critic_loss.append(cl)

    if i % 20 == 0:

        RL_ENG.save_model(str(i))

        env.draw_trace(str(i))

        plt.figure()
        plt.plot(actor_loss)
        plt.savefig("./loss_plot/" + str(i) + "_actor_loss.jpg")
        plt.close()

        plt.figure()
        plt.plot(critic_loss)
        plt.savefig("./loss_plot/" + str(i) + "_critic_loss.jpg")
        plt.close()





