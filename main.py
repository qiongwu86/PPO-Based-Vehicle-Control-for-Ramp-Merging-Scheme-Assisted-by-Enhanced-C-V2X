import matplotlib.pyplot as plt
import engine
import environment
import torch

CACC_ENG = engine.cacc_engine()
RL_ENG = engine.rl_engine(9)
env = environment.Environment(CACC_ENG, RL_ENG)

# load param
RL_ENG.actor.load_state_dict(torch.load("./model_param/2220_actor_param.pkl"))
RL_ENG.critic.load_state_dict(torch.load("./model_param/2220_critic_param.pkl"))
RL_ENG.prep_eval()
env.draw_trace(str(6700))


# actor_loss = []
# critic_loss = []
#
# for i in range(10000):
#     print("------------------------------------------------------")
#     print("episode {0}/{1}".format(i+1, 10000))
#
#     RL_ENG.prep_eval()
#     env.step()
#
#     RL_ENG.prep_train()
#     al, cl = RL_ENG.train(64, 256)
#
#     actor_loss.append(al)
#     critic_loss.append(cl)
#
#     if i % 20 == 0:
#
#         RL_ENG.save_model(str(i))
#
#         RL_ENG.prep_eval()
#         env.draw_trace(str(i))
#
#         plt.figure()
#         plt.plot(actor_loss)
#         plt.savefig("./loss_plot/" + str(i) + "_actor_loss.jpg")
#         plt.close()
#
#         plt.figure()
#         plt.plot(critic_loss)
#         plt.savefig("./loss_plot/" + str(i) + "_critic_loss.jpg")
#         plt.close()
