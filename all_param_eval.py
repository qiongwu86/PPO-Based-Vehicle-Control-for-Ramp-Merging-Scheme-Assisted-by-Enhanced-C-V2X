import matplotlib.pyplot as plt
import engine
import environment
import torch
from matplotlib import ticker


CACC_ENG = engine.cacc_engine()
RL_ENG = engine.rl_engine(9)
env = environment.Environment(CACC_ENG, RL_ENG)


rewards = []
success_rate = []
for epc in range(int(6700/20)+1):
    RL_ENG.actor.load_state_dict(torch.load("./model_param/{}_actor_param.pkl".format(epc*20)))
    RL_ENG.critic.load_state_dict(torch.load("./model_param/{}_critic_param.pkl".format(epc*20)))
    RL_ENG.prep_eval()
    ave_r, succ_rate = env.step_for_average_reward(10)
    # print("epc: {0}, average")
    rewards.append(ave_r)
    success_rate.append(succ_rate)


fig, ax = plt.subplots()
plt.plot(rewards)
plt.grid()
plt.xlabel("training epoch")
plt.ylabel("average reward")
plt.savefig("./images/average_reward.jpg")
plt.close()

fig, ax = plt.subplots()
plt.plot(success_rate)
plt.grid()
plt.xlabel("training epoch")
plt.ylabel("success rate[%]")
ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1))
plt.savefig("./images/success_rate.jpg")
plt.close()



