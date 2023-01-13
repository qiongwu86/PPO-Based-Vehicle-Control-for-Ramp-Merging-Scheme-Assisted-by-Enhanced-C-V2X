import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial
import engine
import socket
import struct
import utilities


header_template = "IIII"
# 1: 包类型 0 = first, 1 = end, 2 = action request
# 2: ID/veh_num
# 3: 包长度
# 4: mode CACC=0, RL=1
header_start = 0
header_end = 16

self_state_template = "ddd"
# x, y, body angle
size_state_template = struct.calcsize(self_state_template)
self_state_start = header_end
self_state_end = self_state_start + size_state_template


cacc_state_template = "ddd?dd"
size_cacc_state = struct.calcsize(cacc_state_template)
cacc_state_start = self_state_end
cacc_state_end = cacc_state_start + size_cacc_state


rl_state_template = "ddddddddd"
size_rl_state = struct.calcsize(rl_state_template)
rl_state_start = self_state_end
rl_state_end = rl_state_start + size_rl_state

send_template = "dd"


class Engine:
    def __init__(self):
        self.cacc_engine = engine.cacc_engine()
        self.rl_engine = engine.rl_engine(9)

        self.current_episode_vehicle_num = 0
        self.all_data = dict()

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def server_start(self):
        try:
            self.listen_sock.bind(("192.168.138.1", 10889))
            self.listen_sock.listen()
        except Exception as e:
            print("bind or listen error: [}".format(e))

        print("server start....")

    def server_run(self):
        while True:
            print("------------------------------")
            conn, addr = self.listen_sock.accept()
            print("client connect to ", addr)
            self.start_control(conn, addr)
            conn.close()
            print("close")
            print("------------------------------")

    def start_control(self, conn, addr):
        self.all_data.clear()
        self.current_episode_vehicle_num = 0

        first_pkt = conn.recv(1024)
        pkt_type, veh_num, _, _ = struct.unpack(header_template, first_pkt[header_start: header_end])
        self.current_episode_vehicle_num = veh_num

        for i in range(veh_num):
            self.all_data[i] = list()

        print("recv first packet, total {} vehicles".format(veh_num))

        conn.send(struct.pack(send_template, *(0.0, 0.0)))

        assert pkt_type == 0

        while True:
            recv_pkt = conn.recv(1024)

            pkt_type, veh_id, pkt_len, mode = struct.unpack(header_template, recv_pkt[header_start: header_end])
            # 包长度
            assert pkt_len == len(recv_pkt)

            if pkt_type == 1:
                print("client close")
                conn.close()
                break
            # 包类型，正常动作包
            assert pkt_type == 2

            # 保存状态
            self_state = struct.unpack(self_state_template, recv_pkt[self_state_start: self_state_end])
            self.all_data[veh_id].append(self_state)
            print("[recv] recv pkt from {0:3.0f}, current state: {1:4.2f}, {2:4.2f}".
                  format(veh_id, self_state[0], self_state[1]))

            # 生成动作并发送
            if mode == 0:
                state = struct.unpack(cacc_state_template, recv_pkt[cacc_state_start: cacc_state_end])
                action = self.cacc_engine.generate_action(state).tolist()
                send_byte = struct.pack(send_template, *action)

            else:
                state = struct.unpack(rl_state_template, recv_pkt[rl_state_start: rl_state_end])
                action = self.rl_engine.generate_action_for_test(np.array(state, dtype=np.float32)).tolist()
                send_byte = struct.pack(send_template, *action)

            conn.send(send_byte)
            print("[send] send action({0}, {1}) to {2}".format(action[0], action[1], veh_id))

        fig = plt.figure()
        ax = plt.axes()
        ax.set_aspect(1)

        plt.xlim(-575, -175)
        plt.ylim(-60, 10)

        rect = plt.Rectangle((-575, -60), 675, 100, color='g')
        ax.add_patch(rect)

        utilities.draw_road()

        all_rect = dict()
        for veh_id in self.all_data:
            all_rect[veh_id] = plt.Rectangle((self.all_data[veh_id][0][0], self.all_data[veh_id][0][1]), 4.5, 2.0, color='r')
            ax.add_patch(all_rect[veh_id])

        frames_num = 0
        for _ in self.all_data:
            if len(self.all_data[_]) > frames_num:
                frames_num = len(self.all_data[_])

        anime = FuncAnimation(fig=fig, func=partial(utilities.update_ns3,
                                                    obj_dict=all_rect,
                                                    obj_data=self.all_data), frames=frames_num,  interval=1000 / 60)

        anime.save("./gifs/demo_" + "ns3" + ".gif")
        plt.close()

e = Engine()
e.server_start()
e.server_run()
