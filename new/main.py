from Astar_formal import AStarPlanner
from Island import getCN
from Agent import getAgent,Agent

import math
import numpy as np
# from matplotlib import pyplot as plt
import time
import datetime
from datetime import datetime, timedelta
import logging
import sys
import os
from pynput import keyboard
import websockets
import asyncio
import json
# import threading

def initialize_agents_and_charge_nodes():
    # Initialize Charge Nodes
    charge_nodes = getCN()
    
    # Initialize Agents
    agents = getAgent()
    for agent in agents:
        agent.CN_list = charge_nodes
    
    return agents, charge_nodes

def initialize_map(obstacles):
    astar = AStarPlanner(obstacles)
    return astar

def create_random_obstacles(random_seed=42):
    obstacles = []
    np.random.seed(random_seed)  # For reproducibility
    map_width = 225 * nm2m
    map_height = 130 * nm2m
    min_side_length = 4000
    max_side_length = 40000
    # Generate random rectangles as obstacles

    for _ in range(10):
        x1 = np.random.uniform(0, map_width - max_side_length)
        y1 = np.random.uniform(0, map_height - max_side_length)
        x2 = x1 + np.random.uniform(min_side_length, max_side_length)
        y2 = y1 + np.random.uniform(min_side_length, max_side_length)
        obstacles.append([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])

    return obstacles

# def plot_map(obstacles, charge_nodes, agents):
#     # 不要用subplot
#     fig = plt.figure(figsize=(10, 8))
#     ax = fig.add_subplot(111)
#     ax.set_xlim(0, 225 * nm2m)
#     ax.set_ylim(0, 130 * nm2m)
#     ax.set_title('Map')
#     ax.set_xlabel('X (m)')
#     ax.set_ylabel('Y (m)')
#     # 绘制障碍物
#     for obstacle in obstacles:
#         polygon = plt.Polygon(obstacle, closed=True, fill=True, color='gray', alpha=0.5)
#         ax.add_patch(polygon)
    
#     # 绘制Charge Nodes, 绿色圆圈
#     for cn in charge_nodes:
#         # print(cn.posX, cn.posY)
#         circle = plt.Circle((cn.posX, cn.posY), 10000, color='green', alpha=0.5)
#         ax.add_patch(circle)
#         ax.text(cn.posX, cn.posY, str(cn.id), fontsize=12, ha='center', va='center', color='black')
        
#     # 绘制船舶代理, plt.plot(x, y)，并用橘色三角形表示
#     agents_plt = []
#     texts_plt = []
#     for agent in agents:
#         moving_agent, = ax.plot([], [], '^', color='#FFD580', markersize=20, label='Agent')
#         moving_text = ax.text([], [], str(agent.id), fontsize=12, ha='center', va='center', color='black')
#         # TODO: agents_plt和texts_plt位置设置方法
#         moving_agent.set_data([agent.posX], [agent.posY])
#         moving_text.set_position([agent.posX, agent.posY])
        
#         agents_plt.append(moving_agent)
#         texts_plt.append(moving_text)
        
#         # plt.plot(agent.posX, agent.posY, '^', color='#FFD580', markersize=20)
#         # plt.text(agent.posX, agent.posY, str(agent.id), fontsize=12, ha='center', va='center', color='black')
    
#     # 绘制Astar路径
#         # path = np.array(agent.routepoints)
#         # plt.plot(path[:, 0], path[:, 1], '--', color='blue', linewidth=2, label='A* Path')
    
#     # 绘制时间戳，使用ax.text动态绘制
#     TimeStamp = ax.text([], [], 'Init', fontsize=20, ha='left', va='top', color='black')
#     TimeStamp.set_position([250000, 0])
    
#     plt.axis('equal')
#     # plt.legend()
#     plt.grid()
#     plt.pause(0.1)
#     return agents_plt, texts_plt, TimeStamp

def apply_Astar(agents:list[Agent], astarplanner:AStarPlanner):
    for agent in agents:
        # 计算路径
        path = astarplanner.planning(agent.posX, agent.posY, *agent.currentTar[:2])
        agent.astar = astarplanner
        agent.routepoints = path

def logging_set():
    '''
    -------------设置log参数---------------
    '''
    # 生成时间戳
    os.chdir('new/')
    file_name_full = os.path.basename(__file__)
    file_name, _ = os.path.splitext(file_name_full)
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_filename = f"{file_name}_{current_time}.log"

    # === 设置 logging ===
    logging.basicConfig(
        # filename= log_filename,     # 保存到文件
        filename= '{}.log'.format(file_name),     # 保存到文件
        filemode='w',   # 每次运行重新写，不追加
        format='%(asctime)s - %(levelname)s - %(message)s', # 格式：时间-等级-信息
        level=logging.INFO # 只记录INFO以上的信息
    )

    # 同时也让日志输出到终端
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)
    os.chdir('..')

def on_press(key):
    global isESC
    if key == keyboard.Key.esc:
        isESC = True
        return False  # 停止监听器

async def websocket_handler(websocket, path):
    global client_ws
    client_ws = websocket
    try:
        async for message in websocket:
            pass  # 这里不处理前端发来的消息
    except websockets.exceptions.ConnectionClosed:
        client_ws = None
        
def start_websocket_server():
    asyncio.set_event_loop(asyncio.new_event_loop())
    loop = asyncio.get_event_loop()
    start_server = websockets.serve(websocket_handler, "localhost", 8848)
    loop.run_until_complete(start_server)
    loop.run_forever()

# async def push_to_frontend(data: dict):
#     global client_ws
#     if client_ws and client_ws.open:
#         message = json.dumps(data)
#         await client_ws.send(message)

# client_ws = None  # 全局变量，用于存储WebSocket客户端连接
# # 启动 WebSocket 服务器（以守护线程方式）
# ws_thread = threading.Thread(target=start_websocket_server, daemon=True)
# ws_thread.start()
nm2m = 1852  # Nautical miles to meters conversion

def start_simulation(push_to_frontend):
    logging_set()
    '''
    初始化充电节点和船舶代理
    '''
    agents , charge_nodes = initialize_agents_and_charge_nodes()
    CN_statying_agents = [[], [], []] # 能量节点对应的停留船舶列表
    for agent in agents:
        agent.CN_staying = CN_statying_agents
    '''
    初始化障碍物与地图
    '''
    # nm2m = 1852  # Nautical miles to meters conversion
    obstacles = create_random_obstacles(random_seed=8)
    astar = initialize_map(obstacles)
    # print('plotting map...')
    apply_Astar(agents, astar)
    # agents_plt, texts_plt, TimeStamp = plot_map(obstacles, charge_nodes, agents)
    # plt.show()
    
    '''
    主循环，频率为1Hz
    '''
    # 仿真步长1s：15min
    T_loop = 2.0
    iter_num = 0
    # 仿真计时器
    current_time = datetime.strptime("00:00", "%H:%M")
    simu_step = timedelta(minutes=15)
    # while True:中接收esc键值自动停止
    isESC = False
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    while not isESC:
        t_start = time.time()
        current_time += simu_step
        time_text = current_time.strftime("%H:%M")
        logging.info("***************************Iteration {}: {}***************************".format(iter_num, time_text))
        # 先更新agents
        for idx in range(len(agents)):
            agents[idx].step()
            # agents_plt[idx].set_data([agents[idx].posX], [agents[idx].posY])
            # texts_plt[idx].set_position([agents[idx].posX, agents[idx].posY])
            # break
        
        # 分配充电
        for i in range(len(CN_statying_agents)):
            # 遍历每个节点
            pass
        
        
        
        
        # 更新CN
        for idx in range(len(charge_nodes)):
            charge_nodes[idx].step(iter_num)
        
        # TimeStamp.set_text(time_text)
        
        '''
        打印参数
        '''
        posInfo = [[agent.posX,agent.posY,agent.v,agent.Bat] for agent in agents]
        # print('posInfo: [[agent.posX, agent.posY, agent.v, agent.Bat], ...]\n', posInfo)
        CNInfo = [[cn.posX,cn.posY,cn.ESS, cn.WindP[math.floor(iter_num/4)], cn.SolarP[math.floor(iter_num/4)]] for cn in charge_nodes]
        # print('CNInfo: [[cn.posX, cn.posY, cn.ESS, cn.WindP, cn.SolarP], ...]\n', CNInfo)
        data_to_send = {
            "posInfo": posInfo,
            "CNInfo": CNInfo,
            "time": time_text
        }
        logging.info(">>>>>>>>>>data_to_send<<<<<<<<<<{}".format(data_to_send))
        push_to_frontend(data_to_send)
        
        # TimeStamp.set_text(time_text)
        # plt.pause(0.01)
        
        # if iter_num > 10:
        #     break
        
        # 休眠时间计算
        iter_num += 1
        logging.info('') # 打印空行
        elapsed = time.time() - t_start
        time.sleep(max(0, T_loop - elapsed))
    
    logging.info("Simulation ended.")
    