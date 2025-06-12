from Astar_formal import AStarPlanner
from Island import getCN
from Agent import getAgent,Agent
import math
import numpy as np
import time
import datetime
import logging
import sys
import os
# from pynput import keyboard
import websockets
import asyncio
import pandas as pd
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
    # current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # log_filename = f"{file_name}_{current_time}.log"

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

# def on_press(key):
#     global isESC
#     if key == keyboard.Key.esc:
#         isESC = True
#         return False  # 停止监听器

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

nm2m = 1852  # Nautical miles to meters conversion

def start_simulation(push_to_frontend=None, stop_check=None):
    """
    启动仿真
    
    参数:
    - push_to_frontend: 用于向前端发送数据的函数
    - stop_check: 可选的回调函数，用于检查是否应该停止仿真
    """
    logging_set()
    CRUISE = 0 # 巡航状态
    CHARGING = 1 # 充电状态
    STAY = 2 # 停留状态
    MISSION = 3 # 任务状态
    DROP= 4 # 电量耗尽状态
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
    
    # obs_to_send = {
    #         "obs_info": np.int64(obstacles).tolist(),
    #     }
    # push_to_frontend(obs_to_send)
    obs_info = np.int64(obstacles).tolist()
    
    astar = initialize_map(obstacles)
    # print('plotting map...')
    apply_Astar(agents, astar)
    # 巡航线路确定,cruise_route[i]代表到达i节点的路线，即i-1到i节点
    Cruise_route = []
    for i in range(len(charge_nodes)):
        route = astar.planning(charge_nodes[(i-1)%3].posX, charge_nodes[(i-1)%3].posY, charge_nodes[i].posX,charge_nodes[i].posY)
        Cruise_route.append(route) 
    for agent in agents:
        agent.Cruise_routes = Cruise_route
    
    '''
    主循环，频率为1Hz
    '''
    # 仿真步长1s：15min
    T_loop = 1.0
    iter_num = 0

    # 在航率
    service_hour = 0
    service = {'service_rate_Total': [], 'service_rate_RT': []}
    time_text_list = []
    while True:
        # 检查是否应该停止仿真
        delta_service_hour = 0
        if stop_check and stop_check():
            logging.info("收到停止信号，正在停止仿真...")
            break
            
        t_start = time.time()

        iter_num += 1
        time_text = f'{iter_num//4}:{iter_num%4*15:02d}'
        time_text_list.append(time_text)
        if iter_num > 24*7*4:
            break
        logging.info("***************************Iteration {}: {}***************************".format(iter_num, time_text))
        # 先更新agents
        for idx in range(len(agents)):
            # if idx !=1:
            #     continue
            agents[idx].step()
            if agents[idx].state == CRUISE:
                service_hour += 0.25
                delta_service_hour += 0.25
            # break
        
        # 分配充电,遍历每个节点
        for i in range(len(CN_statying_agents)):
            # 先来后到充电，充电功率均分
            charge_agent_num = min(len(CN_statying_agents[i]), 3)
            if charge_agent_num == 0: # 防止除数为0
                continue
            # 以100kW为最小分辨率
            max_CP_allow = math.floor(charge_nodes[i].ESS/100/charge_agent_num) * 100
            charge_p = min(max_CP_allow, 500)
            for agent in agents:
                if agent.id in CN_statying_agents[i][:charge_agent_num]:
                    agent.ChargingP = charge_p
                    # 更新节点端充电功率
                    agent.CN_list[i].ESSPOut += charge_p
                    agent.Bat += charge_p * 0.25
                    if agent.Bat >= agent.BatteryCap * 0.9:
                        agent.state = CRUISE
                        agent.ChargingP = 0
                        CN_statying_agents[i].remove(agent.id)
                    logging.info(f"Agent {agent.id} charging at CN {i}: {charge_p} kW, battery: {agent.Bat:.2f} kWh ({agent.Bat/agent.BatteryCap*100:.2f}%)")
        
        
        
        # 更新CN
        for idx in range(len(charge_nodes)):
            charge_nodes[idx].step(iter_num)
        
        service_rate = round(service_hour / (iter_num * len(agents) * 0.25) *100,2)
        service['service_rate_Total'].append(service_rate)
        service['service_rate_RT'].append(round(delta_service_hour/(len(agents)*0.25)*100,2))
        '''
        打印参数
        '''
        posInfo = [[agent.posX,agent.posY,agent.v,agent.Bat] for agent in agents]
        CNInfo = [[cn.posX,cn.posY,cn.ESS, cn.WindP[math.floor(iter_num/4)], cn.SolarP[math.floor(iter_num/4)], round(cn.ESS/cn.ESSCap*100,2)] for cn in charge_nodes]
        CNCurve = [
            [[time_text_list[-96:], cn.ESS_list[-96:]],
              [time_text_list[-96:], cn.windp_list[-96:]],
              [time_text_list[-96:], cn.solarp_list[-96:]],
              [time_text_list[-96:],cn.pout_list[-96:]]]
            for cn in charge_nodes]
        service_rate_curve = [
            [time_text_list[-96:], service['service_rate_Total'][-96:]],
            [time_text_list[-96:], service['service_rate_RT'][-96:]]
                               ] 
        data_to_send = {
            "posInfo": posInfo,
            "CNInfo": CNInfo,
            "time": time_text,
            'service_rate': '{}%'.format(service_rate),
            'CNCurve': CNCurve,
            'service_rate_curve': service_rate_curve,
            'obs_info': obs_info,
        }
        # logging.info(">>>>>>>>>>push_to_frontend<<<<<<<<<<",json.dumps(data_to_send))
        push_to_frontend(data_to_send)
        logging.info(f"Service rate: {service_rate:.2f} %")
        logging.info('') # 打印空行
        elapsed = time.time() - t_start
        time.sleep(max(0, T_loop - elapsed))
    
    logging.info("Simulation ended.")
    pd.DataFrame(service).to_csv('service_rate.csv',index=False)
    ESS_dict = {}
    for cn in charge_nodes:
        ESS_dict[cn.id] = cn.ESS_list
    pd.DataFrame(ESS_dict).to_csv('ESS.csv',index=False)

def allocate_power(P, n, max_per_port=800):
    '''
    分配充电功率：优先满足前面的接口，最大不超过 max_per_port。
    
    :param P: 总功率（必须为100的倍数）
    :param n: 接口数量
    :param max_per_port: 每个接口的最大功率（默认500）
    :return: 长度为n的列表，每个元素是对应接口的分配功率
    '''
    allocation = [0] * n
    for i in range(n):
        if P >= max_per_port:
            allocation[i] = max_per_port
            P -= max_per_port
        else:
            allocation[i] = P
            P = 0
    return allocation

def start_simulation_improve(push_to_frontend=None, stop_check=None):
    """
    启动仿真
    
    参数:
    - push_to_frontend: 用于向前端发送数据的函数
    - stop_check: 可选的回调函数，用于检查是否应该停止仿真
    """
    logging_set()
    CRUISE = 0 # 巡航状态
    CHARGING = 1 # 充电状态
    STAY = 2 # 停留状态
    MISSION = 3 # 任务状态
    DROP= 4 # 电量耗尽状态
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
    
    # obs_to_send = {
    #         "obs_info": np.int64(obstacles).tolist(),
    #     }
    # push_to_frontend(obs_to_send)
    obs_info = np.int64(obstacles).tolist()
    
    astar = initialize_map(obstacles)
    # print('plotting map...')
    apply_Astar(agents, astar)
    # 巡航线路确定,cruise_route[i]代表到达i节点的路线，即i-1到i节点
    Cruise_route = []
    for i in range(len(charge_nodes)):
        route = astar.planning(charge_nodes[(i-1)%3].posX, charge_nodes[(i-1)%3].posY, charge_nodes[i].posX,charge_nodes[i].posY)
        Cruise_route.append(route) 
    for agent in agents:
        agent.Cruise_routes = Cruise_route
    
    '''
    主循环，频率为1Hz
    '''
    # 仿真步长1s：15min
    T_loop = 1.0
    iter_num = 0

    # 在航率
    service_hour = 0
    service = {'service_rate_Total': [], 'service_rate_RT': []}
        
    time_text_list = []
    while True:
        # 检查是否应该停止仿真
        delta_service_hour = 0
        if stop_check and stop_check():
            logging.info("收到停止信号，正在停止仿真...")
            break
            
        t_start = time.time()

        iter_num += 1
        time_text = f'{iter_num//4}:{iter_num%4*15:02d}'
        time_text_list.append(time_text)
#        if iter_num > 24*7*4:
#           break
        logging.info("***************************Iteration {}: {}***************************".format(iter_num, time_text))
        # 先更新agents
        for idx in range(len(agents)):
            # if idx !=1:
            #     continue
            agents[idx].step()
            if agents[idx].state == CRUISE:
                service_hour += 0.25
                delta_service_hour += 0.25
            # break
        
        # 分配充电,遍历每个节点
        for i in range(len(CN_statying_agents)):
            # 先来后到充电，充电功率均分
            charge_agent_num = min(len(CN_statying_agents[i]), 3)
            if charge_agent_num == 0: # 防止除数为0
                continue
            # 以100kW为最小分辨率
            max_CP_allow = math.floor(charge_nodes[i].ESS/100) * 100
            allocate_power_list = allocate_power(max_CP_allow, charge_agent_num)
            for agent in agents:
                if agent.id in CN_statying_agents[i][:charge_agent_num]:
                    idx = CN_statying_agents[i].index(agent.id)
                    agent.ChargingP = allocate_power_list[idx]
                    # 更新节点端充电功率
                    agent.CN_list[i].ESSPOut += agent.ChargingP
                    agent.Bat += agent.ChargingP * 0.25
                    if agent.Bat >= min(agent.BatteryCap * 0.9, agent.nextSegCost+agent.BatteryCap*0.05):
                        agent.state = CRUISE
                        agent.ChargingP = 0
                        CN_statying_agents[i].remove(agent.id)
                    logging.info(f"Agent {agent.id} charging at CN {i}: {agent.ChargingP} kW, battery: {agent.Bat:.2f} kWh ({agent.Bat/agent.BatteryCap*100:.2f}%)")
        
        
        
        # 更新CN
        for idx in range(len(charge_nodes)):
            charge_nodes[idx].step(iter_num)
        
        service_rate = round(service_hour / (iter_num * len(agents) * 0.25) *100,2)
        service['service_rate_Total'].append(service_rate)
        service['service_rate_RT'].append(round(delta_service_hour/(len(agents)*0.25)*100,2))
        
        '''
        打印参数
        '''
        posInfo = [[agent.posX,agent.posY,agent.v,agent.Bat] for agent in agents]
        CNInfo = [[cn.posX,cn.posY,cn.ESS, cn.WindP[math.floor(iter_num/4)], cn.SolarP[math.floor(iter_num/4)], round(cn.ESS/cn.ESSCap*100,2)] for cn in charge_nodes]
        CNCurve = [
            [[time_text_list[-96:], cn.ESS_list[-96:]],
              [time_text_list[-96:], cn.windp_list[-96:]],
              [time_text_list[-96:], cn.solarp_list[-96:]],
              [time_text_list[-96:],cn.pout_list[-96:]]]
            for cn in charge_nodes]
        service_rate_curve = [
            [time_text_list[-96:], service['service_rate_Total'][-96:]],
            [time_text_list[-96:], service['service_rate_RT'][-96:]]
                               ] 
        
        data_to_send = {
            "posInfo": posInfo,
            "CNInfo": CNInfo,
            "time": time_text,
            'service_rate': '{}%'.format(service_rate),
            'CNCurve': CNCurve,
            'service_rate_curve': service_rate_curve,
            'obs_info': obs_info,
        }
        print(np.array(CNCurve).tolist())
        # logging.info(">>>>>>>>>>push_to_frontend<<<<<<<<<<",json.dumps(data_to_send))
        push_to_frontend(data_to_send)
        logging.info(f"Service rate: {service_rate:.2f} %")
        logging.info('') # 打印空行
        elapsed = time.time() - t_start
        time.sleep(max(0, T_loop - elapsed))
    
    logging.info("Simulation ended.")
    pd.DataFrame(service).to_csv('service_rate_improve.csv',index=False)
    ESS_dict = {}
    for cn in charge_nodes:
        ESS_dict[cn.id] = cn.ESS_list
    pd.DataFrame(ESS_dict).to_csv('ESS.csv',index=False)

if __name__ == "__main__":
    start_simulation_improve()
