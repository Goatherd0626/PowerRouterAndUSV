import pandas as pd
import numpy as np
from Astar_formal import nm2m
from Island import getCN,ChargeNode
import math
import logging
import os

TYPE_CN = 1 # 目标点为充电站类

CRUISE = 0 # 巡航状态
CHARGING = 1 # 充电状态
STAY = 2 # 停留状态
MISSION = 3 # 任务状态
DROP= 4 # 电量耗尽状态

file_path = 'USV_init.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(file_path)
'''
id,posX,posY,maxv,w,tarCNid
0,100,50,5,0,1
df格式如上，读取并存储为class
'''

class Agent:
    def __init__(self, init_df:pd.DataFrame, ChargeNodes:list[ChargeNode]):
        self.id = init_df['id']
        self.posX = init_df['posX']
        self.posY = init_df['posY']
        self.maxV = init_df['maxv'] * nm2m/1000  # 速度，km/h
        self.w = init_df['w']  # angular speed
        self.tarCNid = init_df['tarCNid']  # target CN id
        self.v = 25 * nm2m/1000 # 标准20节巡航
        self.CN_list = ChargeNodes
        self.currentTar = [self.CN_list[self.tarCNid].posX, self.CN_list[self.tarCNid].posY, TYPE_CN]  # 目标充电站坐标
        
        self.BatteryCap = 10920 * 0.8 * 0.76  # 10920kg，0.8kWh/kg，0.76包装率
        self.Bat = self.BatteryCap * 0.8  # kWh, 初始化电量为80%
        
        self.astar = None
        self.routepoints = []  # 路径点列表，存储路径点坐标
        # 巡航线路,cruise_route[i]代表到达i节点的路线，即i-1到i节点
        self.Cruise_routes = None
        
        self.CN_staying = None # 充电节点停留船舶列表
        
        self.state = CRUISE  # 巡航状态
        self.ChargingP_Pending_list = [100, 300, 500] # kW
        self.ChargingP = 0 # kW

        
    def getBatCost(self):
        '''
        根据航速计算电池消耗，kWh/km
        '''
        Pall = 2060 # 总功率2060kW
        self.BatCost = Pall/(0.95*0.95)*math.pow(self.v,2)/math.pow(self.maxV,3)/0.8 # kWh/km
        
        return self.BatCost

    def navigation(self, goalxy):
        '''
        给定目标点[[gx, gy], ]和自身位置self.posX, self.posY，以及速度self.v，计算船舶下一个15min的位置
        '''
        gx, gy = goalxy[0]
        dx = gx - self.posX
        dy = gy - self.posY
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            # Already at the goal
            return distance / 1000 # 转换为km
        
        # Calculate the unit vector towards the goal
        unit_dx = dx / distance
        unit_dy = dy / distance
        
        # Calculate the distance the ship can travel in one hour
        travel_distance = self.v * 1000 * 0.25  # 乘1000转化为单位米,15 minutes travel distance
        
        # If the goal is within one hour's travel distance, move directly to the goal
        if travel_distance >= distance:
            self.posX = gx
            self.posY = gy
            return distance / 1000 # 转换为km
        else:
            # Otherwise, move along the direction of the goal
            self.posX += unit_dx * travel_distance
            self.posY += unit_dy * travel_distance
            return travel_distance / 1000 # 转换为km
    
    def navigation2(self, goalxys):
        '''
        给定目标点[[gx, gy], ]和自身位置self.posX, self.posY，以及速度self.v，计算船舶下一个15min的位置
        '''
        gx, gy = goalxys[0]
        dx = gx - self.posX
        dy = gy - self.posY
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            goalxys.pop(0)  # Remove the goal point if already reached
            gx, gy = goalxys[0]
            dx = gx - self.posX
            dy = gy - self.posY
            distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate the unit vector towards the goal
        unit_dx = dx / distance
        unit_dy = dy / distance
        
        # Calculate the distance the ship can travel in one hour
        travel_distance = self.v * 1000 * 0.25  # 乘1000转化为单位米,15 minutes travel distance
        
        # If the goal is within one hour's travel distance, move directly to the goal
        if travel_distance >= distance:
            self.posX = gx
            self.posY = gy
            goalxys.pop(0)  # Remove the goal point if already reached
            return distance / 1000 # 转换为km
        else:
            # Otherwise, move along the direction of the goal
            self.posX += unit_dx * travel_distance
            self.posY += unit_dy * travel_distance
            return travel_distance / 1000 # 转换为km

    def update_tar(self):
        self.tarCNid = (self.tarCNid+1) % 3 # 模3
        self.currentTar = [self.CN_list[self.tarCNid].posX, self.CN_list[self.tarCNid].posY, TYPE_CN]  # 目标充电站坐标
        # self.routepoints = self.astar.planning(self.posX, self.posY, *self.currentTar[:2])
        self.routepoints = self.Cruise_routes[self.tarCNid].copy()  # 更新路径点，使用copy()避免引用
        
    def check_charging(self):
        '''
        检查是否需要充电
        '''
        # 计算routepoints总长度，routepoints = [[px1,py1], [px2,py2], ...]
        routepoints_array = np.array(self.routepoints)
        segment_lengths = np.sqrt(np.sum(np.diff(routepoints_array, axis=0) ** 2, axis=1))
        total_length = np.sum(segment_lengths) / 1000
        self.nextSegCost = self.getBatCost() * total_length
        if self.getBatCost() * total_length > self.Bat - self.BatteryCap*0.1:
            return True
        else:
            return False
    
    def step(self):
        '''
        代理每15min更新一次，计算位置和电量
        '''
        if self.state == CRUISE:
            logging.info(f"Agent {self.id} is cruising to target CN {self.tarCNid}")
            # 计算位置和电量
            if self.routepoints == []:
                self.state = STAY
            else:
                travel_dis = self.navigation2(self.routepoints)
                # 计算电量
                self.Bat -= self.getBatCost() * travel_dis
            if self.Bat < 0:
                self.Bat = 0
                self.state = DROP
            else:
                logging.info(f"Agent {self.id} battery: {self.Bat:.2f} kWh ({self.Bat/self.BatteryCap*100:.2f}%)")
        
        if self.state == STAY:
            print('agent {} stay at CN{}'.format(self.id, self.tarCNid))
            self.update_tar()  # TODO: 此处已经更新下一次的astar路径点
            if self.check_charging():
                self.state = CHARGING
                print('agent {} need to charge'.format(self.id))
                if self.id not in self.CN_staying[self.tarCNid]:
                    self.CN_staying[self.tarCNid].append(self.id)
            else:
                print('agent {} no need to charge'.format(self.id))
                self.state = CRUISE
            # logging.info(f"Agent {self.id} is staying at CN {self.tarCNid}")
                        
        if self.state == CHARGING:
                
            # logging.info(f"Agent {self.id} is charging at CN {self.tarCNid}")
            pass
            
        if self.state == DROP:
            logging.warning(f"Agent {self.id} battery depleted!")
            pass

def getAgent():
    '''
    从file_path中读取数据，返回Agent组成的list
    '''
    CN_list = getCN()
    df = pd.read_csv(file_path)
    agents = []
    for i in range(len(df)):
        # if i > 2: # 控制agent数量
        #     break
        agents.append(Agent(df.iloc[i], CN_list))
    
    return agents
    
if __name__ == "__main__":
    agents = getAgent()
    for agent in agents:
        print(agent.id, agent.posX, agent.posY, agent.maxV, agent.w, agent.tarCNid)
        print(agent.getBatCost())    