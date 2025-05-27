import pandas as pd
import numpy as np
from Astar_formal import nm2m
from matplotlib import pyplot as plt
import math
import logging

file_path = 'CN_init.csv'
'''
id,posX,posY,风机规格,切入风速,额定风速,切出风速,光伏规格
0,100,50,2000kw,3,15,25,20kw
'''

class ChargeNode:
    def __init__(self, init_df:pd.DataFrame):
        self.id = init_df['id']
        self.posX = init_df['posX']
        self.posY = init_df['posY']
        self.wind_spec = init_df['风机规格']  # 风机规格,kW
        self.cut_in_wind_speed = init_df['切入风速']  # 切入风速
        self.rated_wind_speed = init_df['额定风速']  # 额定风速
        self.cut_out_wind_speed = init_df['切出风速']  # 切出风速
        self.solar_spec = init_df['光伏规格']  # 光伏规格,kW
        
        # 储能容量按新能源容量的20%计算,输出功率按两小时计算
        self.ESSCap = (self.wind_spec+self.solar_spec) * 2
        self.ESSPOutUpper = self.ESSCap * 0.5  # kW
        self.ESSPOut = 0
        self.ESS = self.ESSCap * 0.5  # kWh,初始化储能电量为50%
        self.ESS_list = []
        self.windp_list = []
        self.solarp_list = []
        self.pout_list = []
        
        self.ChargingP_Pending_list = [100, 300, 500] # kW
                
    def inputEnergydata(self, WindAndSolarFile):
        df = pd.read_csv(WindAndSolarFile,header=10)
        # print(df)
        self.WS = np.array(df['WS50M'].tolist())
        self.SFC = np.array(df['ALLSKY_SFC_SW_DWN'].tolist())
        # 计算出力功率，2MW级风机P=0.5*rho*v^3*S*Cp，rho取1.22kg/m3，Cp取0.5，单位为W，S=pi*r^2，r取50m，切入风速3，切出22
        Cp=0.25
        rho=1.22
        r=50
        # 转化为kW单位，由于是2MW级风机参数，还需多除1个2000
        self.WindP = np.where(self.WS < self.cut_in_wind_speed, 0, 0.5*rho*self.WS**3*np.pi*r**2*Cp/1e3 /2000 * self.wind_spec)
        # self.WindP = np.where(self.WS100M < 3, 0, 0.5*rho*self.WS100M**3*np.pi*r**2*Cp/1e6)
        # 超过额定风速输出功率稳定
        self.WindP = np.where(self.WS > self.rated_wind_speed+0.1, 0.5*rho*self.rated_wind_speed**3*np.pi*r**2*Cp/1e3 /2000* self.wind_spec * (1.1 - 0.1 * np.exp(-0.2*(self.WS-self.rated_wind_speed))), self.WindP)
        self.WindP = np.where(self.WS > self.cut_out_wind_speed, 0, self.WindP)
        
        # 光伏，P=SFC*PTotal/ratedradian*eta，eta取0.85，光伏板尺寸0.228*0.113m，功率550W
        eta = 0.85
        self.SolarP = self.SFC /1000 * np.sum(self.solar_spec) * eta
    
    def step(self, iternum):
        '''
        充电节点每15min更新一次，计算储能电量和功率
        iternum: 迭代次数,用于计算时间
        '''
        # 计算储能电量和功率
        idx = math.floor(iternum/4) # 15min为一个迭代周期，每小时4个迭代周期

        self.ESS += (self.WindP[idx] + self.SolarP[idx] - self.ESSPOut) * 0.25
        if self.ESS > self.ESSCap:
            self.ESS = self.ESSCap
            logging.warning(f'ChargeNode {self.id} ESS is Fully charged!')
            logging.info(f'ChargeNode {self.id} ESS: {self.ESS:.2f} kWh ({self.ESS/self.ESSCap*100:.2f}%), Wind Power: {self.WindP[idx]:.2f} kW, Solar Power: {self.SolarP[idx]:.2f} kW')
        elif self.ESS < 0:
            self.ESS = 0
            logging.warning(f'ChargeNode {self.id} ESS is empty!')
        else:
            logging.info(f'ChargeNode {self.id} ESS: {self.ESS:.2f} kWh ({self.ESS/self.ESSCap*100:.2f}%), Wind Power: {self.WindP[idx]:.2f} kW, Solar Power: {self.SolarP[idx]:.2f} kW')
        
        self.ESS_list.append(self.ESS)
        self.windp_list.append(self.WindP[idx])
        self.solarp_list.append(self.SolarP[idx])
        self.pout_list.append(self.ESSPOut)
        # 输出功率置0
        self.ESSPOut = 0
    
    def plotPowerCurve(self):
        # Wind Power Curve
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(list(range(len(self.WindP))), self.WindP, label='Wind Power', color='blue')
        ax.set_xlabel('Wind Speed (m/s)')
        ax.set_ylabel('Power Output (kW)')
        ax.set_title('Wind Power Curve')
        ax.grid()
        plt.legend()
        plt.show()
        
        # Solar Power Curve
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(list(range(len(self.SolarP))), self.SolarP, label='Solar Power', color='orange')
        ax.set_xlabel('Solar Radiation (W/m^2)')
        ax.set_ylabel('Power Output (kW)')
        ax.set_title('Solar Power Curve')
        ax.grid()
        plt.legend()
        plt.show()

def getCN():
    '''
    从file_path中读取数据，返回ChargeNode组成的list
    '''
    folder = 'new/WindAndSolarData/'
    filename = ['ysj.csv', 'mjj.csv', 'zbd.csv']
    df = pd.read_csv(file_path)
    cn_list = []
    for i in range(len(df)):
        # print(df.iloc[i])
        cn = ChargeNode(df.iloc[i])
        cn.inputEnergydata(folder + filename[i])
        cn_list.append(cn)
    
    return cn_list

if __name__ == "__main__":
    # # Read the CSV file into a DataFrame
    # df = pd.read_csv(file_path)
    # # Create an instance of ChargeNode
    # charge_node = ChargeNode(df.iloc[0])
    # # Input energy data from the specified file
    # charge_node.inputEnergydata('new/WindAndSolarData/zbd.csv')
    # # Plot the power curves
    # charge_node.plotPowerCurve()
    CN_list = getCN()
        
        
        