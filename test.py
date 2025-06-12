from matplotlib import pyplot as plt
import numpy as np


Pall = 2060 # 总功率2060kW
# self.BatCost = Pall/(0.95*0.95)*math.pow(self.v,2)/math.pow(self.maxV,3)/0.8 # kWh/km
nm2m = 1.852
maxV=36*nm2m
x_list = []
y_list = []
z1_list = []
z2_list = []
Ebat = 10920*0.8*0.76
Ebat2 = 10920*0.8*0.76*2
for v in np.arange(15*nm2m,maxV,0.1):
    BatCost = Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8 # kWh/km
    print(BatCost)
    x_list.append(v)
    y_list.append(BatCost)
    z1_list.append(Ebat/BatCost)  # 充电站充电时间
    z2_list.append(Ebat2/BatCost)  # 充电站充电时间2倍
plt.plot(x_list, y_list)
# 画一条红色虚线，标明是25节航速能量消耗
plt.axvline(x=15*nm2m, color='gray', linestyle='--', label='15knots')
plt.axvline(x=25*nm2m, color='gray', linestyle='--', label='25knots')
plt.axvline(x=33*nm2m, color='gray', linestyle='--', label='33knots')
for v in np.array([15, 25, 33]) * nm2m:
    plt.axhline(y=Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8, color='gray', linestyle='--')
    plt.text(v-10, Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8, f'({v/nm2m:.1f}knots, {Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8:.1f}kWh/km)', fontsize=10, color='red', ha='left', va='bottom')
    # plt横虚线
    
plt.xlabel('speed (km/h)')
plt.ylabel('Energy Cost (kWh/km)')
plt.show()

plt.plot(x_list, z1_list, label='Range(800Wh/kg)')
plt.plot(x_list, z2_list, label='Range(1600Wh/kg)')
# plt.axvline(x=15*nm2m, color='gray', linestyle='--', label='15knots')
plt.axvline(x=25*nm2m, color='gray', linestyle='--')
plt.axvline(x=33*nm2m, color='gray', linestyle='--')
for v in np.array([25, 33]) * nm2m:
    plt.axhline(y=Ebat/(Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8), color='gray', linestyle='--')
    plt.text(v-10, Ebat/(Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8), f'({v/nm2m:.1f}knots, {Ebat/(Pall/(0.95*0.95)*np.power(v,2)/np.power(maxV,3)/0.8):.1f}km)', fontsize=10, color='red', ha='left', va='bottom')

plt.xlabel('speed (km/h)')
plt.ylabel('Range (km)')
plt.legend()
plt.show()