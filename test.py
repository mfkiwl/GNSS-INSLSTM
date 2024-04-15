'''
  ******************************************************************************
  * @file           : test.py
  * @author         : ZhangKai
  * @date           : 2024/3/31
  * @description    : 
  ******************************************************************************
'''

import scipy.signal as signal
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


if __name__ == '__main__':

    delta_velocity = np.array([0.00298, -0.00092, 0.00253])
    velocity=[]
    delta_theta_x = []
    delta_theta_y = []
    delta_theta_z = []
    xx = []
    yy = []
    dt = 0.005

    data = np.genfromtxt("datasets/ICM20602/ICM20602.txt", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z-g]"""
    print(data.shape)
    num=np.arange(0,len(data))
    for i in range(0, len(data)):
        delta_theta_x.append(data[i][1])
        delta_theta_y.append(data[i][2])
        delta_theta_z.append(data[i][3])
        delta_velocity = data[i][4:7]
        delta_velocity[2] += 9.801 * dt
        velocity.append(np.linalg.norm(delta_velocity))

    y_med = signal.medfilt(delta_theta_z, kernel_size=7)



    # plt.plot(num,delta_theta_z,"b")
    plt.plot(num,y_med,"c")
    plt.show()
