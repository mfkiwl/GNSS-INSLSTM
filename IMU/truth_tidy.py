'''
  ******************************************************************************
  * @file           : truth_tidy.py
  * @author         : ZhangKai
  * @date           : 2024/4/15
  * @description    : 
  ******************************************************************************
'''

import numpy as np
from matplotlib import pyplot as plt

if __name__ == '__main__':

    data_truth = np.genfromtxt("../datasets/ICM20602/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    """[ week , seconds of week,velocity_n,velocity_e, velocity_d, roll, Pitch, Yaw]"""
    print(f"Truth:{data_truth.shape}")
    yaw_0 = data_truth[:, 10]
    print(yaw_0.shape)

    for i in range(1, len(yaw_0)):
        if i==50000:
            yaw_0[i:] = yaw_0[i:] + 360

        if yaw_0[i] - yaw_0[i - 1] > 200:
            yaw_0[i] = yaw_0[i] - 360
            print("u",i)
    #         break
    # for i in range(1, len(yaw_0)):
        if yaw_0[i] - yaw_0[i - 1] < -200:
            yaw_0[i] = yaw_0[i] + 360
            print("d",i)
            # break


    plt.plot(yaw_0)
    plt.show()
