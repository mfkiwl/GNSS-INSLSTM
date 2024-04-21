'''
  ******************************************************************************
  * @file           : KF_imu.py
  * @author         : ZhangKai
  * @date           : 2024/4/20
  * @description    : 
  ******************************************************************************
'''

import numpy as np
import matplotlib.pyplot as plt


def KF(data, data_truth,time):
    g = 9.79338
    v = np.zeros(3, dtype=np.float64)
    vg = np.zeros(3, dtype=np.float64)
    pp = np.zeros(3, dtype=np.float64)

    Q = np.identity(3, dtype=np.float64) * 0.02
    R = np.identity(3, dtype=np.float64) * 0.1
    H = np.diag([1.0, 1.0, 0.0])
    A = np.identity(3, dtype=np.float64)
    Bb = np.identity(3, dtype=np.float64)
    Ii = np.identity(3, dtype=np.float64)
    x_ba_k1 = np.zeros(3, dtype=np.float64)
    x_ba_k = np.zeros(3, dtype=np.float64)
    x_ba_k_ = np.zeros(3, dtype=np.float64)
    p_k = np.identity(3, dtype=np.float64)
    p_k1 = np.zeros((3, 3), dtype=np.float64)
    p_k_ = np.zeros((3, 3), dtype=np.float64)
    u_k = np.zeros(3, dtype=np.float64)
    z_k = np.zeros(3, dtype=np.float64)
    K_k = np.zeros((3, 3), dtype=np.float64)

    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z]"""

    yaw_0 = data_truth[:, 10]
    # print(yaw_0.shape)

    for i in range(1, len(yaw_0)):
        if i == 50000:
            yaw_0[i:] = yaw_0[i:] + 360

        if yaw_0[i] - yaw_0[i - 1] > 200:
            yaw_0[i] = yaw_0[i] - 360
            # print("u", i)
        #         break
        # for i in range(1, len(yaw_0)):
        if yaw_0[i] - yaw_0[i - 1] < -200:
            yaw_0[i] = yaw_0[i] + 360
            # print("d", i)
            # break

    count_truth=0
    count_imu=0
    # print(data.shape)
    outputdata = []
    # num = np.arange(0, len(data))
    x_ba_k1 = np.array([1.40664, 0.38285, 276.51564]) * np.pi / 180.0
    x_ba_k = x_ba_k1


    # =========================================================================================================

    for i in range(0, 323150):
    # for i in range(0, 3000):

        time_0 = time[i]
        time_truth = data_truth[count_truth][1]
        time_imu0= data[count_imu][1]

    # =========================================================================================================
        # 真值时间对齐
        # print(count_truth)

        while np.abs(time_0 - time_truth) > np.abs(time_0 - data_truth[count_truth + 1][1]):
            count_truth += 1
            time_truth = data_truth[count_truth][1]
            if count_truth == len(data_truth) - 1:
                break
            # print(time_IMU,time_truth)




        if not i == 0:
            dt = data[i][0] - data[i - 1][0]


        GYRO = np.array([data[i][1], data[i][2], data[i][3]]).T

        ACC = np.array([data[i][4], data[i][5], data[i][6]])

        # 由角速度计算出的状态量
        G = np.array([
            [1, np.sin(x_ba_k1[1]) * np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]),
             np.sin(x_ba_k1[1]) * np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])],
            [0, np.cos(x_ba_k1[0]), -np.sin(x_ba_k1[0])],
            [0, np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]), np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])]
        ])

        u_k = G @ GYRO

        # 计算状态外推方程
        x_ba_k_ = A @ x_ba_k1 + Bb @ u_k
        # 计算先验误差协方差外推方程
        p_k_ = A @ p_k1 @ A.T + Q
        # 计算卡尔曼增益
        K_k = p_k_ @ H.T @ np.linalg.inv(H @ p_k_ @ H.T + R)
        # 更新协方差
        p_k = (Ii - K_k @ H) @ p_k_
        # 由加速度计算姿态角
        z_k[0] = np.arctan(ACC[1] / ACC[2])
        z_k[1] = -np.arctan(ACC[0] / np.linalg.norm(ACC[1] + ACC[2]))
        z_k[2] = 0.0
        # 更新状态量
        x_ba_k = x_ba_k_ + K_k @ (z_k - H @ x_ba_k_)
        # 更新循环量
        p_k1 = p_k
        x_ba_k1 = x_ba_k
        outputdata.append(np.array([x_ba_k[2] * 57.3 , yaw_0[count_truth]]))
        # outputdata.append(np.concatenate(
        #     (x_ba_k * 57.3 % 360, np.array([data_truth[i + 1][8], data_truth[i + 1][9], data_truth[i + 1][10]]))))
        print(i)
    return np.array(outputdata)


dt = 0.005

if __name__ == '__main__':
    # 读取数据
    data1 = np.genfromtxt("../datasets/ADIS16460/ADIS16460.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth1 = np.genfromtxt("../datasets/ADIS16460/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    print(data1.shape)

    data2 = np.genfromtxt("../datasets/ADIS16465/ADIS16465.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth2 = np.genfromtxt("../datasets/ADIS16465/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    print(data2.shape)

    data3 = np.genfromtxt("../datasets/i300/i300.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth3 = np.genfromtxt("../datasets/i300/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    print(data3.shape)

    data4 = np.genfromtxt("../datasets/ICM20602/ICM20602.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth4 = np.genfromtxt("../datasets/ICM20602/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    print(data4.shape)

    time = data4[:,0]


    out1 = KF(data1, data_truth1,time)
    print("out1 shape", out1.shape)
    out2 = KF(data2, data_truth2,time)
    print("out2 shape", out2.shape)
    out3 = KF(data3, data_truth3,time)
    print("out3 shape", out3.shape)
    out4 = KF(data4, data_truth4,time)
    print("out4 shape", out4.shape)

    result = np.concatenate((out1,out2,out3,out4), axis=1)
    print("shape", result.shape)
    np.savetxt("output__KF.txt", result, delimiter=" ")




