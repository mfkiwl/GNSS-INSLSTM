'''
  ******************************************************************************
  * @file           : KF_test.py
  * @author         : ZhangKai
  * @date           : 2024/3/31
  * @description    : 利用卡尔曼滤波来对数据滤波，数据来源为模拟值
  ******************************************************************************
'''

import numpy as np
import matplotlib.pyplot as plt


Q = np.diag([0.002, 0.002, 0.002])
R = np.diag([0.2, 0.2, 0.2])
H = np.diag([1.0, 1.0, 0.0])

A = np.identity(3, dtype=np.float64)
B = np.identity(3, dtype=np.float64)
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

dt =0.01
B = B * dt
# def KF_eigen(time)


if __name__ == "__main__":
    print("start")
    # 读取每行以空格隔开的txt数据文件
    with open('datasets/data.txt', 'r') as file:
        # 读取所有行并处理每一行数据
        data1 = []
        for line in file:
            # 去除行尾的换行符，然后按照逗号分割并转换为浮点数
            values = [float(value) for value in line.strip().split(',')]
            # 将处理后的行添加到数据列表中
            data1.append(values)

    # 读取数据
    # data = np.genfromtxt("datasets/data.txt", dtype=float)  # 将文件中数据加载到data数组里
    """[Time,ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,Roll,Pitch,Yaw]"""
    data = np.array(data1)
    print(data.shape)
    print(data[1])
    num = np.arange(0, len(data))
    # x_ba_k1 = np.array([1.061163, -0.621778, 0.000000] )* np.pi / 180.0

    for i in range(0, len(data)):
        # 由角速度计算出的状态量
        G = np.array([
            [1, np.sin(x_ba_k1[1]) * np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]),
             np.sin(x_ba_k1[1]) * np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])],
            [0, np.cos(x_ba_k1[0]), -np.sin(x_ba_k1[0])],
            [0, np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]), np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])]
        ])
        GYRO = np.array([data[i][4], data[i][5], data[i][6]]).T
        u_k = G @ GYRO

        # 计算状态外推方程
        x_ba_k_ = A @ x_ba_k1 + B @ u_k
        # 计算先验误差协方差外推方程
        p_k_ = A @ p_k1 @ A.T + Q
        # 计算卡尔曼增益
        K_k = p_k_ @ H.T @ np.linalg.inv(H @ p_k_ @ H.T + R)
        # 更新协方差
        p_k = (Ii - K_k @ H) @ p_k_
        # 由加速度计算姿态角
        z_k[0] = np.arctan(data[i][2] / data[i][3])
        z_k[1] = -np.arctan(data[i][1] / np.linalg.norm(data[i][2] + data[i][3]))
        z_k[2] = 0.0
        # 更新状态量
        x_ba_k = x_ba_k_ + K_k @ (z_k - H @ x_ba_k_)
        # 更新循环量
        p_k1 = p_k
        x_ba_k1 = x_ba_k
        print("=============================")
        print(x_ba_k *57.3)
        print(data[i][7:10])
        # break