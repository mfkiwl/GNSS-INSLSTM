'''
  ******************************************************************************
  * @file           : GNSS_INS.py
  * @author         : ZhangKai
  * @date           : 2024/4/21
  * @description    : 
  ******************************************************************************
'''
import math

import numpy as np
import matplotlib.pyplot as plt

g = 9.79338  # 武汉的g
v = np.zeros(3, dtype=np.float64)
vg = np.zeros(3, dtype=np.float64)
pp = np.zeros(3, dtype=np.float64)

Q = np.identity(3, dtype=np.float64) * 0.02
R = np.identity(3, dtype=np.float64) * 0.1
H = np.diag([1.0, 1.0, 1.0])

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

dt = 0.005

def azimuth(lat1, lon1, lat2, lon2):
    # 将经纬度转换为弧度
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # 计算方位角
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    azimuth_rad = math.atan2(y, x)

    # 将弧度转换为角度
    azimuth_deg = math.degrees(azimuth_rad)

    # 调整角度范围在[0, 360)之间
    azimuth_deg = (azimuth_deg + 360) % 360

    return azimuth_deg

if __name__ == '__main__':

    # 读取数据
    data = np.genfromtxt("../datasets/ICM20602/ICM20602.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth = np.genfromtxt("../datasets/ICM20602/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    data_GNSS = np.genfromtxt("../datasets/GNSS RTK/GNSS_RTK.txt", dtype=float)  # 将文件中数据加载到data数组里

    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z]"""
    print(data.shape)
    xx = []
    yy = []
    zz = []
    outputdata = []
    outputdata1 = []

    x_ba_k1 = np.array([1.40664, 0.38285, 276.51564]) * np.pi / 180.0
    x_ba_k = x_ba_k1

    count_truth = 0
    count_GNSS = 0

    # =============================================================================================
    # 真值展开
    yaw_0 = data_truth[:, 10]

    for i in range(1, len(yaw_0)):
        # if i == 50000:
        #     yaw_0[i:] = yaw_0[i:] + 360
        if yaw_0[i] - yaw_0[i - 1] > 220:
            yaw_0[i] = yaw_0[i] - 360
        if yaw_0[i] - yaw_0[i - 1] < -220:
            yaw_0[i] = yaw_0[i] + 360
    # =========================================================================================================
        # GNSS
    yaw_GNSS = 270
    yaw_G=[270]
    print("GNSS",len(data_GNSS))
    if not (data_GNSS[count_GNSS][1] == data_GNSS[count_GNSS + 1][1] and data_GNSS[count_GNSS][2] ==
            data_GNSS[count_GNSS + 1][2]):
        yaw_GNSS = azimuth(data_GNSS[count_GNSS][1], data_GNSS[count_GNSS][2], data_GNSS[count_GNSS + 1][1],
                           data_GNSS[count_GNSS + 1][2])
        yaw_G.append(yaw_GNSS)

    for i in range(0, len(data_GNSS)-1):

        if not (data_GNSS[i][1] == data_GNSS[i + 1][1] and data_GNSS[i][2] ==
                data_GNSS[i + 1][2]):
            yaw_GNSS = azimuth(data_GNSS[i][1], data_GNSS[i][2],
                               data_GNSS[i + 1][1], data_GNSS[i + 1][2])

            yaw_G.append(yaw_GNSS)
        print(len(yaw_G))
        for i in range(1, len(yaw_G)):
            if yaw_G[i] - yaw_G[i - 1] > 220:
                yaw_G[i] = yaw_G[i] - 360
            if yaw_G[i] - yaw_G[i - 1] < -220:
                yaw_G[i] = yaw_G[i] + 360
    # =========================================================================================================


    # =============================================================================================

    for i in range(0, len(data)):
    # for i in range(0, 30000):
        if not i == 0:
            dt = data[i][0] - data[i - 1][0]

        time_IMU = data[i][0]
        time_truth = data_truth[count_truth][1]

        # =========================================================================================================
        # 真值时间对齐
        while np.abs(time_IMU - time_truth) > np.abs(time_IMU - data_truth[count_truth + 1][1]):
            count_truth += 1
            time_truth = data_truth[count_truth][1]
            if count_truth == len(data_truth) - 1:
                break
            # print(time_IMU,time_truth)
        # =============================================================================================
        if  count_GNSS < len(data_GNSS) -2 :
            if time_IMU > data_GNSS[count_GNSS + 1][0]:
                count_GNSS += 1

            k=(yaw_G[count_GNSS+1]-yaw_G[count_GNSS])/(data_GNSS[count_GNSS +1][0]-data_GNSS[count_GNSS ][0] )

            yaw_GNSS = yaw_G[count_GNSS] + k * (time_IMU - data_GNSS[count_GNSS][0])


        # =============================================================================================
        GYRO = np.array([data[i][1], data[i][2], data[i][3]]).T

        ACC = np.array([data[i][4], data[i][5], data[i][6]])

        # 由角速度计算出的状态量
        G = np.array([
            [1, np.sin(x_ba_k1[1]) * np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]),
             np.sin(x_ba_k1[1]) * np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])],
            [0, np.cos(x_ba_k1[0]), -np.sin(x_ba_k1[0])],
            [0, np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]), np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])]
        ])

        # =============================================================================================
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
        z_k[2] = yaw_GNSS / 57.3
        # 更新状态量
        x_ba_k = x_ba_k_ + K_k @ (z_k - H @ x_ba_k_)
        # 更新循环量
        p_k1 = p_k
        x_ba_k1 = x_ba_k
        # =============================================================================================

        outputdata.append(
            np.array([time_IMU, time_truth, yaw_GNSS, x_ba_k[2] * 57.3 , data_truth[i + 1][10]]))
        outputdata1.append(
            np.array([time_IMU, yaw_G[count_GNSS-1], data[i][1], data[i][2], data[i][3],data[i][4], data[i][5], data[i][6], data_truth[i + 1][8], data_truth[i + 1][9], data_truth[i + 1][10]]))

        print(i)

    data_output_o = np.array(outputdata)

    xx = data_output_o[:, 2]
    yy = data_output_o[:, 3]
    zz = data_output_o[:, 4]
    np.savetxt("output_KF_GNSS.txt", outputdata, delimiter=" ")
    np.savetxt("output_verify.txt", outputdata1, delimiter=" ")
    """[  roll_KF,pitch_KF, yaw_KF,roll_truth,pitch_truth,yaw_truth]"""
    plt.plot(xx, color='g')
    plt.plot(yy, color='r')
    plt.plot(zz, color='b')
    plt.show()

    # break
