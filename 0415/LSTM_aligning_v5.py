'''
  ******************************************************************************
  * @file           : LSTM_aligning_v2.py
  * @author         : ZhangKai
  * @date           : 2024/4/14
  * @description    : 仅训练yaw数据进行预测，将所有数据的范围扩展，解决了0-360处的数据问题
                        输出时间以及三个yaw，数据进行切片输出，目前仅输出0——300(GNSS)的数据

  ******************************************************************************
'''
import math

import numpy as np
import winsound


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
x_sum = np.zeros(3, dtype=np.float64)
x_output = np.zeros(9, dtype=np.float64)
dt = 0.005
if __name__ == '__main__':
    data_IMU = np.genfromtxt("datasets/ICM20602/ICM20602.txt", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z-g]"""
    data_truth = np.genfromtxt("datasets/ICM20602/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    """[ week , seconds of week,velocity_n,velocity_e, velocity_d, roll, Pitch, Yaw]"""
    data_GNSS = np.genfromtxt("datasets/GNSS RTK/GNSS_RTK.txt", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, latitude, longitude, height,lat_SD,lon_SD, height_SD]"""
    print(f"IMU:{data_IMU.shape},Truth:{data_truth.shape},GNSS:{data_GNSS.shape}")
    # print(f"IMU:{data_IMU.shape},Truth:{data_truth.shape}")
    # =========================================================================================================
    # 真值展开

    Records = False

    yaw_0 = data_truth[:, 10]
    print(yaw_0.shape)

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
    # =========================================================================================================

    data_output = []
    data_aligning = np.zeros(4)

    """[ seconds of week,gx, gy, gz, vx,vy,vz,latitude, longitude,roll, Pitch, Yaw,]"""
    count_truth = 0
    count_GNSS = 0
    # mkt=
    yaw_GNSS = 270
    if not (data_GNSS[count_GNSS][1] == data_GNSS[count_GNSS + 1][1] and data_GNSS[count_GNSS][2] ==
            data_GNSS[count_GNSS + 1][2]):
        yaw_GNSS = azimuth(data_GNSS[count_GNSS][1], data_GNSS[count_GNSS][2], data_GNSS[count_GNSS + 1][1],
                           data_GNSS[count_GNSS + 1][2])

    # gnss获得的偏航角数据

    # print(data_GNSS[count_GNSS][0])

    x_ba_k1 = np.array([1.40679, 0.38469, 276.51585]) * np.pi / 180.0

    for i in range(0, len(data_IMU)):
        # for i in range(0, 420):

        time_IMU = data_IMU[i][0]
        time_truth = yaw_0[count_truth]
        # =========================================================================================================
        # 真值时间对齐
        # 此处只使用time更新count_truth，输出时取数据
        # print(count_truth)
        while np.abs(time_IMU - time_truth) > np.abs(time_IMU - data_truth[count_truth + 1][1]):
            count_truth += 1
            time_truth = data_truth[count_truth][1]
            if count_truth == len(data_truth) - 1:
                break
        # print(time_IMU,time_truth)
        # =========================================================================================================
        # GNSS时间对齐

        if not count_GNSS == len(data_GNSS) - 1:
            if time_IMU > data_GNSS[count_GNSS + 1][0]:

                count_GNSS += 1
                # print(data_GNSS[count_GNSS][0])
                try:
                    if not (data_GNSS[count_GNSS][1] == data_GNSS[count_GNSS + 1][1] and data_GNSS[count_GNSS][2] ==
                            data_GNSS[count_GNSS + 1][2]):
                        yaw_GNSS = azimuth(data_GNSS[count_GNSS][1], data_GNSS[count_GNSS][2],
                                           data_GNSS[count_GNSS + 1][1], data_GNSS[count_GNSS + 1][2])
                except:
                    print("====================================================================")
                    print(count_GNSS)

        # =========================================================================================================
        # 卡尔曼滤波求解姿态角
        GYRO = np.array([data_IMU[i][1], data_IMU[i][2], data_IMU[i][3]]).T
        x_sum += GYRO
        ACC = np.array([data_IMU[i][4], data_IMU[i][5], data_IMU[i][6]])
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

        x_ba_k1[0:2] = data_truth[count_truth][8:10] * np.pi / 180.0
        x_ba_k1[2] = yaw_0[count_truth] * np.pi / 180.0

        if count_GNSS == 930:
            Records = True
        if count_GNSS == 1311:
            break

        # =========================================================================================================
        # 输出
        if Records:
            print(i)
            data_aligning[0] = data_IMU[i][0]  # time
            data_aligning[1] = yaw_GNSS
            data_aligning[2] = x_ba_k[2] * 57.3
            data_aligning[3] = yaw_0[count_truth]
            data_output.append(data_aligning.copy())
            """[ seconds of week,gx, gy, gz, vx,vy,vz,latitude, longitude,roll, Pitch, Yaw,]"""
            # print(data_aligning[0])

        # =========================================================================================================
        # 真值展开
    data_output_o = np.array(data_output)
    yaw_G = data_output_o[:, 1]
    # print(yaw_G.shape)

    for i in range(1, len(yaw_G)):
        # if i == 50000:
        #     yaw_G[i:] = yaw_G[i:] + 360

        if yaw_G[i] - yaw_G[i - 1] > 320:
            yaw_G[i] = yaw_G[i] - 360
            # print("u", i)
        #         break
        # for i in range(1, len(yaw_0)):
        if yaw_G[i] - yaw_G[i - 1] < -320:
            yaw_G[i] = yaw_G[i] + 360
            # print("d", i)
    data_output_o[:, 1] = yaw_G
    # =========================================================================================================
    np.savetxt("outputData/data_v4_6003.txt", data_output_o, delimiter=" ")

    winsound.Beep(1000, 2000)
