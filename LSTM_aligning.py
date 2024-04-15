'''
  ******************************************************************************
  * @file           : LSTM_aligning.py
  * @author         : ZhangKai
  * @date           : 2024/4/6
  * @description    : 将GNSS IMU数据 真实数据进行时间对齐，真值取近，GNSS线性插值，IMU进行卡尔曼滤波得到姿态角，GNSS进行坐标系转换，
  * @output         : outputData/output_aligning.txt
                    [ seconds of week, roll_KF, Pitch_KF, Yaw_KF, latitude, longitude,roll_truth, Pitch_truth, Yaw_truth]
  ******************************************************************************
'''
import numpy as np
import winsound
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

    data_output = []
    data_aligning = np.zeros(9)

    """[ seconds of week, Omega_x, Omega_y,Omega_z, latitude, longitude,roll, Pitch, Yaw,]"""
    count_truth = 0
    count_GNSS = 0
    # mkt=
    k_lat_gnss = (data_GNSS[count_GNSS + 1][1] - data_GNSS[count_GNSS][1]) / (
            data_GNSS[count_GNSS + 1][0] - data_GNSS[count_GNSS][0])
    k_lon_gnss = (data_GNSS[count_GNSS + 1][2] - data_GNSS[count_GNSS][2]) / (
            data_GNSS[count_GNSS + 1][0] - data_GNSS[count_GNSS][0])
    print(data_GNSS[count_GNSS][0])

    x_ba_k1 = np.array([1.40679  ,  0.38469 ,   276.51585 ]) * np.pi / 180.0

    for i in range(0, len(data_IMU)):
        # for i in range(0, 420):

        time_IMU = data_IMU[i][0]
        time_truth = data_truth[count_truth][1]
        # =========================================================================================================
        # 真值时间对齐
        print(count_truth)

        while np.abs(time_IMU - time_truth) > np.abs(time_IMU - data_truth[count_truth + 1][1]):
            count_truth += 1
            time_truth = data_truth[count_truth][1]
            if  count_truth==len(data_truth)-1:
                break
        # print(time_IMU,time_truth)
        # =========================================================================================================
        # GNSS时间对齐
        if not count_GNSS == len(data_GNSS)-1:
            if time_IMU > data_GNSS[count_GNSS + 1][0]:

                count_GNSS += 1
                # print(data_GNSS[count_GNSS][0])
                try:
                    k_lat_gnss = (data_GNSS[count_GNSS + 1][1] - data_GNSS[count_GNSS][1]) / (
                            data_GNSS[count_GNSS + 1][0] - data_GNSS[count_GNSS][0])
                    k_lon_gnss = (data_GNSS[count_GNSS + 1][2] - data_GNSS[count_GNSS][2]) / (
                            data_GNSS[count_GNSS + 1][0] - data_GNSS[count_GNSS][0])
                except:
                    print("====================================================================")
                    print(count_GNSS)
        lat = data_GNSS[count_GNSS][1] + k_lat_gnss * (data_IMU[i][0] - data_GNSS[count_GNSS][0])
        lon = data_GNSS[count_GNSS][2] + k_lon_gnss * (data_IMU[i][0] - data_GNSS[count_GNSS][0])
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
        x_ba_k1 = x_ba_k
        # =========================================================================================================
        # 输出
        data_aligning[0] = data_IMU[i][0]
        data_aligning[1:4] = (x_ba_k * 180/np.pi )% 360
        data_aligning[4:6] = np.array([lat, lon])
        data_aligning[6:9] = data_truth[count_truth][8:11]
        data_output.append(data_aligning.copy())

        # print(data_aligning[0])
    np.savetxt("outputData/output_aligning_ICM20602.txt", data_output, delimiter=" ")

    winsound.Beep(1000, 2000)
