'''
  ******************************************************************************
  * @file           : IMU_i300_KF.py
  * @author         : ZhangKai
  * @date           : 2024/3/30
  * @description    : 
  * 数据使用工业级IMU可获得效果良好的卡尔曼滤波结果，
  ******************************************************************************
'''
import numpy as np
import matplotlib.pyplot as plt

Q = np.identity(3, dtype=np.float64)*0.02
R = np.identity(3, dtype=np.float64)*0.1
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

dt = 0.005
# B = B * dt

if __name__ == '__main__':

    # 读取数据
    data = np.genfromtxt("datasets/i300/i300.txt", dtype=float)  # 将文件中数据加载到data数组里
    data_truth = np.genfromtxt("datasets/i300/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z]"""
    print(data.shape)
    yy = []
    outputdata=[]
    num = np.arange(0, len(data))
    x_ba_k1 = np.array([1.40664, 0.38285, 276.51564]) * np.pi / 180.0

    for i in range(0, len(data)):
    # for i in range(0, 1000):
        try:
            dt = data[i][0] - data[i - 1][0]
        except:
            print("error")

        GYRO = np.array([data[i][1], data[i][2], data[i][3]]).T

        ACC = np.array([data[i][4], data[i][5], data[i][6]])
        # 由角速度计算出的状态量
        G = np.array([
            [1, np.sin(x_ba_k1[1]) * np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]),
             np.sin(x_ba_k1[1]) * np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])],
            [0, np.cos(x_ba_k1[0]), -np.sin(x_ba_k1[0])],
            [0, np.sin(x_ba_k1[0]) / np.cos(x_ba_k1[1]), np.cos(x_ba_k1[0]) / np.cos(x_ba_k1[1])]
        ])

        # print(G )
        # B = Bb * dt
        # print(B)
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
        # x_ba_k = x_ba_k_
        # x_ba_k = x_ba_k % (2 * np.pi)
        # 更新循环量
        p_k1 = p_k
        x_ba_k1 = x_ba_k
        # print("=============================")
        # print(x_ba_k * 57.3)
        outputdata.append(np.concatenate((x_ba_k* 57.3%360, np.array([data_truth[i+1][8],data_truth[i+1][9],data_truth[i+1][10]]))))
        print(i)


        # xx.append((x_ba_k[0] * 57.3))
        yy.append((x_ba_k[2] * 57.3)%360)
    np.savetxt("outputData/output_i300_KF_rpy.txt", outputdata, delimiter=" ")
    """[  roll_KF,pitch_KF, yaw_KF,roll_truth,pitch_truth,yaw_truth]"""
    # plt.plot(xx, color='g')
    plt.plot(yy, color='r')
    plt.show()

    # break
