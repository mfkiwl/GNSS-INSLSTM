'''
  ******************************************************************************
  * @file           : IMU_quaternion.py
  * @author         : ZhangKai
  * @date           : 2024/3/31
  * @description    : 使用四元数进行姿态解算，四元数更新有问题
  ******************************************************************************
'''
import numpy as np
import matplotlib.pyplot as plt


def update_q(q, delta_theta):
    q_next = q + 0.5 * np.array([
        -delta_theta[0] * q[1] - delta_theta[1] * q[2] - delta_theta[2] * q[3],
        delta_theta[0] * q[0] + delta_theta[2] * q[2] - delta_theta[1] * q[3],
        delta_theta[1] * q[0] - delta_theta[2] * q[1] + delta_theta[0] * q[3],
        delta_theta[2] * q[0] + delta_theta[1] * q[1] - delta_theta[0] * q[2]
    ])
    q_next = q_next / np.linalg.norm(q_next)
    return q_next


def update_v(q, delta_velocity):
    velocity_next = np.array([
        delta_velocity[0] * (q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2) +
        delta_velocity[1] * 2 * (q[1] * q[2] - q[0] * q[3]) +
        delta_velocity[2] * 2 * (q[0] * q[2] + q[1] * q[3]),

        delta_velocity[0] * 2 * (q[1] * q[2] + q[0] * q[3]) +
        delta_velocity[1] * (q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2) +
        delta_velocity[2] * 2 * (q[2] * q[3] - q[0] * q[1]),

        delta_velocity[0] * 2 * (q[1] * q[3] - q[0] * q[2]) +
        delta_velocity[1] * 2 * (q[0] * q[1] + q[2] * q[3]) +
        delta_velocity[2] * (q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2),

    ])
    return velocity_next


def update_v2(q, delta_velocity):
    velocity_next = np.array([
        delta_velocity[0] * (1 - 2 * q[2] ** 2 - 2 * q[3] ** 2) +
        delta_velocity[1] * 2 * (q[1] * q[2] - q[0] * q[3]) +
        delta_velocity[2] * 2 * (q[0] * q[2] + q[1] * q[3]),

        delta_velocity[0] * 2 * (q[1] * q[2] + q[0] * q[3]) +
        delta_velocity[1] * (1 - 2 * q[1] ** 2 - 2 * q[3] ** 2) +
        delta_velocity[2] * 2 * (q[2] * q[3] - q[0] * q[1]),

        delta_velocity[0] * 2 * (q[1] * q[3] - q[0] * q[2]) +
        delta_velocity[1] * 2 * (q[0] * q[1] + q[2] * q[3]) +
        delta_velocity[2] * (1 - 2 * q[1] ** 2 - 2 * q[2] ** 2),

    ])
    return velocity_next


def update_v3(q, delta_velocity):
    velocity_next = np.array([
        delta_velocity[0] * (q[0] ** 2 - q[1] ** 2 + q[2] ** 2 + q[3] ** 2) -
        delta_velocity[1] * 2 * q[1] * q[2] -
        delta_velocity[2] * 2 * q[1] * q[3],

        -delta_velocity[0] * 2 * q[1] * q[2] +
        delta_velocity[1] * (q[0] ** 2 + q[1] ** 2 - q[2] ** 2 + q[3] ** 2) -
        delta_velocity[2] * 2 * q[2] * q[3],

        -delta_velocity[0] * 2 * q[1] * q[3] -
        delta_velocity[1] * 2 * q[2] * q[3] +
        delta_velocity[2] * (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 - q[3] ** 2),

    ])
    return velocity_next


if __name__ == '__main__':
    # 初始化四元数
    # q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]，其中w是实部，x, y, z是虚部
    th = np.array([1.40679, 0.38469, 276.51585])
    th2 = th * np.pi / 360

    q = np.array([
        np.cos(th2[0]) * np.cos(th2[1]) * np.cos(th2[2]) + np.sin(th2[0]) * np.sin(th2[1]) * np.sin(th2[2]),
        np.sin(th2[0]) * np.cos(th2[1]) * np.cos(th2[2]) - np.cos(th2[0]) * np.sin(th2[1]) * np.sin(th2[2]),
        np.cos(th2[0]) * np.sin(th2[1]) * np.cos(th2[2]) + np.sin(th2[0]) * np.cos(th2[1]) * np.sin(th2[2]),
        np.cos(th2[0]) * np.cos(th2[1]) * np.sin(th2[2]) - np.sin(th2[0]) * np.sin(th2[1]) * np.cos(th2[2])
    ])  # [w, x, y, z]，其中w是实部，x, y, z是虚部

    print(q)

    delta_velocity = np.array([0.00298, -0.00092, 0.00253])
    delta_theta=np.array([1.40679  ,  0.38469  ,  276.51585 ])
    position = np.zeros(3)
    xx = []
    yy = []
    delta_velocity1=np.zeros(3)
    dt = 0.005
    xx2 = []
    yy2 = []
    position1=np.zeros(3)
    data = np.genfromtxt("datasets/ICM20602/ICM20602.txt", dtype=float)  # 将文件中数据加载到data数组里
    data2 = np.genfromtxt("datasets/ICM20602/truth.nav", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, Omega_x, Omega_y, Omega_z,velocity_x,velocity_y, velocity_z-g]"""
    print(data.shape)
    for i in range(0, len(data)):
        # dt = data[i][0] - data[i - 1][0]
        # print(dt)

        delta_theta += data[i][1:4]*180/np.pi
        delta_velocity += data[i][4:7]
        delta_velocity[2] += 9.801 * dt






        th = np.array(data2[i][8:11])
        print(delta_theta,th)




        # th2 = th * np.pi / 360
        # q = np.array([
        #     np.cos(th2[0]) * np.cos(th2[1]) * np.cos(th2[2]) + np.sin(th2[0]) * np.sin(th2[1]) * np.sin(th2[2]),
        #     np.sin(th2[0]) * np.cos(th2[1]) * np.cos(th2[2]) - np.cos(th2[0]) * np.sin(th2[1]) * np.sin(th2[2]),
        #     np.cos(th2[0]) * np.sin(th2[1]) * np.cos(th2[2]) + np.sin(th2[0]) * np.cos(th2[1]) * np.sin(th2[2]),
        #     np.cos(th2[0]) * np.cos(th2[1]) * np.sin(th2[2]) - np.sin(th2[0]) * np.sin(th2[1]) * np.cos(th2[2])
        # ])
        # q = q / np.linalg.norm(q)



        delta_velocity1 = data2[i][5:8]
        # position1+=delta_velocity1*dt
        # xx2.append(position1[0])
        # yy2.append(position1[1])







        # q = update_q(q, delta_theta)
        # print(np.arctan2(2*(q[0]*q[1]+q[3]*q[2]),(1-2*(q[1]**2+q[2]**2)))*180/np.pi)
        # print(np.arcsin(2*(q[0]*q[2]-q[3]*q[1]))*180/np.pi)

        velocity = update_v2(q, delta_velocity)

        # nec = np.linalg.norm(delta_velocity)
        # velocity = nec * velocity / np.linalg.norm(velocity)
        # global_velocity+=velocity
        # print(velocity)
        position += velocity * dt
        xx.append(position[0])
        yy.append(position[1])
    plt.plot(xx, yy, 'c')
    # plt.plot(xx2, yy2, 'b')
    plt.show()

    # for i in range(1, len(data)):
    #     dt = data[i][1] - data[i - 1][1]
    #
    #     # delta_theta = data[i][1:4]
    #     delta_velocity = data[i][5:8]
    #
    #
    #     position+=delta_velocity*dt
    #     xx.append(position[0])
    #     yy.append(position[1])
    # plt.plot(xx, yy, 'c')
    # plt.show()
