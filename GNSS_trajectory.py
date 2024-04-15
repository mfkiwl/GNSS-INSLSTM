import math
import time

import numpy as np
import matplotlib.pyplot as plt

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
    azimuth_deg = (azimuth_deg + 360)%360

    return azimuth_deg


if __name__ == '__main__':
    # plt.ion()
    data = np.genfromtxt("datasets/GNSS RTK/GNSS_RTK.txt", dtype=float)  # 将文件中数据加载到data数组里
    # data2 = np.genfromtxt("datasets/GNSS RTK/GNSS_RTK.txt", dtype=float)  # 将文件中数据加载到data数组里
    """[ seconds of week, latitude, longitude, height,lat_SD,lon_SD, height_SD]"""
    lat = []
    lon = []
    GNSS=[]
    h = []
    for i in range(len(data)-1):
        lat.append(data[i][1])
        lon.append(data[i][2])
        h.append(data[i][3])
        count_GNSS = i
        yaw_GNSS = 270

        if not (data[count_GNSS][1] == data[count_GNSS + 1][1] and data[count_GNSS][2] ==
                data[count_GNSS + 1][2]):
            yaw_GNSS = azimuth(data[count_GNSS][1], data[count_GNSS][2], data[count_GNSS + 1][1],
                               data[count_GNSS + 1][2])
        GNSS.append(yaw_GNSS)
        # ax = plt.subplot(projection='3d')  # 创建一个三维的绘图工程
        # ax.set_title('3d_image_show')  # 设置本图名称
        #
        #
        # ax.scatter(lon ,lat, h)  # 绘制数据点 c: 'r'红色，'y'黄色，等颜色
        #
        # ax.set_xlabel('X')  # 设置x坐标轴
        # ax.set_ylabel('Y')  # 设置y坐标轴
        # ax.set_zlabel('Z')  # 设置z坐标轴
        # plt.pause(0.00001)
        # 清除上一次显示
        # plt.cla()
    plt.plot(GNSS, 'c')
        # plt.plot(lat, lon, 'c')
        # plt.xlim((30.445, 30.466))
        # plt.ylim((114.459,114.476))

    # plt.plot(lat,lon,'c')
    plt.show()

# 裁剪区域
# 297-341
# 676-708
# 1311-1349
# 1382-1404