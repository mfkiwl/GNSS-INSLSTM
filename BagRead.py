'''
  ******************************************************************************
  * @file           : BagRead.py
  * @author         : ZhangKai
  * @date           : 2024/3/31
  * @description    : 读取香港的数据集并保存
  ******************************************************************************
'''


import rosbag
import rospy
import numpy as np
import json
import pandas as pd


if __name__ == '__main__':

    bag_path = 'D:\\20190331HH.bag'

    Analyze_imu = True
    Analyze_GNSS = False

    # Analyze_imu = False
    # Analyze_GNSS = True


    num=0
    error_num=0
    nu=[]

    if Analyze_imu:
        # 保存为json时使用的格式
        dict_imu = {
            'stamp': [0.0,0.0],
            'orientation': [0.0,0.0,0.0,0.0],
            'angular_velocity': [0.0,0.0,0.0],
            'linear_acceleration': [0.0,0.0,0.0]
        }
        data_imu = []

    if Analyze_GNSS:
        dict_GNSS = {
            'pos': [0.0,0.0,0.0],
            'pseudorange': 0.0,
            'sat_clk_err':0.0,
            'err_tropo' :0.0,
            'err_iono':0.0,
            'elevation':0.0,
            'azimuth':0.0,
            'snr':0.0
        }
        data_GNSS = []
        GNSS_Raws = []

    print("=========================================\nStart Fusion Project\n=========================================")

    ax=[]
    ay=[]
    az=[]
    gx=[]
    gy=[]
    gz=[]
    count=[]
    bag = rosbag.Bag(bag_path)  # rosbag文件所在路径
    for topic, msg, t in bag.read_messages():

        """ Read bag file || /imu/data """
        if Analyze_imu and topic == "/imu/data":

            print(topic,num)
            count.append(num)
            ax.append(msg.angular_velocity.x)
            ay.append(msg.angular_velocity.y)
            az.append(msg.angular_velocity.z)
            gx.append(msg.angular_velocity.x)
            gy.append(msg.angular_velocity.y)
            gz.append(msg.angular_velocity.z)
            num+=1


            # dict_imu['stamp']= [msg.header.stamp.secs,msg.header.stamp.nsecs]
            # dict_imu["orientation"] = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
            # dict_imu["angular_velocity"]=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
            # dict_imu["linear_acceleration"]= [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            # dict_temp=dict_imu.copy()
            # data_imu.append(dict_temp)
            # print(data_imu)
        """ Read bag file || /GNSS_ """
        if Analyze_GNSS and topic == "/GNSS_":
            num += 1
            print(topic, num)
            # print("msg",msg.GNSS_Raws[0].total_sv)
            # if msg.GNSS_Raws[0].total_sv ==7.0:
            #     print(msg.GNSS_Raws)

            if msg.GNSS_Raws[0].total_sv not in nu:
                nu.append(msg.GNSS_Raws[0].total_sv)

            GNSS_Raws = []  # 清空
            for i in range(int(msg.GNSS_Raws[0].total_sv)):
                # print(i)
                try:
                    dict_GNSS["pseudorange"] = msg.GNSS_Raws[i].pseudorange
                    dict_GNSS["pos"] = [msg.GNSS_Raws[i].sat_pos_x,msg.GNSS_Raws[i].sat_pos_y,msg.GNSS_Raws[i].sat_pos_z]
                    dict_GNSS["sat_clk_err"] = msg.GNSS_Raws[i].sat_clk_err
                    dict_GNSS["err_tropo"] = msg.GNSS_Raws[i].err_tropo
                    dict_GNSS["err_iono"] = msg.GNSS_Raws[i].err_iono
                    dict_GNSS["elevation"] = msg.GNSS_Raws[i].elevation
                    dict_GNSS["azimuth"] = msg.GNSS_Raws[i].azimuth
                    dict_GNSS["snr"] = msg.GNSS_Raws[i].snr

                    dict_temp = dict_GNSS.copy()
                    GNSS_Raws.append(dict_temp)
                except:
                    error_num+=1
                    print(f"ERROR{error_num} : Missing data")
            dict_temp = GNSS_Raws.copy()
            data_GNSS.append(dict_temp)


    bag.close()

    """保存数据"""
    if Analyze_imu:
        # dumps 将数据转换成字符串
        data = {
            'num': count,
            'ax': ax,
            'ay': ay,
            'az': az,
            'gx': gx,
            'gy': gy,
            'gz': gz
        }

        df = pd.DataFrame(data)

        # 将DataFrame保存到Excel文件，指定文件名
        df.to_excel('outputData/IMU_ag.xlsx', index=False)

        # info_json_imu = json.dumps(data_imu, sort_keys=False, indent=4, separators=(',', ': '))
        # print(type(info_json_imu))
        # f = open('imu.json', 'w')
        # f.write(info_json_imu)
        # f.close()

    if Analyze_GNSS:
        info_json_GNSS = json.dumps(data_GNSS, sort_keys=False, indent=4, separators=(',', ': '))
        print(type(info_json_GNSS))
        f = open('GNSS.json', 'w')
        f.write(info_json_GNSS)
        f.close()
        print(nu)


    """读取文件"""
    # f2 = open('imu.json', 'r')
    # info_data = json.load(f2)
    # print(type(info_data))
    # f2.close()

    print("=========================================\n"
          "Fusion Project End"
          "\n=========================================")
