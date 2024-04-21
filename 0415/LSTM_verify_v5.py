'''
  ******************************************************************************
  * @file           : LSTM_verify.py
  * @author         : ZhangKai
  * @date           : 2024/4/6
  * @description    : 
  ******************************************************************************
'''
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim

from sklearn.model_selection import train_test_split
from keras.utils import to_categorical
import numpy as np
import os
from keras.models import Sequential
from keras.layers import LSTM, Dense,Dropout
from keras.callbacks import TensorBoard

import time


if __name__ == '__main__':

    # data_from_txt=True
    data_from_txt = False

    step_time=100

    if data_from_txt:
        data_input = []
        data_output = []
        path = "data"  # 文件夹目录
        files = os.listdir(path)  # 得到文件夹下的所有文件名称
        print(files)
        s = []
        for file in files:  # 遍历文件夹

            data = np.genfromtxt(path + "/" + file, dtype=float)  # 将文件中数据加载到data数组里
            """[ seconds of week, Yaw,Yaw, Yaw]"""

            print(file)  # 打印结果
        # exit()
        # data = np.genfromtxt("0415/data/data_v4_1.txt", dtype=float)  # 将文件中数据加载到data数组里

        # for i in range(10):
            for i in range(0,len(data) - step_time-1,9):
                data_input.append(np.array(data[i:i + step_time, [1,2]]))
                data_output.append(np.array(data[i + step_time][3]))
            print(len(data_input))




        data_I = np.stack(data_input, axis=0)
        data_O =np.array(data_output)
        # data_O= np.stack(data_output, axis=0)
        np.save("output/data_I.npy", data_I)  # 保存文件
        np.save("output/data_O.npy", data_O)  # 保存文件
        print(data_I.shape)
        print(data_O.shape)


    else:
        data_I = np.load("output/data_I.npy")  # 读取文件
        data_O = np.load("output/data_O.npy")  # 读取文件
        print(data_I.shape)
        print(data_O.shape)


model = Sequential()
model.add(LSTM(16, return_sequences=False, activation='relu', input_shape=(step_time, 2)))  # 使用None作为时间步长，使得模型可以处理任意长度的序列
model.add(Dense(3, activation='relu'))
model.add(Dense(1, activation='linear'))  # 输出层的单元数与形状匹配
# # # 编译模型
model.compile(optimizer='adam', loss='mse', metrics=['mse','mae'])
model.load_weights('Ultimate_model0.keras')
# # # 打印模型概要
model.summary()

predictions = model.predict(data_I)
print(predictions)
plt.plot(predictions,'r')
plt.plot(data_O)
plt.show()




