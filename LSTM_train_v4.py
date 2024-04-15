'''
  ******************************************************************************
  * @file           : LSTM_train.py
  * @author         : ZhangKai
  * @date           : 2024/4/6
  * @description    : preconditioning
  ******************************************************************************
'''

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

    data_from_txt=True
    # data_from_txt = False
    step_time=100
    if data_from_txt:
        data = np.genfromtxt("outputData/data_v4_300.txt", dtype=float)  # 将文件中数据加载到data数组里
        """[ seconds of week,roll, Pitch, Yaw,latitude, longitude,roll, Pitch, Yaw,]"""
        data_input = []
        data_output = []
        # for i in range(10):
        for i in range(0,len(data) - step_time-1,6):
            data_input.append(np.array(data[i:i + step_time, [1,2]]))

            data_output.append(np.array(data[i + step_time][3]))
        data_I = np.stack(data_input, axis=0)
        data_O =np.array(data_output)
        # data_O= np.stack(data_output, axis=0)
        np.save("outputData/data_I.npy", data_I)  # 保存文件
        np.save("outputData/data_O.npy", data_O)  # 保存文件
        print(data_I.shape)
        print(data_O.shape)
    else:
        data_I = np.load("outputData/data_I.npy")  # 读取文件
        data_O = np.load("outputData/data_O.npy")  # 读取文件
        print(data_I.shape)
        print(data_O.shape)



# X_train, X_test, y_train, y_test = train_test_split(data_I[0:10000], data_O[0:10000], test_size=0.05)
X_train, X_test, y_train, y_test = train_test_split(data_I, data_O, test_size=0.05)
#
# # 创建TensorBoard回调
print(X_train.shape)
# exit()


log_dir = os.path.join('D://Logs')
# os.makedirs(log_dir, exist_ok=True)
tb_callback = TensorBoard(log_dir)

# step_time = 600
# # # 定义模型
model = Sequential()
model.add(LSTM(16, return_sequences=False, activation='relu', input_shape=(step_time, 2)))  # 使用None作为时间步长，使得模型可以处理任意长度的序列
# model.add(Dropout(rate=0.5))
# model.add(LSTM(20))  # 使用None作为时间步长，使得模型可以处理任意长度的序列
# model.add(LSTM(8, return_sequences=False, activation='relu'))  # 不再需要返回序列，因为只关心最后的输出
# model.add(Dense(3, activation='relu'))
# model.add(Dense(8, activation='relu'))
model.add(Dense(1, activation='linear'))  # 输出层的单元数与形状匹配
# # # 编译模型
model.compile(optimizer='adam', loss='mse', metrics=['mse'])
model.load_weights('GNSS_v4_1.keras')
# # # 打印模型概要
model.summary()
# # # 训练模型
model.fit(X_train, y_train, epochs=50, batch_size=8, validation_data=(X_test, y_test), callbacks=[tb_callback])
# model.fit(X_train, y_train, epochs=20, batch_size=32, validation_data=(X_test, y_test), callbacks=[tb_callback])

# # # 保存模型
model.save('GNSS_v4_1.keras')
#
# tensorboard --logdir Logs --host=127.0.0.1