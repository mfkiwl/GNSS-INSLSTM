'''
  ******************************************************************************
  * @file           : savedata.py
  * @author         : ZhangKai
  * @date           : 2024/3/31
  * @description    : 
  ******************************************************************************
'''
import pandas as pd

# 假设我们有以下数据
data = {
    'Column1': [1, 2, 3, 4],
    'Column2': ['A', 'B', 'C', 'D'],
    'Column3': [5.5, 6.6, 7.7, 8.8]
}

# 创建DataFrame
df = pd.DataFrame(data)

# 将DataFrame保存到Excel文件，指定文件名
df.to_excel('outputData/output.xlsx', index=False)