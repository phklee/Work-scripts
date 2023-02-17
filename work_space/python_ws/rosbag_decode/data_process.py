#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import sys
import pandas as pd

def analyze_other():
  root_dir='/home/user/avos3_0/bugs/robotaxi/速度分析/5_直道_远处后方车辆加速左侧超车/'
  self_car='IDPXJILANTU0000777'
  target_car='IDPXJT102010BB0003'
  car_name=self_car  # self_car/target_car
  # 读取 Excel 文件，假设数据在第一个工作表中
  df = pd.read_excel(root_dir + car_name + '_速度分析前_毫米波优化前.xls', sheet_name='Sheet1')
  # 对时间戳向下取整
  df['time_floor'] = df['time_stamp'].apply(lambda x: int(x))
  # 按照 time_floor 分组，求平均速度
  df = df.groupby('time_floor', as_index=False)['speed'].mean()
  # 重命名列名
  df = df.rename(columns={'speed': 'mean_speed'})
  # 保存结果到新的 Excel 文件
  df.to_excel(root_dir + car_name + '_速度分析后_毫米波优化前.xls', sheet_name='Sheet1')


########################################################################################
def analyze_speed():
  # 获取命令行参数
  source_file = sys.argv[1]
  target_file = sys.argv[2]

  # 读取Excel文件
  df = pd.read_excel(source_file)
  # 对时间戳进行四舍五入并保留1位小数
  df['time_rounded'] = df['time_stamp'].apply(lambda x: round(x, 1))

  # 按照时间戳分组，计算平均速度
  # result_df = df.groupby('time_rounded')['speed'].mean().reset_index()
  result_df = df.groupby('time_rounded', as_index=False)['real_speed'].mean()

  target_file = target_file + 'speed_after_process.xls'
  # 保存结果到新的Excel文件
  result_df.to_excel(target_file, index=False)
  # print("处理完成，结果已保存到", target_file)


########################################################################################
def analyze_loc():
    # 获取命令行参数
    source_file = sys.argv[1]
    target_file = sys.argv[2]

    # 读取Excel文件
    df = pd.read_excel(source_file)

    # 筛选特定数字结尾的时间戳数据
    filtered_df = df[df['time_stamp'].apply(lambda x: int(x * 100) % 10 == 0)]

    # 提取保留的时间戳对应的x和y值，并添加时间戳列
    result_df = filtered_df[['time_stamp', 'x', 'y']]

    target_file = target_file + 'loc_after_process.xls'
    # 保存结果到新的Excel文件
    result_df.to_excel(target_file, index=False)

def main():
  # analyze_speed()
  analyze_loc()

if __name__ == "__main__":
  main()