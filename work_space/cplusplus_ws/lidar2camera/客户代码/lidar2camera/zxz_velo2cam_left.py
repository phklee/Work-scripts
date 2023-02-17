# -*- coding: UTF-8 -*-
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pprint
import copy
import itertools
from tkinter import _flatten
import open3d as o3d
from numpy import mat

from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
import numpy as np
import math

np.set_printoptions(suppress=True)


def read_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)

    return points

def euler_to_rotMat(roll,pitch,yaw):
    Rx_roll = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]])

    Ry_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]])

    Rz_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]])

    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat


img = f'./2_H90l_1656648480600.jpg'
scan = read_pcd("./1656648480600.pcd")


#雷达外参矩阵
#####################################################################################
#雷达外参 roll pitch yaw
lidar_rpy = [0.0, 0.3, -0.086512]
#雷达外侧平移量 xoffset yoffset zoffset
lidar_shift = np.array([[1.4], [0.0], [0.0]])
R_lidar = euler_to_rotMat(lidar_rpy[0], lidar_rpy[1], lidar_rpy[2])  #欧拉角转旋转矩阵

RT_lidar2imu = np.hstack((R_lidar, lidar_shift))   #添加平移量
RT_lidar2imu = np.insert(RT_lidar2imu, 3, values=[0, 0, 0, 1], axis=0) #添加其次坐标
RT_lidar2imu=mat(RT_lidar2imu)
#####################################################################################

######################################################################################
#相机外参数
heading = {
    "w": 0.9228708204710818,
    "z": 0.001407881828232372,
    "y": 0.3851045039042997,
    "x": 0.001407881828232372
}


#相机外参数平移量
poistion = {
    "z": -1.48,
    "x": 0.22,
    "y": -0.06
}



#四元数组转旋转矩阵
R_camera2imu = np.asarray([heading['x'], heading['y'], heading['z'], heading['w']])
R_camera2imu = R.from_quat(R_camera2imu).as_matrix() #四元组转旋转矩阵

shift_camera2imu = np.array([[poistion['x']], [poistion['y']], [poistion['z']]])
RT_camrea2lidar = np.hstack((R_camera2imu, shift_camera2imu)) #添加平移量
RT_camrea2lidar = np.insert(RT_camrea2lidar, 3, values=[0, 0, 0, 1], axis=0) #添加齐次坐标
RT_camrea2lidar=mat(RT_camrea2lidar)

##################################################





####################################################
#相机内参数
ins_matrix = np.array([825.872911, 0, 645.475697, 0, 827.100951, 350.396819, 0, 0, 1]).reshape(3, 3)
ins_matrix=np.insert(ins_matrix, 3, values=[0, 0, 0], axis=1)
ins_matrix = mat(ins_matrix)
###################################################

points = scan[:, 0:3]  # lidar xyz (front, left, up)
lidar = np.insert(points, 3, 1, axis=1).T

lidar2imu = np.dot(RT_lidar2imu.I, lidar)  #雷达坐标到IMU坐标转换
imu2camera =np.dot(RT_camrea2lidar.I,lidar2imu) #imu坐标到相机坐标

cam = np.dot(ins_matrix, imu2camera) #相机坐标系 到图片坐标转换

# get u,v,z
cam[:2] /= cam[2, :]

######显示
plt.figure(figsize=(12, 5), dpi=96, tight_layout=True)
png = mpimg.imread(img)
IMG_H, IMG_W, _ = png.shape
# restrict canvas in range
plt.axis([0, IMG_W, IMG_H, 0])
plt.imshow(png)
# filter point out of canvas
u, v, z = cam
u_out = np.logical_or(u < 0, u > IMG_W)
v_out = np.logical_or(v < 0, v > IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam, np.where(outlier), axis=1)
# generate color map from depth
name = "2_H60_670"
u, v, z = cam
plt.scatter([u], [v], c=[z], cmap='rainbow_r', alpha=0.5, s=2)
plt.title(name)
plt.savefig(f'./{name}.png', bbox_inches='tight')
plt.show()
